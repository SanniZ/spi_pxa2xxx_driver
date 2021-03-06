/*
 * Copyright © 2014 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Ben Widawsky <ben@bwidawsk.net>
 *    Michel Thierry <michel.thierry@intel.com>
 *    Thomas Daniel <thomas.daniel@intel.com>
 *    Oscar Mateo <oscar.mateo@intel.com>
 *
 */

/**
 * DOC: Logical Rings, Logical Ring Contexts and Execlists
 *
 * Motivation:
 * GEN8 brings an expansion of the HW contexts: "Logical Ring Contexts".
 * These expanded contexts enable a number of new abilities, especially
 * "Execlists" (also implemented in this file).
 *
 * One of the main differences with the legacy HW contexts is that logical
 * ring contexts incorporate many more things to the context's state, like
 * PDPs or ringbuffer control registers:
 *
 * The reason why PDPs are included in the context is straightforward: as
 * PPGTTs (per-process GTTs) are actually per-context, having the PDPs
 * contained there mean you don't need to do a ppgtt->switch_mm yourself,
 * instead, the GPU will do it for you on the context switch.
 *
 * But, what about the ringbuffer control registers (head, tail, etc..)?
 * shouldn't we just need a set of those per engine command streamer? This is
 * where the name "Logical Rings" starts to make sense: by virtualizing the
 * rings, the engine cs shifts to a new "ring buffer" with every context
 * switch. When you want to submit a workload to the GPU you: A) choose your
 * context, B) find its appropriate virtualized ring, C) write commands to it
 * and then, finally, D) tell the GPU to switch to that context.
 *
 * Instead of the legacy MI_SET_CONTEXT, the way you tell the GPU to switch
 * to a contexts is via a context execution list, ergo "Execlists".
 *
 * LRC implementation:
 * Regarding the creation of contexts, we have:
 *
 * - One global default context.
 * - One local default context for each opened fd.
 * - One local extra context for each context create ioctl call.
 *
 * Now that ringbuffers belong per-context (and not per-engine, like before)
 * and that contexts are uniquely tied to a given engine (and not reusable,
 * like before) we need:
 *
 * - One ringbuffer per-engine inside each context.
 * - One backing object per-engine inside each context.
 *
 * The global default context starts its life with these new objects fully
 * allocated and populated. The local default context for each opened fd is
 * more complex, because we don't know at creation time which engine is going
 * to use them. To handle this, we have implemented a deferred creation of LR
 * contexts:
 *
 * The local context starts its life as a hollow or blank holder, that only
 * gets populated for a given engine once we receive an execbuffer. If later
 * on we receive another execbuffer ioctl for the same context but a different
 * engine, we allocate/populate a new ringbuffer and context backing object and
 * so on.
 *
 * Finally, regarding local contexts created using the ioctl call: as they are
 * only allowed with the render ring, we can allocate & populate them right
 * away (no need to defer anything, at least for now).
 *
 * Execlists implementation:
 * Execlists are the new method by which, on gen8+ hardware, workloads are
 * submitted for execution (as opposed to the legacy, ringbuffer-based, method).
 * This method works as follows:
 *
 * When a request is committed, its commands (the BB start and any leading or
 * trailing commands, like the seqno breadcrumbs) are placed in the ringbuffer
 * for the appropriate context. The tail pointer in the hardware context is not
 * updated at this time, but instead, kept by the driver in the ringbuffer
 * structure. A structure representing this request is added to a request queue
 * for the appropriate engine: this structure contains a copy of the context's
 * tail after the request was written to the ring buffer and a pointer to the
 * context itself.
 *
 * If the engine's request queue was empty before the request was added, the
 * queue is processed immediately. Otherwise the queue will be processed during
 * a context switch interrupt. In any case, elements on the queue will get sent
 * (in pairs) to the GPU's ExecLists Submit Port (ELSP, for short) with a
 * globally unique 20-bits submission ID.
 *
 * When execution of a request completes, the GPU updates the context status
 * buffer with a context complete event and generates a context switch interrupt.
 * During the interrupt handling, the driver examines the events in the buffer:
 * for each context complete event, if the announced ID matches that on the head
 * of the request queue, then that request is retired and removed from the queue.
 *
 * After processing, if any requests were retired and the queue is not empty
 * then a new execution list can be submitted. The two requests at the front of
 * the queue are next to be submitted but since a context may not occur twice in
 * an execution list, if subsequent requests have the same ID as the first then
 * the two requests must be combined. This is done simply by discarding requests
 * at the head of the queue until either only one requests is left (in which case
 * we use a NULL second context) or the first two requests have unique IDs.
 *
 * By always executing the first two requests in the queue the driver ensures
 * that the GPU is kept as busy as possible. In the case where a single context
 * completes but a second context is still executing, the request for this second
 * context will be at the head of the queue when we remove the first one. This
 * request will then be resubmitted along with a new request for a different context,
 * which will cause the hardware to continue executing the second request and queue
 * the new request (the GPU detects the condition of a context getting preempted
 * with the same context and optimizes the context switch flow by not doing
 * preemption, but just sampling the new tail pointer).
 *
 */

#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "intel_mocs.h"
#include "i915_scheduler.h"

#define GEN9_LR_CONTEXT_RENDER_SIZE (22 * PAGE_SIZE)
#define GEN8_LR_CONTEXT_RENDER_SIZE (20 * PAGE_SIZE)
#define GEN8_LR_CONTEXT_OTHER_SIZE (2 * PAGE_SIZE)

#define RING_EXECLIST_QFULL		(1 << 0x2)
#define RING_EXECLIST1_VALID		(1 << 0x3)
#define RING_EXECLIST0_VALID		(1 << 0x4)
#define RING_EXECLIST_ACTIVE_STATUS	(3 << 0xE)
#define RING_EXECLIST1_ACTIVE		(1 << 0x11)
#define RING_EXECLIST0_ACTIVE		(1 << 0x12)

#define GEN8_CTX_STATUS_IDLE_ACTIVE	(1 << 0)
#define GEN8_CTX_STATUS_PREEMPTED	(1 << 1)
#define GEN8_CTX_STATUS_ELEMENT_SWITCH	(1 << 2)
#define GEN8_CTX_STATUS_ACTIVE_IDLE	(1 << 3)
#define GEN8_CTX_STATUS_COMPLETE	(1 << 4)
#define GEN8_CTX_STATUS_LITE_RESTORE	(1 << 15)

#define CTX_LRI_HEADER_0		0x01
#define CTX_CONTEXT_CONTROL		0x02
#define CTX_RING_HEAD			0x04
#define CTX_RING_TAIL			0x06
#define CTX_RING_BUFFER_START		0x08
#define CTX_RING_BUFFER_CONTROL		0x0a
#define CTX_BB_HEAD_U			0x0c
#define CTX_BB_HEAD_L			0x0e
#define CTX_BB_STATE			0x10
#define CTX_SECOND_BB_HEAD_U		0x12
#define CTX_SECOND_BB_HEAD_L		0x14
#define CTX_SECOND_BB_STATE		0x16
#define CTX_BB_PER_CTX_PTR		0x18
#define CTX_RCS_INDIRECT_CTX		0x1a
#define CTX_RCS_INDIRECT_CTX_OFFSET	0x1c
#define CTX_LRI_HEADER_1		0x21
#define CTX_CTX_TIMESTAMP		0x22
#define CTX_PDP3_UDW			0x24
#define CTX_PDP3_LDW			0x26
#define CTX_PDP2_UDW			0x28
#define CTX_PDP2_LDW			0x2a
#define CTX_PDP1_UDW			0x2c
#define CTX_PDP1_LDW			0x2e
#define CTX_PDP0_UDW			0x30
#define CTX_PDP0_LDW			0x32
#define CTX_LRI_HEADER_2		0x41
#define CTX_R_PWR_CLK_STATE		0x42
#define CTX_GPGPU_CSR_BASE_ADDRESS	0x44

#define GEN8_CTX_VALID (1<<0)
#define GEN8_CTX_FORCE_PD_RESTORE (1<<1)
#define GEN8_CTX_FORCE_RESTORE (1<<2)
#define GEN8_CTX_L3LLC_COHERENT (1<<5)
#define GEN8_CTX_PRIVILEGE (1<<8)

#define ASSIGN_CTX_REG(reg_state, pos, reg, val) do { \
	(reg_state)[(pos)+0] = i915_mmio_reg_offset(reg); \
	(reg_state)[(pos)+1] = (val); \
} while (0)

#define ASSIGN_CTX_PDP(ppgtt, reg_state, n) do {		\
	const u64 _addr = i915_page_dir_dma_addr((ppgtt), (n));	\
	reg_state[CTX_PDP ## n ## _UDW+1] = upper_32_bits(_addr); \
	reg_state[CTX_PDP ## n ## _LDW+1] = lower_32_bits(_addr); \
} while (0)

#define ASSIGN_CTX_PML4(ppgtt, reg_state) do { \
	reg_state[CTX_PDP0_UDW + 1] = upper_32_bits(px_dma(&ppgtt->pml4)); \
	reg_state[CTX_PDP0_LDW + 1] = lower_32_bits(px_dma(&ppgtt->pml4)); \
} while (0)

enum {
	ADVANCED_CONTEXT = 0,
	LEGACY_32B_CONTEXT,
	ADVANCED_AD_CONTEXT,
	LEGACY_64B_CONTEXT
};
#define GEN8_CTX_ADDRESSING_MODE_SHIFT 3
#define GEN8_CTX_ADDRESSING_MODE(dev)  (USES_FULL_48BIT_PPGTT(dev) ?\
		LEGACY_64B_CONTEXT :\
		LEGACY_32B_CONTEXT)
enum {
	FAULT_AND_HANG = 0,
	FAULT_AND_HALT, /* Debug only */
	FAULT_AND_STREAM,
	FAULT_AND_CONTINUE /* Unsupported */
};
#define GEN8_CTX_ID_SHIFT 32
#define GEN8_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT	0x17
#define GEN9_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT	0x26

static int intel_lr_context_pin(struct drm_i915_gem_request *rq);
static void lrc_setup_hardware_status_page(struct intel_engine_cs *engine,
		struct drm_i915_gem_object *default_ctx_obj);


/*
 * Test to see if the ring has sufficient space to submit a given piece
 * of work without causing a stall
 */
static int logical_ring_test_space(struct intel_ringbuffer *ringbuf,
				   int min_space)
{
	if (ringbuf->space < min_space) {
		/* Need to update the actual ring space. Otherwise, the system
		 * hangs forever testing a software copy of the space value that
		 * never changes!
		 */
		intel_ring_update_space(ringbuf);

		if (ringbuf->space < min_space)
			return -EAGAIN;
	}

	return 0;
}

/**
 * intel_sanitize_enable_execlists() - sanitize i915.enable_execlists
 * @dev: DRM device.
 * @enable_execlists: value of i915.enable_execlists module parameter.
 *
 * Only certain platforms support Execlists (the prerequisites being
 * support for Logical Ring Contexts and Aliasing PPGTT or better).
 *
 * Return: 1 if Execlists is supported and has to be enabled.
 */
int intel_sanitize_enable_execlists(struct drm_device *dev, int enable_execlists)
{
	WARN_ON(i915.enable_ppgtt == -1);

	/* On platforms with execlist available, vGPU will only
	 * support execlist mode, no ring buffer mode.
	 */
	if (HAS_LOGICAL_RING_CONTEXTS(dev) && intel_vgpu_active(dev))
		return 1;

	if (INTEL_INFO(dev)->gen >= 9)
		return 1;

	if (enable_execlists == 0)
		return 0;

	if (HAS_LOGICAL_RING_CONTEXTS(dev) && USES_PPGTT(dev) &&
	    i915.use_mmio_flip >= 0)
		return 1;

	return 0;
}

/**
 * intel_execlists_ctx_id() - get the Execlists Context ID
 * @ctx: LR context
 *
 * Do not confuse with ctx->id! Unfortunately we have a name overload
 * here: the old context ID we pass to userspace as a handler so that
 * they can refer to a context, and the new context ID we pass to the
 * ELSP so that the GPU can inform us of the context status via
 * interrupts.
 *
 * Return: 20-bits globally unique context ID.
 *
 * Further the ID given to HW can now be relied on to be constant for
 * the lifetime of the context, unlike previously when we used an
 * associated logical ring context address (which could be repinned at
 * a different address).
 */
u32 intel_execlists_ctx_id(struct intel_context *ctx)
{
	return ctx->global_id;
}

static bool disable_lite_restore_wa(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;

	return (IS_SKL_REVID(dev, 0, SKL_REVID_B0) ||
		IS_BXT_REVID(dev, 0, BXT_REVID_A1)) &&
	       (engine->id == VCS || engine->id == VCS2);
}

uint64_t intel_lr_context_descriptor(struct intel_context *ctx,
				     struct intel_engine_cs *engine)
{
	struct drm_i915_gem_object *ctx_obj = ctx->engine[engine->id].state;
	uint64_t desc;
	uint64_t lrca = i915_gem_obj_ggtt_offset(ctx_obj) +
			LRC_PPHWSP_PN * PAGE_SIZE;

	WARN_ON(lrca & 0xFFFFFFFF00000FFFULL);

	desc = GEN8_CTX_VALID;
	desc |= GEN8_CTX_ADDRESSING_MODE(dev) << GEN8_CTX_ADDRESSING_MODE_SHIFT;
	if (IS_GEN8(ctx_obj->base.dev))
		desc |= GEN8_CTX_L3LLC_COHERENT;
	desc |= GEN8_CTX_PRIVILEGE;
	desc |= lrca;
	desc |= (u64)intel_execlists_ctx_id(ctx) << GEN8_CTX_ID_SHIFT;

	/* TODO: WaDisableLiteRestore when we start using semaphore
	 * signalling between Command Streamers */
	/* desc |= GEN8_CTX_FORCE_RESTORE; */

	/* WaEnableForceRestoreInCtxtDescForVCS:skl */
	/* WaEnableForceRestoreInCtxtDescForVCS:bxt */
	if (disable_lite_restore_wa(engine))
		desc |= GEN8_CTX_FORCE_RESTORE;

	return desc;
}

static void execlists_elsp_write(struct drm_i915_gem_request *rq0,
				 struct drm_i915_gem_request *rq1)
{

	struct intel_engine_cs *engine = rq0->engine;
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned int fw_domains = rq0->engine->fw_domains;
	uint64_t desc[2];

	if (rq1) {
		desc[1] = intel_lr_context_descriptor(rq1->ctx, rq1->engine);
		rq1->elsp_submitted++;
	} else {
		desc[1] = 0;
	}

	desc[0] = intel_lr_context_descriptor(rq0->ctx, rq0->engine);
	rq0->elsp_submitted++;

	/* You must always write both descriptors in the order below. */
	spin_lock(&dev_priv->uncore.lock);
	intel_uncore_forcewake_get__locked(dev_priv, fw_domains);
	I915_WRITE_FW(RING_ELSP(engine), upper_32_bits(desc[1]));
	I915_WRITE_FW(RING_ELSP(engine), lower_32_bits(desc[1]));

	I915_WRITE_FW(RING_ELSP(engine), upper_32_bits(desc[0]));
	/* The context is automatically loaded after the following */
	I915_WRITE_FW(RING_ELSP(engine), lower_32_bits(desc[0]));

	/* ELSP is a wo register, use another nearby reg for posting */
	POSTING_READ_FW(RING_EXECLIST_STATUS_LO(engine));
	intel_uncore_forcewake_put__locked(dev_priv, fw_domains);
	spin_unlock(&dev_priv->uncore.lock);
}

static int execlists_update_context(struct drm_i915_gem_request *rq)
{
	struct intel_engine_cs *engine = rq->engine;
	struct i915_hw_ppgtt *ppgtt = rq->ctx->ppgtt;
	struct drm_i915_gem_object *ctx_obj = rq->ctx->engine[engine->id].state;
	struct page *page;
	uint32_t *reg_state;

	BUG_ON(!ctx_obj);
	WARN_ON(!i915_gem_obj_is_pinned(ctx_obj));

	page = i915_gem_object_get_dirty_page(ctx_obj, LRC_STATE_PN);
	reg_state = kmap_atomic(page);

	reg_state[CTX_RING_TAIL+1] = rq->tail;
	reg_state[CTX_RING_BUFFER_START+1] = rq->ringbuf->vma->node.start;

	if (ppgtt && !USES_FULL_48BIT_PPGTT(ppgtt->base.dev)) {
		/* True 32b PPGTT with dynamic page allocation: update PDP
		 * registers and point the unallocated PDPs to scratch page.
		 * PML4 is allocated during ppgtt init, so this is not needed
		 * in 48-bit mode.
		 */
		ASSIGN_CTX_PDP(ppgtt, reg_state, 3);
		ASSIGN_CTX_PDP(ppgtt, reg_state, 2);
		ASSIGN_CTX_PDP(ppgtt, reg_state, 1);
		ASSIGN_CTX_PDP(ppgtt, reg_state, 0);
	}

	i915_oa_update_reg_state(engine, reg_state);

	kunmap_atomic(reg_state);

	return 0;
}

static void execlists_submit_requests(struct drm_i915_gem_request *rq0,
				      struct drm_i915_gem_request *rq1)
{
	execlists_update_context(rq0);

	if (rq1)
		execlists_update_context(rq1);

	execlists_elsp_write(rq0, rq1);
}

static void execlists_context_unqueue(struct intel_engine_cs *engine)
{
	struct drm_i915_gem_request *req0 = NULL, *req1 = NULL;
	struct drm_i915_gem_request *cursor = NULL, *tmp = NULL;

	assert_spin_locked(&engine->execlist_lock);

	/*
	 * If irqs are not active generate a warning as batches that finish
	 * without the irqs may get lost and a GPU Hang may occur.
	 */
	WARN_ON(!intel_irqs_enabled(engine->dev->dev_private));

	if (list_empty(&engine->execlist_queue))
		return;

	/* Try to read in pairs */
	list_for_each_entry_safe(cursor, tmp, &engine->execlist_queue,
				 execlist_link) {
		if (!req0) {
			req0 = cursor;
		} else if (req0->ctx == cursor->ctx) {
			/* Same ctx: ignore first request, as second request
			 * will update tail past first request's workload */
			cursor->elsp_submitted = req0->elsp_submitted;
			list_del(&req0->execlist_link);
			list_add_tail(&req0->execlist_link,
				&engine->execlist_retired_req_list);
			req0 = cursor;
		} else {
			req1 = cursor;
			break;
		}
	}

	if (IS_GEN8(engine->dev) || IS_GEN9(engine->dev)) {
		/*
		 * WaIdleLiteRestore: make sure we never cause a lite
		 * restore with HEAD==TAIL
		 */
		if (req0->elsp_submitted) {
			/*
			 * Apply the wa NOOPS to prevent ring:HEAD == req:TAIL
			 * as we resubmit the request. See gen8_emit_request()
			 * for where we prepare the padding after the end of the
			 * request.
			 */
			struct intel_ringbuffer *ringbuf;

			ringbuf = req0->ctx->engine[engine->id].ringbuf;
			req0->tail += 8;
			req0->tail &= ringbuf->size - 1;
		}
	}

	WARN_ON(req1 && req1->elsp_submitted);

	execlists_submit_requests(req0, req1);
}

static bool execlists_check_remove_request(struct intel_engine_cs *engine,
					   u32 request_id)
{
	struct drm_i915_gem_request *head_req;

	assert_spin_locked(&engine->execlist_lock);

	head_req = list_first_entry_or_null(&engine->execlist_queue,
					    struct drm_i915_gem_request,
					    execlist_link);

	if (head_req != NULL) {
		if (intel_execlists_ctx_id(head_req->ctx) == request_id) {
			WARN(head_req->elsp_submitted == 0,
			     "Never submitted head request\n");

			if (--head_req->elsp_submitted <= 0) {
				list_del(&head_req->execlist_link);
				list_add_tail(&head_req->execlist_link,
					&engine->execlist_retired_req_list);
				return true;
			}
		}
	}

	return false;
}

/**
 * intel_lrc_irq_handler() - handle Context Switch interrupts
 * @ring: Engine Command Streamer to handle.
 *
 * Check the unread Context Status Buffers and manage the submission of new
 * contexts to the ELSP accordingly.
 */
void intel_lrc_irq_handler(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	u32 status_pointer;
	u8 read_pointer;
	u8 write_pointer;
	u32 status = 0;
	u32 status_id;
	u32 submit_contexts = 0;

	status_pointer = I915_READ(RING_CONTEXT_STATUS_PTR(engine));

	read_pointer = engine->next_context_status_buffer;
	write_pointer = status_pointer & GEN8_CSB_PTR_MASK;
	if (read_pointer > write_pointer)
		write_pointer += GEN8_CSB_ENTRIES;

	spin_lock(&engine->execlist_lock);

	while (read_pointer < write_pointer) {
		read_pointer++;
		status = I915_READ(RING_CONTEXT_STATUS_BUF_LO(engine, read_pointer % GEN8_CSB_ENTRIES));
		status_id = I915_READ(RING_CONTEXT_STATUS_BUF_HI(engine, read_pointer % GEN8_CSB_ENTRIES));

		if (status & GEN8_CTX_STATUS_IDLE_ACTIVE)
			continue;

		if (status & GEN8_CTX_STATUS_PREEMPTED) {
			if (status & GEN8_CTX_STATUS_LITE_RESTORE) {
				if (execlists_check_remove_request(engine, status_id))
					WARN(1, "Lite Restored request removed from queue\n");
			} else
				WARN(1, "Preemption without Lite Restore\n");
		}

		 if ((status & GEN8_CTX_STATUS_ACTIVE_IDLE) ||
		     (status & GEN8_CTX_STATUS_ELEMENT_SWITCH)) {
			if (execlists_check_remove_request(engine, status_id))
				submit_contexts++;
		}
	}

	if (disable_lite_restore_wa(engine)) {
		/* Prevent a ctx to preempt itself */
		if ((status & GEN8_CTX_STATUS_ACTIVE_IDLE) &&
		    (submit_contexts != 0))
			execlists_context_unqueue(engine);
	} else if (submit_contexts != 0) {
		execlists_context_unqueue(engine);
	}

	spin_unlock(&engine->execlist_lock);

	WARN(submit_contexts > 2, "More than two context complete events?\n");
	engine->next_context_status_buffer = write_pointer % GEN8_CSB_ENTRIES;

	I915_WRITE(RING_CONTEXT_STATUS_PTR(engine),
		   _MASKED_FIELD(GEN8_CSB_PTR_MASK << 8,
				 ((u32)engine->next_context_status_buffer &
				  GEN8_CSB_PTR_MASK) << 8));
}

static int execlists_context_queue(struct drm_i915_gem_request *request)
{
	struct intel_engine_cs *engine = request->engine;
	struct drm_i915_gem_request *cursor;
	int num_elements = 0;

	if (request->ctx != engine->default_context)
		intel_lr_context_pin(request);

	i915_gem_request_reference(request);

	spin_lock_irq(&engine->execlist_lock);

	list_for_each_entry(cursor, &engine->execlist_queue, execlist_link)
		if (++num_elements > 2)
			break;

	if (num_elements > 2) {
		struct drm_i915_gem_request *tail_req;

		tail_req = list_last_entry(&engine->execlist_queue,
					   struct drm_i915_gem_request,
					   execlist_link);

		if (request->ctx == tail_req->ctx) {
			WARN(tail_req->elsp_submitted != 0,
				"More than 2 already-submitted reqs queued\n");
			list_del(&tail_req->execlist_link);
			list_add_tail(&tail_req->execlist_link,
				      &engine->execlist_retired_req_list);
		}
	}

	list_add_tail(&request->execlist_link, &engine->execlist_queue);
	if (num_elements == 0)
		execlists_context_unqueue(engine);

	spin_unlock_irq(&engine->execlist_lock);

	return 0;
}

static int logical_ring_invalidate_all_caches(struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = req->engine;
	uint32_t flush_domains;
	int ret;

	flush_domains = 0;
	if (engine->gpu_caches_dirty)
		flush_domains = I915_GEM_GPU_DOMAINS;

	ret = engine->emit_flush(req, I915_GEM_GPU_DOMAINS, flush_domains);
	if (ret)
		return ret;

	engine->gpu_caches_dirty = false;
	return 0;
}

static int execlists_move_to_gpu(struct drm_i915_gem_request *req,
				 struct list_head *vmas)
{
	const unsigned other_rings = ~intel_engine_flag(req->engine);
	struct i915_vma *vma;
	uint32_t flush_domains = 0;
	bool flush_chipset = false;
	int ret;

	list_for_each_entry(vma, vmas, exec_list) {
		struct drm_i915_gem_object *obj = vma->obj;

		if (obj->active & other_rings) {
			ret = i915_gem_object_sync(obj, req->engine, &req, true);
			if (ret)
				return ret;
		}

		if (obj->base.write_domain & I915_GEM_DOMAIN_CPU)
			flush_chipset |= i915_gem_clflush_object(obj, false);

		flush_domains |= obj->base.write_domain;
	}

	if (flush_domains & I915_GEM_DOMAIN_GTT)
		wmb();

	return 0;
}

int intel_logical_ring_alloc_request_extras(struct drm_i915_gem_request *request)
{
	int ret;

	request->ringbuf = request->ctx->engine[request->engine->id].ringbuf;

	if (request->ctx != request->engine->default_context) {
		ret = intel_lr_context_pin(request);
		if (ret)
			return ret;
	}

	if (i915.enable_guc_submission) {
		/*
		 * Check that the GuC has space for the request before
		 * going any further, as the i915_add_request() call
		 * later on mustn't fail ...
		 */
		struct intel_guc *guc = &request->i915->guc;

		ret = i915_guc_wq_check_space(guc->execbuf_client);
		if (ret)
			return ret;
	}

	return 0;
}

static int logical_ring_wait_for_space(struct drm_i915_gem_request *req,
				       int bytes)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	struct intel_engine_cs *engine = req->engine;
	struct drm_i915_gem_request *target;
	unsigned space;
	int ret;

	if (intel_ring_space(ringbuf) >= bytes)
		return 0;

	/* The whole point of reserving space is to not wait! */
	WARN_ON(ringbuf->reserved_in_use);

	list_for_each_entry(target, &engine->request_list, list) {
		/*
		 * The request queue is per-engine, so can contain requests
		 * from multiple ringbuffers. Here, we must ignore any that
		 * aren't from the ringbuffer we're considering.
		 */
		if (target->ringbuf != ringbuf)
			continue;

		/* Would completion of this request free enough space? */
		space = __intel_ring_space(target->postfix, ringbuf->tail,
					   ringbuf->size);
		if (space >= bytes)
			break;
	}

	if (WARN_ON(&target->list == &engine->request_list))
		return -ENOSPC;

	ret = i915_wait_request(target);
	if (ret)
		return ret;

	intel_ring_update_space(ringbuf);
	return 0;
}

/*
 * Reserve space for 2 NOOPs at the end of each request to be
 * used as a workaround for not being allowed to do lite
 * restore with HEAD==TAIL (WaIdleLiteRestore).
 */
#define WA_TAIL_DWORDS 10

/*
 * intel_logical_ring_advance_and_submit() - advance the tail and submit the workload
 * @request: Request to advance the logical ringbuffer of.
 *
 * The tail is updated in our logical ringbuffer struct, not in the actual context. What
 * really happens during submission is that the context and current tail will be placed
 * on a queue waiting for the ELSP to be ready to accept a new context submission. At that
 * point, the tail *inside* the context is updated and the ELSP written to.
 */
static int
intel_logical_ring_advance_and_submit(struct drm_i915_gem_request *request)
{
	struct intel_ringbuffer *ringbuf = request->ringbuf;
	struct drm_i915_private *dev_priv = request->i915;
	struct i915_guc_client *client = dev_priv->guc.execbuf_client;
	static const bool fake = false;	/* true => only pretend to preempt */
	bool preemptive;
	int i;

	intel_logical_ring_advance(ringbuf);
	request->tail = ringbuf->tail;

	/*
	 * Here we add two extra NOOPs as padding to avoid
	 * lite restore of a context with HEAD==TAIL.
	 *
	 * Caller must reserve WA_TAIL_DWORDS for us!
	 */
	for (i = 0; i < WA_TAIL_DWORDS; i++)
		intel_logical_ring_emit(ringbuf, MI_NOOP);
	intel_logical_ring_advance(ringbuf);

	if (intel_engine_stopped(request->engine))
		return 0;

	preemptive = (request->scheduler_flags & I915_REQ_SF_PREEMPT) != 0;
	if (preemptive && dev_priv->guc.preempt_client && !fake)
		client = dev_priv->guc.preempt_client;

	if (client)
		i915_guc_submit(client, request);
	else
		execlists_context_queue(request);

	return 0;
}

static void __wrap_ring_buffer(struct intel_ringbuffer *ringbuf)
{
	uint32_t __iomem *virt;
	int rem = ringbuf->size - ringbuf->tail;

	virt = ringbuf->virtual_start + ringbuf->tail;
	rem /= 4;
	while (rem--)
		iowrite32(MI_NOOP, virt++);

	ringbuf->tail = 0;
	intel_ring_update_space(ringbuf);
}

static int logical_ring_prepare(struct drm_i915_gem_request *req, int bytes)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	int remain_usable = ringbuf->effective_size - ringbuf->tail;
	int remain_actual = ringbuf->size - ringbuf->tail;
	int ret, total_bytes, wait_bytes = 0;
	bool need_wrap = false;

	if (ringbuf->reserved_in_use)
		total_bytes = bytes;
	else
		total_bytes = bytes + ringbuf->reserved_size;

	if (unlikely(bytes > remain_usable)) {
		/*
		 * Not enough space for the basic request. So need to flush
		 * out the remainder and then wait for base + reserved.
		 */
		wait_bytes = remain_actual + total_bytes;
		need_wrap = true;
	} else {
		if (unlikely(total_bytes > remain_usable)) {
			/*
			 * The base request will fit but the reserved space
			 * falls off the end. So don't need an immediate wrap
			 * and only need to effectively wait for the reserved
			 * size space from the start of ringbuffer.
			 */
			wait_bytes = remain_actual + ringbuf->reserved_size;
		} else if (total_bytes > ringbuf->space) {
			/* No wrapping required, just waiting. */
			wait_bytes = total_bytes;
		}
	}

	if (wait_bytes) {
		ret = logical_ring_wait_for_space(req, wait_bytes);
		if (unlikely(ret))
			return ret;

		if (need_wrap)
			__wrap_ring_buffer(ringbuf);
	}

	return 0;
}

/**
 * intel_logical_ring_begin() - prepare the logical ringbuffer to accept some commands
 *
 * @req: The request to start some new work for
 * @num_dwords: number of DWORDs that we plan to write to the ringbuffer.
 *
 * The ringbuffer might not be ready to accept the commands right away (maybe it needs to
 * be wrapped, or wait a bit for the tail to be updated). This function takes care of that
 * and also preallocates a request (every workload submission is still mediated through
 * requests, same as it did with legacy ringbuffer submission).
 *
 * Return: non-zero if the ringbuffer is not ready to be written to.
 */
int intel_logical_ring_begin(struct drm_i915_gem_request *req, int num_dwords)
{
	struct drm_i915_private *dev_priv;
	int ret;

	WARN_ON(req == NULL);
	dev_priv = req->engine->dev->dev_private;

	ret = i915_gem_check_wedge(&dev_priv->gpu_error,
				   req->engine,
				   dev_priv->mm.interruptible);
	if (ret)
		return ret;

	ret = logical_ring_prepare(req, num_dwords * sizeof(uint32_t));
	if (ret)
		return ret;

	req->ringbuf->space -= num_dwords * sizeof(uint32_t);
	return 0;
}

int intel_logical_ring_reserve_space(struct drm_i915_gem_request *request)
{
	/*
	 * The first call merely notes the reserve request and is common for
	 * all back ends. The subsequent localised _begin() call actually
	 * ensures that the reservation is available. Without the begin, if
	 * the request creator immediately submitted the request without
	 * adding any commands to it then there might not actually be
	 * sufficient room for the submission commands.
	 */
	intel_ring_reserved_space_reserve(request->ringbuf, MIN_SPACE_FOR_ADD_REQUEST);

	return intel_logical_ring_begin(request, 0);
}

/**
 * execlists_submission() - submit a batchbuffer for execution, Execlists style
 * @dev: DRM device.
 * @file: DRM file.
 * @ring: Engine Command Streamer to submit to.
 * @ctx: Context to employ for this submission.
 * @args: execbuffer call arguments.
 * @vmas: list of vmas.
 * @batch_obj: the batchbuffer to submit.
 * @exec_start: batchbuffer start virtual address pointer.
 * @dispatch_flags: translated execbuffer call flags.
 *
 * This is the evil twin version of i915_gem_ringbuffer_submission. It abstracts
 * away the submission details of the execbuffer ioctl call.
 *
 * Return: non-zero if the submission fails.
 */
int intel_execlists_submission(struct i915_execbuffer_params *params,
			       struct drm_i915_gem_execbuffer2 *args,
			       struct list_head *vmas)
{
	struct i915_scheduler_queue_entry *qe;
	struct drm_device       *dev = params->dev;
	struct intel_engine_cs *engine = params->engine;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	params->instp_mode = args->flags & I915_EXEC_CONSTANTS_MASK;
	params->instp_mask = I915_EXEC_CONSTANTS_MASK;
	switch (params->instp_mode) {
	case I915_EXEC_CONSTANTS_REL_GENERAL:
	case I915_EXEC_CONSTANTS_ABSOLUTE:
	case I915_EXEC_CONSTANTS_REL_SURFACE:
		if (params->instp_mode != 0 && engine != &dev_priv->engine[RCS]) {
			DRM_DEBUG("non-0 rel constants mode on non-RCS\n");
			return -EINVAL;
		}

		if (params->instp_mode != params->ctx->relative_constants_mode) {
			if (params->instp_mode == I915_EXEC_CONSTANTS_REL_SURFACE) {
				DRM_DEBUG("rel surface constants mode invalid on gen5+\n");
				return -EINVAL;
			}

			/* The HW changed the meaning on this bit on gen6 */
			params->instp_mask &= ~I915_EXEC_CONSTANTS_REL_SURFACE;
		}
		break;
	default:
		DRM_DEBUG("execbuf with unknown constants: %d\n", params->instp_mode);
		return -EINVAL;
	}

	if (args->flags & I915_EXEC_GEN7_SOL_RESET) {
		DRM_DEBUG("sol reset is gen7 only\n");
		return -EINVAL;
	}

	ret = execlists_move_to_gpu(params->request, vmas);
	if (ret)
		return ret;

	i915_gem_execbuffer_move_to_active(vmas, params->request);

	trace_i915_gem_ring_queue(engine, params);

	qe = container_of(params, typeof(*qe), params);
	ret = i915_scheduler_queue_execbuffer(qe);
	if (ret)
		return ret;

	return 0;
}

/*
 * This function stores the specified constant value in the (index)th DWORD
 * of the hardware status page (execlist mode only). See separate code for
 * legacy mode.
 */
static void
emit_store_dw_index(struct drm_i915_gem_request *req, uint32_t value,
		    uint32_t index)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	uint64_t hwpa = req->engine->status_page.gfx_addr;

	hwpa += index << MI_STORE_DWORD_INDEX_SHIFT;

	intel_logical_ring_emit(ringbuf, MI_STORE_DWORD_IMM_GEN4 |
					 MI_GLOBAL_GTT);
	intel_logical_ring_emit(ringbuf, lower_32_bits(hwpa));
	intel_logical_ring_emit(ringbuf, upper_32_bits(hwpa)); /* GEN8+ */
	intel_logical_ring_emit(ringbuf, value);

	req->engine->gpu_caches_dirty = true;
}

#if	0
/*
 * This function stores the specified register value in the (index)th DWORD
 * of the hardware status page (execlist mode only). See separate code for
 * legacy mode.
 */
static void
emit_store_reg_index(struct drm_i915_gem_request *req, i915_reg_t reg,
		     uint32_t index)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	uint64_t hwpa = req->engine->status_page.gfx_addr;

	hwpa += index << MI_STORE_DWORD_INDEX_SHIFT;

	intel_logical_ring_emit(ringbuf, (MI_STORE_REG_MEM+1) | MI_GLOBAL_GTT);
	intel_logical_ring_emit_reg(ringbuf, reg);
	intel_logical_ring_emit(ringbuf, lower_32_bits(hwpa));
	intel_logical_ring_emit(ringbuf, upper_32_bits(hwpa)); /* GEN8+ */

	req->engine->gpu_caches_dirty = true;
}
#endif	/* 0 */

/*
 * This function stores the two specified values in the (index)th DWORD
 * and the following DWORD of the hardware status page (execlist mode only).
 * See separate code for legacy mode.
 */
static int
gen8_emit_flush_qw_store_index(struct drm_i915_gem_request *request,
			       uint32_t flags, uint32_t index,
			       uint32_t data1, uint32_t data2)
{
	struct intel_ringbuffer *ringbuf = request->ringbuf;
	uint32_t cmd;
	int ret;

	cmd = MI_FLUSH_DW;
	cmd += 2;			/* 64-bit address and data	*/
	cmd |= MI_FLUSH_DW_OP_STOREDW;	/* Store {D,Q}Word as post-op	*/
	cmd |= MI_FLUSH_DW_STORE_INDEX;	/* Address is relative to HWSP	*/
	cmd |= flags;			/* Extra (invalidate) bits	*/

	/* The address must be QWord aligned (index must be EVEN) */
	index <<= MI_STORE_DWORD_INDEX_SHIFT;
	if (WARN_ON_ONCE(index & 7))
		return -EINVAL;
	/* w/a: bit 5 needs to be zero for MI_FLUSH_DW QWord address. */
	if (WARN_ON_ONCE(index & (1 << 5)))
		return -EINVAL;
	index |= MI_FLUSH_DW_USE_GTT;

	ret = intel_logical_ring_begin(request, 6);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, cmd);
	intel_logical_ring_emit(ringbuf, index);
	intel_logical_ring_emit(ringbuf, 0);	/* upper_32_bits(index)	*/
	intel_logical_ring_emit(ringbuf, data1);
	intel_logical_ring_emit(ringbuf, data2);

	intel_logical_ring_emit(ringbuf, MI_NOOP);

	intel_logical_ring_advance(ringbuf);
	return 0;
}

/*
 * This function stores the two specified values in the (index)th DWORD
 * and the following DWORD of the hardware status page (execlist mode only).
 * See separate code for legacy mode.
 */
static int
gen8_emit_pipe_control_qw_store_index(struct drm_i915_gem_request *request,
				      uint32_t flags, uint32_t index,
				      uint32_t data1, uint32_t data2)
{
	struct intel_ringbuffer *ringbuf = request->ringbuf;
	const uint32_t cmd = GFX_OP_PIPE_CONTROL(6);
	uint32_t opts;
	int ret;

	/* The address must be QWord aligned (index must be EVEN) */
	index <<= MI_STORE_DWORD_INDEX_SHIFT;
	if (WARN_ON_ONCE(index & 7))
		return -EINVAL;
	/* w/a: bit 5 needs to be zero for MI_FLUSH_DW QWord address. */
	if (WARN_ON_ONCE(index & (1 << 5)))
		return -EINVAL;

	/* We're going to emit 3 PIPE_CONTROLs of 6 DWords each */
	ret = intel_logical_ring_begin(request, 3*6);
	if (ret)
		return ret;

	/* W/A: stall before flush_enable */
	opts = PIPE_CONTROL_CS_STALL | PIPE_CONTROL_STALL_AT_SCOREBOARD;
	intel_logical_ring_emit(ringbuf, cmd);
	intel_logical_ring_emit(ringbuf, opts);
	intel_logical_ring_emit(ringbuf, 0);
	intel_logical_ring_emit(ringbuf, 0);
	intel_logical_ring_emit(ringbuf, 0);
	intel_logical_ring_emit(ringbuf, 0);

	/* W/A: post-op before render flush */
	opts |= PIPE_CONTROL_POSTSYNC_FLUSH;
	opts |= PIPE_CONTROL_GEN7_GLOBAL_GTT;	/* Address via GGTT	*/
	opts |= PIPE_CONTROL_STORE_DATA_INDEX;	/* Index into HWSP	*/
	opts |= PIPE_CONTROL_QW_WRITE;		/* Write QWord		*/
	intel_logical_ring_emit(ringbuf, cmd);
	intel_logical_ring_emit(ringbuf, opts);
	intel_logical_ring_emit(ringbuf, index);
	intel_logical_ring_emit(ringbuf, 0);	/* upper_32_bits(index)	*/
	intel_logical_ring_emit(ringbuf, data1);
	intel_logical_ring_emit(ringbuf, data2);

	opts &= ~PIPE_CONTROL_STALL_AT_SCOREBOARD;
	opts |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
	opts |= flags;				/* Extra flag bits	*/
	intel_logical_ring_emit(ringbuf, cmd);
	intel_logical_ring_emit(ringbuf, opts);
	intel_logical_ring_emit(ringbuf, index);
	intel_logical_ring_emit(ringbuf, 0);	/* upper_32_bits(index)	*/
	intel_logical_ring_emit(ringbuf, data1);
	intel_logical_ring_emit(ringbuf, data2);

	intel_logical_ring_advance(ringbuf);
	return 0;
}

/*
 * Selects which preemption control bits to use.
 * Unset: use 0x20e4 and 0x20ec (non-ctx)
 * Set: use 0x2580 (per-ctx, whitelisted)
 */
#define GEN7_FF_SLICE_CS_CHICKEN1		_MMIO(0x20e0)
#define   GEN9_FFSC_PERCTX_PREEMPT_CTRL		(1<<14)

#define FF_SLICE_CS_CHICKEN2			_MMIO(0x20e4)
/* FF_SLICE_CS_CHICKEN2 and GEN8_CS_CHICKEN1 */
#define	  MID_THREAD_PREEMPTION			(0<<1)
#define	  THREAD_GROUP_PREEMPTION		(1<<1)
#define	  COMMAND_LEVEL_PREEMPTION		(2<<1)
#define	  PREEMPTION_LEVEL_MASK			(3<<1)

#define GEN9_CS_DEBUG_MODE			_MMIO(0x20ec)
/* GEN9_CS_DEBUG_MODE and GEN8_CS_CHICKEN1 */
#define	  GEN9_GFX_REPLAY_MODE			(1<<0)
#define	  GEN10_DISABLE_GPGPU_WALKER_PREEMPTION	(1<<15)

/* Whitelisted user-writable preemption control register */
#define GEN8_CS_CHICKEN1			_MMIO(0x2580)

static void
emit_preemption_control(struct drm_i915_gem_request *req)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	int preemption_level = i915.preemption_level;
	u32 allow_user_control = false;
	u32 allow_mid_object = false;
	u32 allow_mid_walker = false;
	u32 mid_thread_mode = 0;
	u32 data;

	/* Preemption is always disabled while preempting */
	if (req->scheduler_flags & I915_REQ_SF_PREEMPT) {
		data = MI_ARB_ON_OFF | MI_ARB_DISABLE;
		intel_logical_ring_emit(ringbuf, data);
		intel_logical_ring_emit(ringbuf, MI_NOOP);
		return;
	}

	/* At level 0, no preemption will be attempted */
	if (preemption_level == 0)
		return;

	/* All other levels check for preemption before starting the batch */
	intel_logical_ring_emit(ringbuf, MI_ARB_ON_OFF | MI_ARB_ENABLE);
	intel_logical_ring_emit(ringbuf, MI_ARB_CHECK);

	/* Non-render rings support between-batch only, for now */
	if (req->engine->id != RCS)
		preemption_level = 1;
	else if (preemption_level < 0)
		preemption_level = 3;	/* Current safe default	*/

	switch (preemption_level) {
	case 1:
		/*
		 * Between-batch preemption only - no mid-batch allowed.
		 *
		 * XXX: use preemption disable bit in debug register instead
		 * to avoid collision with ARB_OFF/ARB_ON in workaround batch?
		 */
		data = MI_ARB_ON_OFF | MI_ARB_DISABLE;
		intel_logical_ring_emit(ringbuf, data);
		intel_logical_ring_emit(ringbuf, MI_NOOP);
		return;

	case 2:
		/*
		 * Co-operative mid-batch only. This register is not saved
		 * with the context and so must be reprogrammed each time.
		 *
		 * XXX: need to fix up in workaround batch?
		 */
		data = _MASKED_BIT_ENABLE(PREEMPT_ON_ARB_CHK_ONLY);
		intel_logical_ring_emit(ringbuf, MI_NOOP);
		intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(1));
		intel_logical_ring_emit_reg(ringbuf, PREEMPT_DEBUG(req->engine));
		intel_logical_ring_emit(ringbuf, data);
		return;

	case 3:
		/* Mid-batch preemption allowed, but not thread-level */
		mid_thread_mode = _MASKED_FIELD(PREEMPTION_LEVEL_MASK, COMMAND_LEVEL_PREEMPTION);
		break;
	case 4:
		/* Thread-group preemption allowed, but not mid-thread */
		mid_thread_mode = _MASKED_FIELD(PREEMPTION_LEVEL_MASK, THREAD_GROUP_PREEMPTION);
		break;
	case 5:
		/* Thread-group preemption allowed, also mid-object */
		mid_thread_mode = _MASKED_FIELD(PREEMPTION_LEVEL_MASK, THREAD_GROUP_PREEMPTION);
		allow_mid_object = true;
		break;
	case 6:
		/* Thread-group and mid-object preemption allowed; user can override */
		mid_thread_mode = _MASKED_FIELD(PREEMPTION_LEVEL_MASK, THREAD_GROUP_PREEMPTION);
		allow_mid_object = true;
		allow_user_control = true;
		break;
	case 7:
		/* Mid-thread and mid-object preemption allowed; user can override */
		mid_thread_mode = _MASKED_FIELD(PREEMPTION_LEVEL_MASK, MID_THREAD_PREEMPTION);
		allow_mid_object = true;
		allow_user_control = true;
		break;
	case 8:
		/* Mid-thread and mid-walker and mid-object preemption allowed; user can override */
		mid_thread_mode = _MASKED_FIELD(PREEMPTION_LEVEL_MASK, MID_THREAD_PREEMPTION);
		allow_mid_walker = true;
		allow_mid_object = true;
		allow_user_control = true;
		break;

	case 9:
		/* XXX: any further levels of preemption? */
	default:
		/* Default is the maximum level permitted by the hardware */
		return;
	}

	/*
	 * If GEN9_FFSC_PERCTX_PREEMPT_CTRL is disabled, the values in
	 * GEN9_CS_DEBUG_MODE and FF_SLICE_CS_CHICKEN2 will be used in
	 * preference to those in GEN8_CS_CHICKEN1, so the user batch
	 * cannot override them. Enabling GEN9_FFSC_PERCTX_PREEMPT_CTRL
	 * causes the values in GEN8_CS_CHICKEN1 to be used instead;
	 * this register has been whitelisted, so the user batch can
	 * override the levels we set here; in particular, it can ENABLE
	 * mid-thread preemption. We don't offer that option here as it
	 * must only be set for GPGPU workloads, not MEDIA
	 */
	if (allow_user_control)
		allow_user_control = _MASKED_BIT_ENABLE(GEN9_FFSC_PERCTX_PREEMPT_CTRL);
	else
		allow_user_control = _MASKED_BIT_DISABLE(GEN9_FFSC_PERCTX_PREEMPT_CTRL);
	if (allow_mid_object)
		allow_mid_object = _MASKED_BIT_ENABLE(GEN9_GFX_REPLAY_MODE);
	else
		allow_mid_object = _MASKED_BIT_DISABLE(GEN9_GFX_REPLAY_MODE);
	if (allow_mid_walker)
		allow_mid_object |= _MASKED_BIT_DISABLE(GEN10_DISABLE_GPGPU_WALKER_PREEMPTION);
	else
		allow_mid_object |= _MASKED_BIT_ENABLE(GEN10_DISABLE_GPGPU_WALKER_PREEMPTION);

	intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(4));
	intel_logical_ring_emit_reg(ringbuf, GEN7_FF_SLICE_CS_CHICKEN1);
	intel_logical_ring_emit(ringbuf, allow_user_control);
	intel_logical_ring_emit_reg(ringbuf, GEN9_CS_DEBUG_MODE);
	intel_logical_ring_emit(ringbuf, allow_mid_object);
	intel_logical_ring_emit_reg(ringbuf, FF_SLICE_CS_CHICKEN2);
	intel_logical_ring_emit(ringbuf, mid_thread_mode);
	intel_logical_ring_emit_reg(ringbuf, GEN8_CS_CHICKEN1);
	intel_logical_ring_emit(ringbuf, mid_thread_mode | allow_mid_object);
	intel_logical_ring_emit(ringbuf, MI_NOOP);
}

/*
 * Emit the commands to execute when preparing to start a batch
 *
 * The GPU will log the seqno of the batch before it starts
 * running any of the commands to actually execute that batch
 */
static void
emit_preamble(struct drm_i915_gem_request *req)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	uint32_t seqno = i915_gem_request_get_seqno(req);

	WARN_ON(!seqno);

	if (req->scheduler_flags & I915_REQ_SF_PREEMPT)
		emit_store_dw_index(req, seqno, I915_PREEMPTIVE_ACTIVE_SEQNO);
	else
		emit_store_dw_index(req, seqno, I915_BATCH_ACTIVE_SEQNO);

	intel_logical_ring_emit(ringbuf, MI_REPORT_HEAD);
	intel_logical_ring_emit(ringbuf, MI_NOOP);

	req->engine->gpu_caches_dirty = true;
}

static void
emit_relconsts_mode(struct i915_execbuffer_params *params)
{
	if (params->instp_mode != params->ctx->relative_constants_mode) {
		struct intel_ringbuffer *ringbuf = params->request->ringbuf;
		uint32_t val = params->instp_mask << 16 | params->instp_mode;

		intel_logical_ring_emit(ringbuf, MI_NOOP);
		intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(1));
		intel_logical_ring_emit_reg(ringbuf, INSTPM);
		intel_logical_ring_emit(ringbuf, val);

		params->ctx->relative_constants_mode = params->instp_mode;
	}
}

/*
 * This is the main function for sending a batch to the engine.
 * It is called from the scheduler, with the struct_mutex already held.
 */
int intel_execlists_submission_final(struct i915_execbuffer_params *params)
{
	struct drm_i915_private *dev_priv = to_i915(params->dev);
	struct drm_i915_gem_request *req = params->request;
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	struct intel_engine_cs *engine = params->engine;
	u64 exec_start;
	int ret;
	uint32_t min_space;

	/* The mutex must be acquired before calling this function */
	WARN_ON(!mutex_is_locked(&params->dev->struct_mutex));

	/* Check the context wasn't banned between submission and execution: */
	if (params->ctx->hang_stats.banned) {
		DRM_DEBUG("Trying to execute for banned context!\n");
		return -ENOENT;
	}

	/* Make sure the request's seqno is the latest and greatest: */
	if (req->reserved_seqno != dev_priv->last_seqno) {
		ret = i915_gem_get_seqno(engine->dev, &req->reserved_seqno);
		if (ret)
			return ret;
	}
	/*
	 * And make it live because some of the execbuff submission code
	 * requires the seqno to be available up front.
	 */
	WARN_ON(req->seqno);
	req->seqno = req->reserved_seqno;
	WARN_ON(req->seqno != dev_priv->last_seqno);

	ret = intel_logical_ring_reserve_space(req);
	if (ret)
		goto err;

	/*
	 * It would be a bad idea to run out of space while writing commands
	 * to the ring. One of the major aims of the scheduler is to not
	 * stall at any point for any reason. However, doing an early exit
	 * half way through submission could result in a partial sequence
	 * being written which would leave the engine in an unknown state.
	 * Therefore, check in advance that there will be enough space for
	 * the entire submission whether emitted by the code below OR by any
	 * other functions that may be executed before the end of final().
	 *
	 * NB: This test deliberately overestimates, because that's easier
	 * than tracing every potential path that could be taken!
	 *
	 * Current measurements suggest that we may need to emit up to 186
	 * dwords, so this is rounded up to 256 here. Then double that to get
	 * the free space requirement, because the block is not allowed to
	 * span the transition from the end to the beginning of the ring.
	 */
#define I915_BATCH_EXEC_MAX_LEN         256	/* max dwords emitted here */
	min_space = I915_BATCH_EXEC_MAX_LEN * 2 * sizeof(uint32_t);
	ret = logical_ring_test_space(ringbuf, min_space);
	if (ret)
		goto err;

	ret = intel_logical_ring_begin(req, I915_BATCH_EXEC_MAX_LEN);
	if (ret)
		goto err;

	/*
	 * For the case of restarting a mid-batch preempted request,
	 * the ringbuffer already contains all necessary instructions,
	 * so we can just go straight to submitting it
	 */
	if ((req->scheduler_flags & I915_REQ_SF_RESTART)) {
		DRM_DEBUG_DRIVER("restart: req head/tail 0x%x/%x ringbuf 0x%x/%x\n",
			req->head, req->tail, ringbuf->head, ringbuf->tail);
		i915_gem_execbuffer_retire_commands(params);
		return 0;
	}

	/* record where we start filling the ring */
	req->head = intel_ring_get_tail(ringbuf);

	/*
	 * Emit the preemption control and preamble for the batch
	 */
	emit_preemption_control(req);
	emit_preamble(req);

	/*
	 * Unconditionally invalidate gpu caches and ensure that we do flush
	 * any residual writes from the previous batch.
	 */
	ret = logical_ring_invalidate_all_caches(req);
	if (ret)
		goto err;

	if (!(req->scheduler_flags & I915_REQ_SF_PREEMPT)) {
		if (engine == &dev_priv->engine[RCS])
			emit_relconsts_mode(params);

		exec_start = params->batch_obj_vm_offset +
			     params->args_batch_start_offset;

	i915_emit_profiling_data(params->request, params->ctx->user_handle,
				params->tag);

		ret = engine->emit_bb_start(req, exec_start,
					    params->dispatch_flags);
		if (ret)
			goto err;

	i915_emit_profiling_data(params->request, params->ctx->user_handle,
				params->tag);

		trace_i915_gem_ring_dispatch(req, params->dispatch_flags);
	}

	i915_gem_execbuffer_retire_commands(params);

	return 0;

err:
	intel_ring_reserved_space_cancel(params->request->ringbuf);

	return ret;
}

void intel_execlists_retire_requests(struct intel_engine_cs *engine)
{
	struct drm_i915_gem_request *req, *tmp;
	struct list_head retired_list;

	WARN_ON(!mutex_is_locked(&engine->dev->struct_mutex));
	if (list_empty(&engine->execlist_retired_req_list))
		return;

	INIT_LIST_HEAD(&retired_list);
	spin_lock_irq(&engine->execlist_lock);
	list_replace_init(&engine->execlist_retired_req_list, &retired_list);
	spin_unlock_irq(&engine->execlist_lock);

	list_for_each_entry_safe(req, tmp, &retired_list, execlist_link) {
		struct intel_context *ctx = req->ctx;
		struct drm_i915_gem_object *ctx_obj =
				ctx->engine[engine->id].state;

		if (ctx_obj && (ctx != engine->default_context))
			intel_lr_context_unpin(req);
		list_del(&req->execlist_link);
		i915_gem_request_unreference(req);
	}
}

void intel_logical_ring_stop(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	int ret;

	if (!intel_engine_initialized(engine))
		return;

	ret = intel_engine_idle_flush(engine);
	if (ret && !i915_reset_in_progress(&to_i915(engine->dev)->gpu_error))
		DRM_ERROR("failed to quiesce %s whilst cleaning up: %d\n",
			  engine->name, ret);

	/* TODO: Is this correct with Execlists enabled? */
	I915_WRITE_MODE(engine, _MASKED_BIT_ENABLE(STOP_RING));
	if (wait_for((I915_READ_MODE(engine) & MODE_IDLE) != 0, 1000)) {
		DRM_ERROR("%s :timed out trying to stop ring\n", engine->name);
		return;
	}
	I915_WRITE_MODE(engine, _MASKED_BIT_DISABLE(STOP_RING));
}

int logical_ring_flush_all_caches(struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = req->engine;
	int ret;

	if (!engine->gpu_caches_dirty)
		return 0;

	ret = engine->emit_flush(req, 0, I915_GEM_GPU_DOMAINS);
	if (ret)
		return ret;

	engine->gpu_caches_dirty = false;
	return 0;
}

static int intel_lr_context_do_pin(struct intel_engine_cs *engine,
		struct drm_i915_gem_object *ctx_obj,
		struct intel_ringbuffer *ringbuf)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret = 0;

	WARN_ON(!mutex_is_locked(&engine->dev->struct_mutex));
	ret = i915_gem_obj_ggtt_pin(ctx_obj, GEN8_LR_CONTEXT_ALIGN,
			PIN_OFFSET_BIAS | GUC_WOPCM_TOP);
	if (ret)
		return ret;

	ret = intel_pin_and_map_ringbuffer_obj(engine->dev, ringbuf);
	if (ret)
		goto unpin_ctx_obj;

	ctx_obj->dirty = true;

	/* Invalidate GuC TLB. */
	if (i915.enable_guc_submission)
		I915_WRITE(GEN8_GTCR, GEN8_GTCR_INVALIDATE);

	return ret;

unpin_ctx_obj:
	i915_gem_object_ggtt_unpin(ctx_obj);

	return ret;
}

static int intel_lr_context_pin(struct drm_i915_gem_request *rq)
{
	int ret = 0;
	struct intel_engine_cs *engine = rq->engine;
	struct drm_i915_gem_object *ctx_obj = rq->ctx->engine[engine->id].state;
	struct intel_ringbuffer *ringbuf = rq->ringbuf;

	if (rq->ctx->engine[engine->id].pin_count++ == 0) {
		ret = intel_lr_context_do_pin(engine, ctx_obj, ringbuf);
		if (ret)
			goto reset_pin_count;
	}
	return ret;

reset_pin_count:
	rq->ctx->engine[engine->id].pin_count = 0;
	return ret;
}

void intel_lr_context_unpin(struct drm_i915_gem_request *rq)
{
	struct intel_engine_cs *engine = rq->engine;
	struct drm_i915_gem_object *ctx_obj = rq->ctx->engine[engine->id].state;
	struct intel_ringbuffer *ringbuf = rq->ringbuf;

	if (ctx_obj) {
		WARN_ON(!mutex_is_locked(&engine->dev->struct_mutex));
		if (--rq->ctx->engine[engine->id].pin_count == 0) {
			intel_unpin_ringbuffer_obj(ringbuf);
			i915_gem_object_ggtt_unpin(ctx_obj);
		}
	}
}

static int intel_logical_ring_workarounds_emit(struct drm_i915_gem_request *req)
{
	int ret, i;
	struct intel_engine_cs *engine = req->engine;
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_workarounds *w = &dev_priv->workarounds;

	if (w->count == 0)
		return 0;

	engine->gpu_caches_dirty = true;
	ret = logical_ring_flush_all_caches(req);
	if (ret)
		return ret;

	ret = intel_logical_ring_begin(req, w->count * 2 + 2);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(w->count));
	for (i = 0; i < w->count; i++) {
		intel_logical_ring_emit_reg(ringbuf, w->reg[i].addr);
		intel_logical_ring_emit(ringbuf, w->reg[i].value);
	}
	intel_logical_ring_emit(ringbuf, MI_NOOP);

	intel_logical_ring_advance(ringbuf);

	engine->gpu_caches_dirty = true;
	ret = logical_ring_flush_all_caches(req);
	if (ret)
		return ret;

	return 0;
}

#define wa_ctx_emit(batch, index, cmd)					\
	do {								\
		int __index = (index)++;				\
		if (WARN_ON(__index >= (PAGE_SIZE / sizeof(uint32_t)))) { \
			return -ENOSPC;					\
		}							\
		batch[__index] = (cmd);					\
	} while (0)

#define wa_ctx_emit_reg(batch, index, reg) \
	wa_ctx_emit((batch), (index), i915_mmio_reg_offset(reg))

/*
 * In this WA we need to set GEN8_L3SQCREG4[21:21] and reset it after
 * PIPE_CONTROL instruction. This is required for the flush to happen correctly
 * but there is a slight complication as this is applied in WA batch where the
 * values are only initialized once so we cannot take register value at the
 * beginning and reuse it further; hence we save its value to memory, upload a
 * constant value with bit21 set and then we restore it back with the saved value.
 * To simplify the WA, a constant value is formed by using the default value
 * of this register. This shouldn't be a problem because we are only modifying
 * it for a short period and this batch in non-premptible. We can ofcourse
 * use additional instructions that read the actual value of the register
 * at that time and set our bit of interest but it makes the WA complicated.
 *
 * This WA is also required for Gen9 so extracting as a function avoids
 * code duplication.
 */
static inline int gen8_emit_flush_coherentl3_wa(struct intel_engine_cs *engine,
						uint32_t *const batch,
						uint32_t index)
{
	uint32_t l3sqc4_flush = (0x40400000 | GEN8_LQSC_FLUSH_COHERENT_LINES);

	/*
	 * WaDisableLSQCROPERFforOCL:skl
	 * This WA is implemented in skl_init_clock_gating() but since
	 * this batch updates GEN8_L3SQCREG4 with default value we need to
	 * set this bit here to retain the WA during flush.
	 */
	if (IS_SKL_REVID(engine->dev, 0, SKL_REVID_E0))
		l3sqc4_flush |= GEN8_LQSC_RO_PERF_DIS;

	wa_ctx_emit(batch, index, (MI_STORE_REGISTER_MEM_GEN8 |
				   MI_SRM_LRM_GLOBAL_GTT));
	wa_ctx_emit_reg(batch, index, GEN8_L3SQCREG4);
	wa_ctx_emit(batch, index, engine->scratch.gtt_offset + 256);
	wa_ctx_emit(batch, index, 0);

	wa_ctx_emit(batch, index, MI_LOAD_REGISTER_IMM(1));
	wa_ctx_emit_reg(batch, index, GEN8_L3SQCREG4);
	wa_ctx_emit(batch, index, l3sqc4_flush);
	wa_ctx_emit(batch, index, MI_NOOP);

	wa_ctx_emit(batch, index, GFX_OP_PIPE_CONTROL(6));
	wa_ctx_emit(batch, index, (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_STALL_AT_SCOREBOARD));
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);

	wa_ctx_emit(batch, index, GFX_OP_PIPE_CONTROL(6));
	wa_ctx_emit(batch, index, (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_L3_DC_FLUSH));
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);

	wa_ctx_emit(batch, index, (MI_LOAD_REGISTER_MEM_GEN8 |
				   MI_SRM_LRM_GLOBAL_GTT));
	wa_ctx_emit_reg(batch, index, GEN8_L3SQCREG4);
	wa_ctx_emit(batch, index, engine->scratch.gtt_offset + 256);
	wa_ctx_emit(batch, index, 0);

	wa_ctx_emit(batch, index, GFX_OP_PIPE_CONTROL(6));
	wa_ctx_emit(batch, index, (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_POSTSYNC_FLUSH));
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);

	return index;
}

static inline uint32_t wa_ctx_start(struct i915_wa_ctx_bb *wa_ctx,
				    uint32_t offset,
				    uint32_t start_alignment)
{
	return wa_ctx->offset = ALIGN(offset, start_alignment);
}

static inline int wa_ctx_end(struct i915_wa_ctx_bb *wa_ctx,
			     uint32_t offset,
			     uint32_t size_alignment)
{
	wa_ctx->size = offset - wa_ctx->offset;

	WARN(wa_ctx->size % size_alignment,
	     "wa_ctx_bb failed sanity checks: size %d is not aligned to %d\n",
	     wa_ctx->size, size_alignment);
	return 0;
}

/**
 * gen8_init_indirectctx_bb() - initialize indirect ctx batch with WA
 *
 * @ring: only applicable for RCS
 * @wa_ctx: structure representing wa_ctx
 *  offset: specifies start of the batch, should be cache-aligned. This is updated
 *    with the offset value received as input.
 *  size: size of the batch in DWORDS but HW expects in terms of cachelines
 * @batch: page in which WA are loaded
 * @offset: This field specifies the start of the batch, it should be
 *  cache-aligned otherwise it is adjusted accordingly.
 *  Typically we only have one indirect_ctx and per_ctx batch buffer which are
 *  initialized at the beginning and shared across all contexts but this field
 *  helps us to have multiple batches at different offsets and select them based
 *  on a criteria. At the moment this batch always start at the beginning of the page
 *  and at this point we don't have multiple wa_ctx batch buffers.
 *
 *  The number of WA applied are not known at the beginning; we use this field
 *  to return the no of DWORDS written.
 *
 *  It is to be noted that this batch does not contain MI_BATCH_BUFFER_END
 *  so it adds NOOPs as padding to make it cacheline aligned.
 *  MI_BATCH_BUFFER_END will be added to perctx batch and both of them together
 *  makes a complete batch buffer.
 *
 * Return: non-zero if we exceed the PAGE_SIZE limit.
 */

static int gen8_init_indirectctx_bb(struct intel_engine_cs *engine,
				    struct i915_wa_ctx_bb *wa_ctx,
				    uint32_t *const batch,
				    uint32_t *offset)
{
	uint32_t scratch_addr;
	uint32_t index = wa_ctx_start(wa_ctx, *offset, CACHELINE_DWORDS);

	/* WaDisableCtxRestoreArbitration:bdw,chv */
	wa_ctx_emit(batch, index, MI_ARB_ON_OFF | MI_ARB_DISABLE);

	/* WaFlushCoherentL3CacheLinesAtContextSwitch:bdw */
	if (IS_BROADWELL(engine->dev)) {
		int rc = gen8_emit_flush_coherentl3_wa(engine, batch, index);
		if (rc < 0)
			return rc;
		index = rc;
	}

	/* WaClearSlmSpaceAtContextSwitch:bdw,chv */
	/* Actual scratch location is at 128 bytes offset */
	scratch_addr = engine->scratch.gtt_offset + 2*CACHELINE_BYTES;

	wa_ctx_emit(batch, index, GFX_OP_PIPE_CONTROL(6));
	wa_ctx_emit(batch, index, (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_STALL_AT_SCOREBOARD));
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);

	wa_ctx_emit(batch, index, GFX_OP_PIPE_CONTROL(6));
	wa_ctx_emit(batch, index, (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_L3_DC_FLUSH |
				   PIPE_CONTROL_FLUSH_L3));
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);

	wa_ctx_emit(batch, index, GFX_OP_PIPE_CONTROL(6));
	wa_ctx_emit(batch, index, (PIPE_CONTROL_CS_STALL |
				   PIPE_CONTROL_POSTSYNC_FLUSH |
				   PIPE_CONTROL_QW_WRITE |
				   PIPE_CONTROL_GEN7_GLOBAL_GTT));
	wa_ctx_emit(batch, index, scratch_addr);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);
	wa_ctx_emit(batch, index, 0);

	/* Pad to end of cacheline */
	while (index % CACHELINE_DWORDS)
		wa_ctx_emit(batch, index, MI_NOOP);

	/*
	 * MI_BATCH_BUFFER_END is not required in Indirect ctx BB because
	 * execution depends on the length specified in terms of cache lines
	 * in the register CTX_RCS_INDIRECT_CTX
	 */

	return wa_ctx_end(wa_ctx, *offset = index, CACHELINE_DWORDS);
}

/**
 * gen8_init_perctx_bb() - initialize per ctx batch with WA
 *
 * @ring: only applicable for RCS
 * @wa_ctx: structure representing wa_ctx
 *  offset: specifies start of the batch, should be cache-aligned.
 *  size: size of the batch in DWORDS but HW expects in terms of cachelines
 * @batch: page in which WA are loaded
 * @offset: This field specifies the start of this batch.
 *   This batch is started immediately after indirect_ctx batch. Since we ensure
 *   that indirect_ctx ends on a cacheline this batch is aligned automatically.
 *
 *   The number of DWORDS written are returned using this field.
 *
 *  This batch is terminated with MI_BATCH_BUFFER_END and so we need not add padding
 *  to align it with cacheline as padding after MI_BATCH_BUFFER_END is redundant.
 */
static int gen8_init_perctx_bb(struct intel_engine_cs *engine,
			       struct i915_wa_ctx_bb *wa_ctx,
			       uint32_t *const batch,
			       uint32_t *offset)
{
	uint32_t index = wa_ctx_start(wa_ctx, *offset, CACHELINE_DWORDS);

	/* WaDisableCtxRestoreArbitration:bdw,chv */
	wa_ctx_emit(batch, index, MI_ARB_ON_OFF | MI_ARB_ENABLE);

	wa_ctx_emit(batch, index, MI_BATCH_BUFFER_END);

	return wa_ctx_end(wa_ctx, *offset = index, 1);
}

static int gen9_init_indirectctx_bb(struct intel_engine_cs *engine,
				    struct i915_wa_ctx_bb *wa_ctx,
				    uint32_t *const batch,
				    uint32_t *offset)
{
	int ret;
	struct drm_device *dev = engine->dev;
	uint32_t index = wa_ctx_start(wa_ctx, *offset, CACHELINE_DWORDS);

	/* WaDisableCtxRestoreArbitration:skl,bxt */
	if (IS_SKL_REVID(dev, 0, REVID_FOREVER) ||
	    IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		wa_ctx_emit(batch, index, MI_ARB_ON_OFF | MI_ARB_DISABLE);
		wa_ctx_emit(batch, index, MI_NOOP);
	}

	/* WaFlushCoherentL3CacheLinesAtContextSwitch:skl,bxt */
	ret = gen8_emit_flush_coherentl3_wa(engine, batch, index);
	if (ret < 0)
		return ret;
	index = ret;

	/* WaDisableGatherAtSetShaderCommonSlice:skl,bxt,kbl */
	wa_ctx_emit(batch, index, MI_LOAD_REGISTER_IMM(1));
	wa_ctx_emit_reg(batch, index, COMMON_SLICE_CHICKEN2);
	wa_ctx_emit(batch, index, _MASKED_BIT_DISABLE(
			    GEN9_DISABLE_GATHER_AT_SET_SHADER_COMMON_SLICE));
	wa_ctx_emit(batch, index, MI_NOOP);

	/* WaMediaPoolStateCmdInWABB:bxt */
	if (HAS_POOLED_EU(dev)) {
		/*
		 * EU pool configuration is setup along with golden context
		 * during context initialization. This value depends on
		 * device type (2x6 or 3x6) and needs to be updated based
		 * on which subslice is disabled especially for 2x6
		 * devices, however it is safe to load default
		 * configuration of 3x6 device instead of masking off
		 * corresponding bits because HW ignores bits of a disabled
		 * subslice and drops down to appropriate config. Please
		 * see render_state_setup() in i915_gem_render_state.c for
		 * possible configurations, to avoid duplication they are
		 * not shown here again.
		 */
		u32 eu_pool_config = 0x00777000;
		wa_ctx_emit(batch, index, GEN9_MEDIA_POOL_STATE);
		wa_ctx_emit(batch, index, GEN9_MEDIA_POOL_ENABLE);
		wa_ctx_emit(batch, index, eu_pool_config);
		wa_ctx_emit(batch, index, 0);
		wa_ctx_emit(batch, index, 0);
		wa_ctx_emit(batch, index, 0);
	}

	/* Pad to end of cacheline */
	while (index % CACHELINE_DWORDS)
		wa_ctx_emit(batch, index, MI_NOOP);

	return wa_ctx_end(wa_ctx, *offset = index, CACHELINE_DWORDS);
}

static int gen9_init_perctx_bb(struct intel_engine_cs *engine,
			       struct i915_wa_ctx_bb *wa_ctx,
			       uint32_t *const batch,
			       uint32_t *offset)
{
	struct drm_device *dev = engine->dev;
	uint32_t index = wa_ctx_start(wa_ctx, *offset, CACHELINE_DWORDS);

	/* WaSetDisablePixMaskCammingAndRhwoInCommonSliceChicken:skl,bxt */
	if (IS_SKL_REVID(dev, 0, SKL_REVID_B0) ||
	    IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		wa_ctx_emit(batch, index, MI_LOAD_REGISTER_IMM(1));
		wa_ctx_emit_reg(batch, index, GEN9_SLICE_COMMON_ECO_CHICKEN0);
		wa_ctx_emit(batch, index,
			    _MASKED_BIT_ENABLE(DISABLE_PIXEL_MASK_CAMMING));
		wa_ctx_emit(batch, index, MI_NOOP);
	}

	/* WaClearTdlStateAckDirtyBits:bxt */
	if (IS_BXT_REVID(dev, 0, BXT_REVID_B0)) {
		wa_ctx_emit(batch, index, MI_LOAD_REGISTER_IMM(4));

		wa_ctx_emit_reg(batch, index, GEN8_STATE_ACK);
		wa_ctx_emit(batch, index, _MASKED_BIT_DISABLE(GEN9_SUBSLICE_TDL_ACK_BITS));

		wa_ctx_emit_reg(batch, index, GEN9_STATE_ACK_SLICE1);
		wa_ctx_emit(batch, index, _MASKED_BIT_DISABLE(GEN9_SUBSLICE_TDL_ACK_BITS));

		wa_ctx_emit_reg(batch, index, GEN9_STATE_ACK_SLICE2);
		wa_ctx_emit(batch, index, _MASKED_BIT_DISABLE(GEN9_SUBSLICE_TDL_ACK_BITS));

		wa_ctx_emit_reg(batch, index, GEN7_ROW_CHICKEN2);
		/* dummy write to CS, mask bits are 0 to ensure the register is not modified */
		wa_ctx_emit(batch, index, 0x0);
		wa_ctx_emit(batch, index, MI_NOOP);
	}

	/* WaDisableCtxRestoreArbitration:skl,bxt */
	if (IS_SKL_REVID(dev, 0, REVID_FOREVER) ||
	    IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		wa_ctx_emit(batch, index, MI_ARB_ON_OFF | MI_ARB_ENABLE);
		wa_ctx_emit(batch, index, MI_NOOP);
	}

	wa_ctx_emit(batch, index, MI_BATCH_BUFFER_END);

	return wa_ctx_end(wa_ctx, *offset = index, 1);
}

static int lrc_setup_wa_ctx_obj(struct intel_engine_cs *engine, u32 size)
{
	int ret;

	engine->wa_ctx.obj = i915_gem_alloc_object(engine->dev,
						   PAGE_ALIGN(size));
	if (!engine->wa_ctx.obj) {
		DRM_DEBUG_DRIVER("alloc LRC WA ctx backing obj failed.\n");
		return -ENOMEM;
	}

	ret = i915_gem_obj_ggtt_pin(engine->wa_ctx.obj, PAGE_SIZE, 0);
	if (ret) {
		DRM_DEBUG_DRIVER("pin LRC WA ctx backing obj failed: %d\n",
				 ret);
		drm_gem_object_unreference(&engine->wa_ctx.obj->base);
		return ret;
	}

	return 0;
}

static void lrc_destroy_wa_ctx_obj(struct intel_engine_cs *engine)
{
	if (engine->wa_ctx.obj) {
		i915_gem_object_ggtt_unpin(engine->wa_ctx.obj);
		drm_gem_object_unreference(&engine->wa_ctx.obj->base);
		engine->wa_ctx.obj = NULL;
	}
}

static int intel_init_workaround_bb(struct intel_engine_cs *engine)
{
	int ret;
	uint32_t *batch;
	uint32_t offset;
	struct page *page;
	struct i915_ctx_workarounds *wa_ctx = &engine->wa_ctx;

	WARN_ON(engine->id != RCS);

	/* update this when WA for higher Gen are added */
	if (INTEL_INFO(engine->dev)->gen > 9) {
		DRM_ERROR("WA batch buffer is not initialized for Gen%d\n",
			  INTEL_INFO(engine->dev)->gen);
		return 0;
	}

	/* some WA perform writes to scratch page, ensure it is valid */
	if (engine->scratch.obj == NULL) {
		DRM_ERROR("scratch page not allocated for %s\n", engine->name);
		return -EINVAL;
	}

	ret = lrc_setup_wa_ctx_obj(engine, PAGE_SIZE);
	if (ret) {
		DRM_DEBUG_DRIVER("Failed to setup context WA page: %d\n", ret);
		return ret;
	}

	page = i915_gem_object_get_dirty_page(wa_ctx->obj, 0);
	batch = kmap_atomic(page);
	offset = 0;

	if (INTEL_INFO(engine->dev)->gen == 8) {
		ret = gen8_init_indirectctx_bb(engine,
					       &wa_ctx->indirect_ctx,
					       batch,
					       &offset);
		if (ret)
			goto out;

		ret = gen8_init_perctx_bb(engine,
					  &wa_ctx->per_ctx,
					  batch,
					  &offset);
		if (ret)
			goto out;
	} else if (INTEL_INFO(engine->dev)->gen == 9) {
		ret = gen9_init_indirectctx_bb(engine,
					       &wa_ctx->indirect_ctx,
					       batch,
					       &offset);
		if (ret)
			goto out;

		ret = gen9_init_perctx_bb(engine,
					  &wa_ctx->per_ctx,
					  batch,
					  &offset);
		if (ret)
			goto out;
	}

out:
	kunmap_atomic(batch);
	if (ret)
		lrc_destroy_wa_ctx_obj(engine);

	return ret;
}

static int gen8_init_common_ring(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u8 next_context_status_buffer_hw;

	lrc_setup_hardware_status_page(engine,
				engine->default_context->engine[engine->id].state);

	I915_WRITE_IMR(engine,
		       ~(engine->irq_enable_mask | engine->irq_keep_mask));
	I915_WRITE(RING_HWSTAM(engine->mmio_base), 0xffffffff);

	I915_WRITE(RING_MODE_GEN7(engine),
		   _MASKED_BIT_DISABLE(GFX_REPLAY_MODE) |
		   _MASKED_BIT_ENABLE(GFX_RUN_LIST_ENABLE));
	POSTING_READ(RING_MODE_GEN7(engine));

	/*
	 * Instead of resetting the Context Status Buffer (CSB) read pointer to
	 * zero, we need to read the write pointer from hardware and use its
	 * value because "this register is power context save restored".
	 * Effectively, these states have been observed:
	 *
	 *      | Suspend-to-idle (freeze) | Suspend-to-RAM (mem) |
	 * BDW  | CSB regs not reset       | CSB regs reset       |
	 * CHT  | CSB regs not reset       | CSB regs not reset   |
	 */
	next_context_status_buffer_hw = (I915_READ(RING_CONTEXT_STATUS_PTR(engine))
						   & GEN8_CSB_PTR_MASK);

	/*
	 * When the CSB registers are reset (also after power-up / gpu reset),
	 * CSB write pointer is set to all 1's, which is not valid, use '5' in
	 * this special case, so the first element read is CSB[0].
	 */
	if (next_context_status_buffer_hw == GEN8_CSB_PTR_MASK)
		next_context_status_buffer_hw = (GEN8_CSB_ENTRIES - 1);

	engine->next_context_status_buffer = next_context_status_buffer_hw;
	DRM_DEBUG_DRIVER("Execlists enabled for %s\n", engine->name);

	intel_engine_init_hangcheck(engine);

	return 0;
}

static int gen8_init_render_ring(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = gen8_init_common_ring(engine);
	if (ret)
		return ret;

	/* We need to disable the AsyncFlip performance optimisations in order
	 * to use MI_WAIT_FOR_EVENT within the CS. It should already be
	 * programmed to '1' on all products.
	 *
	 * WaDisableAsyncFlipPerfMode:snb,ivb,hsw,vlv,bdw,chv
	 */
	I915_WRITE(MI_MODE, _MASKED_BIT_ENABLE(ASYNC_FLIP_PERF_DISABLE));

	I915_WRITE(INSTPM, _MASKED_BIT_ENABLE(INSTPM_FORCE_ORDERING));

	return init_workarounds_ring(engine);
}

static int gen9_init_render_ring(struct intel_engine_cs *engine)
{
	int ret;

	ret = gen8_init_common_ring(engine);
	if (ret)
		return ret;

	return init_workarounds_ring(engine);
}

static int gen9_init_rcs_context_trtt(struct drm_i915_gem_request *req)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	int ret;

	ret = intel_logical_ring_begin(req, 2 + 2);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(1));

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_TABLE_CONTROL);
	intel_logical_ring_emit(ringbuf, 0);

	intel_logical_ring_emit(ringbuf, MI_NOOP);
	intel_logical_ring_advance(ringbuf);

	return 0;
}

static int gen9_emit_trtt_regs(struct drm_i915_gem_request *req)
{
	struct intel_context *ctx = req->ctx;
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	u64 masked_l3_gfx_address =
		ctx->trtt_info.l3_table_address & GEN9_TRTT_L3_GFXADDR_MASK;
	u32 trva_data_value =
		(ctx->trtt_info.segment_base_addr >> GEN9_TRTT_SEG_SIZE_SHIFT) &
		GEN9_TRVA_DATA_MASK;
	const int num_lri_cmds = 6;
	int ret;

	/*
	 * Emitting LRIs to update the TRTT registers is most reliable, instead
	 * of directly updating the context image, as this will ensure that
	 * update happens in a serialized manner for the context and also
	 * lite-restore scenario will get handled.
	 */
	ret = intel_logical_ring_begin(req, num_lri_cmds * 2 + 2);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(num_lri_cmds));

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_L3_POINTER_DW0);
	intel_logical_ring_emit(ringbuf, lower_32_bits(masked_l3_gfx_address));

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_L3_POINTER_DW1);
	intel_logical_ring_emit(ringbuf, upper_32_bits(masked_l3_gfx_address));

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_NULL_TILE_REG);
	intel_logical_ring_emit(ringbuf, ctx->trtt_info.null_tile_val);

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_INVD_TILE_REG);
	intel_logical_ring_emit(ringbuf, ctx->trtt_info.invd_tile_val);

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_VA_MASKDATA);
	intel_logical_ring_emit(ringbuf,
				GEN9_TRVA_MASK_VALUE | trva_data_value);

	intel_logical_ring_emit_reg(ringbuf, GEN9_TRTT_TABLE_CONTROL);
	intel_logical_ring_emit(ringbuf,
				GEN9_TRTT_IN_GFX_VA_SPACE | GEN9_TRTT_ENABLE);

	intel_logical_ring_emit(ringbuf, MI_NOOP);
	intel_logical_ring_advance(ringbuf);

	return 0;
}

static int intel_logical_ring_emit_pdps(struct drm_i915_gem_request *req)
{
	struct i915_hw_ppgtt *ppgtt = req->ctx->ppgtt;
	struct intel_engine_cs *engine = req->engine;
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	const int num_lri_cmds = GEN8_LEGACY_PDPES * 2;
	int i, ret;

	ret = intel_logical_ring_begin(req, num_lri_cmds * 2 + 2);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, MI_LOAD_REGISTER_IMM(num_lri_cmds));
	for (i = GEN8_LEGACY_PDPES - 1; i >= 0; i--) {
		const dma_addr_t pd_daddr = i915_page_dir_dma_addr(ppgtt, i);

		intel_logical_ring_emit_reg(ringbuf,
					    GEN8_RING_PDP_UDW(engine, i));
		intel_logical_ring_emit(ringbuf, upper_32_bits(pd_daddr));
		intel_logical_ring_emit_reg(ringbuf,
					    GEN8_RING_PDP_LDW(engine, i));
		intel_logical_ring_emit(ringbuf, lower_32_bits(pd_daddr));
	}

	intel_logical_ring_emit(ringbuf, MI_NOOP);
	intel_logical_ring_advance(ringbuf);

	return 0;
}

static int gen8_emit_bb_start(struct drm_i915_gem_request *req,
			      u64 offset, unsigned dispatch_flags)
{
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	bool ppgtt = !(dispatch_flags & I915_DISPATCH_SECURE);
	int ret;

	/* Don't rely in hw updating PDPs, specially in lite-restore.
	 * Ideally, we should set Force PD Restore in ctx descriptor,
	 * but we can't. Force Restore would be a second option, but
	 * it is unsafe in case of lite-restore (because the ctx is
	 * not idle). PML4 is allocated during ppgtt init so this is
	 * not needed in 48-bit.*/
	if (req->ctx->ppgtt &&
	    (intel_engine_flag(req->engine) & req->ctx->ppgtt->pd_dirty_rings)) {
		if (!USES_FULL_48BIT_PPGTT(req->i915) &&
		    !intel_vgpu_active(req->i915->dev)) {
			ret = intel_logical_ring_emit_pdps(req);
			if (ret)
				return ret;
		}

		req->ctx->ppgtt->pd_dirty_rings &= ~intel_engine_flag(req->engine);
	}

	ret = intel_logical_ring_begin(req, 4);
	if (ret)
		return ret;

	/* FIXME(BDW): Address space and security selectors. */
	intel_logical_ring_emit(ringbuf, MI_BATCH_BUFFER_START_GEN8 |
				(ppgtt<<8) |
				(dispatch_flags & I915_DISPATCH_RS ?
				 MI_BATCH_RESOURCE_STREAMER : 0));
	intel_logical_ring_emit(ringbuf, lower_32_bits(offset));
	intel_logical_ring_emit(ringbuf, upper_32_bits(offset));
	intel_logical_ring_emit(ringbuf, MI_NOOP);
	intel_logical_ring_advance(ringbuf);

	return 0;
}

static bool gen8_logical_ring_get_irq(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	if (WARN_ON(!intel_irqs_enabled(dev_priv)))
		return false;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (engine->irq_refcount++ == 0) {
		I915_WRITE_IMR(engine,
			       ~(engine->irq_enable_mask | engine->irq_keep_mask));
		POSTING_READ(RING_IMR(engine->mmio_base));
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return true;
}

static void gen8_logical_ring_put_irq(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (--engine->irq_refcount == 0) {
		I915_WRITE_IMR(engine, ~engine->irq_keep_mask);
		POSTING_READ(RING_IMR(engine->mmio_base));
	}
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static int gen8_emit_flush(struct drm_i915_gem_request *request,
			   u32 invalidate_domains,
			   u32 unused)
{
	uint32_t flags = 0;

	/*
	 * We always require a command barrier so that subsequent commands,
	 * such as breadcrumb interrupts, are strictly ordered w.r.t the
	 * contents of the write cache being flushed to memory (and thus
	 * being coherent from the CPU).
	 */

	if (invalidate_domains & I915_GEM_GPU_DOMAINS) {
		struct drm_i915_private *dev_priv = request->i915;

		if (request->engine == &dev_priv->engine[VCS])
			flags |= MI_INVALIDATE_BSD;
		flags |= MI_INVALIDATE_TLB;
	}

	/* Index must be QWord aligned */
	BUILD_BUG_ON(I915_GEM_HWS_SCRATCH_INDEX & 1);
	/* w/a: bit 5 needs to be zero for MI_FLUSH_DW QWord address. */
	BUILD_BUG_ON(I915_GEM_HWS_SCRATCH_INDEX & (1 << (5 - MI_STORE_DWORD_INDEX_SHIFT)));

	return gen8_emit_flush_qw_store_index(request, flags,
			I915_GEM_HWS_SCRATCH_INDEX, 0, 0);
}

static int gen8_emit_flush_render(struct drm_i915_gem_request *request,
				  u32 invalidate_domains,
				  u32 flush_domains)
{
	struct intel_ringbuffer *ringbuf = request->ringbuf;
	struct intel_engine_cs *engine = ringbuf->engine;
	u32 scratch_addr = engine->scratch.gtt_offset + 2 * CACHELINE_BYTES;
	bool vf_flush_wa;
	u32 flags = 0;
	int ret;

	flags |= PIPE_CONTROL_CS_STALL;

	if (flush_domains) {
		flags |= PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH;
		flags |= PIPE_CONTROL_DEPTH_CACHE_FLUSH;
		flags |= PIPE_CONTROL_L3_DC_FLUSH;
		flags |= PIPE_CONTROL_POSTSYNC_FLUSH;
	}

	if (invalidate_domains) {
		flags |= PIPE_CONTROL_TLB_INVALIDATE;
		flags |= PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_VF_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_CONST_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_STATE_CACHE_INVALIDATE;
		flags |= PIPE_CONTROL_QW_WRITE;
		flags |= PIPE_CONTROL_GEN7_GLOBAL_GTT;
	}

	/*
	 * On GEN9+ Before VF_CACHE_INVALIDATE we need to emit a NULL pipe
	 * control.
	 */
	vf_flush_wa = INTEL_INFO(engine->dev)->gen >= 9 &&
		      flags & PIPE_CONTROL_VF_CACHE_INVALIDATE;

	ret = intel_logical_ring_begin(request, vf_flush_wa ? 12 : 6);
	if (ret)
		return ret;

	if (vf_flush_wa) {
		intel_logical_ring_emit(ringbuf, GFX_OP_PIPE_CONTROL(6));
		intel_logical_ring_emit(ringbuf, (PIPE_CONTROL_CS_STALL |
						  PIPE_CONTROL_STALL_AT_SCOREBOARD));
		intel_logical_ring_emit(ringbuf, 0);
		intel_logical_ring_emit(ringbuf, 0);
		intel_logical_ring_emit(ringbuf, 0);
		intel_logical_ring_emit(ringbuf, 0);
	}

	intel_logical_ring_emit(ringbuf, GFX_OP_PIPE_CONTROL(6));
	intel_logical_ring_emit(ringbuf, flags);
	intel_logical_ring_emit(ringbuf, scratch_addr);
	intel_logical_ring_emit(ringbuf, 0);
	intel_logical_ring_emit(ringbuf, 0);
	intel_logical_ring_emit(ringbuf, 0);
	intel_logical_ring_advance(ringbuf);

	return 0;
}

static u32 gen8_get_seqno(struct intel_engine_cs *engine, bool lazy_coherency)
{
	return intel_read_status_page(engine, I915_GEM_HWS_INDEX);
}

static void gen8_set_seqno(struct intel_engine_cs *engine, u32 seqno)
{
	intel_write_status_page(engine, I915_GEM_HWS_INDEX, seqno);
}

static u32 bxt_a_get_seqno(struct intel_engine_cs *engine,
			   bool lazy_coherency)
{

	/*
	 * On BXT A steppings there is a HW coherency issue whereby the
	 * MI_STORE_DATA_IMM storing the completed request's seqno
	 * occasionally doesn't invalidate the CPU cache. Work around this by
	 * clflushing the corresponding cacheline whenever the caller wants
	 * the coherency to be guaranteed. Note that this cacheline is known
	 * to be clean at this point, since we only write it in
	 * bxt_a_set_seqno(), where we also do a clflush after the write. So
	 * this clflush in practice becomes an invalidate operation.
	 */

	if (!lazy_coherency)
		intel_flush_status_page(engine, I915_GEM_HWS_INDEX);

	return intel_read_status_page(engine, I915_GEM_HWS_INDEX);
}

static void bxt_a_set_seqno(struct intel_engine_cs *engine, u32 seqno)
{
	intel_write_status_page(engine, I915_GEM_HWS_INDEX, seqno);

	/* See bxt_a_get_seqno() explaining the reason for the clflush. */
	intel_flush_status_page(engine, I915_GEM_HWS_INDEX);
}

/*
 * Emit the commands that flag the end of execution of a batch.
 *
 * The GPU will:
 * 1) log the seqno of the request we're just completing.
 * 2) in the case of a preemptive batch, leave the in-progress sequence
 *    number set to the same value; otherwise, clear it. We use MI_FLUSH_DW
 *    to ensure the seqno write completes before the interrupt happens.
 * 3) Issue a USER INTERRUPT to notify the driver that the sequence number
 *    has been updated.
 */

static int gen8_emit_request(struct drm_i915_gem_request *request)
{
	struct intel_ringbuffer *ringbuf = request->ringbuf;
	uint32_t seqno = i915_gem_request_get_seqno(request);
	uint32_t index = I915_GEM_HWS_INDEX;
	uint32_t data2 = 0;
	int ret;

	/* Index must be QWord aligned */
	BUILD_BUG_ON(I915_BATCH_DONE_SEQNO & 1);
	BUILD_BUG_ON(I915_PREEMPTIVE_DONE_SEQNO & 1);

	/* w/a: bit 5 needs to be zero for MI_FLUSH_DW QWord address. */
	BUILD_BUG_ON(I915_BATCH_DONE_SEQNO & (1 << (5 - MI_STORE_DWORD_INDEX_SHIFT)));
	BUILD_BUG_ON(I915_PREEMPTIVE_DONE_SEQNO & (1 << (5 - MI_STORE_DWORD_INDEX_SHIFT)));

	WARN_ON(!seqno);

	if (request->scheduler_flags & I915_REQ_SF_PREEMPT) {
		index = I915_PREEMPTIVE_DONE_SEQNO;
		data2 = seqno;
	}

	ret = gen8_emit_flush_qw_store_index(request, 0, index, seqno, data2);
	if (ret)
		return ret;

	/*
	 * Reserve space for the instructions below, plus some NOOPs
	 * at the end of each request to be used as a workaround for
	 * not being allowed to do lite restore with HEAD==TAIL
	 * (WaIdleLiteRestore).
	 */
	ret = intel_logical_ring_begin(request, 2 + WA_TAIL_DWORDS);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, MI_USER_INTERRUPT);
	intel_logical_ring_emit(ringbuf, MI_NOOP);

	/* Always check for preemption after finishing the request */
	intel_logical_ring_emit(ringbuf, MI_ARB_ON_OFF | MI_ARB_ENABLE);
	intel_logical_ring_emit(ringbuf, MI_ARB_CHECK);

	return intel_logical_ring_advance_and_submit(request);
}

/*
 * Emit the commands that flag the end of execution of a batch.
 *
 * The GPU will:
 * 1) log the seqno of the request we're just completing.
 * 2) in the case of a preemptive batch, leave the in-progress sequence
 *    number set to the same value; otherwise, clear it. We use PIPE_CONTROL
 *    to ensure the seqno write completes before the interrupt happens.
 * 3) Issue a USER INTERRUPT to notify the driver that the sequence number
 *    has been updated.
 */
static int gen8_emit_request_render(struct drm_i915_gem_request *request)
{
	struct intel_ringbuffer *ringbuf = request->ringbuf;
	uint32_t seqno = i915_gem_request_get_seqno(request);
	uint32_t index = I915_GEM_HWS_INDEX;
	uint32_t data2 = 0;
	int ret;

	/* Index must be QWord aligned */
	BUILD_BUG_ON(I915_BATCH_DONE_SEQNO & 1);
	BUILD_BUG_ON(I915_PREEMPTIVE_DONE_SEQNO & 1);

	/* w/a: bit 5 needs to be zero for MI_FLUSH_DW QWord address. */
	BUILD_BUG_ON(I915_BATCH_DONE_SEQNO & (1 << (5 - MI_STORE_DWORD_INDEX_SHIFT)));
	BUILD_BUG_ON(I915_PREEMPTIVE_DONE_SEQNO & (1 << (5 - MI_STORE_DWORD_INDEX_SHIFT)));

	WARN_ON(!seqno);

	if (request->scheduler_flags & I915_REQ_SF_PREEMPT) {
		index = I915_PREEMPTIVE_DONE_SEQNO;
		data2 = seqno;
	}

	/*
	 * w/a for post sync ops following a GPGPU operation we
	 * need a prior CS_STALL, which is emitted by the flush
	 * following the batch.
	 */

	ret = gen8_emit_pipe_control_qw_store_index(request, 0, index, seqno, data2);
	if (ret)
		return ret;

	/*
	 * Reserve space for the instructions below, plus some NOOPs
	 * at the end of each request to be used as a workaround for
	 * not being allowed to do lite restore with HEAD==TAIL
	 * (WaIdleLiteRestore).
	 */
	ret = intel_logical_ring_begin(request, 2 + WA_TAIL_DWORDS);
	if (ret)
		return ret;

	intel_logical_ring_emit(ringbuf, MI_USER_INTERRUPT);
	intel_logical_ring_emit(ringbuf, MI_NOOP);

	/* Always check for preemption after finishing the request */
	intel_logical_ring_emit(ringbuf, MI_ARB_ON_OFF | MI_ARB_ENABLE);
	intel_logical_ring_emit(ringbuf, MI_ARB_CHECK);

	return intel_logical_ring_advance_and_submit(request);
}

static int intel_lr_context_render_state_init(struct drm_i915_gem_request *req)
{
	struct render_state so;
	int ret;

	ret = i915_gem_render_state_prepare(req->engine, &so);
	if (ret)
		return ret;

	if (so.rodata == NULL)
		return 0;

	ret = req->engine->emit_bb_start(req, so.ggtt_offset,
				       I915_DISPATCH_SECURE);
	if (ret)
		goto out;

	ret = req->engine->emit_bb_start(req,
				       (so.ggtt_offset + so.aux_batch_offset),
				       I915_DISPATCH_SECURE);
	if (ret)
		goto out;

	i915_vma_move_to_active(i915_gem_obj_to_ggtt(so.obj), req);

out:
	i915_gem_render_state_fini(&so);
	return ret;
}

static int gen8_init_rcs_context(struct drm_i915_gem_request *req)
{
	int ret;

	ret = intel_logical_ring_workarounds_emit(req);
	if (ret)
		return ret;

	ret = intel_rcs_context_init_mocs(req);
	/*
	 * Failing to program the MOCS is non-fatal.The system will not
	 * run at peak performance. So generate an error and carry on.
	 */
	if (ret)
		DRM_ERROR("MOCS failed to program: expect performance issues.\n");

	return intel_lr_context_render_state_init(req);
}

static int gen9_init_rcs_context(struct drm_i915_gem_request *req)
{
	int ret;

	/*
	 * Explictily disable TR-TT at the start of a new context.
	 * Otherwise on switching from a TR-TT context to a new Non TR-TT
	 * context the TR-TT settings of the outgoing context could get
	 * spilled on to the new incoming context as only the Ring Context
	 * part is loaded on the first submission of a new context, due to
	 * the setting of ENGINE_CTX_RESTORE_INHIBIT bit.
	 */
	ret = gen9_init_rcs_context_trtt(req);
	if (ret)
		return ret;

	return gen8_init_rcs_context(req);
}

/**
 * intel_logical_ring_cleanup() - deallocate the Engine Command Streamer
 *
 * @ring: Engine Command Streamer.
 *
 */
void intel_logical_ring_cleanup(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv;

	if (!intel_engine_initialized(engine))
		return;

	dev_priv = engine->dev->dev_private;

	if (engine->buffer) {
		intel_logical_ring_stop(engine);
		WARN_ON((I915_READ_MODE(engine) & MODE_IDLE) == 0);
	}

	if (engine->cleanup)
		engine->cleanup(engine);

	i915_cmd_parser_fini_ring(engine);
	i915_gem_batch_pool_fini(&engine->batch_pool);

	if (engine->status_page.obj) {
		kunmap(sg_page(engine->status_page.obj->pages->sgl));
		engine->status_page.obj = NULL;
	}

	lrc_destroy_wa_ctx_obj(engine);
	engine->dev = NULL;
}

static int logical_ring_init(struct drm_device *dev,
			     struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum forcewake_domains fw_domains;
	int ret;

	/* Intentionally left blank. */
	engine->buffer = NULL;

	engine->dev = dev;
	INIT_LIST_HEAD(&engine->active_list);
	INIT_LIST_HEAD(&engine->request_list);
	INIT_LIST_HEAD(&engine->fence_signal_list);
	INIT_LIST_HEAD(&engine->fence_unsignal_list);
	INIT_LIST_HEAD(&engine->delayed_free_list);
	spin_lock_init(&engine->fence_lock);
	spin_lock_init(&engine->delayed_free_lock);
	i915_gem_batch_pool_init(dev, &engine->batch_pool);
	init_waitqueue_head(&engine->irq_queue);

	INIT_LIST_HEAD(&engine->buffers);
	INIT_LIST_HEAD(&engine->execlist_queue);
	INIT_LIST_HEAD(&engine->execlist_retired_req_list);
	spin_lock_init(&engine->execlist_lock);

	fw_domains = intel_uncore_forcewake_for_reg(dev_priv,
						    RING_ELSP(engine),
						    FW_REG_WRITE);

	fw_domains |= intel_uncore_forcewake_for_reg(dev_priv,
						     RING_CONTEXT_STATUS_PTR(engine),
						     FW_REG_READ | FW_REG_WRITE);

	fw_domains |= intel_uncore_forcewake_for_reg(dev_priv,
						     RING_CONTEXT_STATUS_BUF_BASE(engine),
						     FW_REG_READ);

	engine->fw_domains = fw_domains;

	ret = i915_cmd_parser_init_ring(engine);
	if (ret)
		goto error;

	ret = intel_lr_context_deferred_alloc(engine->default_context, engine);
	if (ret)
		goto error;

	/* As this is the default context, always pin it */
	ret = intel_lr_context_do_pin(engine,
			engine->default_context->engine[engine->id].state,
			engine->default_context->engine[engine->id].ringbuf);
	if (ret) {
		DRM_ERROR(
			"Failed to pin and map ringbuffer %s: %d\n",
			engine->name, ret);
		goto error;
	}

	return 0;

error:
	intel_logical_ring_cleanup(engine);
	return ret;
}

static int logical_render_ring_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *engine = &dev_priv->engine[RCS];
	int ret;

	engine->name = "render ring";
	engine->id = RCS;
	engine->guc_id = GUC_RENDER_ENGINE;
	engine->mmio_base = RENDER_RING_BASE;
	engine->irq_enable_mask =
		GT_RENDER_USER_INTERRUPT << GEN8_RCS_IRQ_SHIFT;
	engine->irq_keep_mask =
		GT_CONTEXT_SWITCH_INTERRUPT << GEN8_RCS_IRQ_SHIFT;
	if (HAS_L3_DPF(dev))
		engine->irq_keep_mask |= GT_RENDER_L3_PARITY_ERROR_INTERRUPT;

	if (INTEL_INFO(dev)->gen >= 9) {
		engine->init_hw = gen9_init_render_ring;
		engine->init_context = gen9_init_rcs_context;
	} else {
		engine->init_hw = gen8_init_render_ring;
		engine->init_context = gen8_init_rcs_context;
	}

	engine->cleanup = intel_fini_pipe_control;
	if (IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		engine->get_seqno = bxt_a_get_seqno;
		engine->set_seqno = bxt_a_set_seqno;
	} else {
		engine->get_seqno = gen8_get_seqno;
		engine->set_seqno = gen8_set_seqno;
	}
	engine->emit_request = gen8_emit_request_render;
	engine->emit_flush = gen8_emit_flush_render;
	engine->irq_get = gen8_logical_ring_get_irq;
	engine->irq_put = gen8_logical_ring_put_irq;
	engine->emit_bb_start = gen8_emit_bb_start;

	engine->dev = dev;

	ret = intel_init_pipe_control(engine);
	if (ret)
		return ret;

       ret = intel_init_workaround_bb(engine);
	if (ret) {
		/*
		 * We continue even if we fail to initialize WA batch
		 * because we only expect rare glitches but nothing
		 * critical to prevent us from using GPU
		 */
		DRM_ERROR("WA batch buffer initialization failed: %d\n",
			  ret);
	}

	ret = logical_ring_init(dev, engine);
	if (ret) {
		lrc_destroy_wa_ctx_obj(engine);
	}

	return ret;
}

static int logical_bsd_ring_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *engine = &dev_priv->engine[VCS];

	engine->name = "bsd ring";
	engine->id = VCS;
	engine->guc_id = GUC_VIDEO_ENGINE;
	engine->mmio_base = GEN6_BSD_RING_BASE;
	engine->irq_enable_mask =
		GT_RENDER_USER_INTERRUPT << GEN8_VCS1_IRQ_SHIFT;
	engine->irq_keep_mask =
		GT_CONTEXT_SWITCH_INTERRUPT << GEN8_VCS1_IRQ_SHIFT;

	engine->init_hw = gen8_init_common_ring;
	if (IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		engine->get_seqno = bxt_a_get_seqno;
		engine->set_seqno = bxt_a_set_seqno;
	} else {
		engine->get_seqno = gen8_get_seqno;
		engine->set_seqno = gen8_set_seqno;
	}
	engine->emit_request = gen8_emit_request;
	engine->emit_flush = gen8_emit_flush;
	engine->irq_get = gen8_logical_ring_get_irq;
	engine->irq_put = gen8_logical_ring_put_irq;
	engine->emit_bb_start = gen8_emit_bb_start;

	return logical_ring_init(dev, engine);
}

static int logical_bsd2_ring_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *engine = &dev_priv->engine[VCS2];

	engine->name = "bds2 ring";
	engine->id = VCS2;
	engine->guc_id = GUC_VIDEO_ENGINE2;
	engine->mmio_base = GEN8_BSD2_RING_BASE;
	engine->irq_enable_mask =
		GT_RENDER_USER_INTERRUPT << GEN8_VCS2_IRQ_SHIFT;
	engine->irq_keep_mask =
		GT_CONTEXT_SWITCH_INTERRUPT << GEN8_VCS2_IRQ_SHIFT;

	engine->init_hw = gen8_init_common_ring;
	engine->get_seqno = gen8_get_seqno;
	engine->set_seqno = gen8_set_seqno;
	engine->emit_request = gen8_emit_request;
	engine->emit_flush = gen8_emit_flush;
	engine->irq_get = gen8_logical_ring_get_irq;
	engine->irq_put = gen8_logical_ring_put_irq;
	engine->emit_bb_start = gen8_emit_bb_start;

	return logical_ring_init(dev, engine);
}

static int logical_blt_ring_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *engine = &dev_priv->engine[BCS];

	engine->name = "blitter ring";
	engine->id = BCS;
	engine->guc_id = GUC_BLITTER_ENGINE;
	engine->mmio_base = BLT_RING_BASE;
	engine->irq_enable_mask =
		GT_RENDER_USER_INTERRUPT << GEN8_BCS_IRQ_SHIFT;
	engine->irq_keep_mask =
		GT_CONTEXT_SWITCH_INTERRUPT << GEN8_BCS_IRQ_SHIFT;

	engine->init_hw = gen8_init_common_ring;
	if (IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		engine->get_seqno = bxt_a_get_seqno;
		engine->set_seqno = bxt_a_set_seqno;
	} else {
		engine->get_seqno = gen8_get_seqno;
		engine->set_seqno = gen8_set_seqno;
	}
	engine->emit_request = gen8_emit_request;
	engine->emit_flush = gen8_emit_flush;
	engine->irq_get = gen8_logical_ring_get_irq;
	engine->irq_put = gen8_logical_ring_put_irq;
	engine->emit_bb_start = gen8_emit_bb_start;

	return logical_ring_init(dev, engine);
}

static int logical_vebox_ring_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *engine = &dev_priv->engine[VECS];

	engine->name = "video enhancement ring";
	engine->id = VECS;
	engine->guc_id = GUC_VIDEOENHANCE_ENGINE;
	engine->mmio_base = VEBOX_RING_BASE;
	engine->irq_enable_mask =
		GT_RENDER_USER_INTERRUPT << GEN8_VECS_IRQ_SHIFT;
	engine->irq_keep_mask =
		GT_CONTEXT_SWITCH_INTERRUPT << GEN8_VECS_IRQ_SHIFT;

	engine->init_hw = gen8_init_common_ring;
	if (IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		engine->get_seqno = bxt_a_get_seqno;
		engine->set_seqno = bxt_a_set_seqno;
	} else {
		engine->get_seqno = gen8_get_seqno;
		engine->set_seqno = gen8_set_seqno;
	}
	engine->emit_request = gen8_emit_request;
	engine->emit_flush = gen8_emit_flush;
	engine->irq_get = gen8_logical_ring_get_irq;
	engine->irq_put = gen8_logical_ring_put_irq;
	engine->emit_bb_start = gen8_emit_bb_start;

	return logical_ring_init(dev, engine);
}

/**
 * intel_logical_rings_init() - allocate, populate and init the Engine Command Streamers
 * @dev: DRM device.
 *
 * This function inits the engines for an Execlists submission style (the equivalent in the
 * legacy ringbuffer submission world would be i915_gem_init_engines). It does it only for
 * those engines that are present in the hardware.
 *
 * Return: non-zero if the initialization failed.
 */
int intel_logical_rings_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = logical_render_ring_init(dev);
	if (ret)
		return ret;

	if (HAS_BSD(dev)) {
		ret = logical_bsd_ring_init(dev);
		if (ret)
			goto cleanup_render_ring;
	}

	if (HAS_BLT(dev)) {
		ret = logical_blt_ring_init(dev);
		if (ret)
			goto cleanup_bsd_ring;
	}

	if (HAS_VEBOX(dev)) {
		ret = logical_vebox_ring_init(dev);
		if (ret)
			goto cleanup_blt_ring;
	}

	if (HAS_BSD2(dev)) {
		ret = logical_bsd2_ring_init(dev);
		if (ret)
			goto cleanup_vebox_ring;
	}

	return 0;

cleanup_vebox_ring:
	intel_logical_ring_cleanup(&dev_priv->engine[VECS]);
cleanup_blt_ring:
	intel_logical_ring_cleanup(&dev_priv->engine[BCS]);
cleanup_bsd_ring:
	intel_logical_ring_cleanup(&dev_priv->engine[VCS]);
cleanup_render_ring:
	intel_logical_ring_cleanup(&dev_priv->engine[RCS]);

	return ret;
}

static u32
make_rpcs(struct drm_device *dev)
{
	u32 rpcs = 0;

	/*
	 * No explicit RPCS request is needed to ensure full
	 * slice/subslice/EU enablement prior to Gen9.
	*/
	if (INTEL_INFO(dev)->gen < 9)
		return 0;

	/*
	 * Starting in Gen9, render power gating can leave
	 * slice/subslice/EU in a partially enabled state. We
	 * must make an explicit request through RPCS for full
	 * enablement.
	*/
	if (INTEL_INFO(dev)->has_slice_pg) {
		rpcs |= GEN8_RPCS_S_CNT_ENABLE;
		rpcs |= INTEL_INFO(dev)->slice_total <<
			GEN8_RPCS_S_CNT_SHIFT;
		rpcs |= GEN8_RPCS_ENABLE;
	}

	if (INTEL_INFO(dev)->has_subslice_pg) {
		rpcs |= GEN8_RPCS_SS_CNT_ENABLE;
		rpcs |= INTEL_INFO(dev)->subslice_per_slice <<
			GEN8_RPCS_SS_CNT_SHIFT;
		rpcs |= GEN8_RPCS_ENABLE;
	}

	if (INTEL_INFO(dev)->has_eu_pg) {
		rpcs |= INTEL_INFO(dev)->eu_per_subslice <<
			GEN8_RPCS_EU_MIN_SHIFT;
		rpcs |= INTEL_INFO(dev)->eu_per_subslice <<
			GEN8_RPCS_EU_MAX_SHIFT;
		rpcs |= GEN8_RPCS_ENABLE;
	}

	return rpcs;
}

static u32 intel_lr_indirect_ctx_offset(struct intel_engine_cs *engine)
{
	u32 indirect_ctx_offset;

	switch (INTEL_INFO(engine->dev)->gen) {
	default:
		MISSING_CASE(INTEL_INFO(engine->dev)->gen);
		/* fall through */
	case 9:
		indirect_ctx_offset =
			GEN9_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT;
		break;
	case 8:
		indirect_ctx_offset =
			GEN8_CTX_RCS_INDIRECT_CTX_OFFSET_DEFAULT;
		break;
	}

	return indirect_ctx_offset;
}

static int
populate_lr_context(struct intel_context *ctx, struct drm_i915_gem_object *ctx_obj,
		    struct intel_engine_cs *engine,
		    struct intel_ringbuffer *ringbuf)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_hw_ppgtt *ppgtt = ctx->ppgtt;
	struct page *page;
	uint32_t *reg_state;
	int ret;

	if (!ppgtt)
		ppgtt = dev_priv->mm.aliasing_ppgtt;

	ret = i915_gem_object_set_to_cpu_domain(ctx_obj, true);
	if (ret) {
		DRM_DEBUG_DRIVER("Could not set to CPU domain\n");
		return ret;
	}

	ret = i915_gem_object_get_pages(ctx_obj);
	if (ret) {
		DRM_DEBUG_DRIVER("Could not get object pages\n");
		return ret;
	}

	i915_gem_object_pin_pages(ctx_obj);

	/* The second page of the context object contains some fields which must
	 * be set up prior to the first execution. */
	page = i915_gem_object_get_dirty_page(ctx_obj, LRC_STATE_PN);
	reg_state = kmap_atomic(page);

	/* A context is actually a big batch buffer with several MI_LOAD_REGISTER_IMM
	 * commands followed by (reg, value) pairs. The values we are setting here are
	 * only for the first context restore: on a subsequent save, the GPU will
	 * recreate this batchbuffer with new values (including all the missing
	 * MI_LOAD_REGISTER_IMM commands that we are not initializing here). */
	reg_state[CTX_LRI_HEADER_0] =
		MI_LOAD_REGISTER_IMM(engine->id == RCS ? 14 : 11) | MI_LRI_FORCE_POSTED;
	ASSIGN_CTX_REG(reg_state, CTX_CONTEXT_CONTROL,
		       RING_CONTEXT_CONTROL(engine),
		       _MASKED_BIT_ENABLE(CTX_CTRL_INHIBIT_SYN_CTX_SWITCH |
					  CTX_CTRL_ENGINE_CTX_RESTORE_INHIBIT |
					  (HAS_RESOURCE_STREAMER(dev) ?
					    CTX_CTRL_RS_CTX_ENABLE : 0)));
	ASSIGN_CTX_REG(reg_state, CTX_RING_HEAD, RING_HEAD(engine->mmio_base),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_RING_TAIL, RING_TAIL(engine->mmio_base),
		       0);
	/* Ring buffer start address is not known until the buffer is pinned.
	 * It is written to the context image in execlists_update_context()
	 */
	ASSIGN_CTX_REG(reg_state, CTX_RING_BUFFER_START,
		       RING_START(engine->mmio_base), 0);
	ASSIGN_CTX_REG(reg_state, CTX_RING_BUFFER_CONTROL,
		       RING_CTL(engine->mmio_base),
		       ((ringbuf->size - PAGE_SIZE) & RING_NR_PAGES) | RING_VALID);
	ASSIGN_CTX_REG(reg_state, CTX_BB_HEAD_U,
		       RING_BBADDR_UDW(engine->mmio_base), 0);
	ASSIGN_CTX_REG(reg_state, CTX_BB_HEAD_L,
		       RING_BBADDR(engine->mmio_base), 0);
	ASSIGN_CTX_REG(reg_state, CTX_BB_STATE,
		       RING_BBSTATE(engine->mmio_base),
		       RING_BB_PPGTT);
	ASSIGN_CTX_REG(reg_state, CTX_SECOND_BB_HEAD_U,
		       RING_SBBADDR_UDW(engine->mmio_base), 0);
	ASSIGN_CTX_REG(reg_state, CTX_SECOND_BB_HEAD_L,
		       RING_SBBADDR(engine->mmio_base), 0);
	ASSIGN_CTX_REG(reg_state, CTX_SECOND_BB_STATE,
		       RING_SBBSTATE(engine->mmio_base), 0);
	if (engine->id == RCS) {
		ASSIGN_CTX_REG(reg_state, CTX_BB_PER_CTX_PTR,
			       RING_BB_PER_CTX_PTR(engine->mmio_base), 0);
		ASSIGN_CTX_REG(reg_state, CTX_RCS_INDIRECT_CTX,
			       RING_INDIRECT_CTX(engine->mmio_base), 0);
		ASSIGN_CTX_REG(reg_state, CTX_RCS_INDIRECT_CTX_OFFSET,
			       RING_INDIRECT_CTX_OFFSET(engine->mmio_base),
			       0);
		if (engine->wa_ctx.obj) {
			struct i915_ctx_workarounds *wa_ctx = &engine->wa_ctx;
			uint32_t ggtt_offset = i915_gem_obj_ggtt_offset(wa_ctx->obj);

			reg_state[CTX_RCS_INDIRECT_CTX+1] =
				(ggtt_offset + wa_ctx->indirect_ctx.offset * sizeof(uint32_t)) |
				(wa_ctx->indirect_ctx.size / CACHELINE_DWORDS);

			reg_state[CTX_RCS_INDIRECT_CTX_OFFSET+1] =
				intel_lr_indirect_ctx_offset(engine) << 6;

			reg_state[CTX_BB_PER_CTX_PTR+1] =
				(ggtt_offset + wa_ctx->per_ctx.offset * sizeof(uint32_t)) |
				0x01;
		}
	}
	reg_state[CTX_LRI_HEADER_1] = MI_LOAD_REGISTER_IMM(9) | MI_LRI_FORCE_POSTED;
	ASSIGN_CTX_REG(reg_state, CTX_CTX_TIMESTAMP,
		       RING_CTX_TIMESTAMP(engine->mmio_base), 0);
	/* PDP values well be assigned later if needed */
	ASSIGN_CTX_REG(reg_state, CTX_PDP3_UDW, GEN8_RING_PDP_UDW(engine, 3),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP3_LDW, GEN8_RING_PDP_LDW(engine, 3),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP2_UDW, GEN8_RING_PDP_UDW(engine, 2),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP2_LDW, GEN8_RING_PDP_LDW(engine, 2),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP1_UDW, GEN8_RING_PDP_UDW(engine, 1),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP1_LDW, GEN8_RING_PDP_LDW(engine, 1),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP0_UDW, GEN8_RING_PDP_UDW(engine, 0),
		       0);
	ASSIGN_CTX_REG(reg_state, CTX_PDP0_LDW, GEN8_RING_PDP_LDW(engine, 0),
		       0);

	if (USES_FULL_48BIT_PPGTT(ppgtt->base.dev)) {
		/* 64b PPGTT (48bit canonical)
		 * PDP0_DESCRIPTOR contains the base address to PML4 and
		 * other PDP Descriptors are ignored.
		 */
		ASSIGN_CTX_PML4(ppgtt, reg_state);
	} else {
		/* 32b PPGTT
		 * PDP*_DESCRIPTOR contains the base address of space supported.
		 * With dynamic page allocation, PDPs may not be allocated at
		 * this point. Point the unallocated PDPs to the scratch page
		 */
		ASSIGN_CTX_PDP(ppgtt, reg_state, 3);
		ASSIGN_CTX_PDP(ppgtt, reg_state, 2);
		ASSIGN_CTX_PDP(ppgtt, reg_state, 1);
		ASSIGN_CTX_PDP(ppgtt, reg_state, 0);
	}

	if (engine->id == RCS) {
		reg_state[CTX_LRI_HEADER_2] = MI_LOAD_REGISTER_IMM(1);
		ASSIGN_CTX_REG(reg_state, CTX_R_PWR_CLK_STATE, GEN8_R_PWR_CLK_STATE,
			       make_rpcs(dev));
	}

	i915_oa_update_reg_state(engine, reg_state);

	kunmap_atomic(reg_state);
	i915_gem_object_unpin_pages(ctx_obj);

	return 0;
}

/**
 * intel_lr_context_free() - free the LRC specific bits of a context
 * @ctx: the LR context to free.
 *
 * The real context freeing is done in i915_gem_context_free: this only
 * takes care of the bits that are LRC related: the per-engine backing
 * objects and the logical ringbuffer.
 */
void intel_lr_context_free(struct intel_context *ctx)
{
	int i;

	for (i = 0; i < I915_NUM_ENGINES; i++) {
		struct drm_i915_gem_object *ctx_obj = ctx->engine[i].state;

		if (ctx_obj) {
			struct intel_ringbuffer *ringbuf =
					ctx->engine[i].ringbuf;
			struct intel_engine_cs *ring = ringbuf->engine;

			if (ctx == ring->default_context) {
				intel_unpin_ringbuffer_obj(ringbuf);
				i915_gem_object_ggtt_unpin(ctx_obj);
			}
			WARN_ON(ctx->engine[ring->id].pin_count);
			intel_ringbuffer_free(ringbuf);
			drm_gem_object_unreference(&ctx_obj->base);
		}
	}
}

uint32_t intel_lr_context_size(struct intel_engine_cs *engine)
{
	int ret = 0;

	WARN_ON(INTEL_INFO(engine->dev)->gen < 8);

	switch (engine->id) {
	case RCS:
		if (INTEL_INFO(engine->dev)->gen >= 9)
			ret = GEN9_LR_CONTEXT_RENDER_SIZE;
		else
			ret = GEN8_LR_CONTEXT_RENDER_SIZE;
		break;
	case VCS:
	case BCS:
	case VECS:
	case VCS2:
		ret = GEN8_LR_CONTEXT_OTHER_SIZE;
		break;
	}

	return ret;
}

static void lrc_setup_hardware_status_page(struct intel_engine_cs *engine,
		struct drm_i915_gem_object *default_ctx_obj)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct page *page;

	/* The HWSP is part of the default context object in LRC mode. */
	engine->status_page.gfx_addr = i915_gem_obj_ggtt_offset(default_ctx_obj)
			+ LRC_PPHWSP_PN * PAGE_SIZE;
	page = i915_gem_object_get_page(default_ctx_obj, LRC_PPHWSP_PN);
	engine->status_page.page_addr = kmap(page);
	engine->status_page.obj = default_ctx_obj;

	I915_WRITE(RING_HWS_PGA(engine->mmio_base),
			(u32)engine->status_page.gfx_addr);
	POSTING_READ(RING_HWS_PGA(engine->mmio_base));
}

/**
 * intel_lr_context_deferred_alloc() - create the LRC specific bits of a context
 * @ctx: LR context to create.
 * @ring: engine to be used with the context.
 *
 * This function can be called more than once, with different engines, if we plan
 * to use the context with them. The context backing objects and the ringbuffers
 * (specially the ringbuffer backing objects) suck a lot of memory up, and that's why
 * the creation is a deferred call: it's better to make sure first that we need to use
 * a given ring with the context.
 *
 * Return: non-zero on error.
 */

int intel_lr_context_deferred_alloc(struct intel_context *ctx,
				     struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_gem_object *ctx_obj;
	uint32_t context_size;
	struct intel_ringbuffer *ringbuf;
	int ret;

	WARN_ON(ctx->legacy_hw_ctx.rcs_state != NULL);
	WARN_ON(ctx->engine[engine->id].state);

	/* Don't submit non-scheduler requests while the scheduler is busy */
	if (i915_scheduler_is_engine_busy(engine)) {
		mutex_unlock(&dev->struct_mutex);
		msleep(1);
		mutex_lock(&dev->struct_mutex);
		return -EAGAIN;
	}

	context_size = round_up(intel_lr_context_size(engine), 4096);

	/* One extra page as the sharing data between driver and GuC */
	context_size += PAGE_SIZE * LRC_PPHWSP_PN;

	ctx_obj = i915_gem_alloc_object(dev, context_size);
	if (!ctx_obj) {
		DRM_DEBUG_DRIVER("Alloc LRC backing obj failed.\n");
		return -ENOMEM;
	}

	ringbuf = intel_engine_create_ringbuffer(engine, 4 * PAGE_SIZE);
	if (IS_ERR(ringbuf)) {
		ret = PTR_ERR(ringbuf);
		goto error_deref_obj;
	}

	ret = populate_lr_context(ctx, ctx_obj, engine, ringbuf);
	if (ret) {
		DRM_DEBUG_DRIVER("Failed to populate LRC: %d\n", ret);
		goto error_ringbuf;
	}

	/* Create a per context timeline for fences */
	ret = i915_create_fence_timeline(dev, ctx, engine);
	if (ret) {
		DRM_ERROR("Fence timeline creation failed for engine %s, ctx %p\n",
			  engine->name, ctx);
		goto error_ringbuf;
	}

	ctx->engine[engine->id].ringbuf = ringbuf;
	ctx->engine[engine->id].state = ctx_obj;

	if (ctx != engine->default_context && engine->init_context) {
		struct drm_i915_gem_request *req;

		ret = i915_gem_request_alloc(engine,
			ctx, &req);
		if (ret) {
			DRM_ERROR("ring create req: %d\n",
				ret);
			goto error_ringbuf;
		}

		ret = engine->init_context(req);
		if (ret) {
			DRM_ERROR("ring init context: %d\n",
				ret);
			i915_gem_request_cancel(req);
			goto error_ringbuf;
		}
		i915_add_request_no_flush(req);

		/*
		 * GuC firmware will try to collapse its DPC work queue if the
		 * new one is for same context. So the following breadcrumb
		 * could be amended to this batch and submitted as one batch.
		 * Wait here to make sure the context state init is finished
		 * before any other submission to GuC.
		 */
		if (i915.enable_guc_submission)
			ret = i915_wait_request(req);
	}
	return 0;

error_ringbuf:
	intel_ringbuffer_free(ringbuf);
error_deref_obj:
	drm_gem_object_unreference(&ctx_obj->base);
	ctx->engine[engine->id].ringbuf = NULL;
	ctx->engine[engine->id].state = NULL;
	return ret;
}

/*
 * Update the ringbuffer associated with the specified request
 * so that only the section relating to that request is valid.
 * Then propagate the change to the associated context image.
 */
void intel_lr_context_resync_req(struct drm_i915_gem_request *req)
{
	enum intel_engine_id engine_id = req->engine->id;
	struct drm_i915_gem_object *ctx_obj;
	struct intel_ringbuffer *ringbuf;
	struct page *page;
	uint32_t *reg_state;

	ctx_obj = req->ctx->engine[engine_id].state;
	ringbuf = req->ringbuf;

	if (WARN_ON(!ringbuf || !ctx_obj))
		return;
	if (WARN_ON(i915_gem_object_get_pages(ctx_obj)))
		return;

	page = i915_gem_object_get_page(ctx_obj, LRC_STATE_PN);
	reg_state = kmap_atomic(page);

	DRM_DEBUG_DRIVER("Updating ringbuf head/tail, previously 0x%x/%x ...\n",
		ringbuf->head, ringbuf->tail);

	ringbuf->tail = req->tail;
	ringbuf->last_retired_head = req->head;
	intel_ring_update_space(ringbuf);

	DRM_DEBUG_DRIVER("Updated ringbuf, now 0x%x/%x space %d\n",
		ringbuf->head, ringbuf->tail, ringbuf->space);

	reg_state[CTX_RING_TAIL+1] = ringbuf->tail;

	kunmap_atomic(reg_state);
}

/*
 * Empty the ringbuffer associated with the specified request
 * by updating the ringbuffer 'head' to the value of 'tail', or,
 * if 'rezero' is true, setting both 'head' and 'tail' to zero.
 * Then propagate the change to the associated context image.
 */
void intel_lr_context_resync(struct intel_context *ctx,
			     struct intel_engine_cs *engine,
			     bool rezero)
{
	enum intel_engine_id engine_id = engine->id;
	struct drm_i915_gem_object *ctx_obj;
	struct intel_ringbuffer *ringbuf;
	struct page *page;
	uint32_t *reg_state;

	ctx_obj = ctx->engine[engine_id].state;
	ringbuf = ctx->engine[engine_id].ringbuf;

	/*
	 * When resetting, a hardware context might be as-yet-unused
	 * and therefore not-yet-allocated. In other situations, the
	 * ringbuffer and context object must already exist.
	 */
	if (WARN_ON(!ringbuf != !ctx_obj))
		return;
	if (!i915_reset_in_progress(&ctx->i915->gpu_error))
		WARN_ON(!ringbuf || !ctx_obj);
	if (!ringbuf || !ctx_obj)
		return;
	if (WARN_ON(i915_gem_object_get_pages(ctx_obj)))
		return;

	if (i915_gem_object_get_pages(ctx_obj)) {
		WARN(1, "Failed get_pages for context obj\n");
		return;
	}
	page = i915_gem_object_get_dirty_page(ctx_obj, LRC_STATE_PN);
	reg_state = kmap_atomic(page);

	if (rezero)
		ringbuf->tail = 0;
	ringbuf->head = ringbuf->tail;
	ringbuf->last_retired_head = -1;
	intel_ring_update_space(ringbuf);

	reg_state[CTX_RING_HEAD+1] = ringbuf->head;
	reg_state[CTX_RING_TAIL+1] = ringbuf->tail;

	kunmap_atomic(reg_state);
}

void intel_lr_context_reset(struct intel_context *ctx)
{
	struct drm_i915_private *dev_priv = ctx->i915;
	struct intel_engine_cs *engine;
	int i;

	for_each_engine(engine, dev_priv, i) {
		intel_lr_context_resync(ctx, engine, true);
	}
}

int intel_lr_rcs_context_setup_trtt(struct intel_context *ctx)
{
	struct intel_engine_cs *engine = &(ctx->i915->engine[RCS]);
	struct drm_i915_gem_request *req;
	int ret;

	if (!ctx->engine[RCS].state) {
		ret = intel_lr_context_deferred_alloc(ctx, engine);
		if (ret)
			return ret;
	}

	ret = i915_gem_request_alloc(engine, ctx, &req);
	if (ret)
		return ret;

	ret = gen9_emit_trtt_regs(req);
	if (ret) {
		i915_gem_request_cancel(req);
		return ret;
	}

	i915_add_request(req);
	return 0;
}
