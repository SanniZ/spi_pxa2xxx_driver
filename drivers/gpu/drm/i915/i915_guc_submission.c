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
 */
#include <linux/firmware.h>
#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/relay.h>
#include "i915_drv.h"
#include "intel_guc.h"
#include "intel_huc.h"

/**
 * DOC: GuC-based command submission
 *
 * i915_guc_client:
 * We use the term client to avoid confusion with contexts. A i915_guc_client is
 * equivalent to GuC object guc_context_desc. This context descriptor is
 * allocated from a pool of 1024 entries. Kernel driver will allocate doorbell
 * and workqueue for it. Also the process descriptor (guc_process_desc), which
 * is mapped to client space. So the client can write Work Item then ring the
 * doorbell.
 *
 * To simplify the implementation, we allocate one gem object that contains all
 * pages for doorbell, process descriptor and workqueue.
 *
 * The Scratch registers:
 * There are 16 MMIO-based registers start from 0xC180. The kernel driver writes
 * a value to the action register (SOFT_SCRATCH_0) along with any data. It then
 * triggers an interrupt on the GuC via another register write (0xC4C8).
 * Firmware writes a success/fail code back to the action register after
 * processes the request. The kernel driver polls waiting for this update and
 * then proceeds.
 * See host2guc_action()
 *
 * Doorbells:
 * Doorbells are interrupts to uKernel. A doorbell is a single cache line (QW)
 * mapped into process space.
 *
 * Work Items:
 * There are several types of work items that the host may place into a
 * workqueue, each with its own requirements and limitations. Currently only
 * WQ_TYPE_INORDER is needed to support legacy submission via GuC, which
 * represents in-order queue. The kernel driver packs ring tail pointer and an
 * ELSP context descriptor dword into Work Item.
 * See guc_add_workqueue_item()
 *
 */

/*
 * Read GuC command/status register (SOFT_SCRATCH_0)
 * Return true if it contains a response rather than a command
 */
static inline bool host2guc_action_response(struct drm_i915_private *dev_priv,
					    u32 *status)
{
	u32 val = I915_READ(SOFT_SCRATCH(0));
	*status = val;
	return GUC2HOST_IS_RESPONSE(val);
}

static int host2guc_action(struct intel_guc *guc, u32 *data, u32 len)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	u32 status, response;
	int ret, i;

	if (WARN_ON(len < 1 || len > 15))
		return -EINVAL;

	mutex_lock(&guc->action_lock);
	intel_uncore_forcewake_get(dev_priv, FORCEWAKE_ALL);

	dev_priv->guc.action_count += 1;
	dev_priv->guc.action_cmd = data[0];

	for (i = 0; i < len; i++)
		I915_WRITE(SOFT_SCRATCH(i), data[i]);

	I915_WRITE(SOFT_SCRATCH(15), 0);
	POSTING_READ(SOFT_SCRATCH(0));

	I915_WRITE(HOST2GUC_INTERRUPT, HOST2GUC_TRIGGER);

	/*
	 * Fast commands should complete in less than 10us, so sample quickly
	 * up to that length of time, then switch to a slower sleep-wait loop.
	 * No HOST2GUC command should ever take longer than 10ms.
	 */
	ret = wait_for_us(host2guc_action_response(dev_priv, &status), 10);
	if (ret)
		ret = wait_for(host2guc_action_response(dev_priv, &status), 10);
	response = I915_READ(SOFT_SCRATCH(15));
	dev_priv->guc.action_status = status;
	if (status != GUC2HOST_STATUS_SUCCESS) {
		/*
		 * Either the GuC explicitly returned an error (which
		 * we convert to -EIO here) or no response at all was
		 * received within the timeout limit (-ETIMEDOUT)
		 */
		if (ret != -ETIMEDOUT)
			ret = -EIO;

		DRM_ERROR("GuC: host2guc action 0x%X failed. ret=%d "
				"status=0x%08X response=0x%08X\n",
				data[0], ret, status, response);

		dev_priv->guc.action_fail_count += 1;
		dev_priv->guc.action_fail_cmd = data[0];
		dev_priv->guc.action_fail_status = status;
		dev_priv->guc.action_err = ret;
	}

	intel_uncore_forcewake_put(dev_priv, FORCEWAKE_ALL);
	mutex_unlock(&guc->action_lock);

	return ret;
}

/*
 * Tell the GuC to submit a request pre-emptively
 */
static int
host2guc_preempt(struct i915_guc_client *client,
		 struct intel_context *ctx,
		 struct intel_engine_cs *ring)
{
	struct drm_i915_private *dev_priv = to_i915(ring->dev);
	struct intel_guc *guc = &dev_priv->guc;
	uint32_t engine_id = ring->id;
	struct drm_i915_gem_object *ctx_obj = ctx->engine[engine_id].state;
	struct intel_ringbuffer *ringbuf = ctx->engine[engine_id].ringbuf;
	struct guc_process_desc *desc;
	u32 data[8];
	int ret;

	if (WARN_ON(!ctx_obj || !ringbuf))
		return -EINVAL;

	WARN_ON(!i915_gem_obj_is_pinned(ctx_obj));
	WARN_ON(!i915_gem_obj_is_pinned(ringbuf->obj));

	WARN_ON(guc->preempt_client != client);

	desc = client->client_base + client->proc_desc_offset;

	/* Update the tail so it is visible to GuC */
	desc->tail = client->wq_tail;

	data[0] = HOST2GUC_ACTION_REQUEST_PREEMPTION;
	data[1] = guc->preempt_client->ctx_index;		/* preemptive client			*/
	data[2] = /* PREEMPT_ENGINE_OPTIONS */
		  HOST2GUC_PREEMPT_OPTION_IMMEDIATE |		/* submit before return			*/
		  HOST2GUC_PREEMPT_OPTION_DROP_WORK_Q |		/* drop wq for client data[5]		*/
		  HOST2GUC_PREEMPT_OPTION_DROP_SUBMIT_Q;	/* drop submitted (engine, priority)	*/
	data[3] = ring->guc_id;					/* target engine			*/
	data[4] = guc->execbuf_client->priority;		/* victim priority			*/
	data[5] = guc->execbuf_client->ctx_index;		/* victim ctx/wq			*/
	data[6] = i915_gem_obj_ggtt_offset(ctx_obj) + LRC_GUCSHR_PN*PAGE_SIZE;
	data[7] = 0;

	ret = host2guc_action(guc, data, 7);
	if (WARN_ON(ret)) {
		u32 *gsp;
		int i;

		gsp = kmap_atomic(i915_gem_object_get_page(ctx_obj, LRC_GUCSHR_PN));
		DRM_DEBUG_DRIVER("GuC preemption request data:\n");
		for (i = 0; i < 8; i += 4)
			DRM_DEBUG_DRIVER("\t%08x %08x  %08x %08x\n",
				data[i+0], data[i+1], data[i+2], data[i+3]);

		DRM_DEBUG_DRIVER("GuC per-context shared data @ 0x%llx:\n",
			i915_gem_obj_ggtt_offset(ctx_obj) + LRC_GUCSHR_PN*PAGE_SIZE);
		for (i = 0; i < 32; i += 4)
			DRM_DEBUG_DRIVER("\t%08x %08x  %08x %08x\n",
				gsp[i+0], gsp[i+1], gsp[i+2], gsp[i+3]);

		kunmap_atomic(gsp);
	}
	return ret;
}

/*
 * Tell the GuC to allocate or deallocate a specific doorbell
 */

static int host2guc_allocate_doorbell(struct intel_guc *guc,
				      struct i915_guc_client *client)
{
	u32 data[2];

	data[0] = HOST2GUC_ACTION_ALLOCATE_DOORBELL;
	data[1] = client->ctx_index;

	return host2guc_action(guc, data, 2);
}

static int host2guc_release_doorbell(struct intel_guc *guc,
				     struct i915_guc_client *client)
{
	u32 data[2];

	data[0] = HOST2GUC_ACTION_DEALLOCATE_DOORBELL;
	data[1] = client->ctx_index;

	return host2guc_action(guc, data, 2);
}

int host2guc_sample_forcewake(struct intel_guc *guc,
				     bool enable)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct drm_device *dev = dev_priv->dev;
	u32 data[2];

	data[0] = HOST2GUC_ACTION_SAMPLE_FORCEWAKE;

	/* WaRsDisableCoarsePowerGating:skl,bxt */
	if (!enable ||
	    NEEDS_WaRsDisableCoarsePowerGating(dev))
		data[1] = 0;
	else
		/* bit 0 and 1 are for Render and Media domain separately */
		data[1] = GUC_FORCEWAKE_RENDER | GUC_FORCEWAKE_MEDIA;

	return host2guc_action(guc, data, ARRAY_SIZE(data));
}

static int host2guc_logbuffer_flush_complete(struct intel_guc *guc)
{
	u32 data[1];

	data[0] = HOST2GUC_ACTION_LOG_BUFFER_FILE_FLUSH_COMPLETE;

	return host2guc_action(guc, data, 1);
}

static int host2guc_force_logbuffer_flush(struct intel_guc *guc)
{
	u32 data[2];

	data[0] = HOST2GUC_ACTION_FORCE_LOG_BUFFER_FLUSH;
	data[1] = 0;

	return host2guc_action(guc, data, 2);
}

static int host2guc_logging_control(struct intel_guc *guc, u32 control_val)
{
	u32 data[2];

	data[0] = HOST2GUC_ACTION_UK_LOG_ENABLE_LOGGING;
	data[1] = control_val;

	return host2guc_action(guc, data, 2);
}

/*
 * Initialise, update, or clear doorbell data shared with the GuC
 *
 * These functions modify shared data and so need access to the mapped
 * client object which contains the page being used for the doorbell
 */

static void guc_init_doorbell(struct intel_guc *guc,
			      struct i915_guc_client *client)
{
	struct guc_doorbell_info *doorbell;

	doorbell = client->client_base + client->doorbell_offset;

	doorbell->db_status = GUC_DOORBELL_ENABLED;
	doorbell->cookie = 0;
}

static int guc_ring_doorbell(struct i915_guc_client *gc)
{
	struct guc_process_desc *desc;
	union guc_doorbell_qw db_cmp, db_exc, db_ret;
	union guc_doorbell_qw *db;
	int attempt = 2, ret = -EAGAIN;

	desc = gc->client_base + gc->proc_desc_offset;

	/* Update the tail so it is visible to GuC */
	desc->tail = gc->wq_tail;

	/* current cookie */
	db_cmp.db_status = GUC_DOORBELL_ENABLED;
	db_cmp.cookie = gc->doorbell_cookie;

	/* cookie to be updated */
	db_exc.db_status = GUC_DOORBELL_ENABLED;
	db_exc.cookie = gc->doorbell_cookie + 1;
	if (db_exc.cookie == 0)
		db_exc.cookie = 1;

	/* pointer of current doorbell cacheline */
	db = gc->client_base + gc->doorbell_offset;

	while (attempt--) {
		/* lets ring the doorbell */
		db_ret.value_qw = atomic64_cmpxchg((atomic64_t *)db,
			db_cmp.value_qw, db_exc.value_qw);

		/* if the exchange was successfully executed */
		if (db_ret.value_qw == db_cmp.value_qw) {
			/* db was successfully rung */
			gc->doorbell_cookie = db_exc.cookie;
			ret = 0;
			break;
		}

		/* XXX: doorbell was lost and need to acquire it again */
		if (db_ret.db_status == GUC_DOORBELL_DISABLED)
			break;

		DRM_ERROR("Cookie mismatch. Expected %d, returned %d\n",
			  db_cmp.cookie, db_ret.cookie);

		/* update the cookie to newly read cookie from GuC */
		db_cmp.cookie = db_ret.cookie;
		db_exc.cookie = db_ret.cookie + 1;
		if (db_exc.cookie == 0)
			db_exc.cookie = 1;
	}

	return ret;
}

static void guc_disable_doorbell(struct intel_guc *guc,
				 struct i915_guc_client *client)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct guc_doorbell_info *doorbell;
	i915_reg_t drbreg = GEN8_DRBREGL(client->doorbell_id);
	int value;

	doorbell = client->client_base + client->doorbell_offset;

	doorbell->db_status = GUC_DOORBELL_DISABLED;

	I915_WRITE(drbreg, I915_READ(drbreg) & ~GEN8_DRB_VALID);

	value = I915_READ(drbreg);
	WARN_ON((value & GEN8_DRB_VALID) != 0);

	I915_WRITE(GEN8_DRBREGU(client->doorbell_id), 0);
	I915_WRITE(drbreg, 0);

	/* XXX: wait for any interrupts */
	/* XXX: wait for workqueue to drain */
}

/*
 * Select, assign and relase doorbell cachelines
 *
 * These functions track which doorbell cachelines are in use.
 * The data they manipulate is protected by the host2guc lock.
 */

static uint32_t select_doorbell_cacheline(struct intel_guc *guc)
{
	const uint32_t cacheline_size = cache_line_size();
	uint32_t offset;

	/* Doorbell uses a single cache line within a page */
	offset = offset_in_page(guc->db_cacheline);

	/* Moving to next cache line to reduce contention */
	guc->db_cacheline += cacheline_size;

	DRM_DEBUG_DRIVER("selected doorbell cacheline 0x%x, next 0x%x, linesize %u\n",
			offset, guc->db_cacheline, cacheline_size);

	return offset;
}

static uint16_t assign_doorbell(struct intel_guc *guc, uint32_t priority)
{
	/*
	 * The bitmap is split into two halves; the first half is used for
	 * normal priority contexts, the second half for high-priority ones.
	 * Note that logically higher priorities are numerically less than
	 * normal ones, so the test below means "is it high-priority?"
	 */
	const bool hi_pri = (priority <= GUC_CTX_PRIORITY_HIGH);
	const uint16_t half = GUC_MAX_DOORBELLS / 2;
	const uint16_t start = hi_pri ? half : 0;
	const uint16_t end = start + half;
	uint16_t id;

	id = find_next_zero_bit(guc->doorbell_bitmap, end, start);
	if (id == end)
		id = GUC_INVALID_DOORBELL_ID;
	else
		bitmap_set(guc->doorbell_bitmap, id, 1);

	DRM_DEBUG_DRIVER("assigned %s priority doorbell id 0x%x\n",
			hi_pri ? "high" : "normal", id);

	return id;
}

static void release_doorbell(struct intel_guc *guc, uint16_t id)
{
	bitmap_clear(guc->doorbell_bitmap, id, 1);
}

/*
 * Initialise the process descriptor shared with the GuC firmware.
 */
static void guc_init_proc_desc(struct intel_guc *guc,
			       struct i915_guc_client *client)
{
	struct guc_process_desc *desc;

	desc = client->client_base + client->proc_desc_offset;

	memset(desc, 0, sizeof(*desc));

	/*
	 * XXX: pDoorbell and WQVBaseAddress are pointers in process address
	 * space for ring3 clients (set them as in mmap_ioctl) or kernel
	 * space for kernel clients (map on demand instead? May make debug
	 * easier to have it mapped).
	 */
	desc->wq_base_addr = 0;
	desc->db_base_addr = 0;

	desc->context_id = client->ctx_index;
	desc->wq_size_bytes = client->wq_size;
	desc->wq_status = WQ_STATUS_ACTIVE;
	desc->priority = client->priority;
}

/*
 * Initialise/clear the context descriptor shared with the GuC firmware.
 *
 * This descriptor tells the GuC where (in GGTT space) to find the important
 * data structures relating to this client (doorbell, process descriptor,
 * write queue, etc).
 */

static void guc_init_ctx_desc(struct intel_guc *guc,
			      struct i915_guc_client *client)
{
	struct drm_i915_gem_object *client_obj = client->client_obj;
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct intel_engine_cs *engine;
	struct intel_context *ctx = client->owner;
	struct guc_context_desc desc;
	struct sg_table *sg;
	int i;
	u32 gfx_addr;

	memset(&desc, 0, sizeof(desc));

	desc.attribute = GUC_CTX_DESC_ATTR_ACTIVE | GUC_CTX_DESC_ATTR_KERNEL;
	if (client->priority <= GUC_CTX_PRIORITY_HIGH)
		desc.attribute |= GUC_CTX_DESC_ATTR_PREEMPT;
	desc.context_id = client->ctx_index;
	desc.priority = client->priority;
	desc.db_id = client->doorbell_id;

	for_each_engine(engine, dev_priv, i) {
		struct guc_execlist_context *lrc = &desc.lrc[engine->guc_id];
		struct drm_i915_gem_object *obj;
		uint64_t ctx_desc;

		/* TODO: We have a design issue to be solved here. Only when we
		 * receive the first batch, we know which engine is used by the
		 * user. But here GuC expects the lrc and ring to be pinned. It
		 * is not an issue for default context, which is the only one
		 * for now who owns a GuC client. But for future owner of GuC
		 * client, need to make sure lrc is pinned prior to enter here.
		 */
		obj = ctx->engine[i].state;
		if (!obj)
			continue;

		ctx_desc = intel_lr_context_descriptor(ctx, engine);
		lrc->context_desc = (u32)ctx_desc;

		/* The state page is after PPHWSP */
		gfx_addr = i915_gem_obj_ggtt_offset(obj);
		lrc->ring_lcra = gfx_addr + LRC_STATE_PN * PAGE_SIZE;
		lrc->context_id = (client->ctx_index << GUC_ELC_CTXID_OFFSET) |
				(engine->guc_id << GUC_ELC_ENGINE_OFFSET);

		obj = ctx->engine[i].ringbuf->obj;
		gfx_addr = i915_gem_obj_ggtt_offset(obj);

		lrc->ring_begin = gfx_addr;
		lrc->ring_end = gfx_addr + obj->base.size - 1;
		lrc->ring_next_free_location = gfx_addr;
		lrc->ring_current_tail_pointer_value = 0;

		desc.engines_used |= (1 << engine->guc_id);
	}

	WARN_ON(desc.engines_used == 0);

	/*
	 * The doorbell, process descriptor, and workqueue are all parts
	 * of the client object, which the GuC will reference via the GGTT
	 */
	desc.db_trigger_phy = sg_dma_address(client_obj->pages->sgl) +
				client->doorbell_offset;
	desc.db_trigger_cpu = (uintptr_t)client->client_base +
				client->doorbell_offset;
	desc.db_trigger_uk = client->client_gtt + client->doorbell_offset;
	desc.process_desc = client->client_gtt + client->proc_desc_offset;
	desc.wq_addr = client->client_gtt + client->wq_offset;
	desc.wq_size = client->wq_size;

	/*
	 * XXX: Take LRCs from an existing intel_context if this is not an
	 * IsKMDCreatedContext client
	 */
	desc.desc_private = (uintptr_t)client;

	/* Pool context is pinned already */
	sg = guc->ctx_pool_obj->pages;
	sg_pcopy_from_buffer(sg->sgl, sg->nents, &desc, sizeof(desc),
			     sizeof(desc) * client->ctx_index);
}

static void guc_fini_ctx_desc(struct intel_guc *guc,
			      struct i915_guc_client *client)
{
	struct guc_context_desc desc;
	struct sg_table *sg;

	memset(&desc, 0, sizeof(desc));

	sg = guc->ctx_pool_obj->pages;
	sg_pcopy_from_buffer(sg->sgl, sg->nents, &desc, sizeof(desc),
			     sizeof(desc) * client->ctx_index);
}

int i915_guc_wq_check_space(struct i915_guc_client *gc)
{
	const size_t wqi_size = sizeof(struct guc_wq_item);
	struct guc_process_desc *desc;
	u32 freespace;

	if (WARN_ON(gc == NULL))
		return 0;

	desc = gc->client_base + gc->proc_desc_offset;

	freespace = CIRC_SPACE(gc->wq_tail, desc->head, gc->wq_size);
	if (likely(freespace >= wqi_size))
		return 0;

	gc->no_wq_space += 1;

	return -EAGAIN;
}

static void guc_add_workqueue_item(struct i915_guc_client *gc,
				   struct drm_i915_gem_request *rq)
{
	/* wqi_len is in DWords, and does not include the one-word header */
	const size_t wqi_size = sizeof(struct guc_wq_item);
	const u32 wqi_len = wqi_size/sizeof(u32) - 1;
	struct guc_process_desc *desc;
	struct guc_wq_item *wqi;
	void *base;
	u32 freespace, tail, wq_off, wq_page;

	desc = gc->client_base + gc->proc_desc_offset;

	/* Free space is guaranteed, see i915_guc_wq_check_space() above */
	freespace = CIRC_SPACE(gc->wq_tail, desc->head, gc->wq_size);
	WARN_ON(freespace < wqi_size);

	/* The GuC firmware wants the tail index in QWords, not bytes */
	tail = rq->tail;
	WARN_ON(tail & 7);
	tail >>= 3;
	WARN_ON(tail > WQ_RING_TAIL_MAX);

	/* For now workqueue item is 4 DWs; workqueue buffer is 2 pages. So we
	 * should not have the case where structure wqi is across page, neither
	 * wrapped to the beginning. This simplifies the implementation below.
	 *
	 * XXX: if not the case, we need save data to a temp wqi and copy it to
	 * workqueue buffer dw by dw.
	 */
	WARN_ON(wqi_size != 16);

	/* postincrement WQ tail for next time */
	wq_off = gc->wq_tail;
	gc->wq_tail += wqi_size;
	gc->wq_tail &= gc->wq_size - 1;
	WARN_ON(wq_off & (wqi_size - 1));

	/* WQ starts from the page after doorbell / process_desc */
	wq_page = (wq_off + GUC_DB_SIZE) >> PAGE_SHIFT;
	wq_off &= PAGE_SIZE - 1;
	base = kmap_atomic(i915_gem_object_get_page(gc->client_obj, wq_page));
	wqi = (struct guc_wq_item *)((char *)base + wq_off);

	/* Now fill in the 4-word work queue item */
	wqi->header = WQ_TYPE_INORDER |
			(wqi_len << WQ_LEN_SHIFT) |
			(rq->engine->guc_id << WQ_TARGET_SHIFT) |
			WQ_NO_WCFLUSH_WAIT;

	/* The GuC wants only the low-order word of the context descriptor */
	wqi->context_desc = (u32)intel_lr_context_descriptor(rq->ctx,
						             rq->engine);

	wqi->ring_tail = tail << WQ_RING_TAIL_SHIFT;
	wqi->fence_id = rq->seqno;

	kunmap_atomic(base);
}

#define CTX_RING_BUFFER_START		0x08

/* Update the ringbuffer pointer in a saved context image */
static void lr_context_update(struct drm_i915_gem_request *rq)
{
	enum intel_engine_id ring_id = rq->engine->id;
	struct drm_i915_gem_object *ctx_obj = rq->ctx->engine[ring_id].state;
	struct drm_i915_gem_object *rb_obj = rq->ringbuf->obj;
	struct page *page;
	uint32_t *reg_state;

	BUG_ON(!ctx_obj);
	WARN_ON(!i915_gem_obj_is_pinned(ctx_obj));
	WARN_ON(!i915_gem_obj_is_pinned(rb_obj));

	page = i915_gem_object_get_dirty_page(ctx_obj, LRC_STATE_PN);
	reg_state = kmap_atomic(page);

	reg_state[CTX_RING_BUFFER_START+1] = i915_gem_obj_ggtt_offset(rb_obj);

	kunmap_atomic(reg_state);
}

/**
 * i915_guc_submit() - Submit commands through GuC
 * @client:	the guc client where commands will go through
 * @rq:		request associated with the commands
 *
 * Return:	0 if succeed
 */
int i915_guc_submit(struct i915_guc_client *client,
		    struct drm_i915_gem_request *rq)
{
	bool preemptive = client->priority <= GUC_CTX_PRIORITY_HIGH;
	unsigned int engine_id = rq->engine->id;
	unsigned int guc_id = rq->engine->guc_id;
	struct intel_guc *guc = client->guc;
	int b_ret;

	if (engine_id == VCS)
		udelay(500);

	if (WARN_ON(engine_id >= I915_NUM_ENGINES))
		return -ENXIO;
	if (WARN_ON(guc_id >= GUC_MAX_ENGINES_NUM))
		return -ENXIO;

	/* Need this because of the deferred pin ctx and ring */
	/* Shall we move this right after ring is pinned? */
	lr_context_update(rq);

	guc_add_workqueue_item(client, rq);
	if (preemptive)
		b_ret = host2guc_preempt(client, rq->ctx, rq->engine);
	else
		b_ret = guc_ring_doorbell(client);

	client->submissions[guc_id] += 1;
	client->retcode = b_ret;
	if (b_ret)
		client->b_fail += 1;
	else
		rq->elsp_submitted += 1;

	if (preemptive) {
		guc->preemptions[guc_id] += 1;
		guc->last_preempt[guc_id] = rq->seqno;
		if (b_ret)
			guc->preempt_failures[guc_id] += 1;
	} else {
		guc->submissions[guc_id] += 1;
		guc->last_seqno[guc_id] = rq->seqno;
		if (b_ret)
			guc->failures[guc_id] += 1;
	}

	return b_ret;
}

/*
 * Everything below here is concerned with setup & teardown, and is
 * therefore not part of the somewhat time-critical batch-submission
 * path of i915_guc_submit() above.
 */

/**
 * gem_allocate_guc_obj() - Allocate gem object for GuC usage
 * @dev:	drm device
 * @size:	size of object
 *
 * This is a wrapper to create a gem obj. In order to use it inside GuC, the
 * object needs to be pinned lifetime. Also we must pin it to gtt space other
 * than [0, GUC_WOPCM_TOP) because this range is reserved inside GuC.
 *
 * Return:	A drm_i915_gem_object if successful, otherwise error pointer.
 */
static struct drm_i915_gem_object *gem_allocate_guc_obj(struct drm_device *dev,
							u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	int ret;

	obj = i915_gem_alloc_object(dev, size);
	if (IS_ERR(obj))
		return obj;

	ret = i915_gem_object_get_pages(obj);
	if (ret) {
		drm_gem_object_unreference(&obj->base);
		return ERR_PTR(ret);
	}

	ret = i915_gem_obj_ggtt_pin(obj, PAGE_SIZE,
				    PIN_OFFSET_BIAS | GUC_WOPCM_TOP);
	if (ret) {
		drm_gem_object_unreference(&obj->base);
		return ERR_PTR(ret);
	}

	/* Invalidate GuC TLB to let GuC take the latest updates to GTT. */
	I915_WRITE(GEN8_GTCR, GEN8_GTCR_INVALIDATE);

	return obj;
}

/**
 * gem_release_guc_obj() - Release gem object allocated for GuC usage
 * @obj:	gem obj to be released
 */
static void gem_release_guc_obj(struct drm_i915_gem_object *obj)
{
	if (!obj)
		return;

	if (i915_gem_obj_is_pinned(obj))
		i915_gem_object_ggtt_unpin(obj);

	drm_gem_object_unreference(&obj->base);
}

static void guc_client_free(struct drm_device *dev,
			    struct i915_guc_client *client)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;

	if (!client)
		return;

	/*
	 * XXX: wait for any outstanding submissions before freeing memory.
	 * Be sure to drop any locks
	 */

	if (client->client_base) {
		/*
		 * If we got as far as setting up a doorbell, make sure
		 * we shut it down before unmapping & deallocating the
		 * memory. So first disable the doorbell, then tell the
		 * GuC that we've finished with it, finally deallocate
		 * it in our bitmap
		 */
		if (client->doorbell_id != GUC_INVALID_DOORBELL_ID) {
			guc_disable_doorbell(guc, client);
			host2guc_release_doorbell(guc, client);
			release_doorbell(guc, client->doorbell_id);
		}

		kunmap(kmap_to_page(client->client_base));
	}

	gem_release_guc_obj(client->client_obj);

	if (client->ctx_index != GUC_INVALID_CTX_ID) {
		guc_fini_ctx_desc(guc, client);
		ida_simple_remove(&guc->ctx_ids, client->ctx_index);
	}

	kfree(client);
}

/**
 * guc_client_alloc() - Allocate an i915_guc_client
 * @dev:	drm device
 * @priority:	four levels priority _CRITICAL, _HIGH, _NORMAL and _LOW
 * 		The kernel client to replace ExecList submission is created with
 * 		NORMAL priority. Priority of a client for scheduler can be HIGH,
 * 		while a preemption context can use CRITICAL.
 * @ctx:	the context that owns the client (we use the default render
 * 		context)
 *
 * Return:	An i915_guc_client object if success, error pointer on failure.
 */
static struct i915_guc_client *guc_client_alloc(struct drm_device *dev,
						uint32_t priority,
						struct intel_context *ctx)
{
	struct i915_guc_client *client;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct drm_i915_gem_object *obj;
	int ret;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->doorbell_id = GUC_INVALID_DOORBELL_ID;
	client->priority = priority;
	client->owner = ctx;
	client->guc = guc;

	client->ctx_index = (uint32_t)ida_simple_get(&guc->ctx_ids, 0,
			GUC_MAX_GPU_CONTEXTS, GFP_KERNEL);
	if (client->ctx_index >= GUC_MAX_GPU_CONTEXTS) {
		client->ctx_index = GUC_INVALID_CTX_ID;
		ret = -EINVAL;
		goto err;
	}

	/* The first page is doorbell/proc_desc. Two followed pages are wq. */
	obj = gem_allocate_guc_obj(dev, GUC_DB_SIZE + GUC_WQ_SIZE);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		goto err;
	}

	/* We'll keep just the first (doorbell/proc) page permanently kmap'd. */
	client->client_obj = obj;
	client->client_base = kmap(i915_gem_object_get_page(obj, 0));
	client->client_gtt = i915_gem_obj_ggtt_offset(obj);
	client->wq_offset = GUC_DB_SIZE;
	client->wq_size = GUC_WQ_SIZE;

	client->doorbell_offset = select_doorbell_cacheline(guc);

	/*
	 * Since the doorbell only requires a single cacheline, we can save
	 * space by putting the application process descriptor in the same
	 * page. Use the half of the page that doesn't include the doorbell.
	 */
	if (client->doorbell_offset >= (GUC_DB_SIZE / 2))
		client->proc_desc_offset = 0;
	else
		client->proc_desc_offset = (GUC_DB_SIZE / 2);

	client->doorbell_id = assign_doorbell(guc, client->priority);
	if (client->doorbell_id == GUC_INVALID_DOORBELL_ID) {
		/* XXX: evict a doorbell instead */
		ret = -EINVAL;
		goto err;
	}

	guc_init_proc_desc(guc, client);
	guc_init_ctx_desc(guc, client);
	guc_init_doorbell(guc, client);

	/* XXX: Any cache flushes needed? General domain mgmt calls? */

	ret = host2guc_allocate_doorbell(guc, client);
	if (ret)
		goto err;

	DRM_DEBUG_DRIVER("new priority %u client %p: ctx_index %u db_id %u\n",
		priority, client, client->ctx_index, client->doorbell_id);

	return client;

err:
	DRM_ERROR("FAILED to create priority %u GuC client!\n", priority);

	guc_client_free(dev, client);
	return ERR_PTR(ret);
}

/*
 * Sub buffer switch callback. Called whenever relay has to switch to a new
 * sub buffer, relay stays on the same sub buffer if 0 is returned.
 */
static int subbuf_start_callback(struct rchan_buf *buf,
				 void *subbuf,
				 void *prev_subbuf,
				 size_t prev_padding)
{
	/* Use no-overwrite mode by default, where relay will stop accepting
	 * new data if there are no empty sub buffers left.
	 * There is no strict synchronization enforced by relay between Consumer
	 * and Producer. In overwrite mode, there is a possibility of getting
	 * inconsistent/garbled data, the producer could be writing on to the
	 * same sub buffer from which Consumer is reading. This can't be avoided
	 * unless Consumer is fast enough and can always run in tandem with
	 * Producer.
	 */
	if (relay_buf_full(buf))
		return 0;

	return 1;
}

/*
 * file_create() callback. Creates relay file in debugfs.
 */
static struct dentry *create_buf_file_callback(const char *filename,
					       struct dentry *parent,
					       umode_t mode,
					       struct rchan_buf *buf,
					       int *is_global)
{
	struct dentry *buf_file;

	/* This to enable the use of a single buffer for the relay channel and
	 * correspondingly have a single file exposed to User, through which
	 * it can collect the logs in order without any post-processing.
	 * Need to set 'is_global' even if parent is NULL for early logging.
	 */
	*is_global = 1;

	if (!parent)
		return NULL;

	/* Not using the channel filename passed as an argument, since for each
	 * channel relay appends the corresponding CPU number to the filename
	 * passed in relay_open(). This should be fine as relay just needs a
	 * dentry of the file associated with the channel buffer and that file's
	 * name need not be same as the filename passed as an argument.
	 */
	buf_file = debugfs_create_file("guc_log", mode,
				       parent, buf, &relay_file_operations);
	return buf_file;
}

/*
 * file_remove() default callback. Removes relay file in debugfs.
 */
static int remove_buf_file_callback(struct dentry *dentry)
{
	debugfs_remove(dentry);
	return 0;
}

/* relay channel callbacks */
static struct rchan_callbacks relay_callbacks = {
	.subbuf_start = subbuf_start_callback,
	.create_buf_file = create_buf_file_callback,
	.remove_buf_file = remove_buf_file_callback,
};

static void guc_remove_log_relay_file(struct intel_guc *guc)
{
	relay_close(guc->log.relay_chan);
}

static int guc_create_relay_channel(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct rchan *guc_log_relay_chan;
	size_t n_subbufs, subbuf_size;

	/* Keep the size of sub buffers same as shared log buffer */
	subbuf_size = guc->log.obj->base.size;

	/* Store up to 8 snapshots, which is large enough to buffer sufficient
	 * boot time logs and provides enough leeway to User, in terms of
	 * latency, for consuming the logs from relay. Also doesn't take
	 * up too much memory.
	 */
	n_subbufs = 8;

	guc_log_relay_chan = relay_open(NULL, NULL, subbuf_size,
					n_subbufs, &relay_callbacks, dev_priv);
	if (!guc_log_relay_chan) {
		DRM_ERROR("Couldn't create relay chan for GuC logging\n");
		return -ENOMEM;
	}

	guc->log.relay_chan = guc_log_relay_chan;
	return 0;
}

static int guc_create_log_relay_file(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct dentry *log_dir;
	int ret;

	/* For now create the log file in /sys/kernel/debug/dri/0 dir */
	log_dir = dev_priv->dev->primary->debugfs_root;

	/* If /sys/kernel/debug/dri/0 location do not exist, then debugfs is
	 * not mounted and so can't create the relay file.
	 * The relay API seems to fit well with debugfs only, for availing relay
	 * there are 3 requirements which can be met for debugfs file only in a
	 * straightforward/clean manner :-
	 * i)   Need the associated dentry pointer of the file, while opening the
	 *      relay channel.
	 * ii)  Should be able to use 'relay_file_operations' fops for the file.
	 * iii) Set the 'i_private' field of file's inode to the pointer of
	 *	relay channel buffer.
	 */
	if (!log_dir) {
		DRM_ERROR("Debugfs dir not available yet for GuC log file\n");
		return -ENODEV;
	}

	ret = relay_late_setup_files(guc->log.relay_chan, "guc_log", log_dir);
	if (ret) {
		DRM_ERROR("Couldn't associate relay chan with file %d\n", ret);
		return ret;
	}

	return 0;
}

static void guc_move_to_next_buf(struct intel_guc *guc)
{
	/* Make sure the updates made in the sub buffer are visible when
	 * Consumer sees the following update to offset inside the sub buffer.
	 */
	smp_wmb();

	/* All data has been written, so now move the offset of sub buffer. */
	relay_reserve(guc->log.relay_chan, guc->log.obj->base.size);

	/* Switch to the next sub buffer */
	relay_flush(guc->log.relay_chan);
}

static void *guc_get_write_buffer(struct intel_guc *guc)
{
	if (!guc->log.relay_chan)
		return NULL;

	/* Just get the base address of a new sub buffer and copy data into it
	 * ourselves. NULL will be returned in no-overwrite mode, if all sub
	 * buffers are full. Could have used the relay_write() to indirectly
	 * copy the data, but that would have been bit convoluted, as we need to
	 * write to only certain locations inside a sub buffer which cannot be
	 * done without using relay_reserve() along with relay_write(). So its
	 * better to use relay_reserve() alone.
	 */
	return relay_reserve(guc->log.relay_chan, 0);
}

static bool
guc_check_log_buf_overflow(struct intel_guc *guc,
			   enum guc_log_buffer_type type, unsigned int full_cnt)
{
	unsigned int prev_full_cnt = guc->log.prev_overflow_count[type];
	bool overflow = false;

	if (full_cnt != prev_full_cnt) {
		overflow = true;

		guc->log.prev_overflow_count[type] = full_cnt;
		guc->log.total_overflow_count[type] += full_cnt - prev_full_cnt;

		if (full_cnt < prev_full_cnt) {
			/* buffer_full_cnt is a 4 bit counter */
			guc->log.total_overflow_count[type] += 16;
		}
		DRM_ERROR_RATELIMITED("GuC log buffer overflow\n");
	}

	return overflow;
}

static unsigned int guc_get_log_buffer_size(enum guc_log_buffer_type type)
{
	switch (type) {
	case GUC_ISR_LOG_BUFFER:
		return (GUC_LOG_ISR_PAGES + 1) * PAGE_SIZE;
	case GUC_DPC_LOG_BUFFER:
		return (GUC_LOG_DPC_PAGES + 1) * PAGE_SIZE;
	case GUC_CRASH_DUMP_LOG_BUFFER:
		return (GUC_LOG_CRASH_PAGES + 1) * PAGE_SIZE;
	default:
		MISSING_CASE(type);
	}

	return 0;
}

static void guc_read_update_log_buffer(struct intel_guc *guc)
{
	unsigned int buffer_size, read_offset, write_offset, bytes_to_copy, full_cnt;
	struct guc_log_buffer_state *log_buf_state, *log_buf_snapshot_state;
	struct guc_log_buffer_state log_buf_state_local;
	enum guc_log_buffer_type type;
	void *src_data, *dst_data;
	bool new_overflow;

	if (WARN_ON(!guc->log.buf_addr))
		return;

	/* Get the pointer to shared GuC log buffer */
	log_buf_state = src_data = guc->log.buf_addr;

	/* Get the pointer to local buffer to store the logs */
	log_buf_snapshot_state = dst_data = guc_get_write_buffer(guc);

	/* Actual logs are present from the 2nd page */
	src_data += PAGE_SIZE;
	dst_data += PAGE_SIZE;

	for (type = GUC_ISR_LOG_BUFFER; type < GUC_MAX_LOG_BUFFER; type++) {
		/* Make a copy of the state structure, inside GuC log buffer
		 * (which is uncached mapped), on the stack to avoid reading
		 * from it multiple times.
		 */
		memcpy(&log_buf_state_local, log_buf_state,
		       sizeof(struct guc_log_buffer_state));
		buffer_size = guc_get_log_buffer_size(type);
		read_offset = log_buf_state_local.read_ptr;
		write_offset = log_buf_state_local.sampled_write_ptr;
		full_cnt = log_buf_state_local.buffer_full_cnt;

		/* Bookkeeping stuff */
		guc->log.flush_count[type] += log_buf_state_local.flush_to_file;
		new_overflow = guc_check_log_buf_overflow(guc, type, full_cnt);

		/* Update the state of shared log buffer */
		log_buf_state->read_ptr = write_offset;
		log_buf_state->flush_to_file = 0;
		log_buf_state++;

		if (unlikely(!log_buf_snapshot_state))
			continue;

		/* First copy the state structure in snapshot buffer */
		memcpy(log_buf_snapshot_state, &log_buf_state_local,
		       sizeof(struct guc_log_buffer_state));

		/* The write pointer could have been updated by GuC firmware,
		 * after sending the flush interrupt to Host, for consistency
		 * set write pointer value to same value of sampled_write_ptr
		 * in the snapshot buffer.
		 */
		log_buf_snapshot_state->write_ptr = write_offset;
		log_buf_snapshot_state++;

		/* Now copy the actual logs. */
		if (unlikely(new_overflow)) {
			/* copy the whole buffer in case of overflow */
			read_offset = 0;
			write_offset = buffer_size;
		} else if (unlikely((read_offset > buffer_size) ||
				    (write_offset > buffer_size))) {
			DRM_ERROR("invalid log buffer state\n");
			/* copy whole buffer as offsets are unreliable */
			read_offset = 0;
			write_offset = buffer_size;
		}

		/* Just copy the newly written data */
		if (read_offset > write_offset) {
			i915_memcpy_from_wc(dst_data, src_data, write_offset);
			bytes_to_copy = buffer_size - read_offset;
		} else {
			bytes_to_copy = write_offset - read_offset;
		}
		i915_memcpy_from_wc(dst_data + read_offset,
				    src_data + read_offset, bytes_to_copy);

		src_data += buffer_size;
		dst_data += buffer_size;
	}

	if (log_buf_snapshot_state)
		guc_move_to_next_buf(guc);
	else {
		/* Used rate limited to avoid deluge of messages, logs might be
		 * getting consumed by User at a slow rate.
		 */
		DRM_ERROR_RATELIMITED("no sub-buffer to capture logs\n");
		guc->log.capture_miss_count++;
	}
}

static void guc_capture_logs_work(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, guc.log.flush_work);

	i915_guc_capture_logs(dev_priv->dev);
}

static void guc_log_cleanup(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);

	lockdep_assert_held(&dev_priv->dev->struct_mutex);

	if (i915.guc_log_level < 0)
		return;

	/* First disable the flush interrupt */
	gen9_disable_guc_interrupts(dev_priv);

	if (guc->log.flush_wq)
		destroy_workqueue(guc->log.flush_wq);

	guc->log.flush_wq = NULL;

	if (guc->log.relay_chan)
		guc_remove_log_relay_file(guc);

	guc->log.relay_chan = NULL;

	if (guc->log.buf_addr)
		i915_gem_object_unpin_map(guc->log.obj);

	guc->log.buf_addr = NULL;
}

static int guc_create_log_extras(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	void *vaddr;
	int ret;

	lockdep_assert_held(&dev_priv->dev->struct_mutex);

	/* Nothing to do */
	if (i915.guc_log_level < 0)
		return 0;

	if (!guc->log.buf_addr) {
		/* Create a WC (Uncached for read) vmalloc mapping of log
		 * buffer pages, so that we can directly get the data
		 * (up-to-date) from memory.
		 */
		vaddr = i915_gem_object_pin_map(guc->log.obj, I915_MAP_WC);
		if (IS_ERR(vaddr)) {
			ret = PTR_ERR(vaddr);
			DRM_ERROR("Couldn't map log buffer pages %d\n", ret);
			return ret;
		}

		guc->log.buf_addr = vaddr;
	}

	if (!guc->log.relay_chan) {
		/* Create a relay channel, so that we have buffers for storing
		 * the GuC firmware logs, the channel will be linked with a file
		 * later on when debugfs is registered.
		 */
		ret = guc_create_relay_channel(guc);
		if (ret)
			return ret;
	}

	if (!guc->log.flush_wq) {
		INIT_WORK(&guc->log.flush_work, guc_capture_logs_work);

		/* Need a dedicated wq to process log buffer flush interrupts
		 * from GuC without much delay so as to avoid any loss of logs.
		 *
		 * GuC log buffer flush work item has to do register access to
		 * send the ack to GuC and this work item, if not synced before
		 * suspend, can potentially get executed after the GFX device is
		 * suspended.
		 * By marking the WQ as freezable, we don't have to bother about
		 * flushing of this work item from the suspend hooks, the pending
		 * work item if any will be either executed before the suspend
		 * or scheduled later on resume. This way the handling of work
		 * item can be kept same between system suspend & rpm suspend.
		 */
		guc->log.flush_wq =
			alloc_ordered_workqueue("i915-guc_log", WQ_FREEZABLE);
		if (guc->log.flush_wq == NULL) {
			DRM_ERROR("Couldn't allocate the wq for GuC logging\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static void guc_create_log(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct drm_i915_gem_object *obj;
	unsigned long offset;
	uint32_t size, flags;

	if (i915.guc_log_level > GUC_LOG_VERBOSITY_MAX)
		i915.guc_log_level = GUC_LOG_VERBOSITY_MAX;

	/* The first page is to save log buffer state. Allocate one
	 * extra page for others in case for overlap */
	size = (1 + GUC_LOG_DPC_PAGES + 1 +
		GUC_LOG_ISR_PAGES + 1 +
		GUC_LOG_CRASH_PAGES + 1) << PAGE_SHIFT;

	obj = guc->log.obj;
	if (!obj) {
		/* We require SSE 4.1 for fast reads from the GuC log buffer and
		 * it should be present on the chipsets supporting GuC based
		 * submisssions.
		 */
		if (WARN_ON(!i915_memcpy_from_wc(NULL, NULL, 0))) {
			/* logging will not be enabled */
			i915.guc_log_level = -1;
			return;
		}

		obj = gem_allocate_guc_obj(dev_priv->dev, size);
		if (IS_ERR(obj)) {
			/* logging will be off */
			i915.guc_log_level = -1;
			return;
		}

		guc->log.obj = obj;

		if (guc_create_log_extras(guc)) {
			guc_log_cleanup(guc);
			gem_release_guc_obj(guc->log.obj);
			guc->log.obj = NULL;
			i915.guc_log_level = -1;
			return;
		}
	}

	/* each allocated unit is a page */
	flags = GUC_LOG_VALID | GUC_LOG_NOTIFY_ON_HALF_FULL |
		(GUC_LOG_DPC_PAGES << GUC_LOG_DPC_SHIFT) |
		(GUC_LOG_ISR_PAGES << GUC_LOG_ISR_SHIFT) |
		(GUC_LOG_CRASH_PAGES << GUC_LOG_CRASH_SHIFT);

	offset = i915_gem_obj_ggtt_offset(obj) >> PAGE_SHIFT; /* in pages */
	guc->log.flags = (offset << GUC_LOG_BUF_ADDR_SHIFT) | flags;
}

static int guc_log_late_setup(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	int ret;

	lockdep_assert_held(&dev_priv->dev->struct_mutex);

	if (i915.guc_log_level < 0)
		return -EINVAL;

	/* If log_level was set as -1 at boot time, then setup needed to
	 * handle log buffer flush interrupts would not have been done yet,
	 * so do that now.
	 */
	ret = guc_create_log_extras(guc);
	if (ret)
		goto err;

	/* Parent debugfs dir should be available by now, associate the already
	 * opened relay channel with a debugfs file, which will then allow User
	 * to pull the logs.
	 */
	ret = guc_create_log_relay_file(guc);
	if (ret)
		goto err;

	return 0;
err:
	guc_log_cleanup(guc);
	/* logging will remain off */
	i915.guc_log_level = -1;
	return ret;
}

static void init_guc_policies(struct guc_policies *policies)
{
	struct guc_policy *policy;
	u32 p, i;

	policies->dpc_promote_time = 500000;
	policies->max_num_work_items = POLICY_MAX_NUM_WI;

	for (p = 0; p < GUC_CTX_PRIORITY_NUM; p++) {
		for (i = GUC_RENDER_ENGINE; i < GUC_MAX_ENGINES_NUM; i++) {
			policy = &policies->policy[p][i];

			policy->execution_quantum = 1000000;
			policy->preemption_time = 500000;
			policy->fault_time = 250000;
			policy->policy_flags = 0;
		}
	}

	policies->is_valid = 1;
}

/*
 * In this macro it is highly unlikely to exceed max value but even if we did
 * it is not an error so just throw a warning and continue. Only side effect
 * in continuing further means some registers won't be added to save/restore
 * list.
 */
#define GUC_ADD_MMIO_REG_ADS(node, reg_addr, _flags)			\
	do {								\
		u32 __count = node->number_of_registers;		\
		if (WARN_ON(__count >= GUC_REGSET_MAX_REGISTERS))	\
			continue;					\
		node->registers[__count].offset = reg_addr.reg;		\
		node->registers[__count].flags = (_flags);		\
		node->number_of_registers++;				\
	} while (0)

static void guc_create_ads(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct drm_i915_gem_object *obj;
	struct guc_ads *ads;
	struct guc_policies *policies;
	struct guc_mmio_reg_state *reg_state;
	struct intel_engine_cs *engine;
	struct page *page;
	u32 size, i;

	/* The ads obj includes the struct itself and buffers passed to GuC */
	size = sizeof(struct guc_ads) + sizeof(struct guc_policies) +
			sizeof(struct guc_mmio_reg_state) +
			GUC_S3_SAVE_SPACE_PAGES * PAGE_SIZE;

	obj = guc->ads_obj;
	if (!obj) {
		obj = gem_allocate_guc_obj(dev_priv->dev, PAGE_ALIGN(size));
		if (!obj)
			return;

		guc->ads_obj = obj;
	}

	page = i915_gem_object_get_page(obj, 0);
	ads = kmap(page);

	/*
	 * The GuC requires a "Golden Context" when it reinitialises
	 * engines after a reset. Here we use the Render ring default
	 * context, which must already exist and be pinned in the GGTT,
	 * so its address won't change after we've told the GuC where
	 * to find it.
	 */
	engine = &dev_priv->engine[RCS];
	ads->golden_context_lrca = engine->status_page.gfx_addr;

	for_each_engine(engine, dev_priv, i)
		ads->eng_state_size[engine->guc_id] = intel_lr_context_size(engine);

	/* GuC scheduling policies */
	policies = (void *)ads + sizeof(struct guc_ads);
	init_guc_policies(policies);

	ads->scheduler_policies = i915_gem_obj_ggtt_offset(obj) +
			sizeof(struct guc_ads);

	/* MMIO reg state */
	reg_state = (void *)policies + sizeof(struct guc_policies);

	/*
	 * Provide a list of registers to be save/restored during gpu reset.
	 * This is mainly required for Media reset (aka watchdog timeout)
	 * which is completely under the control of GuC including resubmission
	 * of hung workload is handled by GuC.
	 */
	for_each_engine(engine, dev_priv, i) {
		u32 flags;
		struct guc_mmio_regset *eng_reg = &reg_state->engine_reg[engine->guc_id];

		flags = GUC_REGSET_POWERCYCLE | GUC_REGSET_ENGINERESET;

		GUC_ADD_MMIO_REG_ADS(eng_reg, RING_HEAD(engine->mmio_base),
				     flags | GUC_REGSET_SAVE_CURRENT_VALUE);

		GUC_ADD_MMIO_REG_ADS(eng_reg, RING_TAIL(engine->mmio_base),
				     flags | GUC_REGSET_SAVE_CURRENT_VALUE);

		GUC_ADD_MMIO_REG_ADS(eng_reg, RING_HWS_PGA(engine->mmio_base),
				     flags | GUC_REGSET_SAVE_DEFAULT_VALUE);

		GUC_ADD_MMIO_REG_ADS(eng_reg, RING_MODE_GEN7(engine),
				     (flags | GUC_REGSET_MASKED |
				      GUC_REGSET_SAVE_DEFAULT_VALUE));

		GUC_ADD_MMIO_REG_ADS(eng_reg, RING_IMR(engine->mmio_base),
				     flags | GUC_REGSET_SAVE_CURRENT_VALUE);

		DRM_DEBUG_DRIVER("%s register save/restore count: %u\n",
				 engine->name, eng_reg->number_of_registers);
	}

	for_each_engine(engine, dev_priv, i) {
		reg_state->mmio_white_list[engine->guc_id].mmio_start =
			engine->mmio_base + GUC_MMIO_WHITE_LIST_START;

		/* Nothing to be saved or restored for now. */
		reg_state->mmio_white_list[engine->guc_id].count = 0;
	}

	ads->reg_state_addr = ads->scheduler_policies +
			sizeof(struct guc_policies);

	ads->reg_state_buffer = ads->reg_state_addr +
			sizeof(struct guc_mmio_reg_state);

	kunmap(page);
}

/*
 * Set up the memory resources to be shared with the GuC.  At this point,
 * we require just one object that can be mapped through the GGTT.
 */
int i915_guc_submission_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	const size_t ctxsize = sizeof(struct guc_context_desc);
	const size_t poolsize = GUC_MAX_GPU_CONTEXTS * ctxsize;
	const size_t gemsize = round_up(poolsize, PAGE_SIZE);
	struct intel_guc *guc = &dev_priv->guc;
	int ret;

	if (!i915.enable_guc_submission)
		return 0; /* not enabled  */

	if (guc->ctx_pool_obj)
		return 0; /* already allocated */

	guc->ctx_pool_obj = gem_allocate_guc_obj(dev_priv->dev, gemsize);
	if (IS_ERR(guc->ctx_pool_obj)) {
		ret = PTR_ERR(guc->ctx_pool_obj);
		guc->ctx_pool_obj = NULL;
		return ret;
	}

	ida_init(&guc->ctx_ids);
	mutex_init(&guc->action_lock);

	guc_create_log(guc);

	guc_create_ads(guc);

	return 0;
}

int i915_guc_submission_enable(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct intel_context *ctx = dev_priv->engine[RCS].default_context;
	struct i915_guc_client *client;

	/* client for execbuf submission */
	client = guc_client_alloc(dev, GUC_CTX_PRIORITY_KMD_NORMAL, ctx);
	if (IS_ERR(client)) {
		DRM_ERROR("Failed to create execbuf guc_client\n");
		return PTR_ERR(client);
	}
	guc->execbuf_client = client;

	/* 2nd client for preemptive submission */
	client = guc_client_alloc(dev, GUC_CTX_PRIORITY_KMD_HIGH, ctx);
	if (IS_ERR(client)) {
		DRM_ERROR("Failed to create preemptive guc_client\n");
		return PTR_ERR(client);
	}
	guc->preempt_client = client;

	host2guc_sample_forcewake(guc, intel_enable_rc6(dev));

	return 0;
}

void i915_guc_submission_disable(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;

	guc_client_free(dev, guc->preempt_client);
	guc->preempt_client = NULL;
	guc_client_free(dev, guc->execbuf_client);
	guc->execbuf_client = NULL;
}

void i915_guc_submission_fini(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;

	gem_release_guc_obj(dev_priv->guc.ads_obj);
	guc->ads_obj = NULL;

	gem_release_guc_obj(dev_priv->guc.log.obj);
	guc->log.obj = NULL;

	if (guc->ctx_pool_obj)
		ida_destroy(&guc->ctx_ids);
	gem_release_guc_obj(guc->ctx_pool_obj);
	guc->ctx_pool_obj = NULL;
}

/**
 * intel_guc_suspend() - notify GuC entering suspend state
 * @dev:	drm device
 */
int intel_guc_suspend(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct intel_context *ctx;
	u32 data[3];

	if (!i915.enable_guc_submission)
		return 0;

	gen9_disable_guc_interrupts(dev_priv);

	ctx = dev_priv->engine[RCS].default_context;

	data[0] = HOST2GUC_ACTION_ENTER_S_STATE;
	/* any value greater than GUC_POWER_D0 */
	data[1] = GUC_POWER_D1;
	/* first page is shared data with GuC */
	data[2] = i915_gem_obj_ggtt_offset(ctx->engine[RCS].state);

	return host2guc_action(guc, data, ARRAY_SIZE(data));
}


/**
 * intel_guc_resume() - notify GuC resuming from suspend state
 * @dev:	drm device
 */
int intel_guc_resume(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct intel_context *ctx;
	u32 data[3];

	if (!i915.enable_guc_submission)
		return 0;

	if (i915.guc_log_level >= 0)
		gen9_enable_guc_interrupts(dev_priv);

	ctx = dev_priv->engine[RCS].default_context;

	data[0] = HOST2GUC_ACTION_EXIT_S_STATE;
	data[1] = GUC_POWER_D0;
	/* first page is shared data with GuC */
	data[2] = i915_gem_obj_ggtt_offset(ctx->engine[RCS].state);

	return host2guc_action(guc, data, ARRAY_SIZE(data));
}

void i915_guc_capture_logs(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	guc_read_update_log_buffer(&dev_priv->guc);

	/* Generally device is expected to be active only at this
	 * time, so get/put should be really quick.
	 */
	intel_runtime_pm_get(dev_priv);
	host2guc_logbuffer_flush_complete(&dev_priv->guc);
	intel_runtime_pm_put(dev_priv);
}

void i915_guc_flush_logs(struct drm_device *dev, bool can_wait)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!i915.enable_guc_submission || (i915.guc_log_level < 0))
		return;

	/* First disable the interrupts, will be renabled afterwards */
	gen9_disable_guc_interrupts(dev_priv);

	/* Before initiating the forceful flush, wait for any pending/ongoing
	 * flush to complete otherwise forceful flush may not happen, but wait
	 * can't be done for some paths like error state capture in which case
	 * take a chance & directly attempt the forceful flush.
	 */
	if (can_wait)
		flush_work(&dev_priv->guc.log.flush_work);

	/* Ask GuC to update the log buffer state */
	host2guc_force_logbuffer_flush(&dev_priv->guc);
}

void i915_guc_unregister(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!i915.enable_guc_submission)
		return;

	mutex_lock(&dev->struct_mutex);
	guc_log_cleanup(&dev_priv->guc);
	mutex_unlock(&dev->struct_mutex);
}

void i915_guc_register(struct drm_device *dev, bool not_defered)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!i915.enable_guc_submission)
		return;

	/* caller from i915_gem_context_first_open already has the mutex */
	if (not_defered)
		mutex_lock(&dev->struct_mutex);
	else
		lockdep_assert_held(&dev->struct_mutex);

	guc_log_late_setup(&dev_priv->guc);

	if (not_defered)
		mutex_unlock(&dev->struct_mutex);
}

int i915_guc_log_control(struct drm_device *dev, u64 control_val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	union guc_log_control log_param;
	int ret;

	log_param.value = control_val;

	if (log_param.verbosity < GUC_LOG_VERBOSITY_MIN ||
	    log_param.verbosity > GUC_LOG_VERBOSITY_MAX)
		return -EINVAL;

	/* This combination doesn't make sense & won't have any effect */
	if (!log_param.logging_enabled && (i915.guc_log_level < 0))
		return 0;

	ret = host2guc_logging_control(&dev_priv->guc, log_param.value);
	if (ret < 0) {
		DRM_DEBUG_DRIVER("host2guc action failed %d\n", ret);
		return ret;
	}

	i915.guc_log_level = log_param.verbosity;

	/* If log_level was set as -1 at boot time, then the relay channel file
	 * wouldn't have been created by now and interrupts also would not have
	 * been enabled.
	 */
	if (!dev_priv->guc.log.relay_chan) {
		ret = guc_log_late_setup(&dev_priv->guc);
		if (!ret)
			gen9_enable_guc_interrupts(dev_priv);
	} else if (!log_param.logging_enabled) {
		/* Once logging is disabled, GuC won't generate logs & send an
		 * interrupt. But there could be some data in the log buffer
		 * which is yet to be captured. So request GuC to update the log
		 * buffer state and then collect the left over logs.
		 */
		i915_guc_flush_logs(dev, true);

		/* GuC would have updated log buffer by now, so capture it */
		i915_guc_capture_logs(dev);

		/* As logging is disabled, update log level to reflect that */
		i915.guc_log_level = -1;
	} else {
		/* In case interrupts were disabled, enable them now */
		gen9_enable_guc_interrupts(dev_priv);
	}

	return ret;
}

int i915_guc_request_engine_reset(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct intel_context *ctx;
	u32 data[7];
	int ret;

	if (!i915.enable_guc_submission)
		return 0;

	ctx = dev_priv->engine[RCS].default_context;

	/*
	 * The affected context report is populated by GuC and is provided
	 * to the driver using the shared page. We request for it but don't
	 * use it as scheduler has all of these details.
	 */
	data[0] = HOST2GUC_ACTION_REQUEST_ENGINE_RESET;
	data[1] = engine->guc_id;
	data[2] = HOST2GUC_RESET_OPTION_REPORT_AFFECTED_CONTEXTS;
	data[3] = 0;
	data[4] = 0;
	data[5] = guc->execbuf_client->ctx_index;
	/* first page is used to shared data with GuC */
	data[6] = i915_gem_obj_ggtt_offset(ctx->engine[RCS].state);

	ret = host2guc_action(guc, data, ARRAY_SIZE(data));
	if (ret) {
		DRM_ERROR("Reset request for %s failed, ret=%d\n",
			  engine->name, ret);
	}

	return ret;
}

/**
 * intel_huc_ucode_auth() - authenticate ucode
 * @dev: the drm device
 *
 * Triggers a HuC fw authentication request to the GuC via host-2-guc
 * interface.
 */
void intel_huc_ucode_auth(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_guc *guc = &dev_priv->guc;
	struct intel_huc *huc = &dev_priv->huc;
	int ret;
	u32 data[2];

	/* Bypass the case where there is no HuC firmware */
	if (huc->huc_fw.fetch_status == UC_FIRMWARE_NONE ||
	    huc->huc_fw.load_status == UC_FIRMWARE_NONE)
		return;

	if (guc->guc_fw.load_status != UC_FIRMWARE_SUCCESS) {
		DRM_ERROR("HuC: GuC fw wasn't loaded. Can't authenticate");
		return;
	}

	if (huc->huc_fw.load_status != UC_FIRMWARE_SUCCESS) {
		DRM_ERROR("HuC: fw wasn't loaded. Nothing to authenticate");
		return;
	}

	ret = i915_gem_obj_ggtt_pin(huc->huc_fw.uc_fw_obj, 0, 0);
	if (ret) {
		DRM_ERROR("HuC: Pin failed");
		return;
	}

	/* Invalidate GuC TLB to let GuC take the latest updates to GTT. */
	I915_WRITE(GEN8_GTCR, GEN8_GTCR_INVALIDATE);

	/* Specify auth action and where public signature is. It's stored
	 * at the beginning of the gem object, before the fw bits
	 */
	data[0] = HOST2GUC_ACTION_AUTHENTICATE_HUC;
	data[1] = i915_gem_obj_ggtt_offset(huc->huc_fw.uc_fw_obj) +
			huc->huc_fw.rsa_offset;

	ret = host2guc_action(guc, data, ARRAY_SIZE(data));
	if (ret) {
		DRM_ERROR("HuC: GuC did not ack Auth request\n");
		goto out;
	}

	/* Check authentication status, it should be done by now */
	ret = wait_for(
		(I915_READ(HUC_STATUS2) & HUC_FW_VERIFIED) > 0, 5000);
	if (ret) {
		DRM_ERROR("HuC: Authentication failed\n");
		goto out;
	}

out:
	i915_gem_object_ggtt_unpin(huc->huc_fw.uc_fw_obj);
}
