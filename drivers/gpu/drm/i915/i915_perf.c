/*
 * Copyright © 2015 Intel Corporation
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
 */

#include <linux/perf_event.h>
#include <linux/anon_inodes.h>
#include <linux/sizes.h>

#include "i915_drv.h"
#include "intel_ringbuffer.h"
#include "intel_lrc.h"
#include "i915_oa_hsw.h"
#include "i915_oa_bdw.h"
#include "i915_oa_chv.h"
#include "i915_oa_skl.h"

/* Must be a power of two */
#define OA_BUFFER_SIZE	     SZ_16M
#define OA_TAKEN(tail, head) ((tail - head) & (OA_BUFFER_SIZE - 1))

/* frequency for forwarding samples from OA to perf buffer */
#define POLL_FREQUENCY 200
#define POLL_PERIOD max_t(u64, 10000, NSEC_PER_SEC / POLL_FREQUENCY)

static u32 i915_perf_event_paranoid = true;

#define OA_EXPONENT_MAX 0x3f

#define I915_PERF_GEN_NODE_SIZE (8 + (4*I915_GEN_PERF_MMIO_NUM))

/* Returns the engine's ID mask (i.e. I915_EXEC_<engine>) */
#define ENGINE_ID_MASK(engine) ((engine)->id + 1)
static const enum intel_engine_id ring_map[I915_NUM_ENGINES + 1] = {
	[RCS] = I915_EXEC_RENDER,
	[BCS] = I915_EXEC_BLT,
	[VCS] = I915_EXEC_BSD,
	[VCS2] = I915_EXEC_BSD,
	[VECS] = I915_EXEC_VEBOX,
};

/* for sysctl proc_dointvec_minmax of i915_oa_event_min_timer_exponent */
static int zero;
static int oa_exponent_max = OA_EXPONENT_MAX;

/* Theoretically we can program the OA unit to sample every 160ns but don't
 * allow that by default unless root...
 *
 * The period is derived from the exponent as:
 *
 *   period = 80ns * 2^(exponent + 1)
 *
 * Referring to perf's kernel.perf_event_max_sample_rate for a precedent
 * (100000 by default); with an OA exponent of 6 we get a period of 10.240
 * microseconds - just under 100000Hz
 */
static u32 i915_oa_event_min_timer_exponent = 6;

static struct i915_oa_format hsw_oa_formats[I915_OA_FORMAT_MAX] = {
	[I915_OA_FORMAT_A13]	    = { 0, 64 },
	[I915_OA_FORMAT_A29]	    = { 1, 128 },
	[I915_OA_FORMAT_A13_B8_C8]  = { 2, 128 },
	/* A29_B8_C8 Disallowed as 192 bytes doesn't factor into buffer size */
	[I915_OA_FORMAT_B4_C8]	    = { 4, 64 },
	[I915_OA_FORMAT_A45_B8_C8]  = { 5, 256 },
	[I915_OA_FORMAT_B4_C8_A16]  = { 6, 128 },
	[I915_OA_FORMAT_C4_B8]	    = { 7, 64 },
};

static struct i915_oa_format gen8_plus_oa_formats[I915_OA_FORMAT_MAX] = {
	[I915_OA_FORMAT_A12]		    = { 0, 64 },
	[I915_OA_FORMAT_A12_B8_C8]	    = { 2, 128 },
	[I915_OA_FORMAT_A32u40_A4u32_B8_C8] = { 5, 256 },
	[I915_OA_FORMAT_C4_B8]		    = { 7, 64 },
};


/**
 * i915_perf_copy_attr() - copy specific event attributes from userspace
 * @uattr:	The u64 __user attr of drm_i915_perf_open_param
 * @attr:	Destination for copied attributes
 * @v0_size:	The smallest, version 0 size of these attributes
 * @real_size:	The latest size expected by this kernel version
 *
 * Specific events can define a custom attributes structure and for
 * consistency should use this utility for reading the attributes from
 * userspace.
 *
 * Note: although this verifies any unknown members beyond the expected
 * struct size are zeroed it can't check for unused flags
 *
 * Return: 0 if successful, else an error code
 */
static int i915_perf_copy_attr(void __user *uattr,
			       void *attr,
			       u32 v0_size,
			       u32 real_size)
{
	u32 size;
	int ret;

	if (!access_ok(VERIFY_WRITE, uattr, v0_size))
		return -EFAULT;

	/*
	 * zero the full structure, so that a short copy will be nice.
	 */
	memset(attr, 0, real_size);

	ret = get_user(size, (u32 __user *)uattr);
	if (ret)
		return ret;

	if (size > PAGE_SIZE)   /* silly large */
		goto err_size;

	if (size < v0_size)
		goto err_size;

	/*
	 * If we're handed a bigger struct than we know of,
	 * ensure all the unknown bits are 0 - i.e. new
	 * user-space does not rely on any kernel feature
	 * extensions we dont know about yet.
	 */

	if (size > real_size) {
		unsigned char __user *addr;
		unsigned char __user *end;
		unsigned char val;

		addr = (void __user *)uattr + sizeof(*attr);
		end  = (void __user *)uattr + size;

		for (; addr < end; addr++) {
			ret = get_user(val, addr);
			if (ret)
				return ret;
			if (val)
				goto err_size;
		}
		size = sizeof(*attr);
	}

	ret = copy_from_user(attr, uattr, size);
	if (ret)
		return -EFAULT;

out:
	return ret;

err_size:
	put_user(real_size, (u32 __user *)uattr);
	ret = -E2BIG;
	goto out;
}

void i915_emit_profiling_data(struct drm_i915_gem_request *req, u32 ctx_id,
				u32 tag)
{
	struct intel_engine_cs *engine = req->engine;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_perf_event *event;

	if (!dev_priv->perf.initialized)
		return;

	list_for_each_entry(event, &dev_priv->perf.events, link) {
		if (event->emit_profiling_data)
			event->emit_profiling_data(req, ctx_id, tag);
	}
}

/*
 * Emits the commands to capture OA perf report, into the Render CS
 */
void i915_oa_emit_report(struct drm_i915_gem_request *req, u32 ctx_id,
			u32 tag)
{
	struct intel_engine_cs *engine = req->engine;
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_oa_rcs_node *entry;
	u32 addr = 0;
	int ret;

	/* OA counters are only supported on the render engine */
	if (engine->id != RCS)
		return;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		DRM_ERROR("alloc failed\n");
		return;
	}

	if (i915.enable_execlists)
		ret = intel_logical_ring_begin(req, 4);
	else
		ret = intel_ring_begin(req, 4);

	if (ret) {
		kfree(entry);
		return;
	}

	entry->ctx_id = ctx_id;
	entry->pid = current->pid;
	entry->tag = tag;
	i915_gem_request_assign(&entry->req, req);

	spin_lock(&dev_priv->perf.oa.rcs_node_list_lock);
	if (list_empty(&dev_priv->perf.oa.rcs_node_list))
		entry->offset = 0;
	else {
		struct i915_oa_rcs_node *last_entry;
		int max_offset = dev_priv->perf.oa.oa_rcs_buffer.node_count *
				dev_priv->perf.oa.oa_rcs_buffer.format_size;

		last_entry = list_last_entry(&dev_priv->perf.oa.rcs_node_list,
					struct i915_oa_rcs_node, head);
		entry->offset = last_entry->offset +
				dev_priv->perf.oa.oa_rcs_buffer.format_size;

		if (entry->offset > max_offset)
			entry->offset = 0;
	}
	list_add_tail(&entry->head, &dev_priv->perf.oa.rcs_node_list);
	spin_unlock(&dev_priv->perf.oa.rcs_node_list_lock);

	addr = dev_priv->perf.oa.oa_rcs_buffer.gtt_offset + entry->offset;

	/* addr should be 64 byte aligned */
	BUG_ON(addr & 0x3f);

	if (i915.enable_execlists) {
		intel_logical_ring_emit(ringbuf,
				MI_REPORT_PERF_COUNT | (2<<0));
		intel_logical_ring_emit(ringbuf,
				addr | MI_REPORT_PERF_COUNT_GGTT);
		intel_logical_ring_emit(ringbuf, 0);
		intel_logical_ring_emit(ringbuf, req->seqno);
		intel_logical_ring_advance(ringbuf);
	} else {
		if (INTEL_INFO(engine->dev)->gen >= 8) {
			intel_ring_emit(engine, MI_REPORT_PERF_COUNT | (2<<0));
			intel_ring_emit(engine, addr | MI_REPORT_PERF_COUNT_GGTT);
			intel_ring_emit(engine, 0);
			intel_ring_emit(engine, req->seqno);
		} else {
			intel_ring_emit(engine, MI_REPORT_PERF_COUNT | (1<<0));
			intel_ring_emit(engine, addr | MI_REPORT_PERF_COUNT_GGTT);
			intel_ring_emit(engine, req->seqno);
			intel_ring_emit(engine, MI_NOOP);
		}
		intel_ring_advance(engine);
	}
	i915_vma_move_to_active(dev_priv->perf.oa.oa_rcs_buffer.vma, req);
}

/*
 * Emits the commands to capture timestamps, into the CS
 */
void i915_gen_emit_ts_data(struct drm_i915_gem_request *req,
				u32 ctx_id, u32 tag)
{
	struct intel_engine_cs *engine = req->engine;
	struct intel_ringbuffer *ringbuf = req->ringbuf;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_perf_gen_data_node *entry;
	u32 mmio_addr, addr = 0;
	int ret, i, count = 0;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		DRM_ERROR("alloc failed\n");
		return;
	}

	for (count = 0; count < I915_GEN_PERF_MMIO_NUM; count++) {
		if (0 == dev_priv->perf.generic.mmio_list[count])
			break;
	}

	if (i915.enable_execlists)
		ret = intel_logical_ring_begin(req, 6 + 4*count);
	else
		ret = intel_ring_begin(req, 6 + 4*count);

	if (ret) {
		kfree(entry);
		return;
	}

	entry->ctx_id = ctx_id;
	entry->pid = current->pid;
	entry->tag = tag;
	entry->ring_id = ring_map[engine->id];
	i915_gem_request_assign(&entry->req, req);

	spin_lock(&dev_priv->perf.generic.node_list_lock_generic);
	if (list_empty(&dev_priv->perf.generic.node_list_generic))
		entry->offset = 0;
	else {
		struct i915_perf_gen_data_node *last_entry;
		int max_offset = dev_priv->perf.generic.buffer.node_count *
					I915_PERF_GEN_NODE_SIZE;

		last_entry = list_last_entry(
				&dev_priv->perf.generic.node_list_generic,
				struct i915_perf_gen_data_node, head);

		entry->offset = last_entry->offset + I915_PERF_GEN_NODE_SIZE;

		if (entry->offset > max_offset)
			entry->offset = 0;
	}
	list_add_tail(&entry->head, &dev_priv->perf.generic.node_list_generic);
	spin_unlock(&dev_priv->perf.generic.node_list_lock_generic);

	addr = dev_priv->perf.generic.buffer.gtt_offset + entry->offset;
	mmio_addr = addr + 8;

	if (i915.enable_execlists) {
		if (engine->id == RCS) {
			intel_logical_ring_emit(ringbuf,
						GFX_OP_PIPE_CONTROL(6));
			intel_logical_ring_emit(ringbuf,
						PIPE_CONTROL_GEN7_GLOBAL_GTT |
						PIPE_CONTROL_TIMESTAMP_WRITE);
			intel_logical_ring_emit(ringbuf, addr |
						PIPE_CONTROL_GLOBAL_GTT);
			intel_logical_ring_emit(ringbuf, 0);
			intel_logical_ring_emit(ringbuf, 0);
			intel_logical_ring_emit(ringbuf, 0);
		} else {
			uint32_t cmd;

			cmd = MI_FLUSH_DW + 2; /* Gen8+ */

			cmd |= MI_FLUSH_DW_OP_STAMP;

			intel_logical_ring_emit(ringbuf, cmd);
			intel_logical_ring_emit(ringbuf, addr |
						MI_FLUSH_DW_USE_GTT);
			intel_logical_ring_emit(ringbuf, 0);
			intel_logical_ring_emit(ringbuf, 0);
			intel_logical_ring_emit(ringbuf, 0);
			intel_logical_ring_emit(ringbuf, MI_NOOP);
		}
		for (i = 0; i < I915_GEN_PERF_MMIO_NUM; i++) {
			uint32_t cmd;

			if (0 == dev_priv->perf.generic.mmio_list[i])
				break;

			addr = mmio_addr +
				i * sizeof(
					dev_priv->perf.generic.mmio_list[i]);

			cmd = MI_STORE_REGISTER_MEM_GEN8 |
					MI_SRM_LRM_GLOBAL_GTT;

			intel_logical_ring_emit(ringbuf, cmd);
			intel_logical_ring_emit(ringbuf,
					dev_priv->perf.generic.mmio_list[i]);
			intel_logical_ring_emit(ringbuf, addr);
			intel_logical_ring_emit(ringbuf, 0);
		}
		intel_logical_ring_advance(ringbuf);
	} else {
		if (engine->id == RCS) {
			if (INTEL_INFO(engine->dev)->gen >= 8)
				intel_ring_emit(engine, GFX_OP_PIPE_CONTROL(6));
			else
				intel_ring_emit(engine, GFX_OP_PIPE_CONTROL(5));
			intel_ring_emit(engine,
					PIPE_CONTROL_GEN7_GLOBAL_GTT |
					PIPE_CONTROL_TIMESTAMP_WRITE);
			intel_ring_emit(engine, addr | PIPE_CONTROL_GLOBAL_GTT);
			intel_ring_emit(engine, 0);
			if (INTEL_INFO(engine->dev)->gen >= 8) {
				intel_ring_emit(engine, 0);
				intel_ring_emit(engine, 0);
			} else {
				intel_ring_emit(engine, 0);
				intel_ring_emit(engine, MI_NOOP);
			}
		} else {
			uint32_t cmd;

			cmd = MI_FLUSH_DW + 1;
			if (INTEL_INFO(engine->dev)->gen >= 8)
				cmd += 1;

			cmd |= MI_FLUSH_DW_OP_STAMP;

			intel_ring_emit(engine, cmd);
			intel_ring_emit(engine, addr | MI_FLUSH_DW_USE_GTT);
			if (INTEL_INFO(engine->dev)->gen >= 8) {
				intel_ring_emit(engine, 0);
				intel_ring_emit(engine, 0);
				intel_ring_emit(engine, 0);
			} else {
				intel_ring_emit(engine, 0);
				intel_ring_emit(engine, 0);
				intel_ring_emit(engine, MI_NOOP);
			}
			intel_ring_emit(engine, MI_NOOP);
		}
		for (i = 0; i < I915_GEN_PERF_MMIO_NUM; i++) {
			uint32_t cmd;

			if (0 == dev_priv->perf.generic.mmio_list[i])
				break;

			addr = mmio_addr +
				i * sizeof(
					dev_priv->perf.generic.mmio_list[i]);

			if (INTEL_INFO(engine->dev)->gen >= 8)
				cmd = MI_STORE_REGISTER_MEM_GEN8 |
					MI_SRM_LRM_GLOBAL_GTT;
			else
				cmd = MI_STORE_REGISTER_MEM |
					MI_SRM_LRM_GLOBAL_GTT;

			intel_ring_emit(engine, cmd);
			intel_ring_emit(engine,
					dev_priv->perf.generic.mmio_list[i]);
			intel_ring_emit(engine, addr);
			if (INTEL_INFO(engine->dev)->gen >= 8)
				intel_ring_emit(engine, 0);
			else
				intel_ring_emit(engine, MI_NOOP);
		}
		intel_ring_advance(engine);
	}
	i915_vma_move_to_active(dev_priv->perf.generic.buffer.vma, req);
}

static int i915_oa_rcs_wait_gpu(struct drm_i915_private *dev_priv)
{
	struct i915_oa_rcs_node *last_entry = NULL;
	int ret;

	/*
	 * Wait for the last scheduled request to complete. This would
	 * implicitly wait for the prior submitted requests. The refcount
	 * of the requests is not decremented here.
	 */
	spin_lock(&dev_priv->perf.oa.rcs_node_list_lock);

	if (!list_empty(&dev_priv->perf.oa.rcs_node_list)) {
		last_entry = list_last_entry(&dev_priv->perf.oa.rcs_node_list,
			struct i915_oa_rcs_node, head);
	}
	spin_unlock(&dev_priv->perf.oa.rcs_node_list_lock);

	if (!last_entry)
		return 0;

	ret = __i915_wait_request(last_entry->req, atomic_read(
				&dev_priv->gpu_error.reset_counter),
				true, NULL, NULL);
	if (ret) {
		DRM_ERROR("failed to wait\n");
		return ret;
	}
	return 0;
}

static void i915_oa_rcs_free_requests(struct drm_i915_private *dev_priv)
{
	struct i915_oa_rcs_node *entry, *next;

	list_for_each_entry_safe
		(entry, next, &dev_priv->perf.oa.rcs_node_list, head) {
		i915_gem_request_unreference(entry->req);

		spin_lock(&dev_priv->perf.oa.rcs_node_list_lock);
		list_del(&entry->head);
		spin_unlock(&dev_priv->perf.oa.rcs_node_list_lock);
		kfree(entry);
	}
}

static bool append_oa_rcs_sample(struct i915_perf_event *event,
			     struct i915_perf_read_state *read_state,
			     struct i915_oa_rcs_node *node)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	int report_size = dev_priv->perf.oa.oa_rcs_buffer.format_size;
	struct drm_i915_perf_event_header header;
	u32 sample_flags = event->sample_flags;
	u8 *report = dev_priv->perf.oa.oa_rcs_buffer.addr + node->offset;
	enum drm_i915_perf_oa_event_source event_source =
				I915_PERF_OA_EVENT_SOURCE_CS;
	u32 report_ts;

	/*
	 * Forward the periodic OA samples which have the timestamp lower
	 * than timestamp of this sample, before forwarding this sample.
	 * This ensures samples read by user are order acc. to their timestamps
	 */
	report_ts = *(u32 *)(report + 4);
	dev_priv->perf.oa.ops.read(event, read_state, report_ts);

	header.type = DRM_I915_PERF_RECORD_SAMPLE;
	header.misc = 0;
	header.size = sizeof(header);

	/* XXX: could pre-compute this when opening the event... */

	if (sample_flags & I915_PERF_SAMPLE_SOURCE_INFO)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_CTXID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_PID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_TAG)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_OA_REPORT)
		header.size += report_size;

	if ((read_state->count - read_state->read) < header.size)
		return false;

	if (copy_to_user(read_state->buf, &header, sizeof(header)))
		return false;

	read_state->buf += sizeof(header);

	if (sample_flags & I915_PERF_SAMPLE_SOURCE_INFO) {
		if (copy_to_user(read_state->buf, &event_source, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_CTXID) {
		if (copy_to_user(read_state->buf, &node->ctx_id, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_PID) {
		if (copy_to_user(read_state->buf, &node->pid, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_TAG) {
		if (copy_to_user(read_state->buf, &node->tag, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_OA_REPORT) {
		if (copy_to_user(read_state->buf, report, report_size))
			return false;
		read_state->buf += report_size;
	}

	read_state->read += header.size;

	return true;
}

static void oa_rcs_append_reports(struct i915_perf_event *event,
			     struct i915_perf_read_state *read_state)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	struct i915_oa_rcs_node *entry, *next;

	if (!dev_priv->perf.oa.exclusive_event->enabled)
		return;

	list_for_each_entry_safe
		(entry, next, &dev_priv->perf.oa.rcs_node_list, head) {
		if (!i915_gem_request_completed(entry->req))
			break;

		if (!append_oa_rcs_sample(event, read_state, entry))
			break;

		spin_lock(&dev_priv->perf.oa.rcs_node_list_lock);
		list_del(&entry->head);
		spin_unlock(&dev_priv->perf.oa.rcs_node_list_lock);

		i915_gem_request_unreference(entry->req);
		kfree(entry);
	}
}

static bool oa_rcs_buffer_is_empty(struct drm_i915_private *dev_priv)
{
	if (dev_priv->perf.oa.rcs_sample_mode)
		return list_empty(&dev_priv->perf.oa.rcs_node_list);
	else
		return true;
}

static bool oa_buffer_is_empty(struct drm_i915_private *dev_priv)
{
	return (dev_priv->perf.oa.ops.oa_buffer_is_empty(dev_priv) &&
		oa_rcs_buffer_is_empty(dev_priv));
}

static bool gen8_oa_buffer_is_empty(struct drm_i915_private *dev_priv)
{
	u32 head = I915_READ(GEN8_OAHEADPTR);
	u32 tail = I915_READ(GEN8_OATAILPTR);

	return OA_TAKEN(tail, head) == 0;
}

static bool gen7_oa_buffer_is_empty(struct drm_i915_private *dev_priv)
{
	u32 oastatus2 = I915_READ(GEN7_OASTATUS2);
	u32 oastatus1 = I915_READ(GEN7_OASTATUS1);
	u32 head = oastatus2 & GEN7_OASTATUS2_HEAD_MASK;
	u32 tail = oastatus1 & GEN7_OASTATUS1_TAIL_MASK;

	return OA_TAKEN(tail, head) == 0;
}

static bool append_oa_status(struct i915_perf_event *event,
			     struct i915_perf_read_state *read_state,
			     enum drm_i915_perf_record_type type)
{
	struct drm_i915_perf_event_header header = { type, 0, sizeof(header) };

	if ((read_state->count - read_state->read) < header.size)
		return false;

	if (copy_to_user(read_state->buf, &header, sizeof(header)))
		return false;

	read_state->buf += sizeof(header);
	read_state->read += header.size;

	return true;
}

static bool append_oa_sample(struct i915_perf_event *event,
			     struct i915_perf_read_state *read_state,
			     const u8 *report)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	int report_size = dev_priv->perf.oa.oa_buffer.format_size;
	struct drm_i915_perf_event_header header;
	u32 sample_flags = event->sample_flags;
	enum drm_i915_perf_oa_event_source event_source =
				I915_PERF_OA_EVENT_SOURCE_PERIODIC;
	u32 dummy_ctx_id = 0, dummy_pid = 0, dummy_tag = 0;

	header.type = DRM_I915_PERF_RECORD_SAMPLE;
	header.misc = 0;
	header.size = sizeof(header);


	/* XXX: could pre-compute this when opening the event... */

	if (sample_flags & I915_PERF_SAMPLE_SOURCE_INFO)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_CTXID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_PID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_TAG)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_OA_REPORT)
		header.size += report_size;


	if ((read_state->count - read_state->read) < header.size)
		return false;


	if (copy_to_user(read_state->buf, &header, sizeof(header)))
		return false;

	read_state->buf += sizeof(header);

	if (sample_flags & I915_PERF_SAMPLE_SOURCE_INFO) {
		if (copy_to_user(read_state->buf, &event_source, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_CTXID) {
		/* TODO: Extract context ID from OA reports */
		if (copy_to_user(read_state->buf, &dummy_ctx_id, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_PID) {
		if (copy_to_user(read_state->buf, &dummy_pid, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_TAG) {
		if (copy_to_user(read_state->buf, &dummy_tag, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_OA_REPORT) {
		if (copy_to_user(read_state->buf, report, report_size))
			return false;
		read_state->buf += report_size;
	}

	read_state->read += header.size;

	return true;
}

static u32 gen8_append_oa_reports(struct i915_perf_event *event,
				  struct i915_perf_read_state *read_state,
				  u32 head,
				  u32 tail, u32 ts)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	int report_size = dev_priv->perf.oa.oa_buffer.format_size;
	u8 *oa_buf_base = dev_priv->perf.oa.oa_buffer.addr;
	u32 mask = (OA_BUFFER_SIZE - 1);
	u8 *report;
	u32 report_ts, taken;

	head -= dev_priv->perf.oa.oa_buffer.gtt_offset;
	tail -= dev_priv->perf.oa.oa_buffer.gtt_offset;

	/* Note: the gpu doesn't wrap the tail according to the OA buffer size
	 * so when we need to make sure our head/tail values are in-bounds we
	 * use the above mask.
	 */

	while ((taken = OA_TAKEN(tail, head))) {
		u32 ctx_id;

		/* The tail increases in 64 byte increments, not in
		 * format_size steps. */
		if (taken < report_size)
			break;

		/* All the report sizes factor neatly into the buffer
		 * size so we never expect to see a report split
		 * between the beginning and end of the buffer... */
		BUG_ON((OA_BUFFER_SIZE - (head & mask)) < report_size);

		report = oa_buf_base + (head & mask);

		report_ts = *(u32 *)(report + 4);
		if (report_ts > ts)
			break;

		ctx_id = *(u32 *)(report + 12);
		if (i915.enable_execlists) {
			/* XXX: Just keep the lower 20 bits for now since I'm
			 * not entirely sure if the HW touches any of the higher
			 * bits */
			ctx_id &= 0xfffff;
		}

		if (dev_priv->perf.oa.exclusive_event->enabled) {

			/* NB: For Gen 8 we handle per-context report filtering
			 * ourselves instead of programming the OA unit with a
			 * specific context id.
			 *
			 * NB: To allow userspace to calculate all counter
			 * deltas for a specific context we have to send the
			 * first report belonging to any subsequently
			 * switched-too context.
			 */
			if (!dev_priv->perf.oa.exclusive_event->ctx ||
			    (dev_priv->perf.oa.specific_ctx_id == ctx_id ||
			     (dev_priv->perf.oa.specific_ctx_id !=
			      dev_priv->perf.oa.oa_buffer.last_ctx_id))) {

				if (!append_oa_sample(event, read_state,
							report))
					break;
			}
		}

		/* If append_oa_sample() returns false we shouldn't progress
		 * head so we update it afterwards... */
		dev_priv->perf.oa.oa_buffer.last_ctx_id = ctx_id;
		head += report_size;
	}

	return dev_priv->perf.oa.oa_buffer.gtt_offset + head;
}

static void gen8_oa_read(struct i915_perf_event *event,
			 struct i915_perf_read_state *read_state, u32 ts)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	u32 oastatus;
	u32 head;
	u32 tail;

	WARN_ON(!dev_priv->perf.oa.oa_buffer.addr);

	head = I915_READ(GEN8_OAHEADPTR);
	tail = I915_READ(GEN8_OATAILPTR);
	oastatus = I915_READ(GEN8_OASTATUS);

	if (unlikely(oastatus & (GEN8_OASTATUS_OABUFFER_OVERFLOW |
				 GEN8_OASTATUS_REPORT_LOST))) {

		if (oastatus & GEN8_OASTATUS_OABUFFER_OVERFLOW) {
			if (append_oa_status(event, read_state,
				DRM_I915_PERF_RECORD_OA_BUFFER_OVERFLOW))
				oastatus &= ~GEN8_OASTATUS_OABUFFER_OVERFLOW;
		}

		if (oastatus & GEN8_OASTATUS_REPORT_LOST) {
			if (append_oa_status(event, read_state,
				DRM_I915_PERF_RECORD_OA_REPORT_LOST))
				oastatus &= ~GEN8_OASTATUS_REPORT_LOST;
		}

		I915_WRITE(GEN8_OASTATUS, oastatus);
	}

	head = gen8_append_oa_reports(event, read_state, head, tail, ts);

	I915_WRITE(GEN8_OAHEADPTR, head);
}

static u32 gen7_append_oa_reports(struct i915_perf_event *event,
				  struct i915_perf_read_state *read_state,
				  u32 head,
				  u32 tail, u32 ts)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	int report_size = dev_priv->perf.oa.oa_buffer.format_size;
	u8 *oa_buf_base = dev_priv->perf.oa.oa_buffer.addr;
	u32 mask = (OA_BUFFER_SIZE - 1);
	u8 *report;
	u32 report_ts, taken;

	head -= dev_priv->perf.oa.oa_buffer.gtt_offset;
	tail -= dev_priv->perf.oa.oa_buffer.gtt_offset;

	/* Note: the gpu doesn't wrap the tail according to the OA buffer size
	 * so when we need to make sure our head/tail values are in-bounds we
	 * use the above mask.
	 */

	while ((taken = OA_TAKEN(tail, head))) {
		/* The tail increases in 64 byte increments, not in
		 * format_size steps. */
		if (taken < report_size)
			break;

		report = oa_buf_base + (head & mask);

		report_ts = *(u32 *)(report + 4);
		if (report_ts > ts)
			break;

		if (dev_priv->perf.oa.exclusive_event->enabled) {
			if (!append_oa_sample(event, read_state, report))
				break;
		}

		/* If append_oa_sample() returns false we shouldn't progress
		 * head so we update it afterwards... */
		head += report_size;
	}

	return dev_priv->perf.oa.oa_buffer.gtt_offset + head;
}

static void gen7_oa_read(struct i915_perf_event *event,
			 struct i915_perf_read_state *read_state, u32 ts)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	u32 oastatus2;
	u32 oastatus1;
	u32 head;
	u32 tail;

	WARN_ON(!dev_priv->perf.oa.oa_buffer.addr);

	oastatus2 = I915_READ(GEN7_OASTATUS2);
	oastatus1 = I915_READ(GEN7_OASTATUS1);

	head = oastatus2 & GEN7_OASTATUS2_HEAD_MASK;
	tail = oastatus1 & GEN7_OASTATUS1_TAIL_MASK;

	if (unlikely(oastatus1 & (GEN7_OASTATUS1_OABUFFER_OVERFLOW |
				  GEN7_OASTATUS1_REPORT_LOST))) {

		if (oastatus1 & GEN7_OASTATUS1_OABUFFER_OVERFLOW) {
			if (append_oa_status(event, read_state,
				DRM_I915_PERF_RECORD_OA_BUFFER_OVERFLOW))
				oastatus1 &= ~GEN7_OASTATUS1_OABUFFER_OVERFLOW;
		}

		if (oastatus1 & GEN7_OASTATUS1_REPORT_LOST) {
			if (append_oa_status(event, read_state,
				DRM_I915_PERF_RECORD_OA_REPORT_LOST))
				oastatus1 &= ~GEN7_OASTATUS1_REPORT_LOST;
		}

		I915_WRITE(GEN7_OASTATUS1, oastatus1);
	}

	head = gen7_append_oa_reports(event, read_state, head, tail, ts);

	I915_WRITE(GEN7_OASTATUS2, (head & GEN7_OASTATUS2_HEAD_MASK) |
				    OA_MEM_SELECT_GGTT);
}

static bool i915_oa_can_read(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	return !oa_buffer_is_empty(dev_priv);
}

static int i915_oa_wait_unlocked(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	int ret;

	if (dev_priv->perf.oa.rcs_sample_mode) {
		ret = i915_oa_rcs_wait_gpu(dev_priv);
		if (ret)
			return ret;
	}

	/* Note: the oa_buffer_is_empty() condition is ok to run unlocked as it
	 * just performs mmio reads of the OA buffer head + tail pointers and
	 * it's assumed we're handling some operation that implies the event
	 * can't be destroyed until completion (such as a read()) that ensures
	 * the device + OA buffer can't disappear
	 */
	return wait_event_interruptible(dev_priv->perf.oa.poll_wq,
				!oa_buffer_is_empty(dev_priv));
}

static void i915_oa_poll_wait(struct i915_perf_event *event,
			      struct file *file,
			      poll_table *wait)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	poll_wait(file, &dev_priv->perf.oa.poll_wq, wait);
}

static void i915_oa_read(struct i915_perf_event *event,
			 struct i915_perf_read_state *read_state)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	if (dev_priv->perf.oa.rcs_sample_mode)
		oa_rcs_append_reports(event, read_state);
	else
		dev_priv->perf.oa.ops.read(event, read_state, U32_MAX);
}

static void
free_gen_buffer(struct drm_i915_private *i915)
{
	mutex_lock(&i915->dev->struct_mutex);

	vunmap(i915->perf.generic.buffer.addr);
	i915_gem_object_ggtt_unpin(i915->perf.generic.buffer.obj);
	drm_gem_object_unreference(&i915->perf.generic.buffer.obj->base);

	i915->perf.generic.buffer.obj = NULL;
	i915->perf.generic.buffer.gtt_offset = 0;
	i915->perf.generic.buffer.vma = NULL;
	i915->perf.generic.buffer.addr = NULL;

	mutex_unlock(&i915->dev->struct_mutex);
}

static void
free_oa_buffer(struct drm_i915_private *i915)
{
	mutex_lock(&i915->dev->struct_mutex);

	vunmap(i915->perf.oa.oa_buffer.addr);
	i915_gem_object_ggtt_unpin(i915->perf.oa.oa_buffer.obj);
	drm_gem_object_unreference(&i915->perf.oa.oa_buffer.obj->base);

	i915->perf.oa.oa_buffer.obj = NULL;
	i915->perf.oa.oa_buffer.gtt_offset = 0;
	i915->perf.oa.oa_buffer.addr = NULL;

	if (i915->perf.oa.oa_rcs_buffer.obj) {
		vunmap(i915->perf.oa.oa_rcs_buffer.addr);
		i915_gem_object_ggtt_unpin(i915->perf.oa.oa_rcs_buffer.obj);
		drm_gem_object_unreference(
			&i915->perf.oa.oa_rcs_buffer.obj->base);

		i915->perf.oa.oa_rcs_buffer.obj = NULL;
		i915->perf.oa.oa_rcs_buffer.gtt_offset = 0;
		i915->perf.oa.oa_rcs_buffer.vma = NULL;
		i915->perf.oa.oa_rcs_buffer.addr = NULL;
	}

	mutex_unlock(&i915->dev->struct_mutex);
}

static void i915_oa_event_destroy(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	BUG_ON(event != dev_priv->perf.oa.exclusive_event);

	dev_priv->perf.oa.ops.disable_metric_set(dev_priv);

	free_oa_buffer(dev_priv);

	intel_uncore_forcewake_put(dev_priv, FORCEWAKE_ALL);
	intel_runtime_pm_put(dev_priv);

	dev_priv->perf.oa.exclusive_event = NULL;
}

static void *vmap_oa_buffer(struct drm_i915_gem_object *obj)
{
	int i;
	void *addr = NULL;
	struct sg_page_iter sg_iter;
	struct page **pages;

	pages = drm_malloc_ab(obj->base.size >> PAGE_SHIFT, sizeof(*pages));
	if (pages == NULL) {
		DRM_DEBUG_DRIVER("Failed to get space for pages\n");
		goto finish;
	}

	i = 0;
	for_each_sg_page(obj->pages->sgl, &sg_iter, obj->pages->nents, 0) {
		pages[i] = sg_page_iter_page(&sg_iter);
		i++;
	}

	addr = vmap(pages, i, 0, PAGE_KERNEL_NOCACHE);
	if (addr == NULL) {
		DRM_DEBUG_DRIVER("Failed to vmap pages\n");
		goto finish;
	}

finish:
	if (pages)
		drm_free_large(pages);
	return addr;
}

static void gen7_init_oa_buffer(struct drm_i915_private *dev_priv)
{
	/* Pre-DevBDW: OABUFFER must be set with counters off,
	 * before OASTATUS1, but after OASTATUS2 */
	I915_WRITE(GEN7_OASTATUS2, dev_priv->perf.oa.oa_buffer.gtt_offset |
		   OA_MEM_SELECT_GGTT); /* head */
	I915_WRITE(GEN7_OABUFFER, dev_priv->perf.oa.oa_buffer.gtt_offset);
	I915_WRITE(GEN7_OASTATUS1, dev_priv->perf.oa.oa_buffer.gtt_offset |
		   OABUFFER_SIZE_16M); /* tail */
}

static void gen8_init_oa_buffer(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN8_OAHEADPTR,
		   dev_priv->perf.oa.oa_buffer.gtt_offset);
	/* PRM says:
	 *
	 *  "This MMIO must be set before the OATAILPTR
	 *  register and after the OAHEADPTR register. This is
	 *  to enable proper functionality of the overflow
	 *  bit."
	 */
	I915_WRITE(GEN8_OABUFFER, dev_priv->perf.oa.oa_buffer.gtt_offset |
		   OABUFFER_SIZE_16M | OA_MEM_SELECT_GGTT);
	I915_WRITE(GEN8_OATAILPTR,
		   dev_priv->perf.oa.oa_buffer.gtt_offset);
}

static int alloc_obj(struct drm_i915_private *dev_priv,
				struct drm_i915_gem_object **obj)
{
	struct drm_i915_gem_object *bo;
	int ret;

	intel_runtime_pm_get(dev_priv);

	ret = i915_mutex_lock_interruptible(dev_priv->dev);
	if (ret)
		goto out;

	bo = i915_gem_alloc_object(dev_priv->dev, OA_BUFFER_SIZE);
	if (bo == NULL) {
		DRM_ERROR("Failed to allocate OA buffer\n");
		ret = -ENOMEM;
		goto unlock;
	}
	ret = i915_gem_object_set_cache_level(bo, I915_CACHE_LLC);
	if (ret)
		goto err_unref;

	/* PreHSW required 512K alignment, HSW requires 16M */
	ret = i915_gem_obj_ggtt_pin(bo, SZ_16M, 0);
	if (ret)
		goto err_unref;

	*obj = bo;
	goto unlock;

err_unref:
	drm_gem_object_unreference(&bo->base);
unlock:
	mutex_unlock(&dev_priv->dev->struct_mutex);
out:
	intel_runtime_pm_put(dev_priv);
	return ret;
}

static int alloc_oa_buffer(struct drm_i915_private *dev_priv)
{
	struct drm_i915_gem_object *bo;
	int ret;

	BUG_ON(dev_priv->perf.oa.oa_buffer.obj);

	ret = alloc_obj(dev_priv, &bo);
	if (ret)
		return ret;

	dev_priv->perf.oa.oa_buffer.obj = bo;

	dev_priv->perf.oa.oa_buffer.gtt_offset = i915_gem_obj_ggtt_offset(bo);
	dev_priv->perf.oa.oa_buffer.addr = vmap_oa_buffer(bo);

	dev_priv->perf.oa.ops.init_oa_buffer(dev_priv);

	DRM_DEBUG_DRIVER("OA Buffer initialized, gtt offset = 0x%x, vaddr = %p",
			 dev_priv->perf.oa.oa_buffer.gtt_offset,
			 dev_priv->perf.oa.oa_buffer.addr);

	return 0;
}

static int alloc_oa_rcs_buffer(struct drm_i915_private *dev_priv)
{
	struct drm_i915_gem_object *bo;
	int ret;

	BUG_ON(dev_priv->perf.oa.oa_rcs_buffer.obj);

	ret = alloc_obj(dev_priv, &bo);
	if (ret)
		return ret;

	dev_priv->perf.oa.oa_rcs_buffer.obj = bo;

	dev_priv->perf.oa.oa_rcs_buffer.gtt_offset =
				i915_gem_obj_ggtt_offset(bo);
	dev_priv->perf.oa.oa_rcs_buffer.vma = i915_gem_obj_to_ggtt(bo);
	dev_priv->perf.oa.oa_rcs_buffer.addr = vmap_oa_buffer(bo);
	INIT_LIST_HEAD(&dev_priv->perf.oa.rcs_node_list);
	dev_priv->perf.oa.oa_rcs_buffer.node_count = bo->base.size /
				dev_priv->perf.oa.oa_rcs_buffer.format_size;

	DRM_DEBUG_DRIVER(
		"OA RCS Buffer initialized, gtt offset = 0x%x, vaddr = %p",
		 dev_priv->perf.oa.oa_rcs_buffer.gtt_offset,
		 dev_priv->perf.oa.oa_rcs_buffer.addr);

	return 0;
}

static int alloc_gen_buffer(struct drm_i915_private *dev_priv)
{
	struct drm_i915_gem_object *bo;
	int ret;

	BUG_ON(dev_priv->perf.generic.buffer.obj);

	ret = alloc_obj(dev_priv, &bo);
	if (ret)
		return ret;

	dev_priv->perf.generic.buffer.obj = bo;

	dev_priv->perf.generic.buffer.gtt_offset = i915_gem_obj_ggtt_offset(bo);
	dev_priv->perf.generic.buffer.vma = i915_gem_obj_to_ggtt(bo);
	dev_priv->perf.generic.buffer.addr = vmap_oa_buffer(bo);
	INIT_LIST_HEAD(&dev_priv->perf.generic.node_list_generic);
	dev_priv->perf.generic.buffer.node_count = bo->base.size /
					I915_PERF_GEN_NODE_SIZE;

	DRM_DEBUG_DRIVER(
		"Gen Buffer initialized, gtt offset = 0x%x, vaddr = %p",
			 dev_priv->perf.generic.buffer.gtt_offset,
			 dev_priv->perf.generic.buffer.addr);

	return 0;
}

static void config_oa_regs(struct drm_i915_private *dev_priv,
			   const struct i915_oa_reg *regs,
			   int n_regs)
{
	int i;

	for (i = 0; i < n_regs; i++) {
		const struct i915_oa_reg *reg = regs + i;

		I915_WRITE(_MMIO(reg->addr), reg->value);
	}
}

static void hsw_enable_metric_set(struct drm_i915_private *dev_priv)
{
	dev_priv->perf.oa.mux_regs = NULL;
	dev_priv->perf.oa.mux_regs_len = 0;
	dev_priv->perf.oa.flex_regs = NULL;
	dev_priv->perf.oa.flex_regs_len = 0;
	dev_priv->perf.oa.b_counter_regs = NULL;
	dev_priv->perf.oa.b_counter_regs_len = 0;

	I915_WRITE(GDT_CHICKEN_BITS, GT_NOA_ENABLE);

	/* PRM:
	 *
	 * OA unit is using “crclk” for its functionality. When trunk
	 * level clock gating takes place, OA clock would be gated,
	 * unable to count the events from non-render clock domain.
	 * Render clock gating must be disabled when OA is enabled to
	 * count the events from non-render domain. Unit level clock
	 * gating for RCS should also be disabled.
	 */
	I915_WRITE(GEN7_MISCCPCTL, (I915_READ(GEN7_MISCCPCTL) &
				    ~GEN7_DOP_CLOCK_GATE_ENABLE));
	I915_WRITE(GEN6_UCGCTL1, (I915_READ(GEN6_UCGCTL1) |
				  GEN6_CSUNIT_CLOCK_GATE_DISABLE));

	switch (dev_priv->perf.oa.metrics_set) {
	case I915_OA_METRICS_SET_3D:
		config_oa_regs(dev_priv, i915_oa_3d_mux_config_hsw,
			       i915_oa_3d_mux_config_hsw_len);
		config_oa_regs(dev_priv, i915_oa_3d_b_counter_config_hsw,
			       i915_oa_3d_b_counter_config_hsw_len);
		break;
	case I915_OA_METRICS_SET_COMPUTE:
		config_oa_regs(dev_priv, i915_oa_compute_mux_config_hsw,
			       i915_oa_compute_mux_config_hsw_len);
		config_oa_regs(dev_priv, i915_oa_compute_b_counter_config_hsw,
			       i915_oa_compute_b_counter_config_hsw_len);
		break;
	case I915_OA_METRICS_SET_COMPUTE_EXTENDED:
		config_oa_regs(dev_priv,
				i915_oa_compute_extended_mux_config_hsw,
				i915_oa_compute_extended_mux_config_hsw_len);
		config_oa_regs(dev_priv,
				i915_oa_compute_extended_b_counter_config_hsw,
			i915_oa_compute_extended_b_counter_config_hsw_len);
		break;
	case I915_OA_METRICS_SET_MEMORY_READS:
		config_oa_regs(dev_priv, i915_oa_memory_reads_mux_config_hsw,
			       i915_oa_memory_reads_mux_config_hsw_len);
		config_oa_regs(dev_priv,
				i915_oa_memory_reads_b_counter_config_hsw,
				i915_oa_memory_reads_b_counter_config_hsw_len);
		break;
	case I915_OA_METRICS_SET_MEMORY_WRITES:
		config_oa_regs(dev_priv, i915_oa_memory_writes_mux_config_hsw,
			       i915_oa_memory_writes_mux_config_hsw_len);
		config_oa_regs(dev_priv,
				i915_oa_memory_writes_b_counter_config_hsw,
				i915_oa_memory_writes_b_counter_config_hsw_len);
		break;
	case I915_OA_METRICS_SET_SAMPLER_BALANCE:
		config_oa_regs(dev_priv, i915_oa_sampler_balance_mux_config_hsw,
			       i915_oa_sampler_balance_mux_config_hsw_len);
		config_oa_regs(dev_priv,
				i915_oa_sampler_balance_b_counter_config_hsw,
			i915_oa_sampler_balance_b_counter_config_hsw_len);
		break;
	default:
		BUG();
	}
}

static void hsw_disable_metric_set(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN6_UCGCTL1, (I915_READ(GEN6_UCGCTL1) &
				  ~GEN6_CSUNIT_CLOCK_GATE_DISABLE));
	I915_WRITE(GEN7_MISCCPCTL, (I915_READ(GEN7_MISCCPCTL) |
				    GEN7_DOP_CLOCK_GATE_ENABLE));

	I915_WRITE(GDT_CHICKEN_BITS, (I915_READ(GDT_CHICKEN_BITS) &
				      ~GT_NOA_ENABLE));
}

/* Manages updating the per-context aspects of the OA event
 * configuration across all contexts.
 *
 * The awkward consideration here is that OACTXCONTROL controls the
 * exponent for periodic sampling which is primarily used for system
 * wide profiling where we'd like a consistent sampling period even in
 * the face of context switches.
 *
 * Our approach of updating the register state context (as opposed to
 * say using a workaround batch buffer) ensures that the hardware
 * won't automatically reload an out-of-date timer exponent even
 * transiently before a WA BB could be parsed.
 */
static int configure_all_contexts(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = dev_priv->dev;
	struct intel_context *ctx;
	struct intel_engine_cs *engine;
	int ring_id;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	list_for_each_entry(ctx, &dev_priv->context_list, link) {

		for_each_engine(engine, dev_priv, ring_id) {
			/* The actual update of the register state context
			 * will happen the next time this logical ring
			 * is submitted. (See i915_oa_update_reg_state()
			 * which hooks into execlists_update_context())
			 */
			atomic_set(&engine->oa_state_dirty, true);
		}
	}

	mutex_unlock(&dev->struct_mutex);

	/* Now update the current context.
	 *
	 * Note: Using MMIO to update per-context registers requires
	 * some extra care...
	 */
	ret = intel_uncore_begin_ctx_mmio(dev_priv);
	if (ret) {
		DRM_ERROR(
			"Failed to bring RCS out of idle to update current ctx OA state");
		return ret;
	}

	I915_WRITE(GEN8_OACTXCONTROL, ((dev_priv->perf.oa.period_exponent <<
					GEN8_OA_TIMER_PERIOD_SHIFT) |
				      (dev_priv->perf.oa.periodic ?
				       GEN8_OA_TIMER_ENABLE : 0) |
				      GEN8_OA_COUNTER_RESUME));

	config_oa_regs(dev_priv, dev_priv->perf.oa.flex_regs,
			dev_priv->perf.oa.flex_regs_len);

	intel_uncore_end_ctx_mmio(dev_priv);

	return 0;
}

static void bdw_enable_metric_set(struct drm_i915_private *dev_priv)
{
	dev_priv->perf.oa.mux_regs = NULL;
	dev_priv->perf.oa.mux_regs_len = 0;
	dev_priv->perf.oa.b_counter_regs = NULL;
	dev_priv->perf.oa.b_counter_regs_len = 0;
	dev_priv->perf.oa.flex_regs = NULL;
	dev_priv->perf.oa.flex_regs_len = 0;

	switch (dev_priv->perf.oa.metrics_set) {
	case I915_OA_METRICS_SET_3D:
		/*
		 * XXX: double check how VPG's availability check + prioritised
		 * configs should be selected.
		 * It looks like we could fail both of the mux config conditions
		 */
		if (INTEL_INFO(dev_priv)->slice_mask & 0x1) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_3d_mux_config_1_0_slice_mask_0x01_bdw;
			dev_priv->perf.oa.mux_regs_len =
			i915_oa_3d_mux_config_1_0_slice_mask_0x01_bdw_len;
		} else if (INTEL_INFO(dev_priv)->slice_mask & 0x2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_3d_mux_config_1_1_slice_mask_0x02_bdw;
			dev_priv->perf.oa.mux_regs_len =
			i915_oa_3d_mux_config_1_1_slice_mask_0x02_bdw_len;
		}

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_3d_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_3d_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs = i915_oa_3d_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_3d_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE:
		if (INTEL_INFO(dev_priv)->slice_mask & 0x01) {
			dev_priv->perf.oa.mux_regs =
			i915_oa_compute_mux_config_1_0_slice_mask_0x01_bdw;
			dev_priv->perf.oa.mux_regs_len =
			i915_oa_compute_mux_config_1_0_slice_mask_0x01_bdw_len;
		} else if (INTEL_INFO(dev_priv)->slice_mask & 0x02) {
			dev_priv->perf.oa.mux_regs =
			i915_oa_compute_mux_config_1_2_slice_mask_0x02_bdw;
			dev_priv->perf.oa.mux_regs_len =
			i915_oa_compute_mux_config_1_2_slice_mask_0x02_bdw_len;
		}
		dev_priv->perf.oa.b_counter_regs =
			i915_oa_compute_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_compute_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs =
			i915_oa_compute_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_compute_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_RENDER_PIPE_PROFILE:
		dev_priv->perf.oa.mux_regs =
			i915_oa_render_pipe_profile_mux_config_bdw;
		dev_priv->perf.oa.mux_regs_len =
			i915_oa_render_pipe_profile_mux_config_bdw_len;

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_render_pipe_profile_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_render_pipe_profile_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs =
			i915_oa_render_pipe_profile_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_render_pipe_profile_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_MEMORY_READS:
		dev_priv->perf.oa.mux_regs =
			i915_oa_memory_reads_mux_config_bdw;
		dev_priv->perf.oa.mux_regs_len =
			i915_oa_memory_reads_mux_config_bdw_len;

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_memory_reads_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_memory_reads_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs =
			i915_oa_memory_reads_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_memory_reads_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_MEMORY_WRITES:
		dev_priv->perf.oa.mux_regs =
			i915_oa_memory_writes_mux_config_bdw;
		dev_priv->perf.oa.mux_regs_len =
			i915_oa_memory_writes_mux_config_bdw_len;

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_memory_writes_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_memory_writes_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs =
			i915_oa_memory_writes_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_memory_writes_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE_EXTENDED:
		if (INTEL_INFO(dev_priv)->subslice_mask & 0x01) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_0_subslice_mask_0x01_bdw;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_0_subslice_mask_0x01_bdw_len;
		} else if (INTEL_INFO(dev_priv)->subslice_mask & 0x08) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_1_subslice_mask_0x08_bdw;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_1_subslice_mask_0x08_bdw_len;
		} else if (INTEL_INFO(dev_priv)->subslice_mask & 0x02) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_2_subslice_mask_0x02_bdw;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_2_subslice_mask_0x02_bdw_len;
		} else if (INTEL_INFO(dev_priv)->subslice_mask & 0x10) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_3_subslice_mask_0x10_bdw;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_3_subslice_mask_0x10_bdw_len;
		} else if (INTEL_INFO(dev_priv)->subslice_mask & 0x04) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_4_subslice_mask_0x04_bdw;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_4_subslice_mask_0x04_bdw_len;
		} else if (INTEL_INFO(dev_priv)->subslice_mask & 0x20) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_5_subslice_mask_0x20_bdw;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_5_subslice_mask_0x20_bdw_len;
		}
		dev_priv->perf.oa.mux_regs =
			i915_oa_compute_extended_mux_config_bdw;
		dev_priv->perf.oa.mux_regs_len =
			i915_oa_compute_extended_mux_config_bdw_len;

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_compute_extended_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_compute_extended_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs =
			i915_oa_compute_extended_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_compute_extended_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE_L3_CACHE:
		dev_priv->perf.oa.mux_regs =
			i915_oa_compute_l3_cache_mux_config_bdw;
		dev_priv->perf.oa.mux_regs_len =
			i915_oa_compute_l3_cache_mux_config_bdw_len;

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_compute_l3_cache_b_counter_config_bdw;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_compute_l3_cache_b_counter_config_bdw_len;

		dev_priv->perf.oa.flex_regs =
			i915_oa_compute_l3_cache_flex_eu_config_bdw;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_compute_l3_cache_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_DATA_PORT_READS_COALESCING:
		/*
		 * XXX: DATA_PORT_READS_COALESCING: what if the
		 * subslice_mask & 0x1 test fails?
		 */
		if (INTEL_INFO(dev_priv)->subslice_mask & 0x01) {
                        dev_priv->perf.oa.mux_regs =
                                i915_oa_data_port_reads_coalescing_mux_config_1_0_subslice_mask_0x01_bdw;
                        dev_priv->perf.oa.mux_regs_len =
                                i915_oa_data_port_reads_coalescing_mux_config_1_0_subslice_mask_0x01_bdw_len;
                } else
			DRM_DEBUG_DRIVER("undefined MUX config for DATA_PORT_READS_COALESCING");

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_data_port_reads_coalescing_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_data_port_reads_coalescing_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_data_port_reads_coalescing_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_data_port_reads_coalescing_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_DATA_PORT_WRITES_COALESCING:
                if (INTEL_INFO(dev_priv)->subslice_mask & 0x01) {
                        dev_priv->perf.oa.mux_regs =
                                i915_oa_data_port_writes_coalescing_mux_config_1_0_subslice_mask_0x01_bdw;
                        dev_priv->perf.oa.mux_regs_len =
                                i915_oa_data_port_writes_coalescing_mux_config_1_0_subslice_mask_0x01_bdw_len;
                } else
			DRM_DEBUG_DRIVER("undefined MUX config for DATA_PORT_WRITES_COALESCING");

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_data_port_writes_coalescing_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_data_port_writes_coalescing_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_data_port_writes_coalescing_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_data_port_writes_coalescing_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_L3_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_1_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_1_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_1_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_1_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_1_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_1_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_L3_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_2_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_2_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_2_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_2_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_2_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_2_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_L3_3:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_3_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_3_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_3_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_3_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_3_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_3_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_L3_4:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_4_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_4_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_4_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_4_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_4_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_4_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_RASTERIZER_AND_PIXEL_BACKEND:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_rasterizer_and_pixel_backend_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_rasterizer_and_pixel_backend_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_rasterizer_and_pixel_backend_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_SAMPLER_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_sampler_1_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_sampler_1_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_sampler_1_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_sampler_1_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_sampler_1_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_sampler_1_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_SAMPLER_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_sampler_2_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_sampler_2_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_sampler_2_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_sampler_2_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_sampler_2_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_sampler_2_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_TDL_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_tdl_1_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_tdl_1_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_tdl_1_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_tdl_1_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_tdl_1_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_tdl_1_flex_eu_config_bdw_len;
		break;

	case I915_OA_METRICS_SET_TDL_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_tdl_2_mux_config_bdw;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_tdl_2_mux_config_bdw_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_tdl_2_b_counter_config_bdw;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_tdl_2_b_counter_config_bdw_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_tdl_2_flex_eu_config_bdw;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_tdl_2_flex_eu_config_bdw_len;
		break;

	default:
		BUG(); /* should have been validated in _init */
		return;
	}

	I915_WRITE(GDT_CHICKEN_BITS, 0xA0);
	config_oa_regs(dev_priv, dev_priv->perf.oa.mux_regs,
		       dev_priv->perf.oa.mux_regs_len);
	I915_WRITE(GDT_CHICKEN_BITS, 0x80);
	config_oa_regs(dev_priv, dev_priv->perf.oa.b_counter_regs,
		       dev_priv->perf.oa.b_counter_regs_len);

	configure_all_contexts(dev_priv);
}

static void bdw_disable_metric_set(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN6_UCGCTL1, (I915_READ(GEN6_UCGCTL1) &
				  ~GEN6_CSUNIT_CLOCK_GATE_DISABLE));
	I915_WRITE(GEN7_MISCCPCTL, (I915_READ(GEN7_MISCCPCTL) |
				    GEN7_DOP_CLOCK_GATE_ENABLE));
	/*
	 * XXX: Do we need to write to CHICKEN2 to disable DOP clock gating
	 * when idle?
	 */
}

static void chv_enable_metric_set(struct drm_i915_private *dev_priv)
{
	dev_priv->perf.oa.mux_regs = NULL;
	dev_priv->perf.oa.mux_regs_len = 0;
	dev_priv->perf.oa.flex_regs = NULL;
	dev_priv->perf.oa.flex_regs_len = 0;
	dev_priv->perf.oa.b_counter_regs = NULL;
	dev_priv->perf.oa.b_counter_regs_len = 0;

	switch (dev_priv->perf.oa.metrics_set) {
	case I915_OA_METRICS_SET_3D:
		dev_priv->perf.oa.mux_regs = i915_oa_3d_mux_config_chv;
		dev_priv->perf.oa.mux_regs_len = i915_oa_3d_mux_config_chv_len;

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_3d_b_counter_config_chv;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_3d_b_counter_config_chv_len;

		dev_priv->perf.oa.flex_regs = i915_oa_3d_flex_eu_config_chv;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_3d_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_compute_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_compute_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_compute_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_compute_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_compute_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_compute_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_RENDER_PIPE_PROFILE:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_render_pipe_profile_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_render_pipe_profile_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_render_pipe_profile_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_render_pipe_profile_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_render_pipe_profile_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_render_pipe_profile_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_HDC_AND_SF:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_hdc_and_sf_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_hdc_and_sf_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_hdc_and_sf_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_hdc_and_sf_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_hdc_and_sf_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_hdc_and_sf_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_L3_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_1_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_1_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_1_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_1_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_1_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_1_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_L3_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_2_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_2_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_2_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_2_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_2_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_2_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_L3_3:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_3_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_3_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_3_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_3_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_3_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_3_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_L3_4:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_4_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_4_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_4_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_4_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_4_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_4_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_RASTERIZER_AND_PIXEL_BACKEND:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_rasterizer_and_pixel_backend_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_rasterizer_and_pixel_backend_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_rasterizer_and_pixel_backend_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_SAMPLER_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_sampler_1_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_sampler_1_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_sampler_1_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_sampler_1_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_sampler_1_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_sampler_1_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_SAMPLER_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_sampler_2_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_sampler_2_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_sampler_2_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_sampler_2_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_sampler_2_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_sampler_2_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_TDL_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_tdl_1_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_tdl_1_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_tdl_1_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_tdl_1_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_tdl_1_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_tdl_1_flex_eu_config_chv_len;
		break;

	case I915_OA_METRICS_SET_TDL_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_tdl_2_mux_config_chv;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_tdl_2_mux_config_chv_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_tdl_2_b_counter_config_chv;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_tdl_2_b_counter_config_chv_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_tdl_2_flex_eu_config_chv;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_tdl_2_flex_eu_config_chv_len;
		break;

	default:
		BUG(); /* should have been validated in _init */
		return;
	}

	I915_WRITE(GDT_CHICKEN_BITS, 0xA0);
	config_oa_regs(dev_priv, dev_priv->perf.oa.mux_regs,
		       dev_priv->perf.oa.mux_regs_len);
	I915_WRITE(GDT_CHICKEN_BITS, 0x80);
	config_oa_regs(dev_priv, dev_priv->perf.oa.b_counter_regs,
		       dev_priv->perf.oa.b_counter_regs_len);

	configure_all_contexts(dev_priv);
}

static void chv_disable_metric_set(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN6_UCGCTL1, (I915_READ(GEN6_UCGCTL1) &
				  ~GEN6_CSUNIT_CLOCK_GATE_DISABLE));
	I915_WRITE(GEN7_MISCCPCTL, (I915_READ(GEN7_MISCCPCTL) |
				    GEN7_DOP_CLOCK_GATE_ENABLE));
	/*
	 * XXX: Do we need to write to CHICKEN2 to disable DOP clock gating
	 * when idle?
	 */
}

static void skl_enable_metric_set(struct drm_i915_private *dev_priv)
{
	dev_priv->perf.oa.mux_regs = NULL;
	dev_priv->perf.oa.mux_regs_len = 0;
	dev_priv->perf.oa.b_counter_regs = NULL;
	dev_priv->perf.oa.b_counter_regs_len = 0;
	dev_priv->perf.oa.flex_regs = NULL;
	dev_priv->perf.oa.flex_regs_len = 0;

	switch (dev_priv->perf.oa.metrics_set) {
	case I915_OA_METRICS_SET_3D:
		if (dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_3d_mux_config_1_1_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_3d_mux_config_1_1_sku_lt_0x02_skl_len;
		} else {
			dev_priv->perf.oa.mux_regs =
				i915_oa_3d_mux_config_1_1_sku_gte_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_3d_mux_config_1_1_sku_gte_0x02_skl_len;
		}

		dev_priv->perf.oa.b_counter_regs =
			i915_oa_3d_b_counter_config_skl;
		dev_priv->perf.oa.b_counter_regs_len =
			i915_oa_3d_b_counter_config_skl_len;

		dev_priv->perf.oa.flex_regs = i915_oa_3d_flex_eu_config_skl;
		dev_priv->perf.oa.flex_regs_len =
			i915_oa_3d_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE:
                if (INTEL_INFO(dev_priv)->slice_mask & 0x01 &&
		    dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_mux_config_1_0_slice_mask_0x01_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_mux_config_1_0_slice_mask_0x01_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->slice_mask & 0x01 &&
			   dev_priv->dev->pdev->revision >= 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_mux_config_1_0_slice_mask_0x01_sku_gte_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_mux_config_1_0_slice_mask_0x01_sku_gte_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->slice_mask & 0x02 &&
			   dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_mux_config_1_2_slice_mask_0x02_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_mux_config_1_2_slice_mask_0x02_sku_lt_0x02_skl_len;
                }

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_compute_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_compute_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_compute_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_compute_flex_eu_config_skl_len;

		break;

	case I915_OA_METRICS_SET_RENDER_PIPE_PROFILE:
                if (dev_priv->dev->pdev->revision < 2) {
                        dev_priv->perf.oa.mux_regs =
                                i915_oa_render_pipe_profile_mux_config_1_0_sku_lt_0x02_skl;
                        dev_priv->perf.oa.mux_regs_len =
                                i915_oa_render_pipe_profile_mux_config_1_0_sku_lt_0x02_skl_len;
                } else {
                        dev_priv->perf.oa.mux_regs =
                                i915_oa_render_pipe_profile_mux_config_1_0_sku_gte_0x02_skl;
                        dev_priv->perf.oa.mux_regs_len =
                                i915_oa_render_pipe_profile_mux_config_1_0_sku_gte_0x02_skl_len;
                }

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_render_pipe_profile_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_render_pipe_profile_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_render_pipe_profile_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_render_pipe_profile_flex_eu_config_skl_len;

		break;

	case I915_OA_METRICS_SET_MEMORY_READS:
                if (INTEL_INFO(dev_priv)->slice_mask & 0x01 &&
		    dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_memory_reads_mux_config_1_0_slice_mask_0x01_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_memory_reads_mux_config_1_0_slice_mask_0x01_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->slice_mask & 0x01 &&
			   dev_priv->dev->pdev->revision >= 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_memory_reads_mux_config_1_0_slice_mask_0x01_sku_gte_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_memory_reads_mux_config_1_0_slice_mask_0x01_sku_gte_0x02_skl_len;
                }

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_memory_reads_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_memory_reads_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_memory_reads_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_memory_reads_flex_eu_config_skl_len;

		break;

	case I915_OA_METRICS_SET_MEMORY_WRITES:
                if (INTEL_INFO(dev_priv)->slice_mask & 0x01 &&
		    dev_priv->dev->pdev->revision < 0x02) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_memory_writes_mux_config_1_0_slice_mask_0x01_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_memory_writes_mux_config_1_0_slice_mask_0x01_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->slice_mask & 0x01 &&
			   dev_priv->dev->pdev->revision >= 0x02) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_memory_writes_mux_config_1_0_slice_mask_0x01_sku_gte_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_memory_writes_mux_config_1_0_slice_mask_0x01_sku_gte_0x02_skl_len;
                }

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_memory_writes_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_memory_writes_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_memory_writes_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_memory_writes_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE_EXTENDED:
                if (INTEL_INFO(dev_priv)->subslice_mask & 0x1 &&
		    dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_0_subslice_mask_0x01_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_0_subslice_mask_0x01_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->subslice_mask & 0x08 &&
			   dev_priv->dev->pdev->revision < 0x02) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_1_subslice_mask_0x08_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_1_subslice_mask_0x08_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->subslice_mask & 0x02 &&
			   dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_2_subslice_mask_0x02_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_2_subslice_mask_0x02_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->subslice_mask & 0x10 &&
			   dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_3_subslice_mask_0x10_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_3_subslice_mask_0x10_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->subslice_mask & 0x04 &&
			   dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_4_subslice_mask_0x04_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_4_subslice_mask_0x04_sku_lt_0x02_skl_len;
                } else if (INTEL_INFO(dev_priv)->subslice_mask & 0x20 &&
			   dev_priv->dev->pdev->revision < 2) {
			dev_priv->perf.oa.mux_regs =
				i915_oa_compute_extended_mux_config_1_5_subslice_mask_0x20_sku_lt_0x02_skl;
			dev_priv->perf.oa.mux_regs_len =
				i915_oa_compute_extended_mux_config_1_5_subslice_mask_0x20_sku_lt_0x02_skl_len;
                }

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_compute_extended_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_compute_extended_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_compute_extended_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_compute_extended_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_COMPUTE_L3_CACHE:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_compute_l3_cache_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_compute_l3_cache_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_compute_l3_cache_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_compute_l3_cache_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_compute_l3_cache_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_compute_l3_cache_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_HDC_AND_SF:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_hdc_and_sf_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_hdc_and_sf_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_hdc_and_sf_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_hdc_and_sf_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_hdc_and_sf_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_hdc_and_sf_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_L3_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_1_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_1_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_1_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_1_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_1_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_1_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_L3_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_2_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_2_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_2_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_2_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_2_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_2_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_L3_3:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_l3_3_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_l3_3_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_l3_3_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_l3_3_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_l3_3_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_l3_3_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_RASTERIZER_AND_PIXEL_BACKEND:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_rasterizer_and_pixel_backend_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_rasterizer_and_pixel_backend_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_rasterizer_and_pixel_backend_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_rasterizer_and_pixel_backend_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_SAMPLER:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_sampler_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_sampler_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_sampler_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_sampler_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_sampler_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_sampler_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_TDL_1:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_tdl_1_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_tdl_1_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_tdl_1_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_tdl_1_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_tdl_1_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_tdl_1_flex_eu_config_skl_len;
		break;

	case I915_OA_METRICS_SET_TDL_2:
                dev_priv->perf.oa.mux_regs =
                        i915_oa_tdl_2_mux_config_skl;
                dev_priv->perf.oa.mux_regs_len =
                        i915_oa_tdl_2_mux_config_skl_len;

                dev_priv->perf.oa.b_counter_regs =
                        i915_oa_tdl_2_b_counter_config_skl;
                dev_priv->perf.oa.b_counter_regs_len =
                        i915_oa_tdl_2_b_counter_config_skl_len;

                dev_priv->perf.oa.flex_regs =
                        i915_oa_tdl_2_flex_eu_config_skl;
                dev_priv->perf.oa.flex_regs_len =
                        i915_oa_tdl_2_flex_eu_config_skl_len;
		break;

	default:
		BUG(); /* should have been validated in _init */
		return;
	}

	I915_WRITE(GDT_CHICKEN_BITS, 0xA0);
	config_oa_regs(dev_priv, dev_priv->perf.oa.mux_regs,
		       dev_priv->perf.oa.mux_regs_len);
	I915_WRITE(GDT_CHICKEN_BITS, 0x80);
	config_oa_regs(dev_priv, dev_priv->perf.oa.b_counter_regs,
		       dev_priv->perf.oa.b_counter_regs_len);

	configure_all_contexts(dev_priv);
}

static void skl_disable_metric_set(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN6_UCGCTL1, (I915_READ(GEN6_UCGCTL1) &
				  ~GEN6_CSUNIT_CLOCK_GATE_DISABLE));
	I915_WRITE(GEN7_MISCCPCTL, (I915_READ(GEN7_MISCCPCTL) |
				    GEN7_DOP_CLOCK_GATE_ENABLE));
	/*
	 * XXX: Do we need to write to CHICKEN2 to disable DOP clock gating
	 * when idle?
	 */
}

static void gen7_update_oacontrol(struct drm_i915_private *dev_priv)
{
	if (dev_priv->perf.oa.exclusive_event->enabled) {
		unsigned long ctx_id = 0;
		bool pinning_ok = false;

		if (dev_priv->perf.oa.exclusive_event->ctx &&
		    dev_priv->perf.oa.specific_ctx_id) {
			ctx_id = dev_priv->perf.oa.specific_ctx_id;
			pinning_ok = true;
		}

		if (dev_priv->perf.oa.exclusive_event->ctx == NULL ||
		    pinning_ok) {
			bool periodic = dev_priv->perf.oa.periodic;
			u32 period_exponent = dev_priv->perf.oa.period_exponent;
			u32 report_format = dev_priv->perf.oa.oa_buffer.format;

			I915_WRITE(GEN7_OACONTROL,
				   (ctx_id & GEN7_OACONTROL_CTX_MASK) |
				   (period_exponent <<
				    GEN7_OACONTROL_TIMER_PERIOD_SHIFT) |
				   (periodic ?
				    GEN7_OACONTROL_TIMER_ENABLE : 0) |
				   (report_format <<
				    GEN7_OACONTROL_FORMAT_SHIFT) |
				   (ctx_id ?
				    GEN7_OACONTROL_PER_CTX_ENABLE : 0) |
				   GEN7_OACONTROL_ENABLE);
			return;
		}
	}

	I915_WRITE(GEN7_OACONTROL, 0);
}

static void gen7_oa_enable(struct drm_i915_private *dev_priv)
{
	u32 oastatus1, tail;

	gen7_update_oacontrol(dev_priv);

	/* Reset the head ptr so we don't forward reports from before now. */
	oastatus1 = I915_READ(GEN7_OASTATUS1);
	tail = oastatus1 & GEN7_OASTATUS1_TAIL_MASK;
	I915_WRITE(GEN7_OASTATUS2, (tail & GEN7_OASTATUS2_HEAD_MASK) |
				    OA_MEM_SELECT_GGTT);
}

static void gen8_oa_enable(struct drm_i915_private *dev_priv)
{
	u32 report_format = dev_priv->perf.oa.oa_buffer.format;
	u32 tail;

	/* Note: we don't rely on the hardware to perform single context
	 * filtering and instead filter on the cpu based on the context-id
	 * field of reports */
	I915_WRITE(GEN8_OACONTROL, (report_format <<
				    GEN8_OA_REPORT_FORMAT_SHIFT) |
				   GEN8_OA_COUNTER_ENABLE);

	/* Reset the head ptr so we don't forward reports from before now. */
	tail = I915_READ(GEN8_OATAILPTR);
	I915_WRITE(GEN8_OAHEADPTR, tail);
}

static void i915_oa_event_enable(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	dev_priv->perf.oa.ops.oa_enable(dev_priv);

	if (dev_priv->perf.oa.rcs_sample_mode)
		event->emit_profiling_data = i915_oa_emit_report;

	if (dev_priv->perf.oa.periodic)
		hrtimer_start(&dev_priv->perf.oa.poll_check_timer,
			      ns_to_ktime(POLL_PERIOD),
			      HRTIMER_MODE_REL_PINNED);
}

static void gen7_oa_disable(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN7_OACONTROL, 0);
}

static void gen8_oa_disable(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN8_OACONTROL, 0);
}

static void i915_oa_event_disable(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	if (dev_priv->perf.oa.rcs_sample_mode) {
		event->emit_profiling_data = NULL;
		i915_oa_rcs_wait_gpu(dev_priv);
		i915_oa_rcs_free_requests(dev_priv);
	}

	dev_priv->perf.oa.ops.oa_disable(dev_priv);

	if (dev_priv->perf.oa.periodic)
		hrtimer_cancel(&dev_priv->perf.oa.poll_check_timer);
}

static int i915_oa_event_init(struct i915_perf_event *event,
			      struct drm_i915_perf_open_param *param)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	struct drm_i915_perf_oa_attr oa_attr;
	u32 known_flags = 0;
	int format_size;
	int ret;

	BUG_ON(param->type != I915_PERF_OA_EVENT);

	if (!dev_priv->perf.oa.ops.init_oa_buffer) {
		DRM_ERROR("OA unit not supported\n");
		return -ENODEV;
	}

	/* To avoid the complexity of having to accurately filter
	 * counter reports and marshal to the appropriate client
	 * we currently only allow exclusive access */
	if (dev_priv->perf.oa.exclusive_event) {
		DRM_ERROR("OA unit already in use\n");
		return -EBUSY;
	}

	ret = i915_perf_copy_attr(to_user_ptr(param->attr),
					      &oa_attr,
					      I915_OA_ATTR_SIZE_VER0,
					      sizeof(oa_attr));
	if (ret)
		return ret;


	known_flags = I915_OA_FLAG_PERIODIC;
	if (oa_attr.flags & ~known_flags) {
		DRM_ERROR("Unknown drm_i915_perf_oa_attr flag\n");
		return -EINVAL;
	}

	if (oa_attr.oa_format >= I915_OA_FORMAT_MAX) {
		DRM_ERROR("Invalid OA report format\n");
		return -EINVAL;
	}

	format_size = dev_priv->perf.oa.oa_formats[oa_attr.oa_format].size;
	if (!format_size) {
		DRM_ERROR("Invalid OA report format\n");
		return -EINVAL;
	}

	dev_priv->perf.oa.oa_buffer.format_size = format_size;

	dev_priv->perf.oa.oa_buffer.format =
		dev_priv->perf.oa.oa_formats[oa_attr.oa_format].format;

	if (IS_HASWELL(dev_priv->dev)) {
		if (oa_attr.metrics_set <= 0 ||
		    oa_attr.metrics_set > I915_OA_METRICS_SET_MAX) {
			DRM_ERROR("Metric set not available\n");
			return -EINVAL;
		}
	} else if (IS_BROADWELL(dev_priv->dev) ||
		   IS_CHERRYVIEW(dev_priv->dev) ||
		   IS_SKYLAKE(dev_priv->dev) ||
		   IS_BROXTON(dev_priv->dev)) {
		if (oa_attr.metrics_set != I915_OA_METRICS_SET_3D) {
			DRM_ERROR("Metric set not available\n");
			return -EINVAL;
		}
	} else {
		BUG(); /* checked above */
		return -ENODEV;
	}

	dev_priv->perf.oa.metrics_set = oa_attr.metrics_set;

	dev_priv->perf.oa.periodic = !!(oa_attr.flags & I915_OA_FLAG_PERIODIC);

	/* NB: The exponent represents a period as follows:
	 *
	 *   80ns * 2^(period_exponent + 1)
	 */
	if (dev_priv->perf.oa.periodic) {
		u64 period_exponent = oa_attr.oa_timer_exponent;

		if (period_exponent > OA_EXPONENT_MAX)
			return -EINVAL;

		if (period_exponent < i915_oa_event_min_timer_exponent &&
		    !capable(CAP_SYS_ADMIN)) {
			DRM_ERROR(
				"Sampling period too high without root privileges\n");
			return -EACCES;
		}

		dev_priv->perf.oa.period_exponent = period_exponent;
	} else if (oa_attr.oa_timer_exponent) {
		DRM_ERROR(
			"Sampling exponent specified without requesting periodic sampling");
		return -EINVAL;
	}

	if ((param->sample_flags & I915_PERF_SAMPLE_TIMESTAMP) ||
		(param->sample_flags & I915_PERF_SAMPLE_RING_ID) ||
		(param->sample_flags & I915_PERF_SAMPLE_MMIO)) {
		DRM_ERROR("Unsupported sample type for OA event\n");
		return -EINVAL;
	}

	if ((param->sample_flags & I915_PERF_SAMPLE_PID) ||
		(param->sample_flags & I915_PERF_SAMPLE_TAG) ||
		(IS_HASWELL(dev_priv->dev) &&
		(param->sample_flags & I915_PERF_SAMPLE_CTXID))) {
		/*
		 * We check dev.i915.perf_event_paranoid sysctl option
		 * to determine if it's ok to access system wide OA counters
		 * without CAP_SYS_ADMIN privileges.
		 */
		if (i915_perf_event_paranoid && !capable(CAP_SYS_ADMIN)) {
			DRM_ERROR(
				"Insufficient privileges to open perf event\n");
			return -EACCES;
		}

		dev_priv->perf.oa.oa_rcs_buffer.format_size = format_size;
		dev_priv->perf.oa.oa_rcs_buffer.format =
			dev_priv->perf.oa.oa_buffer.format;

		spin_lock_init(&dev_priv->perf.oa.rcs_node_list_lock);

		ret = alloc_oa_rcs_buffer(dev_priv);
		if (ret)
			return ret;
		dev_priv->perf.oa.rcs_sample_mode = true;
	} else
		dev_priv->perf.oa.rcs_sample_mode = false;

	ret = alloc_oa_buffer(dev_priv);
	if (ret)
		return ret;

	if (i915.enable_execlists && event->ctx)
		dev_priv->perf.oa.specific_ctx_id =
			intel_execlists_ctx_id(event->ctx);

	dev_priv->perf.oa.exclusive_event = event;

	/* PRM - observability performance counters:
	 *
	 *   OACONTROL, performance counter enable, note:
	 *
	 *   "When this bit is set, in order to have coherent counts,
	 *   RC6 power state and trunk clock gating must be disabled.
	 *   This can be achieved by programming MMIO registers as
	 *   0xA094=0 and 0xA090[31]=1"
	 *
	 *   In our case we are expected that taking pm + FORCEWAKE
	 *   references will effectively disable RC6.
	 */
	intel_runtime_pm_get(dev_priv);
	intel_uncore_forcewake_get(dev_priv, FORCEWAKE_ALL);

	dev_priv->perf.oa.ops.enable_metric_set(dev_priv);

	event->destroy = i915_oa_event_destroy;
	event->enable = i915_oa_event_enable;
	event->disable = i915_oa_event_disable;
	event->can_read = i915_oa_can_read;
	event->wait_unlocked = i915_oa_wait_unlocked;
	event->poll_wait = i915_oa_poll_wait;
	event->read = i915_oa_read;

	return 0;
}

static int i915_gen_wait_gpu(struct drm_i915_private *dev_priv)
{
	struct i915_perf_gen_data_node *last_entry = NULL;
	int ret;

	/*
	 * Wait for the last scheduled request to complete. This would
	 * implicitly wait for the prior submitted requests. The refcount
	 * of the requests is not decremented here.
	 */
	spin_lock(&dev_priv->perf.generic.node_list_lock_generic);

	if (!list_empty(&dev_priv->perf.generic.node_list_generic)) {
		last_entry = list_last_entry(
				&dev_priv->perf.generic.node_list_generic,
			struct i915_perf_gen_data_node, head);
	}
	spin_unlock(&dev_priv->perf.generic.node_list_lock_generic);

	if (!last_entry)
		return 0;

	ret = __i915_wait_request(last_entry->req, atomic_read(
				&dev_priv->gpu_error.reset_counter),
				true, NULL, NULL);
	if (ret) {
		DRM_ERROR("failed to wait\n");
		return ret;
	}
	return 0;
}

static void i915_gen_free_requests(struct drm_i915_private *dev_priv)
{
	struct i915_perf_gen_data_node *entry, *next;

	list_for_each_entry_safe
		(entry, next, &dev_priv->perf.generic.node_list_generic,
			head) {
		i915_gem_request_unreference(entry->req);

		spin_lock(&dev_priv->perf.generic.node_list_lock_generic);
		list_del(&entry->head);
		spin_unlock(&dev_priv->perf.generic.node_list_lock_generic);
		kfree(entry);
	}
}

static bool append_gen_sample(struct i915_perf_event *event,
			     struct i915_perf_read_state *read_state,
			     struct i915_perf_gen_data_node *node)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	struct drm_i915_perf_event_header header;
	u32 sample_flags = event->sample_flags;
	u8 *ts = dev_priv->perf.generic.buffer.addr + node->offset;

	header.type = DRM_I915_PERF_RECORD_SAMPLE;
	header.misc = 0;
	header.size = sizeof(header);

	/* XXX: could pre-compute this when opening the event... */

	if (sample_flags & I915_PERF_SAMPLE_CTXID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_PID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_TAG)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_RING_ID)
		header.size += 4;

	if (sample_flags & I915_PERF_SAMPLE_TIMESTAMP)
		header.size += 8;

	if (sample_flags & I915_PERF_SAMPLE_MMIO)
		header.size += 4*I915_GEN_PERF_MMIO_NUM;

	if ((read_state->count - read_state->read) < header.size)
		return false;

	if (copy_to_user(read_state->buf, &header, sizeof(header)))
		return false;

	read_state->buf += sizeof(header);

	if (sample_flags & I915_PERF_SAMPLE_CTXID) {
		if (copy_to_user(read_state->buf, &node->ctx_id, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_PID) {
		if (copy_to_user(read_state->buf, &node->pid, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_TAG) {
		if (copy_to_user(read_state->buf, &node->tag, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_RING_ID) {
		if (copy_to_user(read_state->buf, &node->ring_id, 4))
			return false;
		read_state->buf += 4;
	}

	if (sample_flags & I915_PERF_SAMPLE_TIMESTAMP) {
		if (copy_to_user(read_state->buf, ts, 8))
			return false;
		read_state->buf += 8;
	}

	if (sample_flags & I915_PERF_SAMPLE_MMIO) {
		u8 *mmio_data = ts + 8;

		if (copy_to_user(read_state->buf, mmio_data,
					4*I915_GEN_PERF_MMIO_NUM))
			return false;
		read_state->buf += 4*I915_GEN_PERF_MMIO_NUM;
	}

	read_state->read += header.size;

	return true;
}

static bool gen_buffer_is_empty(struct drm_i915_private *dev_priv)
{
	return list_empty(&dev_priv->perf.generic.node_list_generic);
}

static void i915_gen_event_destroy(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	BUG_ON(event != dev_priv->perf.generic.exclusive_event);

	free_gen_buffer(dev_priv);

	dev_priv->perf.generic.exclusive_event = NULL;
}

static void i915_gen_event_enable(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	event->emit_profiling_data = i915_gen_emit_ts_data;

	hrtimer_start(&dev_priv->perf.generic.poll_check_timer,
			ns_to_ktime(POLL_PERIOD), HRTIMER_MODE_REL_PINNED);
}

static void i915_gen_event_disable(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	event->emit_profiling_data = NULL;
	i915_gen_wait_gpu(dev_priv);
	i915_gen_free_requests(dev_priv);

	hrtimer_cancel(&dev_priv->perf.generic.poll_check_timer);
}

static bool i915_gen_can_read(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	return !gen_buffer_is_empty(dev_priv);
}

static int i915_gen_wait_unlocked(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	int ret;

	ret = i915_gen_wait_gpu(dev_priv);
	if (ret)
		return ret;

	/* Note: the gen_buffer_is_empty() condition is ok to run unlocked as
	 * it's assumed we're handling some operation that implies the event
	 * can't be destroyed until completion (such as a read()) that ensures
	 * the device + buffer can't disappear
	 */
	return wait_event_interruptible(dev_priv->perf.generic.poll_wq,
					!gen_buffer_is_empty(dev_priv));
}

static void i915_gen_poll_wait(struct i915_perf_event *event,
			      struct file *file,
			      poll_table *wait)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	poll_wait(file, &dev_priv->perf.generic.poll_wq, wait);
}

static void i915_gen_read(struct i915_perf_event *event,
			 struct i915_perf_read_state *read_state)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	struct i915_perf_gen_data_node *entry, *next;

	if (!dev_priv->perf.generic.exclusive_event->enabled)
		return;

	list_for_each_entry_safe
		(entry, next, &dev_priv->perf.generic.node_list_generic,
			head) {
		if (!i915_gem_request_completed(entry->req))
			break;

		if (!append_gen_sample(event, read_state, entry))
			break;

		spin_lock(&dev_priv->perf.generic.node_list_lock_generic);
		list_del(&entry->head);
		spin_unlock(&dev_priv->perf.generic.node_list_lock_generic);

		i915_gem_request_unreference(entry->req);
		kfree(entry);
	}
}

static enum hrtimer_restart gen_poll_check_timer_cb(struct hrtimer *hrtimer)
{
	struct drm_i915_private *dev_priv =
		container_of(hrtimer, typeof(*dev_priv),
			     perf.generic.poll_check_timer);

	if (!gen_buffer_is_empty(dev_priv))
		wake_up(&dev_priv->perf.generic.poll_wq);

	hrtimer_forward_now(hrtimer, ns_to_ktime(POLL_PERIOD));

	return HRTIMER_RESTART;
}

#define GEN_RANGE(l, h) GENMASK(h, l)

static const struct register_whitelist {
	i915_reg_t offset;
	uint32_t size;
	/* supported gens, 0x10 for 4, 0x30 for 4 and 5, etc. */
	uint32_t gen_bitmask;
} whitelist[] = {
	{ GEN6_GT_GFX_RC6, 4, GEN_RANGE(7, 9) },
	{ GEN6_GT_GFX_RC6p, 4, GEN_RANGE(7, 9) },
};

static int check_mmio_whitelist(struct drm_i915_private *dev_priv,
				struct drm_i915_perf_gen_attr *gen_attr)
{
	struct register_whitelist const *entry = whitelist;
	int i, count;

	for (count = 0; count < I915_GEN_PERF_MMIO_NUM; count++) {
		if (!gen_attr->mmio_list[count])
			break;

		for (i = 0; i < ARRAY_SIZE(whitelist); i++, entry++) {
			if (i915_mmio_reg_offset(entry->offset) ==
				gen_attr->mmio_list[count] &&
			    (1 << INTEL_INFO(dev_priv->dev)->gen &
					entry->gen_bitmask))
				break;
		}

		if (i == ARRAY_SIZE(whitelist))
			return -EINVAL;
	}
	return 0;
}

static int i915_gen_event_init(struct i915_perf_event *event,
			      struct drm_i915_perf_open_param *param)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	struct drm_i915_perf_gen_attr gen_attr;
	int ret;

	BUG_ON(param->type != I915_PERF_GENERIC_EVENT);

	/* To avoid the complexity of having to accurately filter
	 * reports and marshal to the appropriate client
	 * we currently only allow exclusive access */
	if (dev_priv->perf.generic.exclusive_event) {
		DRM_ERROR("Gen perf event already in use\n");
		return -EBUSY;
	}

	if ((param->sample_flags & I915_PERF_SAMPLE_OA_REPORT) ||
		(param->sample_flags & I915_PERF_SAMPLE_SOURCE_INFO)) {
		DRM_ERROR("Unsupported sample type for Gen perf event\n");
		return -EINVAL;
	}

	/*
	 * We check dev.i915.perf_event_paranoid sysctl option
	 * to determine if it's ok to access system wide OA counters
	 * without CAP_SYS_ADMIN privileges.
	 */
	if (i915_perf_event_paranoid && !capable(CAP_SYS_ADMIN)) {
		DRM_ERROR("Insufficient privileges to open perf event\n");
		return -EACCES;
	}

	ret = i915_perf_copy_attr(to_user_ptr(param->attr),
					      &gen_attr,
					      I915_GEN_ATTR_SIZE_VER0,
					      sizeof(gen_attr));
	if (ret)
		return ret;

	if (param->sample_flags & I915_PERF_SAMPLE_MMIO) {
		ret = check_mmio_whitelist(dev_priv, &gen_attr);
		if (ret)
			return ret;

		memcpy(dev_priv->perf.generic.mmio_list, gen_attr.mmio_list,
				sizeof(dev_priv->perf.generic.mmio_list));
	}

	spin_lock_init(&dev_priv->perf.generic.node_list_lock_generic);

	ret = alloc_gen_buffer(dev_priv);
	if (ret)
		return ret;

	dev_priv->perf.generic.exclusive_event = event;

	event->destroy = i915_gen_event_destroy;
	event->enable = i915_gen_event_enable;
	event->disable = i915_gen_event_disable;
	event->can_read = i915_gen_can_read;
	event->wait_unlocked = i915_gen_wait_unlocked;
	event->poll_wait = i915_gen_poll_wait;
	event->read = i915_gen_read;

	return 0;
}

static void gen7_update_specific_hw_ctx_id(struct drm_i915_private *dev_priv,
					   u32 ctx_id)
{
	dev_priv->perf.oa.specific_ctx_id = ctx_id;
	gen7_update_oacontrol(dev_priv);
}

static void i915_oa_context_pin_notify_locked(struct drm_i915_private *dev_priv,
					      struct intel_context *context)
{
	if (i915.enable_execlists ||
	    dev_priv->perf.oa.ops.update_specific_hw_ctx_id == NULL)
		return;

	if (dev_priv->perf.oa.exclusive_event &&
	    dev_priv->perf.oa.exclusive_event->ctx == context) {
		struct drm_i915_gem_object *obj =
			context->legacy_hw_ctx.rcs_state;
		u32 ctx_id = i915_gem_obj_ggtt_offset(obj);

		dev_priv->perf.oa.ops.update_specific_hw_ctx_id(
						dev_priv, ctx_id);
	}
}

void i915_oa_context_pin_notify(struct drm_i915_private *dev_priv,
				struct intel_context *context)
{
	if (!dev_priv->perf.initialized)
		return;

	mutex_lock(&dev_priv->perf.lock);
	i915_oa_context_pin_notify_locked(dev_priv, context);
	mutex_unlock(&dev_priv->perf.lock);
}

static void gen8_legacy_ctx_switch_notify(struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = req->engine;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	const struct i915_oa_reg *flex_regs = dev_priv->perf.oa.flex_regs;
	int n_flex_regs = dev_priv->perf.oa.flex_regs_len;
	int ret;
	int i;

	if (!atomic_read(&engine->oa_state_dirty))
		return;

	ret = intel_ring_begin(req, n_flex_regs * 2 + 4);
	if (ret)
		return;

	intel_ring_emit(engine, MI_LOAD_REGISTER_IMM(n_flex_regs + 1));

	intel_ring_emit(engine, i915_mmio_reg_offset(GEN8_OACTXCONTROL));
	intel_ring_emit(engine,
			(dev_priv->perf.oa.period_exponent <<
			 GEN8_OA_TIMER_PERIOD_SHIFT) |
			(dev_priv->perf.oa.periodic ?
			 GEN8_OA_TIMER_ENABLE : 0) |
			GEN8_OA_COUNTER_RESUME);

	for (i = 0; i < n_flex_regs; i++) {
		intel_ring_emit(engine, flex_regs[i].addr);
		intel_ring_emit(engine, flex_regs[i].value);
	}
	intel_ring_emit(engine, MI_NOOP);
	intel_ring_advance(engine);

	atomic_set(&engine->oa_state_dirty, false);
}

static void
i915_oa_legacy_ctx_switch_notify_locked(struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = req->engine;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;

	if (dev_priv->perf.oa.ops.legacy_ctx_switch_notify == NULL)
		return;

	if (dev_priv->perf.oa.exclusive_event &&
	    dev_priv->perf.oa.exclusive_event->enabled)
		dev_priv->perf.oa.ops.legacy_ctx_switch_notify(req);
}

void i915_oa_legacy_ctx_switch_notify(struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = req->engine;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;

	if (!dev_priv->perf.initialized)
		return;

	mutex_lock(&dev_priv->perf.lock);
	i915_oa_legacy_ctx_switch_notify_locked(req);
	mutex_unlock(&dev_priv->perf.lock);
}

static void i915_oa_update_reg_state_locked(struct intel_engine_cs *engine,
					    uint32_t *reg_state)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	const struct i915_oa_reg *flex_regs = dev_priv->perf.oa.flex_regs;
	int n_flex_regs = dev_priv->perf.oa.flex_regs_len;
	int ctx_oactxctrl = dev_priv->perf.oa.ctx_oactxctrl_off;
	int ctx_flexeu0 = dev_priv->perf.oa.ctx_flexeu0_off;
	int i;

	if (!atomic_read(&engine->oa_state_dirty))
		return;

	reg_state[ctx_oactxctrl] = i915_mmio_reg_offset(GEN8_OACTXCONTROL);
	reg_state[ctx_oactxctrl+1] = (dev_priv->perf.oa.period_exponent <<
				      GEN8_OA_TIMER_PERIOD_SHIFT) |
				     (dev_priv->perf.oa.periodic ?
				      GEN8_OA_TIMER_ENABLE : 0) |
				     GEN8_OA_COUNTER_RESUME;

	for (i = 0; i < n_flex_regs; i++) {
		uint32_t offset = flex_regs[i].addr;

		/* Map from mmio address to register state context
		 * offset... */

		offset -= i915_mmio_reg_offset(EU_PERF_CNTL0);

		offset >>= 5; /* Flex EU mmio registers are separated by 256
			       * bytes, here they are separated by 8 bytes */

		/* EU_PERF_CNTL0 offset in register state context... */
		offset += ctx_flexeu0;

		reg_state[offset] = flex_regs[i].addr;
		reg_state[offset+1] = flex_regs[i].value;
	}

	atomic_set(&engine->oa_state_dirty, false);
}

void i915_oa_update_reg_state(struct intel_engine_cs *ring, uint32_t *reg_state)
{
	struct drm_i915_private *dev_priv = ring->dev->dev_private;

	if (!dev_priv->perf.initialized)
		return;

	i915_oa_update_reg_state_locked(ring, reg_state);
}

static ssize_t i915_perf_read_locked(struct i915_perf_event *event,
				     struct file *file,
				     char __user *buf,
				     size_t count,
				     loff_t *ppos)
{
	struct drm_i915_private *dev_priv = event->dev_priv;
	struct i915_perf_read_state state = { count, 0, buf };
	int ret;

	if (file->f_flags & O_NONBLOCK) {
		if (!event->can_read(event))
			return -EAGAIN;
	} else {
		mutex_unlock(&dev_priv->perf.lock);
		ret = event->wait_unlocked(event);
		mutex_lock(&dev_priv->perf.lock);

		if (ret)
			return ret;
	}

	event->read(event, &state);
	if (state.read == 0)
		return -ENOSPC;

	return state.read;
}

static ssize_t i915_perf_read(struct file *file,
			      char __user *buf,
			      size_t count,
			      loff_t *ppos)
{
	struct i915_perf_event *event = file->private_data;
	struct drm_i915_private *dev_priv = event->dev_priv;
	ssize_t ret;

	mutex_lock(&dev_priv->perf.lock);
	ret = i915_perf_read_locked(event, file, buf, count, ppos);
	mutex_unlock(&dev_priv->perf.lock);

	return ret;
}

static enum hrtimer_restart poll_check_timer_cb(struct hrtimer *hrtimer)
{
	struct drm_i915_private *dev_priv =
		container_of(hrtimer, typeof(*dev_priv),
			     perf.oa.poll_check_timer);

	if (!oa_buffer_is_empty(dev_priv))
		wake_up(&dev_priv->perf.oa.poll_wq);

	hrtimer_forward_now(hrtimer, ns_to_ktime(POLL_PERIOD));

	return HRTIMER_RESTART;
}

static unsigned int i915_perf_poll_locked(struct i915_perf_event *event,
					  struct file *file,
					  poll_table *wait)
{
	unsigned int events = 0;

	event->poll_wait(event, file, wait);

	if (event->can_read(event))
		events |= POLLIN;

	return events;
}

static unsigned int i915_perf_poll(struct file *file, poll_table *wait)
{
	struct i915_perf_event *event = file->private_data;
	struct drm_i915_private *dev_priv = event->dev_priv;
	int ret;

	mutex_lock(&dev_priv->perf.lock);
	ret = i915_perf_poll_locked(event, file, wait);
	mutex_unlock(&dev_priv->perf.lock);

	return ret;
}

static void i915_perf_enable_locked(struct i915_perf_event *event)
{
	if (event->enabled)
		return;

	/* Allow event->enable() to refer to this */
	event->enabled = true;

	if (event->enable)
		event->enable(event);
}

static void i915_perf_disable_locked(struct i915_perf_event *event)
{
	if (!event->enabled)
		return;

	/* Allow event->disable() to refer to this */
	event->enabled = false;

	if (event->disable)
		event->disable(event);
}

static long i915_perf_ioctl_locked(struct i915_perf_event *event,
				   unsigned int cmd,
				   unsigned long arg)
{
	switch (cmd) {
	case I915_PERF_IOCTL_ENABLE:
		i915_perf_enable_locked(event);
		return 0;
	case I915_PERF_IOCTL_DISABLE:
		i915_perf_disable_locked(event);
		return 0;
	}

	return -EINVAL;
}

static long i915_perf_ioctl(struct file *file,
			    unsigned int cmd,
			    unsigned long arg)
{
	struct i915_perf_event *event = file->private_data;
	struct drm_i915_private *dev_priv = event->dev_priv;
	long ret;

	mutex_lock(&dev_priv->perf.lock);
	ret = i915_perf_ioctl_locked(event, cmd, arg);
	mutex_unlock(&dev_priv->perf.lock);

	return ret;
}

static void i915_perf_destroy_locked(struct i915_perf_event *event)
{
	struct drm_i915_private *dev_priv = event->dev_priv;

	if (event->enabled)
		i915_perf_disable_locked(event);

	if (event->destroy)
		event->destroy(event);

	list_del(&event->link);

	if (event->ctx) {
		mutex_lock(&dev_priv->dev->struct_mutex);
		i915_gem_context_unreference(event->ctx);
		mutex_unlock(&dev_priv->dev->struct_mutex);
	}

	kfree(event);
}

static int i915_perf_release(struct inode *inode, struct file *file)
{
	struct i915_perf_event *event = file->private_data;
	struct drm_i915_private *dev_priv = event->dev_priv;

	mutex_lock(&dev_priv->perf.lock);
	i915_perf_destroy_locked(event);
	mutex_unlock(&dev_priv->perf.lock);

	return 0;
}


static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.release	= i915_perf_release,
	.poll		= i915_perf_poll,
	.read		= i915_perf_read,
	.unlocked_ioctl	= i915_perf_ioctl,
};

static struct intel_context *
lookup_context(struct drm_i915_private *dev_priv,
	       struct file *user_filp,
	       u32 ctx_user_handle)
{
	struct intel_context *ctx;

	mutex_lock(&dev_priv->dev->struct_mutex);
	list_for_each_entry(ctx, &dev_priv->context_list, link) {
		struct drm_file *drm_file;

		if (!ctx->file_priv)
			continue;

		drm_file = ctx->file_priv->file;

		if (user_filp->private_data == drm_file &&
		    ctx->user_handle == ctx_user_handle) {
			i915_gem_context_reference(ctx);
			mutex_unlock(&dev_priv->dev->struct_mutex);

			return ctx;
		}
	}
	mutex_unlock(&dev_priv->dev->struct_mutex);

	return NULL;
}

int i915_perf_open_ioctl_locked(struct drm_device *dev, void *data,
				struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_perf_open_param *param = data;
	u32 known_open_flags = 0;
	u64 known_sample_flags = 0;
	struct intel_context *specific_ctx = NULL;
	struct i915_perf_event *event = NULL;
	unsigned long f_flags = 0;
	int event_fd;
	int ret = 0;

	known_open_flags = I915_PERF_FLAG_FD_CLOEXEC |
			   I915_PERF_FLAG_FD_NONBLOCK |
			   I915_PERF_FLAG_SINGLE_CONTEXT |
			   I915_PERF_FLAG_DISABLED;
	if (param->flags & ~known_open_flags) {
		DRM_ERROR("Unknown drm_i915_perf_open_param flag\n");
		ret = -EINVAL;
		goto err;
	}

	known_sample_flags = I915_PERF_SAMPLE_OA_REPORT |
			     I915_PERF_SAMPLE_CTXID |
			     I915_PERF_SAMPLE_SOURCE_INFO |
			     I915_PERF_SAMPLE_PID |
			     I915_PERF_SAMPLE_TAG |
			     I915_PERF_SAMPLE_TIMESTAMP |
			     I915_PERF_SAMPLE_RING_ID |
			     I915_PERF_SAMPLE_MMIO;
	if (param->sample_flags & ~known_sample_flags) {
		DRM_ERROR("Unknown drm_i915_perf_open_param sample_flag\n");
		ret = -EINVAL;
		goto err;
	}

	if (param->flags & I915_PERF_FLAG_SINGLE_CONTEXT) {
		u32 ctx_id = param->ctx_id;

		specific_ctx = lookup_context(dev_priv, file->filp, ctx_id);
		if (!specific_ctx) {
			DRM_ERROR(
				"Failed to look up context with ID %u for opening perf event\n",
				ctx_id);
			ret = -EINVAL;
			goto err;
		}
	}

	/* Similar to perf's kernel.perf_paranoid_cpu sysctl option
	 * we check a dev.i915.perf_event_paranoid sysctl option
	 * to determine if it's ok to access system wide OA counters
	 * without CAP_SYS_ADMIN privileges.
	 */
	if (!specific_ctx &&
	    i915_perf_event_paranoid && !capable(CAP_SYS_ADMIN)) {
		DRM_ERROR("Insufficient privileges to open perf event\n");
		ret = -EACCES;
		goto err_ctx;
	}

	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (!event) {
		ret = -ENOMEM;
		goto err_ctx;
	}

	event->sample_flags = param->sample_flags;
	event->dev_priv = dev_priv;
	event->ctx = specific_ctx;

	switch (param->type) {
	case I915_PERF_OA_EVENT:
		ret = i915_oa_event_init(event, param);
		if (ret)
			goto err_alloc;
		break;
	case I915_PERF_GENERIC_EVENT:
		ret = i915_gen_event_init(event, param);
		if (ret)
			goto err_alloc;
		break;
	default:
		DRM_ERROR("Unknown perf event type\n");
		ret = -EINVAL;
		goto err_alloc;
	}

	list_add(&event->link, &dev_priv->perf.events);

	if (param->flags & I915_PERF_FLAG_FD_CLOEXEC)
		f_flags |= O_CLOEXEC;
	if (param->flags & I915_PERF_FLAG_FD_NONBLOCK)
		f_flags |= O_NONBLOCK;

	event_fd = anon_inode_getfd("[i915_perf]", &fops, event, f_flags);
	if (event_fd < 0) {
		ret = event_fd;
		goto err_open;
	}

	param->fd = event_fd;

	if (!(param->flags & I915_PERF_FLAG_DISABLED))
		i915_perf_enable_locked(event);

	return 0;

err_open:
	list_del(&event->link);
	if (event->destroy)
		event->destroy(event);
err_alloc:
	kfree(event);
err_ctx:
	if (specific_ctx) {
		mutex_lock(&dev_priv->dev->struct_mutex);
		i915_gem_context_unreference(specific_ctx);
		mutex_unlock(&dev_priv->dev->struct_mutex);
	}
err:
	param->fd = -1;

	return ret;
}

int i915_perf_open_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	mutex_lock(&dev_priv->perf.lock);
	ret = i915_perf_open_ioctl_locked(dev, data, file);
	mutex_unlock(&dev_priv->perf.lock);

	return ret;
}


static struct ctl_table oa_table[] = {
	{
	 .procname = "perf_event_paranoid",
	 .data = &i915_perf_event_paranoid,
	 .maxlen = sizeof(i915_perf_event_paranoid),
	 .mode = 0644,
	 .proc_handler = proc_dointvec,
	 },
	{
	 .procname = "oa_event_min_timer_exponent",
	 .data = &i915_oa_event_min_timer_exponent,
	 .maxlen = sizeof(i915_oa_event_min_timer_exponent),
	 .mode = 0644,
	 .proc_handler = proc_dointvec_minmax,
	 .extra1 = &zero,
	 .extra2 = &oa_exponent_max,
	 },
	{}
};

static struct ctl_table i915_root[] = {
	{
	 .procname = "i915",
	 .maxlen = 0,
	 .mode = 0555,
	 .child = oa_table,
	 },
	{}
};

static struct ctl_table dev_root[] = {
	{
	 .procname = "dev",
	 .maxlen = 0,
	 .mode = 0555,
	 .child = i915_root,
	 },
	{}
};

void i915_perf_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);

	if (!(IS_HASWELL(dev) ||
		IS_BROADWELL(dev) || IS_CHERRYVIEW(dev) ||
		IS_SKYLAKE(dev) || IS_BROXTON(dev)))
		return;

	dev_priv->perf.sysctl_header = register_sysctl_table(dev_root);

	hrtimer_init(&dev_priv->perf.oa.poll_check_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev_priv->perf.oa.poll_check_timer.function = poll_check_timer_cb;
	init_waitqueue_head(&dev_priv->perf.oa.poll_wq);

	hrtimer_init(&dev_priv->perf.generic.poll_check_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev_priv->perf.generic.poll_check_timer.function = gen_poll_check_timer_cb;
	init_waitqueue_head(&dev_priv->perf.generic.poll_wq);

	INIT_LIST_HEAD(&dev_priv->perf.events);
	mutex_init(&dev_priv->perf.lock);

	if (IS_HASWELL(dev)) {
		dev_priv->perf.oa.ops.init_oa_buffer = gen7_init_oa_buffer;
		dev_priv->perf.oa.ops.enable_metric_set = hsw_enable_metric_set;
		dev_priv->perf.oa.ops.disable_metric_set =
						hsw_disable_metric_set;
		dev_priv->perf.oa.ops.oa_enable = gen7_oa_enable;
		dev_priv->perf.oa.ops.oa_disable = gen7_oa_disable;
		dev_priv->perf.oa.ops.update_specific_hw_ctx_id =
						gen7_update_specific_hw_ctx_id;
		dev_priv->perf.oa.ops.read = gen7_oa_read;
		dev_priv->perf.oa.ops.oa_buffer_is_empty =
						gen7_oa_buffer_is_empty;

		dev_priv->perf.oa.oa_formats = hsw_oa_formats;
	} else {
		dev_priv->perf.oa.ops.init_oa_buffer = gen8_init_oa_buffer;
		dev_priv->perf.oa.ops.oa_enable = gen8_oa_enable;
		dev_priv->perf.oa.ops.oa_disable = gen8_oa_disable;
		dev_priv->perf.oa.ops.read = gen8_oa_read;
		dev_priv->perf.oa.ops.oa_buffer_is_empty =
						gen8_oa_buffer_is_empty;

		dev_priv->perf.oa.oa_formats = gen8_plus_oa_formats;

		if (!i915.enable_execlists) {
			dev_priv->perf.oa.ops.legacy_ctx_switch_notify =
				gen8_legacy_ctx_switch_notify;
		}

		if (IS_BROADWELL(dev)) {
			dev_priv->perf.oa.ops.enable_metric_set =
				bdw_enable_metric_set;
			dev_priv->perf.oa.ops.disable_metric_set =
				bdw_disable_metric_set;
			dev_priv->perf.oa.ctx_oactxctrl_off = 0x120;
			dev_priv->perf.oa.ctx_flexeu0_off = 0x2ce;
		} else if (IS_CHERRYVIEW(dev)) {
			dev_priv->perf.oa.ops.enable_metric_set =
				chv_enable_metric_set;
			dev_priv->perf.oa.ops.disable_metric_set =
				chv_disable_metric_set;
			dev_priv->perf.oa.ctx_oactxctrl_off = 0x120;
			dev_priv->perf.oa.ctx_flexeu0_off = 0x2ce;
		} else if (IS_SKYLAKE(dev) || IS_BROXTON(dev)) {
			/* Reuse the SKL metrics set for BXT */
			dev_priv->perf.oa.ops.enable_metric_set =
				skl_enable_metric_set;
			dev_priv->perf.oa.ops.disable_metric_set =
				skl_disable_metric_set;
			dev_priv->perf.oa.ctx_oactxctrl_off = 0x128;
			dev_priv->perf.oa.ctx_flexeu0_off = 0x3de;
		}
	}

	dev_priv->perf.initialized = true;
}

void i915_perf_fini(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);

	if (!dev_priv->perf.initialized)
		return;

	unregister_sysctl_table(dev_priv->perf.sysctl_header);

	dev_priv->perf.oa.ops.init_oa_buffer = NULL;

	dev_priv->perf.initialized = false;
}
