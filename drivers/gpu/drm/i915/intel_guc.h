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
#ifndef _INTEL_GUC_H_
#define _INTEL_GUC_H_

#include "intel_guc_fwif.h"
#include "i915_guc_reg.h"

/*
 * This structure primarily describes the GEM object shared with the GuC.
 * The GEM object is held for the entire lifetime of our interaction with
 * the GuC, being allocated before the GuC is loaded with its firmware.
 * Because there's no way to update the address used by the GuC after
 * initialisation, the shared object must stay pinned into the GGTT as
 * long as the GuC is in use. We also keep the first page (only) mapped
 * into kernel address space, as it includes shared data that must be
 * updated on every request submission.
 *
 * The single GEM object described here is actually made up of several
 * separate areas, as far as the GuC is concerned. The first page (kept
 * kmap'd) includes the "process decriptor" which holds sequence data for
 * the doorbell, and one cacheline which actually *is* the doorbell; a
 * write to this will "ring the doorbell" (i.e. send an interrupt to the
 * GuC). The subsequent  pages of the client object constitute the work
 * queue (a circular array of work items), again described in the process
 * descriptor. Work queue pages are mapped momentarily as required.
 *
 * We also keep a few statistics on failures. Ideally, these should all
 * be zero!
 *   no_wq_space: times that the submission pre-check found no space was
 *                available in the work queue (note, the queue is shared,
 *                not per-engine). It is OK for this to be nonzero, but
 *                it should not be huge!
 *   q_fail: failed to enqueue a work item. This should never happen,
 *           because we check for space beforehand.
 *   b_fail: failed to ring the doorbell. This should never happen, unless
 *           somehow the hardware misbehaves, or maybe if the GuC firmware
 *           crashes? We probably need to reset the GPU to recover.
 *   retcode: errno from last guc_submit()
 */
struct i915_guc_client {
	struct intel_guc *guc;
	struct intel_context *owner;
	struct drm_i915_gem_object *client_obj;
	void *client_base;		/* first page (only) of above	*/
	uint64_t client_gtt;		/* GTT offset of client_obj	*/
	uint32_t priority;
	uint32_t ctx_index;

	uint32_t doorbell_offset;	/* offset within client obj	*/
	uint32_t proc_desc_offset;	/* offset within client_obj	*/
	uint32_t wq_offset;		/* offset within client_obj	*/
	uint32_t wq_size;
	uint32_t doorbell_cookie;
	uint16_t doorbell_id;
	uint16_t padding;		/* Maintain alignment		*/

	spinlock_t wq_lock;		/* Protects all data below	*/
	uint32_t wq_tail;
	uint32_t unused;		/* Was 'wq_head'		*/

	uint32_t no_wq_space;
	uint32_t q_fail;		/* No longer used		*/
	uint32_t b_fail;
	int retcode;

	/* Per-engine counts of GuC submissions */
	uint64_t submissions[GUC_MAX_ENGINES_NUM];
};

enum intel_uc_fw_status {
	UC_FIRMWARE_FAIL = -1,
	UC_FIRMWARE_NONE = 0,
	UC_FIRMWARE_PENDING,
	UC_FIRMWARE_SUCCESS
};

#define UC_FW_TYPE_GUC		0
#define UC_FW_TYPE_HUC		1

/*
 * This structure encapsulates all the data needed during the process
 * of fetching, caching, and loading the firmware image into the GuC.
 */
struct intel_uc_fw {
	struct drm_device *		uc_dev;
	const char *			uc_fw_path;
	size_t				uc_fw_size;
	struct drm_i915_gem_object *	uc_fw_obj;
	enum intel_uc_fw_status fetch_status;
	enum intel_uc_fw_status	load_status;

	uint16_t major_ver_wanted;
	uint16_t minor_ver_wanted;
	uint16_t major_ver_found;
	uint16_t minor_ver_found;

	uint32_t fw_type;
	uint32_t header_size;
	uint32_t header_offset;
	uint32_t rsa_size;
	uint32_t rsa_offset;
	uint32_t ucode_size;
	uint32_t ucode_offset;
};

struct intel_guc_log {
	uint32_t flags;
	struct drm_i915_gem_object *obj;
	void *buf_addr;
	struct workqueue_struct *flush_wq;
	struct work_struct flush_work;
	struct rchan *relay_chan;

	/* logging related stats */
	u32 capture_miss_count;
	u32 flush_interrupt_count;
	u32 prev_overflow_count[GUC_MAX_LOG_BUFFER];
	u32 total_overflow_count[GUC_MAX_LOG_BUFFER];
	u32 flush_count[GUC_MAX_LOG_BUFFER];
};

struct intel_guc {
	struct intel_uc_fw guc_fw;
	struct intel_guc_log log;

	/* GuC2Host interrupt related state */
	bool interrupts_enabled;

	struct drm_i915_gem_object *ads_obj;

	struct drm_i915_gem_object *ctx_pool_obj;
	struct ida ctx_ids;

	struct i915_guc_client *execbuf_client;
	struct i915_guc_client *preempt_client;

	DECLARE_BITMAP(doorbell_bitmap, GUC_MAX_DOORBELLS);
	uint32_t db_cacheline;		/* Cyclic counter mod pagesize	*/

	/* Action status & statistics */
	uint64_t action_count;		/* Total commands issued	*/
	uint32_t action_cmd;		/* Last command word		*/
	uint32_t action_status;		/* Last return status		*/

	uint32_t action_fail_count;	/* Total number of failures	*/
	uint32_t action_fail_cmd;	/* Last failed command		*/
	uint32_t action_fail_status;	/* Last bad return status	*/
	int32_t action_err;		/* Last (nonzero) error code	*/

	/* Submission status & statistics */
	uint64_t submissions[GUC_MAX_ENGINES_NUM];
	uint32_t last_seqno[GUC_MAX_ENGINES_NUM];
	uint32_t failures[GUC_MAX_ENGINES_NUM];
	uint32_t preemptions[GUC_MAX_ENGINES_NUM];
	uint32_t last_preempt[GUC_MAX_ENGINES_NUM];
	uint32_t preempt_failures[GUC_MAX_ENGINES_NUM];

	/* To serialize the Host2GuC actions */
	struct mutex action_lock;
};

/* intel_guc_loader.c */
extern void intel_guc_init(struct drm_device *dev);
extern int intel_guc_setup(struct drm_device *dev, bool defer_not_exist);
extern void intel_guc_fini(struct drm_device *dev);
extern const char *intel_uc_fw_status_repr(enum intel_uc_fw_status status);
extern int intel_guc_suspend(struct drm_device *dev);
extern int intel_guc_resume(struct drm_device *dev);
extern u32 guc_wopcm_size(struct drm_device *dev);
int intel_uc_fw_fetch(struct drm_device *dev, struct intel_uc_fw *uc_fw);

/* i915_guc_submission.c */
int i915_guc_submission_init(struct drm_device *dev);
int i915_guc_submission_enable(struct drm_device *dev);
int i915_guc_submit(struct i915_guc_client *client,
		    struct drm_i915_gem_request *rq);
void i915_guc_submission_disable(struct drm_device *dev);
void i915_guc_submission_fini(struct drm_device *dev);
int i915_guc_wq_check_space(struct i915_guc_client *client);
void i915_guc_capture_logs(struct drm_device *dev);
void i915_guc_flush_logs(struct drm_device *dev, bool can_wait);
void i915_guc_register(struct drm_device *dev, bool not_defered);
void i915_guc_unregister(struct drm_device *dev);
int i915_guc_log_control(struct drm_device *dev, u64 control_val);
int host2guc_sample_forcewake(struct intel_guc *guc, bool enable);

#endif
