#ifndef _INTEL_RINGBUFFER_H_
#define _INTEL_RINGBUFFER_H_

#include <linux/hashtable.h>
#include "i915_gem_batch_pool.h"

#define I915_CMD_HASH_ORDER 9

/* Early gen2 devices have a cacheline of just 32 bytes, using 64 is overkill,
 * but keeps the logic simple. Indeed, the whole purpose of this macro is just
 * to give some inclination as to some of the magic values used in the various
 * workarounds!
 */
#define CACHELINE_BYTES 64
#define CACHELINE_DWORDS (CACHELINE_BYTES / sizeof(uint32_t))

/*
 * Gen2 BSpec "1. Programming Environment" / 1.4.4.6 "Ring Buffer Use"
 * Gen3 BSpec "vol1c Memory Interface Functions" / 2.3.4.5 "Ring Buffer Use"
 * Gen4+ BSpec "vol1c Memory Interface and Command Stream" / 5.3.4.5 "Ring Buffer Use"
 *
 * "If the Ring Buffer Head Pointer and the Tail Pointer are on the same
 * cacheline, the Head Pointer must not be greater than the Tail
 * Pointer."
 */
#define I915_RING_FREE_SPACE 64

struct  intel_hw_status_page {
	u32		*page_addr;
	unsigned int	gfx_addr;
	struct		drm_i915_gem_object *obj;
};

#define I915_READ_TAIL(ring) I915_READ(RING_TAIL((ring)->mmio_base))
#define I915_WRITE_TAIL(ring, val) I915_WRITE(RING_TAIL((ring)->mmio_base), val)

#define I915_READ_START(ring) I915_READ(RING_START((ring)->mmio_base))
#define I915_WRITE_START(ring, val) I915_WRITE(RING_START((ring)->mmio_base), val)

#define I915_READ_HEAD(ring)  I915_READ(RING_HEAD((ring)->mmio_base))
#define I915_WRITE_HEAD(ring, val) I915_WRITE(RING_HEAD((ring)->mmio_base), val)

#define I915_READ_CTL(ring) I915_READ(RING_CTL((ring)->mmio_base))
#define I915_WRITE_CTL(ring, val) I915_WRITE(RING_CTL((ring)->mmio_base), val)

#define I915_READ_IMR(ring) I915_READ(RING_IMR((ring)->mmio_base))
#define I915_WRITE_IMR(ring, val) I915_WRITE(RING_IMR((ring)->mmio_base), val)

#define I915_READ_MODE(ring) I915_READ(RING_MI_MODE((ring)->mmio_base))
#define I915_WRITE_MODE(ring, val) I915_WRITE(RING_MI_MODE((ring)->mmio_base), val)

#define I915_READ_UHPTR(ring) \
		I915_READ(RING_UHPTR((ring)->mmio_base))
#define I915_WRITE_UHPTR(ring, val) \
		I915_WRITE(RING_UHPTR((ring)->mmio_base), val)
#define I915_READ_NOPID(ring) I915_READ(RING_NOPID((ring)->mmio_base))

/* seqno size is actually only a uint32, but since we plan to use MI_FLUSH_DW to
 * do the writes, and that must have qw aligned offsets, simply pretend it's 8b.
 */
#define i915_semaphore_seqno_size sizeof(uint64_t)
#define GEN8_SIGNAL_OFFSET(__ring, to)			     \
	(i915_gem_obj_ggtt_offset(dev_priv->semaphore_obj) + \
	((__ring)->id * I915_NUM_ENGINES * i915_semaphore_seqno_size) +	\
	(i915_semaphore_seqno_size * (to)))

#define GEN8_WAIT_OFFSET(__ring, from)			     \
	(i915_gem_obj_ggtt_offset(dev_priv->semaphore_obj) + \
	((from) * I915_NUM_ENGINES * i915_semaphore_seqno_size) + \
	(i915_semaphore_seqno_size * (__ring)->id))

#define GEN8_RING_SEMAPHORE_INIT(e) do { \
	if (!dev_priv->semaphore_obj) { \
		break; \
	} \
	(e)->semaphore.signal_ggtt[RCS] = GEN8_SIGNAL_OFFSET((e), RCS); \
	(e)->semaphore.signal_ggtt[VCS] = GEN8_SIGNAL_OFFSET((e), VCS); \
	(e)->semaphore.signal_ggtt[BCS] = GEN8_SIGNAL_OFFSET((e), BCS); \
	(e)->semaphore.signal_ggtt[VECS] = GEN8_SIGNAL_OFFSET((e), VECS); \
	(e)->semaphore.signal_ggtt[VCS2] = GEN8_SIGNAL_OFFSET((e), VCS2); \
	(e)->semaphore.signal_ggtt[(e)->id] = MI_SEMAPHORE_SYNC_INVALID; \
	} while(0)

enum intel_ring_hangcheck_action {
	HANGCHECK_IDLE = 0,
	HANGCHECK_WAIT,
	HANGCHECK_ACTIVE,
	HANGCHECK_KICK,
	HANGCHECK_HUNG,
};

#define HANGCHECK_SCORE_RING_HUNG 31

struct intel_ring_hangcheck {
	u64 acthd;
	u32 seqno;
	int score;
	enum intel_ring_hangcheck_action action;
	int deadlock;
	u32 instdone[I915_NUM_INSTDONE_REG];
};

struct intel_ringbuffer {
	struct drm_i915_gem_object *obj;
	void __iomem *virtual_start;
	struct i915_vma *vma;

	struct intel_engine_cs *engine;
	struct list_head link;

	u32 head;
	u32 tail;
	int space;
	int size;
	int effective_size;
	int reserved_size;
	int reserved_tail;
	bool reserved_in_use;

	/** We track the position of the requests in the ring buffer, and
	 * when each is retired we increment last_retired_head as the GPU
	 * must have finished processing the request and so we know we
	 * can advance the ringbuffer up to that position.
	 *
	 * last_retired_head is set to -1 after the value is consumed so
	 * we can detect new retirements.
	 */
	u32 last_retired_head;

	/*
	 * Consecutive resubmissions are opportunities for the h/w to do
	 * a 'lite restore' rather than a full context switch. Let's note
	 * when that happens, in case it's useful for hang diagnosis.
	 */
	u32 resubmission_count;
	u64 total_submission_count;

	/* Info about last time this ringbuffer was submitted (to GuC) */
	unsigned long last_submitted_jiffies;
	u32 last_submitted_seqno;
	u32 last_submitted_tail;
};

struct	intel_context;
struct drm_i915_reg_descriptor;

/*
 * we use a single page to load ctx workarounds so all of these
 * values are referred in terms of dwords
 *
 * struct i915_wa_ctx_bb:
 *  offset: specifies batch starting position, also helpful in case
 *    if we want to have multiple batches at different offsets based on
 *    some criteria. It is not a requirement at the moment but provides
 *    an option for future use.
 *  size: size of the batch in DWORDS
 */
struct  i915_ctx_workarounds {
	struct i915_wa_ctx_bb {
		u32 offset;
		u32 size;
	} indirect_ctx, per_ctx;
	struct drm_i915_gem_object *obj;
};

struct  intel_engine_cs {
	const char	*name;
	enum intel_engine_id {
		RCS = 0,
		BCS,
		VCS,
		VCS2,	/* Keep instances of the same type engine together. */
		VECS
	} id;
#define I915_NUM_ENGINES 5
#define _VCS(n) (VCS + (n))
	unsigned int guc_id;
	u32		mmio_base;
	struct		drm_device *dev;
	struct intel_ringbuffer *buffer;
	struct list_head buffers;

	/*
	 * A pool of objects to use as shadow copies of client batch buffers
	 * when the command parser is enabled. Prevents the client from
	 * modifying the batch contents after software parsing.
	 */
	struct i915_gem_batch_pool batch_pool;

	struct intel_hw_status_page status_page;
	struct i915_ctx_workarounds wa_ctx;

	unsigned irq_refcount; /* protected by dev_priv->irq_lock */
	u32		irq_enable_mask;	/* bitmask to enable ring interrupt */
	struct drm_i915_gem_request *trace_irq_req;
	bool __must_check (*irq_get)(struct intel_engine_cs *ring);
	void		(*irq_put)(struct intel_engine_cs *ring);

	int		(*init_hw)(struct intel_engine_cs *ring);

	int		(*init_context)(struct drm_i915_gem_request *req);

	void		(*write_tail)(struct intel_engine_cs *ring,
				      u32 value);
	int __must_check (*flush)(struct drm_i915_gem_request *req,
				  u32	invalidate_domains,
				  u32	flush_domains);
	int		(*add_request)(struct drm_i915_gem_request *req);
	/* Some chipsets are not quite as coherent as advertised and need
	 * an expensive kick to force a true read of the up-to-date seqno.
	 * However, the up-to-date seqno is not always required and the last
	 * seen value is good enough. Note that the seqno will always be
	 * monotonic, even if not coherent.
	 */
	u32		(*get_seqno)(struct intel_engine_cs *ring,
				     bool lazy_coherency);
	void		(*set_seqno)(struct intel_engine_cs *ring,
				     u32 seqno);
	int		(*dispatch_execbuffer)(struct drm_i915_gem_request *req,
					       u64 offset, u32 length,
					       unsigned dispatch_flags);
#define I915_DISPATCH_SECURE 0x1
#define I915_DISPATCH_PINNED 0x2
#define I915_DISPATCH_RS     0x4
	void		(*cleanup)(struct intel_engine_cs *ring);

	/* GEN8 signal/wait table - never trust comments!
	 *	  signal to	signal to    signal to   signal to      signal to
	 *	    RCS		   VCS          BCS        VECS		 VCS2
	 *      --------------------------------------------------------------------
	 *  RCS | NOP (0x00) | VCS (0x08) | BCS (0x10) | VECS (0x18) | VCS2 (0x20) |
	 *	|-------------------------------------------------------------------
	 *  VCS | RCS (0x28) | NOP (0x30) | BCS (0x38) | VECS (0x40) | VCS2 (0x48) |
	 *	|-------------------------------------------------------------------
	 *  BCS | RCS (0x50) | VCS (0x58) | NOP (0x60) | VECS (0x68) | VCS2 (0x70) |
	 *	|-------------------------------------------------------------------
	 * VECS | RCS (0x78) | VCS (0x80) | BCS (0x88) |  NOP (0x90) | VCS2 (0x98) |
	 *	|-------------------------------------------------------------------
	 * VCS2 | RCS (0xa0) | VCS (0xa8) | BCS (0xb0) | VECS (0xb8) | NOP  (0xc0) |
	 *	|-------------------------------------------------------------------
	 *
	 * Generalization:
	 *  f(x, y) := (x->id * NUM_RINGS * seqno_size) + (seqno_size * y->id)
	 *  ie. transpose of g(x, y)
	 *
	 *	 sync from	sync from    sync from    sync from	sync from
	 *	    RCS		   VCS          BCS        VECS		 VCS2
	 *      --------------------------------------------------------------------
	 *  RCS | NOP (0x00) | VCS (0x28) | BCS (0x50) | VECS (0x78) | VCS2 (0xa0) |
	 *	|-------------------------------------------------------------------
	 *  VCS | RCS (0x08) | NOP (0x30) | BCS (0x58) | VECS (0x80) | VCS2 (0xa8) |
	 *	|-------------------------------------------------------------------
	 *  BCS | RCS (0x10) | VCS (0x38) | NOP (0x60) | VECS (0x88) | VCS2 (0xb0) |
	 *	|-------------------------------------------------------------------
	 * VECS | RCS (0x18) | VCS (0x40) | BCS (0x68) |  NOP (0x90) | VCS2 (0xb8) |
	 *	|-------------------------------------------------------------------
	 * VCS2 | RCS (0x20) | VCS (0x48) | BCS (0x70) | VECS (0x98) |  NOP (0xc0) |
	 *	|-------------------------------------------------------------------
	 *
	 * Generalization:
	 *  g(x, y) := (y->id * NUM_RINGS * seqno_size) + (seqno_size * x->id)
	 *  ie. transpose of f(x, y)
	 */
	struct {
		u32	sync_seqno[I915_NUM_ENGINES-1];

		union {
			struct {
				/* our mbox written by others */
				u32		wait[I915_NUM_ENGINES];
				/* mboxes this ring signals to */
				i915_reg_t	signal[I915_NUM_ENGINES];
			} mbox;
			u64		signal_ggtt[I915_NUM_ENGINES];
		};

		/* AKA wait() */
		int	(*sync_to)(struct drm_i915_gem_request *to_req,
				   struct intel_engine_cs *from,
				   u32 seqno);
		int	(*signal)(struct drm_i915_gem_request *signaller_req,
				  /* num_dwords needed by caller */
				  unsigned int num_dwords);
	} semaphore;

	/* Execlists */
	spinlock_t execlist_lock;
	struct list_head execlist_queue;
	struct list_head execlist_retired_req_list;
	unsigned int fw_domains;
	u8 next_context_status_buffer;
	u32             irq_keep_mask; /* bitmask for interrupts that should not be masked */
	int		(*emit_request)(struct drm_i915_gem_request *request);
	int		(*emit_flush)(struct drm_i915_gem_request *request,
				      u32 invalidate_domains,
				      u32 flush_domains);
	int		(*emit_bb_start)(struct drm_i915_gem_request *req,
					 u64 offset, unsigned dispatch_flags);

	/**
	 * List of objects currently involved in rendering from the
	 * ringbuffer.
	 *
	 * Includes buffers having the contents of their GPU caches
	 * flushed, not necessarily primitives.  last_read_req
	 * represents when the rendering involved will be completed.
	 *
	 * A reference is held on the buffer while on this list.
	 */
	struct list_head active_list;

	/**
	 * List of breadcrumbs associated with GPU requests currently
	 * outstanding.
	 */
	struct list_head request_list;

	/**
	 * Seqno of request most recently submitted to request_list.
	 * Used exclusively by hang checker to avoid grabbing lock while
	 * inspecting request list.
	 */
	u32 last_submitted_seqno;
	struct intel_ringbuffer *last_submitted_ringbuf;

	/*
	 * Deferred free list to allow unreferencing requests from interrupt
	 * contexts and from outside of the i915 driver.
	 */
	struct list_head delayed_free_list;
	spinlock_t delayed_free_lock;

	bool gpu_caches_dirty;

	wait_queue_head_t irq_queue;

	struct intel_context *default_context;
	struct intel_context *last_context;

	struct intel_ring_hangcheck hangcheck;

	struct {
		struct drm_i915_gem_object *obj;
		u32 gtt_offset;
		volatile u32 *cpu_page;
	} scratch;

	bool needs_cmd_parser;

	/*
	 * Table of commands the command parser needs to know about
	 * for this ring.
	 */
	DECLARE_HASHTABLE(cmd_hash, I915_CMD_HASH_ORDER);

	/*
	 * Table of registers allowed in commands that read/write registers.
	 */
	const struct drm_i915_reg_descriptor *reg_table;
	int reg_count;

	/*
	 * Table of registers allowed in commands that read/write registers, but
	 * only from the DRM master.
	 */
	const struct drm_i915_reg_descriptor *master_reg_table;
	int master_reg_count;

	/*
	 * Returns the bitmask for the length field of the specified command.
	 * Return 0 for an unrecognized/invalid command.
	 *
	 * If the command parser finds an entry for a command in the ring's
	 * cmd_tables, it gets the command's length based on the table entry.
	 * If not, it calls this function to determine the per-ring length field
	 * encoding for the command (i.e. certain opcode ranges use certain bits
	 * to encode the command length in the header).
	 */
	u32 (*get_cmd_length_mask)(u32 cmd_header);

	spinlock_t fence_lock;
	struct list_head fence_signal_list;
	struct list_head fence_unsignal_list;
	uint32_t last_irq_seqno;
	uint32_t last_batch_start;

	atomic_t oa_state_dirty;
};

static inline bool
intel_engine_initialized(struct intel_engine_cs *engine)
{
	return engine->dev != NULL;
}

static inline unsigned
intel_engine_flag(struct intel_engine_cs *engine)
{
	return 1 << engine->id;
}

static inline u32
intel_ring_sync_index(struct intel_engine_cs *engine,
		      struct intel_engine_cs *other)
{
	int idx;

	/*
	 * rcs -> 0 = vcs, 1 = bcs, 2 = vecs, 3 = vcs2;
	 * vcs -> 0 = bcs, 1 = vecs, 2 = vcs2, 3 = rcs;
	 * bcs -> 0 = vecs, 1 = vcs2. 2 = rcs, 3 = vcs;
	 * vecs -> 0 = vcs2, 1 = rcs, 2 = vcs, 3 = bcs;
	 * vcs2 -> 0 = rcs, 1 = vcs, 2 = bcs, 3 = vecs;
	 */

	idx = (other - engine) - 1;
	if (idx < 0)
		idx += I915_NUM_ENGINES;

	return idx;
}

static inline void
intel_flush_status_page(struct intel_engine_cs *engine, int reg)
{
	drm_clflush_virt_range(&engine->status_page.page_addr[reg],
			       sizeof(uint32_t));
}

static inline u32
intel_read_status_page(struct intel_engine_cs *engine,
		       int reg)
{
	/* Ensure that the compiler doesn't optimize away the load. */
	barrier();
	return engine->status_page.page_addr[reg];
}

static inline void
intel_write_status_page(struct intel_engine_cs *engine,
			int reg, u32 value)
{
	engine->status_page.page_addr[reg] = value;
}

/**
 * Reads a dword out of the status page, which is written to from the command
 * queue by automatic updates, MI_REPORT_HEAD, MI_STORE_DATA_INDEX, or
 * MI_STORE_DATA_IMM.
 *
 * The following dwords have a reserved meaning:
 * 0x00: ISR copy, updated when an ISR bit not set in the HWSTAM changes.
 * 0x04: ring 0 head pointer
 * 0x05: ring 1 head pointer (915-class)
 * 0x06: ring 2 head pointer (915-class)
 * 0x10-0x1b: Context status DWords (GM45)
 * 0x1f: Last written status offset. (GM45)
 * 0x20-0x2f: Reserved (Gen6+)
 *
 * The area from dword 0x30 to 0x3ff is available for driver usage.
 *
 * Note: in general the allocation of these indices is arbitrary, as long
 * as they are all unique. But a few of them are used with instructions that
 * have specific alignment requirements, those particular indices must be
 * chosen carefully to meet those requirements. The list below shows the
 * currently-known alignment requirements:
 *
 *	I915_GEM_HWS_INDEX
 *	I915_GEM_SCRATCH_INDEX
 *		must be EVEN (QWord aligned) but ALSO bit 3 must be ZERO,
 *		so that the resulting address has a 0 in bit 5 (due to H/W
 *		limitation on MI_FLUSH_DW instruction with QWord data).
 *
 *	I915_BATCH_DONE_SEQNO
 *	I915_PREEMPTIVE_DONE_SEQNO
 *		must be EVEN (QWord aligned) but ALSO bit 3 must be ZERO,
 *		so that the resulting address has a 0 in bit 5 (due to H/W
 *		limitation on MI_FLUSH_DW instruction with QWord data).
 *
 *	I915_BATCH_ACTIVE_SEQNO
 *	I915_PREEMPTIVE_ACTIVE_SEQNO
 *		must each be at the odd address one above the corresponding
 *		I915_*_DONE_SEQNO value, as they are addressed both as DWords
 *		in their own right and as half of a QWord containing both the
 *		DONE and ACTIVE values together.
 */

/*
 * Tracking; these are updated by the GPU at the beginning and/or end of every
 * batch. One pair is for regular buffers, the other for preemptive ones.
 */
#define I915_BATCH_DONE_SEQNO		0x30  /* Completed batch seqno        */
#define I915_BATCH_ACTIVE_SEQNO		0x31  /* In progress batch seqno      */
#define I915_PREEMPTIVE_DONE_SEQNO	0x32  /* Completed preemptive batch   */
#define I915_PREEMPTIVE_ACTIVE_SEQNO	0x33  /* In progress preemptive batch */
#define I915_GEM_HWS_SCRATCH_INDEX	0x34  /* QWord, uses 0x35 as well     */
#define I915_GEM_HWS_SCRATCH_ADDR	(I915_GEM_HWS_SCRATCH_INDEX << MI_STORE_DWORD_INDEX_SHIFT)

/* Beware of addresses 0xX8-0xXF due to MI_FLUSH_DW with QWord bug */

#define I915_GEM_HWS_INDEX		I915_BATCH_DONE_SEQNO	/* alias */
//#define I915_GEM_HWS_INDEX_ADDR       (I915_GEM_HWS_INDEX << MI_STORE_DWORD_INDEX_SHIFT)
//#define I915_GEM_ACTIVE_SEQNO_INDEX	I915_BATCH_ACTIVE_SEQNO	/* alias */


struct intel_ringbuffer *
intel_engine_create_ringbuffer(struct intel_engine_cs *engine, int size);
int intel_pin_and_map_ringbuffer_obj(struct drm_device *dev,
				     struct intel_ringbuffer *ringbuf);
void intel_unpin_ringbuffer_obj(struct intel_ringbuffer *ringbuf);
void intel_ringbuffer_free(struct intel_ringbuffer *ring);

void intel_stop_engine(struct intel_engine_cs *engine);
void intel_cleanup_engine(struct intel_engine_cs *engine);

int intel_ring_alloc_request_extras(struct drm_i915_gem_request *request);

int intel_ring_test_space(struct intel_ringbuffer *ringbuf, int min_space);
int __must_check intel_ring_begin(struct drm_i915_gem_request *req, int n);
int __must_check intel_ring_cacheline_align(struct drm_i915_gem_request *req);
static inline void intel_ring_emit(struct intel_engine_cs *engine,
				   u32 data)
{
	struct intel_ringbuffer *ringbuf = engine->buffer;
	iowrite32(data, ringbuf->virtual_start + ringbuf->tail);
	ringbuf->tail += 4;
}
static inline void intel_ring_emit_reg(struct intel_engine_cs *engine,
				       i915_reg_t reg)
{
	intel_ring_emit(engine, i915_mmio_reg_offset(reg));
}
static inline void intel_ring_advance(struct intel_engine_cs *engine)
{
	struct intel_ringbuffer *ringbuf = engine->buffer;
	ringbuf->tail &= ringbuf->size - 1;
}
int __intel_ring_space(int head, int tail, int size);
void intel_ring_update_space(struct intel_ringbuffer *ringbuf);
int intel_ring_space(struct intel_ringbuffer *ringbuf);
bool intel_engine_stopped(struct intel_engine_cs *engine);

#define intel_engine_idle(engine)           __intel_engine_idle((engine), false)
#define intel_engine_idle_flush(engine)     __intel_engine_idle((engine), true)
int __must_check __intel_engine_idle(struct intel_engine_cs *engine, bool flush);
void intel_ring_init_seqno(struct intel_engine_cs *engine, u32 seqno);
int intel_ring_flush_all_caches(struct drm_i915_gem_request *req);
int intel_ring_invalidate_all_caches(struct drm_i915_gem_request *req);

void intel_fini_pipe_control(struct intel_engine_cs *engine);
int intel_init_pipe_control(struct intel_engine_cs *engine);

int intel_init_render_ring_buffer(struct drm_device *dev);
int intel_init_bsd_ring_buffer(struct drm_device *dev);
int intel_init_bsd2_ring_buffer(struct drm_device *dev);
int intel_init_blt_ring_buffer(struct drm_device *dev);
int intel_init_vebox_ring_buffer(struct drm_device *dev);

u64 intel_ring_get_active_head(struct intel_engine_cs *engine);

int init_workarounds_ring(struct intel_engine_cs *engine);

static inline u32 intel_ring_get_tail(struct intel_ringbuffer *ringbuf)
{
	return ringbuf->tail;
}

/*
 * Arbitrary size for largest possible 'add request' sequence. The code paths
 * are complex and variable. Empirical measurement shows that the worst case
 * is ILK at 136 words. Reserving too much is better than reserving too little
 * as that allows for corner cases that might have been missed. So the figure
 * has been rounded up to 160 words.
 */
#define MIN_SPACE_FOR_ADD_REQUEST	160

/*
 * Reserve space in the ring to guarantee that the i915_add_request() call
 * will always have sufficient room to do its stuff. The request creation
 * code calls this automatically.
 */
void intel_ring_reserved_space_reserve(struct intel_ringbuffer *ringbuf, int size);
/* Cancel the reservation, e.g. because the request is being discarded. */
void intel_ring_reserved_space_cancel(struct intel_ringbuffer *ringbuf);
/* Use the reserved space - for use by i915_add_request() only. */
void intel_ring_reserved_space_use(struct intel_ringbuffer *ringbuf);
/* Finish with the reserved space - for use by i915_add_request() only. */
void intel_ring_reserved_space_end(struct intel_ringbuffer *ringbuf);

/* Legacy ringbuffer specific portion of reservation code: */
int intel_ring_reserve_space(struct drm_i915_gem_request *request);

#endif /* _INTEL_RINGBUFFER_H_ */
