/*
 * Copyright © 2008,2010 Intel Corporation
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
 *    Eric Anholt <eric@anholt.net>
 *    Chris Wilson <chris@chris-wilson.co.uk>
 *
 */

#include <linux/syscalls.h>
#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"
#include <linux/dma_remapping.h>
#include <linux/uaccess.h>
#include <../drivers/staging/android/sync.h>
#include "i915_scheduler.h"

#define  __EXEC_OBJECT_HAS_PIN (1<<31)
#define  __EXEC_OBJECT_HAS_FENCE (1<<30)
#define  __EXEC_OBJECT_NEEDS_MAP (1<<29)
#define  __EXEC_OBJECT_NEEDS_BIAS (1<<28)

#define BATCH_OFFSET_BIAS (256*1024)

struct eb_vmas {
	struct list_head vmas;
	int and;
	union {
		struct i915_vma *lut[0];
		struct hlist_head buckets[0];
	};
};

static struct eb_vmas *
eb_create(struct drm_i915_gem_execbuffer2 *args)
{
	struct eb_vmas *eb = NULL;

	if (args->flags & I915_EXEC_HANDLE_LUT) {
		unsigned size = args->buffer_count;
		size *= sizeof(struct i915_vma *);
		size += sizeof(struct eb_vmas);
		eb = kmalloc(size, GFP_TEMPORARY | __GFP_NOWARN | __GFP_NORETRY);
	}

	if (eb == NULL) {
		unsigned size = args->buffer_count;
		unsigned count = PAGE_SIZE / sizeof(struct hlist_head) / 2;
		BUILD_BUG_ON_NOT_POWER_OF_2(PAGE_SIZE / sizeof(struct hlist_head));
		while (count > 2*size)
			count >>= 1;
		eb = kzalloc(count*sizeof(struct hlist_head) +
			     sizeof(struct eb_vmas),
			     GFP_TEMPORARY);
		if (eb == NULL)
			return eb;

		eb->and = count - 1;
	} else
		eb->and = -args->buffer_count;

	INIT_LIST_HEAD(&eb->vmas);
	return eb;
}

static void
eb_reset(struct eb_vmas *eb)
{
	if (eb->and >= 0)
		memset(eb->buckets, 0, (eb->and+1)*sizeof(struct hlist_head));
}

static int
eb_lookup_vmas(struct eb_vmas *eb,
	       struct drm_i915_gem_exec_object2 *exec,
	       const struct drm_i915_gem_execbuffer2 *args,
	       struct i915_address_space *vm,
	       struct drm_file *file)
{
	struct drm_i915_gem_object *obj;
	struct list_head objects;
	int i, ret;

	INIT_LIST_HEAD(&objects);
	spin_lock(&file->table_lock);
	/* Grab a reference to the object and release the lock so we can lookup
	 * or create the VMA without using GFP_ATOMIC */
	for (i = 0; i < args->buffer_count; i++) {
		obj = to_intel_bo(idr_find(&file->object_idr, exec[i].handle));
		if (obj == NULL) {
			spin_unlock(&file->table_lock);
			DRM_DEBUG("Invalid object handle %d at index %d\n",
				   exec[i].handle, i);
			ret = -ENOENT;
			goto err;
		}

		if (!list_empty(&obj->obj_exec_link)) {
			spin_unlock(&file->table_lock);
			DRM_DEBUG("Object %p [handle %d, index %d] appears more than once in object list\n",
				   obj, exec[i].handle, i);
			ret = -EINVAL;
			goto err;
		}

		drm_gem_object_reference(&obj->base);
		list_add_tail(&obj->obj_exec_link, &objects);
	}
	spin_unlock(&file->table_lock);

	i = 0;
	while (!list_empty(&objects)) {
		struct i915_vma *vma;

		obj = list_first_entry(&objects,
				       struct drm_i915_gem_object,
				       obj_exec_link);

		/*
		 * NOTE: We can leak any vmas created here when something fails
		 * later on. But that's no issue since vma_unbind can deal with
		 * vmas which are not actually bound. And since only
		 * lookup_or_create exists as an interface to get at the vma
		 * from the (obj, vm) we don't run the risk of creating
		 * duplicated vmas for the same vm.
		 */
		vma = i915_gem_obj_lookup_or_create_vma(obj, vm);
		if (IS_ERR(vma)) {
			DRM_DEBUG("Failed to lookup VMA\n");
			ret = PTR_ERR(vma);
			goto err;
		}

		/* Transfer ownership from the objects list to the vmas list. */
		list_add_tail(&vma->exec_list, &eb->vmas);
		list_del_init(&obj->obj_exec_link);

		vma->exec_entry = &exec[i];
		if (eb->and < 0) {
			eb->lut[i] = vma;
		} else {
			uint32_t handle = args->flags & I915_EXEC_HANDLE_LUT ? i : exec[i].handle;
			vma->exec_handle = handle;
			hlist_add_head(&vma->exec_node,
				       &eb->buckets[handle & eb->and]);
		}
		++i;
	}

	return 0;


err:
	while (!list_empty(&objects)) {
		obj = list_first_entry(&objects,
				       struct drm_i915_gem_object,
				       obj_exec_link);
		list_del_init(&obj->obj_exec_link);
		drm_gem_object_unreference(&obj->base);
	}
	/*
	 * Objects already transfered to the vmas list will be unreferenced by
	 * eb_destroy.
	 */

	return ret;
}

static struct i915_vma *eb_get_vma(struct eb_vmas *eb, unsigned long handle)
{
	if (eb->and < 0) {
		if (handle >= -eb->and)
			return NULL;
		return eb->lut[handle];
	} else {
		struct hlist_head *head;
		struct hlist_node *node;

		head = &eb->buckets[handle & eb->and];
		hlist_for_each(node, head) {
			struct i915_vma *vma;

			vma = hlist_entry(node, struct i915_vma, exec_node);
			if (vma->exec_handle == handle)
				return vma;
		}
		return NULL;
	}
}

static void
i915_gem_execbuffer_unreserve_vma(struct i915_vma *vma)
{
	struct drm_i915_gem_exec_object2 *entry;
	struct drm_i915_gem_object *obj = vma->obj;

	if (!drm_mm_node_allocated(&vma->node))
		return;

	entry = vma->exec_entry;

	if (entry->flags & __EXEC_OBJECT_HAS_FENCE)
		i915_gem_object_unpin_fence(obj);

	if (entry->flags & __EXEC_OBJECT_HAS_PIN)
		vma->pin_count--;

	entry->flags &= ~(__EXEC_OBJECT_HAS_FENCE | __EXEC_OBJECT_HAS_PIN);
}

static void eb_destroy(struct eb_vmas *eb)
{
	while (!list_empty(&eb->vmas)) {
		struct i915_vma *vma;

		vma = list_first_entry(&eb->vmas,
				       struct i915_vma,
				       exec_list);
		list_del_init(&vma->exec_list);
		i915_gem_execbuffer_unreserve_vma(vma);
		drm_gem_object_unreference(&vma->obj->base);
	}
	kfree(eb);
}

static inline int use_cpu_reloc(struct drm_i915_gem_object *obj)
{
	return (HAS_LLC(obj->base.dev) ||
		obj->base.write_domain == I915_GEM_DOMAIN_CPU ||
		obj->cache_level != I915_CACHE_NONE);
}

/* Used to convert any address to canonical form.
 * Starting from gen8, some commands (e.g. STATE_BASE_ADDRESS,
 * MI_LOAD_REGISTER_MEM and others, see Broadwell PRM Vol2a) require the
 * addresses to be in a canonical form:
 * "GraphicsAddress[63:48] are ignored by the HW and assumed to be in correct
 * canonical form [63:48] == [47]."
 */
#define GEN8_HIGH_ADDRESS_BIT 47
static inline uint64_t gen8_canonical_addr(uint64_t address)
{
	return sign_extend64(address, GEN8_HIGH_ADDRESS_BIT);
}

static inline uint64_t gen8_noncanonical_addr(uint64_t address)
{
	return address & ((1ULL << (GEN8_HIGH_ADDRESS_BIT + 1)) - 1);
}

static inline uint64_t
relocation_target(struct drm_i915_gem_relocation_entry *reloc,
		  uint64_t target_offset)
{
	return gen8_canonical_addr((int)reloc->delta + target_offset);
}

static int
relocate_entry_cpu(struct drm_i915_gem_object *obj,
		   struct drm_i915_gem_relocation_entry *reloc,
		   uint64_t target_offset)
{
	struct drm_device *dev = obj->base.dev;
	uint32_t page_offset = offset_in_page(reloc->offset);
	uint64_t delta = relocation_target(reloc, target_offset);
	char *vaddr;
	int ret;

	ret = i915_gem_object_set_to_cpu_domain(obj, true);
	if (ret)
		return ret;

	vaddr = kmap_atomic(i915_gem_object_get_dirty_page(obj,
				reloc->offset >> PAGE_SHIFT));
	*(uint32_t *)(vaddr + page_offset) = lower_32_bits(delta);

	if (INTEL_INFO(dev)->gen >= 8) {
		page_offset = offset_in_page(page_offset + sizeof(uint32_t));

		if (page_offset == 0) {
			kunmap_atomic(vaddr);
			vaddr = kmap_atomic(i915_gem_object_get_dirty_page(obj,
			    (reloc->offset + sizeof(uint32_t)) >> PAGE_SHIFT));
		}

		*(uint32_t *)(vaddr + page_offset) = upper_32_bits(delta);
	}

	kunmap_atomic(vaddr);

	return 0;
}

static int
relocate_entry_gtt(struct drm_i915_gem_object *obj,
		   struct drm_i915_gem_relocation_entry *reloc,
		   uint64_t target_offset)
{
	struct drm_device *dev = obj->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint64_t delta = relocation_target(reloc, target_offset);
	uint64_t offset;
	void __iomem *reloc_page;
	int ret;

	ret = i915_gem_object_set_to_gtt_domain(obj, true);
	if (ret)
		return ret;

	ret = i915_gem_object_put_fence(obj);
	if (ret)
		return ret;

	/* Map the page containing the relocation we're going to perform.  */
	offset = i915_gem_obj_ggtt_offset(obj);
	offset += reloc->offset;
	reloc_page = io_mapping_map_atomic_wc(dev_priv->gtt.mappable,
					      offset & PAGE_MASK);
	iowrite32(lower_32_bits(delta), reloc_page + offset_in_page(offset));

	if (INTEL_INFO(dev)->gen >= 8) {
		offset += sizeof(uint32_t);

		if (offset_in_page(offset) == 0) {
			io_mapping_unmap_atomic(reloc_page);
			reloc_page =
				io_mapping_map_atomic_wc(dev_priv->gtt.mappable,
							 offset);
		}

		iowrite32(upper_32_bits(delta),
			  reloc_page + offset_in_page(offset));
	}

	io_mapping_unmap_atomic(reloc_page);

	return 0;
}

static void
clflush_write32(void *addr, uint32_t value)
{
	/* This is not a fast path, so KISS. */
	drm_clflush_virt_range(addr, sizeof(uint32_t));
	*(uint32_t *)addr = value;
	drm_clflush_virt_range(addr, sizeof(uint32_t));
}

static int
relocate_entry_clflush(struct drm_i915_gem_object *obj,
		       struct drm_i915_gem_relocation_entry *reloc,
		       uint64_t target_offset)
{
	struct drm_device *dev = obj->base.dev;
	uint32_t page_offset = offset_in_page(reloc->offset);
	uint64_t delta = relocation_target(reloc, target_offset);
	char *vaddr;
	int ret;

	ret = i915_gem_object_set_to_gtt_domain(obj, true);
	if (ret)
		return ret;

	vaddr = kmap_atomic(i915_gem_object_get_dirty_page(obj,
				reloc->offset >> PAGE_SHIFT));
	clflush_write32(vaddr + page_offset, lower_32_bits(delta));

	if (INTEL_INFO(dev)->gen >= 8) {
		page_offset = offset_in_page(page_offset + sizeof(uint32_t));

		if (page_offset == 0) {
			kunmap_atomic(vaddr);
			vaddr = kmap_atomic(i915_gem_object_get_dirty_page(obj,
			    (reloc->offset + sizeof(uint32_t)) >> PAGE_SHIFT));
		}

		clflush_write32(vaddr + page_offset, upper_32_bits(delta));
	}

	kunmap_atomic(vaddr);

	return 0;
}

static int
i915_gem_execbuffer_relocate_entry(struct drm_i915_gem_object *obj,
				   struct eb_vmas *eb,
				   struct drm_i915_gem_relocation_entry *reloc)
{
	struct drm_device *dev = obj->base.dev;
	struct drm_gem_object *target_obj;
	struct drm_i915_gem_object *target_i915_obj;
	struct i915_vma *target_vma;
	uint64_t target_offset;
	int ret;

	/* we've already hold a reference to all valid objects */
	target_vma = eb_get_vma(eb, reloc->target_handle);
	if (unlikely(target_vma == NULL))
		return -ENOENT;
	target_i915_obj = target_vma->obj;
	target_obj = &target_vma->obj->base;

	target_offset = gen8_canonical_addr(target_vma->node.start);

	/* Sandybridge PPGTT errata: We need a global gtt mapping for MI and
	 * pipe_control writes because the gpu doesn't properly redirect them
	 * through the ppgtt for non_secure batchbuffers. */
	if (unlikely(IS_GEN6(dev) &&
	    reloc->write_domain == I915_GEM_DOMAIN_INSTRUCTION)) {
		ret = i915_vma_bind(target_vma, target_i915_obj->cache_level,
				    PIN_GLOBAL);
		if (WARN_ONCE(ret, "Unexpected failure to bind target VMA!"))
			return ret;
	}

	/* Validate that the target is in a valid r/w GPU domain */
	if (unlikely(reloc->write_domain & (reloc->write_domain - 1))) {
		DRM_DEBUG("reloc with multiple write domains: "
			  "obj %p target %d offset %d "
			  "read %08x write %08x",
			  obj, reloc->target_handle,
			  (int) reloc->offset,
			  reloc->read_domains,
			  reloc->write_domain);
		return -EINVAL;
	}
	if (unlikely((reloc->write_domain | reloc->read_domains)
		     & ~I915_GEM_GPU_DOMAINS)) {
		DRM_DEBUG("reloc with read/write non-GPU domains: "
			  "obj %p target %d offset %d "
			  "read %08x write %08x",
			  obj, reloc->target_handle,
			  (int) reloc->offset,
			  reloc->read_domains,
			  reloc->write_domain);
		return -EINVAL;
	}

	target_obj->pending_read_domains |= reloc->read_domains;
	target_obj->pending_write_domain |= reloc->write_domain;

	/* If the relocation already has the right value in it, no
	 * more work needs to be done.
	 */
	if (target_offset == reloc->presumed_offset)
		return 0;

	/* Check that the relocation address is valid... */
	if (unlikely(reloc->offset >
		obj->base.size - (INTEL_INFO(dev)->gen >= 8 ? 8 : 4))) {
		DRM_DEBUG("Relocation beyond object bounds: "
			  "obj %p target %d offset %d size %d.\n",
			  obj, reloc->target_handle,
			  (int) reloc->offset,
			  (int) obj->base.size);
		return -EINVAL;
	}
	if (unlikely(reloc->offset & 3)) {
		DRM_DEBUG("Relocation not 4-byte aligned: "
			  "obj %p target %d offset %d.\n",
			  obj, reloc->target_handle,
			  (int) reloc->offset);
		return -EINVAL;
	}

	/* We can't wait for rendering with pagefaults disabled */
	if (obj->active && pagefault_disabled())
		return -EFAULT;

	if (use_cpu_reloc(obj))
		ret = relocate_entry_cpu(obj, reloc, target_offset);
	else if (obj->map_and_fenceable)
		ret = relocate_entry_gtt(obj, reloc, target_offset);
	else if (cpu_has_clflush)
		ret = relocate_entry_clflush(obj, reloc, target_offset);
	else {
		WARN_ONCE(1, "Impossible case in relocation handling\n");
		ret = -ENODEV;
	}

	if (ret)
		return ret;

	/* and update the user's relocation entry */
	reloc->presumed_offset = target_offset;

	return 0;
}

static int
i915_gem_execbuffer_relocate_vma(struct i915_vma *vma,
				 struct eb_vmas *eb)
{
#define N_RELOC(x) ((x) / sizeof(struct drm_i915_gem_relocation_entry))
	struct drm_i915_gem_relocation_entry stack_reloc[N_RELOC(512)];
	struct drm_i915_gem_relocation_entry __user *user_relocs;
	struct drm_i915_gem_exec_object2 *entry = vma->exec_entry;
	int remain, ret;

	user_relocs = to_user_ptr(entry->relocs_ptr);

	remain = entry->relocation_count;
	while (remain) {
		struct drm_i915_gem_relocation_entry *r = stack_reloc;
		int count = remain;
		if (count > ARRAY_SIZE(stack_reloc))
			count = ARRAY_SIZE(stack_reloc);
		remain -= count;

		if (__copy_from_user_inatomic(r, user_relocs, count*sizeof(r[0])))
			return -EFAULT;

		do {
			u64 offset = r->presumed_offset;

			ret = i915_gem_execbuffer_relocate_entry(vma->obj, eb, r);
			if (ret)
				return ret;

			if (r->presumed_offset != offset &&
			    __copy_to_user_inatomic(&user_relocs->presumed_offset,
						    &r->presumed_offset,
						    sizeof(r->presumed_offset))) {
				return -EFAULT;
			}

			user_relocs++;
			r++;
		} while (--count);
	}

	return 0;
#undef N_RELOC
}

static int
i915_gem_execbuffer_relocate_vma_slow(struct i915_vma *vma,
				      struct eb_vmas *eb,
				      struct drm_i915_gem_relocation_entry *relocs)
{
	const struct drm_i915_gem_exec_object2 *entry = vma->exec_entry;
	int i, ret;

	for (i = 0; i < entry->relocation_count; i++) {
		ret = i915_gem_execbuffer_relocate_entry(vma->obj, eb, &relocs[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int
i915_gem_execbuffer_relocate(struct eb_vmas *eb)
{
	struct i915_vma *vma;
	int ret = 0;

	/* This is the fast path and we cannot handle a pagefault whilst
	 * holding the struct mutex lest the user pass in the relocations
	 * contained within a mmaped bo. For in such a case we, the page
	 * fault handler would call i915_gem_fault() and we would try to
	 * acquire the struct mutex again. Obviously this is bad and so
	 * lockdep complains vehemently.
	 */
	pagefault_disable();
	list_for_each_entry(vma, &eb->vmas, exec_list) {
		ret = i915_gem_execbuffer_relocate_vma(vma, eb);
		if (ret)
			break;
	}
	pagefault_enable();

	return ret;
}

static bool only_mappable_for_reloc(unsigned int flags)
{
	return (flags & (EXEC_OBJECT_NEEDS_FENCE | __EXEC_OBJECT_NEEDS_MAP)) ==
		__EXEC_OBJECT_NEEDS_MAP;
}

static int
i915_gem_execbuffer_reserve_vma(struct i915_vma *vma,
				struct intel_engine_cs *engine,
				bool *need_reloc)
{
	struct drm_i915_gem_object *obj = vma->obj;
	struct drm_i915_gem_exec_object2 *entry = vma->exec_entry;
	uint64_t flags;
	int ret;

	flags = PIN_USER;
	if (entry->flags & EXEC_OBJECT_NEEDS_GTT)
		flags |= PIN_GLOBAL;

	if (!drm_mm_node_allocated(&vma->node)) {
		/* Wa32bitGeneralStateOffset & Wa32bitInstructionBaseOffset,
		 * limit address to the first 4GBs for unflagged objects.
		 */
		if ((entry->flags & EXEC_OBJECT_SUPPORTS_48B_ADDRESS) == 0)
			flags |= PIN_ZONE_4G;
		if (entry->flags & __EXEC_OBJECT_NEEDS_MAP)
			flags |= PIN_GLOBAL | PIN_MAPPABLE;
		if (entry->flags & __EXEC_OBJECT_NEEDS_BIAS)
			flags |= BATCH_OFFSET_BIAS | PIN_OFFSET_BIAS;
		if (entry->flags & EXEC_OBJECT_PINNED)
			flags |= entry->offset | PIN_OFFSET_FIXED;
		if ((flags & PIN_MAPPABLE) == 0)
			flags |= PIN_HIGH;
	}

	ret = i915_gem_object_pin(obj, vma->vm,
				  entry->pad_to_size,
				  entry->alignment,
				  flags);
	if ((ret == -ENOSPC  || ret == -E2BIG) &&
	    only_mappable_for_reloc(entry->flags))
		ret = i915_gem_object_pin(obj, vma->vm,
					  entry->pad_to_size,
					  entry->alignment,
					  flags & ~PIN_MAPPABLE);
	if (ret)
		return ret;

	entry->flags |= __EXEC_OBJECT_HAS_PIN;

	if (entry->flags & EXEC_OBJECT_NEEDS_FENCE) {
		ret = i915_gem_object_get_fence(obj);
		if (ret)
			return ret;

		if (i915_gem_object_pin_fence(obj))
			entry->flags |= __EXEC_OBJECT_HAS_FENCE;
	}

	if (entry->offset != vma->node.start) {
		entry->offset = vma->node.start;
		*need_reloc = true;
	}

	if (entry->flags & EXEC_OBJECT_WRITE) {
		obj->base.pending_read_domains = I915_GEM_DOMAIN_RENDER;
		obj->base.pending_write_domain = I915_GEM_DOMAIN_RENDER;
	}

	return 0;
}

static bool
need_reloc_mappable(struct i915_vma *vma)
{
	struct drm_i915_gem_exec_object2 *entry = vma->exec_entry;

	if (entry->relocation_count == 0)
		return false;

	if (!i915_is_ggtt(vma->vm))
		return false;

	/* See also use_cpu_reloc() */
	if (HAS_LLC(vma->obj->base.dev))
		return false;

	if (vma->obj->base.write_domain == I915_GEM_DOMAIN_CPU)
		return false;

	return true;
}

static bool
eb_vma_misplaced(struct i915_vma *vma)
{
	struct drm_i915_gem_exec_object2 *entry = vma->exec_entry;
	struct drm_i915_gem_object *obj = vma->obj;

	WARN_ON(entry->flags & __EXEC_OBJECT_NEEDS_MAP &&
	       !i915_is_ggtt(vma->vm));

	if (entry->alignment &&
	    vma->node.start & (entry->alignment - 1))
		return true;

	if (vma->node.size < entry->pad_to_size)
		return true;

	if (entry->flags & EXEC_OBJECT_PINNED &&
	    vma->node.start != entry->offset)
		return true;

	if (entry->flags & __EXEC_OBJECT_NEEDS_BIAS &&
	    vma->node.start < BATCH_OFFSET_BIAS)
		return true;

	/* avoid costly ping-pong once a batch bo ended up non-mappable */
	if (entry->flags & __EXEC_OBJECT_NEEDS_MAP && !obj->map_and_fenceable)
		return !only_mappable_for_reloc(entry->flags);

	if ((entry->flags & EXEC_OBJECT_SUPPORTS_48B_ADDRESS) == 0 &&
	    (vma->node.start + vma->node.size - 1) >> 32)
		return true;

	return false;
}

static int
i915_gem_execbuffer_reserve(struct intel_engine_cs *engine,
			    struct list_head *vmas,
			    struct intel_context *ctx,
			    bool *need_relocs)
{
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	struct i915_address_space *vm;
	struct list_head ordered_vmas;
	struct list_head pinned_vmas;
	bool has_fenced_gpu_access = INTEL_INFO(engine->dev)->gen < 4;
	int retry;

	i915_gem_retire_requests_ring(engine);

	vm = list_first_entry(vmas, struct i915_vma, exec_list)->vm;

	INIT_LIST_HEAD(&ordered_vmas);
	INIT_LIST_HEAD(&pinned_vmas);
	while (!list_empty(vmas)) {
		struct drm_i915_gem_exec_object2 *entry;
		bool need_fence, need_mappable;

		vma = list_first_entry(vmas, struct i915_vma, exec_list);
		obj = vma->obj;
		entry = vma->exec_entry;

		if (ctx->flags & CONTEXT_NO_ZEROMAP)
			entry->flags |= __EXEC_OBJECT_NEEDS_BIAS;

		if (!has_fenced_gpu_access)
			entry->flags &= ~EXEC_OBJECT_NEEDS_FENCE;
		need_fence =
			entry->flags & EXEC_OBJECT_NEEDS_FENCE &&
			obj->tiling_mode != I915_TILING_NONE;
		need_mappable = need_fence || need_reloc_mappable(vma);

		if (entry->flags & EXEC_OBJECT_PINNED)
			list_move_tail(&vma->exec_list, &pinned_vmas);
		else if (need_mappable) {
			entry->flags |= __EXEC_OBJECT_NEEDS_MAP;
			list_move(&vma->exec_list, &ordered_vmas);
		} else
			list_move_tail(&vma->exec_list, &ordered_vmas);

		obj->base.pending_read_domains = I915_GEM_GPU_DOMAINS & ~I915_GEM_DOMAIN_COMMAND;
		obj->base.pending_write_domain = 0;
	}
	list_splice(&ordered_vmas, vmas);
	list_splice(&pinned_vmas, vmas);

	/* Attempt to pin all of the buffers into the GTT.
	 * This is done in 3 phases:
	 *
	 * 1a. Unbind all objects that do not match the GTT constraints for
	 *     the execbuffer (fenceable, mappable, alignment etc).
	 * 1b. Increment pin count for already bound objects.
	 * 2.  Bind new objects.
	 * 3.  Decrement pin count.
	 *
	 * This avoid unnecessary unbinding of later objects in order to make
	 * room for the earlier objects *unless* we need to defragment.
	 */
	retry = 0;
	do {
		int ret = 0;

		/* Unbind any ill-fitting objects or pin. */
		list_for_each_entry(vma, vmas, exec_list) {
			if (!drm_mm_node_allocated(&vma->node))
				continue;

			if (eb_vma_misplaced(vma))
				ret = i915_vma_unbind(vma);
			else
				ret = i915_gem_execbuffer_reserve_vma(vma,
								      engine,
								      need_relocs);
			if (ret)
				goto err;
		}

		/* Bind fresh objects */
		list_for_each_entry(vma, vmas, exec_list) {
			if (drm_mm_node_allocated(&vma->node))
				continue;

			ret = i915_gem_execbuffer_reserve_vma(vma, engine,
							      need_relocs);
			if (ret)
				goto err;
		}

err:
		if (ret != -ENOSPC || retry++)
			return ret;

		/* Decrement pin count for bound objects */
		list_for_each_entry(vma, vmas, exec_list)
			i915_gem_execbuffer_unreserve_vma(vma);

		ret = i915_gem_evict_vm(vm, true);
		if (ret)
			return ret;
	} while (1);
}

static int
i915_gem_execbuffer_relocate_slow(struct drm_device *dev,
				  struct drm_i915_gem_execbuffer2 *args,
				  struct drm_file *file,
				  struct intel_engine_cs *engine,
				  struct eb_vmas *eb,
				  struct drm_i915_gem_exec_object2 *exec,
				  struct intel_context *ctx)
{
	struct drm_i915_gem_relocation_entry *reloc;
	struct i915_address_space *vm;
	struct i915_vma *vma;
	bool need_relocs;
	int *reloc_offset;
	int i, total, ret;
	unsigned count = args->buffer_count;

	vm = list_first_entry(&eb->vmas, struct i915_vma, exec_list)->vm;

	/* We may process another execbuffer during the unlock... */
	while (!list_empty(&eb->vmas)) {
		vma = list_first_entry(&eb->vmas, struct i915_vma, exec_list);
		list_del_init(&vma->exec_list);
		i915_gem_execbuffer_unreserve_vma(vma);
		drm_gem_object_unreference(&vma->obj->base);
	}

	mutex_unlock(&dev->struct_mutex);

	total = 0;
	for (i = 0; i < count; i++)
		total += exec[i].relocation_count;

	reloc_offset = drm_malloc_ab(count, sizeof(*reloc_offset));
	reloc = drm_malloc_ab(total, sizeof(*reloc));
	if (reloc == NULL || reloc_offset == NULL) {
		drm_free_large(reloc);
		drm_free_large(reloc_offset);
		mutex_lock(&dev->struct_mutex);
		return -ENOMEM;
	}

	total = 0;
	for (i = 0; i < count; i++) {
		struct drm_i915_gem_relocation_entry __user *user_relocs;
		u64 invalid_offset = (u64)-1;
		int j;

		user_relocs = to_user_ptr(exec[i].relocs_ptr);

		if (copy_from_user(reloc+total, user_relocs,
				   exec[i].relocation_count * sizeof(*reloc))) {
			ret = -EFAULT;
			mutex_lock(&dev->struct_mutex);
			goto err;
		}

		/* As we do not update the known relocation offsets after
		 * relocating (due to the complexities in lock handling),
		 * we need to mark them as invalid now so that we force the
		 * relocation processing next time. Just in case the target
		 * object is evicted and then rebound into its old
		 * presumed_offset before the next execbuffer - if that
		 * happened we would make the mistake of assuming that the
		 * relocations were valid.
		 */
		for (j = 0; j < exec[i].relocation_count; j++) {
			if (__copy_to_user(&user_relocs[j].presumed_offset,
					   &invalid_offset,
					   sizeof(invalid_offset))) {
				ret = -EFAULT;
				mutex_lock(&dev->struct_mutex);
				goto err;
			}
		}

		reloc_offset[i] = total;
		total += exec[i].relocation_count;
	}

	ret = i915_mutex_lock_interruptible(dev);
	if (ret) {
		mutex_lock(&dev->struct_mutex);
		goto err;
	}

	/* reacquire the objects */
	eb_reset(eb);
	ret = eb_lookup_vmas(eb, exec, args, vm, file);
	if (ret)
		goto err;

	need_relocs = (args->flags & I915_EXEC_NO_RELOC) == 0;
	ret = i915_gem_execbuffer_reserve(engine, &eb->vmas, ctx,
				          &need_relocs);
	if (ret)
		goto err;

	list_for_each_entry(vma, &eb->vmas, exec_list) {
		int offset = vma->exec_entry - exec;
		ret = i915_gem_execbuffer_relocate_vma_slow(vma, eb,
							    reloc + reloc_offset[offset]);
		if (ret)
			goto err;
	}

	/* Leave the user relocations as are, this is the painfully slow path,
	 * and we want to avoid the complication of dropping the lock whilst
	 * having buffers reserved in the aperture and so causing spurious
	 * ENOSPC for random operations.
	 */

err:
	drm_free_large(reloc);
	drm_free_large(reloc_offset);
	return ret;
}

static int
i915_gem_execbuffer_move_to_gpu(struct drm_i915_gem_request *req,
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

	if (flush_chipset)
		i915_gem_chipset_flush(req->engine->dev);

	if (flush_domains & I915_GEM_DOMAIN_GTT)
		wmb();

	return 0;
}

static bool
i915_gem_check_execbuffer(struct drm_i915_gem_execbuffer2 *exec)
{
	if (exec->flags & I915_EXEC_UNKNOWN_FLAGS)
		return false;

	/* Kernel clipping was a DRI1 misfeature */
	if (exec->num_cliprects || exec->cliprects_ptr)
		return false;

	if (exec->DR4 == 0xffffffff) {
		DRM_DEBUG("UXA submitting garbage DR4, fixing up\n");
		exec->DR4 = 0;
	}
	if (exec->DR1 || exec->DR4)
		return false;

	if ((exec->batch_start_offset | exec->batch_len) & 0x7)
		return false;

	return true;
}

static int
validate_exec_list(struct drm_device *dev,
		   struct drm_i915_gem_exec_object2 *exec,
		   int count)
{
	unsigned relocs_total = 0;
	unsigned relocs_max = UINT_MAX / sizeof(struct drm_i915_gem_relocation_entry);
	unsigned invalid_flags;
	int i;

	invalid_flags = __EXEC_OBJECT_UNKNOWN_FLAGS;
	if (USES_FULL_PPGTT(dev))
		invalid_flags |= EXEC_OBJECT_NEEDS_GTT;

	for (i = 0; i < count; i++) {
		char __user *ptr = to_user_ptr(exec[i].relocs_ptr);
		int length; /* limited by fault_in_pages_readable() */

		if (exec[i].flags & invalid_flags)
			return -EINVAL;

		/* Offset can be used as input (EXEC_OBJECT_PINNED), reject
		 * any non-page-aligned or non-canonical addresses.
		 */
		if (exec[i].flags & EXEC_OBJECT_PINNED) {
			if (exec[i].offset !=
			    gen8_canonical_addr(exec[i].offset & PAGE_MASK))
				return -EINVAL;

			/* From drm_mm perspective address space is continuous,
			 * so from this point we're always using non-canonical
			 * form internally.
			 */
			exec[i].offset = gen8_noncanonical_addr(exec[i].offset);
		}

		if (exec[i].alignment && !is_power_of_2(exec[i].alignment))
			return -EINVAL;

		/* pad_to_size was once a reserved field, so sanitize it */
		if (exec[i].flags & EXEC_OBJECT_PAD_TO_SIZE) {
			if (offset_in_page(exec[i].pad_to_size))
				return -EINVAL;
		} else {
			exec[i].pad_to_size = 0;
		}

		/* First check for malicious input causing overflow in
		 * the worst case where we need to allocate the entire
		 * relocation tree as a single array.
		 */
		if (exec[i].relocation_count > relocs_max - relocs_total)
			return -EINVAL;
		relocs_total += exec[i].relocation_count;

		length = exec[i].relocation_count *
			sizeof(struct drm_i915_gem_relocation_entry);
		/*
		 * We must check that the entire relocation array is safe
		 * to read, but since we may need to update the presumed
		 * offsets during execution, check for full write access.
		 */
		if (!access_ok(VERIFY_WRITE, ptr, length))
			return -EFAULT;

		if (likely(!i915.prefault_disable)) {
			if (fault_in_multipages_readable(ptr, length))
				return -EFAULT;
		}
	}

	return 0;
}

static struct intel_context *
i915_gem_validate_context(struct drm_device *dev, struct drm_file *file,
			  struct intel_engine_cs *engine, const u32 ctx_id)
{
	struct intel_context *ctx = NULL;
	struct i915_ctx_hang_stats *hs;

	if (engine->id != RCS && ctx_id != DEFAULT_CONTEXT_HANDLE)
		return ERR_PTR(-EINVAL);

	ctx = i915_gem_context_get(file->driver_priv, ctx_id);
	if (IS_ERR(ctx))
		return ctx;

	hs = &ctx->hang_stats;
	if (hs->banned) {
		DRM_DEBUG("Context %u tried to submit while banned\n", ctx_id);
		return ERR_PTR(-EIO);
	}

	if (i915.enable_execlists && !ctx->engine[engine->id].state) {
		int ret = intel_lr_context_deferred_alloc(ctx, engine);
		if (ret) {
			DRM_DEBUG("Could not create LRC %u: %d\n", ctx_id, ret);
			return ERR_PTR(ret);
		}
	}

	return ctx;
}

void
i915_gem_execbuffer_move_to_active(struct list_head *vmas,
				   struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = i915_gem_request_get_engine(req);
	struct i915_vma *vma;

	list_for_each_entry(vma, vmas, exec_list) {
		struct drm_i915_gem_exec_object2 *entry = vma->exec_entry;
		struct drm_i915_gem_object *obj = vma->obj;
		u32 old_read = obj->base.read_domains;
		u32 old_write = obj->base.write_domain;

		obj->dirty = 1; /* be paranoid  */
		obj->base.write_domain = obj->base.pending_write_domain;
		if (obj->base.write_domain == 0)
			obj->base.pending_read_domains |= obj->base.read_domains;
		obj->base.read_domains = obj->base.pending_read_domains;

		i915_vma_move_to_active(vma, req);
		if (obj->base.write_domain) {
			i915_gem_request_assign(&obj->last_write_req, req);

			intel_fb_obj_invalidate(obj, ORIGIN_CS);

			/* update for the implicit flush after a batch */
			obj->base.write_domain &= ~I915_GEM_GPU_DOMAINS;
		}
		if (entry->flags & EXEC_OBJECT_NEEDS_FENCE) {
			i915_gem_request_assign(&obj->last_fenced_req, req);
			if (entry->flags & __EXEC_OBJECT_HAS_FENCE) {
				struct drm_i915_private *dev_priv = to_i915(engine->dev);
				list_move_tail(&dev_priv->fence_regs[obj->fence_reg].lru_list,
					       &dev_priv->mm.fence_list);
			}
		}

		trace_i915_gem_object_change_domain(obj, old_read, old_write);
	}
}

void
i915_gem_execbuffer_retire_commands(struct i915_execbuffer_params *params)
{
	/* Unconditionally force add_request to emit a full flush. */
	params->engine->gpu_caches_dirty = true;

	/* Add a breadcrumb for the completion of the batch buffer */
	__i915_add_request(params->request, params->batch_obj, true);
}

static int
i915_reset_gen7_sol_offsets(struct drm_device *dev,
			    struct drm_i915_gem_request *req)
{
	struct intel_engine_cs *engine = req->engine;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;

	if (!IS_GEN7(dev) || engine != &dev_priv->engine[RCS]) {
		DRM_DEBUG("sol reset is gen7/rcs only\n");
		return -EINVAL;
	}

	for (i = 0; i < 4; i++) {
		intel_ring_emit(engine, MI_LOAD_REGISTER_IMM(1));
		intel_ring_emit_reg(engine, GEN7_SO_WRITE_OFFSET(i));
		intel_ring_emit(engine, 0);
	}

	return 0;
}

static struct drm_i915_gem_object*
i915_gem_execbuffer_parse(struct intel_engine_cs *engine,
			  struct drm_i915_gem_exec_object2 *shadow_exec_entry,
			  struct eb_vmas *eb,
			  struct drm_i915_gem_object *batch_obj,
			  u32 batch_start_offset,
			  u32 batch_len,
			  bool is_master)
{
	struct drm_i915_gem_object *shadow_batch_obj;
	struct i915_vma *vma;
	int ret;

	shadow_batch_obj = i915_gem_batch_pool_get(&engine->batch_pool,
						   PAGE_ALIGN(batch_len));
	if (IS_ERR(shadow_batch_obj))
		return shadow_batch_obj;

	ret = i915_parse_cmds(engine,
			      batch_obj,
			      shadow_batch_obj,
			      batch_start_offset,
			      batch_len,
			      is_master);
	if (ret)
		goto err;

	ret = i915_gem_obj_ggtt_pin(shadow_batch_obj, 0, 0);
	if (ret)
		goto err;

	i915_gem_object_unpin_pages(shadow_batch_obj);

	memset(shadow_exec_entry, 0, sizeof(*shadow_exec_entry));

	vma = i915_gem_obj_to_ggtt(shadow_batch_obj);
	vma->exec_entry = shadow_exec_entry;
	vma->exec_entry->flags = __EXEC_OBJECT_HAS_PIN;
	drm_gem_object_reference(&shadow_batch_obj->base);
	list_add_tail(&vma->exec_list, &eb->vmas);

	shadow_batch_obj->base.pending_read_domains = I915_GEM_DOMAIN_COMMAND;

	return shadow_batch_obj;

err:
	i915_gem_object_unpin_pages(shadow_batch_obj);
	if (ret == -EACCES) /* unhandled chained batch */
		return batch_obj;
	else
		return ERR_PTR(ret);
}

int
i915_gem_ringbuffer_submission(struct i915_execbuffer_params *params,
			       struct drm_i915_gem_execbuffer2 *args,
			       struct list_head *vmas)
{
	struct i915_scheduler_queue_entry *qe;
	struct drm_device *dev = params->dev;
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
			if (INTEL_INFO(dev)->gen < 4) {
				DRM_DEBUG("no rel constants on pre-gen4\n");
				return -EINVAL;
			}

			if (INTEL_INFO(dev)->gen > 5 &&
			    params->instp_mode == I915_EXEC_CONSTANTS_REL_SURFACE) {
				DRM_DEBUG("rel surface constants mode invalid on gen5+\n");
				return -EINVAL;
			}

			/* The HW changed the meaning on this bit on gen6 */
			if (INTEL_INFO(dev)->gen >= 6)
				params->instp_mask &= ~I915_EXEC_CONSTANTS_REL_SURFACE;
		}
		break;
	default:
		DRM_DEBUG("execbuf with unknown constants: %d\n", params->instp_mode);
		return -EINVAL;
	}

	ret = i915_gem_execbuffer_move_to_gpu(params->request, vmas);
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
 * This is the main function for sending a batch to the engine.
 * It is called from the scheduler, with the struct_mutex already held.
 */
int i915_gem_ringbuffer_submission_final(struct i915_execbuffer_params *params)
{
	struct drm_i915_private *dev_priv = to_i915(params->dev);
	struct drm_i915_gem_request *req = params->request;
	struct intel_engine_cs  *engine = params->engine;
	u64 exec_start, exec_len;
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

	ret = intel_ring_reserve_space(req);
	if (ret)
		goto error;

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
	ret = intel_ring_test_space(req->ringbuf, min_space);
	if (ret)
		goto error;

	ret = intel_ring_begin(req, I915_BATCH_EXEC_MAX_LEN);
	if (ret)
		goto error;

	/* record where we start filling the ring */
	req->head = intel_ring_get_tail(req->ringbuf);

	/*
	 * Unconditionally invalidate gpu caches and ensure that we do flush
	 * any residual writes from the previous batch.
	 */
	ret = intel_ring_invalidate_all_caches(req);
	if (ret)
		goto error;

	/* Switch to the correct context for the batch */
	ret = i915_switch_context(req);
	if (ret)
		goto error;

	WARN(params->ctx->ppgtt && params->ctx->ppgtt->pd_dirty_rings & (1<<engine->id),
	     "%s didn't clear reload\n", engine->name);

	if (engine == &dev_priv->engine[RCS] &&
	    params->instp_mode != params->ctx->relative_constants_mode) {
		intel_ring_emit(engine, MI_NOOP);
		intel_ring_emit(engine, MI_LOAD_REGISTER_IMM(1));
		intel_ring_emit_reg(engine, INSTPM);
		intel_ring_emit(engine, params->instp_mask << 16 | params->instp_mode);
		intel_ring_advance(engine);

		params->ctx->relative_constants_mode = params->instp_mode;
	}

	if (params->args_flags & I915_EXEC_GEN7_SOL_RESET) {
		ret = i915_reset_gen7_sol_offsets(params->dev, req);
		if (ret)
			goto error;
	}

	exec_len   = params->args_batch_len;
	exec_start = params->batch_obj_vm_offset +
		     params->args_batch_start_offset;

	i915_emit_profiling_data(params->request, params->ctx->user_handle,
				params->tag);

	ret = engine->dispatch_execbuffer(req,
					  exec_start, exec_len,
					  params->dispatch_flags);
	if (ret)
		goto error;

	i915_emit_profiling_data(params->request, params->ctx->user_handle,
				params->tag);

	trace_i915_gem_ring_dispatch(req, params->dispatch_flags);

	i915_gem_execbuffer_retire_commands(params);

error:
	if (ret)
		intel_ring_reserved_space_cancel(req->ringbuf);

	return ret;
}

/**
 * Find one BSD ring to dispatch the corresponding BSD command.
 * The ring index is returned.
 */
static unsigned int
gen8_dispatch_bsd_ring(struct drm_i915_private *dev_priv, struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;

	/* Check whether the file_priv has already selected one ring. */
	if ((int)file_priv->bsd_ring < 0) {
		/* If not, use the ping-pong mechanism to select one. */
		mutex_lock(&dev_priv->dev->struct_mutex);
		file_priv->bsd_ring = dev_priv->mm.bsd_ring_dispatch_index;
		dev_priv->mm.bsd_ring_dispatch_index ^= 1;
		mutex_unlock(&dev_priv->dev->struct_mutex);
	}

	return file_priv->bsd_ring;
}

static struct drm_i915_gem_object *
eb_get_batch(struct eb_vmas *eb)
{
	struct i915_vma *vma = list_entry(eb->vmas.prev, typeof(*vma), exec_list);

	/*
	 * SNA is doing fancy tricks with compressing batch buffers, which leads
	 * to negative relocation deltas. Usually that works out ok since the
	 * relocate address is still positive, except when the batch is placed
	 * very low in the GTT. Ensure this doesn't happen.
	 *
	 * Note that actual hangs have only been observed on gen7, but for
	 * paranoia do it everywhere.
	 */
	if ((vma->exec_entry->flags & EXEC_OBJECT_PINNED) == 0)
		vma->exec_entry->flags |= __EXEC_OBJECT_NEEDS_BIAS;

	return vma->obj;
}

#define I915_USER_RINGS (4)

static const enum intel_engine_id user_ring_map[I915_USER_RINGS + 1] = {
	[I915_EXEC_DEFAULT]	= RCS,
	[I915_EXEC_RENDER]	= RCS,
	[I915_EXEC_BLT]		= BCS,
	[I915_EXEC_BSD]		= VCS,
	[I915_EXEC_VEBOX]	= VECS
};

static int
eb_select_ring(struct drm_i915_private *dev_priv,
	       struct drm_file *file,
	       struct drm_i915_gem_execbuffer2 *args,
	       struct intel_engine_cs **engine)
{
	unsigned int user_ring_id = args->flags & I915_EXEC_RING_MASK;

	if (user_ring_id > I915_USER_RINGS) {
		DRM_DEBUG("execbuf with unknown ring: %u\n", user_ring_id);
		return -EINVAL;
	}

	if ((user_ring_id != I915_EXEC_BSD) &&
	    ((args->flags & I915_EXEC_BSD_MASK) != 0)) {
		DRM_DEBUG("execbuf with non bsd ring but with invalid "
			  "bsd dispatch flags: %d\n", (int)(args->flags));
		return -EINVAL;
	}

	if (user_ring_id == I915_EXEC_BSD && HAS_BSD2(dev_priv)) {
		unsigned int bsd_idx = args->flags & I915_EXEC_BSD_MASK;

		if (bsd_idx == I915_EXEC_BSD_DEFAULT) {
			bsd_idx = gen8_dispatch_bsd_ring(dev_priv, file);
		} else if (bsd_idx >= I915_EXEC_BSD_RING1 &&
			   bsd_idx <= I915_EXEC_BSD_RING2) {
			bsd_idx--;
		} else {
			DRM_DEBUG("execbuf with unknown bsd ring: %u\n",
				  bsd_idx);
			return -EINVAL;
		}

		*engine = &dev_priv->engine[_VCS(bsd_idx)];
	} else {
		*engine = &dev_priv->engine[user_ring_map[user_ring_id]];
	}

	if (!intel_engine_initialized(*engine)) {
		DRM_DEBUG("execbuf with invalid ring: %u\n", user_ring_id);
		return -EINVAL;
	}

	return 0;
}

/*
 * Do a synchronous wait on any incoming fence object (e.g. in the case where
 * the scheduler is disabled). NB: This must be called before acquiring the
 * driver mutex lock!
 */
static int i915_early_fence_wait(struct intel_engine_cs *ring, int fence_fd)
{
	struct drm_i915_private *dev_priv = to_i915(ring->dev);
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct sync_fence *fence;
	int ret = 0;

	if (fence_fd < 0) {
		DRM_DEBUG("Invalid wait fence fd %d on ring %d\n", fence_fd,
			  (int) ring->id);
		return -EINVAL;
	}

	fence = sync_fence_fdget(fence_fd);
	if (fence == NULL) {
		DRM_DEBUG("Invalid wait fence %d on ring %d\n", fence_fd,
			  (int) ring->id);
		return -EINVAL;
	}

	/*
	 * Wait forever for the fence to be signalled. This is safe
	 * because the the mutex lock has not yet been acquired and
	 * the wait is interruptible.
	 */
	if (i915_safe_to_ignore_fence(ring, fence))
		scheduler->stats[ring->id].fence_ignore++;
	else {
		scheduler->stats[ring->id].fence_wait++;
		ret = sync_fence_wait(fence, -1);
	}

	sync_fence_put(fence);
	return ret;
}

static int
i915_gem_do_execbuffer(struct drm_device *dev, void *data,
		       struct drm_file *file,
		       struct drm_i915_gem_execbuffer2 *args,
		       struct drm_i915_gem_exec_object2 *exec)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct eb_vmas *eb;
	struct drm_i915_gem_object *batch_obj;
	struct drm_i915_gem_exec_object2 shadow_exec_entry;
	struct intel_engine_cs *engine;
	struct intel_context *ctx;
	struct i915_address_space *vm;
	struct i915_scheduler_queue_entry qe;
	struct i915_execbuffer_params *params = &qe.params;
	const u32 ctx_id = i915_execbuffer2_get_context_id(*args);
	u32 dispatch_flags;
	int ret, i;
	bool need_relocs;
	int fd_fence_complete = -1;
	int fd_fence_wait = lower_32_bits(args->rsvd2);
	struct sync_fence *sync_fence;

	/*
	 * Make sure an broken fence handle is not returned no matter
	 * how early an error might be hit. Note that rsvd2 has been
	 * saved away above because it is also an input parameter!
	 */
	if (args->flags & I915_EXEC_CREATE_FENCE)
		args->rsvd2 = (__u64) -1;

	if (!i915_gem_check_execbuffer(args))
		return -EINVAL;

	ret = validate_exec_list(dev, exec, args->buffer_count);
	if (ret)
		return ret;

	dispatch_flags = 0;
	if (args->flags & I915_EXEC_SECURE) {
		if (!file->is_master || !capable(CAP_SYS_ADMIN))
		    return -EPERM;

		dispatch_flags |= I915_DISPATCH_SECURE;
	}
	if (args->flags & I915_EXEC_IS_PINNED)
		dispatch_flags |= I915_DISPATCH_PINNED;

	ret = eb_select_ring(dev_priv, file, args, &engine);
	if (ret)
		return ret;

	if (args->buffer_count < 1) {
		DRM_DEBUG("execbuf with %d buffers\n", args->buffer_count);
		return -EINVAL;
	}

	if (args->flags & I915_EXEC_RESOURCE_STREAMER) {
		if (!HAS_RESOURCE_STREAMER(dev)) {
			DRM_DEBUG("RS is only allowed for Haswell, Gen8 and above\n");
			return -EINVAL;
		}
		if (engine->id != RCS) {
			DRM_DEBUG("RS is not available on %s\n",
				 engine->name);
			return -EINVAL;
		}

		dispatch_flags |= I915_DISPATCH_RS;
	}

	/*
	 * Without a GPU scheduler, any fence waits must be done up front.
	 */
	if ((args->flags & I915_EXEC_WAIT_FENCE) &&
	    (!i915_scheduler_is_enabled(dev))) {
		ret = i915_early_fence_wait(engine, fd_fence_wait);
		if (ret < 0)
			return ret;

		args->flags &= ~I915_EXEC_WAIT_FENCE;
	}

	intel_runtime_pm_get(dev_priv);

	ret = i915_mutex_lock_interruptible(dev);
	if (ret)
		goto pre_mutex_err;

	ctx = i915_gem_validate_context(dev, file, engine, ctx_id);
	if (IS_ERR(ctx)) {
		mutex_unlock(&dev->struct_mutex);
		ret = PTR_ERR(ctx);
		goto pre_mutex_err;
	}

	i915_gem_context_reference(ctx);

	if (ctx->ppgtt)
		vm = &ctx->ppgtt->base;
	else
		vm = &dev_priv->gtt.base;

	memset(&qe, 0x00, sizeof(qe));

	eb = eb_create(args);
	if (eb == NULL) {
		i915_gem_context_unreference(ctx);
		mutex_unlock(&dev->struct_mutex);
		ret = -ENOMEM;
		goto pre_mutex_err;
	}

	qe.objs = kzalloc(sizeof(*qe.objs) * args->buffer_count, GFP_KERNEL);
	if (!qe.objs) {
		ret = -ENOMEM;
		goto err;
	}

	/* Look up object handles */
	ret = eb_lookup_vmas(eb, exec, args, vm, file);
	if (ret)
		goto err;

	/* take note of the batch buffer before we might reorder the lists */
	batch_obj = eb_get_batch(eb);

	/* Move the objects en-masse into the GTT, evicting if necessary. */
	need_relocs = (args->flags & I915_EXEC_NO_RELOC) == 0;
	ret = i915_gem_execbuffer_reserve(engine, &eb->vmas, ctx,
					  &need_relocs);
	if (ret)
		goto err;

	/* The objects are in their final locations, apply the relocations. */
	if (need_relocs)
		ret = i915_gem_execbuffer_relocate(eb);
	if (ret) {
		if (ret == -EFAULT) {
			ret = i915_gem_execbuffer_relocate_slow(dev, args, file,
								engine,
								eb, exec, ctx);
			BUG_ON(!mutex_is_locked(&dev->struct_mutex));
		}
		if (ret)
			goto err;
	}

	/* Set the pending read domains for the batch buffer to COMMAND */
	if (batch_obj->base.pending_write_domain) {
		DRM_DEBUG("Attempting to use self-modifying batch buffer\n");
		ret = -EINVAL;
		goto err;
	}

	params->args_batch_start_offset = args->batch_start_offset;
	if (i915_needs_cmd_parser(engine) && args->batch_len) {
		struct drm_i915_gem_object *parsed_batch_obj;

		parsed_batch_obj = i915_gem_execbuffer_parse(engine,
							     &shadow_exec_entry,
							     eb,
							     batch_obj,
							     args->batch_start_offset,
							     args->batch_len,
							     file->is_master);
		if (IS_ERR(parsed_batch_obj)) {
			ret = PTR_ERR(parsed_batch_obj);
			goto err;
		}

		/*
		 * parsed_batch_obj == batch_obj means batch not fully parsed:
		 * Accept, but don't promote to secure.
		 */

		if (parsed_batch_obj != batch_obj) {
			/*
			 * Batch parsed and accepted:
			 *
			 * Set the DISPATCH_SECURE bit to remove the NON_SECURE
			 * bit from MI_BATCH_BUFFER_START commands issued in
			 * the dispatch_execbuffer implementations. We
			 * specifically don't want that set on batches the
			 * command parser has accepted.
			 */
			dispatch_flags |= I915_DISPATCH_SECURE;
			params->args_batch_start_offset = 0;
			batch_obj = parsed_batch_obj;
		}
	}

	batch_obj->base.pending_read_domains |= I915_GEM_DOMAIN_COMMAND;

	/* snb/ivb/vlv conflate the "batch in ppgtt" bit with the "non-secure
	 * batch" bit. Hence we need to pin secure batches into the global gtt.
	 * hsw should have this fixed, but bdw mucks it up again. */
	if (dispatch_flags & I915_DISPATCH_SECURE) {
		/*
		 * So on first glance it looks freaky that we pin the batch here
		 * outside of the reservation loop. But:
		 * - The batch is already pinned into the relevant ppgtt, so we
		 *   already have the backing storage fully allocated.
		 * - No other BO uses the global gtt (well contexts, but meh),
		 *   so we don't really have issues with multiple objects not
		 *   fitting due to fragmentation.
		 * So this is actually safe.
		 */
		ret = i915_gem_obj_ggtt_pin(batch_obj, 0, 0);
		if (ret)
			goto err;

		params->batch_obj_vm_offset = i915_gem_obj_ggtt_offset(batch_obj);
	} else
		params->batch_obj_vm_offset = i915_gem_obj_offset(batch_obj, vm);

	/* Allocate a request for this batch buffer nice and early. */
	ret = i915_gem_request_alloc(engine, ctx, &params->request);
	if (ret)
		goto err_batch_unpin;

	ret = i915_gem_request_add_to_client(params->request, file);
	if (ret)
		goto err_batch_unpin;

	/*
	 * Save assorted stuff away to pass through to *_submission().
	 * NB: This data should be 'persistent' and not local as it will
	 * kept around beyond the duration of the IOCTL once the GPU
	 * scheduler arrives.
	 */
	params->dev                     = dev;
	params->file                    = file;
	params->engine                    = engine;
	params->dispatch_flags          = dispatch_flags;
	params->args_flags              = args->flags;
	params->args_batch_len          = args->batch_len;
	params->args_num_cliprects      = args->num_cliprects;
	params->args_DR1                = args->DR1;
	params->args_DR4                = args->DR4;
	params->batch_obj               = batch_obj;

	/* Use the out-of-memory priority value as a suitable starting point for
	 * the buffer priority. It seems to be zero for application level tasks
	 * and less than zero for system tasks. */
	qe.priority = (current->signal->oom_score_adj < 0) ?
					-current->signal->oom_score_adj : 0;

	/*
	 * Save away the list of objects used by this batch buffer for the
	 * purpose of tracking inter-buffer dependencies.
	 */
	for (i = 0; i < args->buffer_count; i++) {
		struct drm_i915_gem_object *obj;

		/*
		 * NB: 'drm_gem_object_lookup()' increments the object's
		 * reference count and so must be matched by a
		 * 'drm_gem_object_unreference' call.
		 */
		obj = to_intel_bo(drm_gem_object_lookup(dev, file,
							  exec[i].handle));
		qe.objs[i].obj       = obj;
		qe.objs[i].read_only = obj->base.pending_write_domain == 0;

	}
	qe.num_objs = i;

	/* Lock and save the context object as well. */
	i915_gem_context_reference(ctx);
	params->ctx = ctx;
	params->tag = i915_execbuffer2_get_tag(*args) >> 32;

	if (args->flags & I915_EXEC_WAIT_FENCE) {
		if (fd_fence_wait < 0) {
			DRM_ERROR("Wait fence for engine %d has invalid id %d\n",
				  (int) engine->id, fd_fence_wait);
			ret = -EINVAL;
			goto err_client;
		} else {
			params->fence_wait = sync_fence_fdget(fd_fence_wait);
			if (params->fence_wait == NULL) {
				DRM_ERROR("Invalid wait fence %d\n",
					  fd_fence_wait);
				ret = -EINVAL;
				goto err_client;
			}
		}
	}

	if (args->flags & I915_EXEC_CREATE_FENCE) {
		/*
		 * Caller has requested a sync fence.
		 * User interrupts will be enabled to make sure that
		 * the timeline is signalled on completion.
		 */
		ret = i915_create_sync_fence(params->request, &sync_fence,
					     &fd_fence_complete);
		if (ret) {
			DRM_ERROR("Fence creation failed for engine %d, ctx %p\n",
				  engine->id, ctx);
			goto err_client;
		}
	}

	ret = dev_priv->gt.execbuf_submit(params, args, &eb->vmas);
	if (ret)
		goto err_fence;

	/* the request owns the ref now */
	i915_gem_context_unreference(ctx);

	if (fd_fence_complete != -1) {
		/*
		 * Install the fence into the pre-allocated file
		 * descriptor to the fence object so that user land
		 * can wait on it...
		 */
		i915_install_sync_fence_fd(params->request,
					   sync_fence, fd_fence_complete);

		/* Return the fence through the rsvd2 field */
		args->rsvd2 = (__u64) fd_fence_complete;
	}

	/*
	 * The eb list is no longer required. The scheduler has extracted all
	 * the information than needs to persist.
	 */
	eb_destroy(eb);

	/*
	 * Don't clean up everything that is now saved away in the queue.
	 * Just unlock and return immediately.
	 */
	mutex_unlock(&dev->struct_mutex);

	intel_runtime_pm_put(dev_priv);

	return 0;

err_fence:
	if (fd_fence_complete != -1) {
		sync_fence_put(sync_fence);
		put_unused_fd(fd_fence_complete);
	}

err_client:
	if (params->request->file_priv)
		i915_gem_request_remove_from_client(params->request);

err_batch_unpin:
	if (dispatch_flags & I915_DISPATCH_SECURE)
		i915_gem_execbuf_release_batch_obj(batch_obj);

err:
	i915_gem_context_unreference(ctx);
	eb_destroy(eb);

	/* Need to release the objects: */
	if (qe.objs) {
		for (i = 0; i < qe.num_objs; i++)
			drm_gem_object_unreference(&qe.objs[i].obj->base);

		kfree(qe.objs);
	}

	/* Context too */
	if (params->ctx)
		i915_gem_context_unreference(params->ctx);

	if (params->fence_wait)
		sync_fence_put(params->fence_wait);

	/*
	 * If the request was created but not successfully submitted then it
	 * must be freed again. If it was submitted then it is being tracked
	 * on the active request list and no clean up is required here.
	 */
	if (ret && params->request)
		i915_gem_request_cancel(params->request);

	mutex_unlock(&dev->struct_mutex);

pre_mutex_err:
	/* intel_gpu_busy should also get a ref, so it will free when the device
	 * is really idle. */
	intel_runtime_pm_put(dev_priv);

	dev_priv->scheduler->stats[engine->id].exec_early++;

	return ret;
}

void i915_gem_execbuf_release_batch_obj(struct drm_i915_gem_object *batch_obj)
{
	/*
	 * FIXME: We crucially rely upon the active tracking for the (ppgtt)
	 * batch vma for correctness. For less ugly and less fragility this
	 * needs to be adjusted to also track the ggtt batch vma properly as
	 * active.
	 */
	i915_gem_object_ggtt_unpin(batch_obj);
}

/*
 * Legacy execbuffer just creates an exec2 list from the original exec object
 * list array and passes it to the real function.
 */
int
i915_gem_execbuffer(struct drm_device *dev, void *data,
		    struct drm_file *file)
{
	struct drm_i915_gem_execbuffer *args = data;
	struct drm_i915_gem_execbuffer2 exec2;
	struct drm_i915_gem_exec_object *exec_list = NULL;
	struct drm_i915_gem_exec_object2 *exec2_list = NULL;
	int ret, i;

	if (args->buffer_count < 1) {
		DRM_DEBUG("execbuf with %d buffers\n", args->buffer_count);
		return -EINVAL;
	}

	/* Throttle batch requests per device file */
	if (i915_scheduler_file_queue_wait(file))
		return -EAGAIN;

	/* Copy in the exec list from userland */
	exec_list = drm_malloc_ab(sizeof(*exec_list), args->buffer_count);
	exec2_list = drm_malloc_ab(sizeof(*exec2_list), args->buffer_count);
	if (exec_list == NULL || exec2_list == NULL) {
		DRM_DEBUG("Failed to allocate exec list for %d buffers\n",
			  args->buffer_count);
		drm_free_large(exec_list);
		drm_free_large(exec2_list);
		return -ENOMEM;
	}
	ret = copy_from_user(exec_list,
			     to_user_ptr(args->buffers_ptr),
			     sizeof(*exec_list) * args->buffer_count);
	if (ret != 0) {
		DRM_DEBUG("copy %d exec entries failed %d\n",
			  args->buffer_count, ret);
		drm_free_large(exec_list);
		drm_free_large(exec2_list);
		return -EFAULT;
	}

	for (i = 0; i < args->buffer_count; i++) {
		exec2_list[i].handle = exec_list[i].handle;
		exec2_list[i].relocation_count = exec_list[i].relocation_count;
		exec2_list[i].relocs_ptr = exec_list[i].relocs_ptr;
		exec2_list[i].alignment = exec_list[i].alignment;
		exec2_list[i].offset = exec_list[i].offset;
		if (INTEL_INFO(dev)->gen < 4)
			exec2_list[i].flags = EXEC_OBJECT_NEEDS_FENCE;
		else
			exec2_list[i].flags = 0;
	}

	exec2.buffers_ptr = args->buffers_ptr;
	exec2.buffer_count = args->buffer_count;
	exec2.batch_start_offset = args->batch_start_offset;
	exec2.batch_len = args->batch_len;
	exec2.DR1 = args->DR1;
	exec2.DR4 = args->DR4;
	exec2.num_cliprects = args->num_cliprects;
	exec2.cliprects_ptr = args->cliprects_ptr;
	exec2.flags = I915_EXEC_RENDER;
	i915_execbuffer2_set_context_id(exec2, 0);

	ret = i915_gem_do_execbuffer(dev, data, file, &exec2, exec2_list);
	if (!ret) {
		struct drm_i915_gem_exec_object __user *user_exec_list =
			to_user_ptr(args->buffers_ptr);

		/* Copy the new buffer offsets back to the user's exec list. */
		for (i = 0; i < args->buffer_count; i++) {
			exec2_list[i].offset =
				gen8_canonical_addr(exec2_list[i].offset);
			ret = __copy_to_user(&user_exec_list[i].offset,
					     &exec2_list[i].offset,
					     sizeof(user_exec_list[i].offset));
			if (ret) {
				ret = -EFAULT;
				DRM_DEBUG("failed to copy %d exec entries "
					  "back to user (%d)\n",
					  args->buffer_count, ret);
				break;
			}
		}
	}

	drm_free_large(exec_list);
	drm_free_large(exec2_list);
	return ret;
}

int
i915_gem_execbuffer2(struct drm_device *dev, void *data,
		     struct drm_file *file)
{
	struct drm_i915_gem_execbuffer2 *args = data;
	struct drm_i915_gem_exec_object2 *exec2_list = NULL;
	int ret;

	if (args->buffer_count < 1 ||
	    args->buffer_count > UINT_MAX / sizeof(*exec2_list)) {
		DRM_DEBUG("execbuf2 with %d buffers\n", args->buffer_count);
		return -EINVAL;
	}

	/* Throttle batch requests per device file */
	if (i915_scheduler_file_queue_wait(file))
		return -EAGAIN;

	exec2_list = drm_malloc_gfp(args->buffer_count,
				    sizeof(*exec2_list),
				    GFP_TEMPORARY);
	if (exec2_list == NULL) {
		DRM_DEBUG("Failed to allocate exec list for %d buffers\n",
			  args->buffer_count);
		return -ENOMEM;
	}
	ret = copy_from_user(exec2_list,
			     to_user_ptr(args->buffers_ptr),
			     sizeof(*exec2_list) * args->buffer_count);
	if (ret != 0) {
		DRM_DEBUG("copy %d exec entries failed %d\n",
			  args->buffer_count, ret);
		drm_free_large(exec2_list);
		return -EFAULT;
	}

	ret = i915_gem_do_execbuffer(dev, data, file, args, exec2_list);
	if (!ret) {
		/* Copy the new buffer offsets back to the user's exec list. */
		struct drm_i915_gem_exec_object2 __user *user_exec_list =
				   to_user_ptr(args->buffers_ptr);
		int i;

		for (i = 0; i < args->buffer_count; i++) {
			exec2_list[i].offset =
				gen8_canonical_addr(exec2_list[i].offset);
			ret = __copy_to_user(&user_exec_list[i].offset,
					     &exec2_list[i].offset,
					     sizeof(user_exec_list[i].offset));
			if (ret) {
				ret = -EFAULT;
				DRM_DEBUG("failed to copy %d exec entries "
					  "back to user\n",
					  args->buffer_count);
				break;
			}
		}
	}

	drm_free_large(exec2_list);
	return ret;
}
