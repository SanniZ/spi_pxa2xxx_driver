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
#include "i915_drv.h"
#include "intel_huc.h"

/**
 * DOC: HuC Firmware
 *
 * Motivation:
 * GEN9 introduces a new dedicated firmware for usage in media HEVC (High
 * Efficiency Video Coding) operations. Userspace can use the firmware
 * capabilities by adding HuC specific commands to batch buffers.
 *
 * Implementation:
 * On supported platforms, i915's job is to load the firmware stored on the
 * file system and assist with authentication. It is up to userspace to
 * detect the presence of HuC support on a platform, on their own.
 * For debugging, i915 provides a debugfs file, i915_huc_load_status_info
 * which displays the firmware load status.
 *
 * The unified uC firmware loader is used. Firmware binary is fetched by the
 * loader asynchronously from the driver init process. However, the actual
 * loading to HW is deferred until GEM initialization is done. Be note that HuC
 * firmware loading must be done before GuC loading.
 */

#define I915_SKL_HUC_UCODE "i915/skl_huc_ver1.bin"
MODULE_FIRMWARE(I915_SKL_HUC_UCODE);

#define I915_BXT_HUC_UCODE "i915/huc_gen9lp.bin"
MODULE_FIRMWARE(I915_BXT_HUC_UCODE);

/**
 * intel_huc_load_ucode() - DMA's the firmware
 * @dev: the drm device
 *
 * This function takes the gem object containing the firmware, sets up the DMA
 * engine MMIO, triggers the DMA operation and waits for it to finish.
 *
 * Transfer the firmware image to RAM for execution by the microcontroller.
 *
 * Return: 0 on success, non-zero on failure
 */

int huc_ucode_xfer(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_uc_fw *huc_fw = &dev_priv->huc.huc_fw;
	unsigned long offset = 0;
	u32 size;
	int ret;

	WARN_ON(!mutex_is_locked(&dev_priv->dev->struct_mutex));

	if (huc_fw->fetch_status != UC_FIRMWARE_SUCCESS) {
		DRM_DEBUG_DRIVER("HuC fw not loaded - can't xfer\n");
		return -ENOEXEC;
	}

	ret = i915_gem_object_set_to_gtt_domain(huc_fw->uc_fw_obj, false);
	if (ret) {
		DRM_DEBUG_DRIVER("set-domain failed %d\n", ret);
		return ret;
	}

	ret = i915_gem_obj_ggtt_pin(huc_fw->uc_fw_obj, 0, 0);
	if (ret) {
		DRM_DEBUG_DRIVER("pin failed %d\n", ret);
		return ret;
	}

	/* Invalidate GuC TLB to let GuC take the latest updates to GTT. */
	I915_WRITE(GEN8_GTCR, GEN8_GTCR_INVALIDATE);

	intel_uncore_forcewake_get(dev_priv, FORCEWAKE_ALL);

	/* Set the source address for the uCode */
	offset = i915_gem_obj_ggtt_offset(huc_fw->uc_fw_obj) +
			huc_fw->header_offset;
	I915_WRITE(DMA_ADDR_0_LOW, lower_32_bits(offset));
	I915_WRITE(DMA_ADDR_0_HIGH, upper_32_bits(offset) & 0xFFFF);

	/* Hardware doesn't look at destination address for HuC. Set it to 0,
	 * but still program the correct address space.
	 */
	I915_WRITE(DMA_ADDR_1_LOW, 0);
	I915_WRITE(DMA_ADDR_1_HIGH, DMA_ADDRESS_SPACE_WOPCM);

	size = huc_fw->header_size + huc_fw->ucode_size;
	I915_WRITE(DMA_COPY_SIZE, size);

	/* Start the DMA */
	I915_WRITE(DMA_CTRL, _MASKED_BIT_ENABLE(HUC_UKERNEL | START_DMA));

	/* Wait for DMA to finish */
	ret = wait_for((I915_READ(DMA_CTRL) & START_DMA) == 0, 500);

	DRM_DEBUG_DRIVER("HuC DMA transfer wait over with ret %d\n", ret);

	intel_uncore_forcewake_put(dev_priv, FORCEWAKE_ALL);

	/*
	 * We keep the object pages for reuse during resume. But we can unpin it
	 * now that DMA has completed, so it doesn't continue to take up space.
	 */
	i915_gem_object_ggtt_unpin(huc_fw->uc_fw_obj);

	return ret;
}

/**
 * intel_huc_ucode_init() - initiate HuC firmware loading request
 * @dev: the drm device
 *
 * Called early during driver load, but after GEM is initialised. The loading
 * will continue only when driver explicitly specify firmware name and version.
 * All other cases are considered as UC_FIRMWARE_NONE either because HW is not
 * capable or driver yet support it. And there will be no error message for
 * UC_FIRMWARE_NONE cases.
 *
 * The DMA-copying to HW is done later when intel_huc_ucode_load() is called.
 */
void intel_huc_ucode_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_huc *huc = &dev_priv->huc;
	struct intel_uc_fw *huc_fw = &huc->huc_fw;
	const char *fw_path = NULL;

	huc_fw->uc_dev = dev;
	huc_fw->uc_fw_path = NULL;
	huc_fw->fetch_status = UC_FIRMWARE_NONE;
	huc_fw->load_status = UC_FIRMWARE_NONE;
	huc_fw->fw_type = UC_FW_TYPE_HUC;

	if (!HAS_HUC_UCODE(dev))
		return;

	if (IS_SKYLAKE(dev)) {
		fw_path = I915_SKL_HUC_UCODE;
		huc_fw->major_ver_wanted = 1;
		huc_fw->minor_ver_wanted = 5;
	} else if (IS_BROXTON(dev)) {
		fw_path = I915_BXT_HUC_UCODE;
		huc_fw->major_ver_wanted = 1;
		huc_fw->minor_ver_wanted = 7;
	} else {
		DRM_ERROR("Unexpected: no known firmware for platform\n");
	}

	if (fw_path == NULL)
		return;

	huc_fw->uc_fw_path = fw_path;
	huc_fw->fetch_status = UC_FIRMWARE_PENDING;

	DRM_DEBUG_DRIVER("HuC firmware pending, path %s\n", fw_path);
}

/**
 * intel_huc_ucode_load() - load HuC uCode to device
 * @dev: the drm device
 *
 * Called from gem_init_hw() during driver loading and also after a GPU reset.
 * Be note that HuC loading must be done before GuC loading.
 *
 * The firmware image should have already been fetched into memory by the
 * earlier call to intel_huc_ucode_init(), so here we need only check that
 * is succeeded, and then transfer the image to the h/w.
 *
 * Return:	non-zero code on error
 */
int intel_huc_ucode_load(struct drm_device *dev, bool defer_on_not_exist)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_uc_fw *huc_fw = &dev_priv->huc.huc_fw;
	int err = 0;

	if (huc_fw->fetch_status == UC_FIRMWARE_NONE)
		return 0;

	DRM_DEBUG_DRIVER("%s fw status: fetch %s, load %s\n",
		huc_fw->uc_fw_path,
		intel_uc_fw_status_repr(huc_fw->fetch_status),
		intel_uc_fw_status_repr(huc_fw->load_status));

	if (huc_fw->fetch_status == UC_FIRMWARE_SUCCESS &&
	    huc_fw->load_status == UC_FIRMWARE_FAIL)
		return -ENOEXEC;

	/* Only fetch the firmware if we have not fetched it before */
	if (huc_fw->fetch_status != UC_FIRMWARE_SUCCESS)
		err = intel_uc_fw_fetch(dev, huc_fw);

	if (err == -ENOENT && defer_on_not_exist) {
		huc_fw->fetch_status = UC_FIRMWARE_PENDING;
		DRM_DEBUG_DRIVER("Failed to fetch HuC firmware. deferring\n");
		return -EAGAIN;
	} else if (err != 0) {
		huc_fw->fetch_status = UC_FIRMWARE_FAIL;
		DRM_ERROR(
			"Failed to complete GuC uCode load with ret %d\n", err);
		return err;
	}

	switch (huc_fw->fetch_status) {
	case UC_FIRMWARE_FAIL:
		/* something went wrong :( */
		err = -EIO;
		goto fail;

	case UC_FIRMWARE_NONE:
	case UC_FIRMWARE_PENDING:
	default:
		/* "can't happen" */
		WARN_ONCE(1, "HuC fw %s invalid fetch_status %s [%d]\n",
			huc_fw->uc_fw_path,
			intel_uc_fw_status_repr(huc_fw->fetch_status),
			huc_fw->fetch_status);
		err = -ENXIO;
		goto fail;

	case UC_FIRMWARE_SUCCESS:
		break;
	}

	huc_fw->fetch_status = UC_FIRMWARE_SUCCESS;

	return 0;

fail:
	if (huc_fw->load_status == UC_FIRMWARE_PENDING)
		huc_fw->load_status = UC_FIRMWARE_FAIL;

	DRM_ERROR("Failed to complete HuC uCode load with ret %d\n", err);

	return err;
}

/**
 * intel_huc_ucode_fini() - clean up resources allocated for HuC
 * @dev: the drm device
 *
 * Cleans up by releasing the huc firmware GEM obj.
 */
void intel_huc_ucode_fini(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_uc_fw *huc_fw = &dev_priv->huc.huc_fw;

	mutex_lock(&dev->struct_mutex);
	if (huc_fw->uc_fw_obj)
		drm_gem_object_unreference(&huc_fw->uc_fw_obj->base);
	huc_fw->uc_fw_obj = NULL;
	mutex_unlock(&dev->struct_mutex);

	huc_fw->fetch_status = UC_FIRMWARE_NONE;
}
