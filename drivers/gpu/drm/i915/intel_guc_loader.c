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
 *    Vinit Azad <vinit.azad@intel.com>
 *    Ben Widawsky <ben@bwidawsk.net>
 *    Dave Gordon <david.s.gordon@intel.com>
 *    Alex Dai <yu.dai@intel.com>
 */
#include <linux/firmware.h>
#include "i915_drv.h"
#include "intel_guc.h"

/**
 * DOC: GuC-specific firmware loader
 *
 * intel_guc:
 * Top level structure of guc. It handles firmware loading and manages client
 * pool and doorbells. intel_guc owns a i915_guc_client to replace the legacy
 * ExecList submission.
 *
 * Firmware versioning:
 * The firmware build process will generate a version header file with major and
 * minor version defined. The versions are built into CSS header of firmware.
 * i915 kernel driver set the minimal firmware version required per platform.
 * The firmware installation package will install (symbolic link) proper version
 * of firmware.
 *
 * GuC address space:
 * GuC does not allow any gfx GGTT address that falls into range [0, WOPCM_TOP),
 * which is reserved for Boot ROM, SRAM and WOPCM. Currently this top address is
 * 512K. In order to exclude 0-512K address space from GGTT, all gfx objects
 * used by GuC is pinned with PIN_OFFSET_BIAS along with size of WOPCM.
 *
 * Firmware log:
 * Firmware log is enabled by setting i915.guc_log_level to non-negative level.
 * Log data is printed out via reading debugfs i915_guc_log_dump. Reading from
 * i915_guc_load_status will print out firmware loading status and scratch
 * registers value.
 *
 */

#define I915_SKL_GUC_UCODE "i915/skl_guc_ver6_1.bin"
MODULE_FIRMWARE(I915_SKL_GUC_UCODE);

#define I915_BXT_GUC_UCODE "i915/bxt_guc_ver8.bin"
MODULE_FIRMWARE(I915_BXT_GUC_UCODE);

/* User-friendly representation of an enum */
const char *intel_uc_fw_status_repr(enum intel_uc_fw_status status)
{
	switch (status) {
	case UC_FIRMWARE_FAIL:
		return "FAIL";
	case UC_FIRMWARE_NONE:
		return "NONE";
	case UC_FIRMWARE_PENDING:
		return "PENDING";
	case UC_FIRMWARE_SUCCESS:
		return "SUCCESS";
	default:
		return "UNKNOWN!";
	}
};

static void direct_interrupts_to_host(struct drm_i915_private *dev_priv)
{
	struct intel_engine_cs *engine;
	int i, irqs;

	/* tell all command streamers NOT to forward interrupts or vblank to GuC */
	irqs = _MASKED_FIELD(GFX_FORWARD_VBLANK_MASK, GFX_FORWARD_VBLANK_NEVER);
	irqs |= _MASKED_BIT_DISABLE(GFX_INTERRUPT_STEERING);
	for_each_engine(engine, dev_priv, i)
		I915_WRITE(RING_MODE_GEN7(engine), irqs);

	/* route all GT interrupts to the host */
	I915_WRITE(GUC_BCS_RCS_IER, 0);
	I915_WRITE(GUC_VCS2_VCS1_IER, 0);
	I915_WRITE(GUC_WD_VECS_IER, 0);
}

static void direct_interrupts_to_guc(struct drm_i915_private *dev_priv)
{
	struct intel_engine_cs *engine;
	int i, irqs;
	u32 tmp;

	/* tell all command streamers to forward interrupts (but not vblank) to GuC */
	irqs = _MASKED_BIT_ENABLE(GFX_INTERRUPT_STEERING);
	for_each_engine(engine, dev_priv, i)
		I915_WRITE(RING_MODE_GEN7(engine), irqs);

	/* route USER_INTERRUPT to Host, all others are sent to GuC. */
	irqs = GT_RENDER_USER_INTERRUPT << GEN8_RCS_IRQ_SHIFT |
	       GT_RENDER_USER_INTERRUPT << GEN8_BCS_IRQ_SHIFT;
	/* These three registers have the same bit definitions */
	I915_WRITE(GUC_BCS_RCS_IER, ~irqs);
	I915_WRITE(GUC_VCS2_VCS1_IER, ~irqs);
	I915_WRITE(GUC_WD_VECS_IER, ~irqs);

	/*
	 * If GuC has routed PM interrupts to itself, don't keep it.
	 * and keep other interrupts those are unmasked by GuC.
	*/
	tmp = I915_READ(GEN6_PMINTRMSK);
	if (tmp & GEN8_PMINTR_REDIRECT_TO_NON_DISP) {
		dev_priv->rps.pm_intr_keep |= ~(tmp & ~GEN8_PMINTR_REDIRECT_TO_NON_DISP);
		dev_priv->rps.pm_intr_keep &= ~GEN8_PMINTR_REDIRECT_TO_NON_DISP;
	}
}

static u32 get_gttype(struct drm_i915_private *dev_priv)
{
	/* XXX: GT type based on PCI device ID? field seems unused by fw */
	return 0;
}

static u32 get_core_family(struct drm_i915_private *dev_priv)
{
	switch (INTEL_INFO(dev_priv)->gen) {
	case 9:
		return GFXCORE_FAMILY_GEN9;

	default:
		DRM_ERROR("GUC: unsupported core family\n");
		return GFXCORE_FAMILY_UNKNOWN;
	}
}

static void set_guc_init_params(struct drm_i915_private *dev_priv)
{
	struct intel_guc *guc = &dev_priv->guc;
	u32 params[GUC_CTL_MAX_DWORDS];
	int i;

	memset(&params, 0, sizeof(params));

	params[GUC_CTL_DEVICE_INFO] |=
		(get_gttype(dev_priv) << GUC_CTL_GTTYPE_SHIFT) |
		(get_core_family(dev_priv) << GUC_CTL_COREFAMILY_SHIFT);

	/*
	 * GuC ARAT increment is 10 ns. GuC default scheduler quantum is one
	 * second. This ARAR is calculated by:
	 * Scheduler-Quantum-in-ns / ARAT-increment-in-ns = 1000000000 / 10
	 */
	params[GUC_CTL_ARAT_HIGH] = 0;
	params[GUC_CTL_ARAT_LOW] = 100000000;

	params[GUC_CTL_WA] |= GUC_CTL_WA_UK_BY_DRIVER;

	params[GUC_CTL_FEATURE] |= GUC_CTL_DISABLE_SCHEDULER |
			GUC_CTL_VCS2_ENABLED |
			GUC_CTL_ENABLE_CP;

	params[GUC_CTL_LOG_PARAMS] = guc->log.flags;

	if (i915.guc_log_level >= 0) {
		params[GUC_CTL_DEBUG] =
			i915.guc_log_level << GUC_LOG_VERBOSITY_SHIFT;
	} else
		params[GUC_CTL_DEBUG] = GUC_LOG_DISABLED;

	if (guc->ads_obj) {
		u32 ads = (u32)i915_gem_obj_ggtt_offset(guc->ads_obj)
				>> PAGE_SHIFT;
		params[GUC_CTL_DEBUG] |= ads << GUC_ADS_ADDR_SHIFT;
		params[GUC_CTL_DEBUG] |= GUC_ADS_ENABLED;
	}

	/* If GuC submission is enabled, set up additional parameters here */
	if (i915.enable_guc_submission) {
		u32 pgs = i915_gem_obj_ggtt_offset(dev_priv->guc.ctx_pool_obj);
		u32 ctx_in_16 = GUC_MAX_GPU_CONTEXTS / 16;

		pgs >>= PAGE_SHIFT;
		params[GUC_CTL_CTXINFO] = (pgs << GUC_CTL_BASE_ADDR_SHIFT) |
			(ctx_in_16 << GUC_CTL_CTXNUM_IN16_SHIFT);

		params[GUC_CTL_FEATURE] |= GUC_CTL_KERNEL_SUBMISSIONS;

		/* Unmask this bit to enable the GuC's internal scheduler */
		params[GUC_CTL_FEATURE] &= ~GUC_CTL_DISABLE_SCHEDULER;
	}

	I915_WRITE(SOFT_SCRATCH(0), 0);

	for (i = 0; i < GUC_CTL_MAX_DWORDS; i++)
		I915_WRITE(SOFT_SCRATCH(1 + i), params[i]);
}

/*
 * Read the GuC status register (GUC_STATUS) and store it in the
 * specified location; then return a boolean indicating whether
 * the value matches either of two values representing completion
 * of the GuC boot process.
 *
 * This is used for polling the GuC status in a wait_for()
 * loop below.
 */
static inline bool guc_ucode_response(struct drm_i915_private *dev_priv,
				      u32 *status)
{
	u32 val = I915_READ(GUC_STATUS);
	u32 uk_val = val & GS_UKERNEL_MASK;
	*status = val;
	return (uk_val == GS_UKERNEL_READY ||
		((val & GS_MIA_CORE_STATE) && uk_val == GS_UKERNEL_LAPIC_DONE));
}

/*
 * Transfer the firmware image to RAM for execution by the microcontroller.
 *
 * Architecturally, the DMA engine is bidirectional, and can potentially even
 * transfer between GTT locations. This functionality is left out of the API
 * for now as there is no need for it.
 *
 * Note that GuC needs the CSS header plus uKernel code to be copied by the
 * DMA engine in one operation, whereas the RSA signature is loaded via MMIO.
 */
static int guc_ucode_xfer_dma(struct drm_i915_private *dev_priv)
{
	struct intel_uc_fw *guc_fw = &dev_priv->guc.guc_fw;
	struct drm_i915_gem_object *fw_obj = guc_fw->uc_fw_obj;
	unsigned long offset;
	struct sg_table *sg = fw_obj->pages;
	u32 status, rsa[UOS_RSA_SCRATCH_MAX_COUNT];
	int i, ret = 0;

	/* where RSA signature starts */
	offset = guc_fw->rsa_offset;

	/* Copy RSA signature from the fw image to HW for verification */
	sg_pcopy_to_buffer(sg->sgl, sg->nents, rsa, sizeof(rsa), offset);
	for (i = 0; i < UOS_RSA_SCRATCH_MAX_COUNT; i++)
		I915_WRITE(UOS_RSA_SCRATCH(i), rsa[i]);

	/* The header plus uCode will be copied to WOPCM via DMA, excluding any
	 * other components */
	I915_WRITE(DMA_COPY_SIZE, guc_fw->header_size + guc_fw->ucode_size);

	/* Set the source address for the new blob */
	offset = i915_gem_obj_ggtt_offset(fw_obj) + guc_fw->header_offset;
	I915_WRITE(DMA_ADDR_0_LOW, lower_32_bits(offset));
	I915_WRITE(DMA_ADDR_0_HIGH, upper_32_bits(offset) & 0xFFFF);

	/*
	 * Set the DMA destination. Current uCode expects the code to be
	 * loaded at 8k; locations below this are used for the stack.
	 */
	I915_WRITE(DMA_ADDR_1_LOW, 0x2000);
	I915_WRITE(DMA_ADDR_1_HIGH, DMA_ADDRESS_SPACE_WOPCM);

	/* Finally start the DMA */
	I915_WRITE(DMA_CTRL, _MASKED_BIT_ENABLE(UOS_MOVE | START_DMA) |
			_MASKED_BIT_DISABLE(HUC_UKERNEL));

	/*
	 * Wait for the DMA to complete & the GuC to start up.
	 * NB: Docs recommend not using the interrupt for completion.
	 * Measurements indicate this should take no more than 20ms, so a
	 * timeout here indicates that the GuC has failed and is unusable.
	 * (Higher levels of the driver will attempt to fall back to
	 * execlist mode if this happens.)
	 */
	ret = wait_for(guc_ucode_response(dev_priv, &status), 100);

	DRM_DEBUG_DRIVER("DMA status 0x%x, GuC status 0x%x\n",
			I915_READ(DMA_CTRL), status);

	if ((status & GS_BOOTROM_MASK) == GS_BOOTROM_RSA_FAILED) {
		DRM_ERROR("GuC firmware signature verification failed\n");
		ret = -ENOEXEC;
	}

	DRM_DEBUG_DRIVER("returning %d\n", ret);

	return ret;
}

u32 guc_wopcm_size(struct drm_device *dev)
{
	u32 wopcm_size = GUC_WOPCM_TOP;

	/* On BXT, the top of WOPCM is reserved for RC6 context */
	if (IS_BROXTON(dev))
		wopcm_size -= BXT_GUC_WOPCM_RC6_RESERVED;

	return wopcm_size;
}

/*
 * Load the GuC firmware blob into the MinuteIA.
 */
static int guc_ucode_xfer(struct drm_i915_private *dev_priv)
{
	struct intel_uc_fw *guc_fw = &dev_priv->guc.guc_fw;
	struct drm_device *dev = dev_priv->dev;
	int ret;

	ret = i915_gem_object_set_to_gtt_domain(guc_fw->uc_fw_obj, false);
	if (ret) {
		DRM_DEBUG_DRIVER("set-domain failed %d\n", ret);
		return ret;
	}

	ret = i915_gem_obj_ggtt_pin(guc_fw->uc_fw_obj, 0, 0);
	if (ret) {
		DRM_DEBUG_DRIVER("pin failed %d\n", ret);
		return ret;
	}

	/* Invalidate GuC TLB to let GuC take the latest updates to GTT. */
	I915_WRITE(GEN8_GTCR, GEN8_GTCR_INVALIDATE);

	intel_uncore_forcewake_get(dev_priv, FORCEWAKE_ALL);

	/* Enable MIA caching. GuC clock gating is disabled. */
	I915_WRITE(GUC_SHIM_CONTROL, GUC_SHIM_CONTROL_VALUE);

	/* WaDisableMinuteIaClockGating:skl,bxt */
	if (IS_SKL_REVID(dev, 0, SKL_REVID_B0) ||
	    IS_BXT_REVID(dev, 0, BXT_REVID_A1)) {
		I915_WRITE(GUC_SHIM_CONTROL, (I915_READ(GUC_SHIM_CONTROL) &
					      ~GUC_ENABLE_MIA_CLOCK_GATING));
	}

	/* WaC6DisallowByGfxPause*/
	if (IS_SKL_REVID(dev, 0, SKL_REVID_C0) ||
	    IS_BXT_REVID(dev, 0, BXT_REVID_B0))
		I915_WRITE(GEN6_GFXPAUSE, 0x30FFF);

	if (IS_BROXTON(dev))
		I915_WRITE(GEN9LP_GT_PM_CONFIG, GT_DOORBELL_ENABLE);
	else
		I915_WRITE(GEN9_GT_PM_CONFIG, GT_DOORBELL_ENABLE);

	if (IS_GEN9(dev)) {
		/* DOP Clock Gating Enable for GuC clocks */
		I915_WRITE(GEN7_MISCCPCTL, (GEN8_DOP_CLOCK_GATE_GUC_ENABLE |
					    I915_READ(GEN7_MISCCPCTL)));

		/* allows for 5us before GT can go to RC6 */
		I915_WRITE(GUC_ARAT_C6DIS, 0x1FF);
	}

	set_guc_init_params(dev_priv);

	ret = guc_ucode_xfer_dma(dev_priv);

	intel_uncore_forcewake_put(dev_priv, FORCEWAKE_ALL);

	/*
	 * We keep the object pages for reuse during resume. But we can unpin it
	 * now that DMA has completed, so it doesn't continue to take up space.
	 */
	i915_gem_object_ggtt_unpin(guc_fw->uc_fw_obj);

	return ret;
}

static int i915_reset_guc(struct drm_i915_private *dev_priv)
{
	int ret;
	u32 guc_status;

	ret = intel_guc_reset(dev_priv);
	if (ret) {
		DRM_ERROR("GuC reset failed, ret = %d\n", ret);
		return ret;
	}

	guc_status = I915_READ(GUC_STATUS);
	WARN(!(guc_status & GS_MIA_IN_RESET),
	     "GuC status: 0x%x, MIA core expected to be in reset\n", guc_status);

	return ret;
}

/**
 * intel_guc_setup() - finish preparing the GuC for activity
 * @dev:	drm device
 *
 * Called from gem_init_hw() during driver loading and also after a GPU reset.
 *
 * The main action required here it to load the GuC uCode into the device.
 * The firmware image should have already been fetched into memory by the
 * earlier call to intel_guc_init(), so here we need only check that worked,
 * and then transfer the image to the h/w.
 *
 * Return:	non-zero code on error
 */
int intel_guc_setup(struct drm_device *dev, bool defer_on_not_exist)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_uc_fw *guc_fw = &dev_priv->guc.guc_fw;
	struct intel_uc_fw *huc_fw = &dev_priv->huc.huc_fw;
	int retries = 3, err = 0;

	if (!i915.enable_guc_submission)
		return 0;

	DRM_DEBUG_DRIVER("GuC fw status: fetch %s, load %s\n",
		intel_uc_fw_status_repr(guc_fw->fetch_status),
		intel_uc_fw_status_repr(guc_fw->load_status));

	direct_interrupts_to_host(dev_priv);
	gen9_reset_guc_interrupts(dev_priv);

	if (guc_fw->fetch_status == UC_FIRMWARE_NONE)
		return 0;

	if (guc_fw->fetch_status == UC_FIRMWARE_SUCCESS &&
		guc_fw->load_status == UC_FIRMWARE_FAIL)
			return -ENOEXEC;

	err = intel_uc_fw_fetch(dev, guc_fw);

	if (err == -ENOENT && defer_on_not_exist) {
		guc_fw->fetch_status = UC_FIRMWARE_PENDING;
		DRM_DEBUG_DRIVER("Failed to fetch GuC firmware. deferring\n");
		return -EAGAIN;
	} else if (err != 0) {
		guc_fw->fetch_status = UC_FIRMWARE_FAIL;
		DRM_ERROR(
			"Failed to complete GuC uCode load with ret %d\n", err);
		return err;
	}

	DRM_DEBUG_DRIVER("GuC fw fetch status %s\n",
		intel_uc_fw_status_repr(guc_fw->fetch_status));

	switch (guc_fw->fetch_status) {
	case UC_FIRMWARE_FAIL:
		/* something went wrong :( */
		err = -EIO;
		goto fail;

	case UC_FIRMWARE_NONE:
	case UC_FIRMWARE_PENDING:
	default:
		/* "can't happen" */
		WARN_ONCE(1, "GuC fw %s invalid fetch_status %s [%d]\n",
			guc_fw->uc_fw_path,
			intel_uc_fw_status_repr(guc_fw->fetch_status),
			guc_fw->fetch_status);
		err = -ENXIO;
		goto fail;

	case UC_FIRMWARE_SUCCESS:
		break;
	}

	/*
	 * Mask all interrupts to know which interrupts are needed by GuC.
	 * Restore host side interrupt masks post load.
	*/
	I915_WRITE(GEN6_PMINTRMSK, gen6_sanitize_rps_pm_mask(dev_priv, ~0u));

	err = i915_guc_submission_init(dev);
	if (err)
		goto fail;

	/*
	 * WaEnableuKernelHeaderValidFix:skl,bxt
	 * For BXT, this is only upto B0 but below WA is required for later
	 * steppings also so this is extended as well.
	 */
	/* WaEnableGuCBootHashCheckNotSet:skl,bxt */
	while (retries--) {
		/*
		 * Always reset the GuC just before (re)loading, so
		 * that the state and timing are fairly predictable
		 */
		err = i915_reset_guc(dev_priv);
		if (err && i915.enable_guc_submission) {
			DRM_ERROR("GuC reset failed, err %d\n", err);
			goto fail;
		}

		/* do we have the HuC firmweare loaded? */
		if (huc_fw->fetch_status == UC_FIRMWARE_SUCCESS) {
			DRM_DEBUG_DRIVER("HuC fw fetch status %s\n",
			intel_uc_fw_status_repr(huc_fw->fetch_status));

			err = huc_ucode_xfer(dev);
			if (err) {
				huc_fw->load_status = UC_FIRMWARE_FAIL;
				DRM_DEBUG_DRIVER(
				"HuC firmware failed to transfer: err = %d\n",
						err);
				goto fail;
			}

			huc_fw->load_status = UC_FIRMWARE_SUCCESS;

			DRM_DEBUG_DRIVER("%s fw status: fetch %s, load %s\n",
				huc_fw->uc_fw_path,
				intel_uc_fw_status_repr(
					huc_fw->fetch_status),
				intel_uc_fw_status_repr(
					huc_fw->load_status));
		}

		err = guc_ucode_xfer(dev_priv);
		if (!err)
			break;

		if (retries == 0)
			goto fail;

		DRM_INFO("GuC fw load failed, err %d; will reset and "
			"retry %d more time(s)\n", err, retries);
	}

	guc_fw->load_status = UC_FIRMWARE_SUCCESS;

	DRM_DEBUG_DRIVER("GuC fw status: fetch %s, load %s\n",
		intel_uc_fw_status_repr(guc_fw->fetch_status),
		intel_uc_fw_status_repr(guc_fw->load_status));

	intel_huc_ucode_auth(dev);

	if (i915.enable_guc_submission) {
		/* The execbuf_client will be recreated. Release it first. */
		i915_guc_submission_disable(dev);

		if (i915.guc_log_level >= 0)
			gen9_enable_guc_interrupts(dev_priv);

		err = i915_guc_submission_enable(dev);
		if (err)
			goto fail;
		direct_interrupts_to_guc(dev_priv);
	}

	/*
	 * Below write will ensure mask for RPS interrupts is restored back
	 * w.r.t cur_freq, particularly post reset.
	 */
	I915_WRITE(GEN6_PMINTRMSK,
		   gen6_rps_pm_mask(dev_priv, dev_priv->rps.cur_freq));

	return 0;

fail:
	DRM_ERROR("GuC firmware load failed, err %d\n", err);
	if (guc_fw->load_status == UC_FIRMWARE_PENDING)
		guc_fw->load_status = UC_FIRMWARE_FAIL;

	direct_interrupts_to_host(dev_priv);
	i915_guc_submission_disable(dev);
	i915_guc_submission_fini(dev);

	return err;
}

int intel_uc_fw_fetch(struct drm_device *dev, struct intel_uc_fw *uc_fw)
{
	struct drm_i915_gem_object *obj;
	const struct firmware *fw;
	struct uc_css_header *css;
	size_t size;
	int ret;

	DRM_DEBUG_DRIVER("before requesting firmware: uC fw fetch status %s\n",
		intel_uc_fw_status_repr(uc_fw->fetch_status));

	/*
	 * direct as we are not sure if the partition that the firmware is
	 * on is loaded yet. So save 60 seconds. If the firmware is not on
	 * the system, then it's a failure.
	 */
	ret = request_firmware_direct(&fw, uc_fw->uc_fw_path,
				      &dev->pdev->dev);
	if (ret < 0)
		goto fail;

	if (WARN_ON(fw == NULL)) {
		ret = -EINVAL;
		goto fail;
	}

	DRM_DEBUG_DRIVER("fetch uC fw from %s succeeded, fw %p\n",
		uc_fw->uc_fw_path, fw);

	/* Check the size of the blob before examining buffer contents */
	if (fw->size < sizeof(struct uc_css_header)) {
		DRM_ERROR("Firmware header is missing\n");
		ret = -EINVAL;
		goto fail;
	}

	css = (struct uc_css_header *)fw->data;

	/* Firmware bits always start from header */
	uc_fw->header_offset = 0;
	uc_fw->header_size = (css->header_size_dw - css->modulus_size_dw -
		css->key_size_dw - css->exponent_size_dw) * sizeof(u32);

	if (uc_fw->header_size != sizeof(struct uc_css_header)) {
		DRM_ERROR("CSS header definition mismatch\n");
		ret = -EINVAL;
		goto fail;
	}

	/* then, uCode */
	uc_fw->ucode_offset = uc_fw->header_offset + uc_fw->header_size;
	uc_fw->ucode_size = (css->size_dw - css->header_size_dw) * sizeof(u32);

	/* now RSA */
	if (css->key_size_dw != UOS_RSA_SCRATCH_MAX_COUNT) {
		DRM_ERROR("RSA key size is bad\n");
		ret = -EINVAL;
		goto fail;
	}
	uc_fw->rsa_offset = uc_fw->ucode_offset + uc_fw->ucode_size;
	uc_fw->rsa_size = css->key_size_dw * sizeof(u32);

	/* At least, it should have header, uCode and RSA. Size of all three. */
	size = uc_fw->header_size + uc_fw->ucode_size + uc_fw->rsa_size;
	if (fw->size < size) {
		DRM_ERROR("Missing firmware components\n");
		ret = -EINVAL;
		goto fail;
	}

	/*
	 * The uC firmware image has the version number embedded at a well-known
	 * offset within the firmware blob; note that major / minor version are
	 * TWO bytes each (i.e. u16), although all pointers and offsets are defined
	 * in terms of bytes (u8).
	 */
	switch (uc_fw->fw_type) {
	case UC_FW_TYPE_GUC:
		/* Header and uCode will be loaded to WOPCM. Size of the two. */
		size = uc_fw->header_size + uc_fw->ucode_size;
		if (size > guc_wopcm_size(dev)) {
			DRM_ERROR("Firmware is too large to fit in WOPCM\n");
			ret = -EINVAL;
			goto fail;
		}

		uc_fw->major_ver_found = css->guc_sw_version >> 16;
		uc_fw->minor_ver_found = css->guc_sw_version & 0xFFFF;
		break;
	case UC_FW_TYPE_HUC:
		uc_fw->major_ver_found = css->huc_sw_version >> 16;
		uc_fw->minor_ver_found = css->huc_sw_version & 0xFFFF;
		break;
	default:
		DRM_ERROR("Unknown firmware type %d\n", uc_fw->fw_type);
		ret = -ENOEXEC;
		goto fail;
	}

	if (uc_fw->major_ver_found != uc_fw->major_ver_wanted ||
	    uc_fw->minor_ver_found < uc_fw->minor_ver_wanted) {
		DRM_ERROR("GuC firmware version %d.%d, required %d.%d\n",
			uc_fw->major_ver_found, uc_fw->minor_ver_found,
			uc_fw->major_ver_wanted, uc_fw->minor_ver_wanted);
		ret = -ENOEXEC;
		goto fail;
	}

	DRM_DEBUG_DRIVER("firmware version %d.%d OK (minimum %d.%d)\n",
			uc_fw->major_ver_found, uc_fw->minor_ver_found,
			uc_fw->major_ver_wanted, uc_fw->minor_ver_wanted);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));
	obj = i915_gem_object_create_from_data(dev, fw->data, fw->size);
	if (IS_ERR_OR_NULL(obj)) {
		ret = obj ? PTR_ERR(obj) : -ENOMEM;
		goto fail;
	}

	uc_fw->uc_fw_obj = obj;
	uc_fw->uc_fw_size = fw->size;

	DRM_DEBUG_DRIVER("GuC fw fetch status SUCCESS, obj %p\n",
			uc_fw->uc_fw_obj);

	release_firmware(fw);
	uc_fw->fetch_status = UC_FIRMWARE_SUCCESS;
	return 0;

fail:
	DRM_DEBUG_DRIVER("GuC fw fetch status FAIL; ret %d, fw %p, obj %p\n",
		ret, fw, uc_fw->uc_fw_obj);
	DRM_ERROR("Failed to fetch GuC firmware from %s (error %d)\n",
		  uc_fw->uc_fw_path, ret);

	obj = uc_fw->uc_fw_obj;
	if (obj)
		drm_gem_object_unreference(&obj->base);
	uc_fw->uc_fw_obj = NULL;

	release_firmware(fw);		/* OK even if fw is NULL */
	uc_fw->fetch_status = UC_FIRMWARE_FAIL;

	return ret;
}

/**
 * intel_guc_init() - define parameters and fetch firmware
 * @dev:	drm device
 *
 * Called early during driver load, but after GEM is initialised.
 *
 * The firmware will be transferred to the GuC's memory later,
 * when intel_guc_setup() is called.
 */
void intel_guc_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_uc_fw *guc_fw = &dev_priv->guc.guc_fw;
	const char *fw_path = NULL;

	guc_fw->uc_dev = dev;
	guc_fw->uc_fw_path = NULL;
	guc_fw->fetch_status = UC_FIRMWARE_NONE;
	guc_fw->load_status = UC_FIRMWARE_NONE;

	if (!HAS_GUC_SCHED(dev))
		i915.enable_guc_submission = false;

	if (!HAS_GUC_UCODE(dev))
		return;

	if (IS_SKYLAKE(dev)) {
		fw_path = I915_SKL_GUC_UCODE;
		guc_fw->major_ver_wanted = 4;
		guc_fw->minor_ver_wanted = 3;
	} else if (IS_BROXTON(dev)) {
		fw_path = I915_BXT_GUC_UCODE;
		guc_fw->major_ver_wanted = 8;
		guc_fw->minor_ver_wanted = 7;
	}

	if (fw_path == NULL)
		return;

	guc_fw->uc_fw_path = fw_path;
	guc_fw->fetch_status = UC_FIRMWARE_PENDING;
	DRM_DEBUG_DRIVER("GuC firmware pending, path %s\n", fw_path);
}

/**
 * intel_guc_fini() - clean up all allocated resources
 * @dev:	drm device
 */
void intel_guc_fini(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_uc_fw *guc_fw = &dev_priv->guc.guc_fw;

	mutex_lock(&dev->struct_mutex);
	direct_interrupts_to_host(dev_priv);
	i915_guc_submission_disable(dev);
	i915_guc_submission_fini(dev);

	if (guc_fw->uc_fw_obj)
		drm_gem_object_unreference(&guc_fw->uc_fw_obj->base);
	guc_fw->uc_fw_obj = NULL;
	mutex_unlock(&dev->struct_mutex);

	guc_fw->fetch_status = UC_FIRMWARE_NONE;
}
