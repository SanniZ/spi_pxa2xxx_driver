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
#ifndef _INTEL_HUC_H_
#define _INTEL_HUC_H_

#include "intel_guc.h"

#define HUC_STATUS2		_MMIO(0xD3B0)
#define   HUC_FW_VERIFIED	(1<<7)

struct intel_huc {
	/* Generic uC firmware management */
	struct intel_uc_fw huc_fw;

	/* HuC-specific additions */
};

int huc_ucode_xfer(struct drm_device *dev);
extern void intel_huc_ucode_init(struct drm_device *dev);
extern int intel_huc_ucode_load(struct drm_device *dev, bool defer_not_exist);
extern void intel_huc_ucode_auth(struct drm_device *dev);
extern void intel_huc_ucode_fini(struct drm_device *dev);

#endif
