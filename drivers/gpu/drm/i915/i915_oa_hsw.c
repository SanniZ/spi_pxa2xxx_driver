/*
 * Autogenerated file, DO NOT EDIT manually!
 *
 * Copyright (c) 2015 Intel Corporation
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

#include "i915_drv.h"

const struct i915_oa_reg i915_oa_3d_b_counter_config_hsw[] = {
	{ 0x2724, 0x00800000 },
	{ 0x2720, 0x00000000 },
	{ 0x2714, 0x00800000 },
	{ 0x2710, 0x00000000 },
};
const int i915_oa_3d_b_counter_config_hsw_len = 4;

const struct i915_oa_reg i915_oa_3d_mux_config_hsw[] = {
	{ 0x253A4, 0x01600000 },
	{ 0x25440, 0x00100000 },
	{ 0x25128, 0x00000000 },
	{ 0x2691C, 0x00000800 },
	{ 0x26AA0, 0x01500000 },
	{ 0x26B9C, 0x00006000 },
	{ 0x2791C, 0x00000800 },
	{ 0x27AA0, 0x01500000 },
	{ 0x27B9C, 0x00006000 },
	{ 0x2641C, 0x00000400 },
	{ 0x25380, 0x00000010 },
	{ 0x2538C, 0x00000000 },
	{ 0x25384, 0x0800AAAA },
	{ 0x25400, 0x00000004 },
	{ 0x2540C, 0x06029000 },
	{ 0x25410, 0x00000002 },
	{ 0x25404, 0x5C30FFFF },
	{ 0x25100, 0x00000016 },
	{ 0x25110, 0x00000400 },
	{ 0x25104, 0x00000000 },
	{ 0x26804, 0x00001211 },
	{ 0x26884, 0x00000100 },
	{ 0x26900, 0x00000002 },
	{ 0x26908, 0x00700000 },
	{ 0x26904, 0x00000000 },
	{ 0x26984, 0x00001022 },
	{ 0x26A04, 0x00000011 },
	{ 0x26A80, 0x00000006 },
	{ 0x26A88, 0x00000C02 },
	{ 0x26A84, 0x00000000 },
	{ 0x26B04, 0x00001000 },
	{ 0x26B80, 0x00000002 },
	{ 0x26B8C, 0x00000007 },
	{ 0x26B84, 0x00000000 },
	{ 0x27804, 0x00004844 },
	{ 0x27884, 0x00000400 },
	{ 0x27900, 0x00000002 },
	{ 0x27908, 0x0E000000 },
	{ 0x27904, 0x00000000 },
	{ 0x27984, 0x00004088 },
	{ 0x27A04, 0x00000044 },
	{ 0x27A80, 0x00000006 },
	{ 0x27A88, 0x00018040 },
	{ 0x27A84, 0x00000000 },
	{ 0x27B04, 0x00004000 },
	{ 0x27B80, 0x00000002 },
	{ 0x27B8C, 0x000000E0 },
	{ 0x27B84, 0x00000000 },
	{ 0x26104, 0x00002222 },
	{ 0x26184, 0x0C006666 },
	{ 0x26284, 0x04000000 },
	{ 0x26304, 0x04000000 },
	{ 0x26400, 0x00000002 },
	{ 0x26410, 0x000000A0 },
	{ 0x26404, 0x00000000 },
	{ 0x25420, 0x04108020 },
	{ 0x25424, 0x1284A420 },
	{ 0x2541C, 0x00000000 },
	{ 0x25428, 0x00042049 },
};
const int i915_oa_3d_mux_config_hsw_len = 59;

const struct i915_oa_reg i915_oa_compute_b_counter_config_hsw[] = {
	{ 0x2710, 0x00000000 },
	{ 0x2714, 0x00800000 },
	{ 0x2718, 0xAAAAAAAA },
	{ 0x271C, 0xAAAAAAAA },
	{ 0x2720, 0x00000000 },
	{ 0x2724, 0x00800000 },
	{ 0x2728, 0xAAAAAAAA },
	{ 0x272C, 0xAAAAAAAA },
	{ 0x2740, 0x00000000 },
	{ 0x2744, 0x00000000 },
	{ 0x2748, 0x00000000 },
	{ 0x274C, 0x00000000 },
	{ 0x2750, 0x00000000 },
	{ 0x2754, 0x00000000 },
	{ 0x2758, 0x00000000 },
	{ 0x275C, 0x00000000 },
};
const int i915_oa_compute_b_counter_config_hsw_len = 16;

const struct i915_oa_reg i915_oa_compute_mux_config_hsw[] = {
	{ 0x253A4, 0x00000000 },
	{ 0x2681C, 0x01F00800 },
	{ 0x26820, 0x00001000 },
	{ 0x2781C, 0x01F00800 },
	{ 0x26520, 0x00000007 },
	{ 0x265A0, 0x00000007 },
	{ 0x25380, 0x00000010 },
	{ 0x2538C, 0x00300000 },
	{ 0x25384, 0xAA8AAAAA },
	{ 0x25404, 0xFFFFFFFF },
	{ 0x26800, 0x00004202 },
	{ 0x26808, 0x00605817 },
	{ 0x2680C, 0x10001005 },
	{ 0x26804, 0x00000000 },
	{ 0x27800, 0x00000102 },
	{ 0x27808, 0x0C0701E0 },
	{ 0x2780C, 0x000200A0 },
	{ 0x27804, 0x00000000 },
	{ 0x26484, 0x44000000 },
	{ 0x26704, 0x44000000 },
	{ 0x26500, 0x00000006 },
	{ 0x26510, 0x00000001 },
	{ 0x26504, 0x88000000 },
	{ 0x26580, 0x00000006 },
	{ 0x26590, 0x00000020 },
	{ 0x26584, 0x00000000 },
	{ 0x26104, 0x55822222 },
	{ 0x26184, 0xAA866666 },
	{ 0x25420, 0x08320C83 },
	{ 0x25424, 0x06820C83 },
	{ 0x2541C, 0x00000000 },
	{ 0x25428, 0x00000C03 },
};
const int i915_oa_compute_mux_config_hsw_len = 32;

const struct i915_oa_reg i915_oa_compute_extended_b_counter_config_hsw[] = {
	{ 0x2724, 0xf0800000 },
	{ 0x2720, 0x00000000 },
	{ 0x2714, 0xf0800000 },
	{ 0x2710, 0x00000000 },
	{ 0x2770, 0x0007fe2a },
	{ 0x2774, 0x0000ff00 },
	{ 0x2778, 0x0007fe6a },
	{ 0x277c, 0x0000ff00 },
	{ 0x2780, 0x0007fe92 },
	{ 0x2784, 0x0000ff00 },
	{ 0x2788, 0x0007fea2 },
	{ 0x278c, 0x0000ff00 },
	{ 0x2790, 0x0007fe32 },
	{ 0x2794, 0x0000ff00 },
	{ 0x2798, 0x0007fe9a },
	{ 0x279c, 0x0000ff00 },
	{ 0x27a0, 0x0007ff23 },
	{ 0x27a4, 0x0000ff00 },
	{ 0x27a8, 0x0007fff3 },
	{ 0x27ac, 0x0000fffe },
};
const int i915_oa_compute_extended_b_counter_config_hsw_len = 20;

const struct i915_oa_reg i915_oa_compute_extended_mux_config_hsw[] = {
	{ 0x2681C, 0x3EB00800 },
	{ 0x26820, 0x00900000 },
	{ 0x25384, 0x02AAAAAA },
	{ 0x25404, 0x03FFFFFF },
	{ 0x26800, 0x00142284 },
	{ 0x26808, 0x0E629062 },
	{ 0x2680C, 0x3F6F55CB },
	{ 0x26810, 0x00000014 },
	{ 0x26804, 0x00000000 },
	{ 0x26104, 0x02AAAAAA },
	{ 0x26184, 0x02AAAAAA },
	{ 0x25420, 0x00000000 },
	{ 0x25424, 0x00000000 },
	{ 0x2541C, 0x00000000 },
	{ 0x25428, 0x00000000 },
};
const int i915_oa_compute_extended_mux_config_hsw_len = 15;

const struct i915_oa_reg i915_oa_memory_reads_b_counter_config_hsw[] = {
	{ 0x2724, 0xf0800000 },
	{ 0x2720, 0x00000000 },
	{ 0x2714, 0xf0800000 },
	{ 0x2710, 0x00000000 },
	{ 0x274c, 0x76543298 },
	{ 0x2748, 0x98989898 },
	{ 0x2744, 0x000000e4 },
	{ 0x2740, 0x00000000 },
	{ 0x275c, 0x98a98a98 },
	{ 0x2758, 0x88888888 },
	{ 0x2754, 0x000c5500 },
	{ 0x2750, 0x00000000 },
	{ 0x2770, 0x0007f81a },
	{ 0x2774, 0x0000fc00 },
	{ 0x2778, 0x0007f82a },
	{ 0x277c, 0x0000fc00 },
	{ 0x2780, 0x0007f872 },
	{ 0x2784, 0x0000fc00 },
	{ 0x2788, 0x0007f8ba },
	{ 0x278c, 0x0000fc00 },
	{ 0x2790, 0x0007f87a },
	{ 0x2794, 0x0000fc00 },
	{ 0x2798, 0x0007f8ea },
	{ 0x279c, 0x0000fc00 },
	{ 0x27a0, 0x0007f8e2 },
	{ 0x27a4, 0x0000fc00 },
	{ 0x27a8, 0x0007f8f2 },
	{ 0x27ac, 0x0000fc00 },
};
const int i915_oa_memory_reads_b_counter_config_hsw_len = 28;

const struct i915_oa_reg i915_oa_memory_reads_mux_config_hsw[] = {
	{ 0x253A4, 0x34300000 },
	{ 0x25440, 0x2D800000 },
	{ 0x25444, 0x00000008 },
	{ 0x25128, 0x0E600000 },
	{ 0x25380, 0x00000450 },
	{ 0x25390, 0x00052C43 },
	{ 0x25384, 0x00000000 },
	{ 0x25400, 0x00006144 },
	{ 0x25408, 0x0A418820 },
	{ 0x2540C, 0x000820E6 },
	{ 0x25404, 0xFF500000 },
	{ 0x25100, 0x000005D6 },
	{ 0x2510C, 0x0EF00000 },
	{ 0x25104, 0x00000000 },
	{ 0x25420, 0x02108421 },
	{ 0x25424, 0x00008421 },
	{ 0x2541C, 0x00000000 },
	{ 0x25428, 0x00000000 },
};
const int i915_oa_memory_reads_mux_config_hsw_len = 18;

const struct i915_oa_reg i915_oa_memory_writes_b_counter_config_hsw[] = {
	{ 0x2724, 0xf0800000 },
	{ 0x2720, 0x00000000 },
	{ 0x2714, 0xf0800000 },
	{ 0x2710, 0x00000000 },
	{ 0x274c, 0x76543298 },
	{ 0x2748, 0x98989898 },
	{ 0x2744, 0x000000e4 },
	{ 0x2740, 0x00000000 },
	{ 0x275c, 0xbabababa },
	{ 0x2758, 0x88888888 },
	{ 0x2754, 0x000c5500 },
	{ 0x2750, 0x00000000 },
	{ 0x2770, 0x0007f81a },
	{ 0x2774, 0x0000fc00 },
	{ 0x2778, 0x0007f82a },
	{ 0x277c, 0x0000fc00 },
	{ 0x2780, 0x0007f822 },
	{ 0x2784, 0x0000fc00 },
	{ 0x2788, 0x0007f8ba },
	{ 0x278c, 0x0000fc00 },
	{ 0x2790, 0x0007f87a },
	{ 0x2794, 0x0000fc00 },
	{ 0x2798, 0x0007f8ea },
	{ 0x279c, 0x0000fc00 },
	{ 0x27a0, 0x0007f8e2 },
	{ 0x27a4, 0x0000fc00 },
	{ 0x27a8, 0x0007f8f2 },
	{ 0x27ac, 0x0000fc00 },
};
const int i915_oa_memory_writes_b_counter_config_hsw_len = 28;

const struct i915_oa_reg i915_oa_memory_writes_mux_config_hsw[] = {
	{ 0x253A4, 0x34300000 },
	{ 0x25440, 0x01500000 },
	{ 0x25444, 0x00000120 },
	{ 0x25128, 0x0C200000 },
	{ 0x25380, 0x00000450 },
	{ 0x25390, 0x00052C43 },
	{ 0x25384, 0x00000000 },
	{ 0x25400, 0x00007184 },
	{ 0x25408, 0x0A418820 },
	{ 0x2540C, 0x000820E6 },
	{ 0x25404, 0xFF500000 },
	{ 0x25100, 0x000005D6 },
	{ 0x2510C, 0x1E700000 },
	{ 0x25104, 0x00000000 },
	{ 0x25420, 0x02108421 },
	{ 0x25424, 0x00008421 },
	{ 0x2541C, 0x00000000 },
	{ 0x25428, 0x00000000 },
};
const int i915_oa_memory_writes_mux_config_hsw_len = 18;

const struct i915_oa_reg i915_oa_sampler_balance_b_counter_config_hsw[] = {
	{ 0x2740, 0x00000000 },
	{ 0x2744, 0x00800000 },
	{ 0x2710, 0x00000000 },
	{ 0x2714, 0x00800000 },
	{ 0x2720, 0x00000000 },
	{ 0x2724, 0x00800000 },
};
const int i915_oa_sampler_balance_b_counter_config_hsw_len = 6;

const struct i915_oa_reg i915_oa_sampler_balance_mux_config_hsw[] = {
	{ 0x2eb9c, 0x01906400 },
	{ 0x2fb9c, 0x01906400 },
	{ 0x253a4, 0x00000000 },
	{ 0x26b9c, 0x01906400 },
	{ 0x27b9c, 0x01906400 },
	{ 0x27104, 0x00a00000 },
	{ 0x27184, 0x00a50000 },
	{ 0x2e804, 0x00500000 },
	{ 0x2e984, 0x00500000 },
	{ 0x2eb04, 0x00500000 },
	{ 0x2eb80, 0x00000084 },
	{ 0x2eb8c, 0x14200000 },
	{ 0x2eb84, 0x00000000 },
	{ 0x2f804, 0x00050000 },
	{ 0x2f984, 0x00050000 },
	{ 0x2fb04, 0x00050000 },
	{ 0x2fb80, 0x00000084 },
	{ 0x2fb8c, 0x00050800 },
	{ 0x2fb84, 0x00000000 },
	{ 0x25380, 0x00000010 },
	{ 0x2538c, 0x000000c0 },
	{ 0x25384, 0xaa550000 },
	{ 0x25404, 0xffffc000 },
	{ 0x26804, 0x50000000 },
	{ 0x26984, 0x50000000 },
	{ 0x26b04, 0x50000000 },
	{ 0x26b80, 0x00000084 },
	{ 0x26b90, 0x00050800 },
	{ 0x26b84, 0x00000000 },
	{ 0x27804, 0x05000000 },
	{ 0x27984, 0x05000000 },
	{ 0x27b04, 0x05000000 },
	{ 0x27b80, 0x00000084 },
	{ 0x27b90, 0x00000142 },
	{ 0x27b84, 0x00000000 },
	{ 0x26104, 0xa0000000 },
	{ 0x26184, 0xa5000000 },
	{ 0x25424, 0x00008620 },
	{ 0x2541c, 0x00000000 },
	{ 0x25428, 0x0004a54a },
};
const int i915_oa_sampler_balance_mux_config_hsw_len = 40;
