/*
 * Intel ATOM SOC Telemetry Driver
 * Copyright (c) 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/io.h>

#include <asm/cpu_device_id.h>
#include <asm/intel_pmc_ipc.h>
#include <asm/intel_punit_ipc.h>
#include <asm/atom_soc_telemetry.h>

#define	DRIVER_NAME	KBUILD_MODNAME

#define TELEM_MIN_PERIOD(x)		((x) & 0x7F0000)
#define TELEM_MAX_PERIOD(x)		((x) & 0x7F000000)

#define TELEM_SAMPLING_DEFAULT_PERIOD	0xD

#define TELEM_PSS_EVENTS		155
#define TELEM_IOSS_EVENTS		150

#define TELEM_MAX_EVENTS_SRAM		28
#define TELEM_MAX_OS_ALLOCATED_EVENTS	20

#define TELEM_SSRAM_START		0xFE900000
#define TELEM_SSRAM_SIZE		240
#define IOSS_TELEM_SSRAM_OFFSET		0x1B00
#define PSS_TELEM_SSRAM_OFFSET		0x1A00
#define TELEM_SSRAM_STARTTIME_OFFSET	8
#define TELEM_SSRAM_EVTLOG_OFFSET	16

#define IOSS_TELEM_EVENT_READ		0x0
#define IOSS_TELEM_EVENT_WRITE		0x1
#define IOSS_TELEM_INFO_READ		0x2
#define IOSS_TELEM_EVENT_CTL_READ	0x7
#define IOSS_TELEM_EVENT_CTL_WRITE	0x8
#define IOSS_TELEM_EVT_CTRL_WRITE_SIZE	0x4
#define IOSS_TELEM_READ_FOURBYTES	0x1
#define IOSS_TELEM_EVT_WRITE_SIZE	0x3

#define TELEM_INFO_SRAMEVTS_MASK	0xFF00
#define TELEM_INFO_SRAMEVTS_SHIFT	0x8
#define TELEM_SSRAM_READ_TIMEOUT	10

#define TELEM_INFO_NENABLES_MASK	0xFF
#define TELEM_EVENT_ENABLE		0x8000

#define TELEM_PSS_IDLE_ID		0x2806
#define TELEM_PCS_IDLE_BLOCKED_ID	0x2C00
#define TELEM_PCS_S0IX_BLOCKED_ID	0x2C01
#define TELEM_PSS_WAKEUP_ID		0x2C02
#define TELEM_PSS_LTR_BLOCKING_ID	0x2C03

#define TELEM_PSS_IDLE_EVTS		15
#define TELEM_PSS_IDLE_BLOCKED_EVTS	7
#define TELEM_PSS_S0IX_BLOCKED_EVTS	8
#define TELEM_PSS_S0IX_WAKEUP_EVTS	7
#define TELEM_PSS_LTR_BLOCKING_EVTS	7

#define TELEM_PMC_D0IX_EVTID		0x581A
#define TELEM_PMC_D3_EVTID		0x5819
#define TELEM_PMC_DX_D0IX_EVTS		12

#define TELEM_MASK_BIT			1
#define TELEM_MASK_BYTE			0xFF
#define BYTES_PER_LONG			8

#define TELEM_DISABLE(x)		((x) &= ~(BIT(31)))
#define TELEM_CLEAR_EVENTS(x)		((x) |= (BIT(30)))
#define TELEM_ENABLE_SRAM_EVT_TRACE(x)	((x) &= ~(BIT(30) | BIT(24)))
#define TELEM_ENABLE_PERIODIC(x)	((x) |= (BIT(23) | BIT(31) | BIT(7)))

enum telemetry_action {
	TELEM_UPDATE = 0,
	TELEM_ADD,
	TELEM_RESET,
	TELEM_ACTION_NONE
};

struct telemetry_evtmap {
	const char *name;
	u32 evt_id;
};

struct telemetry_config {
	u8 ssram_iossevts_used;
	u8 ssram_pssevts_used;
	u8 pss_curr_period;
	u8 pss_max_period;
	u8 pss_min_period;
	u8 ioss_curr_period;
	u8 ioss_max_period;
	u8 ioss_min_period;
	u8 telem_in_use;
	u32 ssram_base_addr;
	void __iomem *pss_regmap;
	void __iomem *ioss_regmap;
	struct dentry *telemetry_dbg_dir;
	struct telemetry_evtmap
		pss_telem_evts[TELEM_MAX_EVENTS_SRAM];
	struct telemetry_evtmap
		ioss_telem_evts[TELEM_MAX_EVENTS_SRAM];
};

static struct telemetry_config *telm_conf;


/*  Only 20 allocated to kernel driver. DO NOT EXCEED  */
static const struct telemetry_evtmap
	telemetry_bxt_ioss_default_events[TELEM_MAX_OS_ALLOCATED_EVENTS] = {
	{"SOC_S0IX_TOTAL_RES",			0x4800},
	{"SOC_S0IX_TOTAL_OCC",			0x4000},
	{"SOC_S0IX_SHALLOW_RES",		0x4801},
	{"SOC_S0IX_SHALLOW_OCC",		0x4001},
	{"SOC_S0IX_DEEP_RES",			0x4802},
	{"SOC_S0IX_DEEP_OCC",			0x4002},
	{"PMC_POWER_GATE",			0x5818},
	{"PMC_D3_STATES",			0x5819},
	{"PMC_D0I3_STATES",			0x581A},
	{"PMC_S0IX_WAKE_REASON_GPIO",		0x6000},
	{"PMC_S0IX_WAKE_REASON_TIMER",		0x6001},
	{"PMC_S0IX_WAKE_REASON_LOWPOWER",	0x6002},
	{"PMC_S0IX_WAKE_REASON_EVENTS_EXT",	0x6003},
	{"PMC_S0IX_WAKE_REASON_MISC",		0x6004},
	{"PMC_S0IX_BLOCKING_IPS",		0x6005},
	{"PMC_S0IX_ABORT_CAUSES",		0x6006},
	{"PMC_S0IX_BLOCK_REASONS",		0x6007},
	{"ISH_VNN_REQ_RES",			0x0008},
	{"AVS_VNN_REQ_RES",			0x0009},
	{"CSE_IDLE_RES",			0x000A},
};


/*  Only 20 allocated to kernel driver. DO NOT EXCEED  */
static const struct telemetry_evtmap
	telemetry_bxt_pss_default_events[TELEM_MAX_OS_ALLOCATED_EVENTS] = {
	{"IA_CORE_C6_RES",			0x0400},
	{"IA_CORE_C6_CTR",			0x0000},
	{"IA_MODULE_C7_RES",			0x0410},
	{"IA_MODULE_C7_CTR",			0x000E},
	{"IA_C0_RES",				0x0805},
	{"PCS_LTR",				0x2001},
	{"PSTATES",				0x2002},
	{"SOC_S0I3_RES",			0x0409},
	{"SOC_S0I3_CTR",			0x000A},
	{"PCS_S0I3_CTR",			0x0009},
	{"PCS_C1E_RES",				0x041A},
	{"PCS_IDLE_STATUS",			0x2806},
	{"IA_PERF_LIMITS",			0x280B},
	{"GT_PERF_LIMITS",			0x280C},
	{"PCS_WAKEUP_S0IX_CTR",			0x0030},
	{"PCS_IDLE_BLOCKED",			0x2C00},
	{"PCS_S0IX_BLOCKED",			0x2C01},
	{"PCS_S0IX_WAKE_REASONS",		0x2C02},
	{"PCS_LTR_BLOCKING",			0x2C03},
	{"PC2_AND_MEM_SHALLOW_IDLE_RES",	0x1D40},
};

/* BXT specific Data */
static const struct telemetry_config telem_bxt_config = {
	.ssram_base_addr = TELEM_SSRAM_START,
};

/*Serialize multiple calls to Telemetry*/
static DEFINE_MUTEX(telem_lock);

#define TELEM_CPU(model, cpu) \
	{ X86_VENDOR_INTEL, 6, model, X86_FEATURE_MWAIT, (unsigned long)&cpu }


static const struct x86_cpu_id telemetry_cpu_ids[] = {
	TELEM_CPU(0x5c, telem_bxt_config),
	{}
};

MODULE_DEVICE_TABLE(x86cpu, telemetry_cpu_ids);

struct telem_ssram_region {
	u64 timestamp;
	u64 start_time;
	u64 events[TELEM_MAX_EVENTS_SRAM];
};

struct telem_pss_idle_stateinfo {
	const char *name;
	u32 bit_pos;
};

static struct telem_pss_idle_stateinfo telem_pss_idle_data[] = {
	{"IA_CORE0_C1E",		0},
	{"IA_CORE1_C1E",		1},
	{"IA_CORE2_C1E",		2},
	{"IA_CORE3_C1E",		3},
	{"IA_CORE0_C6",			16},
	{"IA_CORE1_C6",			17},
	{"IA_CORE2_C6",			18},
	{"IA_CORE3_C6",			19},
	{"IA_MODULE0_C7",		32},
	{"IA_MODULE1_C7",		33},
	{"GT_RC6",			40},
	{"IUNIT_PROCESSING_IDLE",	41},
	{"FAR_MEM_IDLE",		43},
	{"DISPLAY_IDLE",		44},
	{"IUNIT_INPUT_SYSTEM_IDLE",	45},
};

struct telem_pcs_blocked_info {
	const char *name;
	u32 bit_pos;
};

static struct telem_pcs_blocked_info telem_pcs_idle_blocked_data[] = {
	{"COMPUTE",			0},
	{"MISC",			8},
	{"MODULE_ACTIONS_PENDING",	16},
	{"LTR",				24},
	{"DISPLAY_WAKE",		32},
	{"ISP_WAKE",			40},
	{"PSF0_ACTIVE",			48},
};


static struct telem_pcs_blocked_info telem_pcs_s0ix_blocked_data[] = {
	{"LTR",				0},
	{"IRTL",			8},
	{"WAKE_DEADLINE_PENDING",	16},
	{"DISPLAY",			24},
	{"ISP",				32},
	{"CORE",			40},
	{"PMC",				48},
	{"MISC",			56},
};

struct telem_pss_ltr_info {
	const char *name;
	u32 bit_pos;
};

static struct telem_pss_ltr_info telem_pss_ltr_data[] = {
	{"CORE_ACTIVE",		0},
	{"MEM_UP",		8},
	{"DFX",			16},
	{"DFX_FORCE_LTR",	24},
	{"DISPLAY",		32},
	{"ISP",			40},
	{"SOUTH",		48},
};

struct telem_pss_wakeup_info {
	const char *name;
	u32 bit_pos;
};

static struct telem_pss_wakeup_info telem_pss_wakeup[] = {
	{"IP_IDLE_ABORT",		0},
	{"DISPLAY_ACTIVE_ABORT",	8},
	{"VOLTAGE_REG_INT_ABORT",	16},
	{"HOT_PLUG_DETECT",		24},
	{"CORE_WAKE",			32},
	{"MISC_S0IX",			40},
	{"MISC_ABORT",			56},
};

struct telem_ioss_d0ix_stateinfo {
	const char *name;
	u32 bit_pos;
};

static struct telem_ioss_d0ix_stateinfo telem_ioss_d0ix_data[] = {
	{"CSE",		0},
	{"HOFFL",	1},
	{"GMM",		2},
	{"XDCI",	3},
	{"XHCI",	4},
	{"ISH",		5},
	{"AVS",		6},
	{"MEXP1",	7},
	{"MEXP0",	8},
	{"LPSS",	9},
	{"SCC",		10},
	{"PWM",		11},
};

static int telem_pss_evtlog_read(struct telem_ssram_region *pss_ssram_region,
	u8 len)
{
	struct telemetry_config *telem_conf = telm_conf;
	int index, timeout = 0;
	u64 timestamp_prev, timestamp_next;

	if (len > (telem_conf->ssram_pssevts_used))
		len = telem_conf->ssram_pssevts_used;

	do {
		timestamp_prev = readq(telem_conf->pss_regmap);
		if (!timestamp_prev) {
			pr_err("PSS SSRAM under update. Please Try Later!!\n");
			return -EBUSY;
		}
		pr_debug("PSS SSRAM timestamp_prev %llx\n",
			timestamp_prev);

		pss_ssram_region->start_time =
			readq(telem_conf->pss_regmap +
				TELEM_SSRAM_STARTTIME_OFFSET);
		pr_debug("PSS SSRAM start_time %llx\n",
				pss_ssram_region->start_time);

		for (index = 0; index < len; index++) {
			pss_ssram_region->events[index] =
				readq(telem_conf->pss_regmap +
					TELEM_SSRAM_EVTLOG_OFFSET +
						BYTES_PER_LONG*index);

			pr_debug("PSS SSRAM index %x, value %llx\n",
				index, pss_ssram_region->events[index]);
		}

		timestamp_next = readq(telem_conf->pss_regmap);
		if (!timestamp_next) {
			pr_err("PSS SSRAM under update. Please Try Later!!\n");
			return -EBUSY;
		}
		pr_debug("PSS SSRAM timestamp_next %llx\n",
			timestamp_next);

		if (timeout++ > TELEM_SSRAM_READ_TIMEOUT) {
			pr_err("Timeout while reading IOSS Events!!\n");
			break;
		}

	} while (timestamp_prev != timestamp_next);

	pss_ssram_region->timestamp = timestamp_next;

	return len;
}

static int telem_pss_states_show(struct seq_file *s, void *unused)
{
	struct telemetry_config *telem_conf = telm_conf;
	struct telem_ssram_region pss_ssram_region;
	int index, idx, ret = 0;
	u32 pss_idle[TELEM_PSS_IDLE_EVTS],
		pcs_idle_blocked[TELEM_PSS_IDLE_BLOCKED_EVTS],
		pcs_s0ix_blocked[TELEM_PSS_S0IX_BLOCKED_EVTS],
		pss_s0ix_wakeup[TELEM_PSS_S0IX_WAKEUP_EVTS],
		pss_ltr_blocked[TELEM_PSS_LTR_BLOCKING_EVTS];

	mutex_lock(&telem_lock);
	ret = telem_pss_evtlog_read(&pss_ssram_region,
		TELEM_MAX_OS_ALLOCATED_EVENTS);
	mutex_unlock(&telem_lock);

	if (ret < 0)
		return ret;

	seq_puts(s, "-----------------------------------------\n");
	seq_puts(s, "\t\tPSS TELEM EVENTLOG\n");
	seq_puts(s, "-----------------------------------------\n");
	for (index = 0; index < ret; index++) {
		seq_printf(s, "%-32s %llx\n",
			telem_conf->pss_telem_evts[index].name,
				pss_ssram_region.events[index]);

		if (telem_conf->pss_telem_evts[index].evt_id ==
						TELEM_PSS_IDLE_ID) {
			for (idx = 0; idx < TELEM_PSS_IDLE_EVTS; idx++) {
				pss_idle[idx] =
				pss_ssram_region.events[index] &
				(TELEM_MASK_BIT <<
					(telem_pss_idle_data[idx].bit_pos));

				if (pss_idle[idx])
					pss_idle[idx] = 1;
			}
		}

		if (telem_conf->pss_telem_evts[index].evt_id ==
					TELEM_PCS_IDLE_BLOCKED_ID) {
			for (idx = 0; idx < TELEM_PSS_IDLE_BLOCKED_EVTS;
									idx++) {
				pcs_idle_blocked[idx] =
				pss_ssram_region.events[index] &
				(TELEM_MASK_BYTE <<
				(telem_pcs_idle_blocked_data[idx].bit_pos));
			}
		}

		if (telem_conf->pss_telem_evts[index].evt_id ==
					TELEM_PCS_S0IX_BLOCKED_ID) {
			for (idx = 0; idx < TELEM_PSS_S0IX_BLOCKED_EVTS;
								idx++) {
				pcs_s0ix_blocked[idx] =
				pss_ssram_region.events[index] &
				(TELEM_MASK_BYTE <<
				(telem_pcs_s0ix_blocked_data[idx].bit_pos));
			}
		}


		if (telem_conf->pss_telem_evts[index].evt_id ==
						TELEM_PSS_WAKEUP_ID) {
			for (idx = 0; idx < TELEM_PSS_S0IX_WAKEUP_EVTS; idx++) {
				pss_s0ix_wakeup[idx] =
				pss_ssram_region.events[index] &
				(TELEM_MASK_BYTE <<
					(telem_pss_wakeup[idx].bit_pos));
			}
		}

		if (telem_conf->pss_telem_evts[index].evt_id ==
					TELEM_PSS_LTR_BLOCKING_ID) {
			for (idx = 0; idx < TELEM_PSS_LTR_BLOCKING_EVTS;
									idx++) {
				pss_ltr_blocked[idx] =
				pss_ssram_region.events[index] &
				(TELEM_MASK_BYTE <<
					(telem_pss_ltr_data[idx].bit_pos));
			}
		}

	}

	seq_puts(s, "\n--------------------------------------\n");
	seq_puts(s, "\tPSS IDLE Status\n");
	seq_puts(s, "\tDevice\t\tIDLE\n");
	for (index = 0; index < TELEM_PSS_IDLE_EVTS; index++) {
		seq_printf(s, "%-32s\t\t%u\n",
			telem_pss_idle_data[index].name,
			pss_idle[index]);
	}

	seq_puts(s, "\tPSS Idle blocked Status (~1ms saturating bucket)\n");
	seq_puts(s, "\tBlocker\t\tDuration\n");
	for (index = 0; index < TELEM_PSS_IDLE_BLOCKED_EVTS; index++) {
		seq_printf(s, "%-32s\t\t%u\n",
			telem_pcs_idle_blocked_data[index].name,
			pcs_idle_blocked[index]);
	}

	seq_puts(s, "\tPSS S0ix blocked Status (~1ms saturating bucket)\n");
	seq_puts(s, "\tBlocker\t\tDuration\n");
	for (index = 0; index < TELEM_PSS_S0IX_BLOCKED_EVTS; index++) {
		seq_printf(s, "%-32s\t\t%u\n",
			telem_pcs_s0ix_blocked_data[index].name,
			pcs_s0ix_blocked[index]);
	}

	seq_puts(s, "\tLTR Blocking Status (~1ms saturating bucket)\n");
	seq_puts(s, "\tBlocker\t\tStatus\n");
	for (index = 0; index < TELEM_PSS_LTR_BLOCKING_EVTS; index++) {
		seq_printf(s, "%-32s\t\t%u\n",
			telem_pss_ltr_data[index].name,
			pss_s0ix_wakeup[index]);
	}

	seq_puts(s, "\tWakes Status (~1ms saturating bucket)\n");
	seq_puts(s, "\tWakes\t\tStatus\n");
	for (index = 0; index < TELEM_PSS_S0IX_WAKEUP_EVTS; index++) {
		seq_printf(s, "%-32s\t\t%u\n", telem_pss_wakeup[index].name,
			pss_ltr_blocked[index]);
	}

	return 0;
}

static int telem_pss_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, telem_pss_states_show, inode->i_private);
}

static const struct file_operations telem_pss_ops = {
	.open		= telem_pss_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int telem_ioss_evtlog_read(struct telem_ssram_region *ioss_ssram_region,
					u8 len)
{
	struct telemetry_config *telem_conf = telm_conf;
	int index, timeout = 0;
	u64 timestamp_prev, timestamp_next;

	if (len > (telem_conf->ssram_iossevts_used))
		len = telem_conf->ssram_iossevts_used;

	do {
		timestamp_prev = readq(telem_conf->ioss_regmap);
		if (!timestamp_prev) {
			pr_err("IOSS SSRAM under update. Try Later!!\n");
			return -EBUSY;
		}
		pr_debug("IOSS SSRAM timestamp_prev %llx\n",
			timestamp_prev);

		ioss_ssram_region->start_time =
			readq(telem_conf->ioss_regmap +
				TELEM_SSRAM_STARTTIME_OFFSET);
		pr_debug("IOSS SSRAM start_time %llx\n",
				ioss_ssram_region->start_time);

		for (index = 0; index < len; index++) {
			ioss_ssram_region->events[index] =
				readq(telem_conf->ioss_regmap +
					TELEM_SSRAM_EVTLOG_OFFSET +
						BYTES_PER_LONG*index);

			pr_debug("IOSS SSRAM index %x, value %llx\n",
				index, ioss_ssram_region->events[index]);
		}

		timestamp_next = readq(telem_conf->ioss_regmap);
		if (!timestamp_next) {
			pr_err("IOSS SSRAM under update. Try Later!!\n");
			return -EBUSY;
		}
		pr_debug("IOSS SSRAM timestamp_next %llx\n",
			timestamp_next);

		if (timeout++ > TELEM_SSRAM_READ_TIMEOUT) {
			pr_err("Timeout while reading IOSS Events!!\n");
			break;
		}
	} while (timestamp_prev != timestamp_next);

	ioss_ssram_region->timestamp = timestamp_next;

	return len;
}

static int telem_ioss_states_show(struct seq_file *s, void *unused)
{
	struct telemetry_config *telem_conf = telm_conf;
	struct telem_ssram_region ioss_ssram_region;
	int index, idx;
	int ret;
	u32 d3_sts[TELEM_PMC_DX_D0IX_EVTS], d0ix_sts[TELEM_PMC_DX_D0IX_EVTS];

	mutex_lock(&telem_lock);
	ret = telem_ioss_evtlog_read(&ioss_ssram_region,
		TELEM_MAX_OS_ALLOCATED_EVENTS);
	mutex_unlock(&telem_lock);

	if (ret < 0)
		return ret;

	seq_puts(s, "--------------------------------------\n");
	seq_puts(s, "\tI0SS TELEMETRY EVENTLOG\n");
	seq_puts(s, "--------------------------------------\n");
	for (index = 0; index < ret; index++) {
		seq_printf(s, "%-32s 0x%llx\n",
			telem_conf->ioss_telem_evts[index].name,
				ioss_ssram_region.events[index]);

		if (telem_conf->ioss_telem_evts[index].evt_id ==
						TELEM_PMC_D3_EVTID) {
			for (idx = 0; idx < TELEM_PMC_DX_D0IX_EVTS; idx++) {
				d3_sts[idx] =
				ioss_ssram_region.events[index] &
				(1<<(telem_ioss_d0ix_data[idx].bit_pos));

				if (d3_sts[idx])
					d3_sts[idx] = 1;
			}
		}

		if (telem_conf->ioss_telem_evts[index].evt_id ==
						TELEM_PMC_D0IX_EVTID) {
			for (idx = 0; idx < TELEM_PMC_DX_D0IX_EVTS; idx++) {
				d0ix_sts[idx] =
				ioss_ssram_region.events[index] &
				(1<<(telem_ioss_d0ix_data[idx].bit_pos));

				if (d0ix_sts[idx])
					d0ix_sts[idx] = 1;
			}
		}
	}

	seq_puts(s, "\n--------------------------------------\n");
	seq_puts(s, "\tIOSS D3/D0i3 Status\n");
	seq_puts(s, "Device\t\t D3\t D0i3\n");
	for (index = 0; index < TELEM_PMC_DX_D0IX_EVTS; index++) {
		seq_printf(s, "%-10s\t\t %u\t %u\n",
			telem_ioss_d0ix_data[index].name, d3_sts[index],
					d0ix_sts[index]);
	}

	return 0;
}

static int telem_ioss_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, telem_ioss_states_show, inode->i_private);
}

static const struct file_operations telem_ioss_ops = {
	.open		= telem_ioss_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int telemetry_check_pssevtid(u32 *evtmap, u8 len,
		enum telemetry_action action)
{
	switch (action) {
	case TELEM_RESET:
		if (len > TELEM_MAX_EVENTS_SRAM)
			return -EINVAL;

		break;

	case TELEM_UPDATE:
		if (len > TELEM_MAX_EVENTS_SRAM)
			return -EINVAL;

		if ((len > 0) && (NULL == evtmap))
			return -EINVAL;

		break;

	case TELEM_ADD:
		if ((len + (telm_conf->ssram_pssevts_used))
				> TELEM_MAX_EVENTS_SRAM) {
			return -EINVAL;
		}

		if ((len > 0) && (NULL == evtmap))
			return -EINVAL;

		break;

	default:
		pr_err("No TELEM_ACTION Specified\n");
		return -EINVAL;
	}

	return 0;
}

static int telemetry_check_iossevtid(u32 *evtmap, u8 len,
		enum telemetry_action action)
{
	switch (action) {
	case TELEM_RESET:
		if (len > TELEM_MAX_EVENTS_SRAM)
			return -EINVAL;

		break;

	case TELEM_UPDATE:
		if (len > TELEM_MAX_EVENTS_SRAM)
			return -EINVAL;


		if ((len > 0) && (NULL == evtmap))
			return -EINVAL;

		break;

	case TELEM_ADD:
		if ((len + (telm_conf->ssram_iossevts_used)) >
				TELEM_MAX_EVENTS_SRAM)
			return -EINVAL;

		if ((len > 0) && (NULL == evtmap))
			return -EINVAL;

		break;

	default:
		pr_err("No TELEM_ACTION Specified\n");
		return -EINVAL;
	}

	return 0;
}

static inline int telemetry_config_ioss_event(u32 evt_id, int index)
{
	u32 write_buf;
	int ret;

	write_buf = evt_id | TELEM_EVENT_ENABLE; /* Event Enable */
	write_buf <<= BITS_PER_BYTE;
	write_buf |= index; /* Set the Index register */
	pr_debug("ioss_event %x, %x\n", evt_id, index);
	ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
				IOSS_TELEM_EVENT_WRITE, (u8 *)&write_buf,
					IOSS_TELEM_EVT_WRITE_SIZE, NULL, 0);

	return ret;
}

static inline int telemetry_config_pss_event(u32 evt_id, int index)
{
	u32 write_buf;
	int ret;

	write_buf = evt_id | TELEM_EVENT_ENABLE; /* Event Enable */
	pr_debug("\npss_event %x, %x\n", evt_id, index);
	ret = intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT,
					index, 0, &write_buf, NULL);

	return ret;
}

static int telemetry_setup_evtconfig(u8 num_pss_evts, u8 num_ioss_evts,
	u32 *pss_evtmap, u32 *ioss_evtmap, u8 pss_period, u8 ioss_period,
		enum telemetry_action action)
{
	int ret, index, idx, evts;
	u32 telem_ctrl;

	mutex_lock(&telem_lock);

	if ((action == TELEM_UPDATE) && (telm_conf->telem_in_use)) {
		mutex_unlock(&telem_lock);
		return -EBUSY;
	}

	ret = telemetry_check_pssevtid(pss_evtmap, num_pss_evts, action);
	if (ret) {
		mutex_unlock(&telem_lock);
		return ret;
	}

	ret = telemetry_check_iossevtid(ioss_evtmap, num_ioss_evts, action);
	if (ret) {
		mutex_unlock(&telem_lock);
		return ret;
	}

	/* Get telemetry EVENT CTL */
	ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
			IOSS_TELEM_EVENT_CTL_READ, NULL, 0, &telem_ctrl,
				IOSS_TELEM_READ_FOURBYTES);
	if (ret) {
		pr_err("IOSS TELEM_CTRL Read Failed\n");
		mutex_unlock(&telem_lock);
		return ret;
	}

	pr_debug("\nIOSS TELEM_CTRL = %x\n", telem_ctrl);

	/* Disable Telemetry */
	TELEM_DISABLE(telem_ctrl);

	ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
		IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
			IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0);
	if (ret) {
		pr_err("IOSS TELEM_CTRL Event Disable Write Failed\n");
		mutex_unlock(&telem_lock);
		return ret;
	}


	/* Reset Everything */
	if (TELEM_RESET == action) {
		/* Clear All Events */
		TELEM_CLEAR_EVENTS(telem_ctrl);

		ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
				IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0);
		if (ret) {
			pr_err("IOSS TELEM_CTRL Event Disable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}
		pr_debug("\nTELEM_CTRL Written with %x\n", telem_ctrl);
		/* Configure Events */
		for (idx = 0, evts = 0; idx < num_ioss_evts; idx++) {
			telm_conf->ioss_telem_evts[idx]
				= telemetry_bxt_ioss_default_events[idx];

			ret = telemetry_config_ioss_event(
				telm_conf->ioss_telem_evts[idx].evt_id, idx);
			if (ret) {
				pr_err("IOSS TELEM_RESET Fail for data: %x\n",
				telemetry_bxt_ioss_default_events[idx].evt_id);
			} else
				evts++;
		}

		telm_conf->ssram_iossevts_used = evts;
		pr_debug("num_ioss_evts configured %x\n", evts);
	}

	/* Re-Configure Everything */
	if (TELEM_UPDATE == action) {
		/* Clear All Events */
		TELEM_CLEAR_EVENTS(telem_ctrl);

		ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
			IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0);
		if (ret) {
			pr_err("IOSS TELEM_CTRL Event Disable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		/* Configure Events */
		for (index = 0, evts = 0; index < num_ioss_evts; index++) {
			telm_conf->ioss_telem_evts[index].evt_id
				 = ioss_evtmap[index];

			ret = telemetry_config_ioss_event(
				telm_conf->ioss_telem_evts[index].evt_id,
					index);
			if (ret) {
				pr_err("IOSS TELEM_UPDATE Fail for Evt%x\n",
					ioss_evtmap[index]);
			} else
				evts++;
		}

		telm_conf->ssram_iossevts_used = evts;
		pr_debug("num_ioss_evts configured %x\n", evts);
	}

	/* Add some Events */
	if (TELEM_ADD == action) {
		/* Configure Events */
		for (index = telm_conf->ssram_iossevts_used, idx = 0, evts = 0;
			idx < num_ioss_evts; index++, idx++) {
			telm_conf->ioss_telem_evts[index].evt_id
				 = ioss_evtmap[idx];

			ret = telemetry_config_ioss_event(
				telm_conf->ioss_telem_evts[index].evt_id,
					index);
			if (ret) {
				pr_err("IOSS TELEM_ADD Fail for Event %x\n",
					ioss_evtmap[idx]);
			} else
				evts++;
		}

		telm_conf->ssram_iossevts_used += evts;
		pr_debug("num_ioss_evts configured %x\n",
				telm_conf->ssram_iossevts_used);
	}

	/* Enable Periodic Telemetry Events and enable SRAM trace */
	TELEM_ENABLE_SRAM_EVT_TRACE(telem_ctrl);
	TELEM_ENABLE_PERIODIC(telem_ctrl);
	telem_ctrl |= ioss_period;
	pr_debug("Final IOSS Write %x\n", telem_ctrl);
	ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
		IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
			IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0);
	if (ret) {
		pr_err("IOSS TELEM_CTRL Event Enable Write Failed\n");
		mutex_unlock(&telem_lock);
		return ret;
	}

	telm_conf->ioss_curr_period = ioss_period;

	/* PSS Config */
	/* Get telemetry EVENT CTL */
	ret = intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT_CTRL,
			0, 0, NULL, &telem_ctrl);
	if (ret) {
		pr_err("PSS TELEM_CTRL Read Failed\n");
		mutex_unlock(&telem_lock);
		return ret;
	}

	pr_debug("PSS TELEM_CTRL Read = %x\n", telem_ctrl);

	/* Disable Telemetry */
	TELEM_DISABLE(telem_ctrl);
	ret = intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
			0, 0, &telem_ctrl, NULL);
	if (ret) {
		pr_err("PSS TELEM_CTRL Event Disable Write Failed\n");
		mutex_unlock(&telem_lock);
		return ret;
	}

	/* Reset Everything */
	if (TELEM_RESET == action) {
		/* Clear All Events */
		TELEM_CLEAR_EVENTS(telem_ctrl);

		ret = intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
				0, 0, &telem_ctrl, NULL);
		if (ret) {
			pr_err("PSS TELEM_CTRL Event Disable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		/* Configure Events */
		for (idx = 0, evts = 0; idx < num_pss_evts; idx++) {
			telm_conf->pss_telem_evts[idx] =
				telemetry_bxt_pss_default_events[idx];

			ret = telemetry_config_pss_event(
				telm_conf->pss_telem_evts[idx].evt_id, idx);
			if (ret) {
				pr_err("PSS TELEM_RESET Fail for Event %x\n",
				telemetry_bxt_pss_default_events[idx].evt_id);
			} else
				evts++;
		}

		telm_conf->ssram_pssevts_used = evts;
		pr_debug("\nTELEM_CTRL PSS events %x\n", evts);
	}

	/* Re-Configure Everything */
	if (TELEM_UPDATE == action) {
		/* Clear All Events */
		TELEM_CLEAR_EVENTS(telem_ctrl);

		ret = intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
				0, 0, &telem_ctrl, NULL);
		if (ret) {
			pr_err("PSS TELEM_CTRL Event Disable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		/* Configure Events */
		for (index = 0, evts = 0; index < num_pss_evts; index++) {
			telm_conf->pss_telem_evts[index].evt_id =
				pss_evtmap[index];

			ret = telemetry_config_pss_event(
				telm_conf->pss_telem_evts[index].evt_id,
					index);
			if (ret) {
				pr_err("PSS TELEM_UPDATE Fail for Event %x\n",
					pss_evtmap[index]);
			} else
				evts++;
		}

		telm_conf->ssram_pssevts_used = evts;
		pr_debug("\nTELEM_CTRL PSS events %x\n", evts);
	}

	/* Add some Events */
	if (TELEM_ADD == action) {
		/* Configure Events */
		for (index = telm_conf->ssram_pssevts_used, idx = 0, evts = 0;
			idx < num_pss_evts; index++, idx++) {
			telm_conf->pss_telem_evts[index].evt_id =
				pss_evtmap[idx];

			ret = telemetry_config_pss_event(
				telm_conf->pss_telem_evts[index].evt_id,
					index);
			if (ret) {
				pr_err("PSS TELEM_ADD Fail for Event %x\n",
					pss_evtmap[idx]);
			} else
				evts++;
		}

		telm_conf->ssram_pssevts_used += evts;
		pr_debug("\nTELEM_CTRL PSS events %x\n",
			telm_conf->ssram_pssevts_used);
	}

	/* Enable Periodic Telemetry Events and enable SRAM trace */
	TELEM_ENABLE_SRAM_EVT_TRACE(telem_ctrl);
	TELEM_ENABLE_PERIODIC(telem_ctrl);
	telem_ctrl |= pss_period;
	pr_debug("\ntelemetry_punit_ipc %x\n", telem_ctrl);
	ret = intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
			0, 0, &telem_ctrl, NULL);
	if (ret) {
		pr_err("PSS TELEM_CTRL Event Enable Write Failed\n");
		mutex_unlock(&telem_lock);
		return ret;
	}

	telm_conf->pss_curr_period = pss_period;

	if (action == TELEM_RESET)
		telm_conf->telem_in_use = 0;
	else if ((action == TELEM_UPDATE) || (action == TELEM_ADD))
		telm_conf->telem_in_use = 1;

	mutex_unlock(&telem_lock);
	return 0;
}


static int telemetry_setup(void)
{
	int ret;
	u32 read_buf, events, event_regs;

	ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY, IOSS_TELEM_INFO_READ,
			NULL, 0, &read_buf, IOSS_TELEM_READ_FOURBYTES);
	if (ret) {
		pr_err("IOSS TELEM_INFO Read Failed\n");
		return ret;
	}

	pr_debug("IOSS TELEM_INFO = %x\n", read_buf);

	/* Get telemetry Info */
	events = (read_buf & TELEM_INFO_SRAMEVTS_MASK) >>
			TELEM_INFO_SRAMEVTS_SHIFT;
	event_regs = read_buf & TELEM_INFO_NENABLES_MASK;
	if ((events < TELEM_MAX_EVENTS_SRAM) ||
			(event_regs < TELEM_MAX_EVENTS_SRAM)) {
		pr_err("IOSS TELEM_INFO::Insufficient Space for SRAM Trace\n");
		pr_err("SRAM Events %d; Event Regs %d\n", events, event_regs);
		return -ENOMEM;
	}

	telm_conf->ioss_min_period = TELEM_MIN_PERIOD(read_buf);
	telm_conf->ioss_max_period = TELEM_MAX_PERIOD(read_buf);

	/* PUNIT Mailbox Setup */
	ret = intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_READ_TELE_INFO, 0, 0,
			NULL, &read_buf);
	if (ret) {
		pr_err("PSS TELEM_INFO Read Failed\n");
		return ret;
	}

	pr_debug("PSS TELEM_INFO = %x\n", read_buf);

	/* Get telemetry Info */
	events = (read_buf & TELEM_INFO_SRAMEVTS_MASK) >>
			TELEM_INFO_SRAMEVTS_SHIFT;
	event_regs = read_buf & TELEM_INFO_SRAMEVTS_MASK;
	if ((events < TELEM_MAX_EVENTS_SRAM) ||
		(event_regs < TELEM_MAX_EVENTS_SRAM)) {
		pr_err("PSS TELEM_INFO::Insufficient Space for SRAM Trace\n");
		pr_err("SRAM Events %d; Event Regs %d\n", events, event_regs);
		return -ENOMEM;
	}

	telm_conf->pss_min_period = TELEM_MIN_PERIOD(read_buf);
	telm_conf->pss_max_period = TELEM_MAX_PERIOD(read_buf);

	ret = telemetry_setup_evtconfig(TELEM_MAX_OS_ALLOCATED_EVENTS,
		TELEM_MAX_OS_ALLOCATED_EVENTS, NULL, NULL,
		TELEM_SAMPLING_DEFAULT_PERIOD, TELEM_SAMPLING_DEFAULT_PERIOD,
		TELEM_RESET);
	if (ret) {
		pr_err("TELEMTRY Setup Failed\n");
		return ret;
	}

	telm_conf->pss_regmap = ioremap_nocache(
					(telm_conf->ssram_base_addr) +
					PSS_TELEM_SSRAM_OFFSET,
					TELEM_SSRAM_SIZE);
	if (!telm_conf->pss_regmap) {
		pr_err("TELEM::PSS-SSRAM ioremap failed\n");
		return -ENOMEM;
	}

	telm_conf->ioss_regmap = ioremap_nocache(
					(telm_conf->ssram_base_addr) +
					IOSS_TELEM_SSRAM_OFFSET,
					TELEM_SSRAM_SIZE);
	if (!telm_conf->ioss_regmap) {
		pr_err("TELEM::IOSS-SSRAM ioremap failed\n");
		iounmap(telm_conf->pss_regmap);
		return -ENOMEM;
	}

	return 0;
}


/*
* telemetry_update_events() - This API updates the IOSS & PSS
* Telemetry configuration to the provided config. Old config
* is overwritten. Call telemetry_reset_events when logging is over
* All sample period values should be in the form of:
* bits[6:3] -> value; bits [0:2]-> Exponent; Period = (Value *16^Exponent)
*
* @num_pss_evts: Number of PSS Events (<29) provided by pss_evtmap
*		Can be 0
* @num_ioss_evts: Number of IOSS Events (<29) provided by ioss_evtmap
*		Can be 0
* @pss_evtmap: Array of PSS Event-IDs to Enable
* @ioss_evtmap: Array of PSS Event-IDs to Enable
* @pss_period : PSS sampling period to set
* @ioss_period: IOSS sampling period to set
*
* Return: 0 success, -ive for failure
*/
int telemetry_update_events(u8 num_pss_evts, u8 num_ioss_evts,
	u32 *pss_evtmap, u32 *ioss_evtmap, u8 pss_period, u8 ioss_period)
{
	int err = 0;

	err = telemetry_setup_evtconfig(num_pss_evts, num_ioss_evts,
		pss_evtmap, ioss_evtmap, pss_period, ioss_period, TELEM_UPDATE);
	if (err)
		pr_err("TELEMTRY Config Failed\n");

	return err;
}
EXPORT_SYMBOL_GPL(telemetry_update_events);


/*
* telemetry_set_sampling_period() - This API sets the IOSS & PSS sampling period
* All values should be in the form of:
* bits[6:3] -> value; bits [0:2]-> Exponent; Period = (Value *16^Exponent)
*
* @pss_period: placeholder for PSS Period to be set.
*		Set to 0 if not required to be updated
* @ioss_period: placeholder for IOSS Period to be set
*		Set to 0 if not required to be updated
* Return: 0 success, -ive for failure
*/
int telemetry_set_sampling_period(u8 pss_period, u8 ioss_period)
{
	int ret;
	u32 telem_ctrl;

	mutex_lock(&telem_lock);
	if (ioss_period) {
		/* Get telemetry EVENT CTL */
		ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
			IOSS_TELEM_EVENT_CTL_READ, NULL, 0, &telem_ctrl,
				IOSS_TELEM_READ_FOURBYTES);
		if (ret) {
			pr_err("IOSS TELEM_CTRL Read Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		pr_debug("%s TELEM_CTRL=%x\n", __func__, telem_ctrl);

		/* Disable Telemetry */
		TELEM_DISABLE(telem_ctrl);

		ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
				IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0);
		if (ret) {
			pr_err("IOSS TELEM_CTRL Event Disable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		/* Enable Periodic Telemetry Events and enable SRAM trace */
		TELEM_ENABLE_SRAM_EVT_TRACE(telem_ctrl);
		TELEM_ENABLE_PERIODIC(telem_ctrl);
		telem_ctrl |= ioss_period;
		pr_debug("%s IOSS TELEM_CTRL Write %x\n", __func__,
				telem_ctrl);
		ret = intel_pmc_ipc_command(PMC_IPC_PMC_TELEMTRY,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
				IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0);
		if (ret) {
			pr_err("IOSS TELEM_CTRL Event Enable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		telm_conf->ioss_curr_period = ioss_period;
	}

	if (pss_period) {
		/* Get telemetry EVENT CTL */
		ret = intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT_CTRL,
				0, 0, NULL, &telem_ctrl);
		if (ret) {
			pr_err("PSS TELEM_CTRL Read Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}
		pr_debug("%s TELEM_CTRL=%x\n", __func__, telem_ctrl);

		/* Disable Telemetry */
		TELEM_DISABLE(telem_ctrl);
		ret = intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
				0, 0, &telem_ctrl, NULL);
		if (ret) {
			pr_err("PSS TELEM_CTRL Event Disable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		/* Enable Periodic Telemetry Events and enable SRAM trace */
		TELEM_ENABLE_SRAM_EVT_TRACE(telem_ctrl);
		TELEM_ENABLE_PERIODIC(telem_ctrl);
		telem_ctrl |= pss_period;
		pr_debug("%s PSS TELEM_CTRL Write %x\n", __func__,
				telem_ctrl);
		ret = intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
				0, 0, &telem_ctrl, NULL);
		if (ret) {
			pr_err("PSS TELEM_CTRL Event Enable Write Failed\n");
			mutex_unlock(&telem_lock);
			return ret;
		}

		telm_conf->pss_curr_period = pss_period;
	}

	mutex_unlock(&telem_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(telemetry_set_sampling_period);

/*
* telemetry_get_sampling_period() - This API gets the IOSS & PSS min and max
* sampling period as supported by the Telemetry Hardware.
* All values should be in the form of:
* bits[6:3] -> value; bits [0:2]-> Exponent; Period = (Value *16^Exponent)
*
* @pss_min_period: placeholder for PSS Min Period supported
* @pss_max_period: placeholder for PSS Max Period supported
* @ioss_min_period: placeholder for IOSS Min Period supported
* @ioss_max_period: placeholder for IOSS Max Period supported
* Return: 0 success, -ive for failure
*/

int telemetry_get_sampling_period(u8 *pss_min_period, u8 *pss_max_period,
				u8 *ioss_min_period, u8 *ioss_max_period)
{
	struct telemetry_config *telem_conf = telm_conf;

	*pss_min_period = telem_conf->pss_min_period;
	*pss_max_period = telem_conf->pss_max_period;
	*ioss_min_period = telem_conf->ioss_min_period;
	*ioss_max_period = telem_conf->ioss_max_period;

	return 0;
}
EXPORT_SYMBOL_GPL(telemetry_get_sampling_period);


/*
* telemetry_reset_events() - This API restores the IOSS & PSS
* Telemetry configuration to the default config set at boot.
*
* Return: 0 success, -ive for failure
*/

int telemetry_reset_events(void)
{
	int err = 0;

	err = telemetry_setup_evtconfig(TELEM_MAX_OS_ALLOCATED_EVENTS,
		TELEM_MAX_OS_ALLOCATED_EVENTS, NULL, NULL,
		TELEM_SAMPLING_DEFAULT_PERIOD, TELEM_SAMPLING_DEFAULT_PERIOD,
		TELEM_RESET);
	if (err)
		pr_err("TELEMTRY Reset Failed\n");

	return err;
}
EXPORT_SYMBOL_GPL(telemetry_reset_events);

/*
* telemetry_get_eventconfig() - This API returns the pss
* and ioss events enabled.
*
* @num_pss_evts: No. of PSS Events enabled (returned by API)
* @num_ioss_evts: No. of IOSS Events enabled (returned by API)
* @pss_evtmap: Array of u32 used to contain events enabled currently
* @ioss_evtmap: Array of u32 used to contain events enabled currently
* @pss_len: No of u32 elements allocated for pss_evtmap array
* @ioss_len: No of u32 elements allocated for ioss_evtmap array
*
* Return: 0 success, -ive for failure
*/
int telemetry_get_eventconfig(u8 *num_pss_evts, u8 *num_ioss_evts,
	u32 *pss_evtmap, u32 *ioss_evtmap, int pss_len, int ioss_len)
{
	struct telemetry_config *telem_conf = telm_conf;
	u32 index;

	mutex_lock(&telem_lock);
	*num_pss_evts = telem_conf->ssram_pssevts_used;
	*num_ioss_evts = telem_conf->ssram_iossevts_used;

	if ((pss_len < telem_conf->ssram_pssevts_used) ||
		(ioss_len < telem_conf->ssram_iossevts_used)) {
		mutex_unlock(&telem_lock);
		return -EINVAL;
	}

	for (index = 0; index < telem_conf->ssram_pssevts_used; index++) {
		pss_evtmap[index] =
			telem_conf->pss_telem_evts[index].evt_id;
	}

	for (index = 0; index < telem_conf->ssram_iossevts_used; index++) {
		ioss_evtmap[index] =
			telem_conf->ioss_telem_evts[index].evt_id;
	}

	mutex_unlock(&telem_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(telemetry_get_eventconfig);

/*
* telemetry_add_events() - This API adds the IOSS & PSS
* telemetry configuration to the existing config. Old config
* is appended. In case of total events > 28, it returns error
* Call telemetry_reset_events to reset config after eventlog done
*
* @num_pss_evts: Number of PSS Events (<29) in pss_evtmap
*		Can be 0
* @num_ioss_evts: Number of IOSS Events (<29) in ioss_evtmap
**		Can be 0
* @pss_evtmap: Array of PSS Event-IDs to Enable
* @ioss_evtmap: Array of PSS Event-IDs to Enable
*
* Return: 0 success, -ive for failure
*/
int telemetry_add_events(u8 num_pss_evts, u8 num_ioss_evts,
		u32 *pss_evtmap, u32 *ioss_evtmap)
{
	int err = 0;
	struct telemetry_config *telem_conf = telm_conf;

	err = telemetry_setup_evtconfig(num_pss_evts, num_ioss_evts,
		pss_evtmap, ioss_evtmap, telem_conf->pss_curr_period,
			telem_conf->ioss_curr_period, TELEM_ADD);
	if (err)
		pr_err("TELEMTRY ADD Failed\n");

	return err;
}
EXPORT_SYMBOL_GPL(telemetry_add_events);

/*
* telemetry_read_eventlog() - This API fetches the Telemetry log
* from PSS or IOSS as specified.
*
* @telem_unit: Specify whether IOSS or PSS Read
* @evtlog: Array of telemetry_evtlog structs to fill data
* @len: Length of array of evtlog
*
* Return: number of eventlogs read for success, -ive for failure
*/

int telemetry_read_eventlog(enum telemetry_unit telem_unit,
		struct telemetry_evtlog *evtlog, int len)
{
	int ret;

	mutex_lock(&telem_lock);
	ret = telemetry_raw_read_eventlog(telem_unit, evtlog, len);
	mutex_unlock(&telem_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(telemetry_read_eventlog);

/*
* telemetry_raw_read_eventlog() - This API fetches the Telemetry log
* from PSS or IOSS as specified. The caller must take care of
* locking in this case.
*
* @telem_unit: Specify whether IOSS or PSS Read
* @evtlog: Array of telemetry_evtlog structs to fill data
* @len: Length of array of evtlog
*
* Return: number of eventlogs read for success, -ive for failure
*/

int telemetry_raw_read_eventlog(enum telemetry_unit telem_unit,
		struct telemetry_evtlog *evtlog, int len)
{
	int index, ret = 0;
	struct telemetry_config *telem_conf = telm_conf;
	struct telem_ssram_region ssram_region;

	switch (telem_unit)	{
	case TELEM_PSS:
		ret = telem_pss_evtlog_read(&ssram_region, len);
		if (ret < 0)
			return ret;

		for (index = 0; index < ret; index++) {
			evtlog->telem_evtlog =
				ssram_region.events[index];
			evtlog->telem_evtid =
				telem_conf->pss_telem_evts[index].evt_id;
			evtlog++;
		}

		break;

	case TELEM_IOSS:
		ret = telem_ioss_evtlog_read(&ssram_region, len);
		if (ret < 0)
			return ret;

		for (index = 0; index < ret; index++) {
			evtlog->telem_evtlog =
				ssram_region.events[index];
			evtlog->telem_evtid =
				telem_conf->ioss_telem_evts[index].evt_id;

			evtlog++;
		}

			break;

	default:
		pr_err("No Telemetry Unit Specified");
		ret = -EINVAL;
		break;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(telemetry_raw_read_eventlog);

static int telemetry_register(void)
{
	int err = -ENODEV;
	const struct x86_cpu_id *id;
	struct dentry *f;

	id = x86_match_cpu(telemetry_cpu_ids);
	if (!id)
		return -ENODEV;

	telm_conf = (struct telemetry_config *)id->driver_data;
	if (!telm_conf->ssram_base_addr)
		return -ENODEV;

	telm_conf->telemetry_dbg_dir =
		debugfs_create_dir("telemetry_atom", NULL);
	if (!(telm_conf->telemetry_dbg_dir))
		return -ENOMEM;

	f = debugfs_create_file("pss_info", S_IFREG | S_IRUGO,
		(telm_conf->telemetry_dbg_dir), NULL, &telem_pss_ops);
	if (!f) {
		pr_err("PSS_Info debugfs register failed\n");
		debugfs_remove_recursive(telm_conf->telemetry_dbg_dir);
		telm_conf->telemetry_dbg_dir = NULL;
		return -ENOMEM;
	}

	f = debugfs_create_file("ioss_info", S_IFREG | S_IRUGO,
		(telm_conf->telemetry_dbg_dir),	NULL, &telem_ioss_ops);
	if (!f) {
		pr_err("IOSS_Info debugfs register failed\n");
		debugfs_remove_recursive(telm_conf->telemetry_dbg_dir);
		telm_conf->telemetry_dbg_dir = NULL;
		return -ENOMEM;
	}

	err = telemetry_setup();
	if (err) {
		pr_err("TELEMTRY Setup Failed\n");
		debugfs_remove_recursive(telm_conf->telemetry_dbg_dir);
		telm_conf->telemetry_dbg_dir = NULL;
		return err;
	}

	return 0;
}

static void telemetry_unregister(void)
{
	debugfs_remove_recursive(telm_conf->telemetry_dbg_dir);
	telm_conf->telemetry_dbg_dir = NULL;

	iounmap(telm_conf->pss_regmap);
	iounmap(telm_conf->ioss_regmap);
}

static int __init telemetry_module_init(void)
{
	int ret;

	pr_info("[TELEM]::Init\n");
	ret = telemetry_register();

	return ret;
}

static void __exit telemetry_module_exit(void)
{
	telemetry_unregister();
}

late_initcall(telemetry_module_init);
module_exit(telemetry_module_exit);

MODULE_AUTHOR("Souvik Kumar Chakravarty <souvik.k.chakravarty@intel.com>");
MODULE_DESCRIPTION("Intel ATOM SoC Telemetry Interface");
MODULE_LICENSE("GPL v2");
