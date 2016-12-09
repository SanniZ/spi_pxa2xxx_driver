/*
 * Intel Atom SOC Telemetry Driver Header File
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

#ifndef ATOM_SOC_TELEMETRY_H
#define ATOM_SOC_TELEMETRY_H

enum telemetry_unit {
	TELEM_PSS = 0,
	TELEM_IOSS,
	TELEM_UNIT_NONE
};

struct telemetry_evtlog {
	u32 telem_evtid;
	u64 telem_evtlog;
};

int telemetry_update_events(u8 num_pss_evts, u8 num_ioss_evts, u32 *pss_evtmap,
		 u32 *ioss_evtmap, u8 pss_period, u8 ioss_period);

int telemetry_add_events(u8 num_pss_evts, u8 num_ioss_evts,
		u32 *pss_evtmap, u32 *ioss_evtmap);

int telemetry_reset_events(void);

int telemetry_get_eventconfig(u8 *num_pss_evts, u8 *num_ioss_evts,
	u32 *pss_evtmap, u32 *ioss_evtmap, int pss_len, int ioss_len);

int telemetry_raw_read_eventlog(enum telemetry_unit telem_unit,
			struct telemetry_evtlog *evtlog, int len);

int telemetry_read_eventlog(enum telemetry_unit telem_unit,
			struct telemetry_evtlog *evtlog, int len);

int telemetry_get_sampling_period(u8 *pss_min_period, u8 *pss_max_period,
				u8 *ioss_min_period, u8 *ioss_max_period);

int telemetry_set_sampling_period(u8 pss_period, u8 ioss_period);

#endif /* ATOM_SOC_TELEMETRY_H */
