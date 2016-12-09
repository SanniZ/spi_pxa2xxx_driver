/*
 * intel_sw_fuel_gauge.h - Intel MID PMIC Fuel Gauge Driver header
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Srinidhi Rao <srinidhi.rao@intel.com>
 *	       Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *         David Bennetts <david.bennetts@intel.com>
 *         Felix Becker <felix.becker@intel.com>
 */

#ifndef __INTEL_SW_FUEL_GAUGE__
#define __INTEL_SW_FUEL_GAUGE__

#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>

#define SWFG_BATTID_STR_LEN 8

enum fg_bat_status {
	SWFG_BAT_STATUS_UNKNOWN = 0,
	SWFG_BAT_STATUS_CHARGING,
	SWFG_BAT_STATUS_DISCHARGING,
	SWFG_BAT_STATUS_NOT_CHARGING,
	SWFG_BAT_STATUS_FULL
};

union fg_input_event_param {
	int cc_delta_mc;
	int target_voltage_mv;
	int battery_capacity;
	int expired_timer_id;
	enum fg_bat_status battery_status;
};

enum fg_input_event {
	SWFG_HAL_EVENT_SOC_UPDATE = 0,
	SWFG_HAL_EVENT_EOC,
	SWFG_BAT_EVENT_PRESENCE,
	SWFG_BAT_EVENT_STATUS_UPDATE,
	SWFG_HAL_EVENT_SHUTDOWN,
	SWFG_HAL_EVENT_SYSTEM,

	SWFG_TIMER_EXPIRED,
};

struct fg_data {
	int key;
	int param;
};

struct fg_msg_type {
	union fg_input_event_param ev_param;
	enum fg_input_event event;
	struct fg_data data;
};

struct set_cc_val {
	bool is_set;
	int val;
};


struct fg_params {

	int vbatt;
	int vavg;
	int vocv;
	int vbatt_boot;
	int ibatt_boot;
	int ibatt;
	int iavg;
	int bat_temp;
	int delta_q;

	int soc;
	int nac;
	int fcc;
	int cycle_count;
	bool calib_cc;

	/* Coulomb Counter I/P Params */
	int up_cc;
	int down_cc;
	int acc_err;
	int delta_thr;
	int long_avg;
	int long_avg_at_ocv;
	int ocv_accuracy;

	/* Coulomb Counter O/P Params */
	struct set_cc_val reset_acc_err;
	struct set_cc_val set_delta_thr;
	struct set_cc_val clr_latched_ibat_avg;

	struct fg_msg_type msg;

	int status;
	bool boot_flag;
	bool is_valid_battery;
	char battid[SWFG_BATTID_STR_LEN + 1];
};

struct intel_fg_batt_spec {
	int volt_min_design;
	int volt_max_design;
	int temp_min;
	int temp_max;
	int charge_full_design;
};

struct intel_fg_input {
	int (*get_batt_params)(int *vbat, int *ibat, int *bat_temp);
	int (*get_v_ocv)(int *v_ocv);
	int (*get_v_ocv_bootup)(int *v_ocv_bootup);
	int (*get_i_bat_bootup)(int *i_bat_bootup);
	int (*get_v_avg)(int *v_avg);
	int (*get_i_avg)(int *i_avg);
	int (*get_delta_q)(int *delta_q);
	int (*calibrate_cc)(void);

	/* Coulomb Counter APIs */
	bool (*wait_for_cc)(void);
	int (*get_up_cc)(int *up_cc);
	int (*get_down_cc)(int *down_cc);
	int (*get_acc_err)(int *acc_err);
	int (*get_delta_thr)(int *delta_thr);
	int (*get_long_avg)(int *long_avg);
	int (*get_long_avg_ocv)(int *long_avg_ocv);
	int (*get_ocv_accuracy)(int *ocv_acuracy);

	int (*reset_acc_err)(int acc_err);
	int (*set_delta_thr)(int delta_thr);
	int (*clr_latched_ibat_avg)(int ibat_avg_at_ocv);
};

#endif
