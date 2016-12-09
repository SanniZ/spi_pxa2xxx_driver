/*
 * charging_algo.h - Charging algorithm core  driver
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CHARGING_ALGO_H__
#define __CHARGING_ALGO_H__

#include <linux/power_supply.h>

struct charging_algo_request {
	char	*supplied_to;
	int	charging_status;
};

struct charging_algo_params {
	/* mandatory params */
	int	charge_current;
	int	charge_voltage;
	/* optional params */
	int	vrechg_delta;
};

enum charging_algo_opcode {
	CHARGING_ALGO_RESET,
	CHARGING_ALGO_START_LEARNING,
};

struct charging_algo {
	char *name;
	char	*supplied_to;
	struct list_head entry;

	int (*get_charging_params)(struct charging_algo_request *req,
				struct charging_algo_params *params);
	int (*send_algo_cmd)(struct charging_algo_request *req,
				enum charging_algo_opcode opcode);
};

#ifdef CONFIG_CHARGING_ALGO
extern int charging_algo_register(struct charging_algo *algo);
extern void charging_algo_unregister(struct charging_algo *algo);
extern int charging_algo_get_params(struct charging_algo_request *req,
				struct charging_algo_params *params);
extern int charging_algo_send_cmd(struct charging_algo_request *req,
				enum charging_algo_opcode opcode);
#else
static inline int charging_algo_register(struct charging_algo *algo)
{
	return 0;
}
static inline void charging_algo_unregister(struct charging_algo *algo)
{
}
static inline int charging_algo_get_params(struct charging_algo_request *req,
				struct charging_algo_params *params)
{
	return -ENODEV;
}
static inline int charging_algo_send_cmd(struct charging_algo_request *req,
				enum charging_algo_opcode opcode)
{
	return -ENODEV;
}
#endif

#endif /* __CHARGING_ALGO_H__ */
