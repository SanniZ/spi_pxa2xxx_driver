/*
 * charging_algo.c - Charging algorithm core driver
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/power/charging_algo.h>

static LIST_HEAD(charging_algo_list);
static DEFINE_MUTEX(charging_algo_lock);

static struct charging_algo *get_charging_algo(const char *supplied_to)
{
	struct charging_algo *ca;

	mutex_lock(&charging_algo_lock);
	list_for_each_entry(ca, &charging_algo_list, entry) {
		if (!strcmp(ca->supplied_to, supplied_to))
			goto out;
	}
	ca = NULL;
out:
	mutex_unlock(&charging_algo_lock);
	return ca;

}

int charging_algo_get_params(struct charging_algo_request *req,
				struct charging_algo_params *params)
{
	struct charging_algo *ca;

	ca = get_charging_algo(req->supplied_to);
	if (!ca || !ca->get_charging_params) {
		pr_err("%s: cannot find charging algo\n", __func__);
		return -ENODEV;
	}

	return ca->get_charging_params(req, params);
}
EXPORT_SYMBOL_GPL(charging_algo_get_params);

int charging_algo_send_cmd(struct charging_algo_request *req,
				enum charging_algo_opcode opcode)
{
	struct charging_algo *ca;

	ca = get_charging_algo(req->supplied_to);
	if (!ca || !ca->send_algo_cmd) {
		pr_err("%s: cannot find charging algo\n", __func__);
		return -ENODEV;
	}

	return ca->send_algo_cmd(req, opcode);
}
EXPORT_SYMBOL_GPL(charging_algo_send_cmd);

int charging_algo_register(struct charging_algo *algo)
{
	if (!algo || !algo->get_charging_params) {
		pr_err("%s: invalid algo\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&charging_algo_lock);
	list_add(&algo->entry, &charging_algo_list);
	mutex_unlock(&charging_algo_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(charging_algo_register);

void charging_algo_unregister(struct charging_algo *algo)
{
	mutex_lock(&charging_algo_lock);
	if (algo)
		list_del(&algo->entry);
	mutex_unlock(&charging_algo_lock);
}
EXPORT_SYMBOL_GPL(charging_algo_unregister);

MODULE_DESCRIPTION("Charging Algo Core driver");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_LICENSE("GPL v2");
