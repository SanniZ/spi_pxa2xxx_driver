/*
 * charging_algo_bxt.c - Intel Broxton PSE Algorithm
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
#include <linux/types.h>
#include <linux/acpi.h>
#include <linux/power_supply.h>
#include <linux/power/charging_algo.h>

#define ACPI_OEM0_SIGN		"OEM0"
#define BATTID_STR_LEN		8
#define BATT_TEMP_NR_RNG	6
#define BATT_TEMP_CRIT		75	/* 75DegC */
#define BATT_TEMP_HIGH		45	/* 45DegC */
#define BATT_TEMP_LOW		0	/* 0DegC */
#define BATT_DEF_FULL_CC	2600	/* 2600mA */

struct pse_charge_table {
	/* upper temperature limit for each zone */
	short int temp_up_lim;
	/* charge current and voltage */
	short int full_chrg_vol;
	short int full_chrg_cur;
	/* maintenance thresholds */
	short int maint_chrg_vol_ll;
	short int maint_chrg_vol_ul;
	short int maint_chrg_cur;
} __packed;

struct intel_oem0_table {
	char batt_id[BATTID_STR_LEN];
	u8 turbo;
	u8 batt_type;
	u16 capacity;
	u16 volt_max;
	u16 chrg_term_ma;
	u16 low_batt_thr;
	u8  safe_dischrg_ul;
	u8  safe_dischrg_ll;
	u16 temp_mon_ranges;
	struct pse_charge_table temp_mon_range[BATT_TEMP_NR_RNG];
	/* temperature lower limit */
	u16 temp_low_lim;
} __packed;

static struct intel_oem0_table *oem0;

static void bxt_chrg_algo_dump_oem0(const struct intel_oem0_table *oem0_table)
{
	u16 i = 0;

	pr_info("OEM0:batt_id = %s\n", oem0_table->batt_id);
	pr_info("OEM0:batt_type = %d\n", oem0_table->batt_type);
	pr_info("OEM0:capacity = %d\n", oem0_table->capacity);
	pr_info("OEM0:volt_max = %d\n", oem0_table->volt_max);
	pr_info("OEM0:chrg_term_ma = %d\n", oem0_table->chrg_term_ma);
	pr_info("OEM0:low_batt_thr = %d\n", oem0_table->low_batt_thr);
	pr_info("OEM0:safe_dischrg_ul = %d\n", oem0_table->safe_dischrg_ul);
	pr_info("OEM0:safe_dischrg_ll = %d\n", oem0_table->safe_dischrg_ll);
	pr_info("OEM0:temp_mon_ranges = %d\n", oem0_table->temp_mon_ranges);
	for (i = 0; i < oem0_table->temp_mon_ranges; i++) {
		pr_info("OEM0:temp_mon_range[%d].up_lim=%d\n",
			i, oem0_table->temp_mon_range[i].temp_up_lim);
		pr_info("OEM0:temp_mon_range[%d].full_chrg_vol=%d\n",
			i, oem0_table->temp_mon_range[i].full_chrg_vol);
		pr_info("OEM0:temp_mon_range[%d].full_chrg_cur=%d\n",
			i, oem0_table->temp_mon_range[i].full_chrg_cur);
		pr_info("OEM0:temp_mon_range[%d].maint_chrg_vol_ll=%d\n", i,
			oem0_table->temp_mon_range[i].maint_chrg_vol_ll);
		pr_info("OEM0:temp_mon_range[%d].main_chrg_vol_ul = %d\n", i,
			oem0_table->temp_mon_range[i].maint_chrg_vol_ul);
		pr_info("OEM0:temp_mon_range[%d].main_chrg_cur = %d\n",
			i, oem0_table->temp_mon_range[i].maint_chrg_cur);
	}
	pr_info("OEM0:temp_low_lim = %d\n", oem0_table->temp_low_lim);
}

static int bxt_chrg_get_acpi_table(char *name, void *data, int data_size)
{
	struct acpi_table_header *acpi_tbl = NULL;
	acpi_size tbl_size;
	acpi_status status;
	int ret = -ENODEV;
	int hdr_size = sizeof(struct acpi_table_header);

	status = acpi_get_table_with_size(name, 0,
					&acpi_tbl, &tbl_size);
	if (ACPI_SUCCESS(status)) {
		pr_info("EM:%s  table found, size=%d\n", name, (int)tbl_size);
		if (tbl_size < (data_size + hdr_size)) {
			pr_err("EM:%s table incomplete!!\n", name);
		} else {
			memcpy(data, ((char *)acpi_tbl) + hdr_size, data_size);
			ret = data_size;
		}
	} else {
		pr_err("EM:%s table not found!!\n", name);
	}

	return ret;
}

static int algo_intel_get_oem0_table(void)
{
	static struct intel_oem0_table oem0_table;
	int i, ret;

	ret = bxt_chrg_get_acpi_table(ACPI_OEM0_SIGN,
			&oem0_table, sizeof(struct intel_oem0_table));
	if (ret < 0)
		return ret;

	/* Fix Upper and lower temperature limits */
	oem0_table.temp_mon_range[0].temp_up_lim = BATT_TEMP_CRIT;
	oem0_table.temp_mon_range[5].temp_up_lim = BATT_TEMP_LOW;
	for (i = 0; i < oem0_table.temp_mon_ranges; i++) {
		if (oem0_table.temp_mon_range[i].temp_up_lim
						== BATT_TEMP_HIGH) {
			oem0_table.temp_mon_range[i].full_chrg_cur =
							BATT_DEF_FULL_CC;
			oem0_table.temp_mon_range[i].maint_chrg_cur =
							BATT_DEF_FULL_CC;
		}
	}

	bxt_chrg_algo_dump_oem0(&oem0_table);
	oem0 = &oem0_table;
	return 0;
}

static struct power_supply *get_fuel_gauge_psy(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *fg_psy;

	if (fg_psy)
		return fg_psy;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		fg_psy = (struct power_supply *)dev_get_drvdata(dev);
		if (fg_psy->desc->type == POWER_SUPPLY_TYPE_BATTERY) {
			pr_info("Battery fuel gauge psy found\n");
			break;
		}
	}
	class_dev_iter_exit(&iter);

	return fg_psy;
}

static int get_temperature_zone(void)
{
	int i = 0, temp;
	struct power_supply *psy;
	union power_supply_propval val;
	int temp_range_cnt = min_t(u16, oem0->temp_mon_ranges,
					BATT_TEMP_NR_RNG);

	psy = get_fuel_gauge_psy();
	if (!psy)
		return -ENODEV;

	temp = psy->desc->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (temp < 0) {
		pr_err("%s: get battery temperature error(%d)\n",
						__func__, temp);
		return -ENODEV;
	}

	temp = val.intval / 10;

	if ((temp < oem0->temp_low_lim) ||
		(temp > oem0->temp_mon_range[0].temp_up_lim))
		return -EINVAL;

	for (i = 0; i < temp_range_cnt; ++i)
		if (temp > oem0->temp_mon_range[i].temp_up_lim)
			break;
	return i-1;
}

static int bxt_algo_get_charge_params(struct charging_algo_request *req,
					struct charging_algo_params *params)
{
	int i;

	if (!oem0)
		return -ENODEV;

	i = get_temperature_zone();
	if (i < 0) {
		pr_err("%s: invalid temp zone(%d)\n", __func__, i);
		params->charge_current = 0;
		params->charge_voltage = 0;
		params->vrechg_delta = 0;
		return 0;
	}

	if (req->charging_status != POWER_SUPPLY_STATUS_FULL) {
		params->charge_current = oem0->temp_mon_range[i].full_chrg_cur;
		params->charge_voltage = oem0->temp_mon_range[i].full_chrg_vol;
	} else {
		params->charge_current = oem0->temp_mon_range[i].maint_chrg_cur;
		params->charge_voltage =
				oem0->temp_mon_range[i].maint_chrg_vol_ul;
	}
	params->vrechg_delta = oem0->temp_mon_range[i].maint_chrg_vol_ul -
				oem0->temp_mon_range[i].maint_chrg_vol_ll;

	/* convert CC/CV to uA and uV */
	params->charge_current *= 1000;
	params->charge_voltage *= 1000;
	params->vrechg_delta *= 1000;
	return 0;
}

static struct charging_algo pse_algo = {
	.name			=	"broxton_pse",
	.supplied_to		=	"INTN0001",
	.get_charging_params	=	bxt_algo_get_charge_params,
};

static int __init bxt_charging_algo_init(void)
{
	int ret;

	ret = algo_intel_get_oem0_table();
	if (ret < 0) {
		pr_err("%s: failed to get oem0 table\n", __func__);
		return ret;
	}

	ret = charging_algo_register(&pse_algo);
	if (ret < 0) {
		pr_err("%s: failed to register algo\n", __func__);
		return ret;
	}

	pr_info("%s: bxt charging algo registration success\n", __func__);
	return 0;
}
early_initcall(bxt_charging_algo_init);

static void __exit bxt_charging_algo_exit(void)
{
	charging_algo_unregister(&pse_algo);
}
module_exit(bxt_charging_algo_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Intel Broxton PSE Algo Driver");
MODULE_LICENSE("GPL v2");
