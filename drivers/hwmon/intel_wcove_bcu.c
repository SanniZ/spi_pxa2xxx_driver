/*
 * intel_wcove_bcu.c: Intel Whiskey Cove PMIC Burst Contorl Unit Driver
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Seee the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Albin Balakrishnan <albin.bala.krishnan@intel.com>
 *	   Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#define pr_fmt(fmt)  DRIVER_NAME": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/version.h>
#include <asm/intel_wcove_bcu.h>

static DEFINE_MUTEX(bcu_update_lock);

struct wcpmic_bcu_info {
	struct device *dev;
	struct platform_device *pdev;
	struct regmap *regmap;
	struct delayed_work vwarn2_evt_enable;
	int irq;
};

static void wcove_bcu_enable_trip_points(struct wcpmic_bcu_info *info)
{
	int i, ret;

	/*
	 * Enable the Voltage comparator logic, so that the output
	 * signals are asserted when a voltage drop occurs.
	 */
	for (i = 0; i < MAX_VOLTAGE_TRIP_POINTS; i++) {
		ret = regmap_update_bits(info->regmap, VWARN1_CFG_REG + i,
					VWARN1_EN, VWARN1_EN);
		if (ret)
			dev_err(info->dev,
				"Error in %s setting register 0x%x\n",
				__func__, (VWARN1_CFG_REG + i));
	}

}

static int wcove_bcu_program(struct wcpmic_bcu_info *info,
		struct wcpmic_bcu_config_data *config, int max_regs)
{
	int ret = 0, i;

	mutex_lock(&bcu_update_lock);
	for (i = 0; i < max_regs; i++) {
		ret = regmap_write(info->regmap,
					config[i].addr, config[i].data);
		if (ret < 0) {
			dev_err(info->dev,
				"%s error(%d) while writing addr 0x%02x\n",
				__func__, ret, config[i].addr);
			break;
		}
	}

	mutex_unlock(&bcu_update_lock);

	return ret;
}

static inline struct power_supply *wcove_bcu_get_psy_battery(enum psy_type type)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *psy;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		psy = (struct power_supply *)dev_get_drvdata(dev);
		if ((type == PSY_TYPE_BATTERY && IS_BATTERY(psy)) ||
			(type == PSY_TYPE_CHARGER && IS_CHARGER(psy))) {
			class_dev_iter_exit(&iter);
			return psy;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

/* Reading the Voltage now value of the battery */
static inline int wcove_bcu_get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = wcove_bcu_get_psy_battery(PSY_TYPE_BATTERY);
	if (!psy)
		return -EINVAL;

	ret = psy->desc->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = val.intval;

	return ret;
}

static int get_charger_online_status(struct wcpmic_bcu_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = wcove_bcu_get_psy_battery(PSY_TYPE_CHARGER);
	if (!psy) {
		dev_warn(info->dev, "unable to get psy\n");
		goto error;
	}

	/* checking the charger status */
	ret = psy->desc->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret < 0)
		goto error;

	return val.intval;

error:
	return -EINVAL;
}

/**
 * Initiate Graceful Shutdown by setting the SOC to 0% via battery driver and
 * post the power supply changed event to indicate the change in battery level.
 */
static int wcove_bcu_action_voltage_drop(struct wcpmic_bcu_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	/* Do not set battery capacity to 0 to initiate graceful shutdown, when
	 * battery is actually charging */
	ret = get_charger_online_status(info);
	if (ret != 0) {
		dev_warn(info->dev, "charger is online or error\n");
		return -EINVAL;
	}

	psy = wcove_bcu_get_psy_battery(PSY_TYPE_BATTERY);
	if (!psy) {
		dev_err(info->dev, "fail in getting psy\n");
		return -EINVAL;
	}

	/* Trigger graceful shutdown via battery driver by setting SOC to 0% */
	dev_info(info->dev, "Triggering Graceful Shutdown\n");
	val.intval = 0;
	ret = psy->desc->set_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret < 0)
		return ret;

	power_supply_changed(psy);
	return 0;
}

/**
 * wcove_bcu_vwarn2_evt_enable: delayed work queue function, which is used to
 * unmask (enable) the VWARN2 interrupt after the specified delay time while
 * scheduling.
 */
static void wcove_bcu_vwarn2_evt_enable(struct work_struct *work)
{
	int ret = 0;
	struct wcpmic_bcu_info *info = container_of(work,
						struct wcpmic_bcu_info,
						vwarn2_evt_enable.work);

	/* Unmasking BCU VWARN2 Interrupt, for the next interrupt occurrence */
	ret = regmap_update_bits(info->regmap, MBCUIRQ_REG,
					MVWARN2, 0);
	if (ret)
		dev_err(info->dev, "Error in unmasking vwarn2 interrupt\n");
}

static void wcove_bcu_handle_vwarn2_event(void *dev_data)
{
	int ret;
	unsigned int status;
	unsigned int beh_data;
	struct wcpmic_bcu_info *info = (struct wcpmic_bcu_info *)dev_data;

	ret = regmap_read(info->regmap, S_BCUIRQ_REG, &status);
	if (ret < 0)
		return;

	dev_dbg(info->dev, "S_BCUIRQ_REG: 0x%x\n", status);

	/* If Vsys is below WARN2 level no action required from driver */
	if (!(status & S_VWARN2)) {
		/* Vsys is above WARN2 level */
		dev_dbg(info->dev, "Recovered from VWARN2 Level\n");

		/* clearing S_BCUDISW2 signal if asserted */
		ret = regmap_read(info->regmap, BCUDISW2_BEH_REG, &beh_data);
		if (ret < 0)
			goto fail;

		if (IS_ASSRT_ON_BCUDISB(beh_data) &&
				IS_BCUDISB_STICKY(beh_data)) {
			/* Clear the Status of the BCUDISB Output Signal */
			ret = regmap_update_bits(info->regmap, S_BCUCTRL_REG,
					S_BCUDISW2, S_BCUDISW2);
			if (ret)
				goto fail;
		}
	} else {

		dev_warn(info->dev, "VWARN2 Event has occurred\n");
		/**
		 * Masking BCU VWARN2 Interrupt, to avoid multiple VWARN2
		 * interrupt occurrence continuously.
		 */
		ret = regmap_update_bits(info->regmap, MBCUIRQ_REG,
					MVWARN2, MVWARN2);
		if (ret) {
			dev_err(info->dev, "Error in masking vwarn2 interrupt\n");
			goto fail;
		}

		cancel_delayed_work_sync(&info->vwarn2_evt_enable);
		/**
		 * Schedule the work to re-enable the VWARN2 interrupt after
		 * 30sec delay
		 */
		schedule_delayed_work(&info->vwarn2_evt_enable,
					VWARN2_INTR_EN_DELAY);
	}

	return;
fail:
	dev_err(info->dev, "Register read/write failed:func:%s()\n", __func__);
}

static void wcove_bcu_handle_vwarn1_event(void *dev_data)
{
	int ret;
	struct wcpmic_bcu_info *info = (struct wcpmic_bcu_info *)dev_data;

	dev_info(info->dev, "VWARN1 Event has occurred\n");
	ret = wcove_bcu_action_voltage_drop(info);
	if (ret) {
		dev_warn(info->dev, "No action taken for vwarn1 event\n");
		return;
	}

	/**
	 * Masking the BCU MVWARN1 and MVWARN2 Interrupt, since software does
	 * graceful shutdown once VWARN1 interrupt occurs. So we never expect
	 * another VWARN1 or VWARN2 interrupt.
	 */
	ret = regmap_update_bits(info->regmap, MBCUIRQ_REG,
				(MVWARN1 | MVWARN2), (MVWARN1 | MVWARN2));
	if (ret)
		dev_err(info->dev, "Error in masking vwarn1 and vwarn2 intr\n");
}

static irqreturn_t wcove_bcu_intr_thread_handler(int irq, void *dev_data)
{
	int ret = IRQ_NONE;
	int bat_volt;
	unsigned int irq_data;
	struct wcpmic_bcu_info *info = (struct wcpmic_bcu_info *)dev_data;

	if (!info)
		return ret;

	mutex_lock(&bcu_update_lock);
	if (wcove_bcu_get_battery_voltage(&bat_volt))
		dev_err(info->dev, "Error in getting battery voltage\n");
	else
		dev_dbg(info->dev, "Battery Volatge= %dmV\n", (bat_volt/1000));

	if (regmap_read(info->regmap, BCUIRQ_REG, &irq_data) < 0)
		goto fail;

	/* No action/Not handling for GSMPULSE and TXPWRTH events */
	if (irq_data & VCRIT)
		/* VCRIT shutdown based on register configuration */
		dev_info(info->dev, "VCRIT Event has occurred\n");

	if (irq_data & VWARN1)
		wcove_bcu_handle_vwarn1_event(dev_data);

	if (irq_data & VWARN2)
		wcove_bcu_handle_vwarn2_event(dev_data);

	if (irq_data & GSMPULSE)
		dev_info(info->dev, "GSMPULSE Event has occurred\n");

	if (irq_data & TXPWRTH)
		dev_info(info->dev, "TXPWRTH Event has occurred\n");

	ret = IRQ_HANDLED;

fail:
	/* Clear the handled interrupts */
	regmap_write(info->regmap, BCUIRQ_REG, irq_data & BCU_INTS);
	mutex_unlock(&bcu_update_lock);
	return ret;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static int wcove_bcu_probe(struct platform_device *pdev)
{
	int ret;
	int pirq;
	struct wcpmic_bcu_info *info;
	struct wcpmic_bcu_config_data *bcu_config;
	struct wcove_bcu_platform_data *pdata;
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);

	pdata = (struct wcove_bcu_platform_data *)pdev->dev.platform_data;
	if (!pdata && !pdata->config) {
		dev_err(&pdev->dev, "no platform/config data supplied\n");
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev,
			sizeof(struct wcpmic_bcu_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdev = pdev;
	info->regmap = wcove->regmap;
	pirq = platform_get_irq(pdev, 0);
	info->irq = regmap_irq_get_virq(wcove->irq_chip_data_level2, pirq);
	platform_set_drvdata(pdev, info);

	/* Registering with hwmon class */
	info->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(info->dev)) {
		ret = PTR_ERR(info->dev);
		info->dev = NULL;
		dev_err(&pdev->dev, "hwmon_dev_regs failed\n");
		goto exit;
	}

	/* Register for Interrupt Handler */
	INIT_DELAYED_WORK(&info->vwarn2_evt_enable,
				wcove_bcu_vwarn2_evt_enable);
	ret = devm_request_threaded_irq(&pdev->dev, info->irq, NULL,
					wcove_bcu_intr_thread_handler,
					IRQF_ONESHOT, DRIVER_NAME, info);
	if (ret) {
		dev_err(&pdev->dev,
			"request_threaded_irq failed:%d\n", ret);
		goto exit_hwmon;
	}
	bcu_config = pdata->config;

	/* Program the BCU with default values read from the platform */
	ret = wcove_bcu_program(info, bcu_config, pdata->num_regs);
	if (ret) {
		dev_err(&pdev->dev, "wcove_bcu_program() failed:%d\n", ret);
		goto exit_hwmon;
	}
	/* enable voltage and current trip points */
	wcove_bcu_enable_trip_points(info);

	/* Unmask BCU interrupts in the mask register */
	ret = regmap_update_bits(info->regmap, MBCUIRQ_REG,
					BCU_DEF_INTR_MASK, 0);
	if (ret) {
		dev_err(&pdev->dev, "Unmasking of BCU failed:%d\n", ret);
		goto exit_hwmon;
	}

	return 0;

exit_hwmon:
	hwmon_device_unregister(info->dev);
exit:
	return ret;
}

static int wcove_bcu_remove(struct platform_device *pdev)
{
	struct wcpmic_bcu_info *info = platform_get_drvdata(pdev);

	if (info) {
		cancel_delayed_work_sync(&info->vwarn2_evt_enable);
		hwmon_device_unregister(info->dev);
	}
	return 0;
}

static const struct platform_device_id wcove_bcu_id_table[] = {
	{ .name = DEVICE_NAME },
	{},
};
MODULE_DEVICE_TABLE(platform, wcove_bcu_id_table);

static struct platform_driver wcpmic_bcu_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = wcove_bcu_probe,
	.remove = wcove_bcu_remove,
	.id_table = wcove_bcu_id_table,
};
module_platform_driver(wcpmic_bcu_driver);

MODULE_AUTHOR("Albin Balakrishnan <albin.bala.krishnan@intel.com>");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Intel Whiskey Cove PMIC Burst Contorl Unit Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
