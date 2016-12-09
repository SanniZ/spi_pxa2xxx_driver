/*
 *  intel_mid_vibra.c - Intel Vibrator for Intel Broxton platform
 *
 *  Copyright (C) 2015 Intel Corp
 *  Author: B, Jayachandran <jayachandran.b@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */



#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>

struct vibra_info {
	const struct attribute_group *vibra_attr_group;
	struct mutex lock;
	struct device *dev;
	struct gpio_desc *gpiod_en;
	struct pwm_device *pwm;
	int enabled;
	unsigned long *base_unit;
	unsigned long *duty_cycle;
	unsigned int max_base_unit;
	void (*enable)(struct vibra_info *info);
	void (*disable)(struct vibra_info *info);
	int (*pwm_configure)(struct vibra_info *info, bool enable);
	void __iomem *shim;
	const char *name;
	u8 max_duty_cycle;
};

struct mid_vibra_pdata {
	int base_unit;
	int gpio_en_index;
	const char *name;
	u8 time_divisor;
};

#define vibra_gpio_set_value_cansleep(info, v) \
	do { \
		if ((info)->gpiod_en) \
			gpiod_set_value_cansleep(((info)->gpiod_en), (v)); \
	} while (0)

#define vibra_gpio_free(info) \
	do { \
		if ((info)->gpiod_en) \
			gpiod_put((info)->gpiod_en); \
	} while (0)

static void vibra_disable(struct vibra_info *info)
{
	pr_debug("%s: Disable\n", __func__);

	mutex_lock(&info->lock);
	vibra_gpio_set_value_cansleep(info, 0);
	info->enabled = false;
	info->pwm_configure(info, false);
	mutex_unlock(&info->lock);
	pm_runtime_put(info->dev);
}

static void vibra_drv_enable(struct vibra_info *info)
{
	pr_debug("%s: Enable\n", __func__);

	pm_runtime_get_sync(info->dev);
	mutex_lock(&info->lock);
	vibra_gpio_set_value_cansleep(info, 1);
	/* Wait for 850us per spec, give 100us buffer */
	usleep_range(950, 1000);
	info->pwm_configure(info, true);
	info->enabled = true;
	mutex_unlock(&info->lock);
}

/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/

static ssize_t vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", info->enabled);

}

static ssize_t vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;
	struct vibra_info *info = dev_get_drvdata(dev);

	if (kstrtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	if (vibrator_enable == info->enabled)
		return len;
	else if (vibrator_enable == 0)
		info->disable(info);
	else if (vibrator_enable == 1)
		info->enable(info);
	else
		return -EINVAL;
	return len;
}

static unsigned long mid_vibra_base_unit;
static unsigned long mid_vibra_duty_cycle;

static DEVICE_ATTR(vibrator, S_IRUGO | S_IWUSR,
		   vibra_show_vibrator, vibra_set_vibrator);
static DEVICE_ULONG_ATTR(pwm_baseunit, S_IRUGO | S_IWUSR,
				 mid_vibra_base_unit);
static DEVICE_ULONG_ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR,
				 mid_vibra_duty_cycle);

static struct attribute *vibra_attrs[] = {
	&dev_attr_vibrator.attr,
	&dev_attr_pwm_baseunit.attr.attr,
	&dev_attr_pwm_ontime_div.attr.attr,
	0,
};

static const struct attribute_group vibra_attr_group = {
	.attrs = vibra_attrs,
};


/*** Module ***/
#if CONFIG_PM
static int intel_vibra_runtime_suspend(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	pr_debug("In %s\n", __func__);
	info->pwm_configure(info, false);
	return 0;
}

static int intel_vibra_runtime_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return 0;
}

static void intel_vibra_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	intel_vibra_runtime_resume(dev);
}

static const struct dev_pm_ops intel_mid_vibra_pm_ops = {
	.prepare = intel_vibra_runtime_suspend,
	.complete = intel_vibra_complete,
	.runtime_suspend = intel_vibra_runtime_suspend,
	.runtime_resume = intel_vibra_runtime_resume,
};
#endif

static struct vibra_info *mid_vibra_setup(struct device *dev,
				 struct mid_vibra_pdata *data)
{
	struct vibra_info *info;

	pr_debug("probe data div %x, base %x, name:%s\n",
							data->time_divisor,
							data->base_unit,
							data->name);

	info =  devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return NULL;

	if (data->gpio_en_index < 0) {
		pr_err("Invalid gpio index: %d\n",
					data->gpio_en_index);
		return NULL;
	}

	info->gpiod_en = gpiod_get_index(dev, KBUILD_MODNAME,
					 data->gpio_en_index, GPIOD_ASIS);
	if (IS_ERR(info->gpiod_en)) {
		pr_err("Failed to get gpio descriptor, %ld\n",
					PTR_ERR(info->gpiod_en));
		return NULL;
	}
	gpiod_direction_output(info->gpiod_en, 0);

	info->name = data->name;

	info->dev = dev;
	mutex_init(&info->lock);
	info->vibra_attr_group = &vibra_attr_group;
	mid_vibra_base_unit = data->base_unit;
	mid_vibra_duty_cycle = data->time_divisor;
	info->base_unit = &mid_vibra_base_unit;
	info->duty_cycle = &mid_vibra_duty_cycle;

	info->enable = vibra_drv_enable;
	info->disable = vibra_disable;

	return info;
}

static int vibra_pwm_configure(struct vibra_info *info, bool enable)
{
	unsigned int freq;
	unsigned int duty_cyc;
	unsigned int period_ns;
	unsigned int ontime_ns;

	if (enable) {
		freq = *info->base_unit;
		duty_cyc = *info->duty_cycle;

		if (freq)
			period_ns = (unsigned int)1000000000/freq;
		else {
			pr_err("%s: Err: PWM frequency is 0\n", __func__);
			return -EINVAL;
		}

		ontime_ns = (period_ns * duty_cyc)/100;

		pr_debug("%s: Config and enable vibra  device\n", __func__);
		pr_debug("frequency=%d; dutycycle=%d\n", freq, duty_cyc);
		pr_debug("period_ns=%d; ontime_ns=%d\n", period_ns, ontime_ns);
		pwm_config(info->pwm, (int)ontime_ns, (int)period_ns);
		pwm_enable(info->pwm);
	} else {
		pr_debug("%s: disable  vibra device\n", __func__);
		pwm_disable(info->pwm);
	}
	return 0;
}

static int intel_mid_plat_vibra_remove(struct platform_device *pdev)
{
	struct vibra_info *info = platform_get_drvdata(pdev);

	vibra_gpio_free(info);
	pwm_put(info->pwm);
	sysfs_remove_group(&info->dev->kobj, info->vibra_attr_group);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int intel_mid_plat_vibra_probe(struct platform_device *pdev)
{
	struct vibra_info *info;
	struct device *dev = &pdev->dev;
	const struct acpi_device_id *id;
	struct mid_vibra_pdata *data;
	int ret;
	int pwm_id = 0;


	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id) {
		pr_err("%s: could not get acpi device\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s for %s\n", __func__, id->id);

	data = (struct mid_vibra_pdata *)id->driver_data;
	if (!data) {
		pr_err("Invalid driver data\n");
		return -ENODEV;
	}

	info = mid_vibra_setup(dev, data);
	if (!info)
		return -ENODEV;

	info->pwm_configure = vibra_pwm_configure;

	info->pwm = pwm_request(pwm_id, "vibra-pwm");
	if (IS_ERR(info->pwm)) {
		pr_err("%s: Could not get pwm device, probe deferred\n",
					__func__);
		vibra_gpio_free(info);
		return -EPROBE_DEFER;
	}
	ret = sysfs_create_group(&dev->kobj, info->vibra_attr_group);
	if (ret) {
		pr_err("Could not register sysfs files\n");
		vibra_gpio_free(info);
		return ret;
	}

	platform_set_drvdata(pdev, info);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	pr_info("%s: vibra probe success\n", __func__);

	return ret;
}

static struct mid_vibra_pdata vibra_pdata_bxt = {
	.name			= "INT34E1",
	.time_divisor	= 80,
	.base_unit		= 20000,
	.gpio_en_index	= 0,
};

const struct acpi_device_id vibra_acpi_ids[] = {
	{ "INT34E1", (kernel_ulong_t) &vibra_pdata_bxt },
	{},
};
MODULE_DEVICE_TABLE(acpi, vibra_acpi_ids);

static struct platform_driver plat_vibra_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(vibra_acpi_ids),
#ifdef CONFIG_PM
		.pm = &intel_mid_vibra_pm_ops,
#endif
	},
	.probe = intel_mid_plat_vibra_probe,
	.remove = intel_mid_plat_vibra_remove,
};

/**
* intel_mid_vibra_init - Module init function
*
* Registers platform
* Init all data strutures
*/
static int __init intel_mid_vibra_init(void)
{
	int ret;

	ret = platform_driver_register(&plat_vibra_driver);
	if (ret)
		pr_err("Platform register failed\n");

	return ret;
}

/**
* intel_mid_vibra_exit - Module exit function
*
* Unregisters platform
* Frees all data strutures
*/
static void __exit intel_mid_vibra_exit(void)
{
	platform_driver_unregister(&plat_vibra_driver);
	pr_debug("intel_mid_vibra driver exited\n");
}

module_init(intel_mid_vibra_init);
module_exit(intel_mid_vibra_exit);

MODULE_ALIAS("acpi:intel_mid_vibra");
MODULE_DESCRIPTION("Intel(R) MID Vibra driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
