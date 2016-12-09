/*
 * wcove_misc_config.c - Intel Whiskey Cove PMIC Miscellaneous Config Driver
 *
 * Copyright (C) 2016 Intel Corporation
 * Author: Albin Balakrishnan <albin.bala.krishnan@intel.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/miscdevice.h>

#define WCOVE_VBUSDETCTRL_REG		0x5e1d
#define VBUSDETCTRL_DET_TYPE		(1 << 1)

#define VBUSDET_TYPE_TEXT_MAX_LEN	8
#define VBUSDET_TYPE_EDGE_TEXT		"edge"
#define VBUSDET_TYPE_LEVEL_TEXT		"level"

struct wcove_misccfg_info {
	struct device *dev;
	struct regmap *regmap;
	struct miscdevice misc_dev;
};

static ssize_t wcove_misccfg_set_vbus_det_type(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t count)
{
	struct wcove_misccfg_info *info = dev_get_drvdata(dev);
	unsigned int data;
	int ret = -EINVAL;

	if (!strncmp(buf, VBUSDET_TYPE_EDGE_TEXT, count - 1))
		data = VBUSDETCTRL_DET_TYPE;
	else if (!strncmp(buf, VBUSDET_TYPE_LEVEL_TEXT, count - 1))
		data = ~VBUSDETCTRL_DET_TYPE;
	else
		goto error;

	ret = regmap_update_bits(info->regmap, WCOVE_VBUSDETCTRL_REG,
					VBUSDETCTRL_DET_TYPE, data);
	if (ret < 0) {
		dev_err(info->dev,
			"%s Error in updating vbus detecting type(%d)\n",
			__func__, data);
		goto update_error;
	}
	return count;

error:
	dev_err(info->dev, "%s Wrong input data{%s}\n", __func__, buf);
update_error:
	return ret;
}

static ssize_t wcove_misccfg_get_vbus_det_type(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct wcove_misccfg_info *info = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	size_t count = 0;

	ret = regmap_read(info->regmap, WCOVE_VBUSDETCTRL_REG, &val);
	if (ret < 0) {
		dev_err(info->dev, "failed to read vbus detctrl reg %d\n", ret);
		return ret;
	}

	if (val & VBUSDETCTRL_DET_TYPE)
		count = snprintf(buf, VBUSDET_TYPE_TEXT_MAX_LEN,
					"%s\n", VBUSDET_TYPE_EDGE_TEXT);
	else
		count = snprintf(buf, VBUSDET_TYPE_TEXT_MAX_LEN,
					"%s\n", VBUSDET_TYPE_LEVEL_TEXT);

	return count;
}

static DEVICE_ATTR(vbus_det_type, S_IWUSR | S_IRUGO,
	wcove_misccfg_get_vbus_det_type, wcove_misccfg_set_vbus_det_type);

static const struct attribute *wcove_misccfg_attrs[] = {
	&dev_attr_vbus_det_type.attr,
	NULL,
};

static int wcove_wakecfg_sysfs_init(struct wcove_misccfg_info *info)
{
	int ret;

	info->misc_dev.minor = MISC_DYNAMIC_MINOR;
	info->misc_dev.name = "pmic";
	info->misc_dev.mode = (S_IWUSR | S_IRUGO);
	ret = misc_register(&info->misc_dev);
	if (ret) {
		dev_err(info->dev,
				"Error(%d) in registering misc class", ret);
		return ret;
	}

	/* create sysfs file for vbus_det_type */
	dev_set_drvdata(info->misc_dev.this_device, info);
	ret = sysfs_create_files(&info->misc_dev.this_device->kobj,
					wcove_misccfg_attrs);
	if (ret) {
		dev_err(info->dev, "cannot create sysfs entry\n");
		misc_deregister(&info->misc_dev);
	}

	return ret;
}

static int wcove_misccfg_probe(struct platform_device *pdev)
{
	struct wcove_misccfg_info *info;
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->regmap = wcove->regmap;
	platform_set_drvdata(pdev, info);

	/* Register and create sysfs interfaces */
	ret = wcove_wakecfg_sysfs_init(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to init sysfs interface %d\n", ret);
		goto failed;
	}

	return 0;

failed:
	return ret;
}

static int wcove_misccfg_remove(struct platform_device *pdev)
{
	struct wcove_misccfg_info *info = platform_get_drvdata(pdev);

	if (info) {
		if (!IS_ERR_OR_NULL(info->misc_dev.this_device)) {
			sysfs_remove_files(&info->misc_dev.this_device->kobj,
						wcove_misccfg_attrs);
			misc_deregister(&info->misc_dev);
		}
	}

	return 0;
}

static struct platform_device_id wcove_misccfg_device_ids[] = {
	{"wcove_misccfg", 0},
	{"bxt_wcove_misccfg", 0},
	{},
};

static struct platform_driver wcove_misccfg_driver = {
	.driver = {
		   .name = "wcove_misccfg",
		   .owner = THIS_MODULE,
		   },
	.probe = wcove_misccfg_probe,
	.remove = wcove_misccfg_remove,
	.id_table = wcove_misccfg_device_ids,
};
module_platform_driver(wcove_misccfg_driver);
MODULE_AUTHOR("Albin Balakrishnan <albin.bala.krishnan@intel.com>");
MODULE_DESCRIPTION("Intel Whiskey Cove PMIC Miscellaneous Config Driver");
MODULE_LICENSE("GPL v2");
