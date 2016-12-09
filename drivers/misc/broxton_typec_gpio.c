/*
 * broxton_typec_gpio.c - Intel Broxton Type-C GPIO Control Driver
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Albin B <albin.bala.krishnan@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/power/bq25890_charger.h>
#include <linux/usb_typec_phy.h>

#define BXT_TYPEC_GPIO_VCHGIN	"vchgin_desc"
#define BXT_TYPEC_GPIO_OTG		"otg_desc"
#define BXT_TYPEC_GPIO_VCONN	"vconn_desc"

#define MAX_UPDATE_VBUS_RETRY_COUNT	3
#define VBUS_UPDATE_DIFFERED_DELAY	100

struct bxt_typec_gpio_info {
	struct platform_device *pdev;
	struct notifier_block nb;
	struct extcon_specific_cable_nb otg_cable_obj;
	struct gpio_desc *gpio_vchgrin;
	struct gpio_desc *gpio_otg;
	struct gpio_desc *gpio_vconn;
	struct list_head gpio_queue;
	struct work_struct gpio_work;
	struct typec_phy *phy;
	struct mutex lock;
	bool is_vbus_connected;
	spinlock_t gpio_queue_lock;
};

static struct bxt_typec_gpio_info *info_ptr;

struct bxt_typec_gpio_event {
	struct list_head node;
	bool is_src_connected;
};

static int bxt_typec_set_vconn(struct typec_phy *phy, bool state)
{
	if (!info_ptr)
		return -ENODEV;

	gpiod_set_value_cansleep(info_ptr->gpio_vconn, state);
	dev_info(&info_ptr->pdev->dev, "%s: vconn=%d\n",
						__func__, state);
	return 0;
}

static int bxt_typec_set_vbus(struct typec_phy *phy, bool state)
{
	if (!info_ptr)
		return -ENODEV;

	/* enable/disable vbus based on the provider(source) event */
	gpiod_set_value_cansleep(info_ptr->gpio_otg, state);
	bq25890_set_vboost_control(state);
	dev_info(&info_ptr->pdev->dev, "%s: VBUS=%d\n",
						__func__, state);
	return 0;
}

static void wcgpio_ctrl_worker(struct work_struct *work)
{
	struct bxt_typec_gpio_info *info =
		container_of(work, struct bxt_typec_gpio_info, gpio_work);
	struct bxt_typec_gpio_event *evt, *tmp;
	unsigned long flags;
	struct list_head new_list;

        if (list_empty(&info->gpio_queue))
                return;

	spin_lock_irqsave(&info->gpio_queue_lock, flags);
        list_replace_init(&info->gpio_queue, &new_list);
	spin_unlock_irqrestore(&info->gpio_queue_lock, flags);

	list_for_each_entry_safe(evt, tmp, &new_list, node) {
		dev_info(&info->pdev->dev,
				"%s:%d state=%d\n", __FILE__, __LINE__,
				evt->is_src_connected);
		info->is_vbus_connected = evt->is_src_connected;

		/* enable/disable vbus based on the provider(source) event */
		gpiod_set_value_cansleep(info->gpio_otg,
						evt->is_src_connected);

		list_del(&evt->node);
		kfree(evt);
	}
}

static int wcgpio_check_events(struct bxt_typec_gpio_info *info,
					struct extcon_dev *edev)
{
	struct bxt_typec_gpio_event *evt;

	if (!edev)
		return -EIO;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&info->pdev->dev,
			"failed to allocate memory for SDP/OTG event\n");
		return -ENOMEM;
	}

	evt->is_src_connected = extcon_get_cable_state(edev, "TYPEC-SRC");
	dev_info(&info->pdev->dev,
			"[extcon notification] evt: Provider - %s\n",
			evt->is_src_connected ? "Connected" : "Disconnected");

	INIT_LIST_HEAD(&evt->node);
	spin_lock(&info->gpio_queue_lock);
	list_add_tail(&evt->node, &info->gpio_queue);
	spin_unlock(&info->gpio_queue_lock);

	schedule_work(&info->gpio_work);
	return 0;
}

static int wcgpio_event_handler(struct notifier_block *nblock,
					unsigned long event, void *param)
{
	int ret = 0;
	struct bxt_typec_gpio_info *info =
			container_of(nblock, struct bxt_typec_gpio_info, nb);
	struct extcon_dev *edev = param;

	ret = wcgpio_check_events(info, edev);

	if (ret < 0)
		return NOTIFY_DONE;

	return NOTIFY_OK;
}

static void check_initial_events(struct bxt_typec_gpio_info *info)
{
	struct extcon_dev *edev;

	edev = extcon_get_extcon_dev("bxt_wcove_usbc");

	wcgpio_check_events(info, edev);
}

static int bxt_typec_gpio_probe(struct platform_device *pdev)
{
	struct bxt_typec_gpio_info *info;
	int ret;

	info = devm_kzalloc(&pdev->dev,
			sizeof(struct bxt_typec_gpio_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto error;
	}

	info->pdev = pdev;
	platform_set_drvdata(pdev, info);
	mutex_init(&info->lock);
	INIT_LIST_HEAD(&info->gpio_queue);
	INIT_WORK(&info->gpio_work, wcgpio_ctrl_worker);
	spin_lock_init(&info->gpio_queue_lock);

	info->nb.notifier_call = wcgpio_event_handler;
	ret = extcon_register_interest(&info->otg_cable_obj, NULL,
						"TYPEC-SRC",
						&info->nb);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to register extcon notifier for otg\n");
		goto error;
	}

	/* FIXME: hardcoding of the index 0, 1 & 2 should fix when upstreaming.
	 * However ACPI _DSD is not support in Gmin yet and we need to live
	 * with it.
	 */
	info->gpio_vchgrin = devm_gpiod_get_index(&pdev->dev,
						BXT_TYPEC_GPIO_VCHGIN, 0,
						GPIOD_OUT_LOW);
	if (IS_ERR(info->gpio_vchgrin)) {
		dev_err(&pdev->dev, "Can't request gpio_vchgrin\n");
		ret = PTR_ERR(info->gpio_vchgrin);
		goto error_gpio;
	}

	info->gpio_otg = devm_gpiod_get_index(&pdev->dev,
						BXT_TYPEC_GPIO_OTG, 1,
						GPIOD_OUT_LOW);
	if (IS_ERR(info->gpio_otg)) {
		dev_err(&pdev->dev, "Can't request gpio_otg\n");
		ret = PTR_ERR(info->gpio_otg);
		goto error_gpio;
	}

	info->gpio_vconn = devm_gpiod_get_index(&pdev->dev,
						BXT_TYPEC_GPIO_VCONN, 2,
						GPIOD_OUT_LOW);
	if (IS_ERR(info->gpio_vconn)) {
		dev_err(&pdev->dev, "Can't request gpio_vconn\n");
		ret = PTR_ERR(info->gpio_vconn);
		goto error_gpio;
	}

	/* Add Type-C VBUS and VCONN control interfaces */
	info->phy = typec_get_phy(USB_TYPE_C);
	info->phy->enable_vbus = bxt_typec_set_vbus;
	info->phy->supply_vconn = bxt_typec_set_vconn;
	info_ptr = info;

	check_initial_events(info);

	return 0;

error_gpio:
	extcon_unregister_interest(&info->otg_cable_obj);
error:
	return ret;
}

static int bxt_typec_gpio_remove(struct platform_device *pdev)
{
	struct bxt_typec_gpio_info *info =  dev_get_drvdata(&pdev->dev);

	if (info)
		extcon_unregister_interest(&info->otg_cable_obj);

	return 0;
}

static int bxt_typec_gpio_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bxt_typec_gpio_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops bxt_typec_gpio_driver_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bxt_typec_gpio_suspend,
				bxt_typec_gpio_resume)
};

static struct acpi_device_id bxt_typec_gpio_acpi_ids[] = {
	{"GPTC0001"},
	{}
};
MODULE_DEVICE_TABLE(acpi, bxt_typec_gpio_acpi_ids);

static struct platform_device_id bxt_typec_gpio_device_ids[] = {
	{"GPTC0001", 0},
	{},
};

static struct platform_driver bxt_typec_gpio_driver = {
	.driver = {
		.name = "gptc0001",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(bxt_typec_gpio_acpi_ids),
		.pm = &bxt_typec_gpio_driver_pm_ops,
	},
	.probe = bxt_typec_gpio_probe,
	.remove = bxt_typec_gpio_remove,
	.id_table = bxt_typec_gpio_device_ids,
};

static int __init bxt_typec_gpio_init(void)
{
	int ret;
	ret =  platform_driver_register(&bxt_typec_gpio_driver);
	return ret;
}
late_initcall(bxt_typec_gpio_init);

static void __exit bxt_typec_gpio_exit(void)
{
	platform_driver_unregister(&bxt_typec_gpio_driver);
}
module_exit(bxt_typec_gpio_exit)

MODULE_AUTHOR("Albin B<albin.bala.krishnan@intel.com>");
MODULE_DESCRIPTION("Intel Whiskey Cove GPIO Driver");
MODULE_LICENSE("GPL");
