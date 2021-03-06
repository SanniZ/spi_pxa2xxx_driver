/*
 * pi3usb30532_mux.c: PI3USB30532 mux driver for selecting configuration
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
 * Author: Albin B <albin.bala.krishnan@intel.com>
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/usb_typec_phy.h>
#include <linux/extcon.h>
#include <linux/acpi.h>

#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/devinfo.h>

/* I2C control register's offsets */
#define PI3USB30532_SLAVE_ADDR_REG	0x00
#define PI3USB30532_VENDOR_ID_REG	0x01
#define PI3USB30532_SEL_CTRL_REG	0x02

#define CONF_MASK                       0x07
#define PI3USB30532_VENDOR_ID		0

#define	PI3USBMUX_OPEN_WITHPD		0 /* switch open with power down */
#define	PI3USBMUX_OPEN_ONLYNOPD		1 /* switch open only, no power down */
#define	PI3USBMUX_4LDP1P2		2 /* 4 Lane of DP1.2 */
#define	PI3USBMUX_4LDP1P2_SWAP		3 /* 4 Lane of DP1.2 swap */
#define	PI3USBMUX_USB30			4 /* USB3.0 only */
#define	PI3USBMUX_USB30_SWAP		5 /* USB3.0 swap */
#define	PI3USBMUX_USB30N2LDP1P2		6 /* USB3.0 + 2 Lane of DP1.2 */
#define	PI3USBMUX_USB30N2LDP1P2_SWAP	7 /* USB3.0 + 2 Lane of DP1.2 swap */

struct pi3usb30532_mux {
	struct i2c_client *client;
	struct device *dev;
	struct notifier_block usb_nb;
	struct notifier_block host_nb;
	struct notifier_block dp_nb;
	struct work_struct mux_work;
	struct mutex event_lock;
	struct typec_phy *phy;
	u8 cur_config;
	struct extcon_dev *edev;
	int dp_cbl_state;
	bool first_hpd;
};

static struct pi3usb30532_mux *chip_ptr;

static int def_conf = PI3USBMUX_OPEN_WITHPD;

static void hpd_trigger(struct pi3usb30532_mux *chip, int state)
{
	dev_info(&chip->client->dev, "[HPD trigger] state : %d\n", state);

	if (state)
		pinctrl_pm_select_default_state(chip->dev);
	else
		pinctrl_pm_select_sleep_state(chip->dev);
}

static int pi330532_set_hpd_state(struct typec_phy *phy, bool state)
{
	if (!chip_ptr)
		return -ENODEV;

	hpd_trigger(chip_ptr, state);
	return 0;
}

/* read/write/modify pi3usb30532 register values */
static inline int pi3usb30532_mux_read_reg(struct i2c_client *client,
						u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "Error(%d) in reading reg %d\n",
				ret, reg);

	return ret;
}

static inline int pi3usb30532_mux_write_reg(struct i2c_client *client,
						u8 reg,
						u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "Error(%d) in writing %d to reg %d\n",
				ret, data, reg);

	return ret;
}

static inline int pi3usb30532_mux_modify_reg(struct i2c_client *client,
						u8 reg,
						u8 mask,
						u8 val)
{
	int ret;

	ret = pi3usb30532_mux_read_reg(client, reg);
	if (ret < 0)
		return ret;
	ret = (ret & ~mask) | (mask & val);
	return pi3usb30532_mux_write_reg(client, reg, ret);
}

static int pi3usb30532_mux_sel_ctrl(struct pi3usb30532_mux *chip, u8 conf)
{
	int ret = 0;

	dev_info(&chip->client->dev, "%s %d\n", __func__, conf);
	ret = pi3usb30532_mux_modify_reg(chip->client,
						PI3USB30532_SEL_CTRL_REG,
						(u8) CONF_MASK,
						conf);
	if (ret)
		dev_err(&chip->client->dev,
				"Error in writing configuration!!\n");

	return ret;
}

static void pi3usb30532_mux_event_worker(struct work_struct *work)
{
	struct pi3usb30532_mux *chip = container_of(work,
						struct pi3usb30532_mux,
						mux_work);
	u8 conf;
	int orientation;
	int dp_state = 0;

	if (IS_ERR_OR_NULL(chip->phy)) {
		dev_err(chip->dev, "cant get phy to determine orientation");
		return;
	}

	mutex_lock(&chip->event_lock);
	/*
	 * By default, system will be in HDMI mode, so
	 * set the the RX invert state to 0 on the first
	 * HPD event to enable the Type-C DP mode.
	 */
	if (!chip->first_hpd) {
		chip->first_hpd = true;
		hpd_trigger(chip, 0);
	}

	if (chip->edev) {
		/* Get display cable state */
		dp_state = extcon_get_cable_state_(chip->edev,
						EXTCON_TYPEC_DP_SRC);
		dev_info(&chip->client->dev, "%s: DP cable state=%d, type=%d\n",
			__func__, dp_state, chip->phy->dp_type);
	}

	orientation = typec_get_cc_orientation(chip->phy);
	dev_info(&chip->client->dev, "%s cable orientation: %d\n",
				 __func__, orientation);
	switch (orientation) {
	case TYPEC_POS_NORMAL:
		conf = PI3USBMUX_USB30;
		if (dp_state) {
			if (chip->phy->dp_type == TYPEC_DP_TYPE_2X)
				conf = PI3USBMUX_USB30N2LDP1P2;
			else if (chip->phy->dp_type == TYPEC_DP_TYPE_4X)
				conf = PI3USBMUX_4LDP1P2;
		}
		break;
	case TYPEC_POS_SWAP:
		conf = PI3USBMUX_USB30_SWAP;
		if (dp_state) {
			if (chip->phy->dp_type == TYPEC_DP_TYPE_2X)
				conf = PI3USBMUX_USB30N2LDP1P2_SWAP;
			else if (chip->phy->dp_type == TYPEC_DP_TYPE_4X)
				conf = PI3USBMUX_4LDP1P2_SWAP;
		}
		break;
	case TYPEC_POS_DISCONNECT:
	default:
		conf = def_conf;
		break;
	}

	chip->cur_config = conf;

	pi3usb30532_mux_sel_ctrl(chip, conf);
	if (chip->dp_cbl_state != dp_state) {
		hpd_trigger(chip, dp_state);
		chip->dp_cbl_state = dp_state;
	}
	mutex_unlock(&chip->event_lock);
}

static int pi3usb30532_usb_event_handler(struct notifier_block *nb,
						unsigned long event,
						void *param)
{
	struct pi3usb30532_mux *chip = container_of(nb,
				struct pi3usb30532_mux, usb_nb);

	schedule_work(&chip->mux_work);
	return NOTIFY_OK;
}

static int pi3usb30532_host_event_handler(struct notifier_block *nb,
						unsigned long event,
						void *param)
{
	struct pi3usb30532_mux *chip = container_of(nb,
				struct pi3usb30532_mux, host_nb);

	schedule_work(&chip->mux_work);
	return NOTIFY_OK;
}

static int pi3usb30532_dp_event_handler(struct notifier_block *nb,
						unsigned long event,
						void *param)
{
	struct pi3usb30532_mux *chip = container_of(nb,
				struct pi3usb30532_mux, dp_nb);

	schedule_work(&chip->mux_work);
	return NOTIFY_OK;
}

static int pi3usb30532_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct pi3usb30532_mux *chip;
	int pi3usb30532_vid;
	int ret;
	struct extcon_dev *edev;

	edev = extcon_get_extcon_dev("bxt_wcove_usbc");
	if (!edev)
		return -EPROBE_DEFER;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"I2C adapter %s doesn't support BYTE DATA transfer\n",
			adapter->name);
		return -EIO;
	}

	pi3usb30532_vid = pi3usb30532_mux_read_reg(client,
				PI3USB30532_VENDOR_ID_REG);
	if (pi3usb30532_vid != PI3USB30532_VENDOR_ID) {
		dev_err(&client->dev,
			"Error (%d) in reading PI3USB30532_VENDOR_ID\n",
			pi3usb30532_vid);
		return -EIO;
	}
	dev_info(&client->dev, "pi3usb30532 Vendor ID: 0x%x found!!\n",
				pi3usb30532_vid);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	chip->dev = &client->dev;
	chip->client = client;
	chip->edev = edev;
	INIT_WORK(&chip->mux_work, pi3usb30532_mux_event_worker);
	chip->usb_nb.notifier_call = pi3usb30532_usb_event_handler;
	chip->host_nb.notifier_call = pi3usb30532_host_event_handler;
	chip->dp_nb.notifier_call = pi3usb30532_dp_event_handler;
	mutex_init(&chip->event_lock);
	i2c_set_clientdata(client, chip);

	ret = extcon_register_notifier(chip->edev, EXTCON_USB_HOST,
			&chip->host_nb);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"failed to register extcon host cable notifier %d\n",
			ret);
		return -EINVAL;
	}
	ret = extcon_register_notifier(chip->edev, EXTCON_USB, &chip->usb_nb);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"failed to register extcon usb cable notifier %d\n",
			ret);
		goto usb_reg_fail;
	}
	ret = extcon_register_notifier(chip->edev, EXTCON_TYPEC_DP_SRC,
			&chip->dp_nb);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"failed to register extcon DP cable notifier %d\n",
			ret);
		goto dp_reg_fail;
	}

	/* Add Type-C HPD control interface */
	chip->phy = typec_get_phy(USB_TYPE_C);
	if (IS_ERR_OR_NULL(chip->phy)) {
		dev_err(&client->dev, "unable to get usb typec phy");
		goto get_phy_fail;
	}
	chip->phy->set_hpd_state = pi330532_set_hpd_state;
	chip_ptr = chip;

	chip->dp_cbl_state = 0;

	return 0;

get_phy_fail:
	extcon_unregister_notifier(chip->edev, EXTCON_TYPEC_DP_SRC,
			&chip->dp_nb);
dp_reg_fail:
	extcon_unregister_notifier(chip->edev, EXTCON_USB, &chip->usb_nb);
usb_reg_fail:
	extcon_unregister_notifier(chip->edev, EXTCON_USB_HOST,
			&chip->host_nb);
	return -EINVAL;
}

static int pi3usb30532_remove(struct i2c_client *client)
{
	struct pi3usb30532_mux *chip = i2c_get_clientdata(client);

	if (chip) {
		extcon_unregister_notifier(chip->edev, EXTCON_TYPEC_DP_SRC,
				&chip->dp_nb);
		extcon_unregister_notifier(chip->edev, EXTCON_USB,
				&chip->usb_nb);
		extcon_unregister_notifier(chip->edev, EXTCON_USB_HOST,
				&chip->host_nb);
	}

	return 0;
}

static int pi3usb30532_suspend(struct device *dev)
{
	struct pi3usb30532_mux *chip = dev_get_drvdata(dev);

	int ret;

	mutex_lock(&chip->event_lock);
	ret = pi3usb30532_mux_write_reg(chip->client,
				PI3USB30532_SEL_CTRL_REG,
				PI3USBMUX_OPEN_WITHPD);
	mutex_unlock(&chip->event_lock);
	dev_info(&chip->client->dev, "pi3usb30532 suspend\n");
	return ret;
}

static int pi3usb30532_resume(struct device *dev)
{
	struct pi3usb30532_mux *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->event_lock);
	pi3usb30532_mux_sel_ctrl(chip, chip->cur_config);
	mutex_unlock(&chip->event_lock);

	dev_info(&chip->client->dev, "pi3usb30532 resume\n");
	return 0;
}

static int pi3usb30532_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int pi3usb30532_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int pi3usb30532_runtime_idle(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops pi3usb30532_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pi3usb30532_suspend,
				pi3usb30532_resume)

	SET_RUNTIME_PM_OPS(pi3usb30532_runtime_suspend,
				pi3usb30532_runtime_resume,
				pi3usb30532_runtime_idle)
};

static const struct i2c_device_id pi3usb30532_id[] = {
	{"PI330532", 0},
	{"i2c-PI330532", 0},
	{"i2c-PI330532:00", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, pi3usb30532_id);

static struct acpi_device_id pi3usb30532_acpi_ids[] = {
	{"PI330532", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, pi3usb30532_acpi_ids);

static struct i2c_driver pi3usb30532_driver = {
	.driver = {
		.name = "pi330532",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(pi3usb30532_acpi_ids),
		.pm = &pi3usb30532_pm_ops,
	},
	.probe = pi3usb30532_probe,
	.remove = pi3usb30532_remove,
	.id_table = pi3usb30532_id,
};

static int __init pi3usb30532_init(void)
{
	return i2c_add_driver(&pi3usb30532_driver);
}
late_initcall(pi3usb30532_init);

static void __exit pi3usb30532_exit(void)
{
	i2c_del_driver(&pi3usb30532_driver);
}
module_exit(pi3usb30532_exit);

MODULE_AUTHOR("Albin B <albin.bala.krishnan@intel.com>");
MODULE_DESCRIPTION("PI3USB30532 - USB Mux Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:pi3usb30532");

module_param(def_conf, int, S_IRUGO);
MODULE_PARM_DESC(def_conf, "Default value of the mux when no cable is detected");
