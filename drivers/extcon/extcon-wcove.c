/*
 * extcon-wcove.c - Intel Whiskey Cove PMIC extcon driver
 *
 * Copyright (C) 2015 Intel Corporation
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
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
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>
#include <linux/usb/otg.h>
#include <linux/notifier.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/extcon.h>

#define WCOVE_TYPEC_EXTCON_DEV_NAME "bxt_wcove_usbc"

#define WCOVE_ID0_REG				0x4e00
#define ID0_MAJREV_MASK				(7 << 3)
#define ID0_MAJREV_AX				(0 << 3)
#define ID0_MAJREV_BX				(1 << 3)
#define ID0_MAJREV_CX				(2 << 3)
#define ID0_MAJREV_DX				(3 << 3)
#define ID0_MAJREV_EX				(4 << 3)
#define ID0_MINREV_MASK				(7 << 0)
#define ID0_MINREV_0				(0 << 0)
#define ID0_MINREV_1				(1 << 0)
#define ID0_MINREV_2				(2 << 0)
#define ID0_MINREV_3				(3 << 0)
#define ID0_MINREV_4				(4 << 0)

#define WCOVE_CHGRIRQ0_REG		0x4e09
#define CHGRIRQ0_CTYPE			(1 << 4)

#define WCOVE_CHGRIRQ1_REG		0x4e0a
#define CHGRIRQ1_USBID_GND_DET		(1 << 4)
#define CHGRIRQ1_USBID_FLT_DET		(1 << 3)
#define CHGRIRQ1_DC_ADP_DET		(1 << 1)
#define CHGRIRQ1_VBUS_DET		(1 << 0)
#define CHGRIRQ1_DEF_IRQ_EN	(CHGRIRQ1_USBID_GND_DET| \
		CHGRIRQ1_USBID_FLT_DET|CHGRIRQ1_DC_ADP_DET|CHGRIRQ1_VBUS_DET)

#define WCOVE_MCHGRIRQ0_REG		0x4e17
#define MCHGRIRQ0_CTYPE			(1 << 4)

#define WCOVE_MCHGRIRQ1_REG		0x4e18
#define MCHGRIRQ1_USBID_GND_DET		(1 << 4)
#define MCHGRIRQ1_USBID_FLT_DET		(1 << 3)
#define MCHGRIRQ1_DC_ADP_DET		(1 << 1)
#define MCHGRIRQ1_VBUS_DET		(1 << 0)

#define WCOVE_SPWRSRC_REG		0x4e20
#define SPWRSRC_USBIDDET_MASK		(3 << 3)
#define SPWRSRC_USBIDDET_RID_ACA	(0 << 3)
#define SPWRSRC_USBIDDET_RID_GND	(1 << 3)
#define SPWRSRC_USBIDDET_RID_FLT	(2 << 3)
#define SPWRSRC_USBIDDET_RESERVE	(3 << 3)
#define SPWRSRC_DC_ADP_DET		(1 << 1)
#define SPWRSRC_VBUS_DET		(1 << 0)

#define WCOVE_USBID_CNTL_REG		0x5e05
#define USBID_CNTL_ACA_DETEN		(1 << 1)
#define USBID_CNTL_ID_DETEN		(1 << 0)

#define WCOVE_USBID_DET_TYPE_REG	0x5e06
#define USBID_DET_CUR_CONTINEOUS	(1 << 1)
#define USBID_DET_TYPE_EDGE		(1 << 0)

#define WCOVE_USBPHY_CTRL_REG		0x5e07
#define USBPHY_CTRL_CTYP_DIS		(1 << 3)
#define USBPHY_CTRL_CTYP_START		(1 << 2)
#define USBPHY_CTRL_CHGDET_N_POL	(1 << 1)
#define USBPHY_CTRL_RSTB		(1 << 0)

#define WCOVE_CHGRCTRL0_REG		0x5e16
#define CHGRCTRL0_CCSM_OFF		(1 << 5)
#define CHGRCTRL0_SW_CONTROL		(1 << 3)

#define WCOVE_CHGRCTRL1_REG		0x5e17
#define CHGRCTRL1_OTGMODE		(1 << 6)

#define WCOVE_USB_SRCDET_STATUS0_REG	0x5e29
#define STATUS0_SDCD_MASK		(3 << 6)
#define STATUS0_SDCD_NOT_DET		(0 << 6)
#define STATUS0_SDCD_PRGS		(1 << 6)
#define STATUS0_SDCD_DONE		(2 << 6)
#define STATUS0_SDCD_FAIL		(3 << 6)
#define STATUS0_RESULT_MASK		(0xf << 2)
#define STATUS0_RESULT_NOT_DET		(0 << 2)
#define STATUS0_RESULT_SDP		(1 << 2)
#define STATUS0_RESULT_DCP		(2 << 2)
#define STATUS0_RESULT_CDP		(3 << 2)
#define STATUS0_RESULT_ACA		(4 << 2)
#define STATUS0_RESULT_SE1		(5 << 2)
#define STATUS0_RESULT_MHL		(6 << 2)
#define STATUS0_RESULT_FLT		(7 << 2)
#define STATUS0_RESULT_OTH		(8 << 2)
#define STATUS0_RESULT_EXT_DCP		(9 << 2)
#define STATUS0_BC12_STAT_MASK		(3 << 0)
#define STATUS0_BC12_STAT_NOT_STR	(0 << 0)
#define STATUS0_BC12_STAT_PRGS		(1 << 0)
#define STATUS0_BC12_STAT_DONE		(2 << 0)
#define STATUS0_BC12_STAT_FAIL		(3 << 0)

#define WCOVE_USB_SRCDET_STATUS1_REG	0x5e2A
#define STATUS1_SSEC_STAT		(1 << 3)
#define STATUS1_SPRIM_STAT		(1 << 2)
#define STATUS1_SACA_STAT_MASK		(3 << 0)
#define STATUS1_SACA_STAT_NOT_DET	(0 << 0)
#define STATUS1_SACA_STAT_RID_GND	(1 << 0)
#define STATUS1_SACA_STAT_RID_NO_FLT	(2 << 0)
#define STATUS1_SACA_STAT_FAIL		(3 << 0)

/* WCOVE ADC RID raw measurement codes */
#define RID_A_MIN	11150
#define RID_A_MAX	13640
#define RID_B_MAX	7480
#define RID_B_MIN	6120
#define RID_C_MAX	4015
#define RID_C_MIN	3285

#define IS_RID_A(rid) (rid > RID_A_MIN && rid < RID_A_MAX)
#define IS_RID_B(rid) (rid > RID_B_MIN && rid < RID_B_MAX)
#define IS_RID_C(rid) (rid > RID_C_MIN && rid < RID_C_MAX)

enum wcove_extcon_irq {
	CHRG_BC12_IRQ = 0,
	PWR_DET_IRQ,
	EXTCON_IRQ_END,
};

static const unsigned int wcove_extcon_cables[] = {
	EXTCON_CHG_USB_SLOW,
	EXTCON_CHG_USB_CDP,
	EXTCON_CHG_USB_FAST,
	EXTCON_CHG_ACA_DOCK,
	EXTCON_CHG_USB_ACA,
	EXTCON_CHG_SE1,
	EXTCON_CHG_MHL,
	EXTCON_CHG_DC,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum wcove_extcon_aca_type {
	RID_UNKNOWN = 0,
	RID_A,
	RID_B,
	RID_C,
	RID_FLOAT,
	RID_GND,
};

enum wcove_extcon_cable_type {
	PMIC_CHARGER_TYPE_NONE = 0,
	PMIC_CHARGER_TYPE_SDP,
	PMIC_CHARGER_TYPE_DCP,
	PMIC_CHARGER_TYPE_CDP,
	PMIC_CHARGER_TYPE_ACA,
	PMIC_CHARGER_TYPE_SE1,
	PMIC_CHARGER_TYPE_MHL,
	PMIC_CHARGER_TYPE_FLOAT,
	PMIC_CHARGER_TYPE_OTHER,
	PMIC_CHARGER_TYPE_DCP_EXTPHY,
};

enum wcove_pmic_rev {
	WCOVE_PMIC_REV_UNKNOWN,
	WCOVE_PMIC_REV_AX,
	WCOVE_PMIC_REV_BX,
	WCOVE_PMIC_REV_CX,
};

struct wcove_extcon_info {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq_chip;
	enum wcove_pmic_rev pmic_rev;
	int irq[EXTCON_IRQ_END];
	struct extcon_dev *edev;
	struct usb_phy *otg;

	/* cables detected by other modules */
	struct {
		struct notifier_block nb;
		//struct extcon_specific_cable_nb host;
		bool host_mode;
		struct extcon_dev *c_edev;
	} cable;
};

static int wcove_extcon_get_rid(struct wcove_extcon_info *info)
{
	int ret;
	struct iio_channel *indio_chan;
	int spwrsrc, rid, id = RID_UNKNOWN;

	ret = regmap_read(info->regmap, WCOVE_SPWRSRC_REG, &spwrsrc);
	if (ret < 0) {
		dev_err(info->dev, "failed to read spwrsrc register\n");
		return RID_UNKNOWN;
	}

	if ((spwrsrc & SPWRSRC_USBIDDET_MASK) == SPWRSRC_USBIDDET_RID_GND)
		return RID_GND;
	else if ((spwrsrc & SPWRSRC_USBIDDET_MASK) == SPWRSRC_USBIDDET_RID_FLT)
		return RID_FLOAT;

	indio_chan = iio_channel_get(NULL, "USBID");
	if (IS_ERR_OR_NULL(indio_chan)) {
		dev_err(info->dev, "Failed to get IIO channel USBID\n");
		return RID_UNKNOWN;
	}

	ret = iio_read_channel_raw(indio_chan, &rid);
	if (ret) {
		dev_err(info->dev, "IIO channel read error for USBID\n");
		goto err_exit;
	}

	if (IS_RID_A(rid))
		id = RID_A;
	else if (IS_RID_B(rid))
		id = RID_B;
	else if (IS_RID_C(rid))
		id = RID_C;

err_exit:
	iio_channel_release(indio_chan);
	return id;
}

static unsigned int wcove_extcon_get_charger_cable(
		struct wcove_extcon_info *info,
		enum wcove_extcon_cable_type *ctype)
{
	int ret, rid, srcdet_stat0;

	/* Check for BC1.2 detection completion */
	ret = regmap_read(info->regmap,
			WCOVE_USB_SRCDET_STATUS0_REG, &srcdet_stat0);
	if (ret < 0) {
		dev_err(info->dev, "failed to srcdetstatus register\n");
		return EXTCON_NONE;
	}

	if ((srcdet_stat0 & STATUS0_BC12_STAT_MASK) !=
					STATUS0_BC12_STAT_DONE) {
		dev_err(info->dev,
			"failed to do BC1.2 charger detection\n");
		return EXTCON_NONE;
	}

	*ctype = (srcdet_stat0 & STATUS0_RESULT_MASK) >> 2;

	switch (*ctype) {
	case PMIC_CHARGER_TYPE_SDP:
	case PMIC_CHARGER_TYPE_FLOAT:
	case PMIC_CHARGER_TYPE_OTHER:
		dev_info(info->dev, "SDP charger detected\n");
		return EXTCON_CHG_USB_SLOW;
	case PMIC_CHARGER_TYPE_DCP:
	case PMIC_CHARGER_TYPE_DCP_EXTPHY:
		dev_info(info->dev, "DCP charger detected\n");
		return EXTCON_CHG_USB_FAST;
	case PMIC_CHARGER_TYPE_CDP:
		dev_info(info->dev, "CDP charger detected\n");
		return EXTCON_CHG_USB_CDP;
	case PMIC_CHARGER_TYPE_ACA:
		dev_info(info->dev, "ACA charger detected\n");
		rid = wcove_extcon_get_rid(info);
		if (rid == RID_A)
			return EXTCON_CHG_ACA_DOCK;
		/*
		 * As PMIC detected the charger as ACA, if RID
		 * detection failed report type as ACA
		 */
		return EXTCON_CHG_USB_ACA;
	case PMIC_CHARGER_TYPE_SE1:
		dev_info(info->dev, "SE1 charger detected\n");
		return EXTCON_CHG_SE1;
	case PMIC_CHARGER_TYPE_MHL:
		dev_info(info->dev, "MHL charger detected\n");
		return EXTCON_CHG_MHL;
	default:
		dev_err(info->dev, "unknown charger detected\n");
		return EXTCON_NONE;
	}

	return EXTCON_NONE;
}

static void wcove_extcon_handle_otg(struct wcove_extcon_info *info, bool enable)
{
	int ret;

	if (enable)
		ret = regmap_update_bits(info->regmap, WCOVE_CHGRCTRL1_REG,
					CHGRCTRL1_OTGMODE, CHGRCTRL1_OTGMODE);
	else
		ret = regmap_update_bits(info->regmap, WCOVE_CHGRCTRL1_REG,
					CHGRCTRL1_OTGMODE, 0);
	if (ret)
		dev_warn(info->dev, "failed to read/write chgrctrl1\n");
}

static void wcove_extcon_cfg_usb_switch(struct wcove_extcon_info *info,
								bool soc)
{
	int ret;

	if (soc)
		ret = regmap_update_bits(info->regmap, WCOVE_USBPHY_CTRL_REG,
					USBPHY_CTRL_RSTB, USBPHY_CTRL_RSTB);
	else
		ret = regmap_update_bits(info->regmap, WCOVE_USBPHY_CTRL_REG,
					USBPHY_CTRL_RSTB, 0);

	if (ret)
		dev_warn(info->dev, "failed to read/write usbphyctrl\n");
}

static int wcove_extcon_handle_cable_event(struct wcove_extcon_info *info)
{
	static bool notify_otg, notify_usb, notify_usb_psy, notify_dc_psy;
	unsigned int cable = EXTCON_NONE;
	int ret, chgirq0, chgirq1, spwrsrc;
	enum wcove_extcon_cable_type ctype;
	bool vbus_attach, adp_attach, otg_attach;

	ret = regmap_read(info->regmap, WCOVE_CHGRIRQ0_REG, &chgirq0);
	if (ret < 0) {
		dev_err(info->dev, "failed to read chgirq0\n");
		goto dev_det_ret;
	}

	ret = regmap_read(info->regmap, WCOVE_CHGRIRQ1_REG, &chgirq1);
	if (ret < 0) {
		dev_err(info->dev, "failed to read chgirq1\n");
		goto dev_det_ret;
	}

	ret = regmap_read(info->regmap, WCOVE_SPWRSRC_REG, &spwrsrc);
	if (ret < 0) {
		dev_err(info->dev, "failed to read spwrsrc\n");
		goto dev_det_ret;
	}

	/* Get cables attach status */
	vbus_attach = (spwrsrc & SPWRSRC_VBUS_DET);
	adp_attach = (spwrsrc & SPWRSRC_DC_ADP_DET);
	otg_attach = ((spwrsrc & SPWRSRC_USBIDDET_MASK)
				== SPWRSRC_USBIDDET_RID_GND);

	if (info->cable.host_mode) {
		dev_info(info->dev,
			"external USB Host cable(%d)\n", info->cable.host_mode);
		/*
		 * Override vbus attach and otg attach flags
		 * host mode detect events.
		 */
		cable = EXTCON_USB_HOST;
		vbus_attach = info->cable.host_mode;
		otg_attach = info->cable.host_mode;
		notify_otg = true;
	} else if (chgirq0 & CHGRIRQ0_CTYPE) {
		dev_info(info->dev, "BC1.2 interrupt\n");
		cable = wcove_extcon_get_charger_cable(info, &ctype);
		if (!cable) {
			dev_err(info->dev, "Invalid cable detected\n");
			goto ignore;
		}
		switch (ctype) {
		case PMIC_CHARGER_TYPE_SDP:
		case PMIC_CHARGER_TYPE_CDP:
		case PMIC_CHARGER_TYPE_ACA:
			notify_usb = true;
			break;
		default:
			notify_usb = false;
		}
		notify_usb_psy = true;

	} else if ((chgirq1 & CHGRIRQ1_USBID_GND_DET) ||
			(chgirq1 & CHGRIRQ1_USBID_FLT_DET)) {
		dev_info(info->dev, "USB ID GND(%d) interrupt\n", otg_attach);
		/*
		 * Override vbus attach flag in case of ID
		 * event as the VBUS is not from charger.
		 */
		vbus_attach = otg_attach;
		if (otg_attach) {
			cable = EXTCON_USB_HOST;
			notify_otg = true;
			notify_usb_psy = true;
		} else if (cable) {
			goto extcon_notify;
		} else {
			goto ignore;
		}
	} else if (chgirq1 & CHGRIRQ1_DC_ADP_DET) {
		dev_info(info->dev, "DC adapter interrupt\n");
		if (adp_attach) {
			cable = EXTCON_CHG_DC;
			notify_dc_psy = true;
		} else if (cable) {
			goto extcon_notify;
		} else {
			goto ignore;
		}
	} else if (chgirq1 & CHGRIRQ1_VBUS_DET) {
		dev_info(info->dev, "VBUS %s interrupt\n",
				vbus_attach ? "attach" : "detach");
		if (!vbus_attach && cable)
			goto extcon_notify;
		else
			goto ignore;
	} else {
		/* unknown event */
		dev_info(info->dev, "Unknown interrupt\n");
		goto ignore;
	}

extcon_notify:

	/* configure the USB switch accordingly */
	wcove_extcon_cfg_usb_switch(info, vbus_attach);
	wcove_extcon_handle_otg(info, otg_attach);

	if (notify_usb_psy)
		extcon_set_cable_state_(info->edev, cable, vbus_attach);

	if (notify_dc_psy)
		extcon_set_cable_state_(info->edev, cable, adp_attach);

	if (notify_usb)
		atomic_notifier_call_chain(&info->otg->notifier,
			vbus_attach ? USB_EVENT_VBUS : USB_EVENT_NONE, NULL);

	if (notify_otg)
		atomic_notifier_call_chain(&info->otg->notifier,
			otg_attach ? USB_EVENT_ID : USB_EVENT_NONE, NULL);

	/* Clear the flags on disconnect events */
	if (!adp_attach)
		notify_dc_psy = false;

	if (!otg_attach)
		notify_otg = false;

	if (!vbus_attach)
		notify_usb = false;

	if (!vbus_attach && !otg_attach)
		notify_usb_psy = false;

ignore:
	/* Clear or Ack the handled interrupts */
	regmap_write(info->regmap, WCOVE_CHGRIRQ0_REG,
					chgirq0 & CHGRIRQ0_CTYPE);
	regmap_write(info->regmap, WCOVE_CHGRIRQ1_REG,
					chgirq1 & CHGRIRQ1_DEF_IRQ_EN);

	return 0;

dev_det_ret:
	if (ret < 0)
		dev_err(info->dev, "BC Mod detection error\n");
	return ret;
}

static irqreturn_t wcove_extcon_threaded_handler(int irq, void *data)
{
	struct wcove_extcon_info *info = data;
	int ret;

	ret = wcove_extcon_handle_cable_event(info);
	if (ret < 0)
		dev_err(info->dev, "failed to handle extcon interrupt\n");

	return IRQ_HANDLED;
}

static int wcove_handle_extcon_events(struct notifier_block *nb,
				   unsigned long event, void *param)
{
	struct wcove_extcon_info *info =
		container_of(nb, struct wcove_extcon_info, cable.nb);

	dev_dbg(info->dev, "external connector event(%ld)\n", event);

	if (extcon_get_cable_state_(info->cable.c_edev,
					EXTCON_USB_HOST)) {
		dev_info(info->dev, "external USB-Host is connected");
		info->cable.host_mode = true;
	} else {
		dev_info(info->dev, "USB Host cable disconnected");
		info->cable.host_mode = false;
	}

	return NOTIFY_OK;
}

static int wcove_extcon_register_notifier(struct wcove_extcon_info *info)
{
	int ret;

	info->cable.nb.notifier_call = wcove_handle_extcon_events;
	ret = extcon_register_notifier(info->cable.c_edev, EXTCON_USB_HOST,
			&info->cable.nb);
	if (ret < 0) {
		dev_warn(info->dev,
			"extcon USB-Host registration failed(%d)\n", ret);
		goto host_reg_failed;
	}

	return 0;

host_reg_failed:
	return -EPROBE_DEFER;
}

static enum wcove_pmic_rev wcove_get_pmic_revision(struct wcove_extcon_info *info)
{
	int ret, version;
	enum wcove_pmic_rev pmic_rev;

	ret = regmap_read(info->regmap, WCOVE_ID0_REG, &version);
	if (ret < 0) {
		dev_err(info->dev, "failed to read wcove_id0 reg\n");
		return WCOVE_PMIC_REV_UNKNOWN;
	}

	if ((version & ID0_MAJREV_MASK) == ID0_MAJREV_AX) {
		dev_info(info->dev,
			"Whiskey Cove Ax Stepping detected, ID0(%x)\n", version);
		pmic_rev = WCOVE_PMIC_REV_AX;
	} else if ((version & ID0_MAJREV_MASK) == ID0_MAJREV_BX) {
		dev_info(info->dev,
			"Whiskey Cove Bx Stepping detected, ID0(%x)\n", version);
		pmic_rev = WCOVE_PMIC_REV_BX;
	} else if ((version & ID0_MAJREV_MASK) == ID0_MAJREV_CX) {
		dev_info(info->dev,
			"Whiskey Cove Bx Stepping detected, ID0(%x)\n", version);
		pmic_rev = WCOVE_PMIC_REV_CX;
	} else {
		dev_info(info->dev,
			"Whiskey Cove Unknown Stepping detected, ID0(%x)\n", version);
		pmic_rev = WCOVE_PMIC_REV_UNKNOWN;
	}

	return pmic_rev;
}

static void wcove_extcon_init_hw_registers(struct wcove_extcon_info *info)
{
	/* Enable SW control */
	regmap_update_bits(info->regmap, WCOVE_CHGRCTRL0_REG,
				CHGRCTRL0_CCSM_OFF | CHGRCTRL0_SW_CONTROL,
				CHGRCTRL0_CCSM_OFF | CHGRCTRL0_SW_CONTROL);

	/* Enable PMIC internal PHY */
	regmap_write(info->regmap, WCOVE_USBPHY_CTRL_REG, 0x0);

	/* Unmask extcon interrupt */
	regmap_update_bits(info->regmap, WCOVE_MCHGRIRQ0_REG,
					CHGRIRQ0_CTYPE, 0);
	regmap_update_bits(info->regmap, WCOVE_MCHGRIRQ1_REG,
					CHGRIRQ1_DEF_IRQ_EN, 0);
}

static int wcove_extcon_probe(struct platform_device *pdev)
{
	struct wcove_extcon_info *info;
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);
	int ret, i, pirq;
	struct extcon_dev *c_edev;
	struct usb_phy *otg;

	otg  = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(otg)) {
		dev_err(&pdev->dev, "failed to get otg transceiver\n");
		return -EPROBE_DEFER;
	}

	/* Get WCOVE Type-C phy extcon dev */
	c_edev = extcon_get_extcon_dev(WCOVE_TYPEC_EXTCON_DEV_NAME);
	if (!c_edev) {
		usb_put_phy(otg);
		dev_err(&pdev->dev, "%s is not ready, probe deferred\n",
				WCOVE_TYPEC_EXTCON_DEV_NAME);
		return -EPROBE_DEFER;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->regmap = wcove->regmap;
	info->regmap_irq_chip = wcove->irq_chip_data_level2;
	platform_set_drvdata(pdev, info);
	info->pmic_rev = wcove_get_pmic_revision(info);
	info->otg = otg;

	/* Initialize extcon device */
	info->edev = devm_extcon_dev_allocate(&pdev->dev,
					      wcove_extcon_cables);
	if (IS_ERR(info->edev)) {
		dev_err(&pdev->dev, "failed to allocate memory for extcon\n");
		return PTR_ERR(info->edev);
	}

	/* Register extcon device */
	ret = devm_extcon_dev_register(&pdev->dev, info->edev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return ret;
	}

	/*
	 * Register for external cable notifications
	 * in case PMIC integrated Type-C PHY.
	 */
	if (info->pmic_rev > WCOVE_PMIC_REV_AX) {
		info->cable.c_edev = c_edev;
		ret = wcove_extcon_register_notifier(info);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < EXTCON_IRQ_END; i++) {
		pirq = platform_get_irq(pdev, i);
		info->irq[i] = regmap_irq_get_virq(info->regmap_irq_chip, pirq);
		if (info->irq[i] < 0) {
			dev_err(&pdev->dev,
				"failed to get virtual interrupt=%d\n", pirq);
			ret = info->irq[i];
			goto gpio_req_failed;
		}

		ret = devm_request_threaded_irq(&pdev->dev, info->irq[i],
					NULL, wcove_extcon_threaded_handler,
					IRQF_ONESHOT, pdev->name, info);
		if (ret) {
			dev_err(&pdev->dev, "failed to request interrupt=%d\n",
							info->irq[i]);
			goto gpio_req_failed;
		}
	}

	/* Enable interrupts */
	wcove_extcon_init_hw_registers(info);

	return 0;

gpio_req_failed:
	return ret;
}

static int wcove_extcon_remove(struct platform_device *pdev)
{
	struct wcove_extcon_info *info = platform_get_drvdata(pdev);

	if (info->pmic_rev > WCOVE_PMIC_REV_AX)
		extcon_unregister_notifier(info->cable.c_edev, EXTCON_USB_HOST,
				&info->cable.nb);
	usb_put_phy(info->otg);
	return 0;
}

static struct platform_device_id wcove_extcon_device_ids[] = {
	{"wcove_extcon", 0},
	{"bxt_wcove_extcon", 0},
	{},
};

static struct platform_driver wcove_extcon_driver = {
	.driver = {
		   .name = "wcove_extcon",
		   .owner = THIS_MODULE,
		   },
	.probe = wcove_extcon_probe,
	.remove = wcove_extcon_remove,
	.id_table = wcove_extcon_device_ids,
};
module_platform_driver(wcove_extcon_driver);
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Intel Whiskey Cove PMIC Extcon driver");
MODULE_LICENSE("GPL v2");
