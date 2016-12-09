/*
 * phy-wcove.c - Intel Whiskey Cove PMIC USBC PHY driver
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
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/usb_typec_phy.h>
#include <linux/pm_qos.h>
#include "usb_typec_detect.h"
#include "pd/system_policy.h"

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

#define WCOVE_CHGRIRQ0_REG			0x4e09
#define CHGRIRQ0_USBC				(1 << 5)

#define WCOVE_CHGRIRQ1_REG			0x4e0a

#define WCOVE_MCHGRIRQ0_REG			0x4e17

#define WCOVE_MCHGRIRQ1_REG			0x4e18
#define MCHGRIRQ1_VBUS_DET			(1 << 0)

#define WCOVE_SPWRSRC_REG			0x4e20
#define SPWRSRC_VBUS_DET			(1 << 0)

#define WCOVE_CHGRCTRL4_REG			0x5e1b
#define CHGRCTRL4_OTGMODE			(1 << 6)

#define WCOVE_USBC_ID_REG			0x7000

#define WCOVE_USBCCTRL1_REG			0x7001
#define USBCCTRL1_DRP_TOGGLE_MASK		(7 << 5)
#define USBCCTRL1_DRP_TOGGLE_RDM		(7 << 5)
#define USBCCTRL1_CURSRC_MASK			(3 << 3)
#define USBCCTRL1_CURSRC_UA0			(0 << 3)
#define USBCCTRL1_CURSRC_UA80			(1 << 3)
#define USBCCTRL1_CURSRC_UA180			(2 << 3)
#define USBCCTRL1_CURSRC_UA330			(3 << 3)
#define USBCCTRL1_MODES_MASK			(7 << 0)
#define USBCCTRL1_MODES_SNK			(0 << 0)
#define USBCCTRL1_MODES_SNKACC			(1 << 0)
#define USBCCTRL1_MODES_SRC			(2 << 0)
#define USBCCTRL1_MODES_SRCACC			(3 << 0)
#define USBCCTRL1_MODES_DRP			(4 << 0)
#define USBCCTRL1_MODES_DRPACC			(5 << 0)

#define WCOVE_USBCCTRL2_REG			0x7002
#define USBCCTRL2_UNSUP_ACC			(1 << 7)
#define USBCCTRL2_TRY_SRC			(1 << 6)
#define USBCCTRL2_ATT_SRC			(1 << 5)
#define USBCCTRL2_ATT_SNK			(1 << 4)
#define USBCCTRL2_ERR_REC			(1 << 3)
#define USBCCTRL2_DIS_ST			(1 << 2)
#define USBCCTRL2_UNATT_SRC			(1 << 1)
#define USBCCTRL2_UNATT_SNK			(1 << 0)

#define WCOVE_USBCCTRL3_REG			0x7003
#define USBCCTRL3_IGNORE_VBUS			(1 << 5)
#define USBCCTRL3_LOOP_BACK			(1 << 4)
#define USBCCTRL3_ATE_TESTMODE			(1 << 3)
#define USBCCTRL3_RESET_PHY			(1 << 2)
#define USBCCTRL3_PD_DIS			(1 << 1)
#define USBCCTRL3_DET_DIS			(1 << 0)

#define WCOVE_CC1CTRL_REG			0x7004
#define WCOVE_CC2CTRL_REG			0x7005
#define CCXCTRL_VBUS_OK				(1 << 7)
#define CCXCTRL_ADC_EN				(1 << 6)
#define CCXCTRL_RDET_EN				(1 << 5)
#define CCXCTRL_CDET_EN				(1 << 4)
#define CCXCTRL_PDWN_EN				(1 << 3)
#define CCXCTRL_TX_EN				(1 << 2)
#define CCXCTRL_VCONN_EN			(1 << 1)
#define CCXCTRL_PU_EN				(1 << 0)

#define WCOVE_CCSEL_REG				0x7006
#define CCSEL_VCONN_SWAP_OFF			(1 << 3)
#define CCSEL_VCONN_SWAP_ON			(1 << 2)
#define CCSEL_CCSEL_MASK			(3 << 0)
#define CCSEL_NONE				(0 << 0)
#define CCSEL_CC1				(1 << 0)
#define CCSEL_CC2				(2 << 0)
#define CCSEL_RSVD				(3 << 0)

#define WCOVE_USBCSTATUS1_REG			0x7007
#define USBCSTATUS1_TCDET_STAT_MASK		(3 << 6)
#define USBCSTATUS1_TCDET_NO_START		(0 << 6)
#define USBCSTATUS1_TCDET_ON_GNG		(1 << 6)
#define USBCSTATUS1_TCDET_COMPLETE		(2 << 6)
#define USBCSTATUS1_TCDET_RSVD			(3 << 6)
#define USBCSTATUS1_CC_ORIENT_MASK		(3 << 4)
#define USBCSTATUS1_CC_ORIENT_POS		4
#define USBCSTATUS1_CC_ORIENT_NONE		(0 << 4)
#define USBCSTATUS1_CC_ORIENT_POS1		(1 << 4)
#define USBCSTATUS1_CC_ORIENT_POS2		(2 << 4)
#define USBCSTATUS1_CC_ORIENT_FLT		(3 << 4)
#define USBCSTATUS1_TCRSLT_MASK			0x0f
#define USBCSTATUS1_TCRSLT_NONE			(0 << 0)
#define USBCSTATUS1_TCRSLT_SRC_DEF		(1 << 0)
#define USBCSTATUS1_TCRSLT_SRC_1P5A		(2 << 0)
#define USBCSTATUS1_TCRSLT_SRC_3A		(3 << 0)
#define USBCSTATUS1_TCRSLT_SNK			(4 << 0)
#define USBCSTATUS1_TCRSLT_DEBUG_ACC		(5 << 0)
#define USBCSTATUS1_TCRSLT_AUDIO_ACC		(6 << 0)
#define USBCSTATUS1_TCRSLT_PWRD_ACC		(7 << 0)
#define USBCSTATUS1_TCRSLT_UNDET		0xf

#define WCOVE_USBCSTATUS2_REG			0x7008
#define USBCSTATUS2_VBUS_REQ			(1 << 5)
#define USBCSTATUS2_PD_ALLOWED			(1 << 4)
#define USBCSTATUS2_ADC_DONE2			(1 << 3)
#define USBCSTATUS2_ADC_DONE1			(1 << 2)
#define USBCSTATUS2_OVER_TEMP			(1 << 1)
#define USBCSTATUS2_SHORT			(1 << 0)

#define WCOVE_USBCSTATUS3_REG			0x7009
#define USBCSTATUS3_TYPEC_ACTIVE		(1 << 7)
#define USBCSTATUS3_TC_STATE_MASK		0x1f
#define USBCSTATUS3_TC_STATE_RESET		0x00
#define USBCSTATUS3_TC_STATE_DISABLED		0x01
#define USBCSTATUS3_TC_STATE_ERR_RECOVERY	0x02
#define USBCSTATUS3_TC_STATE_UNATT_SNK		0x03
#define USBCSTATUS3_TC_STATE_ATT_WAIT_SNK	0x04
#define USBCSTATUS3_TC_STATE_ATT_SNK		0x05
#define USBCSTATUS3_TC_STATE_UNATT_SRC		0x06
#define USBCSTATUS3_TC_STATE_ATT_WAIT_SRC	0x07
#define USBCSTATUS3_TC_STATE_ATT_SRC		0x08
#define USBCSTATUS3_TC_STATE_TRY_SRC		0x09
#define USBCSTATUS3_TC_STATE_TRY_WAIT_SNK	0x0a
#define USBCSTATUS3_TC_STATE_AUDIO_ACC		0x0b
#define USBCSTATUS3_TC_STATE_DEBUG_ACC		0x0c
#define USBCSTATUS3_TC_STATE_UNATT_ACC		0x0d
#define USBCSTATUS3_TC_STATE_ATT_WAIT_ACC	0x0e
#define USBCSTATUS3_TC_STATE_POWERED_ACC	0x0f
#define USBCSTATUS3_TC_STATE_UNSUPPORTED_ACC	0x10

#define WCOVE_CC1CMP_REG			0x700A
#define WCOVE_CC2CMP_REG			0x700B
#define CCXCMP_DET_3A				(1 << 4)
#define CCXCMP_DET_1P5A				(1 << 3)
#define CCXCMP_DET_DEF				(1 << 2)
#define CCXCMP_RD				(1 << 1)
#define CCXCMP_RA				(1 << 0)

#define WCOVE_CC1STAT_REG			0x700C
#define WCOVE_CC2STAT_REG			0x700D
#define CCXSTAT_SRC_RX_MASK			0x60
#define CCXSTAT_SRC_RX_NONE			(0 << 5)
#define CCXSTAT_SRC_RX_RD			(1 << 5)
#define CCXSTAT_SRC_RX_RA			(2 << 5)
#define CCXSTAT_SRC_RX_RSVD			(3 << 5)
#define CCXSTAT_SRC_RP				(1 << 4)
#define CCXSTAT_PWR3A_SNK			(1 << 3)
#define CCXSTAT_PWR1P5A_SNK			(1 << 2)
#define CCXSTAT_PWRDEF_SNK			(1 << 1)
#define CCXSTAT_SNK_RP				(1 << 0)

#define WCOVE_AFE_ADC1_REG			0x700E
#define AFE_ADC1_START1				(1 << 6)
#define AFE_ADC1_RSLT_MASK			0x3f
#define AFE_ADC1_RSLT_LSB			31250	/* uV */

#define WCOVE_AFE_PWRSTATE_REG			0x7013
#define AFE_PWRSTATE_OVERRIDE_PSMODE		(1 << 1)
#define AFE_PWRSTATE_AFE_EN			(1 << 0)

#define WCOVE_USBCIRQ1_REG			0x7015
#define USBCIRQ1_ADC_DONE2			(1 << 3)
#define USBCIRQ1_ADC_DONE1			(1 << 2)
#define USBCIRQ1_OVER_TEMP			(1 << 1)
#define USBCIRQ1_SHORT				(1 << 0)

#define WCOVE_USBCIRQ2_REG			0x7016
#define USBCIRQ2_CC_CHANGE			(1 << 7)
#define USBCIRQ2_RX_PD				(1 << 6)
#define USBCIRQ2_RX_HR				(1 << 5)
#define USBCIRQ2_RX_CR				(1 << 4)
#define USBCIRQ2_TX_SUCCESS			(1 << 3)
#define USBCIRQ2_TX_FAIL			(1 << 2)
#define USBCIRQ2_PDIRQ				0x7c

#define WCOVE_MUSBCIRQ1_REG			0x7017
#define WCOVE_MUSBCIRQ2_REG			0x7018

#define WCOVE_PDCFG1_REG			0x7019
#define PDCFG1_ID_FILL				(1 << 7)
#define PDCFG1_ID_WA				0x1f

#define WCOVE_PDCFG2_REG			0x701A
#define PDCFG2_RX_SOP				(1 << 0)
#define PDCFG2_RX_SOP_P				(1 << 1)
#define PDCFG2_RX_SOP_PP			(1 << 2)
#define PDCFG2_RX_SOP_DP			(1 << 3)
#define PDCFG2_RX_SOP_DPP			(1 << 4)

#define WCOVE_PDCFG3_REG			0x701B
#define PDCFG3_SR_SOP2_MASK			(3 << 6)
#define PDCFG3_SR_SOP2_REV1			(0 << 6)
#define PDCFG3_SR_SOP2_REV2			(1 << 6)
#define PDCFG3_SR_SOP1_MASK			(3 << 4)
#define PDCFG3_SR_SOP1_REV1			(0 << 4)
#define PDCFG3_SR_SOP1_REV2			(1 << 4)
#define PDCFG3_SR_SOP0_MASK			(3 << 2)
#define PDCFG3_SR_SOP0_REV1			(0 << 2)
#define PDCFG3_SR_SOP0_REV2			(1 << 2)
#define PDCFG3_DATA_ROLE_DFP			(1 << 1)
#define PDCFG3_PWR_ROLE_SRC			(1 << 0)

#define WCOVE_PDSTATUS_REG			0x701C
#define PDSTATUS_RX_RSLT_MASK			(7 << 3)
#define PDSTATUS_RX_RSLT_NONE			(0 << 3)
#define PDSTATUS_RX_RSLT_MSG			(1 << 3)
#define PDSTATUS_RX_RSLT_HR			(2 << 3)
#define PDSTATUS_RX_RSLT_CR			(3 << 3)
#define PDSTATUS_TX_RSLT_MASK			(7 << 0)
#define PDSTATUS_TX_RSLT_NONE			(0 << 0)
#define PDSTATUS_TX_RSLT_SUCCESS		(1 << 0)
#define PDSTATUS_TX_RSLT_ERROR			(2 << 0)
#define PDSTATUS_TX_RSLT_BUSY			(3 << 0)

#define WCOVE_RXSTATUS_REG			0x701D
#define RXSTATUS_RXDATA				(1 << 7)
#define RXSTATUS_RXOVERRUN			(1 << 6)
#define RXSTATUS_RXCLEAR			(1 << 0)

#define WCOVE_RXINFO_REG			0x701E
#define RXINFO_RXBYTES_MASK			0xf8
#define RXINFO_RXBYTES_POS			3
#define RXINFO_RXSOP_MASK			(7 << 0)
#define RXINFO_RXSOP_NONE			0
#define RXINFO_RXSOP_SOP0			1
#define RXINFO_RXSOP_SOP1			2
#define RXINFO_RXSOP_SOP2			3
#define RXINFO_RXSOP_SOP1_DBG			4
#define RXINFO_RXSOP_SOP2_DBG			5

#define WCOVE_TXCMD_REG				0x701F
#define TXCMD_TX_CMD_MASK			(7 << 5)
#define TXCMD_TX_CMD_POS			5
#define TXCMD_TX_CMD_NOP			(0 << 5)
#define TXCMD_TX_CMD_MSG			(1 << 5)
#define TXCMD_TX_CMD_CR				(2 << 5)
#define TXCMD_TX_CMD_HR				(3 << 5)
#define TXCMD_TX_CMD_BIST			(4 << 5)
#define TXCMD_TX_START				(1 << 1)
#define TXCMD_TXBUF_RDY				(1 << 0)

#define WCOVE_TXINFO_REG			0x7020
#define TXINFO_TX_RETRIES_MASK			(7 << 3)
#define TXINFO_TX_RETRIES_0			(0 << 3)
#define TXINFO_TX_RETRIES_1			(1 << 3)
#define TXINFO_TX_RETRIES_2			(2 << 3)
#define TXINFO_TX_RETRIES_3			(3 << 3)
#define TXINFO_TX_RETRIES_4			(4 << 3)
#define TXINFO_TX_RETRIES_5			(5 << 3)
#define TXINFO_TX_RETRIES_6			(6 << 3)
#define TXINFO_TX_RETRIES_7			(7 << 3)
#define TXINFO_TX_SOP_MASK			(7 << 0)
#define TXINFO_TX_SOP_NONE			0
#define TXINFO_TX_SOP0				1
#define TXINFO_TX_SOP1				2
#define TXINFO_TX_SOP2				3
#define TXINFO_TX_SOP1_DBG			4
#define TXINFO_TX_SOP2_DBG			5

#define WCOVE_RX_DATA_BASE_REG			0x7028
#define WCOVE_RX_DATA_LEN			30

#define WCOVE_TX_DATA_BASE_REG			0x7047
#define WCOVE_TX_DATA_LEN			30


#define PD_PKT_MAX_LEN				30
#define SOP1					0x12
#define SOP2					0x13
#define SOP3					0x1b
#define RESET1					0x15
#define RESET2					0x16
#define PACKSYM					0x80
#define JAMCRC					0xff
#define EOP					0x14
#define TXON					0xa1
#define TXOFF					0xfe

#define USB_TYPEC_PD_VERSION			2

enum wcove_usbc_fsm_state {
	WCOVE_USBC_FSM_STATE_RESET,
	WCOVE_USBC_FSM_STATE_DISABLED,
	WCOVE_USBC_FSM_STATE_ERR_RECOVERY,
	WCOVE_USBC_FSM_STATE_UNATT_SNK,
	WCOVE_USBC_FSM_STATE_ATT_WAIT_SNK,
	WCOVE_USBC_FSM_STATE_ATT_SNK,
	WCOVE_USBC_FSM_STATE_UNATT_SRC,
	WCOVE_USBC_FSM_STATE_ATT_WAIT_SRC,
	WCOVE_USBC_FSM_STATE_ATT_SRC,
	WCOVE_USBC_FSM_STATE_TRY_SRC,
	WCOVE_USBC_FSM_STATE_TRY_WAIT_SNK,
	WCOVE_USBC_FSM_STATE_AUDIO_ACC,
	WCOVE_USBC_FSM_STATE_DEBUG_ACC,
	WCOVE_USBC_FSM_STATE_UNATT_ACC,
	WCOVE_USBC_FSM_STATE_ATT_WAIT_ACC,
	WCOVE_USBC_FSM_STATE_POWERED_ACC,
	WCOVE_USBC_FSM_STATE_UNSUPPORTED_ACC,
};

enum wcove_usbc_cable_type {
	WCOVE_USBC_CABLE_NONE,
	WCOVE_USBC_CABLE_SRC_DEF,
	WCOVE_USBC_CABLE_SRC_1P5A,
	WCOVE_USBC_CABLE_SRC_3A,
	WCOVE_USBC_CABLE_SNK,
	WCOVE_USBC_CABLE_DBG_ACC,
	WCOVE_USBC_CABLE_AUDIO_ACC,
	WCOVE_USBC_CABLE_PWRD_ACC,
	WCOVE_USBC_CABLE_UNKNOWN,
};

enum wcove_usbc_cc_orient {
	WCOVE_USBC_CC_NONE,
	WCOVE_USBC_CC_POS1,
	WCOVE_USBC_CC_POS2,
	WCOVE_USBC_CC_FAULT,
};

enum wcove_usbc_data_role {
	WCOVE_USBC_DATA_NONE,
	WCOVE_USBC_DATA_HOST,
	WCOVE_USBC_DATA_DEVICE,
};

enum wcove_usbc_power_role {
	WCOVE_USBC_POWER_NONE,
	WCOVE_USBC_POWER_SRC,
	WCOVE_USBC_POWER_SNK,
};

struct wcove_usbc_cable_info {
	enum wcove_usbc_cable_type type;
	enum wcove_usbc_fsm_state state;
	enum wcove_usbc_cc_orient orient;
	enum wcove_usbc_data_role data_role;
	enum wcove_usbc_power_role power_role;
};

enum wcove_pmic_typec_rev {
	WCOVE_PMIC_TYPEC_NONE,
	WCOVE_PMIC_TYPEC_B0,
	WCOVE_PMIC_TYPEC_B1,
	WCOVE_PMIC_TYPEC_B2,
};

struct wcove_usbc_info {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq_chip;
	struct dentry *debug_file;
	enum wcove_pmic_typec_rev pmic_rev;
	struct typec_phy phy;
	struct mutex lock;
	int irq;
	struct wcove_usbc_cable_info cable;

	bool tx_ongoing;

	/* WA for B1 PMIC DRP issue */
	int vbus_status;
	int irq_vbus;
	struct delayed_work poll_work;
	int interval;

	struct delayed_work init_work;
	struct delayed_work vbus_work;

	/* PD hard reset command support */
	struct delayed_work hr_work;
	bool hr_tx_ong;
	bool pr_swap;
	bool auto_retry;
	bool en_sop_prime;
	struct pm_qos_request pm_qos_request;
};

static int wcove_usbc_flush_fifo(struct typec_phy *phy,
				enum typec_fifo fifo_type);

static void wcove_usbc_enable_ignore_vbus(struct wcove_usbc_info *info)
{

	/* Ignore VBUS drop */
	regmap_update_bits(info->regmap,
		WCOVE_USBCCTRL3_REG, USBCCTRL3_IGNORE_VBUS, USBCCTRL3_IGNORE_VBUS);

	/* Disable BC1.2 detection */
	regmap_update_bits(info->regmap, WCOVE_CHGRCTRL4_REG,
				CHGRCTRL4_OTGMODE, CHGRCTRL4_OTGMODE);
}

static void wcove_usbc_clear_ignore_vbus(struct wcove_usbc_info *info)
{
	regmap_update_bits(info->regmap, WCOVE_USBCCTRL3_REG,
						USBCCTRL3_IGNORE_VBUS, 0);
	regmap_update_bits(info->regmap, WCOVE_CHGRCTRL4_REG,
						CHGRCTRL4_OTGMODE, 0);
}

static int wcove_usbc_set_host_current(struct typec_phy *phy,
						enum typec_current cur)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, val;

	dev_dbg(info->dev, "In %s\n", __func__);

	switch (cur) {
	case TYPEC_CURRENT_UNKNOWN:
		val = USBCCTRL1_CURSRC_UA0;
		break;
	case TYPEC_CURRENT_USB:
		val = USBCCTRL1_CURSRC_UA80;
		break;
	case TYPEC_CURRENT_1500:
		val = USBCCTRL1_CURSRC_UA180;
		break;
	case TYPEC_CURRENT_3000:
		val = USBCCTRL1_CURSRC_UA330;
		break;
	default:
		dev_warn(info->dev, "Invalid current(%d)\n", cur);
		return -EINVAL;
	}

	ret = regmap_update_bits(info->regmap,
			WCOVE_USBCCTRL1_REG, USBCCTRL1_CURSRC_MASK, val);

	return ret;
}

static enum typec_current wcove_usbc_get_host_current(struct typec_phy *phy)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	enum typec_current cur;

	dev_dbg(info->dev, "In %s\n", __func__);

	if (info->phy.cc1.valid)
		cur = info->phy.cc1.cur;
	else if (info->phy.cc2.valid)
		cur = info->phy.cc2.cur;
	else
		cur = TYPEC_CURRENT_UNKNOWN;

	return cur;
}

static int wcove_usbc_measure_cc(struct typec_phy *phy, struct cc_pin *pin)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, ccx_cmp;

	dev_dbg(info->dev, "In %s\n", __func__);

	if (pin->id == TYPEC_PIN_CC1)
		ret = regmap_read(info->regmap, WCOVE_CC1CMP_REG, &ccx_cmp);
	else
		ret = regmap_read(info->regmap, WCOVE_CC2CMP_REG, &ccx_cmp);
	if (ret < 0) {
		dev_err(info->dev, "failed to read usbc_cntl1(%d)\n", ret);
		return ret;
	}


	pin->valid = true;

	if (ccx_cmp & CCXCMP_RA) {
		pin->rd = USB_TYPEC_CC_VRA;
		pin->cur = TYPEC_CURRENT_UNKNOWN;
	} else if (ccx_cmp & CCXCMP_DET_3A) {
		pin->cur = TYPEC_CURRENT_3000;
		pin->rd = USB_TYPEC_CC_VRD_3000;
	} else if (ccx_cmp & CCXCMP_DET_1P5A) {
		pin->cur = TYPEC_CURRENT_1500;
		pin->rd = USB_TYPEC_CC_VRD_1500;
	} else if ((ccx_cmp & CCXCMP_DET_DEF) ||
				(ccx_cmp & CCXCMP_RD)) {
		pin->cur = TYPEC_CURRENT_USB;
		pin->rd = USB_TYPEC_CC_VRD_USB;
	} else {
		pin->rd = USB_TYPEC_CC_VRD_UNKNOWN;
		pin->cur = TYPEC_CURRENT_UNKNOWN;
		pin->valid = false;
	}

	return 0;
}

static int wcove_usbc_switch_mode(struct typec_phy *phy, enum typec_mode mode)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, val = USBCCTRL1_MODES_DRP;

	dev_dbg(info->dev, "%s: mode(%d)\n", __func__, mode);

	if (mode == TYPEC_MODE_UFP)
		val = USBCCTRL1_MODES_SNK;
	else if (mode == TYPEC_MODE_DFP)
		val = USBCCTRL1_MODES_SRC;
	else if (mode == TYPEC_MODE_DRP)
		val = USBCCTRL1_MODES_DRP;
	else
		dev_dbg(info->dev, "invalid switch mode\n");

	ret = regmap_update_bits(info->regmap,
			WCOVE_USBCCTRL1_REG, USBCCTRL1_MODES_MASK, val);

	return ret;
}

static int wcove_usbc_pd_version(struct typec_phy *phy)
{

	if (phy->type == USB_TYPE_C)
		return USB_TYPEC_PD_VERSION;
	else
		return 0;
}

static bool wcove_usbc_pd_support(struct typec_phy *phy)
{
	return true;
}

static bool wcove_usbc_is_vconn_enabled(struct typec_phy *phy)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, ccx_ctrl_reg, ccx_ctrl_val = 0x0, ccsel_val = 0x0;

	dev_dbg(info->dev, "In %s\n", __func__);
	if (info->cable.orient == WCOVE_USBC_CC_POS1)
		ccx_ctrl_reg = WCOVE_CC2CTRL_REG;
	else if (info->cable.orient == WCOVE_USBC_CC_POS2)
		ccx_ctrl_reg = WCOVE_CC1CTRL_REG;
	else
		return false;

	ret = regmap_read(info->regmap, ccx_ctrl_reg, &ccx_ctrl_val);
	if (ret < 0)
		dev_err(info->dev, "ccx_ctrl access error(%d)\n", ret);

	ret = regmap_read(info->regmap, WCOVE_CCSEL_REG, &ccsel_val);
	if (ret < 0)
		dev_err(info->dev, "ccsel access error(%d)\n", ret);

	return ((ccx_ctrl_val & CCXCTRL_VCONN_EN) &&
				(ccsel_val & CCSEL_VCONN_SWAP_ON));
}

static int wcove_usbc_enable_vconn(struct typec_phy *phy, bool en)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, ccx_ctrl_reg, ccx_ctrl_val, ccsel_val;

	dev_dbg(info->dev, "%s: en(%d), CC(%d)\n",
		__func__, en, info->cable.orient);
	if (info->cable.orient == WCOVE_USBC_CC_POS1)
		ccx_ctrl_reg = WCOVE_CC2CTRL_REG;
	else if (info->cable.orient == WCOVE_USBC_CC_POS2)
		ccx_ctrl_reg = WCOVE_CC1CTRL_REG;
	else
		return -EINVAL;

	if (en) {
		ccx_ctrl_val = CCXCTRL_VCONN_EN;
		ccsel_val = CCSEL_VCONN_SWAP_ON;
	} else {
		ccx_ctrl_val = 0;
		ccsel_val = CCSEL_VCONN_SWAP_OFF;
	}

	ret = regmap_update_bits(info->regmap, ccx_ctrl_reg,
				CCXCTRL_VCONN_EN, ccx_ctrl_val);
	if (ret < 0) {
		dev_err(info->dev, "ccx_ctrl access error(%d)\n", ret);
		return ret;
	}

	ret = regmap_update_bits(info->regmap, WCOVE_CCSEL_REG,
			CCSEL_VCONN_SWAP_OFF | CCSEL_VCONN_SWAP_ON, ccsel_val);
	if (ret < 0)
		dev_err(info->dev, "ccsel access error(%d)\n", ret);

	return ret;
}

static int wcove_usbc_set_bist_cm2(struct typec_phy *phy, bool en)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret;

	dev_dbg(info->dev, "%s: en (%d)\n", __func__, en);
	if (en)
		ret = regmap_write(info->regmap, WCOVE_TXCMD_REG,
					TXCMD_TX_CMD_BIST | TXCMD_TX_START);
	else
		ret = regmap_write(info->regmap, WCOVE_TXCMD_REG,
							TXCMD_TX_CMD_NOP);

	return ret;
}

static bool wcove_usbc_vbus_state(struct typec_phy *phy)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret = 0, ccx_ctrl = 0;

	dev_dbg(info->dev, "In %s\n", __func__);

	if (info->cable.orient == WCOVE_USBC_CC_POS1)
		ret = regmap_read(info->regmap, WCOVE_CC1CTRL_REG, &ccx_ctrl);
	else if (info->cable.orient == WCOVE_USBC_CC_POS2)
		ret = regmap_read(info->regmap, WCOVE_CC2CTRL_REG, &ccx_ctrl);
	if (ret < 0)
		dev_err(info->dev, "ccx_ctrl read error(%d)\n", ret);

	return (ccx_ctrl & CCXCTRL_VBUS_OK);
}

static int wcove_usbc_setup_cc(struct typec_phy *phy, enum typec_cc_pin cc,
					enum typec_state state)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);

	dev_dbg(info->dev, "In %s\n", __func__);
	return 0;
}

static int wcove_usbc_enable_auto_retry(struct typec_phy *phy, bool en)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);

	dev_dbg(info->dev, "In %s: enable(%d)\n", __func__, en);
	if (en)
		regmap_update_bits(info->regmap, WCOVE_TXINFO_REG,
				TXINFO_TX_RETRIES_MASK, TXINFO_TX_RETRIES_3);
	else
		regmap_update_bits(info->regmap, WCOVE_TXINFO_REG,
				TXINFO_TX_RETRIES_MASK, TXINFO_TX_RETRIES_0);

	info->auto_retry = en;
	return 0;
}

static int wcove_usbc_enable_sop_prime(struct typec_phy *phy, bool en)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);

	dev_dbg(info->dev, "In %s\n", __func__);

	if (!info->en_sop_prime)
		return 0;

	if (en)
		regmap_update_bits(info->regmap,
			WCOVE_PDCFG2_REG, PDCFG2_RX_SOP_P, PDCFG2_RX_SOP_P);
	else
		regmap_update_bits(info->regmap,
			WCOVE_PDCFG2_REG, PDCFG2_RX_SOP_P, 0);

	return 0;
}

static int wcove_usbc_setup_role(struct typec_phy *phy, int data_role, int pwr_role)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, val = 0;

	dev_dbg(info->dev, "In %s\n", __func__);

	if (data_role == PD_DATA_ROLE_DFP)
		val |= PDCFG3_DATA_ROLE_DFP;
	else
		val &= ~PDCFG3_DATA_ROLE_DFP;

	if (pwr_role == PD_POWER_ROLE_PROVIDER ||
		pwr_role == PD_POWER_ROLE_CONSUMER_PROVIDER)
		val |= PDCFG3_PWR_ROLE_SRC;
	else
		val &= ~PDCFG3_PWR_ROLE_SRC;

	/* set PD revision info */
	val |= PDCFG3_SR_SOP0_REV2 | PDCFG3_SR_SOP1_REV2;

	ret = regmap_update_bits(info->regmap, WCOVE_PDCFG3_REG,
			PDCFG3_SR_SOP0_MASK | PDCFG3_SR_SOP1_MASK |
			PDCFG3_DATA_ROLE_DFP | PDCFG3_PWR_ROLE_SRC, val);
	return ret;
}

static int wcove_usbc_enable_autocrc(struct typec_phy *phy, bool en)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret;

	dev_dbg(info->dev, "%s: autocrc(%d)\n", __func__, en);

	if (en) {
		wcove_usbc_flush_fifo(phy, FIFO_TYPE_TX | FIFO_TYPE_RX);
		ret = regmap_update_bits(info->regmap,
			WCOVE_USBCCTRL3_REG, USBCCTRL3_PD_DIS, 0);

		/* allow SOP messages for PD */
		regmap_update_bits(info->regmap, WCOVE_PDCFG2_REG,
					PDCFG2_RX_SOP, PDCFG2_RX_SOP);
		regmap_update_bits(info->regmap,
				(info->cable.orient == WCOVE_USBC_CC_POS1) ?
				WCOVE_CC1CTRL_REG : WCOVE_CC2CTRL_REG,
				CCXCTRL_TX_EN, CCXCTRL_TX_EN);
	} else {
		ret = regmap_update_bits(info->regmap,
			WCOVE_USBCCTRL3_REG, USBCCTRL3_PD_DIS, USBCCTRL3_PD_DIS);

		/* disable SOP messages for PD */
		regmap_write(info->regmap, WCOVE_PDCFG2_REG, 0x00);
		wcove_usbc_flush_fifo(phy, FIFO_TYPE_TX | FIFO_TYPE_RX);
		regmap_update_bits(info->regmap,
				WCOVE_CC1CTRL_REG,
				CCXCTRL_TX_EN, 0);
		regmap_update_bits(info->regmap,
				WCOVE_CC2CTRL_REG,
				CCXCTRL_TX_EN, 0);
	}

	return ret;
}

static int wcove_usbc_set_swap_state(struct typec_phy *phy, bool swap)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);

	dev_dbg(info->dev, "%s\n", __func__);

	if (swap) {
		wcove_usbc_enable_ignore_vbus(info);
		info->pr_swap = true;
	} else {
		wcove_usbc_clear_ignore_vbus(info);
		info->pr_swap = false;
	}

	return 0;
}

static int wcove_usbc_set_pu_pd(struct typec_phy *phy, enum typec_cc_pull pull)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, val;

	dev_dbg(info->dev, "%s\n", __func__);

	if (pull == TYPEC_CC_PULL_UP) {
		/* move the FSM to attached source */
		val = USBCCTRL2_ATT_SRC;
	} else if(pull == TYPEC_CC_PULL_DOWN) {
		/* move the FSM to attached sink */
		val = USBCCTRL2_ATT_SNK;
	} else {
		val = 0;
	}

	ret = regmap_write(info->regmap,
			WCOVE_USBCCTRL2_REG, val);
	return ret;
}

static int wcove_usbc_phy_reset(struct typec_phy *phy)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, tx_cmd;

	dev_dbg(info->dev, "entry %s\n", __func__);

	ret = regmap_read(info->regmap, WCOVE_TXCMD_REG, &tx_cmd);
	if (ret < 0) {
		dev_err(info->dev, "failed to read tx_cmd(%d)\n", ret);
		return ret;
	}

	if (!(tx_cmd & TXCMD_TXBUF_RDY)) {
		dev_err(info->dev, "already tx in progress\n");
		return -EBUSY;
	}

	/* Igone VBUS chnages */
	wcove_usbc_enable_ignore_vbus(info);

	ret = regmap_write(info->regmap, WCOVE_TXCMD_REG,
				TXCMD_TX_CMD_HR | TXCMD_TX_START);
	if (ret < 0) {
		dev_err(info->dev, "TX HR error(%d)\n", ret);
		wcove_usbc_clear_ignore_vbus(info);
	} else {
		dev_dbg(info->dev, "Set HR_TX flag\n");
		info->hr_tx_ong = true;
		schedule_delayed_work(&info->hr_work, HZ);
	}

	return ret;
}

/* This function may not be needed */
static int wcove_usbc_flush_fifo(struct typec_phy *phy, enum typec_fifo fifo_type)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, i, rx_status;

	/* TX FLUSH */
	if (fifo_type & FIFO_TYPE_TX) {
		ret = regmap_write(info->regmap, WCOVE_TXCMD_REG,
							TXCMD_TX_CMD_NOP);
		if (ret < 0) {
			dev_err(info->dev, "failed flush TX_FIFO(%d)\n", ret);
			return ret;
		}
	}

	if (!(fifo_type & FIFO_TYPE_RX))
		return 0;
	/* Make sure clear all the three PMIC RX buffers */
	for (i = 0; i < 3; i++) {
		ret = regmap_read(info->regmap, WCOVE_RXSTATUS_REG, &rx_status);
		if (ret < 0) {
			dev_err(info->dev, "failed to read rx_status(%d)\n", ret);
			return ret;
		}
		if ((rx_status & RXSTATUS_RXDATA) ||
			(rx_status & RXSTATUS_RXOVERRUN)) {
			dev_dbg(info->dev, "%s: clear %i packet\n",
				__func__, i);
			ret = regmap_write(info->regmap,
				WCOVE_RXSTATUS_REG, RXSTATUS_RXCLEAR);
		} else {
			break;
		}
	}

	return 0;
}

static int wcove_usbc_send_pkt(struct typec_phy *phy, u8 *pkt,
					int len, enum pd_pkt_type type)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, i, tx_cmd, sop;

	if ((!info->en_sop_prime) && type != PKT_TYPE_SOP) {
		dev_dbg(info->dev, "%s:Skip non-sop packets!!", __func__);
		return -EINVAL;
	}

	if (!pkt || (len > PD_PKT_MAX_LEN)) {
		dev_err(info->dev, "invalid tx packet\n");
		return -EINVAL;
	}

	ret = regmap_read(info->regmap, WCOVE_TXCMD_REG, &tx_cmd);
	if (ret < 0) {
		dev_err(info->dev, "failed to read tx_cmd(%d)\n", ret);
		return ret;
	}

	if (!(tx_cmd & TXCMD_TXBUF_RDY)) {
		dev_err(info->dev, "already tx in progress\n");
		return -EBUSY;
	}

	dev_dbg(info->dev, "%s: TX pkt len(%d)\n", __func__, len);

	for (i = 0; i < len; i++) {
		ret = regmap_write(info->regmap, WCOVE_TX_DATA_BASE_REG + i, pkt[i]);
		if (ret < 0) {
			dev_err(info->dev, "tx buffer fill error(%d)\n", ret);
			return ret;
		}
	}

	switch (type) {
	case PKT_TYPE_SOP:
		sop = TXINFO_TX_SOP0;
		break;
	case PKT_TYPE_SOP_P:
		sop = TXINFO_TX_SOP1;
		if (!info->en_sop_prime)
			return 0;
		break;
	case PKT_TYPE_SOP_PP:
		sop = TXINFO_TX_SOP2;
		break;
	default:
		ret = -EINVAL;
		return ret;
	}

	ret = regmap_update_bits(info->regmap, WCOVE_TXINFO_REG,
						TXINFO_TX_SOP_MASK, sop);
	if (ret < 0) {
		dev_err(info->dev, "TX packet SOP error(%d)\n", ret);
		return ret;
	}

	/* send the packet */
	ret = regmap_write(info->regmap, WCOVE_TXCMD_REG,
				TXCMD_TX_CMD_MSG | TXCMD_TX_START);
	if (ret < 0) {
		dev_err(info->dev, "TX packet send error(%d)\n", ret);
		return ret;
	}

	info->tx_ongoing = true;
	if (info->auto_retry) {
		usleep_range(1000, 1500);
		phy->notify_protocol(phy, PROT_PHY_EVENT_TX_SENT);
	}

	return ret;
}

static int wcove_usbc_receive_pkt(struct typec_phy *phy,
				u8 *buf, enum pd_pkt_type *type)
{
	struct wcove_usbc_info *info = dev_get_drvdata(phy->dev);
	int ret, i, pd_status, rx_status, rx_info, len, tmp;

	if (!buf) {
		dev_err(info->dev, "invalid rx buffer\n");
		return -EINVAL;
	}

	ret = regmap_read(info->regmap, WCOVE_PDSTATUS_REG, &pd_status);
	if (ret < 0) {
		dev_err(info->dev, "failed to read pd_status\n");
		return ret;
	}

	ret = regmap_read(info->regmap, WCOVE_RXSTATUS_REG, &rx_status);
	if (ret < 0) {
		dev_err(info->dev, "failed to read rx_status(%d)\n", ret);
		return ret;
	}

	if (rx_status & RXSTATUS_RXOVERRUN)
		dev_warn(info->dev, "RX buffer overrun!!\n");

	ret = regmap_read(info->regmap, WCOVE_RXINFO_REG, &rx_info);
	if (ret < 0) {
		dev_err(info->dev, "failed to read rx_info(%d)\n", ret);
		return ret;
	}

	/* get rx message length */
	len = (rx_info & RXINFO_RXBYTES_MASK) >> RXINFO_RXBYTES_POS;

	dev_dbg(info->dev, "%s: RX msg len(%d)\n", __func__, len);

	/* read rx packet */
	for (i = 0; i < len; i++) {
		ret = regmap_read(info->regmap,
				WCOVE_RX_DATA_BASE_REG + i, &tmp);
		if (ret < 0) {
			dev_err(info->dev,
				"failed to read rx buffer(%d)\n", ret);
			return ret;
		}
		buf[i] = (u8)tmp;
	}

	/* update PD message type */
	*type = rx_info & RXINFO_RXSOP_MASK;

	/* clear the rx buffer */
	ret = regmap_update_bits(info->regmap, WCOVE_RXSTATUS_REG,
				RXSTATUS_RXCLEAR, RXSTATUS_RXCLEAR);
	if (ret < 0) {
		dev_err(info->dev, "failed to clear rx buffer(%d)\n", ret);
		return ret;
	}

	ret = regmap_read(info->regmap, WCOVE_PDSTATUS_REG, &pd_status);
	if (ret < 0) {
		dev_err(info->dev, "failed to read pd_status\n");
		return ret;
	}

	ret = regmap_read(info->regmap, WCOVE_RXSTATUS_REG, &rx_status);
	if (ret < 0) {
		dev_err(info->dev, "failed to read rx_status(%d)\n", ret);
		return ret;
	}

	return len;
}

static void wcove_usbc_notify_events(struct wcove_usbc_info *info)
{

	switch (info->cable.type) {
	case WCOVE_USBC_CABLE_NONE:
	case WCOVE_USBC_CABLE_UNKNOWN:
		info->phy.valid_cc = 0;
		atomic_notifier_call_chain(&info->phy.notifier,
					 TYPEC_EVENT_NONE, &info->phy);
		break;
	case WCOVE_USBC_CABLE_SRC_DEF:
	case WCOVE_USBC_CABLE_SRC_1P5A:
	case WCOVE_USBC_CABLE_SRC_3A:
	case WCOVE_USBC_CABLE_PWRD_ACC:
		atomic_notifier_call_chain(&info->phy.notifier,
					 TYPEC_EVENT_UFP, &info->phy);
		break;
	case WCOVE_USBC_CABLE_SNK:
		atomic_notifier_call_chain(&info->phy.notifier,
					 TYPEC_EVENT_DFP, &info->phy);
		break;
	case WCOVE_USBC_CABLE_DBG_ACC:
	case WCOVE_USBC_CABLE_AUDIO_ACC:
	default:
		dev_warn(info->dev, "Cable not supported\n");
	}
}

static int wcove_usbc_drole_init(struct wcove_usbc_info *info)
{
	enum wcove_usbc_data_role drole;

	switch (info->cable.type) {
	case WCOVE_USBC_CABLE_SRC_DEF:
	case WCOVE_USBC_CABLE_SRC_1P5A:
	case WCOVE_USBC_CABLE_SRC_3A:
		drole = WCOVE_USBC_DATA_DEVICE;
		break;
	case WCOVE_USBC_CABLE_SNK:
	case WCOVE_USBC_CABLE_DBG_ACC:
	case WCOVE_USBC_CABLE_AUDIO_ACC:
	case WCOVE_USBC_CABLE_PWRD_ACC:
		drole = WCOVE_USBC_DATA_HOST;
		break;
	default:
		drole = WCOVE_USBC_DATA_NONE;
	}
	return drole;
}

static int wcove_usbc_prole_init(struct wcove_usbc_info *info)
{
	enum wcove_usbc_power_role prole;

	switch (info->cable.type) {
	case WCOVE_USBC_CABLE_SRC_DEF:
	case WCOVE_USBC_CABLE_SRC_1P5A:
	case WCOVE_USBC_CABLE_SRC_3A:
	case WCOVE_USBC_CABLE_PWRD_ACC:
		prole = WCOVE_USBC_POWER_SNK;
		break;
	case WCOVE_USBC_CABLE_SNK:
	case WCOVE_USBC_CABLE_DBG_ACC:
	case WCOVE_USBC_CABLE_AUDIO_ACC:
		prole = WCOVE_USBC_POWER_SRC;
		break;
	default:
		prole = WCOVE_USBC_POWER_NONE;
	}
	return prole;
}

static void wcove_usbc_update_ccsel(struct wcove_usbc_info *info)
{
	int ret, val, cc1_stat, cc2_stat;

	ret = regmap_read(info->regmap, WCOVE_CC1STAT_REG, &cc1_stat);
	if (ret < 0) {
		dev_err(info->dev, "failed to read cc1_stat\n");
		return;
	}

	ret = regmap_read(info->regmap, WCOVE_CC2STAT_REG, &cc2_stat);
	if (ret < 0) {
		dev_err(info->dev, "failed to read cc1_stat\n");
		return;
	}

	if ((cc1_stat & CCXSTAT_SRC_RP) ||
		((cc1_stat & CCXSTAT_SRC_RX_MASK) == CCXSTAT_SRC_RX_RD))
		val = CCSEL_CC1;
	else if ((cc2_stat & CCXSTAT_SRC_RP) ||
		((cc2_stat & CCXSTAT_SRC_RX_MASK) == CCXSTAT_SRC_RX_RD))
		val = CCSEL_CC2;
	else
		return;

	ret = regmap_update_bits(info->regmap, WCOVE_CCSEL_REG,
				CCSEL_CCSEL_MASK, val);
	if (ret < 0)
		dev_err(info->dev, "failed to write ccsel reg(%d)\n", ret);
}

static void wcove_usbc_handle_cc_event(struct wcove_usbc_info *info)
{
	int ret, usbc_stat1, usbc_stat2, usbc_stat3, ret2 = 0, ret3 = 0;

	ret = regmap_read(info->regmap, WCOVE_USBCSTATUS1_REG, &usbc_stat1);
	if (ret < 0) {
		dev_err(info->dev, "failed to read usbc_stat1\n");
		goto dev_det_ret;
	}

	dev_dbg(info->dev, "usbc_stat1(%x) usbc_stat2(%x) usbc_stat3(%x)\n",
			usbc_stat1,
			regmap_read(info->regmap, WCOVE_USBCSTATUS2_REG,
				&usbc_stat2) < 0 ? ret2 = -1 : usbc_stat2,
			regmap_read(info->regmap, WCOVE_USBCSTATUS3_REG,
				&usbc_stat3) < 0 ? ret3 = -1 : usbc_stat3);
	if (ret2 < 0) {
		ret = ret2;
		dev_err(info->dev, "failed to read usbc_stat2\n");
		goto dev_det_ret;
	}
	if (ret3 < 0) {
		ret = ret3;
		dev_err(info->dev, "failed to read usbc_stat3\n");
		goto dev_det_ret;
	}


	if ((usbc_stat1 & USBCSTATUS1_TCDET_STAT_MASK) ==
					USBCSTATUS1_TCDET_COMPLETE) {

		memset(&info->cable, 0x0, sizeof(struct wcove_usbc_cable_info));

		info->cable.type = usbc_stat1 & USBCSTATUS1_TCRSLT_MASK;
		info->cable.orient = (usbc_stat1 & USBCSTATUS1_CC_ORIENT_MASK) >>
						USBCSTATUS1_CC_ORIENT_POS;

		ret = regmap_read(info->regmap, WCOVE_USBCSTATUS3_REG, &usbc_stat3);
		if (ret < 0) {
			dev_err(info->dev, "failed to read usbc_stat3\n");
			goto dev_det_ret;
		}
		info->cable.state = usbc_stat3 & USBCSTATUS3_TC_STATE_MASK;
		info->cable.data_role = wcove_usbc_drole_init(info);
		info->cable.power_role = wcove_usbc_prole_init(info);

		info->phy.valid_cc = info->cable.orient;

		dev_info(info->dev, "cable.type(%x), cable.orient(%x)\n",
					info->cable.type, info->cable.orient);

		wcove_usbc_measure_cc(&info->phy, &info->phy.cc1);
		wcove_usbc_measure_cc(&info->phy, &info->phy.cc2);

		if (wcove_usbc_vbus_state(&info->phy))
			atomic_notifier_call_chain(&info->phy.notifier,
					 TYPEC_EVENT_VBUS, &info->phy);
		/* notify cable event */
		wcove_usbc_notify_events(info);

	} else if ((usbc_stat1 & USBCSTATUS1_TCDET_STAT_MASK) ==
					USBCSTATUS1_TCDET_ON_GNG) {
		dev_dbg(info->dev, "Type-C detection on going\n");
		if (info->pmic_rev < WCOVE_PMIC_TYPEC_B2)
			wcove_usbc_update_ccsel(info);
	} else {
		dev_dbg(info->dev,
			"Type-C detection not started or unknown state\n");
		if (!(usbc_stat1 & USBCSTATUS1_TCRSLT_MASK)) {
			memset(&info->cable,
				0x0, sizeof(struct wcove_usbc_cable_info));
			info->phy.valid_cc = 0;
			atomic_notifier_call_chain(&info->phy.notifier,
					 TYPEC_EVENT_NONE, &info->phy);
			regmap_write(info->regmap, WCOVE_PDCFG2_REG, 0x00);
			wcove_usbc_flush_fifo(&info->phy, FIFO_TYPE_RX);
		}
	}

	return;

dev_det_ret:
	if (ret < 0)
		dev_err(info->dev, "CC change detection error\n");
}

static void wcove_usbc_handle_pd_rx_event(struct wcove_usbc_info *info)
{
	int ret, pd_status, rx_status;
	struct typec_phy *phy = &info->phy;
	bool process_rx;

	do {
		ret = regmap_read(info->regmap, WCOVE_PDSTATUS_REG, &pd_status);
		if (ret < 0) {
			dev_err(info->dev, "failed to read pd_status\n");
			return;
		}

		ret = regmap_read(info->regmap, WCOVE_RXSTATUS_REG, &rx_status);
		if (ret < 0) {
			dev_err(info->dev, "failed to read rx_status(%d)\n", ret);
			return;
		}

		dev_dbg(info->dev, "[PD_RX_INT] pd_status(%x) rx_status(%x)\n",
							pd_status, rx_status);
		process_rx = (!(rx_status & RXSTATUS_RXDATA) && ((pd_status & PDSTATUS_RX_RSLT_MASK) == PDSTATUS_RX_RSLT_MSG)) ||
				(rx_status & RXSTATUS_RXDATA);

		if (process_rx)
			phy->notify_protocol(phy, PROT_PHY_EVENT_MSG_RCV);

	} while (process_rx);

}

static void wcove_usnc_handle_pd_event(struct wcove_usbc_info *info,
				       int rx_status,
				       int usbcirq2)
{
	int ret, pd_status, rx_int_mask;
	struct typec_phy *phy = &info->phy;

	ret = regmap_read(info->regmap, WCOVE_PDSTATUS_REG, &pd_status);
	if (ret < 0) {
		dev_err(info->dev, "failed to read pd_status\n");
		return;
	}

	dev_dbg(info->dev, "[INTR] pd_status(%x) rx_status(%x) usbcirq2(%x)\n",
						pd_status, rx_status, usbcirq2);

	rx_int_mask = USBCIRQ2_RX_PD | USBCIRQ2_RX_HR | USBCIRQ2_RX_CR;

	if (info->tx_ongoing && (usbcirq2 & rx_int_mask))
		phy->notify_protocol(phy, PROT_PHY_EVENT_COLLISION);

	if (!info->auto_retry) {
		if (usbcirq2 & USBCIRQ2_TX_SUCCESS)
			phy->notify_protocol(phy, PROT_PHY_EVENT_TX_SENT);

		if (usbcirq2 & USBCIRQ2_TX_FAIL)
			phy->notify_protocol(phy, PROT_PHY_EVENT_TX_FAIL);
	}

	if (usbcirq2 & USBCIRQ2_RX_PD ||
	    rx_status & RXSTATUS_RXDATA)
		wcove_usbc_handle_pd_rx_event(info);

	if (usbcirq2 & USBCIRQ2_RX_HR) {
		if (info->hr_tx_ong)
			phy->notify_protocol(phy, PROT_PHY_EVENT_TX_HARD_RST);
		else
			phy->notify_protocol(phy, PROT_PHY_EVENT_HARD_RST);
	}

	if (usbcirq2 & USBCIRQ2_RX_CR)
		phy->notify_protocol(phy, PROT_PHY_EVENT_SOFT_RST);
}

static irqreturn_t wcove_usbc_threaded_handler(int irq, void *data)
{
	struct wcove_usbc_info *info = data;
	int ret, chgirq0, usbcirq1, usbcirq2;
	int  rx_status;


	ret = regmap_read(info->regmap, WCOVE_CHGRIRQ0_REG, &chgirq0);
	if (ret < 0) {
		dev_err(info->dev, "failed to read chgirq0\n");
		goto dev_det_ret;
	}

	ret = regmap_read(info->regmap, WCOVE_USBCIRQ1_REG, &usbcirq1);
	if (ret < 0) {
		dev_err(info->dev, "failed to read usbcirq1\n");
		goto dev_det_ret;
	}

	ret = regmap_read(info->regmap, WCOVE_USBCIRQ2_REG, &usbcirq2);
	if (ret < 0) {
		dev_err(info->dev, "failed to read usbcirq2\n");
		goto dev_det_ret;
	}
	ret = regmap_read(info->regmap, WCOVE_RXSTATUS_REG, &rx_status);
	if (ret < 0) {
		dev_err(info->dev, "failed to read rx_status(%d)\n", ret);
		goto dev_det_ret;
	}
	dev_dbg(info->dev, "chgirq0(%x) usbcirq1(%x) usbcirq2(%x)\n",
						chgirq0, usbcirq1, usbcirq2);

	if ((usbcirq2 & USBCIRQ2_CC_CHANGE) && !info->pr_swap)
		wcove_usbc_handle_cc_event(info);

	if (usbcirq2 & USBCIRQ2_PDIRQ ||
	    rx_status & RXSTATUS_RXDATA) {
		wcove_usnc_handle_pd_event(info, rx_status, usbcirq2);
		if (!(usbcirq2 & USBCIRQ2_RX_HR) && info->hr_tx_ong) {
			cancel_delayed_work_sync(&info->hr_work);
			wcove_usbc_clear_ignore_vbus(info);
			info->hr_tx_ong = false;
			dev_dbg(info->dev, "clear HR_TX flag from INTR\n");
		}
		info->tx_ongoing = false;
	}

	/* Clear or Ack the handled interrupts */
	regmap_write(info->regmap, WCOVE_USBCIRQ1_REG, usbcirq1);
	regmap_write(info->regmap, WCOVE_USBCIRQ2_REG, usbcirq2);
	regmap_write(info->regmap, WCOVE_CHGRIRQ0_REG,
					chgirq0 & CHGRIRQ0_USBC);

dev_det_ret:
	if (ret < 0)
		dev_err(info->dev, "USB-C detection error\n");

	return IRQ_HANDLED;
}

static void wcove_usbc_vbus_worker(struct work_struct *work)
{
	struct wcove_usbc_info *info = container_of(work,
			    struct wcove_usbc_info, vbus_work.work);
	int ret, spwrsrc;

	ret = regmap_read(info->regmap, WCOVE_SPWRSRC_REG, &spwrsrc);
	if (ret < 0) {
		dev_err(info->dev, "failed to read spwrsrc\n");
		goto dev_det_ret;
	}

	if (((spwrsrc & SPWRSRC_VBUS_DET) != info->vbus_status) && !info->hr_tx_ong) {
		info->vbus_status = spwrsrc & SPWRSRC_VBUS_DET;

		if (info->vbus_status) {
			dev_info(info->dev, "VBUS Attach detected\n");
			cancel_delayed_work_sync(&info->poll_work);
			wcove_usbc_handle_cc_event(info);
			regmap_update_bits(info->regmap, WCOVE_MUSBCIRQ2_REG,
							USBCIRQ2_CC_CHANGE, 0);
		} else {
			dev_info(info->dev, "VBUS Dettach detected\n");
			regmap_update_bits(info->regmap, WCOVE_MUSBCIRQ2_REG,
					USBCIRQ2_CC_CHANGE, USBCIRQ2_CC_CHANGE);
			regmap_write(info->regmap, WCOVE_PDCFG2_REG, 0x00);
			wcove_usbc_flush_fifo(&info->phy, FIFO_TYPE_RX);

			if (info->interval)
				schedule_delayed_work(&info->poll_work,
						msecs_to_jiffies(info->interval));
		}
	}

	dev_info(info->dev, "[VBUSDET] spwrsrc(%x), vbus_status(%x)\n", spwrsrc, info->vbus_status);

dev_det_ret:
	if (ret < 0)
		dev_err(info->dev, "VBUS detection error(%d)\n", ret);
}

static irqreturn_t wcove_usbc_vbus_handler(int irq, void *data)
{
	struct wcove_usbc_info *info = data;
	int ret, spwrsrc;

	dev_info(info->dev, "%s:\n", __func__);

	if (info->pmic_rev < WCOVE_PMIC_TYPEC_B2)
		schedule_delayed_work(&info->vbus_work, 0);

	ret = regmap_read(info->regmap, WCOVE_SPWRSRC_REG, &spwrsrc);
	if (ret < 0) {
		dev_err(info->dev, "failed to read spwrsrc\n");
		return IRQ_HANDLED;
	}

	/* Notify DPM */
	if (spwrsrc & SPWRSRC_VBUS_DET) {
		dev_dbg(info->dev, "Send VBUS ON DPM event\n");
		dpm_handle_phy_event(&info->phy, PHY_DPM_EVENT_VBUS_ON);
	} else {
		dev_dbg(info->dev, "Send VBUS OFF DPM event\n");
		dpm_handle_phy_event(&info->phy, PHY_DPM_EVENT_VBUS_OFF);
	}

	return IRQ_HANDLED;
}

static void wcove_usbc_reg_dump(struct wcove_usbc_info *info)
{
	int val, addr;

	for (addr = WCOVE_USBC_ID_REG; addr <= WCOVE_TXINFO_REG; addr++) {
		regmap_read(info->regmap, addr, &val);
		dev_err(info->dev, "addr(%x) = val(%x)\n", addr, val);
	}
}

/* WC B1 WA DRP interrupts flood issue  */
static ssize_t set_poll_interval(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct wcove_usbc_info *info = dev_get_drvdata(dev);
	unsigned long val;

	if (!sscanf(buf, "%ld\n", &val))
		return -EINVAL;

	if ((long)val < 0)
		return -EINVAL;

	info->interval = val;
	if (val)
		schedule_delayed_work(&info->poll_work,
				msecs_to_jiffies(info->interval));
	else
		cancel_delayed_work_sync(&info->poll_work);

	return count;
}

static ssize_t get_poll_interval(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct wcove_usbc_info *info = dev_get_drvdata(dev);

	wcove_usbc_reg_dump(info);

	return sprintf(buf, "%d\n", info->interval);
}

static DEVICE_ATTR(poll_interval, S_IWUSR | S_IRUGO,
			get_poll_interval, set_poll_interval);

static void wcove_usbc_polling_worker(struct work_struct *work)
{
	struct wcove_usbc_info *info = container_of(work,
			    struct wcove_usbc_info, poll_work.work);

	/* prevent deep sleep as we will read several values */
	pm_qos_update_request(&info->pm_qos_request, 100);
	wcove_usbc_handle_cc_event(info);
	schedule_delayed_work(&info->poll_work,
				msecs_to_jiffies(info->interval));
	pm_qos_update_request(&info->pm_qos_request, PM_QOS_DEFAULT_VALUE);
}

static void wcove_usbc_hr_worker(struct work_struct *work)
{
	struct wcove_usbc_info *info = container_of(work,
			    struct wcove_usbc_info, hr_work.work);

	wcove_usbc_clear_ignore_vbus(info);
	info->hr_tx_ong = false;
	dev_info(info->dev, "clear HR_TX flag\n");
	if (info->pmic_rev < WCOVE_PMIC_TYPEC_B2)
		schedule_delayed_work(&info->vbus_work, 0);
	else
		wcove_usbc_handle_cc_event(info);
}

static void wcove_usbc_init_worker(struct work_struct *work)
{
	struct wcove_usbc_info *info = container_of(work,
			    struct wcove_usbc_info, init_work.work);
	int ret;

	wcove_usbc_handle_cc_event(info);

	/*
	 * Send Cable Reset command if the system  is booted
	 * with Type-C cable connected. This is needed to
	 * restart the PD communication
	 */
	if (info->cable.type != WCOVE_USBC_CABLE_NONE) {
		ret = regmap_write(info->regmap, WCOVE_TXCMD_REG,
				TXCMD_TX_CMD_CR | TXCMD_TX_START);
		if (ret < 0)
			dev_info(info->dev, "TX CR error(%d)\n", ret);
	}
}

#ifdef CONFIG_DEBUG_FS
static int wcove_usbc_debug_show(struct seq_file *s, void *data)
{
	struct wcove_usbc_info *info = s->private;
	int val, addr;

	for (addr = WCOVE_USBC_ID_REG; addr <= WCOVE_TXINFO_REG; addr++) {
		regmap_read(info->regmap, addr, &val);
		seq_printf(s, "[%x] = %x\n", addr, val);
	}

	return 0;
}

static int debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, wcove_usbc_debug_show, inode->i_private);
}

static const struct file_operations wcove_usbc_debug_fops = {
	.open       = debug_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static void wcove_usbc_create_debugfs(struct wcove_usbc_info *info)
{
	info->debug_file = debugfs_create_file("wcove_usbc_phy", 0666, NULL,
		info, &wcove_usbc_debug_fops);
}

static void wcove_usbc_remove_debugfs(struct wcove_usbc_info *info)
{
	debugfs_remove(info->debug_file);
}
#else
static inline void wcove_usbc_create_debugfs(struct wcove_usbc_info *info)
{
}
static inline void wcove_usbc_remove_debugfs(struct wcove_usbc_info *info)
{
}
#endif

static enum wcove_pmic_typec_rev wcove_get_pmic_revision(struct wcove_usbc_info *info)
{
	int ret, version;
	enum wcove_pmic_typec_rev pmic_rev;

	ret = regmap_read(info->regmap, WCOVE_ID0_REG, &version);
	if (ret < 0) {
		dev_err(info->dev, "failed to read wcove_id0 reg\n");
		return WCOVE_PMIC_TYPEC_NONE;
	}

	if ((version & ID0_MAJREV_MASK) == ID0_MAJREV_AX) {
		dev_info(info->dev,
			"Whiskey Cove Ax Stepping detected, ID0(%x)\n", version);
		pmic_rev = WCOVE_PMIC_TYPEC_NONE;
	} else if ((version & ID0_MAJREV_MASK) == ID0_MAJREV_BX) {
		dev_info(info->dev,
			"Whiskey Cove Bx Stepping detected, ID0(%x)\n", version);
		pmic_rev = (version & ID0_MINREV_MASK) + WCOVE_PMIC_TYPEC_B0;
	} else {
		dev_err(info->dev,
			"Whiskey Cove Unknown Stepping detected, ID0(%x)\n", version);
		pmic_rev = WCOVE_PMIC_TYPEC_NONE;
	}

	dev_info(info->dev, "wcove pmic typec rev(%x)\n", pmic_rev);
	return pmic_rev;
}

static void wcove_usbc_hw_init(struct wcove_usbc_info *info)
{
	int ret, chgirq0 = 0, chgirq1 = 0, usbcirq1 = 0, usbcirq2 = 0;

	ret = regmap_read(info->regmap, WCOVE_CHGRIRQ0_REG, &chgirq0);
	if (ret < 0)
		dev_err(info->dev, "failed to read chgirq0\n");

	ret = regmap_read(info->regmap, WCOVE_USBCIRQ1_REG, &usbcirq1);
	if (ret < 0)
		dev_err(info->dev, "failed to read usbcirq1\n");

	ret = regmap_read(info->regmap, WCOVE_USBCIRQ2_REG, &usbcirq2);
	if (ret < 0)
		dev_err(info->dev, "failed to read usbcirq2\n");

	ret = regmap_read(info->regmap, WCOVE_CHGRIRQ1_REG, &chgirq1);
	if (ret < 0)
		dev_err(info->dev, "failed to read chgirq1\n");
	dev_info(info->dev, "[boot] chgirq0(%x) chgirq1(%x) usbcirq1(%x) usbcirq2(%x)\n",
						chgirq0, chgirq1, usbcirq1, usbcirq2);

	/* Clear or Ack the handled interrupts */
	regmap_write(info->regmap, WCOVE_USBCIRQ1_REG, usbcirq1);
	regmap_write(info->regmap, WCOVE_USBCIRQ2_REG, usbcirq2);
	regmap_write(info->regmap, WCOVE_CHGRIRQ0_REG,
					chgirq0 & CHGRIRQ0_USBC);
	regmap_write(info->regmap, WCOVE_CHGRIRQ1_REG,
					chgirq1);
	/* program phy to drm mode */
	regmap_write(info->regmap, WCOVE_USBCCTRL1_REG, USBCCTRL1_DRP_TOGGLE_RDM
			| USBCCTRL1_CURSRC_UA80 | USBCCTRL1_MODES_DRPACC);

	/* mask the cc chnage and short interrupts for B0 and B1 */
	regmap_write(info->regmap, WCOVE_MUSBCIRQ2_REG, 0x0);
	regmap_write(info->regmap, WCOVE_MUSBCIRQ1_REG, 0x0);
	if (info->pmic_rev < WCOVE_PMIC_TYPEC_B2) {
		regmap_write(info->regmap, WCOVE_MUSBCIRQ2_REG, USBCIRQ2_CC_CHANGE);
		regmap_write(info->regmap, WCOVE_MUSBCIRQ1_REG, USBCIRQ1_SHORT);
	}

	regmap_write(info->regmap, WCOVE_PDCFG1_REG, PDCFG1_ID_FILL);

	regmap_update_bits(info->regmap,
		WCOVE_USBCCTRL3_REG, USBCCTRL3_PD_DIS, USBCCTRL3_PD_DIS);

	regmap_update_bits(info->regmap, WCOVE_PDCFG3_REG,
			PDCFG3_SR_SOP0_MASK | PDCFG3_SR_SOP1_MASK,
			PDCFG3_SR_SOP0_REV2 | PDCFG3_SR_SOP1_REV2);

	regmap_update_bits(info->regmap, WCOVE_CC1CTRL_REG, CCXCTRL_TX_EN, 0);
	regmap_update_bits(info->regmap, WCOVE_CC2CTRL_REG, CCXCTRL_TX_EN, 0);
	wcove_usbc_reg_dump(info);
}

static int wcove_usbc_probe(struct platform_device *pdev)
{
	struct wcove_usbc_info *info;
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);
	int ret, pirq;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->regmap = wcove->regmap;
	info->regmap_irq_chip = wcove->irq_chip_data_level2;
	platform_set_drvdata(pdev, info);

	/* Get PMIC Type-C revision */
	info->pmic_rev = wcove_get_pmic_revision(info);
	if (info->pmic_rev == WCOVE_PMIC_TYPEC_NONE)
		return -ENODEV;

	pirq = platform_get_irq(pdev, 0);
	info->irq = regmap_irq_get_virq(info->regmap_irq_chip, pirq);
	if (info->irq < 0) {
		dev_err(&pdev->dev,
				"failed to get virtual interrupt=%d\n", pirq);
		return info->irq;
	}

	mutex_init(&info->lock);

	info->phy.support_auto_goodcrc = true;

	info->phy.dev = &pdev->dev;
	info->phy.label = "wcove_usbc";
	info->phy.ops.set_host_current = wcove_usbc_set_host_current;
	info->phy.ops.get_host_current = wcove_usbc_get_host_current;
	info->phy.ops.switch_mode = wcove_usbc_switch_mode;
	info->phy.ops.setup_cc = wcove_usbc_setup_cc;
	info->phy.ops.set_bist_cm2 = wcove_usbc_set_bist_cm2;

	info->phy.get_pd_version = wcove_usbc_pd_version;
	info->phy.is_pd_capable = wcove_usbc_pd_support;
	info->phy.phy_reset = wcove_usbc_phy_reset;
	info->phy.flush_fifo = wcove_usbc_flush_fifo;
	info->phy.send_packet = wcove_usbc_send_pkt;
	info->phy.recv_packet = wcove_usbc_receive_pkt;
	info->phy.is_vbus_on = wcove_usbc_vbus_state;
	info->phy.is_vconn_on = wcove_usbc_is_vconn_enabled;
	info->phy.enable_vconn = wcove_usbc_enable_vconn;
	info->phy.set_pu_pd = wcove_usbc_set_pu_pd;
	info->phy.setup_role = wcove_usbc_setup_role;
	info->phy.enable_autocrc = wcove_usbc_enable_autocrc;
	info->phy.set_swap_state = wcove_usbc_set_swap_state;
	info->phy.enable_auto_retry = wcove_usbc_enable_auto_retry;
	info->phy.enable_sop_prime = wcove_usbc_enable_sop_prime;
	info->phy.cc1.id = TYPEC_PIN_CC1;
	info->phy.cc2.id = TYPEC_PIN_CC2;

	pm_qos_add_request(&info->pm_qos_request, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE);
	typec_add_phy(&info->phy);
	typec_bind_detect(&info->phy);
	syspolicy_register_typec_phy(&info->phy);

	wcove_usbc_hw_init(info);

	ret = devm_request_threaded_irq(&pdev->dev, info->irq,
				NULL, wcove_usbc_threaded_handler,
					IRQF_ONESHOT, pdev->name, info);
	if (ret) {
		dev_err(&pdev->dev,
				"failed to request interrupt=%d\n", info->irq);
		typec_unbind_detect(&info->phy);
		typec_remove_phy(&info->phy);
		return ret;
	}

	/* B0/B1 WA to detect VBUS events */
	pirq = platform_get_irq(pdev, 1);
	info->irq_vbus = regmap_irq_get_virq(info->regmap_irq_chip, pirq);
	if (info->irq_vbus < 0) {
		dev_err(&pdev->dev,
				"failed to get virtual interrupt=%d\n", pirq);
	} else {
		ret = devm_request_threaded_irq(&pdev->dev, info->irq_vbus,
					NULL, wcove_usbc_vbus_handler,
					IRQF_ONESHOT, pdev->name, info);
		if (ret)
			dev_err(&pdev->dev,
				"failed to request interrupt=%d\n", info->irq);
	}

	INIT_DELAYED_WORK(&info->init_work, wcove_usbc_init_worker);
	INIT_DELAYED_WORK(&info->hr_work, wcove_usbc_hr_worker);
	INIT_DELAYED_WORK(&info->vbus_work, wcove_usbc_vbus_worker);

	/* B0/B1 WA for DRP interrupt flood issue */
	if (info->pmic_rev < WCOVE_PMIC_TYPEC_B2) {
		info->interval = 5000;
		INIT_DELAYED_WORK(&info->poll_work, wcove_usbc_polling_worker);
		ret = device_create_file(info->dev, &dev_attr_poll_interval);
		if (ret < 0)
			dev_err(&pdev->dev, "failed to create sysfs entry\n");

		schedule_delayed_work(&info->vbus_work, 0);
		schedule_delayed_work(&info->poll_work,
					msecs_to_jiffies(info->interval));
	} else {
		/*
		 * Schedule the init handling to delayed worker
		 * to reduce the driver loading time and to allow
		 * other Type-C dependent components to load.
		 */
		schedule_delayed_work(&info->init_work, 5 * HZ);
	}

	/* Unmask extcon interrupt */
	regmap_update_bits(info->regmap, WCOVE_MCHGRIRQ0_REG, CHGRIRQ0_USBC, 0);
	wcove_usbc_create_debugfs(info);

	return 0;
}

static int wcove_usbc_remove(struct platform_device *pdev)
{
	struct wcove_usbc_info *info = platform_get_drvdata(pdev);
	struct typec_phy *phy = &info->phy;

	pm_qos_remove_request(&info->pm_qos_request);
	wcove_usbc_remove_debugfs(info);
	syspolicy_unregister_typec_phy(phy);
	typec_unbind_detect(phy);
	typec_remove_phy(phy);
	return 0;
}

static struct platform_device_id wcove_usbc_device_ids[] = {
	{"bxt_wcove_usbc", 0},
	{},
};

static struct platform_driver wcove_usbc_driver = {
	.driver = {
		   .name = "wcove_usbc",
		   .owner = THIS_MODULE,
		   },
	.probe = wcove_usbc_probe,
	.remove = wcove_usbc_remove,
	.id_table = wcove_usbc_device_ids,
};
module_platform_driver(wcove_usbc_driver);
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Intel Whiskey Cove Type-C PHY driver");
MODULE_LICENSE("GPL v2");
