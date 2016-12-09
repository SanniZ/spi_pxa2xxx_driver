/*
 * TI BQ25890 charger driver
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/extcon.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>
#include <linux/usb_typec_phy.h>
#include <linux/power/charging_algo.h>
#include <linux/power/bq25890_charger.h>

#include <linux/acpi.h>
#include <linux/of.h>

#define BQ25890_MANUFACTURER		"Texas Instruments"
#define BQ25890_IRQ_PIN			"bq25890_irq"

#define BQ25890_ILIM_REG		0x0
#define ILIM_HIZ_EN			(1 << 7)
#define ILIM_PIN_EN			(1 << 6)

#define BQ25890_CONFIG_REG		0x02
#define CONFIG_ADC_CONV_START		(1 << 7)
#define CONFIG_ADC_CONV_RATE		(1 << 6)
#define CONFIG_BOOST_FREQ		(1 << 5)
#define CONFIG_ICO_EN			(1 << 4)
#define CONFIG_HVDCP_EN			(1 << 3)
#define CONFIG_MAX_EN			(1 << 2)
#define CONFIG_FORCE_DPDM		(1 << 1)
#define CONFIG_AUTO_DPDM		(1 << 0)
/*
 * Default configuration for BQ25890_CONFIG_REG.
 * CONFIG_AUTO_DPDM means detecting USB automatically.
 * This bit makes the status registers of bq25890 more
 * stable. We enable CONFIG_AUTO_DPDM here.
 */
#define CONFIG_DEF_VAL			(CONFIG_ICO_EN | CONFIG_AUTO_DPDM)

#define BQ25890_CHG_CNTL_REG		0x03
#define CHG_CNTL_BAT_LOAD		(1 << 7)
#define CHG_CNTL_WDT_RST		(1 << 6)
#define CHG_CNTL_OTG_EN			(1 << 5)
#define CHG_CNTL_ENABLE			(1 << 4)
#define CHG_SYSMIN_DEF			(5 << 1)

#define BQ25890_ICHG_REG		0x04
#define ICHG_PUMPX_EN			(1 << 7)
#define ICHG_CC_MASK			0x7f

#define BQ25890_IPRECHG_ITERM_REG	0x05
#define IPRECHG_MASK			0xf0
#define IPRECHG_DEF_VAL			(4 << 4)
#define ITERM_MASK			0x0f
#define ITERM_DEF_VAL			(4 << 4)

#define BQ25890_VREG_REG		0x06
#define VREG_CV_MASK			0xfc
#define VREG_CV_OFF			0x2
#define VREG_BATLOWV			(1 << 1)
#define VREG_RECHG_200MV		(1 << 0)

#define BQ25890_TMR_CNTL_REG		0x07
#define TMR_EN_CHG_TERM			(1 << 7)
#define TMR_EN_STAT_DIS			(1 << 6)
#define TMR_WDT_MASK			(3 << 4)
#define TMR_WDT_DISABLE			(0 << 4)
#define TMR_WDT_40SEC			(1 << 4)
#define TMR_WDT_80SEC			(2 << 4)
#define TMR_WDT_160SEC			(3 << 4)
#define TMR_EN_SFT_TMR			(1 << 3)
#define TMR_SFT_TMR_MASK		(3 << 1)
#define TMR_SFT_TMR_5HRS		(0 << 1)
#define TMR_SFT_TMR_8HRS		(1 << 1)
#define TMR_SFT_TMR_12HRS		(2 << 1)
#define TMR_SFT_TMR_20HRS		(3 << 1)
#define TMR_JEITA_I20PERC		(1 << 0)
#define TMR_CNTL_DEF_VAL		0x8f

#define BQ25890_VINDPM_REG		0x0d
#define VINDMP_DFF_VAL			0x90

#define BQ25890_BATT_MODEL		"INTN0001"
#define BQ25890_DEF_CC			2600000	/* in uA */
#define BQ25890_DEF_IPRECHG		256000	/* in uA */
#define BQ25890_DEF_ITERM		128000	/* in uA */
#define BQ25890_DEF_CV			4350000	/* in uV */
#define BQ25890_DEF_VSYSMIN		3500000	/* in uV */
#define BQ25890_DEF_BOOSTV		4700000	/* in uV */
#define BQ25890_DEF_BOOSTI		500000	/* in uA */
#define BQ25890_MAX_CC			2600000	/* in uA */
#define BQ25890_MAX_CV			4350000	/* in uV */
#define BQ25890_MAX_TEMP		100	/* in DegC */
#define BQ25890_MIN_TEMP		0	/* in DegC */

#define BQ25890_SDP_ILIM		500000	/* in uA */
#define BQ25890_CDP_ILIM		1500000	/* in uA */
#define BQ25890_DCP_ILIM		2000000	/* in uA */
#define BQ25890_ACA_ILIM		2000000	/* in uA */


#define BQ25890_ID			3
#define BQ25892_ID			0

enum bq25890_fields {
	F_EN_HIZ, F_EN_ILIM, F_IILIM,				     /* Reg00 */
	F_BHOT, F_BCOLD, F_VINDPM_OFS,				     /* Reg01 */
	F_CONV_START, F_CONV_RATE, F_BOOSTF, F_ICO_EN,
	F_HVDCP_EN, F_MAXC_EN, F_FORCE_DPM, F_AUTO_DPDM_EN,	     /* Reg02 */
	F_BAT_LOAD_EN, F_WD_RST, F_OTG_CFG, F_CHG_CFG, F_SYSVMIN,    /* Reg03 */
	F_PUMPX_EN, F_ICHG,					     /* Reg04 */
	F_IPRECHG, F_ITERM,					     /* Reg05 */
	F_VREG, F_BATLOWV, F_VRECHG,				     /* Reg06 */
	F_TERM_EN, F_STAT_DIS, F_WD, F_TMR_EN, F_CHG_TMR,
	F_JEITA_ISET,						     /* Reg07 */
	F_BATCMP, F_VCLAMP, F_TREG,				     /* Reg08 */
	F_FORCE_ICO, F_TMR2X_EN, F_BATFET_DIS, F_JEITA_VSET,
	F_BATFET_DLY, F_BATFET_RST_EN, F_PUMPX_UP, F_PUMPX_DN,	     /* Reg09 */
	F_BOOSTV, F_BOOSTI,					     /* Reg0A */
	F_VBUS_STAT, F_CHG_STAT, F_PG_STAT, F_SDP_STAT, F_VSYS_STAT, /* Reg0B */
	F_WD_FAULT, F_BOOST_FAULT, F_CHG_FAULT, F_BAT_FAULT,
	F_NTC_FAULT,						     /* Reg0C */
	F_FORCE_VINDPM, F_VINDPM,				     /* Reg0D */
	F_THERM_STAT, F_BATV,					     /* Reg0E */
	F_SYSV,							     /* Reg0F */
	F_TSPCT,						     /* Reg10 */
	F_VBUS_GD, F_VBUSV,					     /* Reg11 */
	F_ICHGR,						     /* Reg12 */
	F_VDPM_STAT, F_IDPM_STAT, F_IDPM_LIM,			     /* Reg13 */
	F_REG_RST, F_ICO_OPTIMIZED, F_PN, F_TS_PROFILE, F_DEV_REV,   /* Reg14 */

	F_MAX_FIELDS
};

/* initial field values, converted to register values */
struct bq25890_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 boostv;	/* boost regulation voltage	*/
	u8 boosti;	/* boost current limit		*/
	u8 boostf;	/* boost frequency		*/
	u8 ilim_en;	/* enable ILIM pin		*/
	u8 treg;	/* thermal regulation threshold */
};

struct bq25890_state {
	u8 online;
	u8 chrg_status;
	u8 chrg_fault;
	u8 vsys_status;
	u8 boost_fault;
	u8 bat_fault;
};

struct bq25890_device {
	struct i2c_client *client;
	struct bq25890_platform_data *pdata;
	struct device *dev;
	struct power_supply *charger;

	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	struct delayed_work chg_refresh_work;
	struct work_struct init_work;
	unsigned long usb_event;
	struct delayed_work thermal_work;

	/* extcon charger cables */
	struct {
		struct work_struct work;
		struct notifier_block nb;
		struct extcon_specific_cable_nb sdp;
		struct extcon_specific_cable_nb cdp;
		struct extcon_specific_cable_nb dcp;
		struct extcon_specific_cable_nb otg;
		struct extcon_specific_cable_nb typec;
		enum power_supply_type chg_type;
		bool boost;
		bool connected;
	} cable;

	struct regmap *rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	int chip_id;
	struct bq25890_init_data init_data;
	struct bq25890_state state;
	bool batt_full;

	struct mutex lock; /* protect state data */

	int cc;
	int cv;
	int inlmt;
	int chg_cntl_lmt;
	int chg_cntl_max;
	int vrechg_delta;
};

static struct bq25890_device *info_ptr;

static const struct regmap_range bq25890_readonly_reg_ranges[] = {
	regmap_reg_range(0x0b, 0x0c),
	regmap_reg_range(0x0e, 0x13),
};

static const struct regmap_access_table bq25890_writeable_regs = {
	.no_ranges = bq25890_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bq25890_readonly_reg_ranges),
};

static const struct regmap_range bq25890_volatile_reg_ranges[] = {
	regmap_reg_range(0x00, 0x00),
	regmap_reg_range(0x09, 0x09),
	regmap_reg_range(0x0b, 0x0c),
	regmap_reg_range(0x0e, 0x14),
};

static const struct regmap_access_table bq25890_volatile_regs = {
	.yes_ranges = bq25890_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25890_volatile_reg_ranges),
};

static const struct regmap_config bq25890_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x14,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bq25890_writeable_regs,
	.volatile_table = &bq25890_volatile_regs,
};

static const struct reg_field bq25890_reg_fields[] = {
	/* REG00 */
	[F_EN_HIZ]		= REG_FIELD(0x00, 7, 7),
	[F_EN_ILIM]		= REG_FIELD(0x00, 6, 6),
	[F_IILIM]		= REG_FIELD(0x00, 0, 5),
	/* REG01 */
	[F_BHOT]		= REG_FIELD(0x01, 6, 7),
	[F_BCOLD]		= REG_FIELD(0x01, 5, 5),
	[F_VINDPM_OFS]		= REG_FIELD(0x01, 0, 4),
	/* REG02 */
	[F_CONV_START]		= REG_FIELD(0x02, 7, 7),
	[F_CONV_RATE]		= REG_FIELD(0x02, 6, 6),
	[F_BOOSTF]		= REG_FIELD(0x02, 5, 5),
	[F_ICO_EN]		= REG_FIELD(0x02, 4, 4),
	[F_HVDCP_EN]		= REG_FIELD(0x02, 3, 3),
	[F_MAXC_EN]		= REG_FIELD(0x02, 2, 2),
	[F_FORCE_DPM]		= REG_FIELD(0x02, 1, 1),
	[F_AUTO_DPDM_EN]	= REG_FIELD(0x02, 0, 0),
	/* REG03 */
	[F_BAT_LOAD_EN]		= REG_FIELD(0x03, 7, 7),
	[F_WD_RST]		= REG_FIELD(0x03, 6, 6),
	[F_OTG_CFG]		= REG_FIELD(0x03, 5, 5),
	[F_CHG_CFG]		= REG_FIELD(0x03, 4, 4),
	[F_SYSVMIN]		= REG_FIELD(0x03, 1, 3),
	/* REG04 */
	[F_PUMPX_EN]		= REG_FIELD(0x04, 7, 7),
	[F_ICHG]		= REG_FIELD(0x04, 0, 6),
	/* REG05 */
	[F_IPRECHG]		= REG_FIELD(0x05, 4, 7),
	[F_ITERM]		= REG_FIELD(0x05, 0, 3),
	/* REG06 */
	[F_VREG]		= REG_FIELD(0x06, 2, 7),
	[F_BATLOWV]		= REG_FIELD(0x06, 1, 1),
	[F_VRECHG]		= REG_FIELD(0x06, 0, 0),
	/* REG07 */
	[F_TERM_EN]		= REG_FIELD(0x07, 7, 7),
	[F_STAT_DIS]		= REG_FIELD(0x07, 6, 6),
	[F_WD]			= REG_FIELD(0x07, 4, 5),
	[F_TMR_EN]		= REG_FIELD(0x07, 3, 3),
	[F_CHG_TMR]		= REG_FIELD(0x07, 1, 2),
	[F_JEITA_ISET]		= REG_FIELD(0x07, 0, 0),
	/* REG08 */
	[F_BATCMP]		= REG_FIELD(0x08, 6, 7),
	[F_VCLAMP]		= REG_FIELD(0x08, 2, 4),
	[F_TREG]		= REG_FIELD(0x08, 0, 1),
	/* REG09 */
	[F_FORCE_ICO]		= REG_FIELD(0x09, 7, 7),
	[F_TMR2X_EN]		= REG_FIELD(0x09, 6, 6),
	[F_BATFET_DIS]		= REG_FIELD(0x09, 5, 5),
	[F_JEITA_VSET]		= REG_FIELD(0x09, 4, 4),
	[F_BATFET_DLY]		= REG_FIELD(0x09, 3, 3),
	[F_BATFET_RST_EN]	= REG_FIELD(0x09, 2, 2),
	[F_PUMPX_UP]		= REG_FIELD(0x09, 1, 1),
	[F_PUMPX_DN]		= REG_FIELD(0x09, 0, 0),
	/* REG0A */
	[F_BOOSTV]		= REG_FIELD(0x0A, 4, 7),
	[F_BOOSTI]		= REG_FIELD(0x0A, 0, 2),
	/* REG0B */
	[F_VBUS_STAT]		= REG_FIELD(0x0B, 5, 7),
	[F_CHG_STAT]		= REG_FIELD(0x0B, 3, 4),
	[F_PG_STAT]		= REG_FIELD(0x0B, 2, 2),
	[F_SDP_STAT]		= REG_FIELD(0x0B, 1, 1),
	[F_VSYS_STAT]		= REG_FIELD(0x0B, 0, 0),
	/* REG0C */
	[F_WD_FAULT]		= REG_FIELD(0x0C, 7, 7),
	[F_BOOST_FAULT]		= REG_FIELD(0x0C, 6, 6),
	[F_CHG_FAULT]		= REG_FIELD(0x0C, 4, 5),
	[F_BAT_FAULT]		= REG_FIELD(0x0C, 3, 3),
	[F_NTC_FAULT]		= REG_FIELD(0x0C, 0, 2),
	/* REG0D */
	[F_FORCE_VINDPM]	= REG_FIELD(0x0D, 7, 7),
	[F_VINDPM]		= REG_FIELD(0x0D, 0, 6),
	/* REG0E */
	[F_THERM_STAT]		= REG_FIELD(0x0E, 7, 7),
	[F_BATV]		= REG_FIELD(0x0E, 0, 6),
	/* REG0F */
	[F_SYSV]		= REG_FIELD(0x0F, 0, 6),
	/* REG10 */
	[F_TSPCT]		= REG_FIELD(0x10, 0, 6),
	/* REG11 */
	[F_VBUS_GD]		= REG_FIELD(0x11, 7, 7),
	[F_VBUSV]		= REG_FIELD(0x11, 0, 6),
	/* REG12 */
	[F_ICHGR]		= REG_FIELD(0x12, 0, 6),
	/* REG13 */
	[F_VDPM_STAT]		= REG_FIELD(0x13, 7, 7),
	[F_IDPM_STAT]		= REG_FIELD(0x13, 6, 6),
	[F_IDPM_LIM]		= REG_FIELD(0x13, 0, 5),
	/* REG14 */
	[F_REG_RST]		= REG_FIELD(0x14, 7, 7),
	[F_ICO_OPTIMIZED]	= REG_FIELD(0x14, 6, 6),
	[F_PN]			= REG_FIELD(0x14, 3, 5),
	[F_TS_PROFILE]		= REG_FIELD(0x14, 2, 2),
	[F_DEV_REV]		= REG_FIELD(0x14, 0, 1)
};

/*
 * Most of the val -> idx conversions can be computed, given the minimum,
 * maximum and the step between values. For the rest of conversions, we use
 * lookup tables.
 */
enum bq25890_table_ids {
	/* range tables */
	TBL_ILIM,
	TBL_ICHG,
	TBL_ITERM,
	TBL_IPRECHG,
	TBL_VREG,
	TBL_BATCMP,
	TBL_VCLAMP,
	TBL_BOOSTV,
	TBL_SYSVMIN,

	/* lookup tables */
	TBL_TREG,
	TBL_BOOSTI,
};

/* Thermal Regulation Threshold lookup table, in degrees Celsius */
static const u32 bq25890_treg_tbl[] = { 60, 80, 100, 120 };

#define BQ25890_TREG_TBL_SIZE		ARRAY_SIZE(bq25890_treg_tbl)

/* Boost mode current limit lookup table, in uA */
static const u32 bq25890_boosti_tbl[] = {
	500000, 700000, 1100000, 1300000, 1600000, 1800000, 2100000, 2400000
};

#define BQ25890_BOOSTI_TBL_SIZE		ARRAY_SIZE(bq25890_boosti_tbl)

struct bq25890_range {
	u32 min;
	u32 max;
	u32 step;
};

struct bq25890_lookup {
	const u32 *tbl;
	u32 size;
};

static const union {
	struct bq25890_range  rt;
	struct bq25890_lookup lt;
} bq25890_tables[] = {
	/* range tables */
	[TBL_ILIM] =	{ .rt = {100000,	3250000, 50000} },	/* uA */
	[TBL_ICHG] =	{ .rt = {0,	  5056000, 64000} },	 /* uA */
	[TBL_ITERM] =	{ .rt = {64000,   1024000, 64000} },	 /* uA */
	[TBL_IPRECHG] =	{ .rt = {64000,   1024000, 64000} },	 /* uA */
	[TBL_VREG] =	{ .rt = {3840000, 4608000, 16000} },	 /* uV */
	[TBL_BATCMP] =	{ .rt = {0,	  140,     20} },	 /* mOhm */
	[TBL_VCLAMP] =	{ .rt = {0,	  224000,  32000} },	 /* uV */
	[TBL_BOOSTV] =	{ .rt = {4550000, 5510000, 64000} },	 /* uV */
	[TBL_SYSVMIN] = { .rt = {3000000, 3700000, 100000} },	 /* uV */

	/* lookup tables */
	[TBL_TREG] =	{ .lt = {bq25890_treg_tbl, BQ25890_TREG_TBL_SIZE} },
	[TBL_BOOSTI] =	{ .lt = {bq25890_boosti_tbl, BQ25890_BOOSTI_TBL_SIZE} }
};

extern int typec_get_pd_profile(struct typec_pd_profile *profile);

static int bq25890_field_read(struct bq25890_device *bq,
			      enum bq25890_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(bq->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}

static int bq25890_field_write(struct bq25890_device *bq,
			       enum bq25890_fields field_id, u8 val)
{
	return regmap_field_write(bq->rmap_fields[field_id], val);
}

static u8 bq25890_find_idx(u32 value, enum bq25890_table_ids id)
{
	u8 idx;

	if (id >= TBL_TREG) {
		const u32 *tbl = bq25890_tables[id].lt.tbl;
		u32 tbl_size = bq25890_tables[id].lt.size;

		for (idx = 1; idx < tbl_size && tbl[idx] <= value; idx++)
			;
	} else {
		const struct bq25890_range *rtbl = &bq25890_tables[id].rt;
		u8 rtbl_size;

		rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

		for (idx = 1;
		     idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value);
		     idx++)
			;
	}

	return idx - 1;
}

static u32 bq25890_find_val(u8 idx, enum bq25890_table_ids id)
{
	const struct bq25890_range *rtbl;

	/* lookup table? */
	if (id >= TBL_TREG)
		return bq25890_tables[id].lt.tbl[idx];

	/* range table */
	rtbl = &bq25890_tables[id].rt;

	return (rtbl->min + idx * rtbl->step);
}

enum bq25890_status {
	STATUS_NOT_CHARGING,
	STATUS_PRE_CHARGING,
	STATUS_FAST_CHARGING,
	STATUS_TERMINATION_DONE,
};

enum bq25890_chrg_fault {
	CHRG_FAULT_NORMAL,
	CHRG_FAULT_INPUT,
	CHRG_FAULT_THERMAL_SHUTDOWN,
	CHRG_FAULT_TIMER_EXPIRED,
};

static int bq25890_get_chip_state(struct bq25890_device *bq,
				  struct bq25890_state *state)
{
	int i, ret;

	struct {
		enum bq25890_fields id;
		u8 *data;
	} state_fields[] = {
		{F_CHG_STAT,	&state->chrg_status},
		{F_PG_STAT,	&state->online},
		{F_VSYS_STAT,	&state->vsys_status},
		{F_BOOST_FAULT, &state->boost_fault},
		{F_BAT_FAULT,	&state->bat_fault},
		{F_CHG_FAULT,	&state->chrg_fault}
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = bq25890_field_read(bq, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	dev_info(bq->dev, "S:CHG/PG/VSYS=%d/%d/%d, F:CHG/BOOST/BAT=%d/%d/%d\n",
		state->chrg_status, state->online, state->vsys_status,
		state->chrg_fault, state->boost_fault, state->bat_fault);

	return 0;
}

static void bq25890_set_ilim(struct bq25890_device *bq, int ilim)
{
	int ret;

	/* disable ilim pin */
	ret = bq25890_field_write(bq, F_EN_ILIM, 0);
	if (ret < 0)
		dev_warn(bq->dev, "OTG disable failed(%d)\n", ret);

	ret = bq25890_field_write(bq, F_IILIM,
				bq25890_find_idx(ilim, TBL_ILIM));
	if (ret < 0)
		dev_warn(bq->dev, "ilim setting failed(%d)\n", ret);

	ret = bq25890_field_write(bq, F_EN_HIZ, ilim ? 0 : 1);
	if (ret < 0)
		dev_warn(bq->dev, "HiZ setting failed(%d)\n", ret);
}

static void bq25890_set_cc(struct bq25890_device *bq, int cc)
{
	int ret;
	u8 val;

	val = ICHG_CC_MASK & bq25890_find_idx(cc, TBL_ICHG);
	ret = regmap_write(bq->rmap, BQ25890_ICHG_REG, val);
	if (ret < 0)
		dev_warn(bq->dev, "cc setting failed(%d)\n", ret);
}

static void bq25890_set_cv(struct bq25890_device *bq, int cv)
{
	int ret;
	u8 val;

	val = bq25890_find_idx(cv, TBL_VREG) << VREG_CV_OFF;
	val = (VREG_CV_MASK & val) | VREG_BATLOWV;
	ret = regmap_write(bq->rmap, BQ25890_VREG_REG, val);
	if (ret < 0)
		dev_warn(bq->dev, "cv setting failed(%d)\n", ret);
}

static void bq25890_set_iterm(struct bq25890_device *bq, int iterm)
{
	int ret;
	u8 val;

	val = IPRECHG_DEF_VAL;
	val |= ITERM_MASK & bq25890_find_idx(iterm, TBL_ITERM);
	ret = regmap_write(bq->rmap, BQ25890_IPRECHG_ITERM_REG, val);
	if (ret < 0)
		dev_warn(bq->dev, "iterm setting failed(%d)\n", ret);
}

static void bq25890_enable_charging(struct bq25890_device *bq, bool enable)
{
	int ret;
	u8 val;

	/* set vindpm to 4.2V */
	ret = regmap_write(bq->rmap, BQ25890_VINDPM_REG, VINDMP_DFF_VAL);
	if (ret < 0)
		dev_warn(bq->dev, "vindpm setting failed(%d)\n", ret);

	ret = regmap_write(bq->rmap, BQ25890_CONFIG_REG, CONFIG_DEF_VAL);
	if (ret < 0)
		dev_warn(bq->dev, "config setting failed(%d)\n", ret);

	if (enable)
		val = CHG_CNTL_ENABLE;
	else
		val = (~CHG_CNTL_ENABLE) & CHG_CNTL_ENABLE;

	val |= CHG_SYSMIN_DEF;
	ret = regmap_write(bq->rmap, BQ25890_CHG_CNTL_REG, val);
	if (ret < 0)
		dev_warn(bq->dev, "charging setting failed(%d)\n", ret);

}

static void bq25890_program_timers(struct bq25890_device *bq)
{
	int ret;

	ret = regmap_write(bq->rmap, BQ25890_TMR_CNTL_REG, TMR_CNTL_DEF_VAL);
	if (ret < 0)
		dev_warn(bq->dev, "timer setting failed(%d)\n", ret);
}

static int bq25890_get_charging_status(struct bq25890_device *bq,
					struct bq25890_state *state)
{
	int charging_status;

	if (bq->cable.boost || (!state->online && !bq->cable.connected))
		charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if ((!state->online && bq->cable.connected) ||
			(state->chrg_status == STATUS_NOT_CHARGING))
		charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (state->chrg_status == STATUS_PRE_CHARGING ||
		 state->chrg_status == STATUS_FAST_CHARGING)
		charging_status = POWER_SUPPLY_STATUS_CHARGING;
	else if (state->chrg_status == STATUS_TERMINATION_DONE)
		charging_status = POWER_SUPPLY_STATUS_FULL;
	else
		charging_status = POWER_SUPPLY_STATUS_UNKNOWN;

	return charging_status;
}

static int bq25890_get_charger_health(struct bq25890_device *bq,
					struct bq25890_state *state)
{
	int charger_health;

	if (bq->cable.boost || (!state->online && !bq->cable.connected))
		charger_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (state->online && !state->chrg_fault &&
			!state->bat_fault && !state->boost_fault)
		charger_health = POWER_SUPPLY_HEALTH_GOOD;
	else if (state->chrg_fault == CHRG_FAULT_TIMER_EXPIRED)
		charger_health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	else if (state->chrg_fault == CHRG_FAULT_THERMAL_SHUTDOWN)
		charger_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		charger_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

	return charger_health;
}

static bool bq25890_state_changed(struct bq25890_device *bq,
				  struct bq25890_state *new_state)
{
	struct bq25890_state old_state;

	mutex_lock(&bq->lock);
	memcpy(&old_state, &bq->state, sizeof(struct bq25890_state));
	mutex_unlock(&bq->lock);

	return (old_state.chrg_status != new_state->chrg_status ||
		old_state.chrg_fault != new_state->chrg_fault	||
		old_state.online != new_state->online		||
		old_state.bat_fault != new_state->bat_fault	||
		old_state.boost_fault != new_state->boost_fault ||
		old_state.vsys_status != new_state->vsys_status);
}

static void bq25890_handle_state_change(struct bq25890_device *bq,
					struct bq25890_state *new_state)
{
	int ret;
	struct bq25890_state old_state;

	mutex_lock(&bq->lock);
	memcpy(&old_state, &bq->state, sizeof(struct bq25890_state));
	mutex_unlock(&bq->lock);

	if (!new_state->online) {			     /* power removed */
		/* disable ADC */
		ret = bq25890_field_write(bq, F_CONV_START, 0);
		if (ret < 0)
			goto error;
	} else if (!old_state.online) {			    /* power inserted */
		/* enable ADC, to have control of charge current/voltage */
		ret = bq25890_field_write(bq, F_CONV_START, 1);
		if (ret < 0)
			goto error;
	}

	return;

error:
	dev_err(bq->dev, "Error communicating with the chip.\n");
}

static irqreturn_t bq25890_irq_handler_thread(int irq, void *private)
{
	struct bq25890_device *bq = private;
	int ret;
	struct bq25890_state state;

	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0)
		goto handled;

	if (!bq25890_state_changed(bq, &state))
		goto handled;

	bq25890_handle_state_change(bq, &state);

	mutex_lock(&bq->lock);
	memcpy(&bq->state, &state, sizeof(struct bq25890_state));
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

handled:
	/* Recover error or fault cases */
	if (bq->cable.connected && !bq->cable.boost)
		schedule_delayed_work(&bq->chg_refresh_work, HZ);
	return IRQ_HANDLED;
}

static void bq25890_setup_charging(struct bq25890_device *bq)
{
	bq25890_set_ilim(bq, bq->inlmt);

	/*
	 * Ff CC or CV is zero then don't
	 * enable the battery charging.
	 */
	if (!bq->cc || !bq->cv) {
		bq25890_enable_charging(bq, false);
		return;
	}

	bq25890_enable_charging(bq, true);
	bq25890_set_cc(bq, bq->cc);
	bq25890_set_iterm(bq,
		bq25890_find_val(bq->init_data.iterm, TBL_ITERM));
	bq25890_set_cv(bq, bq->cv);
	bq25890_program_timers(bq);
}

static int bq25890_chip_reset(struct bq25890_device *bq)
{
	int ret;
	int rst_check_counter = 10;

	ret = bq25890_field_write(bq, F_REG_RST, 1);
	if (ret < 0)
		return ret;

	do {
		ret = bq25890_field_read(bq, F_REG_RST);
		if (ret < 0)
			return ret;

		usleep_range(5, 10);
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
		return -ETIMEDOUT;

	return 0;
}

static int bq25890_hw_init(struct bq25890_device *bq)
{
	int ret;
	int i;
	struct bq25890_state state;

	const struct {
		enum bq25890_fields id;
		u32 value;
	} init_data[] = {
		{F_ICHG,	 bq->init_data.ichg},
		{F_VREG,	 bq->init_data.vreg},
		{F_ITERM,	 bq->init_data.iterm},
		{F_IPRECHG,	 bq->init_data.iprechg},
		{F_SYSVMIN,	 bq->init_data.sysvmin},
		{F_BOOSTV,	 bq->init_data.boostv},
		{F_BOOSTI,	 bq->init_data.boosti},
		{F_BOOSTF,	 bq->init_data.boostf},
		{F_EN_ILIM,	 bq->init_data.ilim_en},
		{F_TREG,	 bq->init_data.treg}
	};

	ret = bq25890_chip_reset(bq);
	if (ret < 0) {
		dev_err(bq->dev, "chip reset failed(%d)\n", ret);
		return ret;
	}

	/* disable watchdog */
	ret = bq25890_field_write(bq, F_WD, 0);
	if (ret < 0) {
		dev_err(bq->dev, "WDT disable failed(%d)\n", ret);
		return ret;
	}

	/* initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = bq25890_field_write(bq, init_data[i].id,
					  init_data[i].value);
		if (ret < 0) {
			dev_err(bq->dev, "init settings failed(%d)\n", ret);
			return ret;
		}
	}

	/* Configure ADC for continuous conversions. This does not enable it. */
	ret = bq25890_field_write(bq, F_CONV_RATE, 1);
	if (ret < 0) {
		dev_err(bq->dev, "ADC enabling failed(%d)\n", ret);
		return ret;
	}

	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0) {
		dev_err(bq->dev, "get chip state failed(%d)\n", ret);
		return ret;
	}

	mutex_lock(&bq->lock);
	memcpy(&bq->state, &state, sizeof(struct bq25890_state));
	mutex_unlock(&bq->lock);

	return 0;
}

static void bq25890_charging_refresh_work(struct work_struct *data)
{
	int ret;
	struct bq25890_device *bq = container_of(data,
			struct bq25890_device, chg_refresh_work.work);
	struct bq25890_state state;

	/* Set up charging */
	bq25890_setup_charging(bq);

	/*
	 * Delay of ~350mS is needed to see the
	 * effect of charger register updates.
	 */
	mdelay(350);
	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0) {
		dev_err(&bq->client->dev, "get status failed\n");
	} else {
		mutex_lock(&bq->lock);
		memcpy(&bq->state, &state, sizeof(struct bq25890_state));
		mutex_unlock(&bq->lock);
	}

	power_supply_changed(bq->charger);
}

static int bq25890_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq25890_device *bq = power_supply_get_drvdata(psy);
	struct bq25890_state state;

	mutex_lock(&bq->lock);
	memcpy(&state, &bq->state, sizeof(struct bq25890_state));
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bq->batt_full)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = bq25890_get_charging_status(bq, &state);
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25890_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (state.online || bq->cable.connected) &&
							!bq->cable.boost;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq25890_get_charger_health(bq, &state);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (!state.online || bq->cable.boost) {
			val->intval = 0;
			break;
		}

		val->intval = bq->inlmt;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (!state.online || bq->cable.boost) {
			val->intval = 0;
			break;
		}

		val->intval = bq->cc;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq25890_tables[TBL_ICHG].rt.max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		if (!state.online || bq->cable.boost) {
			val->intval = 0;
			break;
		}

		val->intval = bq->cv;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq25890_tables[TBL_VREG].rt.max;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq25890_find_val(bq->init_data.iterm, TBL_ITERM);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = bq->chg_cntl_lmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = bq->chg_cntl_max;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25890_power_supply_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct bq25890_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&bq->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		dev_dbg(bq->dev, "set ILIM current to %d\n", val->intval);

		bq->inlmt = val->intval;
		/*
		 * delay the charger settings to settle
		 * VBUS transitions for PD chargers or
		 * high voltage adapters.
		 */
		if (val->intval)
			schedule_delayed_work(&bq->chg_refresh_work, HZ);
		else
			bq25890_set_ilim(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (val->intval < 0 || val->intval >= bq->chg_cntl_max) {
			dev_warn(bq->dev,
				"invalid charge control limit\n");
			ret = -EINVAL;
			break;
		} else if (val->intval == (bq->chg_cntl_max - 1)) {
			dev_dbg(bq->dev, "move charger to High-Z mode\n");
			/*
			 * move the charger into High-Z mode by
			 * setting the input current limit to 0.
			 */
			bq25890_set_ilim(bq, 0);
		} else {
			bq->cc = bq->pdata->def_cc - (val->intval * 100000);
			bq25890_set_ilim(bq, bq->inlmt);
			bq25890_set_cc(bq, bq->cc);
		}
		bq->chg_cntl_lmt = val->intval;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&bq->lock);

	return ret;
}

static int bq25890_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_property bq25890_power_supply_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

static char *bq25890_charger_supplied_to[] = {
	"main-battery",
};

static struct power_supply_desc bq25890_power_supply_desc = {
	.name = "bq25890-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bq25890_power_supply_props,
	.num_properties = ARRAY_SIZE(bq25890_power_supply_props),
	.get_property = bq25890_power_supply_get_property,
	.set_property = bq25890_power_supply_set_property,
	.property_is_writeable = bq25890_property_is_writeable,
};

static struct power_supply *bq25890_get_fg_psy(struct bq25890_device *bq)
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
			dev_info(&bq->client->dev, "fuel gauge psy found\n");
			break;
		}
	}
	class_dev_iter_exit(&iter);

	return fg_psy;
}

static int bq25890_get_batt_ocv(struct bq25890_device *bq)
{
	int vocv;
	struct power_supply *psy;
	union power_supply_propval val;

	psy = bq25890_get_fg_psy(bq);
	if (!psy) {
		dev_warn(&bq->client->dev, "fuel gauge psy not found\n");
		return 0;
	}

	vocv = psy->desc->get_property(psy,
			POWER_SUPPLY_PROP_VOLTAGE_OCV, &val);
	if (vocv < 0) {
		dev_warn(&bq->client->dev, "get batt vocv error(%d)\n", vocv);
		return 0;
	}

	vocv = val.intval;
	return vocv;
}

static void bq25890_thermal_event_work(struct work_struct *work)
{
	struct bq25890_device *bq = container_of(work,
			struct bq25890_device, thermal_work.work);
	int chrg_status, ret, vocv;
	struct charging_algo_request req;
	struct charging_algo_params param;

	/* Get charging status */
	chrg_status = bq25890_get_charging_status(bq, &bq->state);

	/* Check battery recharge threshold */
	if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
		bq->batt_full = true;
	} else if (bq->batt_full) {
		/* Get battery open circut voltage */
		vocv = bq25890_get_batt_ocv(bq);
		if (vocv < (bq->cv - bq->vrechg_delta))
			bq->batt_full = false;
	}

	if (bq->batt_full)
		req.charging_status = POWER_SUPPLY_STATUS_FULL;
	else
		req.charging_status = chrg_status;
	req.supplied_to = BQ25890_BATT_MODEL;

	ret = charging_algo_get_params(&req, &param);
	if (ret < 0) {
		dev_warn(&bq->client->dev, "charging algo error(%d)\n", ret);
		return;
	}

	if (bq->cc == param.charge_current &&
		bq->cv == param.charge_voltage)
		goto twork_end;

	bq->cc = param.charge_current;
	bq->cv = param.charge_voltage;
	bq->vrechg_delta = param.vrechg_delta;

	/* refresh charging parameters */
	schedule_delayed_work(&bq->chg_refresh_work, 0);
	dev_dbg(&bq->client->dev,
		"[chg_algo] cc(%d), cv(%d)\n", bq->cc, bq->cv);

twork_end:
	if (bq->cable.connected && !bq->cable.boost)
		schedule_delayed_work(&bq->thermal_work, HZ * 60);
}

void bq25890_set_vboost_control(bool en)
{
	if (!info_ptr)
		return;

	if (en) {
		dev_dbg(&info_ptr->client->dev, "enable reverse boost\n");
		bq25890_field_write(info_ptr, F_OTG_CFG, 1);
		info_ptr->cable.boost = true;
	} else {
		dev_dbg(&info_ptr->client->dev, "disable reverse boost\n");
		bq25890_field_write(info_ptr, F_OTG_CFG, 0);
		info_ptr->cable.boost = false;
	}
}
EXPORT_SYMBOL(bq25890_set_vboost_control);

static void bq25890_extcon_event_work(struct work_struct *work)
{
	struct bq25890_device *chip =
			container_of(work, struct bq25890_device, cable.work);
	int ret, current_limit = 0;
	struct bq25890_state state;
	struct typec_pd_profile profile;
	bool legacy_cable;
	bool old_connected = chip->cable.connected;

	dev_dbg(&chip->client->dev, "In %s\n", __func__);

	/* determine cable/charger type */
	if (extcon_get_cable_state(chip->cable.typec.edev,
					"USB_TYPEC_SNK") > 0) {
		memset(&profile, 0x0, sizeof(struct typec_pd_profile));
		ret = typec_get_pd_profile(&profile);
		if (ret < 0) {
			dev_warn(&chip->client->dev, "get pd profile error\n");
			return;
		}

		/*
		 * Check any other charger cable is connected and
		 * it's current limit is more than TYPEC_CURRENT_USB.
		 */
		if ((extcon_get_cable_state(chip->cable.sdp.edev,
					"SLOW-CHARGER") > 0) ||
			(extcon_get_cable_state(chip->cable.cdp.edev,
                                        "CHARGE-DOWNSTREAM") > 0) ||
			(extcon_get_cable_state(chip->cable.dcp.edev,
                                        "FAST-CHARGER") > 0))
			legacy_cable = true;
		else
			legacy_cable = false;

		if (((profile.max_current / 1000) > TYPEC_CURRENT_USB) ||
			(profile.max_current && !legacy_cable)) {
			current_limit = profile.max_current;
			chip->cable.connected = true;
			chip->cable.chg_type = POWER_SUPPLY_TYPE_USB_TYPEC;
			dev_dbg(&chip->client->dev, "Type-C cable event\n");
			goto setup_charge_params;
		}
	}

	if (extcon_get_cable_state(chip->cable.sdp.edev,
					"SLOW-CHARGER") > 0) {
		chip->cable.connected = true;
		current_limit = BQ25890_SDP_ILIM;
		chip->cable.chg_type = POWER_SUPPLY_TYPE_USB;
		dev_dbg(&chip->client->dev, "USB SDP charger is connected");
	} else if (extcon_get_cable_state(chip->cable.cdp.edev,
					"CHARGE-DOWNSTREAM") > 0) {
		chip->cable.connected = true;
		current_limit = BQ25890_CDP_ILIM;
		chip->cable.chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		dev_dbg(&chip->client->dev, "USB CDP charger is connected");
	} else if (extcon_get_cable_state(chip->cable.dcp.edev,
					"FAST-CHARGER") > 0) {
		chip->cable.connected = true;
		current_limit = BQ25890_DCP_ILIM;
		chip->cable.chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		dev_dbg(&chip->client->dev, "USB DCP charger is connected");
	} else if (extcon_get_cable_state(chip->cable.otg.edev,
					"USB_TYPEC_SRC") > 0) {
		chip->cable.boost = true;
		chip->cable.connected = true;
		dev_dbg(&chip->client->dev, "USB_TYPEC_SRC cable is connected");
	} else {
		if (old_connected)
			dev_dbg(&chip->client->dev, "USB Cable disconnected");
		chip->batt_full = false;
		chip->cable.connected = false;
		chip->cable.boost = false;
		chip->cable.chg_type = POWER_SUPPLY_TYPE_USB;
	}

setup_charge_params:
	mutex_lock(&chip->lock);
	chip->inlmt = current_limit;
	bq25890_power_supply_desc.type = chip->cable.chg_type;
	mutex_unlock(&chip->lock);

	if (chip->cable.connected && !chip->cable.boost) {
		/* set up charging */
		bq25890_setup_charging(chip);
		schedule_delayed_work(&chip->thermal_work, 0);
	} else if (chip->cable.connected && chip->cable.boost) {
		/* enable boost mode */
		ret = bq25890_field_write(chip, F_OTG_CFG, 1);
		if (ret < 0)
			dev_warn(&chip->client->dev,
				"Boost mode enable failed(%d)\n", ret);
	} else {
		/* disable charging */
		bq25890_enable_charging(chip, false);
	}

	/*
	 * Delay of ~300mS is needed to see the
	 * effect of charger register updates.
	 */
	mdelay(350);

	ret = bq25890_get_chip_state(chip, &state);
	if (ret < 0) {
		dev_err(&chip->client->dev, "get status failed\n");
	} else {
		mutex_lock(&chip->lock);
		memcpy(&chip->state, &state, sizeof(struct bq25890_state));
		mutex_unlock(&chip->lock);
	}

	power_supply_changed(chip->charger);
}

static int bq25890_handle_extcon_events(struct notifier_block *nb,
				   unsigned long event, void *param)
{
	struct bq25890_device *chip =
		container_of(nb, struct bq25890_device, cable.nb);

	dev_dbg(&chip->client->dev, "external connector event(%ld)\n", event);

	schedule_work(&chip->cable.work);
	return NOTIFY_OK;
}

static int bq25890_extcon_register(struct bq25890_device *chip)
{
	int ret;

	INIT_WORK(&chip->cable.work, bq25890_extcon_event_work);
	chip->cable.nb.notifier_call = bq25890_handle_extcon_events;

	ret = extcon_register_interest(&chip->cable.sdp, NULL,
				"SLOW-CHARGER", &chip->cable.nb);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"extcon SLOW-CHARGER registration failed(%d)\n", ret);
		goto sdp_reg_failed;
	}

	ret = extcon_register_interest(&chip->cable.cdp, NULL,
				"CDP", &chip->cable.nb);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
				"extcon CDP registration failed(%d)\n", ret);
		goto cdp_reg_failed;
	}

	ret = extcon_register_interest(&chip->cable.dcp, NULL,
				"FAST-CHARGER", &chip->cable.nb);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"extcon FAST-CHARGER registration failed(%d)\n", ret);
		goto dcp_reg_failed;
	}

	ret = extcon_register_interest(&chip->cable.otg, NULL,
				"TYPEC-SRC", &chip->cable.nb);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"extcon TYPEC-SRC registration failed(%d)\n", ret);
		goto otg_reg_failed;
	}

	ret = extcon_register_interest(&chip->cable.typec, NULL,
				"TYPEC-SNK", &chip->cable.nb);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"extcon TYPEC-SNK registration failed(%d)\n", ret);
		goto typec_reg_failed;
	}

	return 0;

typec_reg_failed:
	extcon_unregister_interest(&chip->cable.otg);
otg_reg_failed:
	extcon_unregister_interest(&chip->cable.dcp);
dcp_reg_failed:
	extcon_unregister_interest(&chip->cable.cdp);
cdp_reg_failed:
	extcon_unregister_interest(&chip->cable.sdp);
sdp_reg_failed:
	return -EPROBE_DEFER;
}

static void bq25890_usb_work(struct work_struct *data)
{
	int ret;
	struct bq25890_device *bq =
			container_of(data, struct bq25890_device, usb_work);

	switch (bq->usb_event) {
	case USB_EVENT_ID:
		/* Enable boost mode */
		ret = bq25890_field_write(bq, F_OTG_CFG, 1);
		if (ret < 0)
			goto error;
		break;

	case USB_EVENT_NONE:
		/* Disable boost mode */
		ret = bq25890_field_write(bq, F_OTG_CFG, 0);
		if (ret < 0)
			goto error;

		power_supply_changed(bq->charger);
		break;
	}

	return;

error:
	dev_err(bq->dev, "Error switching to boost/charger mode.\n");
}

static int bq25890_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct bq25890_device *bq =
			container_of(nb, struct bq25890_device, usb_nb);

	bq->usb_event = val;
	queue_work(system_power_efficient_wq, &bq->usb_work);

	return NOTIFY_OK;
}

static void bq25890_init_worker(struct work_struct *data)
{
	struct bq25890_device *bq =
			container_of(data, struct bq25890_device, init_work);
	int ret;

	ret = bq25890_hw_init(bq);
	if (ret < 0)
		dev_warn(bq->dev, "Cannot initialize the chip\n");

	/* Check for the charger connected boot */
	schedule_work(&bq->cable.work);
}

static int bq25890_power_supply_init(struct bq25890_device *bq)
{
	struct power_supply_config psy_cfg = { .drv_data = bq, };

	psy_cfg.supplied_to = bq25890_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25890_charger_supplied_to);

	bq->charger = power_supply_register(bq->dev, &bq25890_power_supply_desc,
					    &psy_cfg);

	return PTR_ERR_OR_ZERO(bq->charger);
}

static int bq25890_irq_probe(struct bq25890_device *bq)
{
	int ret;
	struct gpio_desc *irq;

	irq = devm_gpiod_get_index(bq->dev, BQ25890_IRQ_PIN, 0, GPIOD_IN);
	if (IS_ERR(irq)) {
		dev_err(bq->dev, "Could not probe irq pin.\n");
		return PTR_ERR(irq);
	}

	ret = gpiod_direction_input(irq);
	if (ret < 0)
		return ret;

	return gpiod_to_irq(irq);
}

#ifdef CONFIG_OF
static int bq25890_fw_read_u32_props(struct bq25890_device *bq)
{
	int ret;
	u32 property;
	int i;
	struct bq25890_init_data *init = &bq->init_data;
	struct {
		char *name;
		bool optional;
		enum bq25890_table_ids tbl_id;
		u8 *conv_data; /* holds converted value from given property */
	} props[] = {
		/* required properties */
		{"ti,charge-current", false, TBL_ICHG, &init->ichg},
		{"ti,battery-regulation-voltage", false, TBL_VREG, &init->vreg},
		{"ti,termination-current", false, TBL_ITERM, &init->iterm},
		{"ti,precharge-current", false, TBL_ITERM, &init->iprechg},
		{"ti,minimum-sys-voltage", false, TBL_SYSVMIN, &init->sysvmin},
		{"ti,boost-voltage", false, TBL_BOOSTV, &init->boostv},
		{"ti,boost-max-current", false, TBL_BOOSTI, &init->boosti},

		/* optional properties */
		{"ti,thermal-regulation-threshold", true, TBL_TREG, &init->treg}
	};

	/* initialize data for optional properties */
	init->treg = 3; /* 120 degrees Celsius */

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = device_property_read_u32(bq->dev, props[i].name,
					       &property);
		if (ret < 0) {
			if (props[i].optional)
				continue;

			return ret;
		}

		*props[i].conv_data = bq25890_find_idx(property,
						       props[i].tbl_id);
	}

	return 0;
}

static int bq25890_fw_probe(struct bq25890_device *bq)
{
	int ret;
	struct bq25890_init_data *init = &bq->init_data;

	ret = bq25890_fw_read_u32_props(bq);
	if (ret < 0)
		return ret;

	init->ilim_en = device_property_read_bool(bq->dev, "ti,use-ilim-pin");
	init->boostf = device_property_read_bool(bq->dev, "ti,boost-low-freq");

	return 0;
}
#else
static int bq25890_fw_probe(struct bq25890_device *bq)
{
	struct bq25890_init_data *init = &bq->init_data;

	/* initialize the BXT platform data */
	init->ichg = bq25890_find_idx(bq->pdata->def_cc, TBL_ICHG);
	init->vreg = bq25890_find_idx(bq->pdata->def_cv, TBL_VREG);
	init->iterm = bq25890_find_idx(bq->pdata->iterm, TBL_ITERM);
	init->iprechg = bq25890_find_idx(bq->pdata->iprechg, TBL_IPRECHG);
	init->sysvmin = bq25890_find_idx(bq->pdata->sysvmin, TBL_SYSVMIN);
	init->boostv = bq25890_find_idx(bq->pdata->boostv, TBL_BOOSTV);
	init->boosti = bq25890_find_idx(bq->pdata->boosti, TBL_BOOSTI);
	init->treg = bq25890_find_idx(bq->pdata->max_temp, TBL_TREG);
	init->boostf = bq->pdata->boostf_low;
	init->ilim_en = bq->pdata->en_ilim_pin;

	bq->cc = bq->pdata->def_cc;
	bq->cv = bq->pdata->def_cv;
	/*
	 * divide the limit into 100mA steps so that
	 * total available steps will n + 2 including
	 * zero and High-Z mode.
	 */
	bq->chg_cntl_max = (bq->pdata->def_cc / 100000) + 2;

	return 0;
}
#endif

static int bq25890_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct bq25890_device *bq;
	int ret;
	int i;

	dev_info(dev, "In %s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);
	INIT_WORK(&bq->init_work, bq25890_init_worker);
	INIT_DELAYED_WORK(&bq->chg_refresh_work, bq25890_charging_refresh_work);
	INIT_DELAYED_WORK(&bq->thermal_work, bq25890_thermal_event_work);

	bq->rmap = devm_regmap_init_i2c(client, &bq25890_regmap_config);
	if (IS_ERR(bq->rmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(bq->rmap);
	}

	for (i = 0; i < ARRAY_SIZE(bq25890_reg_fields); i++) {
		const struct reg_field *reg_fields = bq25890_reg_fields;

		bq->rmap_fields[i] = devm_regmap_field_alloc(dev, bq->rmap,
							     reg_fields[i]);
		if (IS_ERR(bq->rmap_fields[i])) {
			dev_err(dev, "cannot allocate regmap field\n");
			return PTR_ERR(bq->rmap_fields[i]);
		}
	}

	i2c_set_clientdata(client, bq);

	bq->chip_id = bq25890_field_read(bq, F_PN);
	if (bq->chip_id < 0) {
		dev_err(dev, "Cannot read chip ID.\n");
		return bq->chip_id;
	}

	if ((bq->chip_id != BQ25890_ID) && (bq->chip_id != BQ25892_ID)) {
		dev_err(dev, "Chip with ID=%d, not supported!\n", bq->chip_id);
		return -ENODEV;
	}

	if (dev->platform_data)
		bq->pdata = dev->platform_data;
	else if (id)
		bq->pdata = (struct bq25890_platform_data *)id->driver_data;
	else
		bq->pdata = NULL;

	ret = bq25890_fw_probe(bq);
	if (ret < 0) {
		dev_err(dev, "Cannot read device properties.\n");
		return ret;
	}

	ret = bq25890_extcon_register(bq);
	if (ret < 0) {
		dev_warn(dev, "Failed to register extcon notifications\n");
		return ret;
	}

	if (client->irq <= 0)
		client->irq = bq25890_irq_probe(bq);

	if (client->irq < 0) {
		dev_err(dev, "No irq resource found.\n");
		ret = client->irq;
		goto irq_fail;
	}

	/* OTG reporting */
	bq->usb_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(bq->usb_phy)) {
		INIT_WORK(&bq->usb_work, bq25890_usb_work);
		bq->usb_nb.notifier_call = bq25890_usb_notifier;
		usb_register_notifier(bq->usb_phy, &bq->usb_nb);
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					bq25890_irq_handler_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					BQ25890_IRQ_PIN, bq);
	if (ret)
		goto irq_fail;

	ret = bq25890_power_supply_init(bq);
	if (ret < 0) {
		dev_err(dev, "Failed to register power supply\n");
		goto irq_fail;
	}
	/*
	 * Schedule the init worker to set the charging parameters
	 * and check for charger connecetd boot case.
	 */
	schedule_work(&bq->init_work);
	info_ptr = bq;
	dev_info(dev, "%s loaded successfully\n", client->name);

	return 0;

irq_fail:
	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);
	extcon_unregister_interest(&bq->cable.sdp);
	extcon_unregister_interest(&bq->cable.cdp);
	extcon_unregister_interest(&bq->cable.dcp);
	extcon_unregister_interest(&bq->cable.otg);
	extcon_unregister_interest(&bq->cable.typec);

	return ret;
}

static int bq25890_remove(struct i2c_client *client)
{
	struct bq25890_device *bq = i2c_get_clientdata(client);

	power_supply_unregister(bq->charger);

	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);

	extcon_unregister_interest(&bq->cable.sdp);
	extcon_unregister_interest(&bq->cable.cdp);
	extcon_unregister_interest(&bq->cable.dcp);
	extcon_unregister_interest(&bq->cable.otg);
	extcon_unregister_interest(&bq->cable.typec);

	/* reset all registers to default values */
	bq25890_chip_reset(bq);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bq25890_suspend(struct device *dev)
{
	struct bq25890_device *bq = dev_get_drvdata(dev);

	/*
	 * If charger is removed, while in suspend, make sure ADC is diabled
	 * since it consumes slightly more power.
	 */
	return bq25890_field_write(bq, F_CONV_START, 0);
}

static int bq25890_resume(struct device *dev)
{
	int ret;
	struct bq25890_state state;
	struct bq25890_device *bq = dev_get_drvdata(dev);

	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->lock);
	memcpy(&bq->state, &state, sizeof(struct bq25890_state));
	mutex_unlock(&bq->lock);

	/* Re-enable ADC only if charger is plugged in. */
	if (state.online) {
		ret = bq25890_field_write(bq, F_CONV_START, 1);
		if (ret < 0)
			return ret;
	}

	/* signal userspace, maybe state changed while suspended */
	power_supply_changed(bq->charger);

	return 0;
}
#endif

static const struct dev_pm_ops bq25890_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(bq25890_suspend, bq25890_resume)
};

static const struct i2c_device_id bq25890_i2c_ids[] = {
	{ "bq25890", 0 },
	{ "bxt_wcove_charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25890_i2c_ids);

static const struct of_device_id bq25890_of_match[] = {
	{ .compatible = "ti,bq25890", },
	{ },
};
MODULE_DEVICE_TABLE(of, bq25890_of_match);

static const struct acpi_device_id bq25890_acpi_match[] = {
	{"BQ258900", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, bq25890_acpi_match);

static struct i2c_driver bq25890_driver = {
	.driver = {
		.name = "bq25890-charger",
		.of_match_table = of_match_ptr(bq25890_of_match),
		.acpi_match_table = ACPI_PTR(bq25890_acpi_match),
		.pm = &bq25890_pm,
	},
	.probe = bq25890_probe,
	.remove = bq25890_remove,
	.id_table = bq25890_i2c_ids,
};
module_i2c_driver(bq25890_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@intel.com>");
MODULE_DESCRIPTION("bq25890 charger driver");
MODULE_LICENSE("GPL");
