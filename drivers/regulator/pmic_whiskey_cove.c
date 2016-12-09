/*
 * pmic_whiskey_cove.c - CherryTrail/BXT regulator driver
 *
 * Copyright (c) 2014, Intel Corporation.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/regulator/intel_whiskey_cove_pmic.h>

/* voltage control regulator offsets */
#define WCOVE_PLTSEL_REG		0x4FFE

/* CHT specific definitions */
/* buck boost regulators */
#define WCOVE_CHT_V3P3A_CTRL		0x6e5e
/* buck regulators */
#define WCOVE_CHT_V1P8A_CTRL		0x6e56
#define WCOVE_CHT_V1P05A_CTRL		0x123b
#define WCOVE_CHT_V1P15_CTRL		0x123c
#define WCOVE_CHT_VDDQ_CTRL		0x6e58
/* boot regulators */
/* ldo regulators */
#define WCOVE_CHT_VPROG1A_CTRL		0x6e90
#define WCOVE_CHT_VPROG1B_CTRL		0x6e91
#define WCOVE_CHT_VPROG1F_CTRL		0x6e95
#define WCOVE_CHT_V1P8SX_CTRL		0x6e57
#define WCOVE_CHT_V1P2A_CTRL		0x6e59
#define WCOVE_CHT_V1P2SX_CTRL		0x6e5a
#define WCOVE_CHT_VSDIO_CTRL		0x6e67
#define WCOVE_CHT_V2P8SX_CTRL		0x6e5d
#define WCOVE_CHT_V3P3SD_CTRL		0x6e5f
#define WCOVE_CHT_VPROG2D_CTRL		0x6e99
#define WCOVE_CHT_VPROG3A_CTRL		0x6e9a
#define WCOVE_CHT_VPROG3B_CTRL		0x6e9b
#define WCOVE_CHT_VPROG4A_CTRL		0x6e9c
#define WCOVE_CHT_VPROG4B_CTRL		0x6e9d
#define WCOVE_CHT_VPROG4C_CTRL		0x6e9e
#define WCOVE_CHT_VPROG4D_CTRL		0x6e9f
#define WCOVE_CHT_VPROG5A_CTRL		0x6ea0
#define WCOVE_CHT_VPROG5B_CTRL		0x6ea1
#define WCOVE_CHT_VPROG6A_CTRL		0x6ea2
#define WCOVE_CHT_VPROG6B_CTRL		0x6ea3


/* voltage selector regulator offsets */

/* buck boost regulators */
#define WCOVE_CHT_V3P3A_VSEL		0x6e68
/* buck regulators */
#define WCOVE_CHT_V1P8A_VSEL		0x6e5b
#define WCOVE_CHT_V1P05A_VSEL		0x123d
#define WCOVE_CHT_V1P15_VSEL		0x123e
#define WCOVE_CHT_VDDQ_VSEL		0x6e5c
/* boot regulators */
/* ldo regulators */
#define WCOVE_CHT_VPROG1A_VSEL		0x6ec0
#define WCOVE_CHT_VPROG1B_VSEL		0x6ec1
#define WCOVE_CHT_V1P8SX_VSEL		0x6ec2
#define WCOVE_CHT_V1P2SX_VSEL		0x6ec3
#define WCOVE_CHT_V1P2A_VSEL		0x6ec4
#define WCOVE_CHT_VPROG1F_VSEL		0x6ec5
#define WCOVE_CHT_VSDIO_VSEL		0x6ec6
#define WCOVE_CHT_V2P8SX_VSEL		0x6ec7
#define WCOVE_CHT_V3P3SD_VSEL		0x6ec8
#define WCOVE_CHT_VPROG2D_VSEL		0x6ec9
#define WCOVE_CHT_VPROG3A_VSEL		0x6eca
#define WCOVE_CHT_VPROG3B_VSEL		0x6ecb
#define WCOVE_CHT_VPROG4A_VSEL		0x6ecc
#define WCOVE_CHT_VPROG4B_VSEL		0x6ecd
#define WCOVE_CHT_VPROG4C_VSEL		0x6ece
#define WCOVE_CHT_VPROG4D_VSEL		0x6ecf
#define WCOVE_CHT_VPROG5A_VSEL		0x6ed0
#define WCOVE_CHT_VPROG5B_VSEL		0x6ed1
#define WCOVE_CHT_VPROG6A_VSEL		0x6ed2
#define WCOVE_CHT_VPROG6B_VSEL		0x6ed3


/* number of voltage variations exposed */

/* buck boost regulators */
#define WCOVE_CHT_V3P3A_VRANGE		8
/* buck regulators */
#define WCOVE_CHT_V1P8A_VRANGE		256
#define WCOVE_CHT_V1P05A_VRANGE		256
#define WCOVE_CHT_VDDQ_VRANGE		120
/* boot regulators */
/* ldo regulators */
#define WCOVE_CHT_VPROG1A_VRANGE	53
#define WCOVE_CHT_VPROG1B_VRANGE	53
#define WCOVE_CHT_VPROG1F_VRANGE	53
#define WCOVE_CHT_VPROG2D_VRANGE	53
#define WCOVE_CHT_VPROG3A_VRANGE	53
#define WCOVE_CHT_VPROG3B_VRANGE	53
#define WCOVE_CHT_VPROG4A_VRANGE	53
#define WCOVE_CHT_VPROG4B_VRANGE	53
#define WCOVE_CHT_VPROG4C_VRANGE	53
#define WCOVE_CHT_VPROG4D_VRANGE	53
#define WCOVE_CHT_VPROG5A_VRANGE	53
#define WCOVE_CHT_VPROG5B_VRANGE	53
#define WCOVE_CHT_VPROG6A_VRANGE	53
#define WCOVE_CHT_VPROG6B_VRANGE	53
#define WCOVE_CHT_V1P8SX_VRANGE		53
#define WCOVE_CHT_V1P2SX_VRANGE		53
#define WCOVE_CHT_V1P2A_VRANGE		53
#define WCOVE_CHT_VSDIO_VRANGE		53
#define WCOVE_CHT_V2P8SX_VRANGE		53
#define WCOVE_CHT_V3P3SD_VRANGE		53

/* BXT Specific Definitions */
#define WCOVE_BXT_VFLEX_CTRL		0x4E6D
#define WCOVE_BXT_VPROG1A_CTRL		0x4E6F
#define WCOVE_BXT_VPROG1B_CTRL		0x4E70
#define WCOVE_BXT_VPROG1C_CTRL		0x4E71
#define WCOVE_BXT_VPROG1D_CTRL		0x4E72
#define WCOVE_BXT_VPROG2A_CTRL		0x4E73
#define WCOVE_BXT_VPROG2B_CTRL		0x4E74
#define WCOVE_BXT_VPROG2C_CTRL		0x4E75
#define WCOVE_BXT_VPROG3A_CTRL		0x4E76
#define WCOVE_BXT_VPROG3B_CTRL		0x4E77
#define WCOVE_BXT_VPROG1E_CTRL		0x4EA0
#define WCOVE_BXT_VPROG1F_CTRL		0x4EA1
#define WCOVE_BXT_VPROG2D_CTRL		0x4EA2
#define WCOVE_BXT_VPROG4A_CTRL		0x4EA3
#define WCOVE_BXT_VPROG4B_CTRL		0x4EA4
#define WCOVE_BXT_VPROG4C_CTRL		0x4EA5
#define WCOVE_BXT_VPROG4D_CTRL		0x4EA6
#define WCOVE_BXT_VPROG5A_CTRL		0x4EA7
#define WCOVE_BXT_VPROG5B_CTRL		0x4EA8
#define WCOVE_BXT_VPROG6A_CTRL		0x4EA9
#define WCOVE_BXT_VPROG6B_CTRL		0x4EAA

#define WCOVE_BXT_VFLEX_VSEL		0x4E6E
#define WCOVE_BXT_VPROG1A_VSEL		0x4EB6
#define WCOVE_BXT_VPROG1B_VSEL		0x4EB7
#define WCOVE_BXT_VPROG1C_VSEL		0x4EB8
#define WCOVE_BXT_VPROG1D_VSEL		0x4EB9
#define WCOVE_BXT_VPROG1E_VSEL		0x4EBA
#define WCOVE_BXT_VPROG1F_VSEL		0x4EBB
#define WCOVE_BXT_VPROG2A_VSEL		0x4EBC
#define WCOVE_BXT_VPROG2B_VSEL		0x4EBD
#define WCOVE_BXT_VPROG2C_VSEL		0x4EBE
#define WCOVE_BXT_VPROG2D_VSEL		0x4EBF
#define WCOVE_BXT_VPROG3A_VSEL		0x4EC0
#define WCOVE_BXT_VPROG3B_VSEL		0x4EC1
#define WCOVE_BXT_VPROG4A_VSEL		0x4EC2
#define WCOVE_BXT_VPROG4B_VSEL		0x4EC3
#define WCOVE_BXT_VPROG4C_VSEL		0x4EC4
#define WCOVE_BXT_VPROG4D_VSEL		0x4EC5
#define WCOVE_BXT_VPROG5A_VSEL		0x4EC6
#define WCOVE_BXT_VPROG5B_VSEL		0x4EC7
#define WCOVE_BXT_VPROG6A_VSEL		0x4EC8
#define WCOVE_BXT_VPROG6B_VSEL		0x4EC9


#define WCOVE_BXT_VFLEX_VRANGE		256
#define WCOVE_BXT_VPROG1A_VRANGE	53
#define WCOVE_BXT_VPROG1B_VRANGE	53
#define WCOVE_BXT_VPROG1C_VRANGE	53
#define WCOVE_BXT_VPROG1D_VRANGE	53
#define WCOVE_BXT_VPROG1E_VRANGE	53
#define WCOVE_BXT_VPROG1F_VRANGE	53
#define WCOVE_BXT_VPROG2A_VRANGE	53
#define WCOVE_BXT_VPROG2B_VRANGE	53
#define WCOVE_BXT_VPROG2C_VRANGE	53
#define WCOVE_BXT_VPROG2D_VRANGE	53
#define WCOVE_BXT_VPROG3A_VRANGE	53
#define WCOVE_BXT_VPROG3B_VRANGE	53
#define WCOVE_BXT_VPROG4A_VRANGE	53
#define WCOVE_BXT_VPROG4B_VRANGE	53
#define WCOVE_BXT_VPROG4C_VRANGE	53
#define WCOVE_BXT_VPROG4D_VRANGE	53
#define WCOVE_BXT_VPROG5A_VRANGE	53
#define WCOVE_BXT_VPROG5B_VRANGE	53
#define WCOVE_BXT_VPROG6A_VRANGE	53
#define WCOVE_BXT_VPROG6B_VRANGE	53

enum regulator_plt {
	REGULATOR_PLT_BXT,
	REGULATOR_PLT_CHT,
	UNUSED = -1
};

static enum regulator_plt  wc_regulator_plt = REGULATOR_PLT_BXT;

/* voltage tables */
static unsigned int WCOVE_CHT_V3P3A_VSEL_TABLE[WCOVE_CHT_V3P3A_VRANGE],
		    WCOVE_CHT_V1P8A_VSEL_TABLE[WCOVE_CHT_V1P8A_VRANGE],
		    WCOVE_CHT_V1P05A_VSEL_TABLE[WCOVE_CHT_V1P05A_VRANGE],
		    WCOVE_CHT_VDDQ_VSEL_TABLE[WCOVE_CHT_VDDQ_VRANGE],
		    WCOVE_CHT_V1P8SX_VSEL_TABLE[WCOVE_CHT_V1P8SX_VRANGE],
		    WCOVE_CHT_V1P2SX_VSEL_TABLE[WCOVE_CHT_V1P2SX_VRANGE],
		    WCOVE_CHT_V1P2A_VSEL_TABLE[WCOVE_CHT_V1P2A_VRANGE],
		    WCOVE_CHT_V2P8SX_VSEL_TABLE[WCOVE_CHT_V2P8SX_VRANGE],
		    WCOVE_CHT_V3P3SD_VSEL_TABLE[WCOVE_CHT_V3P3SD_VRANGE],
		    WCOVE_CHT_VPROG1A_VSEL_TABLE[WCOVE_CHT_VPROG1A_VRANGE],
		    WCOVE_CHT_VPROG1B_VSEL_TABLE[WCOVE_CHT_VPROG1B_VRANGE],
		    WCOVE_CHT_VPROG1F_VSEL_TABLE[WCOVE_CHT_VPROG1F_VRANGE],
		    WCOVE_CHT_VPROG2D_VSEL_TABLE[WCOVE_CHT_VPROG2D_VRANGE],
		    WCOVE_CHT_VPROG3A_VSEL_TABLE[WCOVE_CHT_VPROG3A_VRANGE],
		    WCOVE_CHT_VPROG3B_VSEL_TABLE[WCOVE_CHT_VPROG3B_VRANGE],
		    WCOVE_CHT_VPROG4A_VSEL_TABLE[WCOVE_CHT_VPROG4A_VRANGE],
		    WCOVE_CHT_VPROG4B_VSEL_TABLE[WCOVE_CHT_VPROG4B_VRANGE],
		    WCOVE_CHT_VPROG4C_VSEL_TABLE[WCOVE_CHT_VPROG4C_VRANGE],
		    WCOVE_CHT_VPROG4D_VSEL_TABLE[WCOVE_CHT_VPROG4D_VRANGE],
		    WCOVE_CHT_VPROG5A_VSEL_TABLE[WCOVE_CHT_VPROG5A_VRANGE],
		    WCOVE_CHT_VPROG5B_VSEL_TABLE[WCOVE_CHT_VPROG5B_VRANGE],
		    WCOVE_CHT_VPROG6A_VSEL_TABLE[WCOVE_CHT_VPROG6A_VRANGE],
		    WCOVE_CHT_VPROG6B_VSEL_TABLE[WCOVE_CHT_VPROG6B_VRANGE];


static unsigned int WCOVE_BXT_VFLEX_VSEL_TABLE[WCOVE_BXT_VFLEX_VRANGE],
		    WCOVE_BXT_VPROG1A_VSEL_TABLE[WCOVE_BXT_VPROG1A_VRANGE],
		    WCOVE_BXT_VPROG1B_VSEL_TABLE[WCOVE_BXT_VPROG1B_VRANGE],
		    WCOVE_BXT_VPROG1C_VSEL_TABLE[WCOVE_BXT_VPROG1C_VRANGE],
		    WCOVE_BXT_VPROG1D_VSEL_TABLE[WCOVE_BXT_VPROG1D_VRANGE],
		    WCOVE_BXT_VPROG1E_VSEL_TABLE[WCOVE_BXT_VPROG1E_VRANGE],
		    WCOVE_BXT_VPROG1F_VSEL_TABLE[WCOVE_BXT_VPROG1F_VRANGE],
		    WCOVE_BXT_VPROG2A_VSEL_TABLE[WCOVE_BXT_VPROG2A_VRANGE],
		    WCOVE_BXT_VPROG2B_VSEL_TABLE[WCOVE_BXT_VPROG2B_VRANGE],
		    WCOVE_BXT_VPROG2C_VSEL_TABLE[WCOVE_BXT_VPROG2C_VRANGE],
		    WCOVE_BXT_VPROG2D_VSEL_TABLE[WCOVE_BXT_VPROG2D_VRANGE],
		    WCOVE_BXT_VPROG3A_VSEL_TABLE[WCOVE_BXT_VPROG3A_VRANGE],
		    WCOVE_BXT_VPROG3B_VSEL_TABLE[WCOVE_BXT_VPROG3B_VRANGE],
		    WCOVE_BXT_VPROG4A_VSEL_TABLE[WCOVE_BXT_VPROG4A_VRANGE],
		    WCOVE_BXT_VPROG4B_VSEL_TABLE[WCOVE_BXT_VPROG4B_VRANGE],
		    WCOVE_BXT_VPROG4C_VSEL_TABLE[WCOVE_BXT_VPROG4C_VRANGE],
		    WCOVE_BXT_VPROG5A_VSEL_TABLE[WCOVE_BXT_VPROG5A_VRANGE],
		    WCOVE_BXT_VPROG5B_VSEL_TABLE[WCOVE_BXT_VPROG5B_VRANGE],
		    WCOVE_BXT_VPROG6A_VSEL_TABLE[WCOVE_BXT_VPROG6A_VRANGE],
		    WCOVE_BXT_VPROG6B_VSEL_TABLE[WCOVE_BXT_VPROG6B_VRANGE];

/*
 * The VSDIO regulator should only support 1.8V and 3.3V. All other
 * voltages are invalid for sd card, so disable them here.
 */
static unsigned int WCOVE_CHT_VSDIO_VSEL_TABLE[WCOVE_CHT_VSDIO_VRANGE] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1800000, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 3300000, 0, 0
};

/* VPROG4D used for BXT sd card. So only 1.8V and 3.3V support */
static unsigned int WCOVE_BXT_VPROG4D_VSEL_TABLE[WCOVE_BXT_VPROG4D_VRANGE] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1800000, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 3300000, 0, 0
};

static int wcove_regulator_enable(struct regulator_dev *rdev)
{
	struct wcove_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(pmic_info->regmap, pmic_info->vctl_reg, &reg_val);
	if (ret < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", ret);
		return ret;
	}

	reg_val &= ~pmic_info->vctl_mask;
	reg_val |= pmic_info->reg_enbl_mask;

	return regmap_write(pmic_info->regmap, pmic_info->vctl_reg, reg_val);
}

static int wcove_regulator_disable(struct regulator_dev *rdev)
{
	struct wcove_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(pmic_info->regmap, pmic_info->vctl_reg, &reg_val);
	if (ret < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", ret);
		return ret;
	}

	reg_val &= ~pmic_info->vctl_mask;
	reg_val |= pmic_info->reg_dsbl_mask;

	return regmap_write(pmic_info->regmap, pmic_info->vctl_reg, reg_val);
}

static int wcove_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct wcove_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(pmic_info->regmap, pmic_info->vctl_reg, &reg_val);
	if (ret < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", ret);
		return ret;
	}

	reg_val &= pmic_info->vctl_mask;

	return reg_val & pmic_info->reg_enbl_mask;
}

static int wcove_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct wcove_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	unsigned int reg_val;
	int ret, vsel;

	ret = regmap_read(pmic_info->regmap, pmic_info->vsel_reg, &reg_val);
	if (ret < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", ret);
		return ret;
	}

	vsel = (reg_val & pmic_info->vsel_mask) - pmic_info->start;

	return vsel;
}

static int wcove_regulator_set_voltage_sel(struct regulator_dev *rdev,
		unsigned selector)
{
	struct wcove_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(pmic_info->regmap, pmic_info->vsel_reg, &reg_val);
	if (ret < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", ret);
		return ret;
	}

	reg_val &= ~pmic_info->vsel_mask;
	reg_val |= (selector + pmic_info->start);

	return regmap_write(pmic_info->regmap, pmic_info->vsel_reg, reg_val);
}

/* regulator ops */
static struct regulator_ops wcove_regulator_ops = {
	.enable = wcove_regulator_enable,
	.disable = wcove_regulator_disable,
	.is_enabled = wcove_regulator_is_enabled,
	.get_voltage_sel = wcove_regulator_get_voltage_sel,
	.set_voltage_sel = wcove_regulator_set_voltage_sel,
	.list_voltage = regulator_list_voltage_table,
};

static struct regulator_consumer_supply sd_vmmc_consumer[] = {
	REGULATOR_SUPPLY("vmmc", "8086:1ACA"),
};

/* vsdcard regulator */
static struct regulator_init_data vmmc_init_data = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(sd_vmmc_consumer),
	.consumer_supplies	= sd_vmmc_consumer,
};

/*
 * define some BXT regulators. min/max voltages are taken from regulator
 * descriptions table. see initialize_vtable() function.
 *
 * Consumers are not known at wcove init phase. Those can be added
 * later on using regulator alias mechanism.
 */
static struct regulator_init_data vprog5a_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

static struct regulator_init_data vflex_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

static struct regulator_init_data vprog4b_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

static struct regulator_init_data vprog4c_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

static struct regulator_init_data vprog2c_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

static struct regulator_init_data vprog1c_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

static struct regulator_init_data vprog1e_init_data = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.keep_on_at_complete	= true,
	},
};

#define WCOVE_REG(_plt, _id, _init, minv, maxv, strt, vselmsk, vscale,\
					vctlmsk, enbl, dsbl, rt_flag)\
{\
	.desc = {\
		.name	= ""#_id,\
		.ops	= &wcove_regulator_ops,\
		.type	= REGULATOR_VOLTAGE,\
		.id	= WCOVE_ID_##_id,\
		.owner	= THIS_MODULE,\
	},\
	.regulator = NULL,\
	.init_data = (_init),\
	.vctl_reg	= WCOVE_##_plt##_##_id##_CTRL,\
	.vsel_reg	= WCOVE_##_plt##_##_id##_VSEL,\
	.min_mV		= (minv),\
	.max_mV		= (maxv),\
	.start		= (strt),\
	.vsel_mask	= (vselmsk),\
	.scale		= (vscale),\
	.nvolts		= WCOVE_##_plt##_##_id##_VRANGE,\
	.vctl_mask	= (vctlmsk),\
	.reg_enbl_mask	= (enbl),\
	.reg_dsbl_mask	= (dsbl),\
	.vtable		= WCOVE_##_plt##_##_id##_VSEL_TABLE,\
	.runtime_table	= (rt_flag),\
}

/* Regulator descriptions */
static struct wcove_regulator_info cht_regulators_info[] = {
	WCOVE_REG(CHT, V3P3A, NULL, 3000, 3350, 0x0, 0x07, 50,
					0x1, 0x1, 0x0, true),
	WCOVE_REG(CHT, V1P8A, NULL, 250, 2100, 0x0, 0xff, 10,
					0x1, 0x1, 0x0, true),
	WCOVE_REG(CHT, V1P05A, NULL, 250, 2100, 0x0, 0xff, 10,
					0x1, 0x1, 0x0, true),
	WCOVE_REG(CHT, VDDQ, NULL, 250, 1440, 0x0, 0x7f, 10,
					0x1, 0x1, 0x0, true),
	WCOVE_REG(CHT, V1P8SX, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(CHT, V1P2A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(CHT, V1P2SX, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(CHT, V2P8SX, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(CHT, VSDIO, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, false),
	WCOVE_REG(CHT, V3P3SD, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG1A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG1B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG1F, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG2D, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG3A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG3B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG4A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG4B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG4C, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG4D, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG5A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG5B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG6A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
	WCOVE_REG(CHT, VPROG6B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x01, 0x0, true),
};

/* Regulator descriptions */
static struct wcove_regulator_info bxt_regulators_info[] = {
	WCOVE_REG(BXT, VFLEX, &vflex_init_data, 250, 2100, 0x0, 0xff, 10,
					0x1, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG1A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG1B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG1C, &vprog1c_init_data, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG1D, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG1E, &vprog1e_init_data, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG1F, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG2A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG2B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG2C, &vprog2c_init_data, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG2D, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG3A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG3B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG4A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG4B, &vprog4b_init_data, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG4C, &vprog4c_init_data, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG4D, &vmmc_init_data, 800, 3400, 0x0b, 0x3f,
					50, 0x07, 0x1, 0x0, false),
	WCOVE_REG(BXT, VPROG5A, &vprog5a_init_data, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG5B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG6A, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
	WCOVE_REG(BXT, VPROG6B, NULL, 800, 3400, 0x0b, 0x3f, 50,
					0x07, 0x1, 0x0, true),
};

static inline struct wcove_regulator_info *wcove_find_regulator_info(int id)
{
	struct wcove_regulator_info *reg_info;
	int i;

	if (wc_regulator_plt == REGULATOR_PLT_BXT) {
		for (i = 0; i < ARRAY_SIZE(bxt_regulators_info); i++) {
			if (bxt_regulators_info[i].desc.id == id) {
				reg_info = &bxt_regulators_info[i];
				return reg_info;
			}
		}
	}

	else {
		for (i = 0; i < ARRAY_SIZE(cht_regulators_info); i++) {
			if (cht_regulators_info[i].desc.id == id) {
				reg_info = &cht_regulators_info[i];
				return reg_info;
			}
		}
	}

	return NULL;
}

static void initialize_vtable(struct wcove_regulator_info *reg_info)
{
	unsigned int i, volt;
	struct regulator_init_data *init_data = reg_info->init_data;

	if (reg_info->runtime_table == true) {
		for (i = 0; i < reg_info->nvolts; i++) {
			volt = reg_info->min_mV + (i * reg_info->scale);
			if (volt < reg_info->min_mV)
				volt = reg_info->min_mV;
			if (volt > reg_info->max_mV)
				volt = reg_info->max_mV;
			/* set value in uV */
			reg_info->vtable[i] = volt*1000;
		}
	}
	reg_info->desc.volt_table = reg_info->vtable;
	reg_info->desc.n_voltages = reg_info->nvolts;

	if (init_data &&
	    init_data->constraints.valid_ops_mask & REGULATOR_CHANGE_VOLTAGE) {
		init_data->constraints.min_uV = reg_info->min_mV * 1000;
		init_data->constraints.max_uV = reg_info->max_mV * 1000;
	}
}

/* BXT regulators list */
static int bxt_reg_list[] = {
	WCOVE_ID_VPROG4C,
	WCOVE_ID_VFLEX,
	WCOVE_ID_VPROG1C,
	WCOVE_ID_VPROG2C,
	WCOVE_ID_VPROG4B,
	WCOVE_ID_VPROG1E,
	WCOVE_ID_VPROG4D,
	WCOVE_ID_VPROG5A
};

/* BXT apecific initialization */
static int wcove_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct regulator_config config = { };
	struct wcove_regulator_info *reg_info = NULL;
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);
	int ret = 0;
	int i;

	/* Run through the list of BXTN regulator list and register them */
	for (i = 0; i < ARRAY_SIZE(bxt_reg_list); i++) {
		reg_info = wcove_find_regulator_info(bxt_reg_list[i]);
		if (reg_info == NULL) {
			dev_err(&pdev->dev,
				 "invalid regulator %d\n", bxt_reg_list[i]);
			ret = -EINVAL;
			continue;
		}

		config.init_data = reg_info->init_data;
		initialize_vtable(reg_info);
		reg_info->regmap = wcove->regmap;
		config.dev = &pdev->dev;
		config.driver_data = reg_info;

		rdev = regulator_register(&reg_info->desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				 "regl register err %s\n", reg_info->desc.name);
			ret = PTR_ERR(rdev);
			continue;
		}

		platform_set_drvdata(pdev, rdev);
		dev_dbg(&pdev->dev, "registered whiskey cove regulator as %s\n",
				dev_name(&rdev->dev));
	}

	return ret;
}

static int wcove_regulator_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id wcove_regulator_id_table[] = {
	{ "wcove_regulator", 0},
	{ },
};

MODULE_DEVICE_TABLE(platform, wcove_regulator_id_table);

static struct platform_driver wcove_regulator_driver = {
	.driver = {
		.name = "wcove_regulator",
		.owner = THIS_MODULE,
	},
	.probe = wcove_regulator_probe,
	.remove = wcove_regulator_remove,
	.id_table = wcove_regulator_id_table,
};

static int __init wcove_regulator_init(void)
{
	return platform_driver_register(&wcove_regulator_driver);
}
arch_initcall_sync(wcove_regulator_init);

static void __exit wcove_regulator_exit(void)
{
	platform_driver_unregister(&wcove_regulator_driver);
}
module_exit(wcove_regulator_exit);

MODULE_DESCRIPTION("WhiskeyCove regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:intel_regulator");
