/*
 * Register map access API - Intel PMIC IPC support
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 */

#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <asm/intel_pmc_ipc.h>
#include "internal.h"

#define I2C_ADDR_MASK		0xFF00
#define I2C_ADDR_SHIFT		8
#define I2C_REG_MASK		0xFF
#define I2COVRCTRL_REG		0x5E24

static int regmap_ipc_byte_reg_read(void *context, unsigned int reg,
				      unsigned int *val)
{
	int ret;
	int i2c_addr;
	u8 ipc_in[2];
	u8 ipc_out[4];
	struct intel_soc_pmic *pmic = context;

	if (reg & I2C_ADDR_MASK)
		i2c_addr = (reg & I2C_ADDR_MASK) >> I2C_ADDR_SHIFT;
	else {
		i2c_addr = pmic->default_i2c_addr;
		if (!i2c_addr) {
			dev_err(pmic->dev, "Need to specify i2c addr\n");
			return -EINVAL;
		}
	}
	reg &= I2C_REG_MASK;

	ipc_in[0] = reg;
	ipc_in[1] = i2c_addr;
	ret = intel_pmc_ipc_command(PMC_IPC_PMIC_ACCESS,
			PMC_IPC_PMIC_ACCESS_READ,
			ipc_in, sizeof(ipc_in), (u32 *)ipc_out, 1);
	if (ret) {
		dev_err(pmic->dev, "Err: ipc read pmic\n");
		return ret;
	}
	*val = ipc_out[0];
	return 0;
}

static int regmap_ipc_byte_reg_write(void *context, unsigned int reg,
				       unsigned int val)
{
	int ret;
	int i2c_addr;
	int reg_addr;
	u8 ipc_in[4];
	struct intel_soc_pmic *pmic = context;

	if (reg & I2C_ADDR_MASK)
		i2c_addr = (reg & I2C_ADDR_MASK) >> I2C_ADDR_SHIFT;
	else {
		i2c_addr = pmic->default_i2c_addr;
		if (!i2c_addr) {
			dev_err(pmic->dev, "Need to specify i2c addr\n");
			return -EINVAL;
		}
	}
	reg_addr = reg & I2C_REG_MASK;

	ipc_in[0] = reg_addr;
	ipc_in[1] = i2c_addr;
	ipc_in[2] = val;
	/*
	 * In case of charger IC read/write operation
	 * use R-M-W atomic command as per PMC FW FAS.
	 */
	if (reg == I2COVRCTRL_REG) {
		ipc_in[3] = val;
		ret = intel_pmc_ipc_command(PMC_IPC_PMIC_ACCESS,
				PMC_IPC_PMIC_ACCESS_RMW,
				ipc_in, sizeof(ipc_in), NULL, 0);
	} else {
		ret = intel_pmc_ipc_command(PMC_IPC_PMIC_ACCESS,
				PMC_IPC_PMIC_ACCESS_WRITE,
				ipc_in, 3, NULL, 0);
	}
	if (ret)
		dev_err(pmic->dev, "Err: ipc write pmic\n");

	return ret;
}

static struct regmap_bus ipc_regmap_bus = {
	.reg_write = regmap_ipc_byte_reg_write,
	.reg_read = regmap_ipc_byte_reg_read,
};

/**
 * regmap_init_pmic_ipc(): Initialise register map
 *
 * @pmic: Device that will be interacted with
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer to
 * a struct regmap.
 */
struct regmap *__regmap_init_pmic_ipc(struct intel_soc_pmic *pmic,
				      const struct regmap_config *config,
				      struct lock_class_key *lock_key,
				      const char *lock_name)
{
	return __regmap_init(pmic->dev, &ipc_regmap_bus, pmic, config,
			     lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__regmap_init_pmic_ipc);

/**
 * devm_regmap_init_pmic_ipc(): Initialise managed register map
 *
 * @pmic: Device that will be interacted with
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct regmap.  The regmap will be automatically freed by the
 * device management code.
 */
struct regmap *__devm_regmap_init_pmic_ipc(struct intel_soc_pmic *pmic,
					   const struct regmap_config *config,
					   struct lock_class_key *lock_key,
					   const char *lock_name)
{
	return __devm_regmap_init(pmic->dev, &ipc_regmap_bus, pmic, config,
				  lock_key, lock_name);
}
EXPORT_SYMBOL_GPL(__devm_regmap_init_pmic_ipc);

MODULE_AUTHOR("qipeng.zha@intel.com");
MODULE_LICENSE("GPL");
