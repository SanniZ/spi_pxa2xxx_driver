/*
 * i2c-wcove-pmic.c: Whiskey Cove PMIC I2C adapter driver.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Yegnesh Iyer <yegnesh.s.iyer@intel.com>
	Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/types.h>
#include <linux/acpi.h>
#include <linux/mfd/intel_soc_pmic.h>

#define D7 (1 << 7)
#define D6 (1 << 6)
#define D5 (1 << 5)
#define D4 (1 << 4)
#define D3 (1 << 3)
#define D2 (1 << 2)
#define D1 (1 << 1)
#define D0 (1 << 0)


#define I2C_MSG_LEN		4

#define CHRTTADDR_ADDR		0x5E22
#define CHRTTDATA_ADDR		0x5E23

#define I2COVRCTRL_ADDR		0x5E24
#define I2COVRDADDR_ADDR	0x5E25
#define I2COVROFFSET_ADDR	0x5E26
#define I2COVRWRDATA_ADDR	0x5E27
#define I2COVRRDDATA_ADDR	0x5E28

#define MCHGRIRQ0_ADDR		0x4E17

#define PMIC_I2C_INTR_MASK	(D3|D2|D1)
#define PMIC_CHGR_INTR_MASK	D0

#define I2COVRCTRL_I2C_RD	D1
#define I2COVRCTRL_I2C_WR	D0
#define CHGRIRQ0_ADDR		0x4E09

#define IRQ0_I2C_BIT_POS	1
#define I2C_REG_RW_MAX_TOUT	3

/* pmic charger info */
#define PMIC_CHG_DEF_CC			2600000	/* in uA */
#define PMIC_CHG_DEF_IPRECHG		256000	/* in uA */
#define PMIC_CHG_DEF_ITERM		128000	/* in uA */
#define PMIC_CHG_DEF_CV			4350000	/* in uV */
#define PMIC_CHG_DEF_VSYSMIN		3500000	/* in uV */
#define PMIC_CHG_DEF_BOOSTV		4700000	/* in uV */
#define PMIC_CHG_DEF_BOOSTI		500000	/* in uA */
#define PMIC_CHG_MAX_CC			2600000	/* in uA */
#define PMIC_CHG_MAX_CV			4350000	/* in uV */
#define PMIC_CHG_MAX_TEMP		100	/* in DegC */
#define PMIC_CHG_MIN_TEMP		0	/* in DegC */

enum wcove_pmic_i2c_irq {
	EXT_I2C_IRQ = 0,
	EXT_CHRG_IRQ,
	PMIC_I2C_IRQ_END,
};

struct charger_platform_data {
	int def_cc;	/* in uA */
	int def_cv;	/* in uV */
	int iterm;	/* in uA */
	int iprechg;	/* in uA */
	int sysvmin;	/* in uA */
	int boosti;	/* in uA */
	int boostv;	/* in uV */
	int max_cv;	/* in uV */
	int max_cc;	/* in uA */
	int min_temp;	/* in DegC */
	int max_temp;	/* in DegC */
	bool boostf_low;
	bool en_thermal_reg;
	bool en_ilim_pin;
};

struct pmic_i2c_dev {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq_chip;
	int irq[PMIC_I2C_IRQ_END];
	int chgr_virq;
	u32 pmic_intr_sram_addr;
	struct i2c_adapter adapter;
	int i2c_rw;
	wait_queue_head_t i2c_wait;
	struct mutex i2c_pmic_rw_lock;
};

enum I2C_STATUS {
	I2C_WR = 1,
	I2C_RD,
	I2C_NACK = 4
};

static struct pmic_i2c_dev *pmic_dev;
struct i2c_adapter *wcove_pmic_i2c_adapter;

static irqreturn_t pmic_thread_handler(int id, void *data)
{
	int ret, irq0_int;

	ret = regmap_read(pmic_dev->regmap, CHGRIRQ0_ADDR, &irq0_int);
	if (ret < 0) {
		dev_warn(&pmic_dev->pdev->dev, "chgrirq0 read error\n");
		return IRQ_HANDLED;
	}
	pmic_dev->i2c_rw = (irq0_int >> IRQ0_I2C_BIT_POS);
	wake_up(&(pmic_dev->i2c_wait));
	/* Clear pmic i2c interrupt */
	regmap_write(pmic_dev->regmap, CHGRIRQ0_ADDR,
				irq0_int & PMIC_I2C_INTR_MASK);
	return IRQ_HANDLED;
}

/* PMIC i2c read msg */
static inline int pmic_i2c_read_xfer(struct i2c_msg msg)
{
	int ret = 0, val;
	u16 i;
	u8 mask = (I2C_RD | I2C_NACK);

	for (i = 0; i < msg.len ; i++) {
		pmic_dev->i2c_rw = 0;
		ret = regmap_write(pmic_dev->regmap,
					I2COVRDADDR_ADDR, msg.addr);
		if (ret)
			return ret;
		ret = regmap_write(pmic_dev->regmap,
					I2COVROFFSET_ADDR, msg.buf[0] + i);
		if (ret)
			return  ret;

		ret = regmap_write(pmic_dev->regmap,
					I2COVRCTRL_ADDR, I2COVRCTRL_I2C_RD);
		if (ret)
			return ret;

		ret = wait_event_timeout(pmic_dev->i2c_wait,
				(pmic_dev->i2c_rw & mask),
				msecs_to_jiffies(I2C_REG_RW_MAX_TOUT));

		if (pmic_dev->i2c_rw == I2C_NACK) {
			ret =  -EIO;
		} else {
			/*
			 * Read the i2c data register on both interrupt
			 * completion or timeout event.
			 */
			ret = regmap_read(pmic_dev->regmap,
						I2COVRRDDATA_ADDR, &val);
			if (ret < 0)
				ret = -EIO;
			else
				msg.buf[i] = val;
		}
	}
	return ret;
}

/* PMIC i2c write msg */
static inline int pmic_i2c_write_xfer(struct i2c_msg msg)
{
	int ret;
	u16 i;
	u8 mask = (I2C_WR | I2C_NACK);

	for (i = 1; i <= msg.len ; i++) {
		pmic_dev->i2c_rw = 0;
		ret = regmap_write(pmic_dev->regmap,
					I2COVRDADDR_ADDR, msg.addr);
		if (ret)
			return ret;

		ret = regmap_write(pmic_dev->regmap,
					I2COVRWRDATA_ADDR, msg.buf[i]);
		if (ret)
			return ret;

		ret = regmap_write(pmic_dev->regmap,
					I2COVROFFSET_ADDR, msg.buf[0] + i - 1);
		if (ret)
			return ret;

		ret = regmap_write(pmic_dev->regmap,
					I2COVRCTRL_ADDR, I2COVRCTRL_I2C_WR);
		if (ret)
			return ret;

		ret = wait_event_timeout(pmic_dev->i2c_wait,
				(pmic_dev->i2c_rw & mask),
				msecs_to_jiffies(I2C_REG_RW_MAX_TOUT));
		if (pmic_dev->i2c_rw == I2C_NACK)
			return -EIO;
	}
	return 0;
}

static int (*xfer_fn[]) (struct i2c_msg) = {
	pmic_i2c_write_xfer,
	pmic_i2c_read_xfer
};

/* PMIC I2C Master transfer algorithm function */
static int pmic_master_xfer(struct i2c_adapter *adap,
				struct i2c_msg msgs[],
				int num)
{
	int ret = 0;
	int i;
	u8 index;

	mutex_lock(&pmic_dev->i2c_pmic_rw_lock);
	for (i = 0 ; i < num ; i++) {
		index = msgs[i].flags & I2C_M_RD;
		ret = (xfer_fn[index])(msgs[i]);

		if (ret == -EACCES)
			dev_info(&pmic_dev->pdev->dev, "Blocked Access!\n");

		/* If access is restricted, return true to
		*  avoid extra error handling in client
		*/

		if (ret != 0 && ret != -EACCES && !(i <= num))
			goto transfer_err_exit;
	}

	ret = num;

transfer_err_exit:
	mutex_unlock(&pmic_dev->i2c_pmic_rw_lock);
	return ret;
}

/* PMIC I2C adapter capability function */
static u32 pmic_master_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA;
}

static int pmic_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				unsigned short flags, char read_write,
				u8 command, int size,
				union i2c_smbus_data *data)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	msg.addr = addr;
	msg.flags = flags & I2C_M_TEN;
	msg.buf = buf;
	msg.buf[0] = command;
	if (read_write == I2C_SMBUS_WRITE) {
		msg.len = 1;
		msg.buf[1] = data->byte;
	} else {
		msg.flags |= I2C_M_RD;
		msg.len = 1;
	}

	ret = pmic_master_xfer(adap, &msg, 1);
	if (ret == 1) {
		if (read_write == I2C_SMBUS_READ)
			data->byte = msg.buf[0];
		return 0;
	}
	return ret;
}


static const struct i2c_algorithm pmic_i2c_algo = {
	.master_xfer = pmic_master_xfer,
	.functionality = pmic_master_func,
	.smbus_xfer = pmic_smbus_xfer,
};

static irqreturn_t pmic_charger_thread_handler(int id, void *data)
{
	int ret, irq0_int;

	ret = regmap_read(pmic_dev->regmap, CHGRIRQ0_ADDR, &irq0_int);
	if (ret < 0) {
		dev_warn(&pmic_dev->pdev->dev, "chgrirq0 read error\n");
		return IRQ_HANDLED;
	}
	handle_nested_irq(pmic_dev->chgr_virq);
	/* Clear pmic external charger interrupt */
	regmap_write(pmic_dev->regmap, CHGRIRQ0_ADDR,
				irq0_int & PMIC_CHGR_INTR_MASK);
	return IRQ_HANDLED;
}

static int pmic_i2c_charger_interrupt_setup(struct pmic_i2c_dev *pmic)
{
	int ret, chgr_virq;

	chgr_virq = irq_alloc_desc(0);
	if (chgr_virq < 0) {
		dev_warn(&pmic->pdev->dev,
			"failed to allocate IRQ: %d\n", chgr_virq);
		ret = chgr_virq;
		goto ext_chgr_err;
	}
	irq_set_chip_data(chgr_virq, pmic);
	irq_set_chip_and_handler(chgr_virq,
		&dummy_irq_chip, handle_edge_irq);
	irq_set_nested_thread(chgr_virq, 1);
	irq_set_noprobe(chgr_virq);
	pmic->chgr_virq = chgr_virq;

	ret = devm_request_threaded_irq(&pmic->pdev->dev,
					pmic->irq[EXT_CHRG_IRQ], NULL,
					pmic_charger_thread_handler,
					IRQF_ONESHOT, "pmic_ext_charger", pmic);
	if (ret) {
		dev_err(&pmic->pdev->dev, "failed to request interrupt=%d\n",
						pmic->irq[EXT_CHRG_IRQ]);
		goto ext_chgr_err;
	}

	return 0;

ext_chgr_err:
	return ret;
}

static bool pmic_check_oem0_acpi_table(struct pmic_i2c_dev *pmic)
{
	struct acpi_table_header *acpi_tbl = NULL;
	acpi_size tbl_size;
	acpi_status status;

	status = acpi_get_table_with_size("OEM0", 0, &acpi_tbl, &tbl_size);
	if (ACPI_SUCCESS(status) && acpi_tbl) {
		dev_info(&pmic->pdev->dev,
			"OEM0 table size (%d)\n", (int)tbl_size);
		return true;
	} else {
		dev_warn(&pmic->pdev->dev, "OEM0 table NOT present\n");
	}

	return false;
}

static struct charger_platform_data charger_drvdata = {
	.def_cc		= PMIC_CHG_DEF_CC,
	.def_cv		= PMIC_CHG_DEF_CV,
	.iterm		= PMIC_CHG_DEF_ITERM,
	.iprechg	= PMIC_CHG_DEF_IPRECHG,
	.sysvmin	= PMIC_CHG_DEF_VSYSMIN,
	.boostv		= PMIC_CHG_DEF_BOOSTV,
	.boosti		= PMIC_CHG_DEF_BOOSTI,
	.max_cc		= PMIC_CHG_MAX_CC,
	.max_cv		= PMIC_CHG_MAX_CV,
	.min_temp	= PMIC_CHG_MIN_TEMP,
	.max_temp	= PMIC_CHG_MAX_TEMP,
};

static int pmic_i2c_add_external_charger(struct pmic_i2c_dev *pmic)
{
	static struct i2c_board_info i2c_info;
	int ret;

	/*
	 * On Intel Whiskey Cove PMIC based platforms
	 * OEM0 ACPI table will always contain the battery
	 * charging related information. So check for OEM0 table
	 * and set the default charging parameters accordingly.
	 */
	if (!pmic_check_oem0_acpi_table(pmic)) {
		charger_drvdata.def_cc = 0;
		charger_drvdata.def_cv = 0;
		charger_drvdata.max_cc = 0;
		charger_drvdata.max_cv = 0;
	}

	regmap_write(pmic->regmap, CHRTTADDR_ADDR, 0x0);
	/*
	 * Delay the TT read by 2ms to ensure that the data
	 * is populated in data register.
	 */
	usleep_range(2000, 3000);

	strncpy(i2c_info.type, "bxt_wcove_charger", I2C_NAME_SIZE);

	/* Get pmic i2c charger slave address */
	ret = regmap_read(pmic->regmap, CHRTTDATA_ADDR, (int *)&i2c_info.addr);
	if (ret < 0) {
		dev_err(&pmic->pdev->dev, "i2c slave addr read error%d\n", ret);
		goto ext_chgr_err;
	}

	/* Set up external charger interrupt */
	ret = pmic_i2c_charger_interrupt_setup(pmic);
	if (ret < 0) {
		dev_err(&pmic->pdev->dev,
			"failed to set up pmic-external-charger IRQ%d\n", ret);
		goto ext_chgr_err;
	}
	i2c_info.irq = pmic->chgr_virq;
	i2c_info.platform_data = &charger_drvdata;
	i2c_new_device(wcove_pmic_i2c_adapter, &i2c_info);

	/* Unmask external charger interrupt */
	regmap_update_bits(pmic_dev->regmap, MCHGRIRQ0_ADDR,
				PMIC_CHGR_INTR_MASK, 0x0);
	return 0;

ext_chgr_err:
	return ret;
}

static int pmic_i2c_add_adapter(struct pmic_i2c_dev *pmic)
{
	int ret;

	/* Request i2c adapter interrupt */
	ret = devm_request_threaded_irq(&pmic->pdev->dev,
				pmic->irq[EXT_I2C_IRQ], NULL,
				pmic_thread_handler, IRQF_ONESHOT,
				"pmic_i2c_adapter", pmic);
	if (ret) {
		dev_err(&pmic->pdev->dev, "failed to request interrupt=%d\n",
						pmic->irq[EXT_I2C_IRQ]);
		return ret;
	}

	/* Unmask PMIC I2C interrupt */
	regmap_update_bits(pmic->regmap, MCHGRIRQ0_ADDR,
					PMIC_I2C_INTR_MASK, 0x0);

	wcove_pmic_i2c_adapter = &pmic->adapter;
	wcove_pmic_i2c_adapter->owner = THIS_MODULE;
	wcove_pmic_i2c_adapter->class = I2C_CLASS_HWMON;
	wcove_pmic_i2c_adapter->algo = &pmic_i2c_algo;
	strcpy(wcove_pmic_i2c_adapter->name, "PMIC I2C Adapter");
	wcove_pmic_i2c_adapter->nr = pmic->pdev->id;
	ret = i2c_add_numbered_adapter(wcove_pmic_i2c_adapter);

	return ret;
}

static int pmic_i2c_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);
	int i, ret, pirq;

	pmic_dev = devm_kzalloc(&pdev->dev, sizeof(*pmic_dev), GFP_KERNEL);
	if (!pmic_dev)
		return -ENOMEM;

	pmic_dev->pdev = pdev;
	pmic_dev->regmap = wcove->regmap;
	pmic_dev->regmap_irq_chip = wcove->irq_chip_data_level2;

	mutex_init(&pmic_dev->i2c_pmic_rw_lock);
	init_waitqueue_head(&(pmic_dev->i2c_wait));

	for (i = 0; i < PMIC_I2C_IRQ_END; i++) {
		pirq = platform_get_irq(pdev, i);
		pmic_dev->irq[i] =
			regmap_irq_get_virq(pmic_dev->regmap_irq_chip, pirq);
		if (pmic_dev->irq[i] < 0) {
			dev_err(&pdev->dev,
				"failed to get virtual interrupt=%d\n", pirq);
			ret = pmic_dev->irq[i];
			goto error;
		}
	}

	/* Add PMIC i2c adapter */
	ret = pmic_i2c_add_adapter(pmic_dev);
	if (ret) {
		dev_err(&pdev->dev, "error adding the adapter\n");
		goto error;
	}

	/* Add PMIC i2c slave info */
	ret = pmic_i2c_add_external_charger(pmic_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "error adding pmic-external-charger\n");
		goto error;
	}
	return 0;

error:
	return ret;
}

static int pmic_i2c_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device_id wcove_i2c_device_ids[] = {
	{"wcove_pmic_i2c", 0},
	{"bxt_wcove_pmic_i2c", 0},
	{},
};

struct platform_driver pmic_i2c_driver = {
	.driver = {
		.name = "wcove_pmic_i2c",
		.owner = THIS_MODULE,
	},
	.probe = pmic_i2c_probe,
	.remove = pmic_i2c_remove,
	.id_table = wcove_i2c_device_ids,
};
module_platform_driver(pmic_i2c_driver);

MODULE_AUTHOR("Yegnesh Iyer <yegnesh.s.iyer@intel.com>");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("WCove PMIC I2C Master driver");
MODULE_LICENSE("GPL");
