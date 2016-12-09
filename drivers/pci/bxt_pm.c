/*
 * Device Idle support for Intel BXT platform
 *
 * Copyright (C) 2015, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/pm_qos.h>
#include "pci.h"

/* devIdle control register CIP bit */
#define DEVIDLE_CTRL_CIP	0x1
/* devIdle cptr register valid bit */
#define DEVIDLE_CPTR_VALID	0x1

static int bxt_devidle_wait_for_cip(struct pci_dev *dev)
{
	unsigned long timeout = 100;

	/* Check for CIP (CommandInProgress) bit */
	do {
		if (!(readl(dev->devidle_ctrl) & DEVIDLE_CTRL_CIP))
			return 0;
		udelay(2);

	} while (--timeout);

	return -EBUSY;
}

static int bxt_devidle_enter(struct pci_dev *dev)
{
	int ret;
	unsigned long flags;

	/* Wait for CIP if any previous transaction */
	ret = bxt_devidle_wait_for_cip(dev);
	if (ret)
		return ret;

	raw_spin_lock_irqsave(&pci_lock, flags);

	/*
	 * Enter the devIdle state. Blindly overrite the
	 * control register with the required bit set. Other
	 * bits are to be ignored..
	 */
	writel(0x1 << PCI_VNDR_DEVIDLE_MASK, dev->devidle_ctrl);

	raw_spin_unlock_irqrestore(&pci_lock, flags);

	/* Wait for CIP */
	ret = bxt_devidle_wait_for_cip(dev);
	if (ret)
		return ret;

	return 0;
}

static int bxt_devidle_exit(struct pci_dev *dev)
{
	int ret;
	unsigned long flags;

	/* Wait for CIP if any previous transaction */
	ret = bxt_devidle_wait_for_cip(dev);
	if (ret)
		return ret;

	raw_spin_lock_irqsave(&pci_lock, flags);

	/*
	 * Exit out of devIdle state without any completion
	 * interrupt.
	 */
	writel(0, dev->devidle_ctrl);

	raw_spin_unlock_irqrestore(&pci_lock, flags);

	/* Wait for CIP */
	ret = bxt_devidle_wait_for_cip(dev);
	if (ret)
		return ret;

	return 0;
}

static void bxt_devidle_set_ltr(struct device *gendev, s32 val)
{
	struct pci_dev *dev = container_of(gendev, struct pci_dev, dev);
	u32 ltr;

	ltr = readl(dev->devidle_ltr + DEVIDLE_ACTIVE_SW_LTR);
	if (val == PM_QOS_LATENCY_ANY || val < 0)
		ltr &= ~DEVIDLE_LTR_REQ;	/* No requirement */
	else {
		ltr |= DEVIDLE_LTR_REQ;
		ltr &= DEVIDLE_LTR_SCALE_MASK;
		ltr &= DEVIDLE_LTR_VALUE_MASK;

		/* Consider only snoop latency */
		if (val > DEVIDLE_LTR_VALUE_MASK)
			ltr |= DEVIDLE_LTR_SCALE_32US | val >> 5; /* 32 ns */
		else
			ltr |= DEVIDLE_LTR_SCALE_1US | val;	/* 1 ns */
	}

	writel(ltr, dev->devidle_ltr + DEVIDLE_ACTIVE_SW_LTR);
	writel(ltr, dev->devidle_ltr + DEVIDLE_IDLE_SW_LTR);
}

static int bxt_devidle_set_power_state(struct pci_dev *pdev, pci_power_t state)
{
	int error = 0;
	u32 data;
	struct acpi_device *adev;

	switch (state) {

	case PCI_D0:
	case PCI_D1:
	case PCI_D2:
		error = bxt_devidle_exit(pdev);
		/*
		 * When RestoreRequired is set by the HW upon
		 * exit, then we must restore the state and
		 * clear it by writing logical '1'
		 */
		data = readl(pdev->devidle_ctrl);
		if (data & PCI_VNDR_DEVIDLE_RR_MASK) {
			writel(data | PCI_VNDR_DEVIDLE_RR_MASK,
				pdev->devidle_ctrl);
		/*
		 * Less likely to restore the bars..It is here in case
		 * hardware does not behave as expected..
		 */
			pci_restore_bars(pdev);
		}
		break;
	case PCI_D3hot:
	case PCI_D3cold:
		/* enter driver assisted devIdle state */
		error = bxt_devidle_enter(pdev);
		break;
	}

	/*
	 * Check if there is associated ACPI device. if yes, utilize
	 * the acpi-pci wrapper to invoke acpi specific flows if any
	 * (power resources and _PSx methods).
	 *
	 * Need ensure the device is power manageable for invoking
	 * acpi-pci flows.
	 *
	 */
	adev = ACPI_COMPANION(&pdev->dev);
	if (adev && adev->flags.power_manageable)
		error = acpi_pci_set_power_state(pdev, state);

	if (!error)
		dev_dbg(&pdev->dev, "power state changed to %d\n", state);

	return error;
}

static bool bxt_devidle_power_manageable(struct pci_dev *dev)
{
	return true;
}

static pci_power_t bxt_devidle_choose_state(struct pci_dev *pdev)
{
	return PCI_D3hot;
}

static struct pci_device_pm_ops bxt_devidle_platform_pm = {
	.is_manageable = bxt_devidle_power_manageable,
	.choose_state = bxt_devidle_choose_state,
	.set_state = bxt_devidle_set_power_state,
};

static int intel_bxt_devidle_discover(struct pci_dev *pdev)
{
	int pos, bar_num;
	u32 data;

	/* DevIdle encoded in vendor specific capability record */
	pos = pci_find_capability(pdev, PCI_CAP_ID_VNDR);
	pci_read_config_dword(pdev, pos + PCI_VNDR_DEVIDLE_CPTR, &data);

	/* Check if we are compliant to the devIdle definition */
	if (data & DEVIDLE_CPTR_VALID) {
		/*
		 * The function implements MMIO mapped devIdle
		 * registers; Let's extract the BARNUM and
		 * map the control register in the MMIO space.
		 */
		bar_num = PCI_VNDR_DEVIDLE_GET_BARNUM(data);
		pdev->devidle_ctrl = pci_ioremap_bar(pdev, bar_num);
		if (!pdev->devidle_ctrl) {
			dev_err(&pdev->dev, "Failed to locate devIdle "
					"register\n");
			goto err;
		}
		pdev->devidle_ctrl += PCI_VNDR_DEVIDLE_GET_OFFSET(data);
		pci_set_device_pm(pdev, &bxt_devidle_platform_pm);

		/* get the ltr capability */
		pci_read_config_dword(pdev, pos + PCI_VNDR_DEVIDLE_LTR, &data);
		bar_num = PCI_VNDR_DEVIDLE_GET_BARNUM(data);
		pdev->devidle_ltr = pci_ioremap_bar(pdev, bar_num);
		if (!pdev->devidle_ltr) {
			dev_err(&pdev->dev, "Failed to locate devIdle "
					"ltr register\n");
			goto err;
		}
		pdev->devidle_ltr += PCI_VNDR_DEVIDLE_GET_OFFSET(data);
		/* Set the ltr callbacks through PM QoS */
		pdev->dev.power.set_latency_tolerance = bxt_devidle_set_ltr;
		dev_pm_qos_expose_latency_tolerance(&pdev->dev);
	} else {
		dev_err(&pdev->dev, "DevIdle not discovered for the device\n");
		goto err;
	}

	return 0;
err:
	return -EINVAL;
}

static const struct pci_device_id intel_bxt_devidle_ids[] = {
	/* Devices which supports devIdle */
	{ PCI_VDEVICE(INTEL, 0x0a98), },	/* snd_soc_hda */
	{ PCI_VDEVICE(INTEL, 0x0a9a), },	/* mei_me */
	{ PCI_VDEVICE(INTEL, 0x0aa2), },	/* heci_ish */
	{ PCI_VDEVICE(INTEL, 0x0aa8), },	/* xhci_hcd */
	{ PCI_VDEVICE(INTEL, 0x0aaa), },	/* dwc3-pci */
	{ PCI_VDEVICE(INTEL, 0x0aae), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0ab0), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0ab2), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0ab4), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0ab6), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0ab8), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0aba), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x0abc), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x0abe), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x0ac0), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x0ac2), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x0ac4), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x0ac6), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x0aee), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x0aca), },	/* sdhci-pci */
	{ PCI_VDEVICE(INTEL, 0x0acc), },	/* sdhci-pci */
	{ PCI_VDEVICE(INTEL, 0x0ace), },	/* ufshcd */
	{ PCI_VDEVICE(INTEL, 0x0ad0), },	/* sdhci */

	/* Following are BXT B0 PCI devices */
	{ PCI_VDEVICE(INTEL, 0x1a9a), },	/* mei_me (heci1) */
	{ PCI_VDEVICE(INTEL, 0x1a9e), },	/* heci3 */
	{ PCI_VDEVICE(INTEL, 0x1aa2), },	/* heci_ish */
	{ PCI_VDEVICE(INTEL, 0x1aa8), },	/* xhci_hcd */
	{ PCI_VDEVICE(INTEL, 0x1aaa), },	/* xdci */
	{ PCI_VDEVICE(INTEL, 0x1aac), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1aae), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1ab0), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1ab2), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1ab4), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1ab6), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1ab8), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1aba), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x1abc), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x1abe), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x1ac0), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x1ac2), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x1ac4), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x1ac6), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x1aee), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x1aca), },	/* sdhci-pci */
	{ PCI_VDEVICE(INTEL, 0x1acc), },	/* sdhci-pci */
	{ PCI_VDEVICE(INTEL, 0x1ace), },	/* ufshcd */
	{ PCI_VDEVICE(INTEL, 0x1ad0), },	/* sdhci */

	/* Following are BXT-P A0 PCI devices */
	{ PCI_VDEVICE(INTEL, 0x5a9a), },	/* mei_me (heci1) */
	{ PCI_VDEVICE(INTEL, 0x5a9e), },	/* heci3 */
	{ PCI_VDEVICE(INTEL, 0x5aa2), },	/* heci_ish */
	{ PCI_VDEVICE(INTEL, 0x5aaa), },	/* xdci */
	{ PCI_VDEVICE(INTEL, 0x5aac), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5aae), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5ab0), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5ab2), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5ab4), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5ab6), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5ab8), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5aba), },	/* i2c-designware-pci */
	{ PCI_VDEVICE(INTEL, 0x5abc), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x5abe), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x5ac0), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x5ac2), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x5ac4), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x5ac6), },	/* pxa2xx_spi_pci */
	{ PCI_VDEVICE(INTEL, 0x5aee), },	/* serial */
	{ PCI_VDEVICE(INTEL, 0x5ac8), },	/* pwm-lpss */
	{ PCI_VDEVICE(INTEL, 0x5aca), },	/* sdhci-pci */
	{ PCI_VDEVICE(INTEL, 0x5acc), },	/* sdhci-pci */
	{ PCI_VDEVICE(INTEL, 0x5ace), },	/* ufshcd */
	{ PCI_VDEVICE(INTEL, 0x5ad0), },	/* sdhci */
};
MODULE_DEVICE_TABLE(pci, intel_bxt_devidle_ids);

static ssize_t devidle_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	u32 d0ix_sts;

	d0ix_sts = readl(pci_dev->devidle_ctrl);
	d0ix_sts = (d0ix_sts & (1 << PCI_VNDR_DEVIDLE_MASK)) ? 1 : 0;

	return sprintf(buf, "%s\n", d0ix_sts ? "DevIdle" : "D0");
}

static DEVICE_ATTR(devidle_state, S_IRUSR, devidle_state_show, NULL);

static struct attribute *devidle_attrs[] = {
	&dev_attr_devidle_state.attr,
	NULL,
};

static struct attribute_group devidle_attr_grp = {
	.attrs = devidle_attrs,
	.name = "state",
};

static int intel_bxt_devidle_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct pci_dev *pci_dev = to_pci_dev(data);
	int r;

	if (!pci_match_id(intel_bxt_devidle_ids, pci_dev))
		return 0;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		r = intel_bxt_devidle_discover(pci_dev);
		if (r)
			return r;
		return sysfs_create_group(&pci_dev->dev.kobj,
				&devidle_attr_grp);
	case BUS_NOTIFY_DEL_DEVICE:
		sysfs_remove_group(&pci_dev->dev.kobj, &devidle_attr_grp);
	default:
		break;
	}

	return 0;
}

static struct notifier_block device_nb = {
	.notifier_call = intel_bxt_devidle_notifier,
};

static int __init register_bxt_devidle_pci_notifier(void)
{
	return bus_register_notifier(&pci_bus_type, &device_nb);
}
arch_initcall(register_bxt_devidle_pci_notifier);

MODULE_AUTHOR("Srinidhi Kasagar <srinidhi.kasagar@intel.com>");
MODULE_LICENSE("GPL v2");
