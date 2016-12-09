/*
 * intel_punit_ipc.c: Driver for the Broxton Punit Mailbox IPC mechanism
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/pci.h>
#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>
#include <asm/intel-mid.h>
#include <asm/intel_punit_ipc.h>

/*Three external mailbox*/
enum mailbox_type {
	BIOS_MAILBOX,
	GTDRIVER_MAILBOX,
	ISPDRIVER_MAILBOX,
	RESERVED_MAILBOX,
};

/*Mailbox registers*/
#define MAILBOX_INTERFACE		0x4
#define		CMD_RUN			(1 <<  31)
#define		CMD_ERRCODE_MASK	(0xFFu)
#define		CMD_PARA1_SHIFT		8
#define		CMD_PARA2_SHIFT		16
#define 	CMD_MASK		0xFF
#define MAILBOX_DATA_LOW		0x0
#define MAILBOX_DATA_HIGH		0x8

#define CMD_TIMEOUT_SECONDS		3

static int  punit_ipc_pm_callback(struct notifier_block *nb,
					unsigned long action,
					void *ignored);

static struct notifier_block punit_ipc_pm_notifier = {
	.notifier_call = punit_ipc_pm_callback,
	.priority = 1,
};

struct intel_punit_ipc_controller {
	struct platform_device *pdev;
	spinlock_t lock;
	struct wake_lock wake_lock;
	void __iomem *base[RESERVED_MAILBOX];
	struct completion cmd_complete;
	int irq;
	bool suspend_status;

	int cmd;
	enum mailbox_type type;
};

static struct intel_punit_ipc_controller ipcdev;

static char *ipc_err_sources[] = {
	[IPC_ERR_SUCCESS] =
		"no error",
	[IPC_ERR_INVALID_CMD] =
		"invalid command",
	[IPC_ERR_INVALID_PARAMETER] =
		"invalid parameter",
	[IPC_ERR_CMD_TIMEOUT] =
		"command timeout",
	[IPC_ERR_CMD_LOCKED] =
		"command locked",
	[IPC_ERR_INVALID_VR_ID] =
		"invalid vr id",
	[IPC_ERR_VR_ERR] =
		"vr error",
};

static bool suspend_in_progress(void)
{
	return ipcdev.suspend_status;
}

static void set_suspend_status(bool status)
{
	spin_lock(&ipcdev.lock);
	ipcdev.suspend_status = status;
	spin_unlock(&ipcdev.lock);
}

static int punit_ipc_pm_callback(struct notifier_block *nb,
			unsigned long action, void *ignored)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		set_suspend_status(true);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		set_suspend_status(false);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static inline u32 ipc_read_status(void)
{
	return readl(ipcdev.base[ipcdev.type] + MAILBOX_INTERFACE);
}

static inline void ipc_write_cmd(u32 cmd)
{
	writel(cmd, ipcdev.base[ipcdev.type] + MAILBOX_INTERFACE);
}

static inline u32 ipc_read_data_low(void)
{
	return readl(ipcdev.base[ipcdev.type] + MAILBOX_DATA_LOW);
}

static inline u32 ipc_read_data_high(void)
{
	return readl(ipcdev.base[ipcdev.type] + MAILBOX_DATA_HIGH);
}

static inline void ipc_write_data_low(u32 data)
{
	writel(data, ipcdev.base[ipcdev.type] + MAILBOX_DATA_LOW);
}

static inline void ipc_write_data_high(u32 data)
{
	writel(data, ipcdev.base[ipcdev.type] + MAILBOX_DATA_HIGH);
}

static void intel_punit_ipc_lock(void)
{
	spin_lock(&ipcdev.lock);
	if (!suspend_in_progress())
		wake_lock(&ipcdev.wake_lock);
}

static void intel_punit_ipc_unlock(void)
{
	if (!suspend_in_progress())
		wake_unlock(&ipcdev.wake_lock);
	spin_unlock(&ipcdev.lock);
}

static void intel_punit_ipc_send_command(u32 cmd)
{
	if ((cmd & CMD_MASK) < IPC_GTD_PUNIT_CMD_BASE)
		ipcdev.type = BIOS_MAILBOX;
	else
		return;
	ipcdev.cmd = cmd;
	reinit_completion(&ipcdev.cmd_complete);
	ipc_write_cmd(cmd);
}

static int intel_punit_ipc_check_status(void)
{
	int ret = 0;
	int status;
	int errcode;
	int loops = CMD_TIMEOUT_SECONDS * USEC_PER_SEC;

	if (ipcdev.irq) {
		if (0 == wait_for_completion_timeout(
				&ipcdev.cmd_complete,
				CMD_TIMEOUT_SECONDS * HZ)) {
			dev_err(&ipcdev.pdev->dev,
				"IPC timed out, IPC_CMD=0x%x\n", ipcdev.cmd);
			return -ETIMEDOUT;
		}
	} else {
		while ((ipc_read_status() & CMD_RUN) && --loops)
			udelay(1);
		if (loops == 0) {
			dev_err(&ipcdev.pdev->dev,
				"IPC timed out, IPC_CMD=0x%x\n", ipcdev.cmd);
			return -ETIMEDOUT;
		}
	}

	status = ipc_read_status();
	errcode = status & CMD_ERRCODE_MASK;
	if (errcode) {
		ret = -EIO;
		if (errcode < ARRAY_SIZE(ipc_err_sources))
			dev_err(&ipcdev.pdev->dev,
				"IPC failed: %s, IPC_STS=0x%x, IPC_CMD=0x%x\n",
				ipc_err_sources[errcode], status, ipcdev.cmd);
		else
			dev_err(&ipcdev.pdev->dev,
				"IPC failed: unknown err,STS=0x%x, CMD=0x%x\n",
				status, ipcdev.cmd);
	}

	return ret;
}

int intel_punit_ipc_simple_command(int cmd, int para1, int para2)
{
	int ret;

	intel_punit_ipc_lock();
	intel_punit_ipc_send_command(CMD_RUN |
				para2 << CMD_PARA2_SHIFT |
				para1 << CMD_PARA1_SHIFT |
				cmd);
	ret = intel_punit_ipc_check_status();
	intel_punit_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL(intel_punit_ipc_simple_command);

static int intel_punit_ipc_raw_cmd(u32 cmd, u32 para1, u32 para2, u32 *in, u32 *out)
{
	int ret;

	if (in) {
		ipc_write_data_low(*in);
		if (ipcdev.type == GTDRIVER_MAILBOX ||
				ipcdev.type == ISPDRIVER_MAILBOX) {
			in++;
			ipc_write_data_high(*in);
		}
	}
	intel_punit_ipc_send_command(CMD_RUN |
				para2 << CMD_PARA2_SHIFT |
				para1 << CMD_PARA1_SHIFT |
				cmd);
	ret = intel_punit_ipc_check_status();
	if (out) {
		*out = ipc_read_data_low();
		if (ipcdev.type == GTDRIVER_MAILBOX ||
				ipcdev.type == ISPDRIVER_MAILBOX) {
			out++;
			*out = ipc_read_data_high();
		}
	}

	return ret;
}

int intel_punit_ipc_command(u32 cmd, u32 para1, u32 para2, u32 *in, u32 *out)
{
	int ret;

	intel_punit_ipc_lock();
	ret = intel_punit_ipc_raw_cmd(cmd, para1, para2, in, out);
	intel_punit_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(intel_punit_ipc_command);

static irqreturn_t ioc(int irq, void *dev_id)
{
	complete(&ipcdev.cmd_complete);
	return IRQ_HANDLED;
}

static int intel_punit_get_bars(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	void __iomem *addr;
	int size;
	int ret;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res0) {
		dev_err(&pdev->dev, "Fail to get iomem resource\n");
		return -EINVAL;
	}
	size = resource_size(res0);
	if (!request_mem_region(res0->start, size, pdev->name)) {
		dev_err(&pdev->dev, "Fail to request iomem resouce\n");
		return -EBUSY;
	}

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1) {
		dev_err(&pdev->dev, "Fail to get iomem resource1\n");
		return -EINVAL;
	}
	size = resource_size(res1);
	if (!request_mem_region(res1->start, size, pdev->name)) {
		dev_err(&pdev->dev, "Fail to request iomem resouce1\n");
		ret = -EBUSY;
		goto err_res1;
	}

	addr = ioremap_nocache(res0->start,
			resource_size(res0) + resource_size(res1));
	if (!addr) {
		dev_err(&pdev->dev, "I/O memory remapping failed\n");
		ret = -ENOMEM;
		goto err_map;
	}
	ipcdev.base[BIOS_MAILBOX] = addr;

	return 0;

err_map:
	release_mem_region(res1->start, resource_size(res1));
err_res1:
	release_mem_region(res0->start, resource_size(res0));
	return ret;
}

static int intel_punit_ipc_probe(struct platform_device *pdev)
{
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ipcdev.irq = 0;
		dev_warn(&pdev->dev, "No irq\n");
	} else {
		if (request_irq(irq, ioc, IRQF_NO_SUSPEND, "intel_punit_ipc",
			&ipcdev)) {
			dev_err(&pdev->dev, "request irq %d\n", irq);
			return -EBUSY;
		}
		ipcdev.irq = irq;
	}

	ret = intel_punit_get_bars(pdev);
	if (ret) {
		if (ipcdev.irq)
			free_irq(ipcdev.irq, &ipcdev);
		return ret;
	}

	ipcdev.pdev = pdev;
	spin_lock_init(&ipcdev.lock);
	wake_lock_init(&ipcdev.wake_lock, WAKE_LOCK_SUSPEND, "intel_punit_ipc");
	init_completion(&ipcdev.cmd_complete);
	return 0;
}

static int intel_punit_ipc_remove(struct platform_device *pdev)
{
	struct resource *res;

	if (ipcdev.irq)
		free_irq(ipcdev.irq, &ipcdev);
	iounmap(ipcdev.base[BIOS_MAILBOX]);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res)
		release_mem_region(res->start, resource_size(res));
	return 0;
}

static struct platform_driver intel_punit_ipc_driver = {
	.probe = intel_punit_ipc_probe,
	.remove = intel_punit_ipc_remove,
	.driver = {
		.name = "intel_punit_ipc",
	},
};

static int __init intel_punit_ipc_init(void)
{
	register_pm_notifier(&punit_ipc_pm_notifier);
	return platform_driver_register(&intel_punit_ipc_driver);
}

static void __exit intel_punit_ipc_exit(void)
{
	platform_driver_unregister(&intel_punit_ipc_driver);
}

MODULE_AUTHOR("qipeng.zha@intel.com");
MODULE_DESCRIPTION("Intel Punit Mailbox IPC driver");
MODULE_LICENSE("GPL V2");

rootfs_initcall(intel_punit_ipc_init);
module_exit(intel_punit_ipc_exit);
