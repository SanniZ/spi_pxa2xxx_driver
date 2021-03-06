/*
 * Intel CherryTrail USB OTG Transceiver driver
 *
 * Copyright (C) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/usb/otg-fsm.h>
#include <linux/usb/otg.h>

struct cht_otg {
	struct usb_phy phy;
	struct otg_fsm fsm;
	struct notifier_block nb;
	struct extcon_dev *c_edev;
	struct notifier_block id_nb;
	struct work_struct fsm_work;
	void __iomem *regs;
	u8 event_count;
	spinlock_t event_lock;
};
