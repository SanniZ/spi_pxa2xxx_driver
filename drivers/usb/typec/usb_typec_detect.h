/*
 * usb_typec_detect.h: usb type-c cable detection header file
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
 * Author: Kannappan, R <r.kannappan@intel.com>
 */

#ifndef __USB_TYPEC_DETECT_H__
#define __USB_TYPEC_DETECT_H__

#include <linux/timer.h>
#include <linux/extcon.h>
#include <linux/usb_typec_phy.h>

enum typec_detect_state {
	DETECT_STATE_UNATTACHED_UFP,
	DETECT_STATE_UNATTACHED_DFP,
	DETECT_STATE_UNATTACHED_DRP,
	DETECT_STATE_ATTACHED_DFP,
	DETECT_STATE_ATTACH_DFP_DRP_WAIT,
	DETECT_STATE_LOCK_UFP,
	DETECT_STATE_ATTACHED_UFP,
};

struct typec_detect {
	struct typec_phy *phy;
	struct extcon_dev *edev;
	enum typec_detect_state state;
	enum typec_detect_state old_state;
	enum typec_event event;
	struct notifier_block nb;
	struct work_struct phy_ntf_work;
	struct work_struct dfp_work;
	struct work_struct ufp_work;
	struct list_head list;
	struct notifier_block otg_nb;
	struct mutex lock;

	/* One bit status variables. */
	unsigned is_pd_capable:1;
	unsigned usb_state:1;
	unsigned usb_host_state:1;
	unsigned snk_state:1;
	unsigned src_state:1;
};

extern int typec_bind_detect(struct typec_phy *phy);
extern int typec_unbind_detect(struct typec_phy *phy);
extern void typec_notify_cable_state(struct typec_phy *phy,
			unsigned int type, bool state);

#endif /* __USB_TYPEC_DETECT_H__ */
