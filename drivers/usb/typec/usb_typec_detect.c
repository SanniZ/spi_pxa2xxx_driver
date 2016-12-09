/*
 * usb_typec_detect.c: usb type-c cable detecton driver
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
 * Author: Albin B <albin.bala.krishnan@intel.com>
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/wait.h>
#include <linux/usb/phy.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/usb_typec_phy.h>
#include "usb_typec_detect.h"

#define CC_OPEN(x)		(x == USB_TYPEC_CC_VRD_UNKNOWN)
#define CC_RD(x)		(x > USB_TYPEC_CC_VRA)
#define CC_RA(x)		(x == USB_TYPEC_CC_VRA)

enum typec_cable_type {
	E_TYPEC_CABLE_UNKNOWN,
	E_TYPEC_CABLE_USB,
	E_TYPEC_CABLE_USB_HOST,
	E_TYPEC_CABLE_USB_SNK,
	E_TYPEC_CABLE_USB_SRC,
	E_TYPEC_CABLE_DP_SRC,
};

static void detect_update_ufp_state(struct typec_detect *detect);

static const unsigned int pd_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_TYPEC_SNK,
	EXTCON_TYPEC_SRC,
	EXTCON_TYPEC_DP_SRC,
	EXTCON_NONE,
};

static LIST_HEAD(typec_detect_list);
static DEFINE_SPINLOCK(slock);

static struct typec_detect *get_typec_detect(struct typec_phy *phy)
{
	struct typec_detect *detect;

	spin_lock(&slock);
	list_for_each_entry(detect, &typec_detect_list, list) {
		if (!strncmp(detect->phy->label, phy->label, MAX_LABEL_SIZE)) {
			spin_unlock(&slock);
			return detect;
		}
	}
	spin_unlock(&slock);

	return NULL;
}

static void typec_detect_notify_extcon(struct typec_detect *detect,
					unsigned int id, bool state)
{
	bool notify_otg = false;
	int otg_evt;

	dev_dbg(detect->phy->dev, "%s: id = %d state = %d\n",
				 __func__, id, state);

	mutex_lock(&detect->lock);

	switch (id) {
	case EXTCON_TYPEC_SNK:
		if (detect->snk_state == state)
			break;
		detect->snk_state = state;
		if (state)
			detect->state = DETECT_STATE_ATTACHED_UFP;
		else
			detect->state = DETECT_STATE_UNATTACHED_UFP;
		break;

	case EXTCON_TYPEC_SRC:
		if (detect->src_state == state)
			break;

		detect->src_state = state;
		if (state)
			detect->state = DETECT_STATE_ATTACHED_DFP;
		else
			detect->state = DETECT_STATE_UNATTACHED_DFP;
		break;

	case EXTCON_USB_HOST:
		if (detect->usb_host_state == state)
			break;

		detect->usb_host_state = state;
		/* Send ID notification to USB subsystem so that
		 *  system will switch host mode of operation.
		 */
		notify_otg = true;
		if (state)
			otg_evt = USB_EVENT_ID;
		else
			otg_evt = USB_EVENT_NONE;
		break;

	case EXTCON_USB:
		if (detect->usb_state == state)
			break;

		detect->usb_state = state;

		notify_otg = true;
		if (state)
			otg_evt = USB_EVENT_VBUS;
		else
			otg_evt = USB_EVENT_NONE;
		break;

	case EXTCON_TYPEC_DP_SRC:
		break;

	default:
		goto notify_ext_err;
	}

	extcon_set_cable_state_(detect->edev, id, state);

	if (notify_otg) {
		struct usb_phy *otg;

		otg = usb_get_phy(USB_PHY_TYPE_USB2);
		if (!IS_ERR_OR_NULL(otg)) {
			atomic_notifier_call_chain(&otg->notifier, otg_evt,
					NULL);
			usb_put_phy(otg);
		}
	}

notify_ext_err:
	mutex_unlock(&detect->lock);
}

void typec_notify_cable_state(struct typec_phy *phy, unsigned int type,
		bool state)
{
	struct typec_detect *detect;

	detect = get_typec_detect(phy);
	if (!detect)
		return;

	typec_detect_notify_extcon(detect, type, state);
	/* If all four cables are disconnected, then start DRP toggling*/
	if (!(detect->usb_state || detect->snk_state
		|| detect->usb_host_state || detect->src_state)
			&& detect->state != DETECT_STATE_UNATTACHED_DRP) {
		dev_info(detect->phy->dev,
			"%s: Cable Disconnected, Move to DRP Mode",
				__func__);
		typec_enable_autocrc(detect->phy, false);
		mutex_lock(&detect->lock);
		detect->state = DETECT_STATE_UNATTACHED_DRP;
		mutex_unlock(&detect->lock);
		typec_switch_mode(detect->phy, TYPEC_MODE_DRP);
	}

}
EXPORT_SYMBOL_GPL(typec_notify_cable_state);

static enum typec_cc_pin get_active_cc(int cc1_rd, int cc2_rd)
{
	int ret = 0;

	if (CC_RD(cc1_rd) && (CC_OPEN(cc2_rd) || CC_RA(cc2_rd)))
		ret = TYPEC_PIN_CC1;
	else if (CC_RD(cc2_rd) && (CC_OPEN(cc1_rd) || CC_RA(cc1_rd)))
		ret = TYPEC_PIN_CC2;

	return ret;
}

static int detect_src_attached(struct cc_pin *cc1, struct cc_pin *cc2)
{
	int ret = false;

	if ((CC_RA(cc1->rd) || (CC_OPEN(cc1->rd))) && CC_RD(cc2->rd))
		ret = true;
	else if	(CC_RD(cc1->rd) && (CC_RA(cc2->rd) || CC_OPEN(cc2->rd)))
		ret = true;
	return ret;
}

static int detect_debug_attached(struct cc_pin *cc1, struct cc_pin *cc2)
{
	return CC_RD(cc1->rd) && CC_RD(cc2->rd);
}

static int detect_audio_attached(struct cc_pin *cc1, struct cc_pin *cc2)
{
	return CC_RA(cc1->rd) && CC_RA(cc2->rd);
}

static void detect_dfp_work(struct work_struct *work)
{
	struct typec_detect *detect =
		container_of(work, struct typec_detect, dfp_work);
	enum typec_cc_pin use_cc = 0;
	struct typec_phy *phy = detect->phy;

	if (detect_src_attached(&phy->cc1, &phy->cc2)) {
		use_cc = get_active_cc(phy->cc1.rd, phy->cc2.rd);
		typec_setup_cc(phy, use_cc, TYPEC_STATE_ATTACHED_DFP);
	} else if (detect_audio_attached(&phy->cc1, &phy->cc2)) {
		dev_info(detect->phy->dev, "Audio Accessory Detected");
	} else if (detect_debug_attached(&phy->cc1, &phy->cc2)) {
		dev_info(detect->phy->dev, "Debug Accessory Detected");
	} else
		goto end;

	mutex_lock(&detect->lock);
	detect->state = DETECT_STATE_ATTACHED_DFP;
	mutex_unlock(&detect->lock);

	typec_detect_notify_extcon(detect,
				EXTCON_TYPEC_SRC, true);
	typec_detect_notify_extcon(detect,
				EXTCON_USB_HOST, true);

	return;

end:
	mutex_lock(&detect->lock);
	detect->state = DETECT_STATE_UNATTACHED_DRP;
	mutex_unlock(&detect->lock);
	typec_switch_mode(phy, TYPEC_MODE_DRP);
}

static void detect_update_ufp_state(struct typec_detect *detect)
{

	mutex_lock(&detect->lock);
	detect->state = DETECT_STATE_ATTACHED_UFP;
	mutex_unlock(&detect->lock);

	typec_detect_notify_extcon(detect,
				EXTCON_TYPEC_SNK, true);
	typec_detect_notify_extcon(detect,
				EXTCON_USB, true);

}

static void detect_ufp_work(struct work_struct *work)
{
	struct typec_detect *detect = container_of(work, struct typec_detect,
					ufp_work);
	struct typec_phy *phy;
	int use_cc = 0;

	phy = detect->phy;
	/*
	 * check valid Rp + vbus presence
	 * emit notifications
	 */
	if (phy->cc1.valid && CC_RD(phy->cc1.rd)) {
		use_cc = TYPEC_PIN_CC1;
	} else if (phy->cc2.valid && CC_RD(phy->cc2.rd)) {
		use_cc = TYPEC_PIN_CC2;
	} else {
		goto end;
	}

	typec_setup_cc(phy, use_cc, TYPEC_STATE_ATTACHED_UFP);
	detect_update_ufp_state(detect);
	return;
end:
	typec_switch_mode(phy, TYPEC_MODE_DRP);
}

static void update_phy_state(struct work_struct *work)
{
	struct typec_phy *phy;
	struct typec_detect *detect;
	int state;

	detect = container_of(work, struct typec_detect, phy_ntf_work);
	phy = detect->phy;

	switch (detect->event) {
	case TYPEC_EVENT_VBUS:
		mutex_lock(&detect->lock);
		state = detect->state;
		mutex_unlock(&detect->lock);
		break;
	case TYPEC_EVENT_NONE:
		dev_dbg(phy->dev, "EVENT NONE: state = %d", detect->state);

		if (detect->state == DETECT_STATE_ATTACHED_UFP) {
			dev_dbg(phy->dev, "%s: UFP Disconnected, state=%d",
				__func__, detect->state);
			typec_detect_notify_extcon(detect,
						EXTCON_TYPEC_SNK, false);
			if (detect->usb_state) {
				typec_detect_notify_extcon(detect,
						EXTCON_USB, false);
			} else if (detect->usb_host_state) {
				typec_detect_notify_extcon(detect,
						EXTCON_USB_HOST, false);
			} else
				dev_warn(phy->dev, "%s:Unknown date role!!\n",
						__func__);

			typec_enable_autocrc(detect->phy, false);

			mutex_lock(&detect->lock);
			detect->state = DETECT_STATE_UNATTACHED_DRP;
			mutex_unlock(&detect->lock);
			typec_switch_mode(phy, TYPEC_MODE_DRP);

		} else if (detect->state == DETECT_STATE_ATTACHED_DFP) {
			/* state = DFP; disable VBUS */
			typec_detect_notify_extcon(detect,
						EXTCON_TYPEC_SRC, false);
			if (detect->usb_state) {
				typec_detect_notify_extcon(detect,
						EXTCON_USB, false);
			} else if (detect->usb_host_state) {
				typec_detect_notify_extcon(detect,
						EXTCON_USB_HOST, false);
			} else
				dev_warn(phy->dev, "%s:Unknown date role!!\n",
						__func__);


			typec_enable_autocrc(detect->phy, false);
		}

		break;

	default:
		dev_err(detect->phy->dev, "unknown event %d", detect->event);
	}
}

static int typec_handle_phy_ntf(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct typec_phy *phy;
	struct typec_detect *detect =
		container_of(nb, struct typec_detect, nb);
	int handled = NOTIFY_OK;

	phy = detect->phy;
	if (!phy)
		return NOTIFY_BAD;

	switch (event) {
	case TYPEC_EVENT_UFP:
		schedule_work(&detect->ufp_work);
		break;
	case TYPEC_EVENT_VBUS:
	case TYPEC_EVENT_NONE:
		detect->event = event;
		/* Do not enable drp toggle here as this EVENT_NONE
		 * could be due to pwr role swap.
		 */
		schedule_work(&detect->phy_ntf_work);
		break;
	case TYPEC_EVENT_DFP:
		/* Ignore event in case of DFP ATTATCHED state */
		if (detect->state == DETECT_STATE_ATTACHED_DFP)
			break;
		detect->state = DETECT_STATE_UNATTACHED_DFP;
		schedule_work(&detect->dfp_work);
		break;
	default:
		handled = NOTIFY_DONE;
	}
	return handled;
}

static void detect_remove(struct typec_detect *detect)
{
	struct typec_phy *phy;
	if (!detect)
		return;

	phy = detect->phy;
	cancel_work_sync(&detect->phy_ntf_work);
	cancel_work_sync(&detect->dfp_work);

	if (detect->edev)
		extcon_dev_unregister(detect->edev);
	kfree(detect);
}

static struct typec_detect *dptr;

int typec_bind_detect(struct typec_phy *phy)
{
	struct typec_detect *detect;
	int ret;

	detect = kzalloc(sizeof(struct typec_detect), GFP_KERNEL);

	if (!detect) {
		pr_err("typec fsm: no memory");
		return -ENOMEM;
	}

	if (!phy) {
		pr_err("%s: no valid phy provided", __func__);
		return -EINVAL;
	}

	detect->phy = phy;
	if (phy->is_pd_capable)
		detect->is_pd_capable = phy->is_pd_capable(phy);
	detect->nb.notifier_call = typec_handle_phy_ntf;

	ret = typec_register_notifier(phy, &detect->nb);
	if (ret  < 0) {
		dev_err(phy->dev, "unable to register notifier");
		goto error;
	}


	INIT_WORK(&detect->phy_ntf_work, update_phy_state);
	INIT_WORK(&detect->dfp_work, detect_dfp_work);
	INIT_WORK(&detect->ufp_work, detect_ufp_work);

	detect->state = DETECT_STATE_UNATTACHED_DRP;

	mutex_init(&detect->lock);

	dptr = detect;

	detect->edev = devm_extcon_dev_allocate(phy->dev, pd_extcon_cable);
	if (IS_ERR(detect->edev)) {
		dev_err(phy->dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(phy->dev, detect->edev);
	if (ret) {
		dev_err(phy->dev, "Unable to register extcon dev\n");
		goto error;
	}

	list_add_tail(&detect->list, &typec_detect_list);
	return 0;

error:
	detect_remove(detect);
	return ret;
}

int typec_unbind_detect(struct typec_phy *phy)
{
	struct typec_detect *detect, *temp;

	spin_lock(&slock);
	list_for_each_entry_safe(detect, temp, &typec_detect_list, list) {
		if (!strncmp(detect->phy->label, phy->label, MAX_LABEL_SIZE)) {
			list_del(&detect->list);
			detect_remove(detect);
		}
	}
	spin_unlock(&slock);

	return 0;
}

int typec_get_pd_profile(struct typec_pd_profile *profile)
{
	struct typec_phy *phy = dptr->phy;
	bool state;

	state = extcon_get_cable_state_(dptr->edev, EXTCON_TYPEC_SNK);

	if (state) {
		profile->nom_voltage = TYPEC_NOMINAL_VOLTAGE;
		profile->max_current = typec_get_host_current(phy) * 1000;
	} else {
		dev_dbg(dptr->phy->dev, "%s: disconnect\n", __func__);
		profile->nom_voltage = 0;
		profile->max_current = 0;
	}

	return 0;
}
