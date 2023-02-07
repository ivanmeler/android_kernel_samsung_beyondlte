/*
 * Copyright (C) 2018-2020 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "TCM: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sec_class.h>
#include <linux/sec_batt.h>

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif
#include <linux/usb_notify.h>
#include <linux/ccic/ccic_core.h>
#include <linux/ccic/ccic_notifier.h>
#include <linux/power_supply.h>
#if defined(CONFIG_BATTERY_SAMSUNG_V2)
#include "../../battery_v2/include/sec_charging_common.h"
#else
#include <linux/battery/sec_charging_common.h>
#endif
#if IS_ENABLED(CONFIG_CABLE_TYPE_NOTIFIER)
#include <linux/cable_type_notifier.h>
#endif
#if IS_ENABLED(CONFIG_SEC_PD)
#include <linux/battery/sec_pd.h>
#elif defined(CONFIG_BATTERY_NOTIFIER)
#include <linux/battery/battery_notifier.h>
#endif
#include <linux/usb/manager/usb_typec_manager_notifier.h>
#if defined(CONFIG_USB_HW_PARAM)
#include <linux/usb/manager/usb_typec_manager_hwparam.h>
#endif

#if defined(CONFIG_SEC_ABC)
#include <linux/sti/abc_common.h>
#endif
#if defined(CONFIG_SEC_FACTORY)
#include <linux/sysfs.h>
#endif

static int manager_notifier_init_done = 0;
static int confirm_manager_notifier_register = 0;

static struct device *manager_device;
static manager_data_t typec_manager;

static int manager_notifier_init(void);
static void manager_usb_enum_state_check(uint time_ms);
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
static void manager_event_processing_by_vbus(bool run);
#endif

#if defined(CONFIG_PDIC_SWITCH)
static bool is_factory_jig;
#endif

static const char *manager_notify_string(int mns)
{
	switch (mns) {
	case MANAGER_NOTIFY_MUIC_NONE: return "muic_none";
	case MANAGER_NOTIFY_MUIC_USB: return "muic_usb";
	case MANAGER_NOTIFY_MUIC_OTG: return "muic_otg";
	case MANAGER_NOTIFY_MUIC_CHARGER: return "muic_charger";
	case MANAGER_NOTIFY_MUIC_TIMEOUT_OPEN_DEVICE:
		return "muic_timeout_open_device";
	case MANAGER_NOTIFY_MUIC_UART: return "muic_uart";

	case MANAGER_NOTIFY_CCIC_INITIAL: return "pdic_initial";
	case MANAGER_NOTIFY_CCIC_WACOM: return "pdic_wacom";
	case MANAGER_NOTIFY_CCIC_SENSORHUB: return "pdic_sensorhub";
	case MANAGER_NOTIFY_CCIC_USBDP: return "pdic_usbdp";
	case MANAGER_NOTIFY_CCIC_DP: return "pdic_dp";
	case MANAGER_NOTIFY_CCIC_SUB_BATTERY: return "pdic_sub_battery";
	case MANAGER_NOTIFY_CCIC_BATTERY: return "pdic_battery";
	case MANAGER_NOTIFY_CCIC_USB: return "pdic_usb";
	case MANAGER_NOTIFY_CCIC_MUIC: return "pdic_muic";
	default:
		return "undefined";
	}
}

int manager_check_to_send_water_event(MANAGER_NOTI_TYPEDEF event)
{
	/* Check if the same event has already been sent */
	if (typec_manager.water.wVbus_det == event.sub1)
		return -1;

	pr_info("%s: Sending %s event\n", __func__, event.sub1 ? "WATER" : "DRY");
	typec_manager.water.wVbus_det = event.sub1;
#if defined(CONFIG_USB_HW_PARAM)
	wVbus_time_update(typec_manager.water.wVbus_det);
#endif
	return 0;
}

void manager_dp_state_change(MANAGER_NOTI_TYPEDEF event)
{
	switch (event.dest) {
	case CCIC_NOTIFY_DEV_DP:
		switch (event.id) {
		case CCIC_NOTIFY_ID_DP_CONNECT:
			typec_manager.dp.attach_state = event.sub1;
			typec_manager.dp.is_connect = 0;
			typec_manager.dp.hs_connect = 0;
			break;
		case CCIC_NOTIFY_ID_DP_HPD:
			typec_manager.dp.hpd_state = event.sub1;
			break;
		case CCIC_NOTIFY_ID_DP_LINK_CONF:
			typec_manager.dp.cable_type = event.sub1;
			break;
		}
		break;
	case CCIC_NOTIFY_DEV_USB_DP:
		if (event.id == CCIC_NOTIFY_ID_USB_DP) {
			typec_manager.dp.is_connect = event.sub1;
			typec_manager.dp.hs_connect = event.sub2;
		}
		break;
	default:
		break;
	}
}

static void manager_event_notify(struct work_struct *data)
{
	struct typec_manager_event_work *event_work =
		container_of(data, struct typec_manager_event_work, typec_manager_work);
	int ret = 0;

	pr_info("%s: src:%s dest:%s id:%s sub1:%02x sub2:%02x sub3:%02x\n", __func__,
		pdic_event_src_string(event_work->event.src),
		pdic_event_dest_string(event_work->event.dest),
		pdic_event_id_string(event_work->event.id),
		event_work->event.sub1, event_work->event.sub2, event_work->event.sub3);

	switch (event_work->event.dest) {
	case CCIC_NOTIFY_DEV_BATTERY:
		if (event_work->event.sub3 == typec_manager.water.report_type
			&& manager_check_to_send_water_event(event_work->event))
			return;
		break;
	case CCIC_NOTIFY_DEV_DP:
	case CCIC_NOTIFY_DEV_USB_DP:
		manager_dp_state_change(event_work->event);
		break;
	default:
		break;
	}

#ifdef CONFIG_USB_NOTIFY_PROC_LOG
	if (event_work->event.id != CCIC_NOTIFY_ID_POWER_STATUS)
		store_usblog_notify(NOTIFY_MANAGER, (void *)&(event_work->event), NULL);
#endif

	ret = blocking_notifier_call_chain(&(typec_manager.manager_notifier),
				event_work->event.id, &(event_work->event));

	switch (ret) {
	case NOTIFY_DONE:
	case NOTIFY_OK:
		if (event_work->event.dest == CCIC_NOTIFY_DEV_USB
			&& event_work->event.sub2 == USB_STATUS_NOTIFY_ATTACH_UFP) {
			if (typec_manager.classified_cable_type == MANAGER_NOTIFY_MUIC_TIMEOUT_OPEN_DEVICE)
				manager_usb_enum_state_check(MIN_USB_DWORK_TIME);
			else
				manager_usb_enum_state_check(MAX_USB_DWORK_TIME);
		}
		pr_info("%s: notify done(0x%x)\n", __func__, ret);
		break;
	case NOTIFY_STOP_MASK:
	case NOTIFY_BAD:
	default:
		if (event_work->event.dest == CCIC_NOTIFY_DEV_USB)
			pr_info("%s: UPSM case (0x%x)\n", __func__, ret);
		else
			pr_info("%s: notify error occur(0x%x)\n", __func__, ret);
		break;
	}

	kfree(event_work);
}

static void manager_muic_event_notify(struct work_struct *data)
{
	struct typec_manager_event_work *event_work =
		container_of(data, struct typec_manager_event_work, typec_manager_work);
	int ret = 0;

	pr_info("%s: id:%s sub1:%02x sub2:%02x sub3:%02x\n", __func__,
		pdic_event_id_string(event_work->event.id),
		event_work->event.sub1, event_work->event.sub2, event_work->event.sub3);

#ifdef CONFIG_USB_NOTIFY_PROC_LOG
	store_usblog_notify(NOTIFY_MANAGER, (void *)&(event_work->event), NULL);
#endif
	ret = blocking_notifier_call_chain(&(typec_manager.manager_muic_notifier),
				event_work->event.id, &(event_work->event));

	pr_info("%s: done(0x%x)\n", __func__, ret);
	kfree(event_work);
}

static void manager_event_work(int src, int dest, int id, int sub1, int sub2, int sub3)
{
	struct typec_manager_event_work *event_work;

	pr_info("%s src:%s dest:%s\n", __func__,
		pdic_event_src_string(src), pdic_event_dest_string(dest));
	event_work = kmalloc(sizeof(struct typec_manager_event_work), GFP_ATOMIC);
	if (!event_work) {
		pr_err("%s: failed to alloc for event_work\n", __func__);
		return;
	}

	event_work->event.src = src;
	event_work->event.dest = dest;
	event_work->event.id = id;
	event_work->event.sub1 = sub1;
	event_work->event.sub2 = sub2;
	event_work->event.sub3 = sub3;
	event_work->event.pd = typec_manager.pd;

	if (event_work->event.dest == CCIC_NOTIFY_DEV_MUIC) {
		INIT_WORK(&event_work->typec_manager_work, manager_muic_event_notify);
		queue_work(typec_manager.manager_muic_noti_wq, &event_work->typec_manager_work);
	} else {
		INIT_WORK(&event_work->typec_manager_work, manager_event_notify);
		queue_work(typec_manager.manager_noti_wq, &event_work->typec_manager_work);
	}
}

void probe_typec_manager_gadget_ops(struct typec_manager_gadget_ops *ops)
{
	typec_manager.gadget_ops = ops;
}
EXPORT_SYMBOL(probe_typec_manager_gadget_ops);

static void manager_usb_event_send(uint state)
{
	if (typec_manager.manager_dr_state == state) {
		pr_info("%s(%s): Duplicate event\n", __func__, pdic_usbstatus_string(state));
		return;
	}

	switch (state) {
	case USB_STATUS_NOTIFY_ATTACH_UFP:
		if (typec_manager.classified_cable_type != MANAGER_NOTIFY_MUIC_USB
				&& typec_manager.classified_cable_type != MANAGER_NOTIFY_MUIC_TIMEOUT_OPEN_DEVICE
				&& typec_manager.classified_cable_type != MANAGER_NOTIFY_MUIC_OTG) {
			pr_info("%s(%s): Skip event (%s)\n", __func__, pdic_usbstatus_string(state),
				manager_notify_string(typec_manager.classified_cable_type));
			return;
		}
		break;
	case USB_STATUS_NOTIFY_ATTACH_DFP:
	case USB_STATUS_NOTIFY_DETACH:
		if (typec_manager.manager_dr_state == USB_STATUS_NOTIFY_ATTACH_UFP) {
			manager_usb_enum_state_check(CANCEL_USB_DWORK);
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
			manager_event_processing_by_vbus(false);
#endif
			set_usb_enumeration_state(0);
		}
		break;
	default:
		pr_info("%s(%s): Invalid event\n", __func__, pdic_usbstatus_string(state));
		return;
	}

	pr_info("%s(%s)\n", __func__, pdic_usbstatus_string(state));
	typec_manager.manager_dr_state = state;
	manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_USB,
		CCIC_NOTIFY_ID_USB, !!state, state, 0);
}

int __weak dwc3_gadget_get_cmply_link_state_wrapper(void)
{
	if (typec_manager.gadget_ops && typec_manager.gadget_ops->get_cmply_link_state) {
		void *dev = typec_manager.gadget_ops->driver_data;

		return typec_manager.gadget_ops->get_cmply_link_state(dev);
	}
	return -ENODEV;
}

static void manager_usb_enum_state_check_work(struct work_struct *work)
{
	int dwc3_link_check = 0;

	dwc3_link_check = dwc3_gadget_get_cmply_link_state_wrapper();

	if ((typec_manager.manager_dr_state != USB_STATUS_NOTIFY_ATTACH_UFP)
			|| (dwc3_link_check == 1)) {
		pr_info("%s: skip case : dwc3_link = %d\n", __func__, dwc3_link_check);
		return;
	}
	pr_info("%s: usb=0x%X, pd=%d dwc3_link=%d\n", __func__,
		typec_manager.usb_enum_state, typec_manager.pd_con_state, dwc3_link_check);

	if (!typec_manager.usb_enum_state) {
#ifdef CONFIG_USB_CONFIGFS_F_MBIM
		/* Make usb soft reconnection */
		pr_info("%s: For 5G module : Try usb reconnect here\n", __func__);
		manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		msleep(600);
		manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_UFP);
#else
		manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_MUIC,
			CCIC_NOTIFY_ID_USB, CCIC_NOTIFY_DETACH, USB_STATUS_NOTIFY_DETACH, 0);
#endif
	} else {
#ifndef CONFIG_USB_CONFIGFS_F_MBIM
		/* PD-USB cable Type */
		if (typec_manager.pd_con_state)
			manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_BATTERY,
				CCIC_NOTIFY_ID_USB, 0, 0, PD_USB_TYPE);
#endif
	}
}

static void manager_usb_enum_state_check(uint time_ms)
{
	if (typec_manager.usb_enable_state) {
		if (delayed_work_pending(&typec_manager.usbenumchk_work))
			cancel_delayed_work(&typec_manager.usbenumchk_work);

		if (time_ms)
			schedule_delayed_work(&typec_manager.usbenumchk_work, msecs_to_jiffies(time_ms));
	}
}

bool get_usb_enumeration_state(void)
{
	return typec_manager.usb_enum_state ? 1 : 0;
}
EXPORT_SYMBOL(get_usb_enumeration_state);

void set_usb_enumeration_state(int state)
{
	if (typec_manager.usb_enum_state != state) {
		typec_manager.usb_enum_state = state;
#if defined(CONFIG_USB_HW_PARAM)
		usb_enum_hw_param_data_update(typec_manager.usb_enum_state);
#endif
	}
}
EXPORT_SYMBOL(set_usb_enumeration_state);

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
static int manager_check_vbus_by_otg(void)
{
	int otg_power = 0;
#ifdef MANAGER_DEBUG
	unsigned long cur_stamp;
	int otg_power_time = 0;
#endif
#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
	union power_supply_propval val;

	psy_do_property("otg", get,
		POWER_SUPPLY_PROP_ONLINE, val);
	otg_power = val.intval;
#endif

	if (typec_manager.otg_stamp) {
#ifdef MANAGER_DEBUG
		cur_stamp = jiffies;
		otg_power_time = time_before(cur_stamp,
				typec_manager.otg_stamp+msecs_to_jiffies(OTG_VBUS_CHECK_TIME));
		pr_info("%s [OTG Accessory VBUS] duration-time=%u(ms), time_before(%d)\n", __func__,
			jiffies_to_msecs(cur_stamp-typec_manager.otg_stamp), otg_power_time);
		if (otg_power_time)
			typec_manager.vbus_by_otg_detection = 1;
#else
		if (time_before(jiffies, typec_manager.otg_stamp+msecs_to_jiffies(OTG_VBUS_CHECK_TIME)))
			typec_manager.vbus_by_otg_detection = 1;
#endif
		typec_manager.otg_stamp = 0;
	}

	otg_power |= typec_manager.vbus_by_otg_detection;

#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
	pr_info("%s otg power? %d (otg?%d, vbusTimeCheck?%d)\n", __func__,
		otg_power, val.intval, typec_manager.vbus_by_otg_detection);
#endif
	return otg_power;
}

static int manager_get_otg_power_mode(void)
{
	int otg_power = 0;
#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
	union power_supply_propval val;

	psy_do_property("otg", get,
		POWER_SUPPLY_PROP_ONLINE, val);
	otg_power = val.intval | typec_manager.vbus_by_otg_detection;

	pr_info("%s otg power? %d (otg?%d, vbusTimeCheck?%d)\n", __func__,
		otg_power, val.intval, typec_manager.vbus_by_otg_detection);
#endif
	return otg_power;
}
#endif

void set_usb_enable_state(void)
{
	if (!typec_manager.usb_enable_state) {
		typec_manager.usb_enable_state = true;
		if (typec_manager.manager_dr_state == USB_STATUS_NOTIFY_ATTACH_UFP)
			manager_usb_enum_state_check(BOOT_USB_DWORK_TIME);
	}
}
EXPORT_SYMBOL(set_usb_enable_state);

void manager_notifier_usbdp_support(void)
{

	if (typec_manager.dp.check_done == 1) {
		manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_USB_DP,
			CCIC_NOTIFY_ID_USB_DP, typec_manager.dp.is_connect, typec_manager.dp.hs_connect, 0);

		typec_manager.dp.check_done = 0;
	}
}
EXPORT_SYMBOL(manager_notifier_usbdp_support);

static void manager_set_alternate_mode(int listener)
{
	pccic_data_t ppdic_data;
	struct device *pdic_device = get_ccic_device();

	pr_info("%s : listener=%s(%d)\n", __func__,
			manager_notify_string(listener), listener);

	if (listener == MANAGER_NOTIFY_CCIC_BATTERY)
		typec_manager.alt_is_support |= PDIC_BATTERY;
	else if (listener == MANAGER_NOTIFY_CCIC_USB)
		typec_manager.alt_is_support |= PDIC_USB;
	else if (listener == MANAGER_NOTIFY_CCIC_DP)
		typec_manager.alt_is_support |= PDIC_DP;
	else if (listener == MANAGER_NOTIFY_CCIC_DELAY_DONE)
		typec_manager.alt_is_support |= PDIC_DELAY_DONE;
	else
		pr_info("no support driver to start alternate mode\n");

	if (IS_ERR(pdic_device)) {
		pr_err("%s: pdic_device is null.\n", __func__);
		return;
	}

	ppdic_data = dev_get_drvdata(pdic_device);
	if (!ppdic_data) {
		pr_err("there is no ppdic_data for set_enable_alternate_mode\n");
		return;
	}
	if (!ppdic_data->set_enable_alternate_mode) {
		pr_err("there is no set_enable_alternate_mode\n");
		return;
	}

	pr_info("%s : dp_is_support %d, alt_is_support %d\n", __func__,
			typec_manager.dp.is_support,
			typec_manager.alt_is_support);

	if (typec_manager.dp.is_support) {
		if (typec_manager.alt_is_support == (PDIC_DP|PDIC_USB|PDIC_BATTERY|PDIC_DELAY_DONE))
			ppdic_data->set_enable_alternate_mode(ALTERNATE_MODE_READY | ALTERNATE_MODE_START);
	} else {
		if (typec_manager.alt_is_support == (PDIC_USB|PDIC_BATTERY|PDIC_DELAY_DONE))
			ppdic_data->set_enable_alternate_mode(ALTERNATE_MODE_READY | ALTERNATE_MODE_START);
	}
}

static int manager_external_notifier_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int ret = 0;
	int enable = *(int *)data;

	switch (action) {
	case EXTERNAL_NOTIFY_DEVICEADD:
		pr_info("%s EXTERNAL_NOTIFY_DEVICEADD, enable=%d\n", __func__, enable);
		pr_info("dr_state: %s, muic.attach_state: %d\n",
			pdic_usbstatus_string(typec_manager.manager_dr_state),
			typec_manager.muic.attach_state);
		if (enable &&
#if !IS_ENABLED(CONFIG_CABLE_TYPE_NOTIFIER)
			typec_manager.muic.attach_state != MUIC_NOTIFY_CMD_DETACH &&
#endif
			typec_manager.manager_dr_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
			pr_info("%s: a usb device is added in host mode\n", __func__);
			/* USB Cable type */
			manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_BATTERY,
				CCIC_NOTIFY_ID_USB, 0, 0, PD_USB_TYPE);
		}
		break;
	case EXTERNAL_NOTIFY_POSSIBLE_USB:
		pr_info("%s EXTERNAL_NOTIFY_POSSIBLE_USB, enable=%d\n", __func__, enable);
		manager_set_alternate_mode(MANAGER_NOTIFY_CCIC_DELAY_DONE);
		break;
	default:
		break;
	}
	return ret;
}

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
static void manager_event_processing_by_vbus(bool run)
{
	if (delayed_work_pending(&typec_manager.chkforvbus_work))
		cancel_delayed_work(&typec_manager.chkforvbus_work);

	if (run) {
		if (typec_manager.manager_dr_state == USB_STATUS_NOTIFY_ATTACH_UFP)
			schedule_delayed_work(&typec_manager.chkforvbus_work,
				msecs_to_jiffies(VBUS_USB_OFF_TIMEOUT));
	}
}

static void manager_event_processing_by_vbus_work(struct work_struct *work)
{
	pr_info("%s: dr=%s, rid=%s\n", __func__,
		pdic_usbstatus_string(typec_manager.manager_dr_state),
		pdic_rid_string(typec_manager.pdic_rid_state));

	if (typec_manager.pdic_rid_state == RID_523K ||  typec_manager.pdic_rid_state == RID_619K
		|| typec_manager.classified_cable_type == MANAGER_NOTIFY_MUIC_UART
		|| typec_manager.manager_dr_state == USB_STATUS_NOTIFY_ATTACH_DFP
		|| typec_manager.vbus_state == STATUS_VBUS_HIGH) {
		return;
	} else if (typec_manager.muic.attach_state == MUIC_NOTIFY_CMD_DETACH) {
		manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		return;
	}

	manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_MUIC,
		CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH, 0, typec_manager.muic.cable_type);
}
#endif

static void manager_water_status_update(int status)
{
		if (status) {	/* attach */
			if (!typec_manager.water.detected) {
				typec_manager.water.detected = 1;
#if defined(CONFIG_USB_HW_PARAM)
				/*update water time */
				water_dry_time_update(status);
#endif

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
				mutex_lock(&typec_manager.mo_lock);
				if (typec_manager.vbus_state == STATUS_VBUS_HIGH
						&& !manager_get_otg_power_mode())
					manager_event_work(CCIC_NOTIFY_DEV_PDIC, CCIC_NOTIFY_DEV_BATTERY,
						CCIC_NOTIFY_ID_WATER, status, 0, typec_manager.water.report_type);
				mutex_unlock(&typec_manager.mo_lock);
#endif
			}
#if defined(CONFIG_SEC_ABC)
			sec_abc_send_event("MODULE=pdic@ERROR=water_det");
#endif
		} else {
			typec_manager.water.detected = 0;
			typec_manager.water.detOnPowerOff = 0;
#if defined(CONFIG_USB_HW_PARAM)
			/* update run_dry time */
			water_dry_time_update(status);
#endif
			if (typec_manager.water.wVbus_det)
				manager_event_work(CCIC_NOTIFY_DEV_PDIC, CCIC_NOTIFY_DEV_BATTERY,
					CCIC_NOTIFY_ID_WATER, status, 0, typec_manager.water.report_type);
		}
}

static int manager_handle_pdic_notification(struct notifier_block *nb,
				unsigned long action, void *data)
{
	MANAGER_NOTI_TYPEDEF p_noti = *(MANAGER_NOTI_TYPEDEF *)data;
	int ret = 0;

	pr_info("%s: src:%s dest:%s id:%s sub1:%02x sub2:%02x sub3:%02x\n", __func__,
		pdic_event_src_string(p_noti.src),
		pdic_event_dest_string(p_noti.dest),
		pdic_event_id_string(p_noti.id),
		p_noti.sub1, p_noti.sub2, p_noti.sub3);

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	if (p_noti.src != CCIC_NOTIFY_ID_INITIAL)
		manager_event_processing_by_vbus(false);
#endif

	switch (p_noti.id) {
	case CCIC_NOTIFY_ID_POWER_STATUS:
		if (p_noti.sub1 && !typec_manager.pd_con_state) {
			typec_manager.pd_con_state = 1;
#ifdef CONFIG_USB_NOTIFY_PROC_LOG
			store_usblog_notify(NOTIFY_MANAGER, (void *)&p_noti, NULL);
#endif
		}
		p_noti.dest = CCIC_NOTIFY_DEV_BATTERY;
		if (typec_manager.pd == NULL)
			typec_manager.pd = p_noti.pd;
		break;
	case CCIC_NOTIFY_ID_ATTACH:		// for MUIC
			if (typec_manager.pdic_attach_state != p_noti.sub1) {
				typec_manager.pdic_attach_state = p_noti.sub1;
				if (typec_manager.pdic_attach_state == CCIC_NOTIFY_ATTACH) {
					pr_info("%s: CCIC_NOTIFY_ATTACH\n", __func__);
					if (typec_manager.water.detected)
						manager_water_status_update(0);
					typec_manager.pd_con_state = 0;
					if (p_noti.sub2)
						typec_manager.otg_stamp = jiffies;
				}
			}

			if (typec_manager.pdic_attach_state == CCIC_NOTIFY_DETACH) {
				pr_info("%s: CCIC_NOTIFY_DETACH (pd=%d)\n", __func__,
					typec_manager.pd_con_state);
				if (typec_manager.pd_con_state) {
					typec_manager.pd_con_state = 0;
					manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_BATTERY,
						CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH, 0, ATTACHED_DEV_UNOFFICIAL_ID_ANY_MUIC);
				}
			}
		break;
	case CCIC_NOTIFY_ID_RID:
		typec_manager.pdic_rid_state = p_noti.sub1;
		break;
	case CCIC_NOTIFY_ID_USB:
		manager_usb_event_send(p_noti.sub2);
		return 0;
	case CCIC_NOTIFY_ID_WATER:
		if (p_noti.sub1) {	/* attach */
			if (!typec_manager.water.detected)
				manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_MUIC,
					CCIC_NOTIFY_ID_WATER, p_noti.sub1, p_noti.sub2, p_noti.sub3);
			manager_water_status_update(p_noti.sub1);
		} else {
			manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_MUIC,
					CCIC_NOTIFY_ID_WATER, p_noti.sub1, p_noti.sub2, p_noti.sub3);
			manager_water_status_update(p_noti.sub1);
		}
		return 0;
	case CCIC_NOTIFY_ID_POFF_WATER:
		if (p_noti.sub1) { /* power off water detect */
			typec_manager.water.detOnPowerOff = 1;
			pr_info("%s: power off water case.\n", __func__);
			return 0;
		} else
			return 0;
	case CCIC_NOTIFY_ID_WATER_CABLE:
		/* Ignore no water case */
		if (!typec_manager.water.detected)
			return 0;

		manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_MUIC,
			p_noti.id, p_noti.sub1, p_noti.sub2, p_noti.sub3);

		if (p_noti.sub1) {
			/* Send water cable event to battery */
			manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_BATTERY,
					CCIC_NOTIFY_ID_WATER, CCIC_NOTIFY_ATTACH, p_noti.sub2,
					typec_manager.water.report_type);

			/* make detach event like hiccup case*/
			manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_BATTERY,
					CCIC_NOTIFY_ID_WATER, CCIC_NOTIFY_DETACH, p_noti.sub2,
					typec_manager.water.report_type);
		}
		return 0;
	case CCIC_NOTIFY_ID_INITIAL:
		return 0;
	default:
		break;
	}

	manager_event_work(p_noti.src, p_noti.dest,
		p_noti.id, p_noti.sub1, p_noti.sub2, p_noti.sub3);

	return ret;
}

static int manager_handle_muic_notification(struct notifier_block *nb,
				unsigned long action, void *data)
{
	CC_NOTI_ATTACH_TYPEDEF p_noti = *(CC_NOTI_ATTACH_TYPEDEF *)data;

	pr_info("%s: src:%d attach:%d, cable_type:%d\n", __func__,
		p_noti.src, p_noti.attach, p_noti.cable_type);

	switch (p_noti.src) {
#ifdef CONFIG_USE_DEDICATED_MUIC
	case CCIC_NOTIFY_DEV_DEDICATED_MUIC:
#if defined(CONFIG_PDIC_SWITCH)
		{
			bool usb_en = false;
			bool is_jig_board = !!get_pdic_info();
			bool skip = false;

			switch (p_noti.cable_type) {
			case ATTACHED_DEV_JIG_UART_OFF_MUIC:
			case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
			case ATTACHED_DEV_JIG_UART_ON_MUIC:
				is_factory_jig = true;
				break;
			case ATTACHED_DEV_JIG_USB_OFF_MUIC:
			case ATTACHED_DEV_JIG_USB_ON_MUIC:
				is_factory_jig = true;
				usb_en = true;
				break;
			case ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC:
			case ATTACHED_DEV_AFC_CHARGER_5V_MUIC:
			case ATTACHED_DEV_AFC_CHARGER_9V_MUIC:
			case ATTACHED_DEV_QC_CHARGER_PREPARE_MUIC:
			case ATTACHED_DEV_QC_CHARGER_5V_MUIC:
			case ATTACHED_DEV_QC_CHARGER_9V_MUIC:
				skip = true;
				break;
			default:
				if (!is_factory_jig && !is_jig_board)
					usb_en = true;
				break;
			}

			if (!skip) {
				if (usb_en)
					manager_event_work(CCIC_NOTIFY_DEV_MANAGER,
						CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
						CCIC_NOTIFY_ATTACH, USB_STATUS_NOTIFY_ATTACH_UFP, 0);
				else
					manager_event_work(CCIC_NOTIFY_DEV_MANAGER,
						CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
						CCIC_NOTIFY_DETACH, USB_STATUS_NOTIFY_DETACH, 0);
			}
		}
#endif
		manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_SUB_BATTERY,
			p_noti.id, p_noti.attach, p_noti.rprd, p_noti.cable_type);
		return 0;
#endif
#ifdef CONFIG_USE_SECOND_MUIC
	case CCIC_NOTIFY_DEV_SECOND_MUIC:
		typec_manager.second_muic.attach_state = p_noti.attach;
		typec_manager.second_muic.cable_type = p_noti.cable_type;
		manager_event_work(p_noti.src, CCIC_NOTIFY_DEV_SUB_BATTERY,
			p_noti.id, p_noti.attach, p_noti.rprd, p_noti.cable_type);
		return 0;
#endif
	case CCIC_NOTIFY_DEV_MUIC:
	default:
		typec_manager.muic.attach_state = p_noti.attach;
		typec_manager.muic.cable_type = p_noti.cable_type;
		break;
	}

	if (typec_manager.water.detected) {
		pr_info("%s: Just returned because the moisture was detected\n", __func__);
		return 0;
	}

	if (!p_noti.attach)
		typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_NONE;

	switch (p_noti.cable_type) {
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		pr_info("%s: JIG USB(%d) %s, PDIC: %s\n", __func__,
			p_noti.cable_type, p_noti.attach ? "Attached" : "Detached",
			typec_manager.pdic_attach_state ? "Attached" : "Detached");

		if (p_noti.attach) {
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_USB;
			manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_UFP);
		} else
			manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		break;

	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_USB_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC:
		pr_info("%s: USB(%d) %s, PDIC: %s\n", __func__,
			p_noti.cable_type, p_noti.attach ? "Attached" : "Detached",
			typec_manager.pdic_attach_state ? "Attached" : "Detached");

		if (p_noti.attach) {
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_USB;
			if (typec_manager.pdic_attach_state)
				manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_UFP);
		} else
			if (!typec_manager.pdic_attach_state)
				manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		break;

	case ATTACHED_DEV_OTG_MUIC:
		if (p_noti.attach)
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_OTG;
		break;

	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_FG_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
		if (p_noti.attach)
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_UART;
		break;

	case ATTACHED_DEV_TA_MUIC:
		pr_info("%s: TA(%d) %s\n", __func__, p_noti.cable_type,
			p_noti.attach ? "Attached" : "Detached");

		if (p_noti.attach)
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_CHARGER;
		break;

	case ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC:
	case ATTACHED_DEV_QC_CHARGER_PREPARE_MUIC:
		pr_info("%s: AFC or QC Prepare(%d) %s\n", __func__,
			p_noti.cable_type, p_noti.attach ? "Attached" : "Detached");

		if (p_noti.attach)
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_CHARGER;
		break;

	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
		pr_info("%s: DCD Timeout device is detected(%d) %s, cable:%s pdic:%s\n",
			__func__, p_noti.cable_type,
			p_noti.attach ? "Attached" : "Detached",
			pdic_usbstatus_string(typec_manager.manager_dr_state),
			typec_manager.pdic_attach_state ? "Attached" : "Detached");

		if (p_noti.attach) {
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_TIMEOUT_OPEN_DEVICE;
			if (typec_manager.manager_dr_state != USB_STATUS_NOTIFY_ATTACH_DFP)
				manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_UFP);
		} else {
			if (!typec_manager.pdic_attach_state)
				manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		}
		break;

	default:
		pr_info("%s: Cable(%d) %s\n", __func__, p_noti.cable_type,
			p_noti.attach ? "Attached" : "Detached");
		break;
	}

	if (!(p_noti.attach) && typec_manager.pd_con_state &&
			p_noti.cable_type != typec_manager.water.report_type) {
		pr_info("%s: Don't send the MUIC detach event when the PD charger is connected\n", __func__);
	} else
		manager_event_work(CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_DEV_BATTERY,
			p_noti.id, p_noti.attach, p_noti.rprd, p_noti.cable_type);

	return 0;
}

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
static int manager_handle_vbus_notification(struct notifier_block *nb,
				unsigned long action, void *data)
{
	vbus_status_t vbus_type = *(vbus_status_t *)data;

	mutex_lock(&typec_manager.mo_lock);
	pr_info("%s: cmd=%lu, vbus_type=%s, WATER DET=%d ATTACH=%s\n", __func__,
		action, vbus_type == STATUS_VBUS_HIGH ? "HIGH" : "LOW", typec_manager.water.detected,
		typec_manager.pdic_attach_state == CCIC_NOTIFY_ATTACH ? "ATTACH":"DETACH");

	typec_manager.vbus_state = vbus_type;

	switch (vbus_type) {
	case STATUS_VBUS_HIGH:
		if (!manager_check_vbus_by_otg() && typec_manager.water.detected)
			manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_BATTERY,
				CCIC_NOTIFY_ID_WATER, CCIC_NOTIFY_ATTACH, 0, typec_manager.water.report_type);
		break;
	case STATUS_VBUS_LOW:
		typec_manager.vbus_by_otg_detection = 0;
		if (typec_manager.water.wVbus_det)
			manager_event_work(CCIC_NOTIFY_DEV_MANAGER, CCIC_NOTIFY_DEV_BATTERY,
				CCIC_NOTIFY_ID_ATTACH, CCIC_NOTIFY_DETACH, 0, typec_manager.water.report_type);
		manager_event_processing_by_vbus(true);
		break;
	default:
		break;
	}

	mutex_unlock(&typec_manager.mo_lock);
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_CABLE_TYPE_NOTIFIER)
static int manager_cable_type_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	cable_type_attached_dev_t attached_dev = *(cable_type_attached_dev_t *)data;

	pr_info("%s action=%lu, attached_dev=%d\n",
		__func__, action, attached_dev);

	if (!action)
		typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_NONE;

	switch (attached_dev) {
	case CABLE_TYPE_USB:
	case CABLE_TYPE_USB_SDP:
	case CABLE_TYPE_USB_CDP:
		if (action) {
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_USB;
			manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_UFP);
		} else
			manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		break;
	case CABLE_TYPE_OTG:
		if (action) {
			typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_OTG;
			manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_DFP);
		} else
			manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		break;
	default:
		manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
		break;
	}

	return 0;
}
#endif


int manager_notifier_register(struct notifier_block *nb, notifier_fn_t notifier,
			manager_notifier_device_t listener)
{
	int ret = 0;
	MANAGER_NOTI_TYPEDEF m_noti = {0, };
#if defined(CONFIG_USB_HW_PARAM)
	struct otg_notify *o_notify = get_otg_notify();
#endif

	pr_info("%s: listener=%s(%d) register\n", __func__,
			manager_notify_string(listener), listener);

#if IS_BUILTIN(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	if (!manager_notifier_init_done)
		manager_notifier_init();

	ccic_notifier_init();
#endif

	/* Check if MANAGER Notifier is ready. */
	if (!manager_device) {
		pr_err("%s: Not Initialized...\n", __func__);
		return -1;
	}

	if (listener == MANAGER_NOTIFY_CCIC_MUIC) {
		SET_MANAGER_NOTIFIER_BLOCK(nb, notifier, listener);
		ret = blocking_notifier_chain_register(&(typec_manager.manager_muic_notifier), nb);
		if (ret < 0)
			pr_err("%s: muic blocking_notifier_chain_register error(%d)\n",
					__func__, ret);
	} else {
		SET_MANAGER_NOTIFIER_BLOCK(nb, notifier, listener);
		ret = blocking_notifier_chain_register(&(typec_manager.manager_notifier), nb);
		if (ret < 0)
			pr_err("%s: manager blocking_notifier_chain_register error(%d)\n",
					__func__, ret);
	}

	switch (listener) {
	case MANAGER_NOTIFY_CCIC_BATTERY:
#if !IS_ENABLED(CONFIG_CABLE_TYPE_NOTIFIER)
		m_noti.src = CCIC_NOTIFY_DEV_MANAGER;
		m_noti.dest = CCIC_NOTIFY_DEV_BATTERY;
		m_noti.pd = typec_manager.pd;
		if (typec_manager.water.detected) {
			if (typec_manager.muic.attach_state
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
				|| typec_manager.water.detOnPowerOff
				|| typec_manager.vbus_state == STATUS_VBUS_HIGH
#endif
			) {
				pr_info("%s: [BATTERY] power_off_water_det:%d , vbus_state:%d\n", __func__,
					typec_manager.water.detOnPowerOff, typec_manager.vbus_state);

				m_noti.id = CCIC_NOTIFY_ID_WATER;
				m_noti.sub1 = CCIC_NOTIFY_ATTACH;
				m_noti.sub3 = typec_manager.water.report_type;
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
				if (typec_manager.vbus_state != STATUS_VBUS_HIGH) {
					nb->notifier_call(nb, m_noti.id, &(m_noti));
					m_noti.sub1 = CCIC_NOTIFY_DETACH;
				}
#endif
			}
		} else {
			m_noti.id = CCIC_NOTIFY_ID_ATTACH;
			if (typec_manager.pd_con_state) {
				m_noti.id = CCIC_NOTIFY_ID_POWER_STATUS;
				m_noti.sub1 = CCIC_NOTIFY_ATTACH;
			} else if (typec_manager.muic.attach_state) {
				m_noti.sub1 = CCIC_NOTIFY_ATTACH;
				m_noti.sub3 = typec_manager.muic.cable_type;
			}
		}
		pr_info("%s: [BATTERY] id:%s, cable_type=%d %s\n", __func__,
			pdic_event_id_string(m_noti.id),
			m_noti.sub3, m_noti.sub1 ? "Attached" : "Detached");
		nb->notifier_call(nb, m_noti.id, &(m_noti));
#else
		pr_info("%s: [BATTERY] Registration completed\n", __func__);
#endif
		manager_set_alternate_mode(listener);
		break;
	case MANAGER_NOTIFY_CCIC_SUB_BATTERY:
		m_noti.src = CCIC_NOTIFY_DEV_MANAGER;
		m_noti.dest = CCIC_NOTIFY_DEV_SUB_BATTERY;
		m_noti.id = CCIC_NOTIFY_ID_ATTACH;
#ifdef CONFIG_USE_SECOND_MUIC
		if (typec_manager.second_muic.attach_state) {
			m_noti.sub1 = CCIC_NOTIFY_ATTACH;
			m_noti.sub3 = typec_manager.second_muic.cable_type;
		}
#endif
		pr_info("%s: [BATTERY2] cable_type=%d %s\n", __func__,
			m_noti.sub3, m_noti.sub1 ? "Attached" : "Detached");
		nb->notifier_call(nb, m_noti.id, &(m_noti));
		break;
	case MANAGER_NOTIFY_CCIC_USB:
		m_noti.src = CCIC_NOTIFY_DEV_MANAGER;
		m_noti.dest = CCIC_NOTIFY_DEV_USB;
		m_noti.id = CCIC_NOTIFY_ID_USB;
		if (typec_manager.manager_dr_state) {
			m_noti.sub1 = CCIC_NOTIFY_ATTACH;
			m_noti.sub2 = typec_manager.manager_dr_state;
		} else if (typec_manager.classified_cable_type == MANAGER_NOTIFY_MUIC_USB) {
			m_noti.sub1 = CCIC_NOTIFY_ATTACH;
			typec_manager.manager_dr_state = USB_STATUS_NOTIFY_ATTACH_UFP;
			m_noti.sub2 = USB_STATUS_NOTIFY_ATTACH_UFP;
		}
		pr_info("%s: [USB] %s\n", __func__, pdic_usbstatus_string(m_noti.sub2));
		nb->notifier_call(nb, m_noti.id, &(m_noti));
		manager_set_alternate_mode(listener);
#if defined(CONFIG_USB_HW_PARAM)
		if (o_notify)
			register_hw_param_manager(o_notify, manager_hw_param_update);
#endif
		break;
	case MANAGER_NOTIFY_CCIC_DP:
		pr_info("%s: [DP] %d\n", __func__,
			typec_manager.dp.attach_state);
		m_noti.src = CCIC_NOTIFY_DEV_MANAGER;
		m_noti.dest = CCIC_NOTIFY_DEV_DP;
		if (typec_manager.dp.attach_state == CCIC_NOTIFY_ATTACH) {
			m_noti.id = CCIC_NOTIFY_ID_DP_CONNECT;
			m_noti.sub1 = typec_manager.dp.attach_state;
			nb->notifier_call(nb, m_noti.id, &(m_noti));

			m_noti.id = CCIC_NOTIFY_ID_DP_LINK_CONF;
			m_noti.sub1 = typec_manager.dp.cable_type;
			nb->notifier_call(nb, m_noti.id, &(m_noti));

			if (typec_manager.dp.hpd_state == CCIC_NOTIFY_HIGH) {
				m_noti.id = CCIC_NOTIFY_ID_DP_HPD;
				m_noti.sub1 = typec_manager.dp.hpd_state;
				nb->notifier_call(nb, m_noti.id, &(m_noti));
			}
		}
		manager_set_alternate_mode(listener);
		break;
	default:
		break;
	}

	return ret;
}
EXPORT_SYMBOL(manager_notifier_register);

int manager_notifier_unregister(struct notifier_block *nb)
{
	int ret = 0;

	pr_info("%s: listener=%d unregister\n", __func__, nb->priority);

	if (nb->priority == MANAGER_NOTIFY_CCIC_MUIC) {
		ret = blocking_notifier_chain_unregister(&(typec_manager.manager_muic_notifier), nb);
		if (ret < 0)
			pr_err("%s: muic blocking_notifier_chain_unregister error(%d)\n",
					__func__, ret);
		DESTROY_MANAGER_NOTIFIER_BLOCK(nb);
	} else {
		ret = blocking_notifier_chain_unregister(&(typec_manager.manager_notifier), nb);
		if (ret < 0)
			pr_err("%s: pdic blocking_notifier_chain_unregister error(%d)\n",
					__func__, ret);
		DESTROY_MANAGER_NOTIFIER_BLOCK(nb);
	}
	return ret;
}
EXPORT_SYMBOL(manager_notifier_unregister);

static void delayed_manger_notifier_init(struct work_struct *work)
{
	int ret = 0;
	int notifier_result = 0;
	static int retry_count = 1;
	int max_retry_count = 5;

	pr_info("%s : %d = times!\n", __func__, retry_count);
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	if (confirm_manager_notifier_register & (1 << VBUS_NOTIFIER)) {
		ret = vbus_notifier_register(&typec_manager.vbus_nb,
				manager_handle_vbus_notification, VBUS_NOTIFY_DEV_MANAGER);
		if (ret)
			notifier_result |= (1 << VBUS_NOTIFIER);
	}
#endif
	if (confirm_manager_notifier_register & (1 << PDIC_NOTIFIER)) {
		ret = ccic_notifier_register(&typec_manager.pdic_nb,
				manager_handle_pdic_notification, CCIC_NOTIFY_DEV_MANAGER);
		if (ret)
			notifier_result |= (1 << PDIC_NOTIFIER);
	}

	if (confirm_manager_notifier_register & (1 << MUIC_NOTIFIER)) {
		ret = muic_notifier_register(&typec_manager.muic_nb,
				manager_handle_muic_notification, MUIC_NOTIFY_DEV_MANAGER);
		if (ret)
			notifier_result |= (1 << MUIC_NOTIFIER);
	}

	confirm_manager_notifier_register = notifier_result;
	pr_info("%s : result of register = %d!\n",
		__func__, confirm_manager_notifier_register);

	if (confirm_manager_notifier_register) {
		pr_err("Manager notifier init time is %d.\n", retry_count);
		if (retry_count++ != max_retry_count)
			schedule_delayed_work(&typec_manager.manager_init_work,
				msecs_to_jiffies(NOTIFIER_REG_RETRY));
		else
			pr_err("fail to init manager notifier\n");
	} else
		pr_info("%s : done!\n", __func__);
}

static void managet_dp_status_init(void)
{
	struct device_node *np = NULL;

	typec_manager.dp.is_connect = 0;
	typec_manager.dp.hs_connect = 0;
	typec_manager.dp.check_done = 1;

	np = of_find_node_by_name(NULL, "displayport");
	if (!of_get_property(np, "dp,displayport_not_support", NULL)) {
		pr_info("%s: usb_host: support DP\n", __func__);
		typec_manager.dp.is_support = 1;
	} else {
		pr_info("%s: usb_host: no support DP\n", __func__);
		typec_manager.dp.is_support = 0;
	}
}

#if defined(CONFIG_SEC_FACTORY)
static ssize_t fac_usb_control_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	char *cmd;
	int ret;

	cmd = kzalloc(size+1, GFP_KERNEL);
	if (!cmd)
		goto error2;

	ret = sscanf(buf, "%s", cmd);
	if (ret != 1)
		goto error1;

	pr_info("%s cmd=%s\n", __func__, cmd);
	if (!strcmp(cmd, "Off_All"))
		manager_usb_event_send(USB_STATUS_NOTIFY_DETACH);
	else if (!strcmp(cmd, "On_DEVICE"))
		manager_usb_event_send(USB_STATUS_NOTIFY_ATTACH_UFP);

	strncpy(typec_manager.fac_control,
		cmd, sizeof(typec_manager.fac_control)-1);

error1:
	kfree(cmd);
error2:
	return size;
}

static ssize_t fac_usb_control_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("%s typec_manager.fac_control %s\n",
		__func__, typec_manager.fac_control);
	return sprintf(buf, "%s\n", typec_manager.fac_control);
}

static DEVICE_ATTR(fac_usb_control, 0664,
		fac_usb_control_show, fac_usb_control_store);

static struct attribute *typec_manager_attributes[] = {
	&dev_attr_fac_usb_control.attr,
	NULL
};

const struct attribute_group typec_manager_sysfs_group = {
	.attrs = typec_manager_attributes,
};
#endif

static int manager_notifier_init(void)
{
	struct device *pdic_device = get_ccic_device();
	pccic_data_t ppdic_data = NULL;
	int ret = 0;
	int notifier_result = 0;

	pr_info("%s (ver %d.%d)\n", __func__,
		TYPEC_MANAGER_MAJ_VERSION,
		TYPEC_MANAGER_MIN_VERSION);

	if (IS_ERR(pdic_device))
		pr_err("%s: pdic_device is null.\n", __func__);
	else
		ppdic_data = dev_get_drvdata(pdic_device);

#if IS_BUILTIN(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	ccic_notifier_init();
#endif

	if (manager_notifier_init_done) {
		pr_err("%s already registered\n", __func__);
		goto out;
	}

	manager_notifier_init_done = 1;

#if IS_ENABLED(CONFIG_DRV_SAMSUNG)
	manager_device = sec_device_create(NULL, "typec_manager");
#endif

	if (IS_ERR(manager_device)) {
		pr_err("%s Failed to create device(switch)!\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	typec_manager.pdic_attach_state = CCIC_NOTIFY_DETACH;
	typec_manager.manager_dr_state = USB_STATUS_NOTIFY_DETACH;
	typec_manager.muic.attach_state = MUIC_NOTIFY_CMD_DETACH;
	typec_manager.muic.cable_type = ATTACHED_DEV_NONE_MUIC;
#ifdef CONFIG_USE_SECOND_MUIC
	typec_manager.second_muic.attach_state = MUIC_NOTIFY_CMD_DETACH;
	typec_manager.second_muic.cable_type = ATTACHED_DEV_NONE_MUIC;
#endif
	typec_manager.classified_cable_type = MANAGER_NOTIFY_MUIC_NONE;
	typec_manager.usb_enum_state = 0;
	typec_manager.otg_stamp = 0;
	typec_manager.vbus_by_otg_detection = 0;
	typec_manager.water.detected = 0;
	typec_manager.water.wVbus_det = 0;
	typec_manager.water.detOnPowerOff = 0;
	typec_manager.alt_is_support = 0;
	strncpy(typec_manager.fac_control,
		"On_All", sizeof(typec_manager.fac_control)-1);

#if defined(CONFIG_USB_HW_PARAM)
	manager_hw_param_init();
#endif
	managet_dp_status_init();

	usb_external_notify_register(&typec_manager.manager_external_notifier_nb,
		manager_external_notifier_notification, EXTERNAL_NOTIFY_DEV_MANAGER);
	typec_manager.vbus_state = 0;
	typec_manager.pdic_rid_state = RID_UNDEFINED;
	typec_manager.pd = NULL;
#if (IS_ENABLED(CONFIG_HICCUP_CHARGER) || IS_ENABLED(CONFIG_SEC_HICCUP)) && IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
	typec_manager.water.report_type = lpcharge ?
		ATTACHED_DEV_UNDEFINED_RANGE_MUIC :
		ATTACHED_DEV_HICCUP_MUIC;
#else
	typec_manager.water.report_type = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#endif

	typec_manager.manager_noti_wq =
		create_singlethread_workqueue("typec_manager_event");
	typec_manager.manager_muic_noti_wq =
		create_singlethread_workqueue("typec_manager_muic_event");

	BLOCKING_INIT_NOTIFIER_HEAD(&(typec_manager.manager_notifier));
	BLOCKING_INIT_NOTIFIER_HEAD(&(typec_manager.manager_muic_notifier));

	INIT_DELAYED_WORK(&typec_manager.manager_init_work,
		delayed_manger_notifier_init);

	INIT_DELAYED_WORK(&typec_manager.usbenumchk_work,
		manager_usb_enum_state_check_work);

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	INIT_DELAYED_WORK(&typec_manager.chkforvbus_work,
		manager_event_processing_by_vbus_work);
#endif

	if (ppdic_data && ppdic_data->set_enable_alternate_mode)
		ppdic_data->set_enable_alternate_mode(ALTERNATE_MODE_NOT_READY);

	mutex_init(&typec_manager.mo_lock);

	/*
	 * Register manager handler to pdic notifier block list
	 */
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	ret = vbus_notifier_register(&typec_manager.vbus_nb,
			manager_handle_vbus_notification, VBUS_NOTIFY_DEV_MANAGER);
	if (ret)
		notifier_result |= (1 << VBUS_NOTIFIER);
#endif
	ret = ccic_notifier_register(&typec_manager.pdic_nb,
			manager_handle_pdic_notification, CCIC_NOTIFY_DEV_MANAGER);
	if (ret)
		notifier_result |= (1 << PDIC_NOTIFIER);
#if IS_ENABLED(CONFIG_CABLE_TYPE_NOTIFIER)
	ret = cable_type_notifier_register(&typec_manager.cable_type_nb,
			manager_cable_type_handle_notification, CABLE_TYPE_NOTIFY_DEV_USB);
#else
	ret = muic_notifier_register(&typec_manager.muic_nb,
			manager_handle_muic_notification, MUIC_NOTIFY_DEV_MANAGER);
#endif
	if (ret)
		notifier_result |= (1 << MUIC_NOTIFIER);


	confirm_manager_notifier_register = notifier_result;
	pr_info("%s : result of register = %d\n",
		__func__, confirm_manager_notifier_register);

	if (confirm_manager_notifier_register)
		schedule_delayed_work(&typec_manager.manager_init_work,
			msecs_to_jiffies(NOTIFIER_REG_RETRY));

#if defined(CONFIG_DRV_SAMSUNG) && defined(CONFIG_SEC_FACTORY)
	/* create sysfs group */
	ret = sysfs_create_group(&manager_device->kobj, &typec_manager_sysfs_group);
#endif

	pr_info("%s end\n", __func__);
out:
	return ret;
}

static void __exit manager_notifier_exit(void)
{
	pr_info("%s exit\n", __func__);
	mutex_destroy(&typec_manager.mo_lock);
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_unregister(&typec_manager.vbus_nb);
#endif
	pdic_notifier_unregister(&typec_manager.pdic_nb);
#if IS_ENABLED(CONFIG_CABLE_TYPE_NOTIFIER)
	cable_type_notifier_unregister(&typec_manager.cable_type_nb);
#else
	muic_notifier_unregister(&typec_manager.muic_nb);
#endif
	usb_external_notify_unregister(&typec_manager.manager_external_notifier_nb);
}

device_initcall(manager_notifier_init);
module_exit(manager_notifier_exit);

MODULE_AUTHOR("Samsung USB Team");
MODULE_DESCRIPTION("USB Typec Manager Notifier");
MODULE_LICENSE("GPL");
