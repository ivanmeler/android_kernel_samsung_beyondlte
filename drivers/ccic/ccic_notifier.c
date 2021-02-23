#include <linux/device.h>
#include <linux/module.h>

#include <linux/notifier.h>
#include <linux/ccic/ccic_notifier.h>
#include <linux/sec_class.h>
#include <linux/ccic/ccic_sysfs.h>
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
#include <linux/battery/battery_notifier.h>
#endif
#include <linux/usb_notify.h>
#include <linux/ccic/ccic_core.h>

#define DEBUG
#define SET_CCIC_NOTIFIER_BLOCK(nb, fn, dev) do {	\
		(nb)->notifier_call = (fn);		\
		(nb)->priority = (dev);			\
	} while (0)

#define DESTROY_CCIC_NOTIFIER_BLOCK(nb)			\
		SET_CCIC_NOTIFIER_BLOCK(nb, NULL, -1)

static struct ccic_notifier_struct ccic_notifier;

extern struct device *ccic_device;
static int ccic_notifier_init_done;

const char *pdic_event_src_string(ccic_notifier_device_t src)
{
	/* enum pdic_notifier_device */
	switch (src) {
	case CCIC_NOTIFY_DEV_INITIAL:
		return "INITIAL";
	case CCIC_NOTIFY_DEV_USB:
		return "USB";
	case CCIC_NOTIFY_DEV_BATTERY:
		return "BATTERY";
	case CCIC_NOTIFY_DEV_PDIC:
		return "PDIC";
	case CCIC_NOTIFY_DEV_MUIC:
		return "MUIC";
	case CCIC_NOTIFY_DEV_CCIC:
		return "CCIC";
	case CCIC_NOTIFY_DEV_MANAGER:
		return "MANAGER";
	case CCIC_NOTIFY_DEV_DP:
		return "DP";
	case CCIC_NOTIFY_DEV_USB_DP:
		return "USBDP";
	case CCIC_NOTIFY_DEV_SUB_BATTERY:
		return "BATTERY2";
	case CCIC_NOTIFY_DEV_SECOND_MUIC:
		return "MUIC2";
	case CCIC_NOTIFY_DEV_DEDICATED_MUIC:
		return "DMUIC";
	case CCIC_NOTIFY_DEV_ALL:
		return "ALL";
	default:
		return "UNDEFINED";
	}
}
EXPORT_SYMBOL(pdic_event_src_string);

const char *pdic_event_dest_string(ccic_notifier_device_t dest)
{
	return pdic_event_src_string(dest);
}
EXPORT_SYMBOL(pdic_event_dest_string);

const char *pdic_event_id_string(ccic_notifier_id_t id)
{
	/* enum pdic_notifier_id_t */
	switch (id) {
	case CCIC_NOTIFY_ID_INITIAL:
		return "ID_INITIAL";
	case CCIC_NOTIFY_ID_ATTACH:
		return "ID_ATTACH";
	case CCIC_NOTIFY_ID_RID:
		return "ID_RID";
	case CCIC_NOTIFY_ID_USB:
		return "ID_USB";
	case CCIC_NOTIFY_ID_POWER_STATUS:
		return "ID_POWER_STATUS";
	case CCIC_NOTIFY_ID_WATER:
		return "ID_WATER";
	case CCIC_NOTIFY_ID_VCONN:
		return "ID_VCONN";
	case CCIC_NOTIFY_ID_OTG:
		return "ID_OTG";
	case CCIC_NOTIFY_ID_TA:
		return "ID_TA";
	case CCIC_NOTIFY_ID_DP_CONNECT:
		return "ID_DP_CONNECT";
	case CCIC_NOTIFY_ID_DP_HPD:
		return "ID_DP_HPD";
	case CCIC_NOTIFY_ID_DP_LINK_CONF:
		return "ID_DP_LINK_CONF";
	case CCIC_NOTIFY_ID_USB_DP:
		return "ID_USB_DP";
	case CCIC_NOTIFY_ID_ROLE_SWAP:
		return "ID_ROLE_SWAP";
	case CCIC_NOTIFY_ID_FAC:
		return "ID_FAC";
	case CCIC_NOTIFY_ID_CC_PIN_STATUS:
		return "ID_PIN_STATUS";
	case CCIC_NOTIFY_ID_WATER_CABLE:
		return "ID_WATER_CABLE";
	case CCIC_NOTIFY_ID_POFF_WATER:
		return "ID_POFF_WATER";
	default:
		return "UNDEFINED";
	}
}
EXPORT_SYMBOL(pdic_event_id_string);

const char *pdic_rid_string(ccic_notifier_rid_t rid)
{
	switch (rid) {
	case RID_UNDEFINED:
		return "RID_UNDEFINED";
	case RID_000K:
		return "RID_000K";
	case RID_001K:
		return "RID_001K";
	case RID_255K:
		return "RID_255K";
	case RID_301K:
		return "RID_301K";
	case RID_523K:
		return "RID_523K";
	case RID_619K:
		return "RID_619K";
	case RID_OPEN:
		return "RID_OPEN";
	default:
		return "RID_UNDEFINED";
	}
}
EXPORT_SYMBOL(pdic_rid_string);

const char *pdic_usbstatus_string(USB_STATUS usbstatus)
{
	switch (usbstatus) {
	case USB_STATUS_NOTIFY_DETACH:
		return "USB_DETACH";
	case USB_STATUS_NOTIFY_ATTACH_DFP:
		return "USB_ATTACH_DFP";
	case USB_STATUS_NOTIFY_ATTACH_UFP:
		return "USB_ATTACH_UFP";
	case USB_STATUS_NOTIFY_ATTACH_DRP:
		return "USB_ATTACH_DRP";
	default:
		return "UNDEFINED";
	}
}
EXPORT_SYMBOL(pdic_usbstatus_string);

const char *pdic_ccpinstatus_string(ccic_notifier_pin_status_t ccpinstatus)
{
	switch (ccpinstatus) {
	case CCIC_NOTIFY_PIN_STATUS_NO_DETERMINATION:
		return "NO_DETERMINATION";
	case CCIC_NOTIFY_PIN_STATUS_CC1_ACTIVE:
		return "CC1_ACTIVE";
	case CCIC_NOTIFY_PIN_STATUS_CC2_ACTIVE:
		return "CC2_ACTIVE";
	case CCIC_NOTIFY_PIN_STATUS_AUDIO_ACCESSORY:
		return "AUDIO_ACCESSORY";
	case CCIC_NOTIFY_PIN_STATUS_DEBUG_ACCESSORY:
		return "DEBUG_ACCESSORY";
	case CCIC_NOTIFY_PIN_STATUS_CCIC_ERROR:
		return "CCIC_ERROR";
	case CCIC_NOTIFY_PIN_STATUS_DISABLED:
		return "DISABLED";
	case CCIC_NOTIFY_PIN_STATUS_RFU:
		return "RFU";
	default:
		return "NO_DETERMINATION";
	}
}

int ccic_notifier_register(struct notifier_block *nb, notifier_fn_t notifier,
			ccic_notifier_device_t listener)
{
	int ret = 0;

	pr_info("%s: listener=%d register\n", __func__, listener);

	/* Check if CCIC Notifier is ready. */
	if (!ccic_notifier_init_done)
		ccic_notifier_init();

	if (!ccic_device) {
		pr_err("%s: Not Initialized...\n", __func__);
		return -1;
	}

	SET_CCIC_NOTIFIER_BLOCK(nb, notifier, listener);
	ret = blocking_notifier_chain_register(&(ccic_notifier.notifier_call_chain), nb);
	if (ret < 0)
		pr_err("%s: blocking_notifier_chain_register error(%d)\n",
				__func__, ret);

	/* current ccic's attached_device status notify */
	nb->notifier_call(nb, 0,
			&(ccic_notifier.ccic_template));

	return ret;
}

int ccic_notifier_unregister(struct notifier_block *nb)
{
	int ret = 0;

	pr_info("%s: listener=%d unregister\n", __func__, nb->priority);

	ret = blocking_notifier_chain_unregister(&(ccic_notifier.notifier_call_chain), nb);
	if (ret < 0)
		pr_err("%s: blocking_notifier_chain_unregister error(%d)\n",
				__func__, ret);
	DESTROY_CCIC_NOTIFIER_BLOCK(nb);

	return ret;
}

static void ccic_uevent_work(int id, int state)
{
	char *water[2] = { "CCIC=WATER", NULL };
	char *dry[2] = { "CCIC=DRY", NULL };
	char *vconn[2] = { "CCIC=VCONN", NULL };
#if defined(CONFIG_SEC_FACTORY)
	char ccicrid[15] = {0,};
	char *rid[2] = {ccicrid, NULL};
	char ccicFacErr[20] = {0,};
	char *facErr[2] = {ccicFacErr, NULL};
	char ccicPinStat[20] = {0,};
	char *pinStat[2] = {ccicPinStat, NULL};
#endif

	pr_info("usb: %s: id=%s state=%d\n", __func__, pdic_event_id_string(id), state);

	switch (id) {
	case CCIC_NOTIFY_ID_WATER:
		if (state)
			kobject_uevent_env(&ccic_device->kobj, KOBJ_CHANGE, water);
		else
			kobject_uevent_env(&ccic_device->kobj, KOBJ_CHANGE, dry);
		break;
	case CCIC_NOTIFY_ID_VCONN:
		kobject_uevent_env(&ccic_device->kobj, KOBJ_CHANGE, vconn);
		break;
#if defined(CONFIG_SEC_FACTORY)
	case CCIC_NOTIFY_ID_RID:
		snprintf(ccicrid, sizeof(ccicrid), "%s", pdic_rid_string(state));
		kobject_uevent_env(&ccic_device->kobj, KOBJ_CHANGE, rid);
		break;
	case CCIC_NOTIFY_ID_FAC:
		snprintf(ccicFacErr, sizeof(ccicFacErr), "%s:%d", "ERR_STATE", state);
		kobject_uevent_env(&ccic_device->kobj, KOBJ_CHANGE, facErr);
		break;
	case CCIC_NOTIFY_ID_CC_PIN_STATUS:
		snprintf(ccicPinStat, sizeof(ccicPinStat), "%s", pdic_ccpinstatus_string(state));
		kobject_uevent_env(&ccic_device->kobj, KOBJ_CHANGE, pinStat);
		break;
#endif
	default:
		break;
	}
}

/* ccic's attached_device attach broadcast */
int ccic_notifier_notify(CC_NOTI_TYPEDEF *p_noti, void *pd, int pdic_attach)
{
	int ret = 0;

	ccic_notifier.ccic_template = *p_noti;

	switch (p_noti->id) {
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
	case CCIC_NOTIFY_ID_POWER_STATUS:
		pr_info("%s: src:%01x dest:%01x id:%02x "
			"attach:%02x cable_type:%02x rprd:%01x\n", __func__,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->src,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->dest,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->id,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->attach,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->cable_type,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->rprd);

		if (pd != NULL) {
			if (!((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->attach &&
				((struct pdic_notifier_struct *)pd)->event != PDIC_NOTIFY_EVENT_CCIC_ATTACH) {
				((struct pdic_notifier_struct *)pd)->event = PDIC_NOTIFY_EVENT_DETACH;
			}
			ccic_notifier.ccic_template.pd = pd;

			pr_info("%s: PD event:%d, num:%d, sel:%d \n", __func__,
				((struct pdic_notifier_struct *)pd)->event,
				((struct pdic_notifier_struct *)pd)->sink_status.available_pdo_num,
				((struct pdic_notifier_struct *)pd)->sink_status.selected_pdo_num);
		}
		break;
#endif
	case CCIC_NOTIFY_ID_ATTACH:
		pr_info("%s: src:%01x dest:%01x id:%02x "
			"attach:%02x cable_type:%02x rprd:%01x\n", __func__,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->src,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->dest,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->id,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->attach,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->cable_type,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->rprd);
		break;
	case CCIC_NOTIFY_ID_RID:
		pr_info("%s: src:%01x dest:%01x id:%02x rid:%02x\n", __func__,
			((CC_NOTI_RID_TYPEDEF *)p_noti)->src,
			((CC_NOTI_RID_TYPEDEF *)p_noti)->dest,
			((CC_NOTI_RID_TYPEDEF *)p_noti)->id,
			((CC_NOTI_RID_TYPEDEF *)p_noti)->rid);
#if defined(CONFIG_SEC_FACTORY)
			ccic_uevent_work(CCIC_NOTIFY_ID_RID, ((CC_NOTI_RID_TYPEDEF *)p_noti)->rid);
#endif
		break;
#ifdef CONFIG_SEC_FACTORY
	case CCIC_NOTIFY_ID_FAC:
		pr_info("%s: src:%01x dest:%01x id:%02x ErrState:%02x\n", __func__,
			p_noti->src, p_noti->dest, p_noti->id, p_noti->sub1);
			ccic_uevent_work(CCIC_NOTIFY_ID_FAC, p_noti->sub1);
			return 0;
#endif
	case CCIC_NOTIFY_ID_WATER:
		pr_info("%s: src:%01x dest:%01x id:%02x attach:%02x\n", __func__,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->src,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->dest,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->id,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->attach);
			ccic_uevent_work(CCIC_NOTIFY_ID_WATER, ((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->attach);
		break;
	case CCIC_NOTIFY_ID_VCONN:
		ccic_uevent_work(CCIC_NOTIFY_ID_VCONN, 0);
		break;
	case CCIC_NOTIFY_ID_ROLE_SWAP:
		pr_info("%s: src:%01x dest:%01x id:%02x sub1:%02x\n", __func__,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->src,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->dest,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->id,
			((CC_NOTI_ATTACH_TYPEDEF *)p_noti)->attach);
		break;
#ifdef CONFIG_SEC_FACTORY
	case CCIC_NOTIFY_ID_CC_PIN_STATUS:
		pr_info("%s: src:%01x dest:%01x id:%02x pinStatus:%02x\n", __func__,
			p_noti->src, p_noti->dest, p_noti->id, p_noti->sub1);
			ccic_uevent_work(CCIC_NOTIFY_ID_CC_PIN_STATUS, p_noti->sub1);
			return 0;
#endif
	default:
		pr_info("%s: src:%01x dest:%01x id:%02x "
			"sub1:%d sub2:%02x sub3:%02x\n", __func__,
			((CC_NOTI_TYPEDEF *)p_noti)->src,
			((CC_NOTI_TYPEDEF *)p_noti)->dest,
			((CC_NOTI_TYPEDEF *)p_noti)->id,
			((CC_NOTI_TYPEDEF *)p_noti)->sub1,
			((CC_NOTI_TYPEDEF *)p_noti)->sub2,
			((CC_NOTI_TYPEDEF *)p_noti)->sub3);
		break;
	}
#ifdef CONFIG_USB_NOTIFY_PROC_LOG
	if (p_noti->id != CCIC_NOTIFY_ID_POWER_STATUS)
		store_usblog_notify(NOTIFY_CCIC_EVENT, (void *)p_noti, NULL);
#endif
	ret = blocking_notifier_call_chain(&(ccic_notifier.notifier_call_chain),
			p_noti->id, &(ccic_notifier.ccic_template));


	switch (ret) {
	case NOTIFY_STOP_MASK:
	case NOTIFY_BAD:
		pr_err("%s: notify error occur(0x%x)\n", __func__, ret);
		break;
	case NOTIFY_DONE:
	case NOTIFY_OK:
		pr_info("%s: notify done(0x%x)\n", __func__, ret);
		break;
	default:
		pr_info("%s: notify status unknown(0x%x)\n", __func__, ret);
		break;
	}

	return ret;

}

int ccic_notifier_init(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	if (ccic_notifier_init_done) {
		pr_err("%s already registered\n", __func__);
		goto out;
	}
	ccic_notifier_init_done = 1;
	ccic_core_init();
	BLOCKING_INIT_NOTIFIER_HEAD(&(ccic_notifier.notifier_call_chain));

out:
	return ret;
}

static void __exit ccic_notifier_exit(void)
{
	pr_info("%s: exit\n", __func__);
}

device_initcall(ccic_notifier_init);
module_exit(ccic_notifier_exit);
