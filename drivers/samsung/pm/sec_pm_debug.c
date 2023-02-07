/*
 * sec_pm_debug.c
 *
 *  Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *  Minsung Kim <ms925.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sec_class.h>
#include <linux/sec_pm_debug.h>
#include <linux/suspend.h>

u8 pmic_onsrc;
u8 pmic_offsrc;

static unsigned int sleep_count;
static struct timespec64 total_sleep_time;
static ktime_t last_monotime; /* monotonic time before last suspend */
static ktime_t curr_monotime; /* monotonic time after last suspend */
static ktime_t last_stime; /* monotonic boottime offset before last suspend */
static ktime_t curr_stime; /* monotonic boottime offset after last suspend */

static ssize_t sleep_time_sec_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", total_sleep_time.tv_sec);
}

static ssize_t sleep_count_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sleep_count);
}

static ssize_t pwr_on_off_src_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ONSRC:0x%02X OFFSRC:0x%02X\n", pmic_onsrc,
			pmic_offsrc);
}

static DEVICE_ATTR_RO(sleep_time_sec);
static DEVICE_ATTR_RO(sleep_count);
static DEVICE_ATTR_RO(pwr_on_off_src);

static struct attribute *sec_pm_debug_attrs[] = {
	&dev_attr_sleep_time_sec.attr,
	&dev_attr_sleep_count.attr,
	&dev_attr_pwr_on_off_src.attr,
	NULL
};
ATTRIBUTE_GROUPS(sec_pm_debug);

static int suspend_resume_pm_event(struct notifier_block *notifier,
		unsigned long pm_event, void *unused)
{
	struct timespec64 sleep_time;
	struct timespec64 total_time;
	struct timespec64 suspend_resume_time;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		/* monotonic time since boot */
		last_monotime = ktime_get();
		/* monotonic time since boot including the time spent in suspend */
		last_stime = ktime_get_boottime();
		break;
	case PM_POST_SUSPEND:
		/* monotonic time since boot */
		curr_monotime = ktime_get();
		/* monotonic time since boot including the time spent in suspend */
		curr_stime = ktime_get_boottime();

		total_time = ktime_to_timespec64(ktime_sub(curr_stime, last_stime));
		suspend_resume_time =
			ktime_to_timespec64(ktime_sub(curr_monotime, last_monotime));
		sleep_time = timespec64_sub(total_time, suspend_resume_time);

		total_sleep_time = timespec64_add(total_sleep_time, sleep_time);
		sleep_count++;
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block sec_pm_notifier_block = {
	.notifier_call = suspend_resume_pm_event,
};

static int __init sec_pm_debug_init(void)
{
	struct device *sec_pm_dev;
	int ret;

	ret = register_pm_notifier(&sec_pm_notifier_block);
	if (ret) {
		pr_err("%s: failed to register PM notifier(%d)\n",
				__func__, ret);
		return ret;
	}
	total_sleep_time.tv_sec = 0;
	total_sleep_time.tv_nsec = 0;

	sec_pm_dev = sec_device_create(NULL, "pm");

	if (IS_ERR(sec_pm_dev)) {
		pr_err("%s: fail to create sec_pm_dev\n", __func__);
		return PTR_ERR(sec_pm_dev);
	}

	ret = sysfs_create_groups(&sec_pm_dev->kobj, sec_pm_debug_groups);
	if (ret) {
		pr_err("%s: failed to create sysfs groups(%d)\n", __func__, ret);
		goto err_create_sysfs;
	}

	return 0;

err_create_sysfs:
	sec_device_destroy(sec_pm_dev->devt);

	return ret;
}
late_initcall(sec_pm_debug_init);
