/*
 * intel_fuel_gauge.c - Intel MID Fuel Gauge Driver
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *         Srinidhi Rao <srinidhi.rao@intel.com>
 *         David Bennetts <david.bennetts@intel.com>
 *         Felix Becker <felix.becker@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/version.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/async.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/power/intel_sw_fuel_gauge.h>
#include <linux/kdev_t.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/poll.h>

#define DRIVER_NAME	  "intel_fuel_gauge"
#define FG_IFACE_NAME "intel_fg_iface"


#define INTEL_FG_WAKELOCK_TIMEOUT	(1 * HZ)
#define INTEL_FG_DISP_LOWBATT_TIMEOUT   (3 * HZ)
#define SOC_WARN_LVL1			14
#define SOC_WARN_LVL2			4
#define SOC_WARN_LVL3			0

#define BATT_OVP_OFFSET			50000 /* 50mV */

#define FG_ADC_VBATT_OFF_ADJ		5000 /* 5mV */
#define FG_ADC_IBATT_OFF_ADJ		30000 /* 30mA */

#define FG_OCV_SMOOTH_DIV_NOR		20
#define FG_OCV_SMOOTH_DIV_FULL		100
#define FG_OCV_SMOOTH_CAP_LIM		97

#define BOUND(min_val, x, max_val) min(max(x, min_val), max_val)

struct intel_fg_wakeup_event {
	int soc_bfr_sleep;
	bool wake_enable;
	struct wake_lock wakelock;
};

struct fg_iface_attr {
	struct mutex iface_lock;

	wait_queue_head_t wait;
	bool uevent_ack;
	bool suspended;

	struct miscdevice intel_fg_misc_device;
};


struct intel_fg_info {
	struct device *dev;
	struct intel_fg_batt_spec *batt_spec;
	struct intel_fg_input *input;
	struct intel_fg_algo *algo;
	struct intel_fg_algo *algo_sec;
	struct delayed_work fg_worker;
	struct delayed_work fg_update_eoc;
	struct delayed_work fg_init;
	struct power_supply *psy;
	struct mutex lock;

	struct intel_fg_wakeup_event wake_ui;
	struct fg_params params;
	struct fg_iface_attr fg_attr;
	struct iio_cb_buffer *buf;
	struct completion cc_tick_complete;
};

static struct intel_fg_info *info_ptr;

/* default battery spec data */
static struct intel_fg_batt_spec bspec = {
	.volt_min_design = 3400000,
	.volt_max_design = 4350000,
	.temp_min = 0,
	.temp_max = 450,
	.charge_full_design = 2650000,
};

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

enum write_chan_num {
	IIO_DELTA_THR,
	IIO_CLR_LATCH_AVG,
	IIO_CLR_ACC_ERR,
};

static int wait_for_cc_tick(void)
{
	int ret;
	struct iio_channel *indio_chan;

	indio_chan = iio_channel_get(NULL, "IRQ_CC_TICK");
	if (IS_ERR_OR_NULL(indio_chan)) {
		ret = PTR_ERR(indio_chan);
		return ret;
	}
	ret = iio_event_wait(indio_chan->indio_dev);
	if ((ret & POLLIN) || (ret & POLLRDNORM)) {
		dev_err(info_ptr->dev, "Rxd CC TICK\n");
		ret = 0;
	} else
		dev_err(info_ptr->dev, "No CC TCK event\n");

	return ret;
}

static bool is_irq_supported(void)
{
	int ret, val;
	struct iio_channel *indio_chan;

	indio_chan = iio_channel_get(NULL, "IRQ_CC_TICK");
	if (IS_ERR_OR_NULL(indio_chan)) {
		ret = PTR_ERR(indio_chan);
		return false;
	}
	ret = iio_read_channel_raw(indio_chan, &val);
	if (ret) {
		pr_err("SWFG: IIO channel read error\n");
		return false;
	}

	if (val)
		return true;
	else
		return false;
}

static int intel_swfg_read_adc_val(const char *name, int *raw_val)
{
	int ret, val;
	struct iio_channel *indio_chan;

	indio_chan = iio_channel_get(NULL, name);
	if (IS_ERR_OR_NULL(indio_chan)) {
		ret = PTR_ERR(indio_chan);
		goto exit;
	}
	ret = iio_read_channel_raw(indio_chan, &val);
	if (ret) {
		pr_err("SWFG: IIO channel read error\n");
		goto err_exit;
	}

	*raw_val = val;

err_exit:
	iio_channel_release(indio_chan);
exit:
	return ret;
}

static int intel_swfg_write_adc_val(const char *name, int *raw_val)
{
	int ret;
	struct iio_channel *indio_chan;

	indio_chan = iio_channel_get(NULL, name);
	if (IS_ERR_OR_NULL(indio_chan)) {
		ret = PTR_ERR(indio_chan);
		goto exit;
	}
	ret = iio_write_channel_raw(indio_chan, *raw_val);
	if (ret) {
		pr_err("SWFG: IIO channel write error\n");
		goto err_exit;
	}

err_exit:
	iio_channel_release(indio_chan);
exit:
	return ret;
}

static ssize_t fg_iface_get_volt_now(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("VBAT", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_volt_ocv(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("PEAK", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_volt_boot(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("PEAK", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_ibat_boot(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("IBAT_BOOT", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_cur_now(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("IBAT_SHORT", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_cur_avg(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("IBAT_LONG", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_batt_temp(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("BATTEMP0", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_delta_q(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("DELTAQ", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_capacity(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.soc;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_set_capacity(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->fg_attr.uevent_ack = true;
	info_ptr->params.soc = val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	return count;
}

static ssize_t fg_iface_get_nac(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.nac;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_set_nac(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.nac = val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	return count;
}

static ssize_t fg_iface_get_fcc(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.fcc;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_set_fcc(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.fcc = val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	return count;
}

static ssize_t fg_iface_get_cyc_cnt(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.cycle_count;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_set_cyc_cnt(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.cycle_count = val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	return count;
}


static ssize_t fg_iface_get_cc_calib(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.calib_cc;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_set_cc_calib(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.calib_cc = val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	return count;
}

/* Coulomb Counter input attribute APIs */

static ssize_t fg_iface_get_up_cc(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("UP_CC", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_down_cc(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("DOWN_CC", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_acc_err(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("ACC_ERR", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_delta_thr(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("DELTA_THR", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_long_avg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("IBAT_LONG", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_long_avg_ocv(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("IBAT_LONG_LATCH", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_ocv_accuracy(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val, raw_val;

	ret = intel_swfg_read_adc_val("VOCV_ACCURACY", &raw_val);
	if (ret < 0)
		return ret;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = raw_val;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}


/* Coulomb Counter Output attributes */

static ssize_t fg_iface_set_delta_thr(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.set_delta_thr.val = val;
	info_ptr->params.set_delta_thr.is_set = true;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);

	ret = intel_swfg_write_adc_val("SET_DELTA_THR",
		&info_ptr->params.set_delta_thr.val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t fg_iface_clr_acc_err(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.reset_acc_err.val = val;
	info_ptr->params.reset_acc_err.is_set = true;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);

	ret = intel_swfg_write_adc_val("CLR_ACC_ERR", 0);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t fg_iface_clr_latched_ibat_avg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	info_ptr->params.clr_latched_ibat_avg.val = val;
	info_ptr->params.clr_latched_ibat_avg.is_set = true;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);

	ret = intel_swfg_write_adc_val("CLR_LATCH_IBAT", 0);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t fg_iface_get_ip_ev(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.msg.event;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fg_iface_get_ip_ev_param(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val;

	mutex_lock(&info_ptr->fg_attr.iface_lock);
	val = info_ptr->params.msg.ev_param.target_voltage_mv;
	mutex_unlock(&info_ptr->fg_attr.iface_lock);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static DEVICE_ATTR(volt_now, S_IRUGO,
			fg_iface_get_volt_now, NULL);
static DEVICE_ATTR(volt_ocv, S_IRUGO,
			fg_iface_get_volt_ocv, NULL);
static DEVICE_ATTR(volt_boot, S_IRUGO,
			fg_iface_get_volt_boot, NULL);
static DEVICE_ATTR(ibat_boot, S_IRUGO,
			fg_iface_get_ibat_boot, NULL);
static DEVICE_ATTR(cur_now, S_IRUGO,
			fg_iface_get_cur_now, NULL);
static DEVICE_ATTR(cur_avg, S_IRUGO,
			fg_iface_get_cur_avg, NULL);
static DEVICE_ATTR(batt_temp, S_IRUGO,
			fg_iface_get_batt_temp, NULL);
static DEVICE_ATTR(delta_q, S_IRUGO,
			fg_iface_get_delta_q, NULL);


static DEVICE_ATTR(capacity, S_IWUSR | S_IRUGO,
		fg_iface_get_capacity, fg_iface_set_capacity);
static DEVICE_ATTR(nac, S_IWUSR | S_IRUGO,
		fg_iface_get_nac, fg_iface_set_nac);
static DEVICE_ATTR(fcc, S_IWUSR | S_IRUGO,
		fg_iface_get_fcc, fg_iface_set_fcc);
static DEVICE_ATTR(cyc_cnt, S_IWUSR | S_IRUGO,
		fg_iface_get_cyc_cnt, fg_iface_set_cyc_cnt);
static DEVICE_ATTR(cc_calib, S_IWUSR | S_IRUGO,
		fg_iface_get_cc_calib, fg_iface_set_cc_calib);


/* Coulomb Counter Read-only attribute */
static DEVICE_ATTR(up_cc, S_IRUGO,
			fg_iface_get_up_cc, NULL);
static DEVICE_ATTR(down_cc, S_IRUGO,
			fg_iface_get_down_cc, NULL);
static DEVICE_ATTR(long_avg, S_IRUGO,
			fg_iface_get_long_avg, NULL);
static DEVICE_ATTR(long_avg_ocv, S_IRUGO,
			fg_iface_get_long_avg_ocv, NULL);
static DEVICE_ATTR(ocv_accuracy, S_IRUGO,
			fg_iface_get_ocv_accuracy, NULL);
static DEVICE_ATTR(acc_err, S_IRUGO,
			fg_iface_get_acc_err, NULL);

/* Coulomb Counter Read/Write attributes */
static DEVICE_ATTR(clr_acc_err, S_IWUSR | S_IRUGO,
		NULL, fg_iface_clr_acc_err);
static DEVICE_ATTR(delta_thr, S_IWUSR | S_IRUGO,
		fg_iface_get_delta_thr, fg_iface_set_delta_thr);
static DEVICE_ATTR(clr_latched_ibat_avg, S_IWUSR | S_IRUGO,
		NULL, fg_iface_clr_latched_ibat_avg);

static DEVICE_ATTR(ip_ev, S_IRUGO,
			fg_iface_get_ip_ev, NULL);
static DEVICE_ATTR(ip_ev_param, S_IRUGO,
			fg_iface_get_ip_ev_param, NULL);

static struct attribute *fg_iface_sysfs_attributes[] = {
	&dev_attr_volt_now.attr,
	&dev_attr_volt_ocv.attr,
	&dev_attr_volt_boot.attr,
	&dev_attr_ibat_boot.attr,
	&dev_attr_cur_now.attr,
	&dev_attr_cur_avg.attr,
	&dev_attr_batt_temp.attr,
	&dev_attr_delta_q.attr,
	&dev_attr_capacity.attr,
	&dev_attr_nac.attr,
	&dev_attr_fcc.attr,
	&dev_attr_cyc_cnt.attr,
	&dev_attr_cc_calib.attr,
	/* Coulomb Counter attributes */
	&dev_attr_up_cc.attr,
	&dev_attr_down_cc.attr,
	&dev_attr_long_avg.attr,
	&dev_attr_long_avg_ocv.attr,
	&dev_attr_ocv_accuracy.attr,
	&dev_attr_acc_err.attr,

	&dev_attr_clr_acc_err.attr,
	&dev_attr_delta_thr.attr,
	&dev_attr_clr_latched_ibat_avg.attr,

	&dev_attr_ip_ev.attr,
	&dev_attr_ip_ev_param.attr,
	NULL,
};

static const struct attribute_group fg_iface_sysfs_attr_group = {
	.attrs = fg_iface_sysfs_attributes,
};

static int fg_iface_sysfs_init(struct intel_fg_info *info)
{
	int ret;

	info->fg_attr.intel_fg_misc_device.minor = MISC_DYNAMIC_MINOR;
	info->fg_attr.intel_fg_misc_device.name = FG_IFACE_NAME;
	info->fg_attr.intel_fg_misc_device.mode = (S_IWUSR | S_IRUGO);
	ret = misc_register(&info->fg_attr.intel_fg_misc_device);
	if (ret) {
		dev_err(info->dev,
			"\n Err %d in registering misc class", ret);
		return ret;
	}
	ret = sysfs_create_group(
		&info->fg_attr.intel_fg_misc_device.this_device->kobj,
		&fg_iface_sysfs_attr_group);
	if (ret) {
		dev_err(info->dev,
			"\nError %d in creating sysfs group", ret);
		misc_deregister(&info->fg_attr.intel_fg_misc_device);
	}
	return ret;
}

static void fg_iface_sysfs_exit(struct intel_fg_info *info)
{
	sysfs_remove_group(
		&info->fg_attr.intel_fg_misc_device.this_device->kobj,
			&fg_iface_sysfs_attr_group);
	if (info->fg_attr.intel_fg_misc_device.this_device)
		misc_deregister(&info->fg_attr.intel_fg_misc_device);
}

/**
 * intel_fg_check_low_batt_event - Checks low batt condition
 * @info : Pointer to the intel_fg_info structure instance
 *
 * Returns 0 if success
 */
static int intel_fg_check_low_batt_event(struct intel_fg_info *info)
{
	int ret = 0;

	/*
	 * Compare the previously stored capacity before going to suspend mode,
	 * with the current capacity during resume, along with the SOC_WARN_LVLs
	 * and if the new SOC during resume has fell below any of the low batt
	 * warning levels, hold the wake lock for 1 sec so that Android user
	 * space will have sufficient time to display the warning message.
	 */
	if (BOUND(info->params.soc, SOC_WARN_LVL1,
				info->wake_ui.soc_bfr_sleep) == SOC_WARN_LVL1)
		info->wake_ui.wake_enable = true;
	else if (BOUND(info->params.soc, SOC_WARN_LVL2,
				info->wake_ui.soc_bfr_sleep) == SOC_WARN_LVL2)
		info->wake_ui.wake_enable = true;
	else if (info->params.soc == SOC_WARN_LVL3)
		info->wake_ui.wake_enable = true;
	else {
		if (wake_lock_active(&info_ptr->wake_ui.wakelock))
			wake_unlock(&info_ptr->wake_ui.wakelock);
		info->wake_ui.wake_enable = false;
	}

	if (info->wake_ui.wake_enable) {
		wake_lock_timeout(&info_ptr->wake_ui.wakelock,
			INTEL_FG_DISP_LOWBATT_TIMEOUT);
		info->wake_ui.wake_enable = false;
	}
	return ret;
}
static int intel_fg_vbatt_soc_calc(struct intel_fg_info *info, int vbatt)
{
	int soc;

	soc = (vbatt - info->batt_spec->volt_min_design) * 100;
	soc /= (info->batt_spec->volt_max_design -
			info->batt_spec->volt_min_design);

	/* limit the capacity to 0 to 100 */
	soc = clamp(soc, 0, 100);

	return soc;
}

static int intel_fg_worker_async(struct intel_fg_info *fg_info)
{
	/*
	 * This is the space for legacy implementation and is no
	 * longer required.
	 */
	return 0;
}
static int intel_fg_worker_sync(struct intel_fg_info *fg_info)
{
	mutex_lock(&fg_info->lock);
	mutex_lock(&fg_info->fg_attr.iface_lock);

	fg_info->params.msg.event = SWFG_HAL_EVENT_SOC_UPDATE;
	fg_info->params.msg.ev_param.cc_delta_mc =
			info_ptr->params.delta_q;

	dev_dbg(fg_info->dev,
			"Sending uevent from intel_sw_fuel_gauge\n");

	if (!IS_ERR_OR_NULL(
			fg_info->fg_attr.intel_fg_misc_device.this_device))
		sysfs_notify(
		&fg_info->fg_attr.intel_fg_misc_device.this_device->kobj,
			NULL, "uevent");

	mutex_unlock(&fg_info->fg_attr.iface_lock);
	mutex_unlock(&fg_info->lock);

	return 0;
}
static void intel_fg_worker(struct work_struct *work)
{
	struct intel_fg_info *fg_info = container_of(
			to_delayed_work(work), struct intel_fg_info,
			fg_worker);
	int sched_time;

	/* Blocking wait call if PMIC supports CCTICK IRQ */
	if (is_irq_supported()) {
		wait_for_cc_tick();
		sched_time = 2;
		intel_fg_worker_sync(fg_info);
	 } else {
		sched_time = 30;
		intel_fg_worker_async(fg_info);
	}

	power_supply_changed(fg_info->psy);

	if (fg_info->wake_ui.wake_enable)
		intel_fg_check_low_batt_event(fg_info);

	schedule_delayed_work(&fg_info->fg_worker, sched_time * HZ);
}

static void intel_fuel_gauge_update_eoc(struct work_struct *work)
{
	struct intel_fg_info *fg_info = container_of(
			to_delayed_work(work), struct intel_fg_info,
			fg_update_eoc);

	dev_info(fg_info->dev,
			"intel_fuel_gauge_update_eoc called\n");

	mutex_lock(&fg_info->lock);
	mutex_lock(&fg_info->fg_attr.iface_lock);

	fg_info->params.msg.event = SWFG_HAL_EVENT_EOC;

	dev_dbg(fg_info->dev, "Sending EOC event\n");

	if (!IS_ERR_OR_NULL(
		fg_info->fg_attr.intel_fg_misc_device.this_device))
		sysfs_notify(
		&fg_info->fg_attr.intel_fg_misc_device.this_device->kobj,
		NULL, "uevent");

	mutex_unlock(&fg_info->fg_attr.iface_lock);
	mutex_unlock(&fg_info->lock);
}

static int intel_fg_battery_health(struct intel_fg_info *info)
{
	int health;


	if (!info->params.is_valid_battery)
		health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (info->params.vbatt > info->batt_spec->volt_max_design
			+ BATT_OVP_OFFSET)
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (info->params.bat_temp > info->batt_spec->temp_max ||
			info->params.bat_temp < info->batt_spec->temp_min)
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (info->params.vocv < info->batt_spec->volt_min_design)
		health = POWER_SUPPLY_HEALTH_DEAD;
	else
		health = POWER_SUPPLY_HEALTH_GOOD;

	return health;
}

static int intel_fuel_gauge_get_property(struct power_supply *psup,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	struct intel_fg_info *fg_info = power_supply_get_drvdata(
					psup);
	int raw_val;

	mutex_lock(&fg_info->lock);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_info->params.status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 0x1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = intel_fg_battery_health(fg_info);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = intel_swfg_read_adc_val("VBAT", &raw_val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = intel_swfg_read_adc_val("PEAK", &raw_val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = fg_info->batt_spec->volt_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = fg_info->batt_spec->volt_max_design;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = intel_swfg_read_adc_val("IBAT_SHORT",
					&raw_val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = intel_swfg_read_adc_val("IBAT_LONG",
					&raw_val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (fg_info->fg_attr.uevent_ack)
			val->intval = fg_info->params.soc;
		else
			val->intval = intel_fg_vbatt_soc_calc(fg_info,
			intel_swfg_read_adc_val("VBAT", &raw_val));
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = intel_swfg_read_adc_val("BATTEMP0",
					&raw_val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = fg_info->params.nac;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = fg_info->params.fcc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = fg_info->batt_spec->charge_full_design;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = (int)(fg_info->params.down_cc -
					fg_info->params.up_cc);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = fg_info->params.battid;
		break;
	default:
		mutex_unlock(&fg_info->lock);
		return -EINVAL;
	}

	mutex_unlock(&fg_info->lock);
	return 0;
}

static int intel_fuel_gauge_set_property(struct power_supply *psup,
				enum power_supply_property prop,
				const union power_supply_propval *val)
{
	struct intel_fg_info *fg_info = power_supply_get_drvdata(psup);
	int ret, raw_val;

	mutex_lock(&fg_info->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		fg_info->params.status = val->intval;
		if (val->intval == POWER_SUPPLY_STATUS_FULL) {
			pr_err("Battery reached full\n");
			ret = intel_swfg_read_adc_val("PEAK", &raw_val);
			fg_info->params.msg.ev_param.target_voltage_mv =
				raw_val;
			pr_info("EOC VOCV = %d\n",
			fg_info->params.msg.ev_param.target_voltage_mv);
			schedule_delayed_work(&fg_info->fg_update_eoc,
				1 * HZ);
		}
		break;
	default:
		dev_warn(fg_info->dev, "invalid psy prop\b");
		mutex_unlock(&fg_info->lock);
		return -EINVAL;
	}
	mutex_unlock(&fg_info->lock);
	return 0;
}

static void intel_fg_ext_psy_changed(struct power_supply *psy)
{
	struct intel_fg_info *fg_info = power_supply_get_drvdata(psy);

	dev_info(fg_info->dev, "%s\n", __func__);
	power_supply_changed(fg_info->psy);
}

static const struct power_supply_desc swfg_psy_desc = {

	.name = "intel_fuel_gauge",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = &intel_fuel_gauge_get_property,
	.set_property = &intel_fuel_gauge_set_property,
	.external_power_changed = &intel_fg_ext_psy_changed,
	.properties = fg_props,
	.num_properties = ARRAY_SIZE(fg_props),
};

static void intel_fg_init_worker(struct work_struct *work)
{
	struct intel_fg_info *fg_info = container_of(
			to_delayed_work(work), struct intel_fg_info,
			fg_init);

	mutex_lock(&fg_info->lock);
	mutex_lock(&fg_info->fg_attr.iface_lock);

	fg_info->params.msg.event = SWFG_BAT_EVENT_PRESENCE;
	fg_info->params.msg.ev_param.battery_capacity = 1;

	dev_dbg(fg_info->dev, "Sending Battery event\n");

	if (!IS_ERR_OR_NULL(
		fg_info->fg_attr.intel_fg_misc_device.this_device))
		sysfs_notify(
		&fg_info->fg_attr.intel_fg_misc_device.this_device->kobj,
		NULL, "uevent");

		fg_info->psy = devm_power_supply_register(fg_info->dev,
					&swfg_psy_desc, NULL);
	if (IS_ERR(fg_info->psy)) {
		dev_err(fg_info->dev, "power supply reg failed %ld\n",
				PTR_ERR(fg_info->psy));

	mutex_unlock(&fg_info->fg_attr.iface_lock);
	mutex_unlock(&fg_info->lock);
		return;
	}

	mutex_unlock(&fg_info->fg_attr.iface_lock);
	mutex_unlock(&fg_info->lock);

	schedule_delayed_work(&fg_info->fg_worker, 20 * HZ);

}

static int intel_fuel_gauge_probe(struct platform_device *pdev)
{
	struct intel_fg_info *fg_info;

	fg_info = devm_kzalloc(&pdev->dev, sizeof(*fg_info), GFP_KERNEL);
	if (!fg_info)
		return -ENOMEM;

	fg_info->dev = &pdev->dev;
	fg_info->batt_spec = &bspec;
	platform_set_drvdata(pdev, fg_info);

	mutex_init(&fg_info->lock);
	mutex_init(&fg_info->fg_attr.iface_lock);
	INIT_DELAYED_WORK(&fg_info->fg_worker, &intel_fg_worker);
	INIT_DELAYED_WORK(&fg_info->fg_update_eoc,
		&intel_fuel_gauge_update_eoc);
	INIT_DELAYED_WORK(&fg_info->fg_init, &intel_fg_init_worker);
	fg_info->params.status = POWER_SUPPLY_STATUS_DISCHARGING;

	/* Hardcode batt ID string on broxton */
	fg_info->params.is_valid_battery = true;
	strncpy(fg_info->params.battid, "INTN0001",
				SWFG_BATTID_STR_LEN);

	wake_lock_init(&fg_info->wake_ui.wakelock, WAKE_LOCK_SUSPEND,
				"intel_fg_wakelock");

	info_ptr = fg_info;

	fg_iface_sysfs_init(info_ptr);
	schedule_delayed_work(&fg_info->fg_init, 1 * HZ);

	return 0;
}

static int intel_fuel_gauge_remove(struct platform_device *pdev)
{
	struct intel_fg_info *fg_info = platform_get_drvdata(pdev);

	wake_lock_destroy(&fg_info->wake_ui.wakelock);

	fg_iface_sysfs_exit(info_ptr);

	return 0;
}

static int intel_fuel_gauge_suspend(struct device *dev)
{
	/*
	 * Store the current SOC value before going to suspend as
	 * this value will be used by the worker function in resume to
	 * check whether the low battery threshold has been crossed.
	 */
	info_ptr->wake_ui.soc_bfr_sleep = info_ptr->params.soc;
	cancel_delayed_work_sync(&info_ptr->fg_worker);
	return 0;
}

static int intel_fuel_gauge_resume(struct device *dev)
{
	/*
	 * Set the wake_enable flag as true and schedule the
	 * work queue at 0 secs so that the worker function is
	 * scheduled immediately at the next available tick.
	 * Once the intel_fg_worker function starts executing
	 * It can check and clear the wake_enable flag and hold
	 * the wakelock if low batt warning notification has to
	 * be sent
	 */
	wake_lock_timeout(&info_ptr->wake_ui.wakelock,
			  INTEL_FG_WAKELOCK_TIMEOUT);
	info_ptr->wake_ui.wake_enable = true;
	schedule_delayed_work(&info_ptr->fg_worker, 0);
	return 0;
}

static int intel_fuel_gauge_runtime_suspend(struct device *dev)
{
	return 0;
}
static int intel_fuel_gauge_runtime_resume(struct device *dev)
{
	return 0;
}
static int intel_fuel_gauge_runtime_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops intel_fuel_gauge_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_fuel_gauge_suspend,
			intel_fuel_gauge_resume)
	SET_RUNTIME_PM_OPS(intel_fuel_gauge_runtime_suspend,
			intel_fuel_gauge_runtime_resume,
			intel_fuel_gauge_runtime_idle)
};

static const struct platform_device_id intel_fuel_gauge_id[] = {
	{.name = DRIVER_NAME},
	{.name = "bxt_swfg"},
	{ },
};
MODULE_DEVICE_TABLE(platform, intel_fuel_gauge_id);

static struct platform_driver intel_fuel_gauge_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &intel_fuel_gauge_pm_ops,
	},
	.probe = intel_fuel_gauge_probe,
	.remove = intel_fuel_gauge_remove,
	.id_table = intel_fuel_gauge_id,
};

module_platform_driver(intel_fuel_gauge_driver);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_AUTHOR("Srinidhi Rao <srinidhi.rao@intel.com>");
MODULE_AUTHOR("David Bennetts <david.bennetts@intel.com>");
MODULE_AUTHOR("Felix Becker <felix.becker@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel MID S/W Fuel Gauge Driver");
