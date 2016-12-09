/*
 * iio_whiskeycove_gpadc.c - Intel  Whiskey Cove GPADC Driver
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
 * Author: Bin Yang <bin.yang@intel.com>
 * Author: Jenny TC <jenny.tc@intel.com>
 * Author: Pavan Kumar S <pavan.kumar.s@intel.com>
 * Author: Srinidhi Rao <srinidhi.rao@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/types.h>
#include <linux/iio/consumer.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/iio/intel_wcove_gpadc.h>
#include <linux/platform_device.h>

#define OHM_MULTIPLIER 10

struct gpadc_info {
	u8 irq_status;
	u8 pmic_id;
	u8 intr_mask;
	int irq;
	int initialized;
	int sample_done;
	int channel_num;
	/* This mutex protects gpadc sample/config from concurrent conflict.
	   Any function, which does the sample or config, needs to
	   hold this lock.
	   If it is locked, it also means the gpadc is in active mode.
	*/
	struct mutex lock;
	struct device *dev;
	struct gpadc_regmap_t *gpadc_regmaps;
	struct gpadc_regs_t *gpadc_regs;
	struct regmap *regmap;
	bool is_pmic_provisioned;
	wait_queue_head_t wait;
};

static struct gpadc_regmap_t whiskeycove_gpadc_regmaps[GPADC_NUM_CHANNELS] = {
	{"VBAT",    5, 5,   0x4F03, 0x4F04, 0xFF, 0xFF, 0xFF, 0xFF},
	{"BATID",   4, 4,   0x4F06, 0x4F07, 0xFF, 0xFF, 0xFF, 0xFF},
	{"PMICTEMP",    3, 3,   0x4F42, 0x4F43, 0x4F33, 0x4F34, 0x4F33, 0x4F34},
	{"BATTEMP0",    2, 2,   0x4F15, 0x4F16, 0xFF, 0xFF, 0xFF, 0xFF},
	{"BATTEMP1",    2, 2,   0x4F17, 0x4F18, 0xFF, 0xFF, 0xFF, 0xFF},
	{"SYSTEMP0",    3, 3,   0x4F38, 0x4F39, 0x4F23, 0x4F24, 0x4F25, 0x4F26},
	{"SYSTEMP1",    3, 3,   0x4F3A, 0x4F3B, 0x4F27, 0x4F28, 0x4F29, 0x4F2A},
	{"SYSTEMP2",    3, 3,   0x4F3C, 0x4F3D, 0x4F2B, 0x4F2C, 0x4F2D, 0x4F2E},
	{"USBID",   1, 0,   0x4F08, 0x4F09, 0xFF, 0xFF, 0xFF, 0xFF},
	{"PEAK",    7, 1,   0x4F13, 0x4F14, 0xFF, 0xFF, 0xFF, 0xFF},
	{"AGND",    6, 6,   0x4F0A, 0x4F0B, 0xFF, 0xFF, 0xFF, 0xFF},
	{"VREF",    6, 6,   0x4F0A, 0x4F0B, 0xFF, 0xFF, 0xFF, 0xFF},
};

static struct gpadc_regs_t whiskeycove_gpadc_regs = {
	.gpadcreq   =   0x4F02,
	.gpadcreq_irqen =   0,
	.gpadcreq_busy  =   (1 << 0),
	.mirqlvl1   =   0x4e0E,
	.mirqlvl1_adc   =   (1 << 4),
	.adc1cntl   =   0x4F05,
	.adcirq     =   0x4E08,
	.madcirq    =   0x4E16,
	.thrmmonctl =   0x4F1E,
	.batthermonctl =    0x4F1F,
	.vbatmonctl =   0x4F20,
	.gpmonctl   =   0x4F21,
};

#define THERM_MON_VAL        0x1B
#define BATTHERM_MON_VAL     0x7B
#define VBAT_MON_VAL         0x1B
#define GPM_MON_VAL          0x00

#define MSIC_ADC_MAP(_adc_channel_label,          \
					_consumer_dev_name,           \
					_consumer_channel)            \
	{                                             \
		.adc_channel_label = _adc_channel_label,  \
		.consumer_dev_name = _consumer_dev_name,  \
		.consumer_channel = _consumer_channel,    \
	}

/* for consumer drivers */
static struct iio_map wc_iio_maps[] = {
	MSIC_ADC_MAP("CH0", "VIBAT", "VBAT"),
	MSIC_ADC_MAP("CH1", "BATID", "BATID"),
	MSIC_ADC_MAP("CH2", "PMICTEMP", "PMICTEMP"),
	MSIC_ADC_MAP("CH3", "BATTEMP", "BATTEMP0"),
	MSIC_ADC_MAP("CH4", "BATTEMP", "BATTEMP1"),
	MSIC_ADC_MAP("CH5", "SYSTEMP", "SYSTEMP0"),
	MSIC_ADC_MAP("CH6", "SYSTEMP", "SYSTEMP1"),
	MSIC_ADC_MAP("CH7", "SYSTEMP", "SYSTEMP2"),
	MSIC_ADC_MAP("CH8", "USBID", "USBID"),
	MSIC_ADC_MAP("CH9", "PEAK", "PEAK"),
	MSIC_ADC_MAP("CH10", "GPMEAS", "AGND"),
	MSIC_ADC_MAP("CH11", "GPMEAS", "VREF"),
	{ },
};

#define MSIC_ADC_CHANNEL(_type, _channel, _datasheet_name) \
	{                               \
		.indexed = 1,               \
		.type = _type,              \
		.channel = _channel,        \
		.datasheet_name = _datasheet_name,      \
	}

static const struct iio_chan_spec const wc_adc_channels[] = {
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 0, "CH0"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 1, "CH1"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 2, "CH2"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 3, "CH3"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 4, "CH4"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 5, "CH5"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 6, "CH6"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 7, "CH7"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 8, "CH8"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 9, "CH9"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 10, "CH10"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 11, "CH11"),
};

static const char * const iio_ev_type_text[] = {
	[IIO_EV_TYPE_THRESH] = "thresh",
	[IIO_EV_TYPE_MAG] = "mag",
	[IIO_EV_TYPE_ROC] = "roc",
	[IIO_EV_TYPE_THRESH_ADAPTIVE] = "thresh_adaptive",
	[IIO_EV_TYPE_MAG_ADAPTIVE] = "mag_adaptive",
};

static const char * const iio_ev_dir_text[] = {
	[IIO_EV_DIR_EITHER] = "either",
	[IIO_EV_DIR_RISING] = "rising",
	[IIO_EV_DIR_FALLING] = "falling"
};

static const char * const iio_ev_info_text[] = {
	[IIO_EV_INFO_ENABLE] = "en",
	[IIO_EV_INFO_VALUE] = "value",
	[IIO_EV_INFO_HYSTERESIS] = "hysteresis",
};

static int gpadc_busy_wait(struct gpadc_info *info,
			struct gpadc_regs_t *regs)
{
	int timeout = 0, tmp = 0, ret = 0;

	ret = regmap_read(info->regmap, regs->gpadcreq, &tmp);
	if (ret)
		return ret;

	while (tmp & regs->gpadcreq_busy && timeout < 500) {
		ret = regmap_read(info->regmap, regs->gpadcreq, &tmp);
		if (ret)
			return ret;
		usleep_range(1800, 2000);
		timeout++;
	}

	if (tmp & regs->gpadcreq_busy)
		return -EBUSY;
	else
		return 0;
}

static void gpadc_dump(struct gpadc_info *info)
{
	int tmp, ret = 0;
	struct gpadc_regs_t *regs = info->gpadc_regs;

	dev_err(info->dev, "GPADC registers dump:\n");

	ret = regmap_read(info->regmap, regs->adcirq, &tmp);
	if (ret)
		dev_err(info->dev, "Err in reading ADCIRQ\n");
	else
		dev_err(info->dev, "ADCIRQ: 0x%x\n", tmp);

	ret = regmap_read(info->regmap, regs->madcirq, &tmp);
	if (ret)
		dev_err(info->dev, "Err in reading MADCIRQ\n");
	else
		dev_err(info->dev, "MADCIRQ: 0x%x\n", tmp);

	ret = regmap_read(info->regmap, regs->gpadcreq, &tmp);
	if (ret)
		dev_err(info->dev, "Err in reading GPADCREQ\n");
	else
		dev_err(info->dev, "GPADCREQ: 0x%x\n", tmp);

	ret = regmap_read(info->regmap, regs->adc1cntl, &tmp);
	if (ret)
		dev_err(info->dev, "Err in reading ADC1CNTL\n");
	else
		dev_err(info->dev, "ADC1CNTL: 0x%x\n", tmp);
}

static irqreturn_t gpadc_threaded_isr(int irq, void *data)
{
	struct gpadc_info *info = iio_priv(data);

	info->sample_done = 1;
	wake_up(&info->wait);

	return IRQ_HANDLED;
}

/**
 * iio_whiskeycove_gpadc_sample - do gpadc sample.
 * @indio_dev: industrial IO GPADC device handle
 * @ch: gpadc bit set of channels to sample, for example, set ch = (1<<0)|(1<<2)
 *	means you are going to sample both channel 0 and 2 at the same time.
 * @res:gpadc sampling result
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */
int iio_whiskeycove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res)
{
	struct gpadc_info *info = iio_priv(indio_dev);
	int i, ret = 0, reg_val;
	int adc_req = 0, irq_en = 0, th, tl, adc_irq, mask = 0;
	u8 cursrc;
	unsigned long rlsb;
	static const unsigned long rlsb_array[] = {
		0, 260420, 130210, 65100, 32550, 16280,
		8140, 4070, 2030, 0, 260420, 130210};

	struct gpadc_regs_t *regs = info->gpadc_regs;

	if (!info->initialized)
		return -ENODEV;

	mutex_lock(&info->lock);

	for (i = 0; i < info->channel_num; i++) {
		if (ch & (1 << i)) {
			adc_req |= (1 << info->gpadc_regmaps[i].cntl);
			irq_en |= (1 << info->gpadc_regmaps[i].irq_en);
		}
	}
	regmap_update_bits(info->regmap, regs->madcirq, irq_en, 0x0);
	regmap_update_bits(info->regmap, regs->mirqlvl1,
				regs->mirqlvl1_adc, 0x0);
	regmap_update_bits(info->regmap, regs->adcirq, 0xFF, 0xFF);

	info->sample_done = 0;

	ret = gpadc_busy_wait(info, regs);
	if (ret) {
		dev_err(info->dev, "GPADC is busy\n");
		goto done;
	}

	regmap_write(info->regmap, regs->gpadcreq, adc_req);

	ret = wait_event_timeout(info->wait, info->sample_done, HZ);

	/* Chek if the required IRQ bits are set */
	ret = regmap_read(info->regmap, regs->adcirq, &adc_irq);
	if (ret)
		goto done;
	if (!(adc_irq & irq_en)) {
		gpadc_dump(info);
		ret = -EIO;
		dev_err(info->dev, "Error in Sample%d\n", ret);
		goto done;
	}

	ret = 0;

	for (i = 0; i < info->channel_num; i++) {
		if (ch & (1 << i)) {
			ret = regmap_read(info->regmap,
					info->gpadc_regmaps[i].rsltl, &tl);
			ret |= regmap_read(info->regmap,
					info->gpadc_regmaps[i].rslth, &th);
			if (ret)
				goto done;

			reg_val = ((th & 0xF) << 8) + tl;
			switch (i) {
			case PMIC_GPADC_CHANNEL_VBUS:
			case PMIC_GPADC_CHANNEL_PMICTEMP:
			case PMIC_GPADC_CHANNEL_PEAK:
			case PMIC_GPADC_CHANNEL_AGND:
			case PMIC_GPADC_CHANNEL_VREF:
				/* Auto mode not applicable */
				res->data[i] = reg_val;
				break;
			case PMIC_GPADC_CHANNEL_BATID:
			case PMIC_GPADC_CHANNEL_BATTEMP0:
			case PMIC_GPADC_CHANNEL_BATTEMP1:
			case PMIC_GPADC_CHANNEL_SYSTEMP0:
			case PMIC_GPADC_CHANNEL_SYSTEMP1:
			case PMIC_GPADC_CHANNEL_SYSTEMP2:
			case PMIC_GPADC_CHANNEL_USBID:
				/* Auto mode without Scaling */
				cursrc = (th & 0xF0) >> 4;
				rlsb = rlsb_array[cursrc];
				res->data[i] = (reg_val * rlsb)/10000;
				break;
			}
		}
	}
done:
	regmap_update_bits(info->regmap, regs->mirqlvl1,
					regs->mirqlvl1_adc, regs->mirqlvl1_adc);
	regmap_update_bits(info->regmap, regs->madcirq, mask, mask);
	regmap_update_bits(info->regmap, regs->adcirq, mask, mask);
	mutex_unlock(&info->lock);
	return ret;
}
EXPORT_SYMBOL(iio_whiskeycove_gpadc_sample);

static struct gpadc_result sample_result;
static int chs;

static ssize_t intel_whiskeycove_gpadc_store_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	if (kstrtol(buf, 16, (long *)&chs) != 1) {
		dev_err(dev, "one channel argument is needed\n");
		return -EINVAL;
	}

	if (chs < (1 << 0) || chs >= (1 << info->channel_num)) {
		dev_err(dev, "invalid channel, should be in [0x1 - 0x%x]\n",
				info->channel_num);
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_whiskeycove_gpadc_show_channel(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", chs);
}

static ssize_t intel_whiskeycove_gpadc_store_sample(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int value, ret;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	memset(sample_result.data, 0, sizeof(sample_result.data));

	if (kstrtol(buf, 10, (long *)&value) != 1) {
		dev_err(dev, "one argument is needed\n");
		return -EINVAL;
	}

	if (value == 1) {
		ret = iio_whiskeycove_gpadc_sample(indio_dev, chs,
						&sample_result);
		if (ret) {
			dev_err(dev, "sample failed\n");
			return ret;
		}
	} else {
		dev_err(dev, "input '1' to sample\n");
		return -EINVAL;
	}

	return size;
}
static ssize_t intel_whiskeycove_gpadc_show_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	int used = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	for (i = 0; i < info->channel_num; i++) {
		used += snprintf(buf + used, PAGE_SIZE - used,
				"sample_result[%s] = %x\n",
				info->gpadc_regmaps[i].name,
				sample_result.data[i]);
	}

	return used;
}

static DEVICE_ATTR(channel, S_IWUSR | S_IRUGO,
		intel_whiskeycove_gpadc_show_channel,
		intel_whiskeycove_gpadc_store_channel);
static DEVICE_ATTR(sample, S_IWUSR, NULL, intel_whiskeycove_gpadc_store_sample);
static DEVICE_ATTR(result, S_IRUGO, intel_whiskeycove_gpadc_show_result, NULL);

static struct attribute *intel_whiskeycove_gpadc_attrs[] = {
	&dev_attr_channel.attr,
	&dev_attr_sample.attr,
	&dev_attr_result.attr,
	NULL,
};
static struct attribute_group intel_whiskeycove_gpadc_attr_group = {
	.name = "whiskeycove_gpadc",
	.attrs = intel_whiskeycove_gpadc_attrs,
};
static u16 get_tempzone_val(u16 resi_val)
{
	u8 cursel = 0, hys = 0;
	u16 trsh = 0, count = 0, bsr_num = 0;
	u16 tempzone_val = 0;

	/* multiply to convert into Ohm*/
	resi_val *= OHM_MULTIPLIER;

	/* CUR = max(floor(log2(round(ADCNORM/2^5)))-7,0)
	 * TRSH = round(ADCNORM/(2^(4+CUR)))
	 * HYS = if(∂ADCNORM>0 then max(round(∂ADCNORM/(2^(7+CUR))),1)
	 * else 0
	 */

	/*
	 * while calculating the CUR[2:0], instead of log2
	 * do a BSR (bit scan reverse) since we are dealing with integer values
	 */
	bsr_num = resi_val;
	bsr_num /= (1 << 5);

	while (bsr_num >>= 1)
		count++;

	cursel = max((count - 7), 0);

	/* calculate the TRSH[8:0] to be programmed */
	trsh = ((resi_val) / (1 << (4 + cursel)));

	tempzone_val = (hys << 12) | (cursel << 9) | trsh;

	return tempzone_val;
}

static int whiskeycove_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	int ret;
	int ch = chan->channel;
	struct gpadc_info *info = iio_priv(indio_dev);
	struct gpadc_result res;

	ret = iio_whiskeycove_gpadc_sample(indio_dev, (1 << ch), &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		return -EINVAL;
	}

	*val = res.data[ch];

	return ret;
}

static int whiskeycove_adc_read_event_value(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type, enum iio_event_direction dir,
			enum iio_event_info info, int *val, int *val2)
{
	int ch = chan->channel;
	u16 reg_h, reg_l;
	int val_h, val_l, ret = 0;
	struct gpadc_info *gp_info = iio_priv(indio_dev);

	dev_dbg(gp_info->dev, "ch:%d, adc_read_event: %s-%s-%s\n", ch,
			iio_ev_type_text[type], iio_ev_dir_text[dir],
			iio_ev_info_text[info]);

	if (type == IIO_EV_TYPE_THRESH) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_max_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_max_l;
			break;
		case IIO_EV_DIR_FALLING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_min_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_min_l;
			break;
		default:
			dev_err(gp_info->dev,
				"iio_event_direction %d not supported\n",
				dir);
			return -EINVAL;
		}
	} else {
		dev_err(gp_info->dev,
				"iio_event_type %d not supported\n",
				type);
		return -EINVAL;
	}

	dev_dbg(gp_info->dev, "reg_h:%x, reg_l:%x\n", reg_h, reg_l);

	ret = regmap_read(gp_info->regmap, reg_l, &val_l);
	if (ret)
		return -EIO;
	ret = regmap_read(gp_info->regmap, reg_h, &val_h);
	if (ret)
		return -EIO;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = ((val_h & 0xF) << 8) + val_l;
		if (ch != PMIC_GPADC_CHANNEL_PMICTEMP) {
				int thrsh = *val & 0x01FF;
				int cur = (*val >> 9) & 0x07;

				*val = thrsh * (1 << (4 + cur))
					/ OHM_MULTIPLIER;
		}
		break;
	case IIO_EV_INFO_HYSTERESIS:
		*val = (val_h >> 4) & 0x0F;
		break;
	default:
		dev_err(gp_info->dev,
				"iio_event_info %d not supported\n",
				info);
		return -EINVAL;
	}

	dev_dbg(gp_info->dev, "read_event_sent: %x\n", *val);
	return IIO_VAL_INT;
}

static int whiskeycove_adc_write_event_value(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type, enum iio_event_direction dir,
			enum iio_event_info info, int val, int val2)
{
	int err;
	int ch = chan->channel;
	u16 reg_h, reg_l;
	u8 val_h, val_l, mask;
	struct gpadc_info *gp_info = iio_priv(indio_dev);

	dev_dbg(gp_info->dev, "ch:%d, adc_write_event: %s-%s-%s\n", ch,
			iio_ev_type_text[type], iio_ev_dir_text[dir],
			iio_ev_info_text[info]);

	dev_dbg(gp_info->dev, "write_event_value: %x\n", val);

	if (type == IIO_EV_TYPE_THRESH) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_max_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_max_l;
			break;
		case IIO_EV_DIR_FALLING:
			reg_h = gp_info->gpadc_regmaps[ch].alrt_min_h;
			reg_l = gp_info->gpadc_regmaps[ch].alrt_min_l;
			break;
		default:
			dev_err(gp_info->dev,
				"iio_event_direction %d not supported\n",
				dir);
			return -EINVAL;
		}
	} else {
		dev_err(gp_info->dev,
				"iio_event_type %d not supported\n",
				type);
		return -EINVAL;
	}

	dev_dbg(gp_info->dev, "reg_h:%x, reg_l:%x\n", reg_h, reg_l);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (ch != PMIC_GPADC_CHANNEL_PMICTEMP)
			val = get_tempzone_val(val);

		val_h = (val >> 8) & 0xF;
		val_l = val & 0xFF;
		mask = 0x0F;

		err = regmap_update_bits(gp_info->regmap, reg_l, 0xFF, val_l);
		if (err) {
			dev_err(gp_info->dev, "Error updating register:%X\n",
				reg_l);
			return -EINVAL;
		}
		break;
	case IIO_EV_INFO_HYSTERESIS:
		val_h = (val << 4) & 0xF0;
		mask = 0xF0;
		break;
	default:
		dev_err(gp_info->dev,
				"iio_event_info %d not supported\n",
				info);
		return -EINVAL;
	}

	err = regmap_update_bits(gp_info->regmap, reg_h, mask, val_h);
	if (err) {
		dev_err(gp_info->dev, "Error updating register:%X\n", reg_h);
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_info whiskeycove_adc_info = {
	.read_raw = &whiskeycove_adc_read_raw,
	.read_event_value = &whiskeycove_adc_read_event_value,
	.write_event_value = &whiskeycove_adc_write_event_value,
	.driver_module = THIS_MODULE,
};

static int wcove_gpadc_probe(struct platform_device *pdev)
{
	int err, irq, virq;
	u8 pmic_prov;
	struct gpadc_info *info;
	struct iio_dev *indio_dev;
	struct gpadc_regs_t *regs;
	struct intel_soc_pmic *wcove = dev_get_drvdata(pdev->dev.parent);
	struct regmap_irq_chip_data *regmap_irq_chip;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "allocating iio device failed\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);

	mutex_init(&info->lock);
	init_waitqueue_head(&info->wait);
	info->dev = &pdev->dev;
	irq = platform_get_irq(pdev, 0);
	info->intr_mask = MUSBID | MPEAK | MBATTEMP
		| MSYSTEMP | MBATT | MVIBATT | MGPMEAS | MCCTICK;
	info->channel_num = GPADC_NUM_CHANNELS;
	info->gpadc_regmaps = whiskeycove_gpadc_regmaps;
	info->gpadc_regs = &whiskeycove_gpadc_regs;
	info->regmap = wcove->regmap;

	regmap_irq_chip = wcove->irq_chip_data_level2;

	virq = regmap_irq_get_virq(regmap_irq_chip, irq);
	if (virq < 0) {
		dev_err(&pdev->dev,
				"failed to get virtual irq for :%d\n", irq);
		return virq;
	}


	err = devm_request_threaded_irq(info->dev, virq,
			NULL, gpadc_threaded_isr, IRQF_ONESHOT,
			"wcove_gpadc", indio_dev);
	if (err) {
		dev_err(&pdev->dev, "unable to register irq %d\n", info->irq);
		return err;
	}

	regs = info->gpadc_regs;
	regmap_update_bits(info->regmap, regs->mirqlvl1,
			regs->mirqlvl1_adc, regs->mirqlvl1_adc);
	regmap_update_bits(info->regmap, regs->madcirq, 0xFF, 0xFF);

	/* configure sample periods */
	regmap_write(info->regmap, regs->thrmmonctl, THERM_MON_VAL);
	regmap_write(info->regmap, regs->batthermonctl,
			BATTHERM_MON_VAL);
	regmap_write(info->regmap, regs->vbatmonctl, VBAT_MON_VAL);
	regmap_write(info->regmap, regs->gpmonctl, GPM_MON_VAL);

	regmap_update_bits(info->regmap, regs->adcirq, 0xFF, 0xFF);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;

	indio_dev->channels =  wc_adc_channels;
	indio_dev->num_channels = GPADC_NUM_CHANNELS;
	indio_dev->info = &whiskeycove_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_map_array_register(indio_dev, wc_iio_maps);
	if (err)
		return err;

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto err_array_unregister;

	err = regmap_read(info->regmap, PMIC_ID_ADDR,
				(int *)&info->pmic_id);
	if ((err) || (info->pmic_id < 0)) {
		dev_err(&pdev->dev, "Error reading PMIC ID register\n");
		goto err_iio_device_unregister;
	}

	dev_info(&pdev->dev, "PMIC-ID: %x\n", info->pmic_id);
	/* Check if PMIC is provisioned */
	err = regmap_read(info->regmap, PMIC_SPARE03_ADDR,
				(int *)&pmic_prov);
	if ((err) || (pmic_prov < 0)) {
		dev_err(&pdev->dev,
				"Error reading PMIC SPARE03 REG\n");
		goto err_iio_device_unregister;
	}

	if ((pmic_prov & PMIC_PROV_MASK) == PMIC_PROVISIONED) {
		dev_info(&pdev->dev, "PMIC provisioned\n");
		info->is_pmic_provisioned = true;
	} else {
		dev_info(info->dev, "PMIC not provisioned\n");
	}

	err = sysfs_create_group(&pdev->dev.kobj,
			&intel_whiskeycove_gpadc_attr_group);
	if (err) {
		dev_err(&pdev->dev, "Unable to export sysfs interface, error: %d\n",
			err);
		goto err_iio_device_unregister;
	}

	info->initialized = 1;

	dev_dbg(&pdev->dev, "wcove adc probed\n");

	return 0;

err_iio_device_unregister:
	iio_device_unregister(indio_dev);
err_array_unregister:
	iio_map_array_unregister(indio_dev);

	return err;
}

static int wcove_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj,
			&intel_whiskeycove_gpadc_attr_group);

	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int wcove_gpadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	if (!mutex_trylock(&info->lock))
		return -EBUSY;

	return 0;
}

static int wcove_gpadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	mutex_unlock(&info->lock);
	return 0;
}
#endif

static const struct dev_pm_ops wcove_gpadc_driver_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wcove_gpadc_suspend,
				wcove_gpadc_resume)
};

static struct platform_device_id wcove_gpadc_id_table[] = {
	{ .name = "wcove_gpadc" },
	{ .name = "bxt_wcove_gpadc" },
	{},
};
MODULE_DEVICE_TABLE(platform, wcove_gpadc_id_table);

static struct platform_driver wcove_gpadc_driver = {
	.driver = {
		   .name = "whiskey_cove_gpadc",
		   .owner = THIS_MODULE,
		   .pm = &wcove_gpadc_driver_pm_ops,
		   },
	.probe = wcove_gpadc_probe,
	.remove = wcove_gpadc_remove,
	.id_table = wcove_gpadc_id_table,
};
module_platform_driver(wcove_gpadc_driver);

MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Whiskey Cove GPADC Driver");
MODULE_LICENSE("GPL");
