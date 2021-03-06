/*
 *  temt6200.c - Linux kernel modules for ambient light sensor
 * 
 *  Copyright (C) 2013 Ran Meyerstein <ranm@micronet.co.il>
 *  Copyright (C) 2007 Rodolfo Giometti <giometti@linux.it>
 *  Copyright (C) 2007 Eurotech S.p.A. <info@eurotech.it>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#define TEMT6200_DRV_NAME	"temt6200"
#define DRIVER_VERSION		"1.0"

/*
 * Defines
 */
#define LIGHTE_SENSE_SAMPLES 20

/* start time delay for light sensor in nano seconds */
#define LIGHT_SENSOR_START_TIME_DELAY 50000000

#define BUFFER_NUM	6

#define temt6200_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
//#define temt6200_dbgmsg(str, args...) printk("%s: " str, __func__, ##args)
/*
 * Structs
 */

struct temt6200_data {
	struct input_dev *light_input_dev;
	struct workqueue_struct *wq;
	struct wake_lock prx_wake_lock;
	struct work_struct work_light;
	struct hrtimer timer;
	ktime_t light_poll_delay;
	int lux_value_buf[BUFFER_NUM];
	int index_count;
	bool buf_initialized;

	unsigned int power_state:1;
	unsigned int operating_mode:1;
	unsigned int enable:1;
};

/*
 * Global data
 */

static struct temt6200_data * temt6200;
int mvArr[]  = {14, 17, 23,  25,  28,  50,  70,  104, 143, 170, 250,  320,  400,  450,  500,  1700,  3000}; //measured table
int luxArr[] = {0,  35, 65, 240, 380, 560, 600, 700, 750, 684, 1400, 2500, 3200, 5000, 6000, 10000, 20000};//measured table

/*
 * Management functions
 */

static int temt6200_calculate_lux(int ch0)
{
	int lux, i = 0;

	/* Calculate LUX */
	//lux = ((ch0 * 17) / 10 ) - 49;//Linear
	//lux = (((ch0 ^ 3) * 4)/ 1000000) - (((ch0 ^ 2) * 43) / 10000) + ((ch0 * 249) / 100) - 63; // Third degree polynom

	//Table linear inetrpolation
	while ( mvArr[i] < ch0) {
		i += 1;
	}

	if (i == 0)
	{ 	
		lux = luxArr[0];
	}
	else if (i > 16) {
		lux = luxArr[16];
	}
	else {
		lux = (((luxArr[i] - luxArr[i-1]) * ((( ch0 - mvArr[i-1]) << 10) / (mvArr[i] - mvArr[i-1]))) >> 10) + luxArr[i-1];
	}

	return lux;
}

static void temt6200_light_enable(struct temt6200_data *data)
{
	temt6200_dbgmsg("starting poll timer, delay %lldns\n",
		    ktime_to_ns(data->light_poll_delay));
	/* push -1 to input subsystem to enable real value to go through next */
	input_report_abs(data->light_input_dev, ABS_MISC, -1);
	hrtimer_start(&data->timer, ktime_set(0, LIGHT_SENSOR_START_TIME_DELAY),
					HRTIMER_MODE_REL);
}

static void temt6200_light_disable(struct temt6200_data *data)
{
	temt6200_dbgmsg("cancelling poll timer\n");
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work_light);
	/* mark the adc buff as not initialized
	 * so that it will be filled again on next light sensor start
	 */
	data->buf_initialized = false;
}

/*
 * SysFS support
 */

static ssize_t temt6200_show_power_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct temt6200_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->power_state);
}

static ssize_t temt6200_store_power_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct temt6200_data *data = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	data->power_state = new_value;
	/* Save power state for suspend/resume */
	data->enable = new_value;

	if (new_value)
		temt6200_light_enable(data);
	else
		temt6200_light_disable(data);

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO | S_IWGRP,
		   temt6200_show_power_state, temt6200_store_power_state);

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct temt6200_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(data->light_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct temt6200_data *data = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	temt6200_dbgmsg("new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(data->light_poll_delay));
	if (new_delay != ktime_to_ns(data->light_poll_delay)) {
		data->light_poll_delay = ns_to_ktime(new_delay);
		if (data->power_state) {
			temt6200_light_disable(data);
			temt6200_light_enable(data);
		}
	}

	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   poll_delay_show, poll_delay_store);

static ssize_t temt6200_show_operating_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct temt6200_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->operating_mode);
}

static ssize_t temt6200_store_operating_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct temt6200_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val < 0 || val > 1)
		return -EINVAL;

	if (data->power_state == 0)
		return -EBUSY;

	data->operating_mode = val;

	return count;
}

static DEVICE_ATTR(operating_mode, S_IWUSR | S_IRUGO,
		   temt6200_show_operating_mode, temt6200_store_operating_mode);

static int __temt6200_show_lux(void)
{
	int ret, averageVolts = 0, i, mv = 0;
	
	for (i = 0; i < (LIGHTE_SENSE_SAMPLES); i++) 
	{
		// Sense ambient light
		ret = ls_conversion();

		mv = (ret >> 16) & 0xFFFF;

		averageVolts += mv;

		msleep(50);
	}

	averageVolts /= LIGHTE_SENSE_SAMPLES;

	temt6200_dbgmsg("averageVolts %d\n", averageVolts);

	/* Do the job */
	ret = temt6200_calculate_lux(averageVolts);
	if (ret < 0)
		return 0;

	return ret;
}

static struct attribute *temt6200_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_operating_mode.attr,
	NULL
};

static const struct attribute_group temt6200_attr_group = {
	.attrs = temt6200_attributes,
};

static int lightsensor_get_luxvalue(struct temt6200_data *data)
{
#ifdef AVERAGE_TEMT6200
	int i = 0;
	int j = 0;
	unsigned int lux_total = 0;
	int lux_avr_value;
	unsigned int index = 0;
	unsigned int lux_max = 0;
	unsigned int lux_min = 0;
#endif
	int value = 0;

	/* get lux value */
	value = __temt6200_show_lux();

	if (value < 0) {
		pr_err("lightsensor returned error %d\n", value);
		return value;
	}
	temt6200_dbgmsg("light value %d\n", value);
#ifdef AVERAGE_TEMT6200
	index = (data->index_count++) % BUFFER_NUM;

	/* buffer initialize (light sensor off ---> light sensor on) */
	if (!data->buf_initialized) {
		data->buf_initialized = true;
		for (j = 0; j < BUFFER_NUM; j++)
			data->lux_value_buf[j] = value;
	} else
		data->lux_value_buf[index] = value;

	lux_max = data->lux_value_buf[0];
	lux_min = data->lux_value_buf[0];

	for (i = 0; i < BUFFER_NUM; i++) {
		lux_total += data->lux_value_buf[i];

		if (lux_max < data->lux_value_buf[i])
			lux_max = data->lux_value_buf[i];

		if (lux_min > data->lux_value_buf[i])
			lux_min = data->lux_value_buf[i];
	}
	lux_avr_value = (lux_total-(lux_max+lux_min))/(BUFFER_NUM-2);

	if (data->index_count == BUFFER_NUM)
		data->index_count = 0;

	temt6200_dbgmsg("average light value %d\n", lux_avr_value);
	return lux_avr_value;
#else
	return value;
#endif

}

static void temt6200_work_func_light(struct work_struct *work)
{
	struct temt6200_data *data = container_of(work, struct temt6200_data,
					      work_light);

	int adc = lightsensor_get_luxvalue(data);
	if (adc >= 0) {
		input_report_abs(data->light_input_dev, ABS_MISC, adc);
		input_sync(data->light_input_dev);
	}
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart temt6200_timer_func(struct hrtimer *timer)
{
	struct temt6200_data *data = container_of(timer, struct temt6200_data, timer);
	queue_work(data->wq, &data->work_light);
	hrtimer_forward_now(&data->timer, data->light_poll_delay);
	return HRTIMER_RESTART;
}


static int __devinit temt6200_probe(struct platform_device *pdev)
{
	struct input_dev *input_dev;
	struct temt6200_data *data;
	int err;

	data = kzalloc(sizeof(struct temt6200_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		goto exit_kfree;
	}

	temt6200 = data;
	platform_set_drvdata(pdev, temt6200);

	wake_lock_init(&data->prx_wake_lock, WAKE_LOCK_SUSPEND,
		"prx_wake_lock");

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->light_poll_delay = ns_to_ktime(50 * NSEC_PER_MSEC);
	data->timer.function = temt6200_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	 * to read the i2c (can be slow and blocking)
	 */
	data->wq = create_singlethread_workqueue("temt6200_wq");
	if (!data->wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&data->work_light, temt6200_work_func_light);

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, data);
	input_dev->name = "lightsensor-level";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	printk("registering lightsensor-level input device\n");
	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	data->light_input_dev = input_dev;
	err = sysfs_create_group(&input_dev->dev.kobj,
				 &temt6200_attr_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	printk("support ver. %s enabled\n", DRIVER_VERSION);

	return 0;
	/* error, unwind it all */
err_sysfs_create_group_light:
	input_unregister_device(data->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(data->wq);
err_create_workqueue:
	wake_lock_destroy(&data->prx_wake_lock);
exit_kfree:
	kfree(data);

	return err;
}

static int __devexit temt6200_remove(struct platform_device *pdev)
{
	struct temt6200_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&data->light_input_dev->dev.kobj,
			   &temt6200_attr_group);

	destroy_workqueue(data->wq);
	input_unregister_device(data->light_input_dev);

	/* Power down the device */
	data->power_state = 0;

	wake_lock_destroy(&data->prx_wake_lock);

	return 0;
}

#ifdef CONFIG_PM

static int temt6200_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct temt6200_data *data = platform_get_drvdata(pdev);

	if (data->enable)
		temt6200_light_disable(data);

	data->power_state = 0;

	return 0;
}

static int temt6200_resume(struct platform_device *pdev)
{
	struct temt6200_data *data = platform_get_drvdata(pdev);

	data->power_state = 1;

	return 0;
}

#else

#define temt6200_suspend		NULL
#define temt6200_resume		NULL

#endif /* CONFIG_PM */


static struct platform_driver temt6200_driver = {
	.driver = {
		.name	= TEMT6200_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = temt6200_suspend,
	.resume	= temt6200_resume,
	.probe	= temt6200_probe,
	.remove	= __devexit_p(temt6200_remove),
};

static int __init temt6200_init(void)
{
	return platform_driver_register(&temt6200_driver);
}

static void __exit temt6200_exit(void)
{
	platform_driver_unregister(&temt6200_driver);
}

MODULE_AUTHOR("Ran Meyerstein <ranm@micronet.co.il>");
MODULE_DESCRIPTION("TEMT6200 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(temt6200_init);
module_exit(temt6200_exit);
