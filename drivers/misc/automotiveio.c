/*
 *  automotiveio.c - Linux kernel modules for automotive input and output
 * 
 *  Copyright (C) 2013 Ran Meyerstein <ranm@micronet.co.il>
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

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/automotiveio.h>
#include <linux/ignition.h>
#include <linux/irq.h>
#include <linux/delay.h>


/*
 * Defines
 */

#if defined (CONFIG_MACH_A300)
#define A300AUTOMOTIVEIO_DRV_NAME	"A300 AutomotiveIO"
#else
#define A300AUTOMOTIVEIO_DRV_NAME	"a31x AutomotiveIO"
#endif
#define DRIVER_VERSION		"1.0"
#define AUTIO_INPUT_EVENTS_1_MASK 1 //bit 0 is input 1 indication 
#define AUTIO_INPUT_EVENTS_2_MASK 2 //bit 1 is input 2 indication
#define AUTIO_INPUT_EVENTS_3_MASK 4 //bit 2 is cradle indication
#define AUTIO_INPUT_EVENTS_4_MASK 8 //bit 3 is wall (power cable connector indicator)  2 indication
#define ACTIVE_LOW 1

#define automotiveio_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
//#define automotiveio_dbgmsg(str, args...) printk("%s: " str, __func__, ##args)
/*
 * Structs
 */

struct automotiveio_data {
	struct input_dev *autmotiveio_input_dev;
	struct automotiveio_platform_data *pdata;
	struct hrtimer timer_modem;
	struct delayed_work wall_debounce_work;
	struct delayed_work cradle_debounce_work;
	struct delayed_work input2_debounce_work;
	struct delayed_work input1_debounce_work;
	struct workqueue_struct *wq;
	struct work_struct modem_on_off_debounce_work;
	spinlock_t irq_lock;
	struct wake_lock wake_lock;
	struct mutex 		work_lock;
	int debounce_time;
	int irq_input1;
	int input1_num_read;
	int input1startreadtime;
	int enable;
	int linestateirq;
	int irq_input2;
	int input2_num_read;
	int input2startreadtime;
	int wall_num_read;
	int wallstartreadtime;
	int irq_wall;
	int cradlestateirq;
	int cradle_num_read;
	int cradlestartreadtime;
	int irq_cradle;
	int inputreport;
	int modemstate;
	int modemstateinprocess;
	int modemresetstate;
};

/*
 * Global data
 */

struct automotiveio_data * aio_data;

/* Notification for ignition events */
static RAW_NOTIFIER_HEAD(automotiveio_chain);

/* Protection for the above */
static DEFINE_RAW_SPINLOCK(automotive_lock);

static int last_in1_sent = 0;
static int last_in1_suspend = 0;

/*
 * External API
 */

int automotiveio_get_input_state()
{
	int ret = -1;

	if (aio_data) {
		ret	= gpio_get_value(aio_data->pdata->gpio_in_1);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(automotiveio_get_input_state);

int automotiveio_get_input2_state()
{
	int ret = -1;

	if ( -1 == aio_data->pdata->gpio_in_2) {
		return ret;
	}

	if (aio_data) {
		ret	= gpio_get_value(aio_data->pdata->gpio_in_2);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(automotiveio_get_input2_state);

int automotiveio_get_cradle_state()
{
	int ret = -1;

	if ( -1 == aio_data->pdata->gpio_cradle) {
		return ret;
	}

	if (aio_data) {
		ret	= (ACTIVE_LOW != gpio_get_value(aio_data->pdata->gpio_cradle));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(automotiveio_get_cradle_state);

int automotiveio_get_wall_state()
{
	int ret = -1;

	if ( -1 == aio_data->pdata->gpio_wall ) {
		return ret;
	}

	if (aio_data) {
		ret	= gpio_get_value(aio_data->pdata->gpio_wall);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(automotiveio_get_wall_state);

/*
 * SysFS support
 */
int automotive_modem_power_control_show_state(char *buf, const struct kernel_param *kp)
{
	int gpio;

	if ( aio_data->pdata->boardversion < 16 ) {
		gpio = aio_data->pdata->gpio_out;
	}
	else
	{
		gpio = aio_data->pdata->gpio_modem_power_ctl;
	}
	
	if ( -1 == gpio ) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	return sprintf(buf, "%d", gpio_get_value(gpio));
}

int automotive_modem_power_control_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value = 0, gpio;

	if (aio_data->pdata->boardversion < 16) {
		gpio = aio_data->pdata->gpio_out;
	}
	else
	{
		gpio = aio_data->pdata->gpio_modem_power_ctl;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;

	if ( -1 != gpio ) {
		gpio_set_value(gpio, new_value);
	}
	else
	{
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}
		
	return 0;
}

static struct kernel_param_ops automotive_modem_power_control_ops_str = {
	.set = automotive_modem_power_control_set_state,
	.get = automotive_modem_power_control_show_state,
};
module_param_cb(modem_power_control, &automotive_modem_power_control_ops_str, NULL, 0664);

int automotive_modem_usb_en_show_state(char *buf, const struct kernel_param *kp)
{
	if ( -1 != aio_data->pdata->gpio_modem_usb_en ) {
		return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_modem_usb_en));
	}
	return sprintf(buf, "NO HW Support revision is not B");
}

int automotive_modem_usb_en_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value = 0;

	if (-1 == aio_data->pdata->gpio_modem_usb_en) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;

	gpio_set_value(aio_data->pdata->gpio_modem_usb_en, new_value); //M307i 846 rev B
		
	return 0;
}

static struct kernel_param_ops automotive_modem_usb_en_ops_str = {
	.set = automotive_modem_usb_en_set_state,
	.get = automotive_modem_usb_en_show_state,
};
module_param_cb(modem_usb_en, &automotive_modem_usb_en_ops_str, NULL, 0664);

int automotive_direct_reset_modem_show_state(char *buf, const struct kernel_param *kp)
{
	if (-1 == aio_data->pdata->gpio_modem_uncon_shutdown ) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_modem_uncon_shutdown));
}

int automotive_direct_reset_modem_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value = 0;

	if ( -1 == aio_data->pdata->gpio_modem_uncon_shutdown) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (sysfs_streq(buf, "1")) {
		new_value = 1;
	}

	gpio_set_value(aio_data->pdata->gpio_modem_uncon_shutdown, new_value);

	return 0;
}

static struct kernel_param_ops automotive_direct_reset_modem_ops_str = {
	.set = automotive_direct_reset_modem_set_state,
	.get = automotive_direct_reset_modem_show_state,
};
module_param_cb(direct_reset_modem, &automotive_direct_reset_modem_ops_str, NULL, 0664);

int automotive_reset_modem_show_state(char *buf, const struct kernel_param *kp)
{
	if (aio_data->pdata->boardversion < 16) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", aio_data->modemresetstate);
}

int automotive_reset_modem_set_state(const char *buf, const struct kernel_param *kp)
{
	ktime_t ktime;
	int new_value;//If 3G MODEM is on, line must be up for 3 seconds

	if ( -1 == aio_data->pdata->gpio_modem_on_off || 
		 -1 == aio_data->pdata->gpio_modem_uncon_shutdown) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else {
		pr_err("%s: invalid value [%c] set '1' for reset MODEM\n", __func__, *buf);
		return -EINVAL;
	}
	/* Don't accept any commands untill turn off/on process is completed */
	if ((aio_data->modemresetstate != 0) || (aio_data->modemstateinprocess != 0))
	{
		pr_err("%s: Command aborted - MODEM is in reset or on/off process\n", __func__);
		return 0;
	}

	aio_data->modemresetstate = 1;
	aio_data->modemstateinprocess = 1;

	if ( -1 != aio_data->pdata->gpio_modem_usb_en ) { //M307i 846 rev B
		gpio_set_value(aio_data->pdata->gpio_modem_usb_en, aio_data->pdata->gpio_modem_usb_en_pol ? 0 : 1); //Disconnect USB each turn OFF/ON	
	}

	gpio_set_value(aio_data->pdata->gpio_modem_uncon_shutdown, new_value);			
	gpio_set_value(aio_data->pdata->gpio_modem_on_off, new_value);

	ktime = ktime_set(5,0);

    hrtimer_start(&aio_data->timer_modem, ktime, HRTIMER_MODE_REL); 

	return 0;
}

static struct kernel_param_ops automotive_reset_modem_ops_str = {
	.set = automotive_reset_modem_set_state,
	.get = automotive_reset_modem_show_state,
};
module_param_cb(reset_modem, &automotive_reset_modem_ops_str, NULL, 0664);

int automotive_direct_on_off_modem_show_state(char *buf, const struct kernel_param *kp)
{
	if (-1 == aio_data->pdata->gpio_modem_on_off ) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_modem_on_off));
}

int automotive_direct_on_off_modem_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value = 0;

	if ( -1 == aio_data->pdata->gpio_modem_on_off ) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (sysfs_streq(buf, "1")) {
		new_value = 1;
	}

	gpio_set_value(aio_data->pdata->gpio_modem_on_off, new_value);

	return 0;
}

static struct kernel_param_ops automotive_direct_on_off_modem_ops_str = {
	.set = automotive_direct_on_off_modem_set_state,
	.get = automotive_direct_on_off_modem_show_state,
};
module_param_cb(direct_on_off_modem, &automotive_direct_on_off_modem_ops_str, NULL, 0664);


int automotive_modem_on_off_show_state(char *buf, const struct kernel_param *kp)
{
	if ((aio_data->pdata->boardversion < 16) || //Board version is not M307i
		((aio_data->pdata->modemtype != 0) && (aio_data->pdata->boardversion >= 16))) //Internal MODEM doesn't exist
	{
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}

	return sprintf(buf, "%d", aio_data->modemstate);
}

int __automotive_modem_on_off_set_state(void)
{
	ktime_t ktime;
	int delay = 3;//If 3G MODEM is on, line must be up for 3 seconds

	if ((aio_data->pdata->boardversion < 16) || //Board version is not M307i
		((aio_data->pdata->modemtype != 0) && (aio_data->pdata->boardversion >= 16))) //Internal MODEM doesn't exist
    {
		pr_err("%s: NO HW SUPPORT\n", __func__);
	    return -EINVAL;	
	}

	aio_data->modemstateinprocess = 1;
		
	if ( -1 != aio_data->pdata->gpio_modem_usb_en ) { //M307i 846 rev B
		gpio_set_value(aio_data->pdata->gpio_modem_usb_en, aio_data->pdata->gpio_modem_usb_en_pol ? 0 : 1); //Disconnect USB each turn OFF/ON	
	}

	if ( -1 != aio_data->pdata->gpio_modem_on_off)
		gpio_set_value(aio_data->pdata->gpio_modem_on_off, 1); 

	if (aio_data->modemstate == 0) {
		delay = 5; //if 3G MODEM is off, line must be up for 5 seconds
	}

	ktime = ktime_set(delay,0);

    hrtimer_start(&aio_data->timer_modem, ktime, HRTIMER_MODE_REL); 

	return 0;
}


int automotive_modem_on_off_set_state(const char *buf, const struct kernel_param *kp)
{
	if ((aio_data->pdata->boardversion < 16) || //Board version is not M307i
		((aio_data->pdata->modemtype != 0) && (aio_data->pdata->boardversion >= 16))) //Internal MODEM doesn't exist
    {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
	    return -EINVAL;	
	}

	if (sysfs_streq(buf, "1") == false) {
		pr_err("%s: invalid value [%c] set '1' for toggle\n", __func__, *buf);
		return -EINVAL;
	}
	/* Don't accept any commands untill turn off/on process is completed */
	if (aio_data->modemstateinprocess != 0) 
	{
		pr_err("%s: Command aborted - MODEM is in turn %s process\n", __func__,  
			   (aio_data->modemstate == 0) ? "ON":"OFF");
		return 0;
	}

	return __automotive_modem_on_off_set_state();
}

static struct kernel_param_ops automotive_modem_on_off_ops_str = {
	.set = automotive_modem_on_off_set_state,
	.get = automotive_modem_on_off_show_state,
};
module_param_cb(toggle_modem_on_off, &automotive_modem_on_off_ops_str, NULL, 0664);

int automotive_audioswitch_show_state(char *buf, const struct kernel_param *kp)
{
	if ( -1 == aio_data->pdata->gpio_audswitch ) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_audswitch));
}

int automotive_audioswitch_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value;

	if ( -1 == aio_data->pdata->gpio_audswitch ) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	gpio_set_value(aio_data->pdata->gpio_audswitch, new_value);

	return 0;
}

static struct kernel_param_ops automotive_audioswitch_ops_str = {
	.set = automotive_audioswitch_set_state,
	.get = automotive_audioswitch_show_state,
};
module_param_cb(audioswitch, &automotive_audioswitch_ops_str, NULL, 0664);

int automotive_output_1_show_state(char *buf, const struct kernel_param *kp)
{
	if (-1 == aio_data->pdata->gpio_out_1 ) {
		return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_out));
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_out_1));
}

int automotive_output_1_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value;

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	if ( -1 != aio_data->pdata->gpio_out_1 ) {
		gpio_set_value(aio_data->pdata->gpio_out_1, new_value);
		return 0;
	}
	
	if ( -1 != aio_data->pdata->gpio_out) {
		gpio_set_value(aio_data->pdata->gpio_out, new_value);
	}

	return 0;
}

static struct kernel_param_ops automotive_output_1_ops_str = {
	.set = automotive_output_1_set_state,
	.get = automotive_output_1_show_state,
};

module_param_cb(automotive_output_1, &automotive_output_1_ops_str, NULL, 0664);

int automotive_output_2_show_state(char *buf, const struct kernel_param *kp)
{
	if ( -1 == aio_data->pdata->gpio_out_2 ) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}

	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_out_2));
}

int automotive_output_2_set_state(const char *buf, const struct kernel_param *kp)
{
	int new_value;

	if ( -1 == aio_data->pdata->gpio_out_2 ) {
		pr_err("%s: NO HW SUPPORT %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	gpio_set_value(aio_data->pdata->gpio_out_2, new_value);

	return 0;
}

static struct kernel_param_ops automotive_output_2_ops_str = {
	.set = automotive_output_2_set_state,
	.get = automotive_output_2_show_state,
};

module_param_cb(automotive_output_2, &automotive_output_2_ops_str, NULL, 0664);

int automotive_input_1_show_state(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_in_1));
}

static struct kernel_param_ops automotive_input_1_ops_str = {
	.set = NULL,
	.get = automotive_input_1_show_state,
};

module_param_cb(automotive_input_1, &automotive_input_1_ops_str, NULL, 0444);

int automotive_input_2_show_state(char *buf, const struct kernel_param *kp)
{
	if ( -1 == aio_data->pdata->gpio_in_2) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_in_2));
}

static struct kernel_param_ops automotive_input_2_ops_str = {
	.set = NULL,
	.get = automotive_input_2_show_state,
};

module_param_cb(automotive_input_2, &automotive_input_2_ops_str, NULL, 0444);

int automotive_cradle_show_state(char *buf, const struct kernel_param *kp)
{
	if ( -1 == aio_data->pdata->gpio_cradle ) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_cradle));
}

static struct kernel_param_ops automotive_cradle_ops_str = {
	.set = NULL,
	.get = automotive_cradle_show_state,
};

module_param_cb(automotive_cradle, &automotive_cradle_ops_str, NULL, 0444);

int automotive_wall_show_state(char *buf, const struct kernel_param *kp)
{
	if ( -1 == aio_data->pdata->gpio_wall) {
		return sprintf(buf, "%s: NO HW SUPPORT\n", __func__);
	}
	return sprintf(buf, "%d", gpio_get_value(aio_data->pdata->gpio_wall));
}

static struct kernel_param_ops automotive_wall_ops_str = {
	.set = NULL,
	.get = automotive_wall_show_state,
};

module_param_cb(automotive_wall, &automotive_wall_ops_str, NULL, 0444);

static ssize_t autmotiveio_show_debounce_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct automotiveio_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->debounce_time);
}

static ssize_t autmotiveio_store_debounce_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct automotiveio_data *data = dev_get_drvdata(dev);
	long new_value;
	char *endb;

	new_value = simple_strtol(buf, &endb, 10);

	automotiveio_dbgmsg("Automotive In Store Debounce Time [%d]msec\n", (int)new_value);

	data->debounce_time = new_value;

	return count;
}

static DEVICE_ATTR(debounce_time, S_IWUSR | S_IRUGO | S_IWGRP,
		   autmotiveio_show_debounce_time, autmotiveio_store_debounce_time);

static ssize_t autmotiveio_show_report_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct automotiveio_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->enable);
}

static ssize_t autmotiveio_store_report_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct automotiveio_data *data = dev_get_drvdata(dev);
	unsigned int new_value;

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	data->enable = new_value; //always enabled

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO | S_IWGRP,
		   autmotiveio_show_report_state, autmotiveio_store_report_state);


static struct attribute *automotivein_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_debounce_time.attr,
	NULL
};

static const struct attribute_group automotivein_attr_group = {
	.attrs = automotivein_attributes
};

static enum hrtimer_restart automotiveio_event_modem_timer_func(struct hrtimer *timer)
{
	queue_work(aio_data->wq, &aio_data->modem_on_off_debounce_work);

	return HRTIMER_NORESTART;
}

/**
 * automotiveevents_register_notifier - register an automotive 
 * events change listener 
 */
int automotiveevents_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&automotive_lock, flags);
	ret = raw_notifier_chain_register(&automotiveio_chain, nb);
	raw_spin_unlock_irqrestore(&automotive_lock, flags);

	return ret;
}

/**
 * automotivenevents_notify - notification about wall, cradle 
 * and input2 events 
 */
void automotivenevents_notify(unsigned long reason, void *arg) 
{
	unsigned long flags;

	raw_spin_lock_irqsave(&automotive_lock, flags);
	raw_notifier_call_chain(&automotiveio_chain, reason, NULL);
	raw_spin_unlock_irqrestore(&automotive_lock, flags);
}

static irqreturn_t automotiveio_input_1_irq(int irq, void *devid)
{
	struct automotiveio_data *data = (struct automotiveio_data *)devid;

//	printk("input_1_irq\n");
	disable_irq_nosync(data->irq_input1);

	data->input1_num_read = 0;

	data->input1startreadtime = ktime_to_ms(ktime_get());

	schedule_delayed_work(&data->input1_debounce_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t automotiveio_input_2_irq(int irq, void *devid)
{
	struct automotiveio_data *data = (struct automotiveio_data *)devid;

	disable_irq_nosync(data->irq_input2);

	data->input2_num_read = 0;

	data->input2startreadtime = ktime_to_ms(ktime_get());

	schedule_delayed_work(&data->input2_debounce_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t automotiveio_wall_irq(int irq, void *devid)
{
	struct automotiveio_data *data = (struct automotiveio_data *)devid;

	disable_irq_nosync(data->irq_wall);

	data->wall_num_read = 0;

	data->wallstartreadtime = ktime_to_ms(ktime_get());

	schedule_delayed_work(&data->wall_debounce_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t automotiveio_cradle_irq(int irq, void *devid)
{
	struct automotiveio_data *data = (struct automotiveio_data *)devid;

	disable_irq_nosync(data->irq_cradle);

	data->cradle_num_read = 0;

	data->cradlestartreadtime = ktime_to_ms(ktime_get());

	schedule_delayed_work(&data->cradle_debounce_work, 0);

	return IRQ_HANDLED;
}

static void do_modem_on_off_debounce_work(struct work_struct *work)
{
	if (aio_data->modemresetstate == 1)
	{/* reset command - Turn unconditional shutdown bit off.
	    MODEM is turned ON after reset */
		if (-1 != aio_data->pdata->gpio_modem_uncon_shutdown)
			gpio_set_value(aio_data->pdata->gpio_modem_uncon_shutdown, 0);			
		aio_data->modemresetstate = 0;
		aio_data->modemstate = 0;
	}
	else
	{//toggle MODEM on/off state - not a reset command
		aio_data->modemstate = (aio_data->modemstate ^ 1);
	}

	//Set modem on off line back to zero after required delay
	if ( -1 != aio_data->pdata->gpio_modem_on_off)
		gpio_set_value(aio_data->pdata->gpio_modem_on_off, 0); 

	aio_data->modemstateinprocess = 0;//turn on/off process is completed

	if ((-1 != aio_data->pdata->gpio_modem_usb_en ) && (aio_data->modemstate == 1)) { //M307i 846 rev B
		gpio_set_value(aio_data->pdata->gpio_modem_usb_en, aio_data->pdata->gpio_modem_usb_en_pol ? 1 : 0); //Connect USB when MODEM is ON
	}
}


static void do_input1_debounce_work(struct work_struct *work)
{
	struct automotiveio_data *data = aio_data;
	int signalstate, oldstate, ig_event, done;

	mutex_lock(&data->work_lock);
	//printk("%s: [%d]\n", __func__, data->pdata->gpio_in_1);
	signalstate = gpio_get_value(data->pdata->gpio_in_1) ^ data->pdata->gpio_in_active_low_1;

	data->input1_num_read++;

	oldstate = data->inputreport & AUTIO_INPUT_EVENTS_1_MASK;

	if (signalstate == oldstate) {
		data->input1_num_read = 0;
	}

	if ((data->input1_num_read >= 20) || (ktime_to_ms(ktime_get()) > (data->input1startreadtime + 1000)))
	{
		if (signalstate != oldstate){
			data->inputreport = signalstate | (data->inputreport & ~AUTIO_INPUT_EVENTS_1_MASK);

			if (signalstate) {
				ig_event = IGNITION_EVT_NOTIFY_IGN_OFF;
			} else {
				ig_event = IGNITION_EVT_NOTIFY_IGN_ON;
			}
			ignitionevents_notify(ig_event, NULL);
			input_report_abs(data->autmotiveio_input_dev, ABS_MISC, data->inputreport);
			input_sync(data->autmotiveio_input_dev);
			last_in1_sent = signalstate;
		}
		//printk("%s: finished readings total time[%d]ms\n", __func__, ktime_to_ms(ktime_get()) - data->input1startreadtime);
		done = 1;
	}
	else
	{
		done = 0;
	}

	mutex_unlock(&data->work_lock);
	if (done) {
		enable_irq(data->irq_input1);
	} else {
		schedule_delayed_work(&data->input1_debounce_work, msecs_to_jiffies(1));
	}
}

static void do_input2_debounce_work(struct work_struct *work)
{
	struct automotiveio_data *data = aio_data;
	int signalstate, oldstate, done;

	if ( -1 == aio_data->pdata->gpio_in_2) {
		return;
	}

	mutex_lock(&data->work_lock);
	//printk("%s: [%d]\n", __func__, data->pdata->gpio_in_2);
	signalstate = gpio_get_value(data->pdata->gpio_in_2) ^ ACTIVE_LOW;

	data->input2_num_read++;

	oldstate = (data->inputreport & AUTIO_INPUT_EVENTS_2_MASK) >> 1;

	if (signalstate == oldstate) {
		data->input2_num_read = 0;
	}

	if ((data->input2_num_read >= 20) || (ktime_to_ms(ktime_get()) > (data->input2startreadtime + 1000)))
	{
		if (signalstate != oldstate){
			data->inputreport = (signalstate << 1) | (data->inputreport & ~AUTIO_INPUT_EVENTS_2_MASK);

			automotivenevents_notify(signalstate ? AUTOMOTIVE_EVT_NOTIFY_IN2_ON : AUTOMOTIVE_EVT_NOTIFY_IN2_OFF, NULL);
			input_report_abs(data->autmotiveio_input_dev, ABS_MISC, data->inputreport);
			input_sync(data->autmotiveio_input_dev);
		}
		//printk("%s: finished readings total time[%d]ms\n", __func__, ktime_to_ms(ktime_get()) - data->input2startreadtime);
		done = 1;
	}
	else
	{
		done = 0;
	}

	mutex_unlock(&data->work_lock);
	if (done) {
		enable_irq(data->irq_input2);
	} else {
		schedule_delayed_work(&data->input2_debounce_work, msecs_to_jiffies(1));
	}
}

static void do_wall_debounce_work(struct work_struct *work)
{
	struct automotiveio_data *data = aio_data;
	int signalstate, oldstate, done;

	if ( -1 == aio_data->pdata->gpio_wall) {
		return;
	}

	mutex_lock(&data->work_lock);
	signalstate = gpio_get_value(data->pdata->gpio_wall);

	data->wall_num_read++;

	oldstate = (data->inputreport & AUTIO_INPUT_EVENTS_4_MASK) >> 3;

	if (signalstate == oldstate) {
		data->wall_num_read = 0;
	}

	if ((data->wall_num_read > 4) || (ktime_to_ms(ktime_get()) > (data->wallstartreadtime + 200)))
	{
		if (signalstate != oldstate){
			data->inputreport = (signalstate << 3) | (data->inputreport & ~AUTIO_INPUT_EVENTS_4_MASK);

			automotivenevents_notify(signalstate ? AUTOMOTIVE_EVT_NOTIFY_WALL_ON : AUTOMOTIVE_EVT_NOTIFY_WALL_OFF, NULL);
			input_report_abs(data->autmotiveio_input_dev, ABS_MISC, data->inputreport);
			input_sync(data->autmotiveio_input_dev);
		}
		//printk("%s: finished readings total time[%d]ms\n", __func__, ktime_to_ms(ktime_get()) - data->wallstartreadtime);
		done = 1;
	}
	else
	{
		done = 0;
	}

	mutex_unlock(&data->work_lock);
	if (done) {
		enable_irq(data->irq_wall);
	} else {
		schedule_delayed_work(&data->wall_debounce_work, msecs_to_jiffies(1));
	}
}

static void do_cradle_debounce_work(struct work_struct *work)
{
	struct automotiveio_data *data = aio_data;
	int signalstate, oldstate, done;

	if ( -1 == aio_data->pdata->gpio_cradle) {
		return;
	}
	mutex_lock(&data->work_lock);
	signalstate = gpio_get_value(data->pdata->gpio_cradle) ^ ACTIVE_LOW;

	data->cradle_num_read++;

	oldstate = (data->inputreport & AUTIO_INPUT_EVENTS_3_MASK) >> 2;

	if (signalstate == oldstate) {
		data->cradle_num_read = 0;
	}

	if ((data->cradle_num_read >= 4) || (ktime_to_ms(ktime_get()) > (data->cradlestartreadtime + 200)))
	{
		if (signalstate != oldstate){
			data->inputreport = (signalstate << 2) | (data->inputreport & ~AUTIO_INPUT_EVENTS_3_MASK);

			automotivenevents_notify(signalstate ? AUTOMOTIVE_EVT_NOTIFY_CRADLE_ON : AUTOMOTIVE_EVT_NOTIFY_CRADLE_OFF, NULL);
			input_report_abs(data->autmotiveio_input_dev, ABS_MISC, data->inputreport);
			input_sync(data->autmotiveio_input_dev);
		}
		//printk("%s: finished readings total time[%d]ms\n", __func__, ktime_to_ms(ktime_get()) - data->cradlestartreadtime);
		done = 1;
	}
	else
	{
		done = 0;
	}

	mutex_unlock(&data->work_lock);
	if (done) {
		enable_irq(data->irq_cradle);
	} else {
		schedule_delayed_work(&data->cradle_debounce_work, msecs_to_jiffies(1));
	}
}

static int automotiveio_init_m317(struct platform_device *pdev)
{
	int err;
	struct automotiveio_platform_data *pdata = pdev->dev.platform_data;

	err = gpio_request(pdata->gpio_out, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_out, err);
		goto exit_error;
	}
	err = gpio_direction_output(pdata->gpio_out, 0);
	if (err) {
		dev_err(&pdev->dev, "Failed to set gpio [%d] to output: %d\n", pdata->gpio_out, err);
		goto exit_error;
	}

	if ( -1 != pdata->gpio_modem_usb_en ) {
		err = gpio_request(pdata->gpio_modem_usb_en, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_usb_en, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_modem_usb_en, aio_data->pdata->gpio_modem_usb_en_pol ? 0 : 1); //disconnect MODEM USB
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to modem usb enable: %d\n", pdata->gpio_modem_usb_en, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_modem_on_off) {
		err = gpio_request(pdata->gpio_modem_on_off, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_on_off, err);
			goto exit_error;
		}

		err = gpio_direction_output(pdata->gpio_modem_on_off, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to audio switch: %d\n", pdata->gpio_modem_on_off, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_modem_uncon_shutdown ) {
		err = gpio_request(pdata->gpio_modem_uncon_shutdown, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_uncon_shutdown, err);
			goto exit_error;
		}

		err = gpio_direction_output(pdata->gpio_modem_uncon_shutdown, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to unconditional shutdown: %d\n", pdata->gpio_modem_uncon_shutdown, err);
			goto exit_error;
		}
	}

	if (-1 != pdata->gpio_modem_power_ctl) {
		err = gpio_request(pdata->gpio_modem_power_ctl, dev_name(&pdev->dev));
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_power_ctl, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_modem_power_ctl, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to power control MODEM circuit: %d\n", pdata->gpio_modem_power_ctl, err);
			goto exit_error;
		}
	}

	aio_data->modemresetstate = 0;
	aio_data->modemstateinprocess = 0;
	aio_data->modemstate = 0;//3G MODEM is always off when booting
	aio_data->wq = create_singlethread_workqueue("modem3g_on_off");
	if (!aio_data->wq) {
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}

	INIT_WORK(&aio_data->modem_on_off_debounce_work, do_modem_on_off_debounce_work);

	return 0;

err_create_workqueue:
	destroy_workqueue(aio_data->wq);
exit_error:
	return err;
}

static int automotiveio_init_m307i(struct platform_device *pdev)
{
	int err, irq;
	struct automotiveio_platform_data *pdata = pdev->dev.platform_data;

	if ( -1 != pdata->gpio_modem_usb_en ) { //M307i - 846V2 - MODEM USB Enable
		err = gpio_request(pdata->gpio_modem_usb_en, dev_name(&pdev->dev));
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_usb_en, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_modem_usb_en, aio_data->pdata->gpio_modem_usb_en_pol ? 0 : 1); //disconnect MODEM USB
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to modem usb enable: %d\n", pdata->gpio_modem_usb_en, err);
			goto exit_error;
		}
	}
//	else
//	{
//		pdata->gpio_modem_usb_en = -1; //identify 846 revA
//	}

	if ( -1 != pdata->gpio_out_1) {
		err = gpio_request(pdata->gpio_out_1, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_out_1, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_out_1, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to output 1: %d\n", pdata->gpio_out_1, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_out_2 ) {
		err = gpio_request(pdata->gpio_out_2, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_out_2, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_out_2, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to output 1: %d\n", pdata->gpio_out_2, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_audswitch ) {
		err = gpio_request(pdata->gpio_audswitch, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_audswitch, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_audswitch, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to audio switch: %d\n", pdata->gpio_audswitch, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_modem_on_off ) {
		err = gpio_request(pdata->gpio_modem_on_off, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_on_off, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_modem_on_off, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to audio switch: %d\n", pdata->gpio_modem_on_off, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_modem_uncon_shutdown ) {
		err = gpio_request(pdata->gpio_modem_uncon_shutdown, dev_name(&pdev->dev)); 
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_uncon_shutdown, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_modem_uncon_shutdown, 0);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to audio switch: %d\n", pdata->gpio_modem_uncon_shutdown, err);
			goto exit_error;
		}
	}

	if ( -1 != pdata->gpio_modem_power_ctl ) {
		err = gpio_request(pdata->gpio_modem_power_ctl, dev_name(&pdev->dev));	
		if (err) {
			dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_modem_power_ctl, err);
			goto exit_error;
		}
		err = gpio_direction_output(pdata->gpio_modem_power_ctl, 1);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to power control MODEM circuit: %d\n", pdata->gpio_modem_power_ctl, err);
			goto exit_error;
		}
	}
	
	if ( -1 != pdata->gpio_in_2 ) {
		err = gpio_request(pdata->gpio_in_2, "automotive input_2"); 
		if (err) {
			dev_err(&pdev->dev, "can't request automotive input 2 gpio %d, err: %d\n",
				pdata->gpio_in_2, err);
		}

		err = gpio_direction_input(pdata->gpio_in_2);
		if (err) {
			dev_err(&pdev->dev, "Failed to set gpio [%d] to input: %d\n", pdata->gpio_in_2, err);
		}

		irq = gpio_to_irq(pdata->gpio_in_2);
		if (irq > 0) {
			set_irq_type(irq, IRQ_TYPE_EDGE_FALLING | IRQF_TRIGGER_RISING);
			err = request_irq(irq, automotiveio_input_2_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			dev_name(&pdev->dev), aio_data);
			if (err < 0)
				dev_warn(&pdev->dev, "Failed to request input 2 irq: %d\n", err);
			else
			{
				disable_irq_nosync(irq);
				aio_data->irq_input2 = irq;
			}
		}
	}

	if ( -1 != pdata->gpio_wall ) {
		irq = gpio_to_irq(pdata->gpio_wall); 
		if (irq > 0) {
			set_irq_type(irq, IRQ_TYPE_EDGE_FALLING | IRQF_TRIGGER_RISING);
			err = request_irq(irq, automotiveio_wall_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			dev_name(&pdev->dev), aio_data);
			if (err < 0)
				dev_warn(&pdev->dev, "Failed to request wall irq: %d\n", err);
			else
			{
				disable_irq_nosync(irq);
				aio_data->irq_wall = irq;
			}
		}
	}

	if ( -1 != pdata->gpio_cradle ) {
		irq = gpio_to_irq(pdata->gpio_cradle); 
		if (irq > 0) {
			set_irq_type(irq, IRQ_TYPE_EDGE_FALLING | IRQF_TRIGGER_RISING);
			err = request_irq(irq, automotiveio_cradle_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			dev_name(&pdev->dev), aio_data);
			if (err < 0)
				dev_warn(&pdev->dev, "Failed to request cradle irq: %d\n", err);
			else
			{
				aio_data->irq_cradle = irq;
				disable_irq_nosync(irq);
			}
		}
	}

	aio_data->modemresetstate = 0;
	aio_data->modemstateinprocess = 0;
	aio_data->modemstate = 0;//3G MODEM is always off when booting
	aio_data->wq = create_singlethread_workqueue("modem3g_on_off");
	if (!aio_data->wq) {
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}

	INIT_WORK(&aio_data->modem_on_off_debounce_work, do_modem_on_off_debounce_work);

	if ( -1 != pdata->gpio_wall) {
		aio_data->inputreport |= gpio_get_value(aio_data->pdata->gpio_wall) << 3; 
	}

	if ( -1 != pdata->gpio_cradle) {
		aio_data->inputreport |= gpio_get_value(aio_data->pdata->gpio_cradle) << 2; 
	}

	if (-1 != pdata->gpio_in_2) {
		aio_data->inputreport |= gpio_get_value(aio_data->pdata->gpio_in_2) << 1; 
	}

	INIT_DELAYED_WORK(&aio_data->wall_debounce_work, do_wall_debounce_work);
	schedule_delayed_work(&aio_data->wall_debounce_work, 0);

	INIT_DELAYED_WORK(&aio_data->input2_debounce_work, do_input2_debounce_work);
	schedule_delayed_work(&aio_data->input2_debounce_work, 0);

	INIT_DELAYED_WORK(&aio_data->cradle_debounce_work, do_cradle_debounce_work);
	schedule_delayed_work(&aio_data->cradle_debounce_work, 0);

	return 0;

err_create_workqueue:
	destroy_workqueue(aio_data->wq);
exit_error:
	return err;
}

static int automotiveio_init_m307(struct platform_device *pdev)
{
	int err;

	struct automotiveio_platform_data *pdata = pdev->dev.platform_data;

	err = gpio_request(pdata->gpio_out, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_out, err);
		goto exit_error;
	}
	err = gpio_direction_output(pdata->gpio_out, 0);
	if (err) {
		dev_err(&pdev->dev, "Failed to set gpio [%d] to output: %d\n", pdata->gpio_out, err);
		goto exit_error;
	}

	exit_error:
	return err;
}

static int __devinit automotiveio_probe(struct platform_device *pdev)
{
	struct input_dev *input_dev;
	struct automotiveio_data *data;
	struct automotiveio_platform_data *pdata = pdev->dev.platform_data;
	int err, irq;

	data = kzalloc(sizeof(struct automotiveio_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		pr_err("%s: failed to alloc memory for module data\n",
		    __func__);
		goto exit_kfree;
	}

	platform_set_drvdata(pdev, data);

    data->pdata = pdata;
	aio_data = data;

	aio_data->inputreport = 0;

	err = gpio_request(pdata->gpio_in_1, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "Failed to request gpio pin[%d]: %d\n", pdata->gpio_in_1, err);
		goto exit_kfree;
	}
	err = gpio_direction_input(pdata->gpio_in_1);
	if (err) {
		dev_err(&pdev->dev, "Failed to set gpio [%d] to input: %d\n", pdata->gpio_in_1, err);
		goto exit_kfree;
    }

    wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "automotive_io");
    spin_lock_init(&data->irq_lock); 

    irq = gpio_to_irq(pdata->gpio_in_1);  //gpio in 1 is used both in M307 and M307i
    if (irq > 0) {
	    err = request_irq(irq, automotiveio_input_1_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
		dev_name(&pdev->dev), data);
	    if (err < 0)
		    dev_warn(&pdev->dev, "Failed to request irq: %d\n", err);
	    else
		{
		    data->irq_input1 = irq;   
			disable_irq_nosync(irq);
		}
    }

	/* allocate automotive-input input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto exit_kfree;
	}
	input_set_drvdata(input_dev, data);
	input_dev->name = "automotive-input";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	printk("registering automotive-input input device\n");
	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto exit_kfree;
	}
	data->autmotiveio_input_dev = input_dev;
	err = sysfs_create_group(&input_dev->dev.kobj,
	    &automotivein_attr_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_autmotiveio;
	}

	mutex_init(&aio_data->work_lock);

	if (aio_data->pdata->boardversion == 846) {
		err = automotiveio_init_m317(pdev);
	}
	else if (aio_data->pdata->boardversion > 15)
	{
		err = automotiveio_init_m307i(pdev);
	}
	else
	{
		err = automotiveio_init_m307(pdev);
	}
	if (err)  goto  exit_kfree;

	data->debounce_time = 20;//initial time 20 msec

	aio_data->inputreport |= (gpio_get_value(aio_data->pdata->gpio_in_1) ^ aio_data->pdata->gpio_in_active_low_1) & ~AUTIO_INPUT_EVENTS_1_MASK;

	INIT_DELAYED_WORK(&aio_data->input1_debounce_work, do_input1_debounce_work);
	schedule_delayed_work(&aio_data->input1_debounce_work, 0);

	hrtimer_init(&data->timer_modem, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer_modem.function = automotiveio_event_modem_timer_func;

	printk("Autmotive IO support ver. %s enabled\n", DRIVER_VERSION);

	//turn ON MODEM to save rising time
	if ((aio_data->pdata->modemtype == 0) && (aio_data->pdata->boardversion >= 16)) //Internal MODEM exists
    {
		__automotive_modem_on_off_set_state();
	}

	return 0;
	/* error, unwind it all */
	err_sysfs_create_group_autmotiveio:
	input_unregister_device(data->autmotiveio_input_dev);
	exit_kfree:
	kfree(data);

	return err;
}

static int __devexit automotiveio_remove(struct platform_device *pdev)
{
	struct automotiveio_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&data->autmotiveio_input_dev->dev.kobj,
	    &automotivein_attr_group);

	input_unregister_device(data->autmotiveio_input_dev);

	return 0;
}
#if defined CONFIG_PM
static int automotiveio_driver_suspend(struct device *dev)
{
	last_in1_suspend = last_in1_sent;

	return 0;
}

static int automotiveio_driver_resume(struct device *dev)
{
#if defined (CONFIG_MACH_A300)
	struct automotiveio_data *data = dev_get_drvdata(dev);

	if(last_in1_suspend && last_in1_suspend == last_in1_sent)//exit from suspend
	{
		input_report_abs(data->autmotiveio_input_dev, ABS_MISC, 0);
		input_report_abs(data->autmotiveio_input_dev, ABS_MISC, data->inputreport);
		input_sync(data->autmotiveio_input_dev);
		printk("%s: (signalstate %d, data %d) done\n", __func__, last_in1_suspend, data->inputreport);
	}
#endif

	return 0;
}
#else
#define automotiveio_driver_suspend NULL
#define automotiveio_driver_resume NULL
#endif

static const struct dev_pm_ops automotiveio_driver_pm_ops =
{
	.suspend	= automotiveio_driver_suspend,
	.resume		= automotiveio_driver_resume,
};

static struct platform_driver automotiveio_driver = {
	.driver = {
		.name	= A300AUTOMOTIVEIO_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm = &automotiveio_driver_pm_ops,
	},
	.probe	= automotiveio_probe,
	.remove	= __devexit_p(automotiveio_remove),
};

static int __init automotiveio_init(void)
{
	return platform_driver_register(&automotiveio_driver);
}

static void __exit automotiveio_exit(void)
{
	platform_driver_unregister(&automotiveio_driver);
}

MODULE_AUTHOR("Ran Meyerstein <ranm@micronet.co.il>");
MODULE_DESCRIPTION("A300 M307 M307i Automotive Input Output driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(automotiveio_init);
module_exit(automotiveio_exit);

