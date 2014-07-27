/*
 *  ignition.c - Linux kernel modules for ignition control
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/ignition.h>
#include <linux/automotiveio.h>

#define IGNITION_DRV_NAME	"Ignition"
#define DRIVER_VERSION		"1.0"


/* Notification for ignition events */
static RAW_NOTIFIER_HEAD(ignition_chain);

/* Protection for the above */
static DEFINE_RAW_SPINLOCK(ignitionevents_lock);

/*
 * Defines
 */

#define ignition_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
//#define ignition_dbgmsg(str, args...) printk("%s: " str, __func__, ##args)
/*
 * Structs
 */

struct ignition_data {
	struct input_dev *ignition_input_dev;
	struct ignition_platform_data *pdata;
	bool   enabled;
};

/*
 * Global data
 */

struct ignition_data * ign_data;


/**
 * ignitionevents_register_notifier - register an igntion events
 * change listener 
 */
int ignitionevents_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&ignitionevents_lock, flags);
	ret = raw_notifier_chain_register(&ignition_chain, nb);
	raw_spin_unlock_irqrestore(&ignitionevents_lock, flags);

	return ret;
}

/**
 * ignitionevents_notify - notification about relevant events
 */
void ignitionevents_notify(unsigned long reason, void *arg)
{
	unsigned long flags;
	if (ign_data != NULL) {
		if (ign_data->enabled) {
			raw_spin_lock_irqsave(&ignitionevents_lock, flags);
			raw_notifier_call_chain(&ignition_chain, reason, NULL);
			raw_spin_unlock_irqrestore(&ignitionevents_lock, flags);
		}
	}
}
EXPORT_SYMBOL_GPL(ignitionevents_notify);

int ignition_get_state(void)
{
//	printk("%s %x %x <--- %d\n", __func__, ign_data, (ign_data)?ign_data->pdata:0, automotiveio_get_input_state());
	return (ign_data && ign_data->pdata && (ign_data->pdata->active_level == automotiveio_get_input_state()));
}
EXPORT_SYMBOL_GPL(ignition_get_state);

int ignition_show_state(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%d", ignition_get_state());
}
static struct kernel_param_ops ignition_ops_str = {
	.set = NULL,
	.get = ignition_show_state,
};

module_param_cb(ignition, &ignition_ops_str, NULL, 0444);

/*
 * SysFS support
 */

static ssize_t show_ignition_state(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%u", ign_data->enabled);
}

static ssize_t store_ignition_state(const char *buf, const struct kernel_param *kp)
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

	if (new_value)
	{
		//notify listeners state of ignition when set to enable
		ign_data->enabled = true;
		ignitionevents_notify(ignition_get_state(), NULL);
	}
	else
		ign_data->enabled = false;

	return 0;
}

static struct kernel_param_ops ignition_attr_ops_str = {
	.set = store_ignition_state,
	.get = show_ignition_state,
};

module_param_cb(enable, &ignition_attr_ops_str, NULL, 0664);

static int __devinit ignition_probe(struct platform_device *pdev)
{
	struct ignition_data *data;
	struct ignition_platform_data *pdata = pdev->dev.platform_data;
	int err;

	printk("%s: %s pdata %p\n", __func__, pdev->name, pdata);

	data = kzalloc(sizeof(struct ignition_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		pr_err("%s: failed to alloc memory for module data\n",
		    __func__);
		goto exit_kfree;
	}

	platform_set_drvdata(pdev, data);

	//By default ignition is enabled
	data->enabled = true;

    data->pdata = pdata;
	ign_data = data;

	return 0;
	/* error, unwind it all */
	exit_kfree:
	kfree(data);

	return err;
}

static int __devexit ignition_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver ignition_driver = {
	.driver = {
		.name	= IGNITION_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= ignition_probe,
	.remove	= __devexit_p(ignition_remove),
};

static int __init ignition_init(void)
{
	return platform_driver_register(&ignition_driver);
}

static void __exit ignition_exit(void)
{
	platform_driver_unregister(&ignition_driver);
}

MODULE_AUTHOR("Ran Meyerstein <ranm@micronet.co.il>");
MODULE_DESCRIPTION("A300 Ignition driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(ignition_init);
module_exit(ignition_exit);


