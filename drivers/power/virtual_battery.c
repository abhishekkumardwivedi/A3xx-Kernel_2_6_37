/*
 * Virtual battery driver
 *
 * Copyright (c) 2013, Micronet Ltd.
 *
 * virtual_battery.c
 *
 *  Created on: May 29, 2013
 *      Author: Vladimir Zatulovsky
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/ignition.h>
#include <linux/notifier.h>

enum {
	REG_MANUFACTURER_DATA,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_CAPACITY,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CYCLE_COUNT,
	REG_SERIAL_NUMBER,
	REG_REMAINING_CAPACITY,
	REG_FULL_CHARGE_CAPACITY,
	REG_DESIGN_CAPACITY,
	REG_DESIGN_VOLTAGE,
};

#define VB_DATA(_psp, _min_value, _max_value) { \
	.psp = _psp, .min_value = _min_value, .max_value = _max_value, \
}

static enum power_supply_property virtual_power_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct virtual_battery_device_data
{
	enum power_supply_property psp;
	int min_value;
	int max_value;
}virtual_battery_data[] = {
	[REG_MANUFACTURER_DATA] = VB_DATA(POWER_SUPPLY_PROP_PRESENT, 	 0, 65535),
	[REG_TEMPERATURE] 		= VB_DATA(POWER_SUPPLY_PROP_TEMP, 	 	 0, 65535),
	[REG_VOLTAGE] 			= VB_DATA(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0, 20000),
	[REG_CURRENT] 			= VB_DATA(POWER_SUPPLY_PROP_CURRENT_NOW, -32768, 32767),
	[REG_CAPACITY] 			= VB_DATA(POWER_SUPPLY_PROP_CAPACITY, 	 0, 100),
	[REG_REMAINING_CAPACITY]   = VB_DATA(POWER_SUPPLY_PROP_ENERGY_NOW, 0, 65535),
	[REG_FULL_CHARGE_CAPACITY] = VB_DATA(POWER_SUPPLY_PROP_ENERGY_FULL, 0, 65535),
	[REG_TIME_TO_EMPTY] 	= VB_DATA(POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, 0, 65535),
	[REG_TIME_TO_FULL] 		= VB_DATA(POWER_SUPPLY_PROP_TIME_TO_FULL_AVG, 0, 65535),
	[REG_STATUS] 			= VB_DATA(POWER_SUPPLY_PROP_STATUS, 	 0, 65535),
	[REG_CYCLE_COUNT] 		= VB_DATA(POWER_SUPPLY_PROP_CYCLE_COUNT, 0, 65535),
	[REG_DESIGN_CAPACITY] 	= VB_DATA(POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN, 0, 65535),
	[REG_DESIGN_VOLTAGE] 	= VB_DATA(POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, 0, 65535),
	[REG_SERIAL_NUMBER] 	= VB_DATA(POWER_SUPPLY_PROP_SERIAL_NUMBER, 0, 0),
};

static enum power_supply_property virtual_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
};

struct power_supply_info_i {
	struct device 		*dev;
	struct power_supply	power_supply_ac;
	struct power_supply	power_supply_bat;
	struct mutex 		work_lock;
	int ig_in;
	int bat_status;
	int ac_status;
};
struct power_supply_info_i *ps;

static int virtual_power_ac_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	switch(psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
			ps->ac_status = ignition_get_state();
			if(ps->ac_status == 1) // low
				val->intval = 1;
			else // high
				val->intval = 0;
			//val->intval = (gpio_get_value(ps->ig_in) == 0);
			//printk("%s: POWER_SUPPLY_PROP_ONLINE <- %d\n", __func__, val->intval);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static char virtual_battery_serial[5];

static int virtual_battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	switch(psp)
	{
		case POWER_SUPPLY_PROP_PRESENT:
			// virtual battery is present always
			//printk("%s: POWER_SUPPLY_PROP_PRESENT <- 1\n", __func__);
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			// virtual battery is good always
			//printk("%s: POWER_SUPPLY_PROP_PRESENT <- POWER_SUPPLY_HEALTH_GOOD\n", __func__);
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			//printk("%s: POWER_SUPPLY_PROP_TECHNOLOGY <- POWER_SUPPLY_TECHNOLOGY_LION\n", __func__);
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_ENERGY_NOW:
			val->intval = 100; //virtual_battery_data[REG_REMAINING_CAPACITY].max_value;
			//printk("%s: POWER_SUPPLY_PROP_ENERGY_NOW <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_ENERGY_FULL:
			val->intval = virtual_battery_data[REG_FULL_CHARGE_CAPACITY].max_value;
			//printk("%s: POWER_SUPPLY_PROP_ENERGY_FULL <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
			val->intval = virtual_battery_data[REG_DESIGN_CAPACITY].max_value;
			//printk("%s: POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = virtual_battery_data[REG_CAPACITY].max_value;
			//printk("%s: POWER_SUPPLY_PROP_CAPACITY <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			sprintf(virtual_battery_serial, "%04x", virtual_battery_data[REG_SERIAL_NUMBER].max_value);
			val->strval = virtual_battery_serial;
			//printk("%s: POWER_SUPPLY_PROP_SERIAL_NUMBER <- %s\n", __func__, virtual_battery_serial);
			break;
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = ps->bat_status;
			//printk("%s: POWER_SUPPLY_PROP_STATUS <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			val->intval = 1000;
			//printk("%s: POWER_SUPPLY_PROP_CYCLE_COUNT <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = 3500;
			//printk("%s: POWER_SUPPLY_PROP_VOLTAGE_NOW <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = 250;
			//printk("%s: POWER_SUPPLY_PROP_CURRENT_NOW <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = 30;
			//printk("%s: POWER_SUPPLY_PROP_TEMP <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			val->intval = virtual_battery_data[REG_TIME_TO_EMPTY].max_value;
			//printk("%s: POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
			val->intval = virtual_battery_data[REG_TIME_TO_FULL].min_value;
			//printk("%s: POWER_SUPPLY_PROP_TIME_TO_FULL_AVG <- %d\n", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = virtual_battery_data[REG_DESIGN_VOLTAGE].max_value;
			//printk("%s: POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN <- %d\n", __func__, val->intval);
			break;
		default:
			printk("%s: INVALID property\n", __func__);
			return -EINVAL;
	}

	return 0;
}


static void virtual_battery_power_source_changed(struct power_supply *psy)
{
	int status;

	if(psy == &ps->power_supply_bat)
	{
		mutex_lock(&ps->work_lock);
		if(power_supply_am_i_supplied(psy))
		{
			printk("%s: completely charged\n", __func__);
			status = POWER_SUPPLY_STATUS_FULL;
		}
		else
		{
			printk("%s: discharging\n", __func__);
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		mutex_unlock(&ps->work_lock);
		if(status != ps->bat_status)
		{
			ps->bat_status = status;
			power_supply_changed(psy);
		}
	}
}

static char *virtual_power_ac_supplied_to[] = {
	"virtual-battery",
};


static int virtual_ps_update(struct notifier_block *nb, unsigned long reason, void *dev)
{
    switch(reason)
    {
    	case IGNITION_EVT_NOTIFY_IGN_ON:
    		//Whatever it needs
    		ps->ac_status = 1;
    		break;
    	case IGNITION_EVT_NOTIFY_IGN_OFF:
    	default:
    		//Whatever it needs
    		ps->ac_status = 0;
            break;
    }

	power_supply_changed(&ps->power_supply_ac);

    return NOTIFY_OK;
}

static struct notifier_block virtual_ps_change_notifier = {
	.notifier_call = virtual_ps_update,
};


static __devinit int virtual_battery_probe(struct platform_device *pdev)
{
	struct power_supply_info_i *pps;
	struct ignition_platform_data *pdata = pdev->dev.platform_data;

	int rc;

	// TODO: register ignition notifications
	// the callback should return current state of ignition
	// 	int (*ignition_level_change)(int *level);
	// 	int (*register_ignition_notification)(ignition_level_change func, int id, int free);
	// Temporary for simulation will use scheduled task
	pps = kzalloc(sizeof(struct power_supply_info_i), GFP_KERNEL);
	if(!pps)
	{
		printk("%s: Add some memory\n", __func__);
		return -ENOMEM;
	}

	pps->dev = &pdev->dev;

	pps->power_supply_ac.name = "virtual-power-ac";
	pps->power_supply_ac.type = POWER_SUPPLY_TYPE_MAINS;
	pps->power_supply_ac.supplied_to = virtual_power_ac_supplied_to;
	pps->power_supply_ac.num_supplicants = ARRAY_SIZE(virtual_power_ac_supplied_to),
	pps->power_supply_ac.properties = virtual_power_ac_properties;
	pps->power_supply_ac.num_properties = ARRAY_SIZE(virtual_power_ac_properties);
	pps->power_supply_ac.get_property = virtual_power_ac_get_property;

	pps->power_supply_bat.name = "virtual-battery";
	pps->power_supply_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	pps->power_supply_bat.properties = virtual_battery_properties;
	pps->power_supply_bat.num_properties = ARRAY_SIZE(virtual_battery_properties);
	pps->power_supply_bat.get_property = virtual_battery_get_property;
	pps->power_supply_bat.external_power_changed = virtual_battery_power_source_changed;
	ps = pps;
	ps->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	ps->ac_status = 1;
	mutex_init(&ps->work_lock);
	ps->ig_in = pdata->ig_in;
	//gpio_request(ps->ig_in, "ignition-state");
	//gpio_direction_input(ps->ig_in);

	platform_set_drvdata(pdev, ps);
	rc = power_supply_register(ps->dev, &ps->power_supply_ac);
	if(rc)
	{
		printk("%s: Failed to register power supply %s\n", __func__, ps->power_supply_ac.name);
		kfree(ps);
		return rc;
	}

	rc = power_supply_register(ps->dev, &ps->power_supply_bat);
	if(rc)
	{
		printk("%s: Failed to register power supply %s\n", __func__, ps->power_supply_bat.name);
		power_supply_unregister(&ps->power_supply_ac);
		kfree(ps);
		return rc;
	}

	ignitionevents_register_notifier(&virtual_ps_change_notifier);

	printk("%s: battery gas gauge device registered (ig_in %d)\n", __func__, ps->ig_in);

	return 0;
}

static __devexit
int virtual_battery_remove(struct platform_device *pdev)
{
	struct power_supply_info_i *pps = platform_get_drvdata(pdev);

	//gpio_free(ps->ig_in);
	mutex_destroy(pps->work_lock);
	power_supply_unregister(&pps->power_supply_bat);
	power_supply_unregister(&pps->power_supply_ac);
	kfree(ps);
	ps = pps = 0;

	return 0;
}

static struct platform_driver virtual_battery_driver = {
	.probe		= virtual_battery_probe,
	.remove		= __devexit_p(virtual_battery_remove),
	.driver = {
		.name	= "virtual-battery",
		.owner = THIS_MODULE,
	},
};

static int __init
virtual_battery_init(void)
{
	printk("%s: register battery\n", __func__);
	return platform_driver_register(&virtual_battery_driver);
}
module_init(virtual_battery_init);

static void __exit
virtual_battery_destroy(void)
{
	printk("%s: Destroy power supply\n", __func__);
	platform_driver_unregister(&virtual_battery_driver);
}
module_exit(virtual_battery_destroy);


MODULE_DESCRIPTION("Virtual battery monitor driver");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimirz@micronet.co.il>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:virtual-battery");
