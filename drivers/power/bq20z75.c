/*
 * Gas Gauge driver for TI's BQ20Z75
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * (C) Copyright 2012 - 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 *
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/bq24172.h>
#include <linux/delay.h>

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

	REG_MODEL_NAME,
	REG_MANUFACTURER,
};

#define BATTERY_POLL_TIME 8000
#define BATTERY_CAP_DIFF  1
#define BATTERY_TEMP_DIFF  1

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_DISCHARGING		0x40
#define BATTERY_FULL_CHARGED		0x20
#define BATTERY_FULL_DISCHARGED		0x10

#define BQ20Z75_DATA(_psp, _addr, _min_value, _max_value) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

static const struct bq20z75_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq20z75_data[] = {
	[REG_MANUFACTURER_DATA] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_PRESENT, 0x00, 0, 65535),
	[REG_TEMPERATURE] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0x09, 0, 20000),
	[REG_CURRENT] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_CURRENT_NOW, 0x0A, -32768,
			32767),
	[REG_CAPACITY] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_CAPACITY, 0x0E, 0, 100),
	[REG_REMAINING_CAPACITY] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_ENERGY_NOW, 0x0F, 0, 65535),
	[REG_FULL_CHARGE_CAPACITY] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_ENERGY_FULL, 0x10, 0, 65535),
	[REG_TIME_TO_EMPTY] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, 0x12, 0,
			65535),
	[REG_TIME_TO_FULL] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_TIME_TO_FULL_AVG, 0x13, 0,
			65535),
	[REG_STATUS] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_STATUS, 0x16, 0, 65535),
	[REG_CYCLE_COUNT] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_CYCLE_COUNT, 0x17, 0, 65535),
	[REG_DESIGN_CAPACITY] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN, 0x18, 0,
			65535),
	[REG_DESIGN_VOLTAGE] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, 0x19, 0,
			65535),
	[REG_SERIAL_NUMBER] =
		BQ20Z75_DATA(POWER_SUPPLY_PROP_SERIAL_NUMBER, 0x1C, 0, 65535),

// Vladimir
// TODO: add next information
//	[REG_MODEL_NAME] =
//		BQ20Z75_DATA(POWER_SUPPLY_PROP_MODEL_NAME, 0x21, 0, 65535),
//	[REG_MANUFACTURER] =
//		BQ20Z75_DATA(POWER_SUPPLY_PROP_MANUFACTURER, 0x20, 0, 65535),
};

static enum power_supply_property bq20z75_properties[] = {
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

//	POWER_SUPPLY_PROP_MODEL_NAME,
//	POWER_SUPPLY_PROP_MANUFACTURER,
//	POWER_SUPPLY_PROP_CAPACITY_LEVEL: POWER_SUPPLY_CAPACITY_LEVEL_LOW/POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
};

struct bq20z75_info {
	struct i2c_client	*client;
	struct power_supply	power_supply;
	struct delayed_work change_status_work;
	int supplier_status;
	int prev_bat_status;
	int prev_charge_status;
	int prev_temp_status;
	int bat_errors;
};

static int bq20z75_read_word_data(struct i2c_client *client, u8 address)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, address);
	if (ret < 0) {
//		printk("%s: i2c read at address 0x%x failed\n", __func__, address);
		return ret;
	}
	return le16_to_cpu(ret);
}

static int bq20z75_write_word_data(struct i2c_client *client, u8 address,
	u16 value)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, address, le16_to_cpu(value));
	if (ret < 0) {
//		printk("%s: i2c write to address 0x%x failed\n", __func__, address);
		return ret;
	}
	return 0;
}

static int bq20z75_get_battery_presence_and_health(
	struct i2c_client *client, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = bq20z75_write_word_data(client,
		bq20z75_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_STATUS);
	if (ret < 0)
		return ret;


	ret = bq20z75_read_word_data(client,
		bq20z75_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0)
		return ret;

	if (ret < bq20z75_data[REG_MANUFACTURER_DATA].min_value ||
	    ret > bq20z75_data[REG_MANUFACTURER_DATA].max_value) {
		val->intval = 0;
		return 0;
	}

	/* Mask the upper nibble of 2nd byte and
	 * lower byte of response then
	 * shift the result by 8 to get status*/
	ret &= 0x0F00;
	ret >>= 8;
	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		if (ret == 0x0F)
			/* battery removed */
			val->intval = 0;
		else
			val->intval = 1;
	} else if (psp == POWER_SUPPLY_PROP_HEALTH) {
		if (ret == 0x09)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (ret == 0x0B)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (ret == 0x0C)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}

	return 0;
}

static int bq20z75_get_battery_property(struct i2c_client *client,
	int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

	ret = bq20z75_read_word_data(client, bq20z75_data[reg_offset].addr);
	if (ret < 0)
		return ret;

	/* returned values are 16 bit */
	if (bq20z75_data[reg_offset].min_value < 0)
		ret = (s16)ret;

	if (ret >= bq20z75_data[reg_offset].min_value &&
	    ret <= bq20z75_data[reg_offset].max_value) {
		val->intval = ret;
		if (psp == POWER_SUPPLY_PROP_STATUS) {
//			printk("%s: POWER_SUPPLY_PROP_STATUS %d\n", __func__, ret);

			if (ret & BATTERY_FULL_CHARGED)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if (ret & BATTERY_FULL_DISCHARGED)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else if (ret & BATTERY_DISCHARGING)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
	} else {
		if (psp == POWER_SUPPLY_PROP_STATUS)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else
			val->intval = 0;
	}

	return 0;
}

static void  bq20z75_unit_adjustment(struct i2c_client *client,
	enum power_supply_property psp, union power_supply_propval *val)
{
#define BASE_UNIT_CONVERSION		1000
#define BATTERY_MODE_CAP_MULT_WATT	(10 * BASE_UNIT_CONVERSION)
#define TIME_UNIT_CONVERSION		600
#define TEMP_KELVIN_TO_CELCIUS		2731
	switch (psp) {
	case POWER_SUPPLY_PROP_ENERGY_NOW:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		//val->intval *= BATTERY_MODE_CAP_MULT_WATT;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//val->intval *= BASE_UNIT_CONVERSION;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* bq20z75 provides battery temperature in 0.1°K
		 * so convert it to 0.1°C */
		// °C == °K - 273.15
		val->intval -= TEMP_KELVIN_TO_CELCIUS;
		//val->intval += 5; // half divider
		//val->intval /= 10;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		//val->intval *= TIME_UNIT_CONVERSION;
		break;

	default:
		dev_dbg(&client->dev,
			"%s: no need for unit conversion %d\n", __func__, psp);
	}
}

static int bq20z75_get_battery_capacity(struct i2c_client *client,
	int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

	if (psp == POWER_SUPPLY_PROP_CAPACITY) {
		s32 ef;
		/* bq20z75 spec says that this can be >100 % even if max value is 100 % */
		ef = bq20z75_read_word_data(client, bq20z75_data[REG_FULL_CHARGE_CAPACITY].addr);
		if (ef < 0)
			return ef;

		ret = bq20z75_read_word_data(client, bq20z75_data[REG_REMAINING_CAPACITY].addr);
		if (ret < 0)
			return ret;

		ret *= 100;
		ret /= ef;
		val->intval = min(ret, 100);
	}
	else
	{
		ret = bq20z75_read_word_data(client, bq20z75_data[reg_offset].addr);
		if (ret < 0)
			return ret;
		val->intval = ret;
	}
	return 0;
}

static char bq20z75_serial[5];
static int bq20z75_get_battery_serial_number(struct i2c_client *client,
	union power_supply_propval *val)
{
	int ret;

	ret = bq20z75_read_word_data(client,
		bq20z75_data[REG_SERIAL_NUMBER].addr);
	if (ret < 0)
		return ret;

	ret = sprintf(bq20z75_serial, "%04x", ret);
	val->strval = bq20z75_serial;

	return 0;
}

static int bq20z75_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int count;
	int ret;
	struct bq20z75_info *bq20z75_device;
	struct i2c_client *client;

	bq20z75_device = container_of(psy, struct bq20z75_info, power_supply);
	client = bq20z75_device->client;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq20z75_get_battery_presence_and_health(client, psp, val);
		//printk("%s: POWER_SUPPLY_PROP_HEALTH %d\n", __func__, ret);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_ENERGY_NOW:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CAPACITY:
		for (count = 0; count < ARRAY_SIZE(bq20z75_data); count++) {
			if (psp == bq20z75_data[count].psp)
				break;
		}

		ret = bq20z75_get_battery_capacity(client, count, psp, val);
		//printk("%s: POWER_SUPPLY_PROP_CAPACITY %d %d <-- %d\n", __func__, ret, bq20z75_data[count].addr, val->intval);
		if (ret)
			return ret;

		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = bq20z75_get_battery_serial_number(client, val);
		//printk("%s: POWER_SUPPLY_PROP_SERIAL_NUMBER %d %s\n", __func__, ret, val->strval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		for (count = 0; count < ARRAY_SIZE(bq20z75_data); count++) {
			if (psp == bq20z75_data[count].psp)
				break;
		}

		ret = bq20z75_get_battery_property(client, count, psp, val);
		//printk("%s: POWER_SUPPLY_PROP_STATUS %d %d <-- %d\n", __func__, ret, bq20z75_data[count].addr, val->intval);
		if (ret)
			return ret;
		if(POWER_SUPPLY_PROP_STATUS == psp)
		{
			if(power_supply_am_i_supplied(psy))
			{
				if(POWER_SUPPLY_STATUS_DISCHARGING == val->intval)
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			else
			{
				if(POWER_SUPPLY_STATUS_CHARGING == val->intval)
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		}
		break;

	default:
		printk("%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	/* Convert units to match requirements for power supply class */
	bq20z75_unit_adjustment(client, psp, val);

	//printk("%s: %d(%d)\n", __func__, psp, val->intval);

	return 0;
}

static char *power_supplier_name[] = { "bq24172-charger" };

static void bq20z75_power_source_changed(struct power_supply *psy)
{
	int supplier_status;
	struct bq20z75_info *bq20z75_device;
	struct power_supply *pss;
	struct power_supply_info_i *pps;
	char status[128] = {0};

	bq20z75_device = container_of(psy, struct bq20z75_info, power_supply);
	pss = power_supply_get_by_name(power_supplier_name[0]);
	pps = container_of(pss, struct power_supply_info_i, power_supply_ac);

	supplier_status = pps->ac_status;

	sprintf(status, "%s", (bq20z75_device->prev_charge_status == POWER_SUPPLY_STATUS_FULL)?"full":
						  (bq20z75_device->prev_charge_status == POWER_SUPPLY_STATUS_CHARGING)?"charging":
						  (bq20z75_device->prev_charge_status == POWER_SUPPLY_STATUS_DISCHARGING)?"discharging":
						  (bq20z75_device->prev_charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING)?"not charging":
						  "unknown");

	if(power_supply_am_i_supplied(psy))
	{
		printk("%s: %s is supplied by %s (%s, %d)\n", __func__, psy->name, pss->name, status, bq20z75_device->prev_bat_status);
	}
	else
	{
		printk("%s: %s not supplied by any supplier  (%s, %d)\n", __func__, psy->name, status, bq20z75_device->prev_bat_status);
		//supplier_status = BQ24172_GOOD;
	}

	if(supplier_status != bq20z75_device->supplier_status)
	{
//		printk("%s: %s supplier status(%x)\n", __func__, psy->name, bq20z75_device->supplier_status);
		bq20z75_device->supplier_status = supplier_status;
		power_supply_changed(psy);
	}
}
static void do_change_status_work(struct work_struct *work)
{
	struct bq20z75_info *bq20z75_device;
	union power_supply_propval val;
	int err, update;

	bq20z75_device = container_of(work, struct bq20z75_info, change_status_work.work);

	if(bq20z75_device->bat_errors > 4)
		return;

	update = 0;
	do
	{
		err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_PRESENT, &val);
		if(err || !val.intval)
		{
			bq20z75_device->bat_errors++;
			break;
		}
		bq20z75_device->bat_errors = 0;
		msleep(100);
		err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_CAPACITY, &val);
		if(err)
		{
			msleep(100);
			err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_CAPACITY, &val);
		}
		if(err)
			break;

		err = (val.intval > bq20z75_device->prev_bat_status)?
				(val.intval - bq20z75_device->prev_bat_status):(bq20z75_device->prev_bat_status - val.intval);
		if(err > BATTERY_CAP_DIFF)
		{
			bq20z75_device->prev_bat_status = val.intval;
			update = 1;
		}
		err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_TEMP, &val);
		if(err)
			err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_TEMP, &val);
		if(err)
			break;

		err = (val.intval > bq20z75_device->prev_temp_status)?
				(val.intval - bq20z75_device->prev_temp_status):(bq20z75_device->prev_temp_status - val.intval);
		err /= 10;
		if(err > BATTERY_TEMP_DIFF)
		{
			bq20z75_device->prev_temp_status = val.intval;
			update = 1;
		}

		err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_STATUS, &val);
		if(err)
			err = bq20z75_get_property(&bq20z75_device->power_supply, POWER_SUPPLY_PROP_STATUS, &val);
		if(err)
			break;
		if(val.intval != bq20z75_device->prev_charge_status)
		{
			bq20z75_device->prev_charge_status = val.intval;
			update = 1;
		}
	}while(0);

	if(update)
		power_supply_changed(&bq20z75_device->power_supply);
	schedule_delayed_work(&bq20z75_device->change_status_work, msecs_to_jiffies(BATTERY_POLL_TIME));
}

static int bq20z75_probe(struct i2c_client *client,	const struct i2c_device_id *id)
{
	struct bq20z75_info *bq20z75_device;
	int rc;

	bq20z75_device = kzalloc(sizeof(struct bq20z75_info), GFP_KERNEL);
	if (!bq20z75_device)
		return -ENOMEM;

	bq20z75_device->client = client;
	bq20z75_device->power_supply.name = "bq20z75-battery";
	bq20z75_device->power_supply.type = POWER_SUPPLY_TYPE_BATTERY;
	bq20z75_device->power_supply.properties = bq20z75_properties;
	bq20z75_device->power_supply.num_properties = ARRAY_SIZE(bq20z75_properties);
	bq20z75_device->power_supply.get_property = bq20z75_get_property;
	bq20z75_device->power_supply.external_power_changed = bq20z75_power_source_changed;
	bq20z75_device->supplier_status = BQ24172_NOT_CHARGING | BQ24172_GOOD;

	i2c_set_clientdata(client, bq20z75_device);

	rc = power_supply_register(&client->dev, &bq20z75_device->power_supply);
	if (rc) {
		printk("%s: Failed to register power supply\n", __func__);
		kfree(bq20z75_device);
		return rc;
	}

	INIT_DELAYED_WORK(&bq20z75_device->change_status_work, do_change_status_work);
	schedule_delayed_work(&bq20z75_device->change_status_work, msecs_to_jiffies(BATTERY_POLL_TIME));
	printk("%s: %s battery gas gauge device registered\n", __func__, client->name);

	return 0;
}

static int bq20z75_remove(struct i2c_client *client)
{
	struct bq20z75_info *bq20z75_device = i2c_get_clientdata(client);

	power_supply_unregister(&bq20z75_device->power_supply);
	kfree(bq20z75_device);
	bq20z75_device = NULL;

	return 0;
}

#if defined CONFIG_PM
static int bq20z75_suspend(struct i2c_client *client,
	pm_message_t state)
{
	s32 ret;

	struct bq20z75_info *bq20z75_device = i2c_get_clientdata(client);

	if(bq20z75_device->bat_errors > 4)
		return 0;

	/* write to manufacturer access with sleep command */
	ret = bq20z75_write_word_data(client,
		bq20z75_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_SLEEP);
	if (ret < 0)
		return ret;

	return 0;
}
#else
#define bq20z75_suspend		NULL
#endif
/* any smbus transaction will wake up bq20z75 */
#define bq20z75_resume		NULL

static const struct i2c_device_id bq20z75_id[] = {
	{ "bq20z75", 0 },
	{}
};

static struct i2c_driver bq20z75_battery_driver = {
	.probe		= bq20z75_probe,
	.remove		= bq20z75_remove,
	.suspend	= bq20z75_suspend,
	.resume		= bq20z75_resume,
	.id_table	= bq20z75_id,
	.driver = {
		.name	= "bq20z75-battery",
	},
};

static int __init bq20z75_battery_init(void)
{
	//printk("%s: \n", __func__);
	return i2c_add_driver(&bq20z75_battery_driver);
}
module_init(bq20z75_battery_init);

static void __exit bq20z75_battery_exit(void)
{
	i2c_del_driver(&bq20z75_battery_driver);
}
module_exit(bq20z75_battery_exit);

MODULE_DESCRIPTION("BQ20z75 battery monitor driver");
MODULE_LICENSE("GPL");