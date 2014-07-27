/*
 * bq24172.c
 *
 *  Created on: Jul 11, 2013
 *      Author: Vladimir Zatulovsky
 *
 * Copyright (c) 2013, Micronet Ltd.
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
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bq24172.h>
#include <linux/ignition.h>
#include <linux/automotiveio.h>
#include <linux/notifier.h>
#include <linux/delay.h>

#define BQ24172_BLINK_PERIOD	1000 // .5 Hz
#define BQ24172_TOTAL_FAULTS	8

struct power_supply_info_i *ps_info;

static char *bq24172_charger_supplied_to[] = {
	"bq20z75-battery",
};

static enum power_supply_property bq24172_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
};
static void do_charge_status_work(struct work_struct *work)
{
	struct power_supply_info_i *pps;
//	struct irq_desc *desc_stat;
	signed long long now;
	int charge_status;

	pps = container_of(work, struct power_supply_info_i, charge_status_work.work);
//	desc_stat = irq_to_desc(pps->charge_status_irq);

	if(-1 == pps->charge_status_in)
		return;

	now = ktime_to_ms(ktime_get());
	charge_status = gpio_get_value_cansleep(pps->charge_status_in);
//	if(pps->prev_charge_status != charge_status)
//	{
//		msleep(10);
//		charge_status = gpio_get_value_cansleep(pps->charge_status_in);
//	}
	// check for blinking two and half periods
	if(pps->status_count < 5)
	{
//		printk("%s: check status %d(%d, %d) %lld\n", __func__, charge_status, pps->prev_charge_status, pps->status_count, now);

		if(pps->prev_charge_status != charge_status)
		{
			mutex_lock(&pps->work_lock);
			pps->prev_status_time = now;
			pps->prev_charge_status = charge_status;
			mutex_unlock(&pps->work_lock);
			if(pps->status_count > 1)
			{
				struct power_supply *pss;
				union power_supply_propval val;
				int supplicant_present = 0, err;

				pss = power_supply_get_by_name(bq24172_charger_supplied_to[0]);
				if(pss)
				{
					err = pss->get_property(pss, POWER_SUPPLY_PROP_PRESENT, &val);
					if(!err)
						supplicant_present = val.intval;
				}
				mutex_lock(&pps->work_lock);
				pps->status_count = 0;

				if(!supplicant_present)
					pps->total_faults++;

				pps->ac_status &= ~BQ24172_STATUS_MASK;
				//pps->ac_status &= ~BQ24172_ERROR_MASK;
				pps->ac_status |= (BQ24172_NOT_CHARGING | BQ24172_BAT_ABS | BQ24172_UNSPEC_FAILURE);
				// temporary for debugging purposes only
				pps->ac_status |= BQ24172_ONLINE;
				mutex_unlock(&pps->work_lock);
				printk("%s: notify failure(%d) supplier %s of %s\n", __func__, pps->total_faults,
						pps->power_supply_ac.name, (pps->power_supply_ac.supplied_to[0])?pps->power_supply_ac.supplied_to[0]:"unknown");
				power_supply_changed(&pps->power_supply_ac);
				if(pps->total_faults < BQ24172_TOTAL_FAULTS)
					enable_irq(pps->charge_status_irq);
				return;
			}
		}
		mutex_lock(&pps->work_lock);
		pps->status_count++;
		mutex_unlock(&pps->work_lock);
		schedule_delayed_work(&pps->charge_status_work, msecs_to_jiffies(BQ24172_BLINK_PERIOD));
		return;
	}

//	printk("%s: %scharging %lld\n", __func__, (charge_status)?"not ":" ", ktime_to_ms(ktime_get()));

	charge_status = (charge_status == pps->charge_status_lvl)?BQ24172_CHARGING:BQ24172_NOT_CHARGING;

	mutex_lock(&pps->work_lock);
	pps->ac_status &= ~BQ24172_STATUS_MASK;
	pps->ac_status &= ~(BQ24172_BAT_ABS | BQ24172_UNSPEC_FAILURE);
	pps->ac_status |= charge_status;

	pps->status_count = 0;
	pps->total_faults = 0;
	pps->prev_charge_status = charge_status;
	pps->prev_status_time = now;
	mutex_unlock(&pps->work_lock);

//	printk("%s: notify change supplier %s of %s\n", __func__,
//			pps->power_supply_ac.name, (pps->power_supply_ac.supplied_to[0])?pps->power_supply_ac.supplied_to[0]:"unknown");
	power_supply_changed(&pps->power_supply_ac);

#if 0
	desc_stat->action->flags &= ~(pps->irq_triger);
	if(charge_status)
		pps->irq_triger = IRQF_TRIGGER_FALLING;
	else
		pps->irq_triger = IRQF_TRIGGER_RISING;

	desc_stat->action->flags |= pps->irq_triger;
	setup_irq(pps->charge_status_irq, desc_stat->action);
#endif

	enable_irq(pps->charge_status_irq);

//	schedule_delayed_work(&pps->charge_status_work, msecs_to_jiffies(BQ24172_BLINK_PERIOD));
}

static irqreturn_t do_charge_status_irq(int irq, void *irq_data)
{
	struct power_supply_info_i *pps = irq_data;

	// interrupt should be handled only
	// do nothing

	disable_irq_nosync(pps->charge_status_irq);
//	printk("%s: %lld\n", __func__, ktime_to_ms(ktime_get()));
//	cancel_delayed_work(&pps->charge_status_work);
//	mutex_lock(&pps->work_lock);
	pps->prev_charge_status = !gpio_get_value(pps->charge_status_in);
	pps->status_count = 0;
//	mutex_unlock(&pps->work_lock);
	schedule_delayed_work(&pps->charge_status_work, 0);

	return IRQ_HANDLED;
}

static void do_pwr_low_work(struct work_struct *work)
{
	struct power_supply_info_i *pps;
	signed long long now;
	int pwr_low_status, states = 4;

	pps = container_of(work, struct power_supply_info_i, pwr_low_work.work);

	if(-1 == pps->pwr_too_low_in)
	{
		printk("%s: low detection not configured %lld\n", __func__, ktime_to_ms(ktime_get()));
		return;
	}

	now = ktime_to_ms(ktime_get());
	do{
		pwr_low_status = gpio_get_value_cansleep(pps->pwr_too_low_in);
		msleep(10);
	}while(states--);
//	printk("%s: %d <- %d %lld\n", __func__, pps->pwr_too_low_in, pwr_low_status, ktime_to_ms(ktime_get()));
	if(pps->pwr_too_low_lvl != pwr_low_status)
	{
		pwr_low_status = 0;
//		printk("%s: notify restore power %s of %s\n", __func__,
//				pps->power_supply_ac.name, (pps->power_supply_ac.supplied_to[0])?pps->power_supply_ac.supplied_to[0]:"unknown");
	}
	else
	{
		printk("%s: power is below 8.2 V %lld\n", __func__, ktime_to_ms(ktime_get()));

		pwr_low_status = BQ24172_UNDERVOLTAGE;

//		printk("%s: notify fault supplier %s of %s\n", __func__,
//				pps->power_supply_ac.name, (pps->power_supply_ac.supplied_to[0])?pps->power_supply_ac.supplied_to[0]:"unknown");

	}
	mutex_lock(&pps->work_lock);
	pps->ac_status &= ~BQ24172_UNDERVOLTAGE;
	pps->ac_status |= pwr_low_status;
	mutex_unlock(&pps->work_lock);

	// TODO: notify about registered client
	power_supply_changed(&pps->power_supply_ac);
	enable_irq(pps->pwr_too_low_irq);
}

static irqreturn_t do_pwr_low_irq(int irq, void *irq_data)
{
	struct power_supply_info_i *pps = irq_data;

	// interrupt should be handled only
	// do nothing

	disable_irq_nosync(pps->pwr_too_low_irq);
//	printk("%s: %lld\n", __func__, ktime_to_ms(ktime_get()));
	schedule_delayed_work(&pps->pwr_low_work, 0);

	return IRQ_HANDLED;
}

static void do_pwr_high_work(struct work_struct *work)
{
	struct power_supply_info_i *pps;
	signed long long now;
	int pwr_high_status, states = 4;

	pps = container_of(work, struct power_supply_info_i, pwr_high_work.work);

	if(-1 == pps->pwr_too_high_in)
		return;

	now = ktime_to_ms(ktime_get());
	do{
		pwr_high_status = gpio_get_value_cansleep(pps->pwr_too_high_in);
		msleep(10);
	}while(states--);

//	printk("%s: %d <- %d %lld\n", __func__, pps->pwr_too_high_in, pwr_high_status, ktime_to_ms(ktime_get()));
	if(pps->pwr_too_high_lvl != pwr_high_status)
	{
		pwr_high_status = 0;
//		printk("%s: notify restore power %s of %s\n", __func__,
//				pps->power_supply_ac.name, (pps->power_supply_ac.supplied_to[0])?pps->power_supply_ac.supplied_to[0]:"unknown");
	}
	else
	{
		printk("%s: power is above 13 V %lld\n", __func__, ktime_to_ms(ktime_get()));

		pwr_high_status = BQ24172_OVERVOLTAGE;

//		printk("%s: notify fault supplier %s of %s\n", __func__,
//				pps->power_supply_ac.name, (pps->power_supply_ac.supplied_to[0])?pps->power_supply_ac.supplied_to[0]:"unknown");

	}
	mutex_lock(&pps->work_lock);
	pps->ac_status &= ~BQ24172_OVERVOLTAGE;
	pps->ac_status |= pwr_high_status;
	mutex_unlock(&pps->work_lock);
	// TODO: notify about registered client
	power_supply_changed(&pps->power_supply_ac);
	enable_irq(pps->pwr_too_high_irq);
}

static irqreturn_t do_pwr_high_irq(int irq, void *irq_data)
{
	struct power_supply_info_i *pps = irq_data;

	// interrupt should be handled only
	// do nothing

	disable_irq_nosync(pps->pwr_too_high_irq);
	//printk("%s: %lld\n", __func__, ktime_to_ms(ktime_get()));
	schedule_delayed_work(&pps->pwr_high_work, 0);

	return IRQ_HANDLED;
}

static int ignition_notifier(struct notifier_block *nb, unsigned long reason, void *dev)
{
	int stat;

	stat = ignition_get_state();

	mutex_lock(&ps_info->work_lock);
	switch(reason)
    {
    	case IGNITION_EVT_NOTIFY_IGN_ON:
    		//Whatever it needs
    		break;
    	case IGNITION_EVT_NOTIFY_IGN_OFF:
    	default:
    		//Whatever it needs
            break;
    }

	if(stat)
		ps_info->ac_status |= BQ24172_IGN_ONLINE;
	else
		ps_info->ac_status &= ~BQ24172_IGN_ONLINE;

	// check state in any case
	mutex_unlock(&ps_info->work_lock);

	printk("%s: ignition is %s, %lld\n", __func__, (stat)?"on":"off", ktime_to_ms(ktime_get()));
	power_supply_changed(&ps_info->power_supply_ac);

    return NOTIFY_OK;
}

static struct notifier_block ignition_change_notifier = {
	.notifier_call = ignition_notifier,
};

static int automotive_io_notifier(struct notifier_block *nb, unsigned long reason, void *dev)
{
	int stat;
	char *input_name, *state_name;

	//stat = ignition_get_state();

	mutex_lock(&ps_info->work_lock);
	switch(reason)
    {
    	case AUTOMOTIVE_EVT_NOTIFY_CRADLE_ON:
    		ps_info->ac_status |= BQ24172_CRDL_ONLINE;
    		input_name = "cradle";
    		state_name = "on";
    		break;
    	case AUTOMOTIVE_EVT_NOTIFY_CRADLE_OFF:
    		ps_info->ac_status &= ~BQ24172_CRDL_ONLINE;
    		input_name = "cradle";
    		state_name = "off";
    		break;
    	case AUTOMOTIVE_EVT_NOTIFY_WALL_ON:
    		ps_info->ac_status |= BQ24172_ONLINE;
    		input_name = "wall";
    		state_name = "on";
            break;
    	case AUTOMOTIVE_EVT_NOTIFY_WALL_OFF:
    		ps_info->ac_status &= ~BQ24172_ONLINE;
    		input_name = "wall";
    		state_name = "off";
            break;
    	default:
    		//Whatever it needs
    		input_name = "default";
    		state_name = "unknown";
            break;
    }

	// check state in any case
	mutex_unlock(&ps_info->work_lock);

	printk("%s: %s input is %s\n", __func__, input_name, state_name);
	power_supply_changed(&ps_info->power_supply_ac);

    return NOTIFY_OK;
}

static struct notifier_block automotive_change_notifier = {
	.notifier_call = automotive_io_notifier,
};


static int bq24172_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	int is_powered, any_in_connected;
	struct power_supply_info_i *pps;

	pps = container_of(psy, struct power_supply_info_i, power_supply_ac);
	is_powered = !(pps->ac_status & (BQ24172_OVERVOLTAGE | BQ24172_UNDERVOLTAGE));
	any_in_connected = (pps->ac_status & BQ24172_ONLINE) || (pps->ac_status & BQ24172_CRDL_ONLINE);

	switch(psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
			if((pps->ac_status & BQ24172_IGN_ONLINE) || (any_in_connected && is_powered))
				val->intval = 1;
//			else if(pps->ac_status & BQ24172_CHARGING)
//				val->intval = 1; // temporary for debugging purposes only
			else
				val->intval = 0;
//			printk("%s: POWER_SUPPLY_PROP_ONLINE (%s line)\n", __func__, (val->intval)?"on":"off");
			break;
		case POWER_SUPPLY_PROP_STATUS:
//			if(pps->ac_status & (BQ24172_ERROR_MASK & ~BQ24172_UNDERVOLTAGE))
//				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
//			else
			if(pps->ac_status & BQ24172_FULL)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if((pps->ac_status & BQ24172_CHARGING) && is_powered && any_in_connected)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			if(pps->ac_status & BQ24172_ERROR_MASK)
			{
				if(pps->ac_status & BQ24172_OVERHEAT)
					val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
				else if(pps->ac_status & BQ24172_DEAD)
					val->intval = POWER_SUPPLY_HEALTH_DEAD;
				else if(pps->ac_status & BQ24172_BAT_ABS)
					val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
				else if(pps->ac_status & BQ24172_OVERVOLTAGE)
					val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				else if(pps->ac_status & BQ24172_UNDERVOLTAGE)
					val->intval = POWER_SUPPLY_HEALTH_COLD;
				else
					val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

			}
			else if(pps->ac_status & BQ24172_GOOD)
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
			else
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static __devinit int bq24172_probe(struct platform_device *pdev)
{
	struct power_supply_info_i *pps;
	struct bq24172_platform_data *pdata = pdev->dev.platform_data;

	int rc;

	pps = kzalloc(sizeof(struct power_supply_info_i), GFP_KERNEL);
	if(!pps)
	{
		printk("%s: Add some memory\n", __func__);
		return -ENOMEM;
	}

	pps->dev = &pdev->dev;

	pps->power_supply_ac.name = "bq24172-charger";
	pps->power_supply_ac.type = POWER_SUPPLY_TYPE_MAINS;
	pps->power_supply_ac.supplied_to = bq24172_charger_supplied_to;
	pps->power_supply_ac.num_supplicants = ARRAY_SIZE(bq24172_charger_supplied_to),
	pps->power_supply_ac.properties = bq24172_props;
	pps->power_supply_ac.num_properties = ARRAY_SIZE(bq24172_props);
	pps->power_supply_ac.get_property = bq24172_get_property;

	pps->charge_status_in  = pdata->charge_status_in;
	pps->charge_status_lvl = pdata->charge_status_lvl;
	pps->pwr_too_low_in    = pdata->pwr_too_low_in;
	pps->pwr_too_low_lvl   = pdata->pwr_too_low_lvl;
	pps->pwr_too_high_in   = pdata->pwr_too_high_in;
	pps->pwr_too_high_lvl  = pdata->pwr_too_high_lvl;
	pps->v5_good_in 	   = pdata->v5_good_in;
	pps->v5_good_lvl 	   = pdata->v5_good_lvl;
	pps->irq_triger = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	pps->status_count = 0;
	pps->total_faults = 0;
	pps->prev_charge_status = -1;
	pps->prev_status_time = ktime_to_ms(ktime_get());

	ps_info = pps;

	mutex_init(&pps->work_lock);

	pps->ac_status = BQ24172_DISCHARGING | BQ24172_GOOD;
	do{
		if(pps->charge_status_in != -1)
		{
			rc = gpio_request(pps->charge_status_in, "charger-status");
			gpio_direction_input(pps->charge_status_in);
			pps->charge_status_irq = gpio_to_irq(pps->charge_status_in);
			rc = request_irq(pps->charge_status_irq, do_charge_status_irq, pps->irq_triger | IRQF_ONESHOT, pdev->name, pps);
			if(rc)
			{
				printk("%s: failed to allocate wake request irq (%d)\n", __func__, pps->charge_status_irq);
				break;
			}
			//enable_irq_wake(pps->charge_status_irq);
			disable_irq_nosync(pps->charge_status_irq);
		}

		if(pps->pwr_too_low_in != -1)
		{
			rc = gpio_request(pps->pwr_too_low_in, "power-too-low-status");
			gpio_direction_input(pps->pwr_too_low_in);
			pps->pwr_too_low_irq = gpio_to_irq(pps->pwr_too_low_in);
			rc = request_irq(pps->pwr_too_low_irq, do_pwr_low_irq, pps->irq_triger | IRQF_ONESHOT, pdev->name, pps);
			if(rc)
			{
				printk("%s: failed to allocate wake request irq (%d)\n", __func__, pps->pwr_too_low_irq);
				break;
			}
			//enable_irq_wake(pps->charge_status_irq);
			disable_irq_nosync(pps->pwr_too_low_irq);
		}

		if(pps->pwr_too_high_in != -1)
		{
			rc = gpio_request(pps->pwr_too_high_in, "power-too-high-status");
			gpio_direction_input(pps->pwr_too_high_in);
			pps->pwr_too_high_irq = gpio_to_irq(pps->pwr_too_high_in);
			rc = request_irq(pps->pwr_too_high_irq, do_pwr_high_irq, pps->irq_triger | IRQF_ONESHOT, pdev->name, pps);
			if(rc)
			{
				printk("%s: failed to allocate wake request irq (%d)\n", __func__, pps->pwr_too_high_irq);
				break;
			}
			//enable_irq_wake(pps->charge_status_irq);
			disable_irq_nosync(pps->pwr_too_high_irq);
		}

		platform_set_drvdata(pdev, pps);
		rc = power_supply_register(pps->dev, &pps->power_supply_ac);
		if(rc)
		{
			printk("%s: Failed to register power supply %s\n", __func__, pps->power_supply_ac.name);
			break;
		}

		ignitionevents_register_notifier(&ignition_change_notifier);
		automotiveevents_register_notifier(&automotive_change_notifier);
		if(automotiveio_get_wall_state())
			pps->ac_status |= BQ24172_ONLINE;
		if(automotiveio_get_cradle_state())
			pps->ac_status |= BQ24172_CRDL_ONLINE;

		// get it if configured only
//		if(ignition_get_state())
//			pps->ac_status |= BQ24172_CRDL_ONLINE;
		pps->prev_status_time = 0;
		INIT_DELAYED_WORK(&pps->charge_status_work, do_charge_status_work);
		INIT_DELAYED_WORK(&pps->pwr_low_work, do_pwr_low_work);
		INIT_DELAYED_WORK(&pps->pwr_high_work, do_pwr_high_work);
		schedule_delayed_work(&pps->charge_status_work, msecs_to_jiffies(100));
		schedule_delayed_work(&pps->pwr_low_work, msecs_to_jiffies(200));
		schedule_delayed_work(&pps->pwr_high_work, msecs_to_jiffies(300));

		printk("%s: bq24172 charger registered (ig_in %d)\n", __func__, pps->charge_status_in);

		return 0;
	}while(0);

	if(pps->charge_status_in != -1)
	{
		if(pps->charge_status_irq)
		{
			disable_irq_nosync(pps->charge_status_irq);
			free_irq(pps->charge_status_irq, &pdev->dev);
		}
		gpio_free(pps->charge_status_in);
	}
	if(pps->pwr_too_low_in != -1)
	{
		if(pps->pwr_too_low_irq)
		{
			disable_irq_nosync(pps->pwr_too_low_irq);
			free_irq(pps->pwr_too_low_irq, &pdev->dev);
		}
		gpio_free(pps->pwr_too_low_in);
	}
	if(pps->pwr_too_high_in != -1)
	{
		if(pps->pwr_too_high_irq)
		{
			disable_irq_nosync(pps->pwr_too_high_irq);
			free_irq(pps->pwr_too_high_irq, &pdev->dev);
		}
		gpio_free(pps->pwr_too_high_in);
	}
	power_supply_unregister(&pps->power_supply_ac);
	kfree(pps);

	return rc;
}

static __devexit int bq24172_remove(struct platform_device *pdev)
{
	struct power_supply_info_i *pps = platform_get_drvdata(pdev);

	if(-1 != pps->charge_status_in)
	{
		disable_irq_nosync(pps->charge_status_irq);
		cancel_delayed_work(&pps->charge_status_work);
		free_irq(pps->charge_status_irq, &pdev->dev);
		gpio_free(pps->charge_status_in);
	}
	if(pps->pwr_too_low_in != -1)
	{
		if(pps->pwr_too_low_irq)
		{
			disable_irq_nosync(pps->pwr_too_low_irq);
			free_irq(pps->pwr_too_low_irq, &pdev->dev);
		}
		gpio_free(pps->pwr_too_low_in);
	}
	if(pps->pwr_too_high_in != -1)
	{
		if(pps->pwr_too_high_irq)
		{
			disable_irq_nosync(pps->pwr_too_high_irq);
			free_irq(pps->pwr_too_high_irq, &pdev->dev);
		}
		gpio_free(pps->pwr_too_high_in);
	}

	mutex_destroy(pps->work_lock);
	power_supply_unregister(&pps->power_supply_ac);

	kfree(pps);
	//ps_info = pps = 0;

	return 0;
}

static struct platform_driver bq24172_charger_driver = {
	.probe		= bq24172_probe,
	.remove		= __devexit_p(bq24172_remove),
	.driver = {
		.name	= "bq24172-charger",
		.owner = THIS_MODULE,
	},
};

static int __init bq24172_init(void)
{
//	printk("%s: register power\n", __func__);
	return platform_driver_register(&bq24172_charger_driver);
}
module_init(bq24172_init);


static void __exit bq24172_destroy(void)
{
//	printk("%s: destroy power supply\n", __func__);
	platform_driver_unregister(&bq24172_charger_driver);
}
module_exit(bq24172_destroy);


MODULE_DESCRIPTION("bq24172 charger monitor driver");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimirz@micronet.co.il>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bq24172");
