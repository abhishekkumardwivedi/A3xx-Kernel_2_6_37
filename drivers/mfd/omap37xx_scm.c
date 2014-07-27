/*
 * OMAP37XX system control module driver file
 *
 * Copyright (C) 2013 Micronet IL - http://www.micronet.co.il/
 * Author: R Meyerstein <ranm@micronet.co.il>
 *
 * (C) Copyright 2012 - 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * A31x boards adaptation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <plat/omap_device.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/module.h>
#include <plat/scm.h>
#include <linux/mfd/omap3_scm.h>
#include <linux/syscalls.h>

#include <linux/thermal.h>
#include <linux/omap_thermal.h>

#include <linux/irq.h>

#define DRIVER_NAME "SCM3703"

u32 omap37XX_scm_readl(struct scm *scm_ptr, u32 reg)
{
	return __raw_readl(scm_ptr->base + reg);
}
EXPORT_SYMBOL_GPL(omap37XX_scm_readl);

void omap37XX_scm_writel(struct scm *scm_ptr, u32 val, u32 reg)
{
	__raw_writel(val, scm_ptr->base + reg);
}
EXPORT_SYMBOL_GPL(omap37XX_scm_writel);

static void omap37XX_scm_client_register(struct temp_sensor_hwmon *tsh_ptr,
				   int i, const char *name)
{
	int ret;

	printk("omap37XX_scm_client_register entered\n");

	tsh_ptr->pdev = platform_device_alloc(name, i);

	if (tsh_ptr->pdev == NULL) {
		dev_err(tsh_ptr->scm_ptr->dev, "Failed to allocate %s\n", name);
		return;
	}

	tsh_ptr->pdev->dev.parent = tsh_ptr->scm_ptr->dev;
	platform_set_drvdata(tsh_ptr->pdev, tsh_ptr);
	ret = platform_device_add(tsh_ptr->pdev);
	if (ret != 0) {
		dev_err(tsh_ptr->scm_ptr->dev, "Failed to register %s: %d\n",
			name, ret);
		platform_device_put(tsh_ptr->pdev);
		tsh_ptr->pdev = NULL;
	}
}

static irqreturn_t omap37XX_tshut_irq_handler(int irq, void *data)
{
    sys_sync();

    sys_umount("/data", MNT_DETACH);
    sys_umount("/", MNT_DETACH);
    kernel_power_off(); 

	return IRQ_HANDLED;
}

static irqreturn_t do_overheat_irq(int irq, void *irq_data)
{
	struct scm *scm_ptr = irq_data;

	disable_irq_nosync(scm_ptr->overheat_irq);
	schedule_delayed_work(&scm_ptr->overhet_work, 0);

	return IRQ_HANDLED;
}

static void do_overheat_work(struct work_struct *work)
{
	struct scm *scm_ptr;
	signed long long now;

	scm_ptr = container_of(work, struct scm, overhet_work.work);

	now = ktime_to_ms(ktime_get());
	if(gpio_get_value_cansleep(scm_ptr->overheat_irq_in) == scm_ptr->overheat_irq_pol)
	{
		printk("%s: over heat %lld\n", __func__, now);
		mutex_lock(&scm_ptr->scm_mutex);
		omap3703_scm_read_temp(scm_ptr, 0);
		mutex_unlock(&scm_ptr->scm_mutex);
	}
	enable_irq(scm_ptr->overheat_irq);
}

static int __devinit omap37XX_scm_probe(struct platform_device *pdev)
{
	struct omap37XX_scm_pdata *pdata = pdev->dev.platform_data;
	struct scm *scm_ptr;
	int ret = 0, i;

	printk("omap37XX_scm_probe entered\n");

	if (!pdata) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -EINVAL;
	}

	scm_ptr = kzalloc(sizeof(*scm_ptr), GFP_KERNEL);
	if (!scm_ptr) {
		dev_err(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	scm_ptr->cnt = pdata->cnt;
	scm_ptr->overheat_irq_in = pdata->overheat_irq_in;
	scm_ptr->overheat_irq_pol = pdata->overheat_irq_pol;

	mutex_init(&scm_ptr->scm_mutex);

	scm_ptr->irq = gpio_to_irq(127);
	if (scm_ptr->irq < 0) {
		dev_err(&pdev->dev, "gpio_to_irq failed\n");
		ret = scm_ptr->irq;
		goto plat_res_err;
	}
	printk("omap37XX_scm_probe: Got irq[%d]\n",scm_ptr->irq);

	if(scm_ptr->overheat_irq_in != -1)
	{
		if(gpio_request(scm_ptr->overheat_irq_in, "over-heat-notify") < 0)
			printk("%s, failure to request over heat notification\n", __func__);
		else
		{
			gpio_direction_input(scm_ptr->overheat_irq_in);
			scm_ptr->overheat_irq = gpio_to_irq(scm_ptr->overheat_irq_in);
			if(request_irq(scm_ptr->overheat_irq, do_overheat_irq,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, pdev->name, scm_ptr) < 0)
			{
				printk("%s: failed to allocate over heat IRQ (%d)\n", __func__, scm_ptr->overheat_irq);
			}
			else
			{
				//enable_irq_wake(pps->charge_status_irq);
				disable_irq_nosync(scm_ptr->overheat_irq);
			}
		}
	}
	scm_ptr->dev = &pdev->dev;

	dev_set_drvdata(&pdev->dev, scm_ptr);

	scm_ptr->tsh_ptr = kzalloc(sizeof(*scm_ptr->tsh_ptr) * scm_ptr->cnt, GFP_KERNEL);
	if (!scm_ptr)
		dev_err(&pdev->dev, "Memory allocation failed for tsh\n");

/* Initialize temperature sensors */
	ret = omap3703_temp_sensor_init(scm_ptr);

	if (ret) dev_err(&pdev->dev, "Temperature sensor init failed\n");
	if (scm_ptr->rev == 1) ret = omap3703_tshut_init(scm_ptr);

	for (i = 0; i < scm_ptr->cnt; i++) {
		scm_ptr->tsh_ptr[i].scm_ptr = scm_ptr;
		omap37XX_scm_client_register(&(scm_ptr->tsh_ptr[i]), i, "temp_sensor_hwmon");
	}

	ret = request_threaded_irq(gpio_to_irq(127), NULL,
							   omap37XX_tshut_irq_handler,
							   IRQF_TRIGGER_RISING, "tshut",
							   NULL);
	if (ret) {
		pr_err("request irq failed for TSHUT");
		goto plat_res_err;
	}

	if(scm_ptr->overheat_irq_in != -1)
	{
		INIT_DELAYED_WORK(&scm_ptr->overhet_work, do_overheat_work);
		schedule_delayed_work(&scm_ptr->overhet_work, 0);
	}

	return 0;

plat_res_err:
	printk("failed in omap37XX_scm_probe\n");
	mutex_destroy(&scm_ptr->scm_mutex);

	return ret;
}

static int __devexit
omap37XX_scm_remove(struct platform_device *pdev)
{
	struct scm *scm_ptr = platform_get_drvdata(pdev);

	printk("omap37XX_scm_remove entered\n");


	if(scm_ptr->overheat_irq_in != -1)
	{
		cancel_delayed_work(&scm_ptr->overhet_work);
		disable_irq_nosync(scm_ptr->overheat_irq);
		free_irq(scm_ptr->overheat_irq, &pdev->dev);
		gpio_free(scm_ptr->overheat_irq_in);
	}
	free_irq(scm_ptr->irq, scm_ptr);
	clk_disable(scm_ptr->fclock);
	clk_put(scm_ptr->fclock);
	clk_put(scm_ptr->div_clk);
	iounmap(scm_ptr->base);
	dev_set_drvdata(&pdev->dev, NULL);
	mutex_destroy(&scm_ptr->scm_mutex);
	kfree(scm_ptr);

	return 0;
}

static void omap37XX_scm_save_ctxt(struct scm *scm_ptr)
{
	printk("omap37XX_scm_save_ctxt entered\n");
}


static void omap37XX_scm_restore_ctxt(struct scm *scm_ptr)
{

	printk("omap37XX_scm_restore_ctxt entered\n");

#if 0
	int i, temp = 0;

	for (i = 0; i < scm_ptr->cnt; i++) {
		if ((omap37XX_scm_readl(scm_ptr,
			scm_ptr->registers[i]->bgap_counter) == 0)) {
			omap37XX_scm_writel(scm_ptr,
				scm_ptr->regval[i]->bg_threshold,
				scm_ptr->registers[i]->bgap_threshold);
			omap37XX_scm_writel(scm_ptr,
				scm_ptr->regval[i]->tshut_threshold,
					scm_ptr->registers[i]->tshut_threshold);
			/* Force immediate temperature measurement and update
			 * of the DTEMP field
			 */
			omap_temp_sensor_force_single_read(scm_ptr, i);
			omap37XX_scm_writel(scm_ptr,
				scm_ptr->regval[i]->bg_counter,
				scm_ptr->registers[i]->bgap_counter);
			omap37XX_scm_writel(scm_ptr,
				scm_ptr->regval[i]->bg_mode_ctrl,
				scm_ptr->registers[i]->bgap_mode_ctrl);
			omap37XX_scm_writel(scm_ptr,
				scm_ptr->regval[i]->bg_ctrl,
				scm_ptr->registers[i]->bgap_mask_ctrl);
		} else {
			temp = omap37XX_scm_readl(scm_ptr,
				scm_ptr->registers[i]->temp_sensor_ctrl);
			temp &= (scm_ptr->registers[i]->bgap_dtemp_mask);
			if (temp == 0) {
				omap_temp_sensor_force_single_read(scm_ptr, i);
				temp = omap37XX_scm_readl(scm_ptr,
					scm_ptr->registers[i]->bgap_mask_ctrl);
				temp |= 1 <<
				__ffs(scm_ptr->registers[i]->mode_ctrl_mask);
				omap37XX_scm_writel(scm_ptr, temp,
					scm_ptr->registers[i]->bgap_mask_ctrl);
			}
		}
	}
#endif
}

static int omap37XX_scm_suspend(struct device *dev)
{
	struct scm *scm_ptr = dev_get_drvdata(dev);

	printk("omap37XX_scm_suspend entered\n");

	omap37XX_scm_save_ctxt(scm_ptr);
	clk_disable(scm_ptr->fclock);

	return 0;
}

static int omap37XX_scm_resume(struct device *dev)
{
	struct scm *scm_ptr = dev_get_drvdata(dev);

	printk("omap37XX_scm_resume entered\n");

	clk_enable(scm_ptr->fclock);
	omap37XX_scm_restore_ctxt(scm_ptr);

	return 0;
}
static const struct dev_pm_ops omap37XX_scm_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(omap37XX_scm_suspend,
			omap37XX_scm_resume)
};

#define DEV_PM_OPS	(&omap37XX_scm_dev_pm_ops)

static struct platform_driver omap37XX_scm_driver = {
	.probe = omap37XX_scm_probe,
	.remove = omap37XX_scm_remove,
	.driver = {
			.name = "omap37XX_scm",
			.pm = DEV_PM_OPS,
		   },
};

static int __init
omap37XX_scm_init(void)
{
	printk("omap37XX_scm_init entered\n");
	return platform_driver_register(&omap37XX_scm_driver);
}

module_init(omap37XX_scm_init);

static void __exit
omap37XX_scm_exit(void)
{
	printk("omap37XX_scm_exit entered\n");
	platform_driver_unregister(&omap37XX_scm_driver);
}

module_exit(omap37XX_scm_exit);

MODULE_DESCRIPTION("OMAP3 system control module Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("R Meyerstein <ranm@micronet.co.il>");

