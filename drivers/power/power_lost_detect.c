/**
 * power_lost_detect.c - Power Lost Detect Driver
 *
 * Copyright (C) 2013 Micronet Ltd.
 *
 * Written by Vladimir Zatulovsky <vladimirz@micronet.co.il>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <generated/autoconf.h>
#include <linux/init.h>
#include <linux/syscalls.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/namei.h>

#define DEFAULT_RST_DELAY 	1000
#define DEFAULT_IGNRE_DELAY 250
#define DEFAULT_USD_DELAY 	0
enum {
	pwl_unspecified = -1,
	pwl_display_off,
	pwl_fs_off
}e_pwl_status;

struct power_lost_detect_platform_data {
	int pl_det;
	int rst_det;
	int wake_en;
	int pwron_clk_en;
	int pl_act_lvl;
	int rst_act_lvl;
	int clk_en_pol;
};

struct power_lost_detect_info {
	int pl_irq;
	int rst_irq;
	int wake_req_irq;
	int	ps;
	int pld_work_timer;
	int rst_work_timer;
	int pld_ignore_timer;
	int pld_usd_timer;
	int rst_reset_timer;
	struct device 		*dev;
	struct mutex 		work_lock;
	struct delayed_work pld_work;
	struct delayed_work rst_work;
	struct power_lost_detect_platform_data in;
};
struct power_lost_detect_info *ppd;

static ssize_t power_lost_detect_show_usd(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct power_lost_detect_info *pd = dev_get_drvdata(pdev);

	return sprintf(buf, "%u\n", pd->pld_usd_timer);
}

static ssize_t power_lost_detect_show_rst(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct power_lost_detect_info *pd = dev_get_drvdata(pdev);

	return sprintf(buf, "%u\n", pd->rst_reset_timer);
}

static ssize_t power_lost_detect_set_usd(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_lost_detect_info *pd = dev_get_drvdata(pdev);
	int new_value = 0;

	sscanf(buf, "%d", &new_value);

	printk("%s: new urgent shutdown timer (%d, %s -> %d) %lld\n", __func__, count, buf, new_value, ktime_to_ms(ktime_get()));

	mutex_lock(&pd->work_lock);
	pd->pld_usd_timer = new_value;
	mutex_unlock(&pd->work_lock);

	return count;
}

static ssize_t power_lost_detect_set_rst(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_lost_detect_info *pd = dev_get_drvdata(pdev);
	int new_value = 0;

	sscanf(buf, "%d", &new_value);

	printk("%s: new reset timer (%d, %s -> %d) %lld\n", __func__, count, buf, new_value, ktime_to_ms(ktime_get()));

	mutex_lock(&pd->work_lock);
	pd->rst_reset_timer = new_value;
	mutex_unlock(&pd->work_lock);

	return count;
}

static DEVICE_ATTR(usd_delay, S_IWUSR | S_IRUGO | S_IWGRP, power_lost_detect_show_usd, power_lost_detect_set_usd);
static DEVICE_ATTR(rst_delay, S_IWUSR | S_IRUGO | S_IWGRP, power_lost_detect_show_rst, power_lost_detect_set_rst);

static ssize_t power_lost_detect_set_suspend(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{

	return count;
}

static DEVICE_ATTR(suspend, S_IWUSR | S_IRUGO | S_IWGRP, 0, power_lost_detect_set_suspend);

static ssize_t power_lost_detect_set_fs_sync(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
//	struct power_lost_detect_info *pd = dev_get_drvdata(pdev);
	int new_value = 0;

	sscanf(buf, "%d", &new_value);

	if(new_value)
	{
		sys_sync();
		//mount()
		//sys_mount
		//sys_umount("/data", MNT_DETACH);
	}
	else
	{
//		sys_umount("/data", MNT_DETACH);
	}

	return count;
}

static DEVICE_ATTR(fs_sync, S_IWUSR | S_IRUGO | S_IWGRP, 0, power_lost_detect_set_fs_sync);

static struct attribute *power_lost_detect_attributes[] = {
	&dev_attr_usd_delay.attr,
	&dev_attr_rst_delay.attr,
	&dev_attr_suspend.attr,
	&dev_attr_fs_sync.attr,
	NULL
};

static const struct attribute_group power_lost_detect_attr_group = {
	.attrs = power_lost_detect_attributes
};

extern int dpm_dev_suspend(char *);
extern int dpm_dev_resume(char *);

static void do_rst_work(struct work_struct *work)
{
	struct power_lost_detect_info *pld;
	struct irq_desc *desc_rst;// = irq_to_desc(pld->rst_irq);

	printk("%s: %lld\n", __func__, ktime_to_ms(ktime_get()));

	pld = container_of(work, struct power_lost_detect_info, rst_work.work);
	desc_rst = irq_to_desc(pld->rst_irq);

	mutex_lock(&pld->work_lock);
	if(pld->in.rst_act_lvl != gpio_get_value_cansleep(pld->in.rst_det))
	{
		printk("%s: do nothing, wait for interrupt %lld\n", __func__, ktime_to_ms(ktime_get()));

		pld->rst_work_timer = -1;
		mutex_unlock(&pld->work_lock);
		while(0 < desc_rst->depth)
			enable_irq(pld->rst_irq);

		return;
	}

	if(-1 == pld->rst_work_timer)
	{
		// interrupt just occurred
		printk("%s: wait for reset timeout %lld\n", __func__, ktime_to_ms(ktime_get()));

		pld->rst_work_timer = pld->rst_reset_timer;
		// continue to wait
	}
	else
	{
		printk("%s: sync fs %lld\n", __func__, ktime_to_ms(ktime_get()));
		sys_sync();
		printk("%s: platform will reset now %lld\n", __func__, ktime_to_ms(ktime_get()));
		kernel_restart(NULL);
		while(1);
	}
	mutex_unlock(&pld->work_lock);
	schedule_delayed_work(&pld->rst_work, (pld->rst_work_timer)?msecs_to_jiffies(pld->rst_work_timer): 0);
}

static void do_pld_work(struct work_struct *work)
{
	struct power_lost_detect_info *pld;
	struct path path;
	struct irq_desc *desc_pl;

	pld = container_of(work, struct power_lost_detect_info, pld_work.work);
	desc_pl = irq_to_desc(pld->pl_irq);

	mutex_lock(&pld->work_lock);
	if(pld->in.pl_act_lvl != gpio_get_value_cansleep(pld->in.pl_det))
	{
		if(pld->ps != pwl_unspecified)
		{
			if(pld->ps == pwl_fs_off) //!= pwl_unspecified)
			{
				// restore fs power state
				// currently should not come here
				sys_mount("ubi0:rootfs", "/", "ubifs", MS_SILENT, 0);
				sys_mount("/dev/ubi1_1", "/data", "ubifs", MS_SILENT, 0);
				printk("%s: restore fs power state %lld\n", __func__, ktime_to_ms(ktime_get()));
			}
			// restore display and lights power state
			printk("%s: restore display and lights power state %lld\n", __func__, ktime_to_ms(ktime_get()));
			dpm_dev_resume("omapdss");//a300-panel");

			pld->ps = pwl_unspecified;
		}
		pld->pld_work_timer = -1;
		mutex_unlock(&pld->work_lock);

		while(0 < desc_pl->depth)
			enable_irq(pld->pl_irq);
		return;
	}

	if(-1 == pld->pld_work_timer)
	{
		if(user_path_at(AT_FDCWD, "/data", 0, &path))
		{
			printk("%s: system not ready; urgent shutdown %lld\n", __func__, ktime_to_ms(ktime_get()));
			machine_power_off();
		}
		// interrupt just occurred
		if(pld->ps != pwl_unspecified)
		{
			printk("%s: oops - bug bug bug %lld\n", __func__, ktime_to_ms(ktime_get()));
		}

		printk("%s: Shutdown display and lights %lld\n", __func__, ktime_to_ms(ktime_get()));

		pld->pld_work_timer = pld->pld_ignore_timer;
		pld->ps = pwl_display_off;

		dpm_dev_suspend("omapdss");//a300-panel");
		// continue to wait
	}
	else
	{
		if(pld->ps == pwl_display_off)
		{
			printk("%s: Shutdown fs %lld\n", __func__, ktime_to_ms(ktime_get()));

			pld->pld_work_timer = pld->pld_usd_timer;
			pld->ps = pwl_fs_off;

			// do something here
			if(user_path_at(AT_FDCWD, "/data", 0, &path))
				printk("%s: impossible unmount partitions %lld\n", __func__, ktime_to_ms(ktime_get()));
			else
			{
				sys_sync();

				sys_umount("/data", MNT_FORCE | MNT_DETACH);
				sys_umount("/", MNT_FORCE | MNT_DETACH);

				printk("%s: fs in safe mode %lld\n", __func__, ktime_to_ms(ktime_get()));
			}
		}
		else
		{
			printk("%s: urgent shutdown %lld\n", __func__, ktime_to_ms(ktime_get()));
			//kernel_power_off();
			machine_power_off();
		}
	}
	mutex_unlock(&pld->work_lock);
	schedule_delayed_work(&pld->pld_work, (pld->pld_work_timer)?msecs_to_jiffies(pld->pld_work_timer): 0);
}

static irqreturn_t do_pld_irq(int irq, void *irq_data)
{
	struct power_lost_detect_info *pld = irq_data;

	disable_irq_nosync(pld->pl_irq);
	printk("%s: power is lost %lld\n", __func__, ktime_to_ms(ktime_get()));
	mutex_lock(&pld->work_lock);
	pld->pld_work_timer = -1;
	mutex_unlock(&pld->work_lock);
	schedule_delayed_work(&pld->pld_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t do_rst_irq(int irq, void *irq_data)
{
	struct power_lost_detect_info *pld = irq_data;

	disable_irq_nosync(pld->rst_irq);
	printk("%s: reset indication %lld\n", __func__, ktime_to_ms(ktime_get()));
	mutex_lock(&pld->work_lock);
	pld->rst_work_timer = -1;
	mutex_unlock(&pld->work_lock);
	schedule_delayed_work(&pld->rst_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t do_wake_irq(int irq, void *irq_data)
{
	struct power_lost_detect_info *pld = irq_data;

	// interrupt should be handled only
	// do nothing
	//	printk("!!!do_wake_irq!!!\n");
	disable_irq_nosync(pld->wake_req_irq);

	return IRQ_HANDLED;
}

static __devinit int power_lost_detect_probe(struct platform_device *pdev)
{
	int rc, irq_type;
	struct power_lost_detect_info *pld;
	struct power_lost_detect_platform_data *pdata = pdev->dev.platform_data;

	pld = kzalloc(sizeof(struct power_lost_detect_info), GFP_KERNEL);
	if(!pld)
	{
		printk("%s: Add some memory\n", __func__);
		return -ENOMEM;
	}

	pld->dev = &pdev->dev;
	memcpy(&pld->in, pdata, sizeof(struct power_lost_detect_platform_data));
	ppd = pld;

	mutex_init(&ppd->work_lock);

	ppd->ps = pwl_unspecified;

	ppd->pld_ignore_timer = DEFAULT_IGNRE_DELAY;
	ppd->pld_usd_timer = DEFAULT_USD_DELAY;
	ppd->pl_irq = -1;
	if(ppd->in.pl_det != -1)
	{
		gpio_request(ppd->in.pl_det, "power-detector-state");
		gpio_direction_input(ppd->in.pl_det);
		ppd->pl_irq = gpio_to_irq(ppd->in.pl_det);
		gpio_set_debounce(ppd->in.pl_det, 1023);
	}
	ppd->rst_reset_timer = DEFAULT_RST_DELAY;
	gpio_request(ppd->in.rst_det, "warm-reset-request");
	gpio_direction_input(ppd->in.rst_det);
	ppd->rst_irq = gpio_to_irq(ppd->in.rst_det);
	gpio_set_debounce(ppd->in.rst_det, 340);

	gpio_request(ppd->in.wake_en, "wake-request");
	gpio_direction_input(ppd->in.wake_en);
	ppd->wake_req_irq = gpio_to_irq(ppd->in.wake_en);

	gpio_request(ppd->in.pwron_clk_en, "pwron-clk-enable");
	gpio_direction_output(ppd->in.pwron_clk_en, !ppd->in.clk_en_pol);

	do
	{
		rc = request_irq(ppd->wake_req_irq, do_wake_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_NO_SUSPEND, pdev->name, ppd);
		if(rc)
		{
			printk("%s: failed to allocate wake request irq (%d)\n", __func__, ppd->pl_irq);
			break;
		}

		if(ppd->in.pl_det != -1)
		{
			if(ppd->in.pl_act_lvl)
				irq_type = IRQF_TRIGGER_HIGH;
			else
				irq_type = IRQF_TRIGGER_LOW;
			rc = request_irq(ppd->pl_irq, do_pld_irq, irq_type | IRQF_ONESHOT, pdev->name, ppd);
			if(rc)
			{
				printk("%s: failed to allocate power lost detect irq (%d)\n", __func__, ppd->pl_irq);
				break;
			}
		}
		rc = request_irq(ppd->rst_irq, do_rst_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT, pdev->name, ppd);
		if(rc)
		{
			printk("%s: failed to allocate warm reset irq (%d)\n", __func__, ppd->rst_irq);
			break;
		}
		if(ppd->in.pl_det != -1)
			enable_irq_wake(ppd->pl_irq);
		enable_irq_wake(ppd->rst_irq);
		enable_irq_wake(ppd->wake_req_irq);

		disable_irq_nosync(ppd->wake_req_irq);
		disable_irq_nosync(ppd->rst_irq);

		platform_set_drvdata(pdev, ppd);

		pld->pld_work_timer = -1;
		pld->rst_work_timer = -1;
		INIT_DELAYED_WORK(&ppd->pld_work, do_pld_work);
		INIT_DELAYED_WORK(&ppd->rst_work, do_rst_work);

		rc = sysfs_create_group(&pld->dev->kobj, &power_lost_detect_attr_group);

		schedule_delayed_work(&pld->rst_work, (pld->rst_work_timer)?msecs_to_jiffies(4000): 0);

		printk("%s: power lost detector registered (%d, %d)\n", __func__, ppd->in.pl_det, ppd->in.rst_det);

		return 0;
	}while(0);

	free_irq(ppd->pl_irq, ppd);
	free_irq(ppd->rst_irq, ppd);
	free_irq(ppd->wake_req_irq, ppd);

	gpio_free(ppd->in.rst_det);
	if(ppd->in.pl_det != -1)
		gpio_free(ppd->in.pl_det);
	gpio_free(ppd->in.wake_en);
	gpio_free(ppd->in.pwron_clk_en);

	kfree(ppd);
	ppd = pld = 0;

	return rc;
}


static __devexit
int power_lost_detect_remove(struct platform_device *pdev)
{
	struct power_lost_detect_info *pl = platform_get_drvdata(pdev);

	sysfs_remove_group(&pl->dev->kobj, &power_lost_detect_attr_group);
	cancel_delayed_work(&pl->pld_work);
	cancel_delayed_work(&pl->rst_work);
	free_irq(ppd->pl_irq, pl);
	free_irq(ppd->rst_irq, pl);
	if(pl->in.pl_det != -1)
		gpio_free(pl->in.pl_det);
	gpio_free(pl->in.rst_det);
	mutex_destroy(pl->work_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(pl);
	ppd = pl = 0;

	return 0;
}

#if defined CONFIG_PM
static int power_lost_detect_suspend(struct device *dev)
{
	int irq_type;
	struct platform_device *pdev = to_platform_device(dev);
	struct power_lost_detect_info *pl = platform_get_drvdata(pdev);
	struct irq_desc *desc_pl; //= irq_to_desc(pl->pl_irq);
	struct irq_desc *desc_rst = irq_to_desc(pl->rst_irq);
	struct irq_desc *desc_wake = irq_to_desc(pl->wake_req_irq);

	if(pl->pl_irq != -1)
		desc_pl = irq_to_desc(pl->pl_irq);

	set_irq_type(pl->wake_req_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND );

	while(0 < desc_wake->depth)
		enable_irq(pl->wake_req_irq);
	enable_irq_wake(pl->wake_req_irq);
	gpio_set_value(pl->in.pwron_clk_en, pl->in.clk_en_pol);

	if(pl->in.pl_act_lvl)
		irq_type = IRQF_TRIGGER_HIGH;
	else
		irq_type = IRQF_TRIGGER_LOW;

	if(pl->pl_irq != -1)
	{
		desc_pl->action->flags &= ~irq_type;
		desc_pl->action->flags |= IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
	}
	desc_rst->action->flags &= ~IRQF_TRIGGER_HIGH;
	desc_rst->action->flags |= IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;

	if(pl->pl_irq != -1)
		setup_irq(pl->pl_irq, desc_pl->action);
	setup_irq(pl->rst_irq, desc_rst->action);

	printk("%s\n", __func__);

	return 0;
}

static int power_lost_detect_resume(struct device *dev)
{
	int irq_type;
	struct platform_device *pdev = to_platform_device(dev);
	struct power_lost_detect_info *pl = platform_get_drvdata(pdev);
	struct irq_desc *desc_pl; // = irq_to_desc(pl->pl_irq);
	struct irq_desc *desc_rst = irq_to_desc(pl->rst_irq);

	if(pl->pl_irq != -1)
		desc_pl = irq_to_desc(pl->pl_irq);

	set_irq_type(pl->wake_req_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_NO_SUSPEND);
	disable_irq_nosync(pl->wake_req_irq);
	disable_irq_wake(pl->wake_req_irq);
	gpio_set_value(pl->in.pwron_clk_en, !pl->in.clk_en_pol);

	if(pl->in.pl_act_lvl)
		irq_type = IRQF_TRIGGER_HIGH;
	else
		irq_type = IRQF_TRIGGER_LOW;

	if(pl->pl_irq != -1)
	{
		desc_pl->action->flags &= ~(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
		desc_pl->action->flags |=  irq_type;
	}
	desc_rst->action->flags &= ~(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	desc_rst->action->flags |=  IRQF_TRIGGER_HIGH;

	if(pl->pl_irq != -1)
		setup_irq(pl->pl_irq, desc_pl->action);
	setup_irq(pl->rst_irq, desc_rst->action);

	printk("%s:\n", __func__);

	return 0;
}
#else
#define power_lost_detect_suspend NULL
#define power_lost_detect_resume NULL
#endif

static const struct dev_pm_ops power_lost_detect_pm_ops =
{
	.suspend	= power_lost_detect_suspend,
	.resume		= power_lost_detect_resume,
};

static struct platform_driver power_lost_detect_driver = {
	.probe		= power_lost_detect_probe,
	.remove		= __devexit_p(power_lost_detect_remove),
//	.suspend	= power_lost_detect_suspend,
//	.resume		= power_lost_detect_resume,
	.driver		= {
		.name	= "power-lost-detect",
		.owner	= THIS_MODULE,
		.pm = &power_lost_detect_pm_ops,
	},
};

static int __init
power_lost_detect_init(void)
{
	return platform_driver_register(&power_lost_detect_driver);
}
module_init(power_lost_detect_init);

static void __exit
power_lost_detect_exit(void)
{
	platform_driver_unregister(&power_lost_detect_driver);
}
module_exit(power_lost_detect_exit);

MODULE_DESCRIPTION("Power Lost Detect Driver");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimirz@micronet.co.il>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:power-lost-detect");

