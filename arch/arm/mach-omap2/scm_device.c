/*
 * OMAP3703 SCM device file
 *
 * Copyright (C) 2013 Micronet IL - http://www.micronet.co.il/
 * Author: R Meyerstein <ranm@micronet.co.il>
 *
 * (C) Copyright 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * A31x board adaptation
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

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/idr.h>
#include <plat/omap_device.h>
#include "pm.h"
#include <plat/scm.h>

static DEFINE_IDR(scm_device_idr);

static int scm_dev_init(struct omap_hwmod *oh, void *user)
{
	struct	omap37XX_scm_pdata		*scm_pdata;
	struct	omap_device			    *od;
	int					ret = 0;
	int					num;

	scm_pdata = kzalloc(sizeof(*scm_pdata), GFP_KERNEL);
	if (!scm_pdata) {
		dev_err(&oh->od->pdev.dev,
			"Unable to allocate memory for scm pdata\n");
		return -ENOMEM;
	}

#if defined (CONFIG_MACH_A317)
	scm_pdata->overheat_irq_in  = 37;
	scm_pdata->overheat_irq_pol = 1;
#else
	scm_pdata->overheat_irq_in = -1;
#endif
	ret = idr_pre_get(&scm_device_idr, GFP_KERNEL);

	if (ret == 0)
		goto fail_id;

	ret = idr_get_new(&scm_device_idr, scm_pdata, &num);
	if (ret < 0)
		goto fail_id;

	od = omap_device_build("omap37XX_scm", num, oh, scm_pdata, sizeof(*scm_pdata), NULL, 0, false);

	if (IS_ERR(od)) {
		dev_warn(&oh->od->pdev.dev,
			"Could not build omap_device for %s\n", oh->name);
		ret = PTR_ERR(od);
	}

fail_id:
	kfree(scm_pdata);

	return ret;
}

static int __init
omap3703_devinit_scm(void)
{
	return omap_hwmod_for_each_by_class("bandgap", scm_dev_init, NULL);
}

arch_initcall(omap3703_devinit_scm);

