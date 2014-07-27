/*
 * bq24172.h
 *
 *  Created on: Jul 14, 2013
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

#ifndef BQ24172_H_
#define BQ24172_H_
#include <linux/power_supply.h>

#define BQ24172_ONLINE_MASK		(7 << 29)
#define BQ24172_ONLINE 			(1 << 31)
#define BQ24172_IGN_ONLINE 		(1 << 30)
#define BQ24172_CRDL_ONLINE 	(1 << 29)
#define BQ24172_STATUS_MASK		(31 << 24)
#define BQ24172_FULL			(1 << 28)
#define BQ24172_CHARGING		(1 << 27)
#define BQ24172_DISCHARGING		(1 << 26)
#define BQ24172_NOT_CHARGING	(1 << 25)

#define BQ24172_HELTH_MASK		(255 < 16)
#define BQ24172_GOOD			(1 << 23)

#define BQ24172_ERROR_MASK		(255)
#define BQ24172_OVERHEAT		(1 << 7)
#define BQ24172_OVERVOLTAGE		(1 << 6)
#define BQ24172_UNDERVOLTAGE	(1 << 5)
#define BQ24172_DEAD			(1 << 4)
#define BQ24172_BAT_ABS			(1 << 1)
#define BQ24172_UNSPEC_FAILURE	(1 << 0)

struct bq24172_platform_data {
	int charge_status_in;
	// high - charge is complete,
	// low charge in progress,
	// blink at 0.5Hz in following cases:
	// fault occurs, including charge suspend, input over-voltage, timer fault and battery absent.
	//
	int charge_status_lvl;
	int pwr_too_low_in;
	int pwr_too_low_lvl;
	int pwr_too_high_in;
	int pwr_too_high_lvl;
	int v5_good_in;
	int v5_good_lvl;
};

struct power_supply_info_i {
	struct device 		*dev;
	struct power_supply	power_supply_ac;
	struct mutex 		work_lock;
	struct delayed_work charge_status_work;
	struct delayed_work pwr_low_work;
	struct delayed_work pwr_high_work;
	struct delayed_work v5_good_work;
	int 				charge_status_irq;
	int 				charge_status_in;
	int					charge_status_lvl;
	int 				pwr_too_low_irq;
	int 				pwr_too_low_in;
	int 				pwr_too_low_lvl;
	int 				pwr_too_high_irq;
	int 				pwr_too_high_in;
	int 				pwr_too_high_lvl;
	int 				v5_good_irq;
	int 				v5_good_in;
	int 				v5_good_lvl;
	int					irq_triger;
	int					prev_charge_status;
	signed long long	prev_status_time;
	int					status_count;
	int					total_faults;
	int 				ac_status;
};

#endif /* BQ24172_H_ */
