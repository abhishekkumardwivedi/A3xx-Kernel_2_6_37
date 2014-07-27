/*
 *  Copyright (C) 2013, Ran Meyerstein <ranm@micronet.co.il>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __AUTOMOTIVEIO_H__
#define __AUTOMOTIVEIO_H__

#include <linux/types.h>


enum Automotive_event_nofitiers {
	AUTOMOTIVE_EVT_NOTIFY_CRADLE_OFF,
	AUTOMOTIVE_EVT_NOTIFY_CRADLE_ON,
	AUTOMOTIVE_EVT_NOTIFY_WALL_OFF,
	AUTOMOTIVE_EVT_NOTIFY_WALL_ON,
	AUTOMOTIVE_EVT_NOTIFY_IN2_OFF,
	AUTOMOTIVE_EVT_NOTIFY_IN2_ON,
};

/**
 * struct automotiveio_platform_data - platform_data for 
 * gpio_charger devices 
 * @name: Name for automotive io device
 * @gpio_in: GPIO which is used for automotive input
 * @gpio_in_active_low: Should be set to 1 if the GPIO is active
 *  				  low otherwise 0
 * @gpio_out: GPIO which is used for automotive output
 */
struct automotiveio_platform_data {
        const char *name;

	    /* Board Version to differentiate between M307i and M307/A300 */
		int boardversion;
		int modemtype; //M307i Internal modem exists. 0 - exists, 1 - doesn't exist

        int gpio_in_1; //307i and 307
        int gpio_in_2; //307i only
        int gpio_in_active_low_1; //307i and 307
        int gpio_in_active_low_2; //307i only
        int gpio_cradle; //307i only
        int gpio_wall;   //307i only
        int gpio_out_1;  //307i only
        int gpio_out_2;  //307i only
        int gpio_out;    //307 only
		int gpio_audswitch; //307i only
		int gpio_modem_on_off; //307i only
		int gpio_modem_uncon_shutdown; //307i only
		int gpio_modem_usb_en; //307i only Revision B only
		int gpio_modem_usb_en_pol;
		int gpio_modem_power_ctl;
};


extern int automotiveio_get_input_state(void);
extern int automotiveio_get_input2_state(void);
extern int automotiveio_get_cradle_state(void);
extern int automotiveio_get_wall_state(void);
extern int automotiveevents_register_notifier(struct notifier_block *);

#endif

