/*
 * twl4030_madc.h - Header for TWL4030 MADC
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * J Keerthy <j-keerthy@ti.com>
 *
 * Copyright (C) 2013 micronet ltd. - http://www.micronet.co.il/
 * Vladimir Zatulovsky <vladimirz@micronet.co.il>
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

#ifndef _TWL4030_MADC_H
#define _TWL4030_MADC_H

#if defined (CONFIG_MACH_A300)
#define TPS65930 1
#endif

struct twl4030_madc_conversion_method {
	u8 sel;
	u8 avg;
	u8 rbase;
	u8 ctrl;
};
#if defined (TPS65930)
// in reality the MADC of TPS65930 has 11 channels, they arn't sequential
// TODO: place them in the table and handle separatly
#define TWL4030_MADC_MAX_CHANNELS 16
#else
#define TWL4030_MADC_MAX_CHANNELS 16
#endif

/*
 * twl4030_madc_request- madc request packet for channel conversion
 * @channels:	16 bit bitmap for individual channels
 * @do_avgP:	sample the input channel for 4 consecutive cycles
 * @method:	RT, SW1, SW2
 * @type:	Polling or interrupt based method
 */

struct twl4030_madc_request {
	unsigned long channels;
	u16 do_avg;
	u16 method;
	u16 type;
	bool active;
	bool result_pending;
	int rbuf[TWL4030_MADC_MAX_CHANNELS << 1];
	void (*func_cb)(int len, int channels, int *buf);
};

enum conversion_methods {
	TWL4030_MADC_RT,
	TWL4030_MADC_SW1,
	TWL4030_MADC_SW2,
	TWL4030_MADC_NUM_METHODS
};

enum sample_type {
	TWL4030_MADC_WAIT,
	TWL4030_MADC_IRQ_ONESHOT,
	TWL4030_MADC_IRQ_REARM
};

/* Step size and pre-scaler ratio */
#define TEMP_STEP_SIZE          147
#define TEMP_PSR_R              100
#define CURR_STEP_SIZE			147
#define CURR_PSR_R1				44
#define CURR_PSR_R2				88

struct twl4030_madc_user_parms {
	int channel;
	int average;
	int status;
	u16 result;
};

int twl4030_madc_conversion(struct twl4030_madc_request *conv);
int twl4030_get_madc_conversion(int channel_no);
int thermistor_conversion(void);
int analog_in_conversion(void);
int ls_conversion(void);
#endif
 
