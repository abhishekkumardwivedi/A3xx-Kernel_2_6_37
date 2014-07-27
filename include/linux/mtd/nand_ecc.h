/*
 *  drivers/mtd/nand_ecc.h
 *
 *  Copyright (C) 2000-2010 Steven J. Hill <sjhill@realitydiluted.com>
 *			    David Woodhouse <dwmw2@infradead.org>
 *			    Thomas Gleixner <tglx@linutronix.de>
 *
 * (C) Copyright 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * Micron on die ECC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file is the header for the ECC algorithm.
 */

#ifndef __MTD_NAND_ECC_H__
#define __MTD_NAND_ECC_H__

#define CONFIG_MTD_NAND_ONDIE_ECC 1
#if defined (CONFIG_MTD_NAND_ONDIE_ECC)
	#define MICRON_ECC_ONDIE 0xFF
#endif
struct mtd_info;

/*
 * Calculate 3 byte ECC code for eccsize byte block
 */
void __nand_calculate_ecc(const u_char *dat, unsigned int eccsize, u_char *ecc_code);

/*
 * Calculate 3 byte ECC code for 256/512 byte block
 */
int nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code);

/*
 * Detect and correct a 1 bit error for eccsize byte block
 */
int __nand_correct_data(u_char *dat, u_char *read_ecc, u_char *calc_ecc, unsigned int eccsize);

/*
 * Detect and correct a 1 bit error for 256/512 byte block
 */
int nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc);

#endif /* __MTD_NAND_ECC_H__ */
