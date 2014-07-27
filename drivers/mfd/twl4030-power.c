/*
 * linux/drivers/i2c/chips/twl4030-power.c
 *
 * Handle TWL4030 Power initialization
 *
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2006 Texas Instruments, Inc
 *
 * Written by 	Kalle Jokiniemi
 *		Peter De Schrijver <peter.de-schrijver@nokia.com>
 * Several fixes by Amit Kucheria <amit.kucheria@verdurent.com>
 *
  * Copyright (c) 2013, Micronet Ltd.
 * Vladimir Zatulovsky <vladimirz@micronet.co.il>
 * Power on source only during wake
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

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>

#include <plat/smartreflex.h>

static u8 twl4030_start_script_address = 0x2b;

#define PWR_P1_SW_EVENTS	0x10
#define PWR_DEVOFF	(1<<0)

#define PHY_TO_OFF_PM_MASTER(p)		(p - 0x36)
#define PHY_TO_OFF_PM_RECEIVER(p)	(p - 0x5b)

/* resource - hfclk */
#define R_HFCLKOUT_DEV_GRP 	PHY_TO_OFF_PM_RECEIVER(0xe6)

/* PM events */
#define R_P1_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x46)
#define R_P2_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x47)
#define R_P3_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x48)
#define R_CFG_P1_TRANSITION	PHY_TO_OFF_PM_MASTER(0x36)
#define R_CFG_P2_TRANSITION	PHY_TO_OFF_PM_MASTER(0x37)
#define R_CFG_P3_TRANSITION	PHY_TO_OFF_PM_MASTER(0x38)
#define R_WATCHDOG_CFG		PHY_TO_OFF_PM_RECEIVER(0x5E)

#define LVL_WAKEUP	0x08

#define ENABLE_WARMRESET (1<<4)

#define END_OF_SCRIPT		0x3f

#define R_SEQ_ADD_A2S		PHY_TO_OFF_PM_MASTER(0x55)
#define R_SEQ_ADD_S2A12		PHY_TO_OFF_PM_MASTER(0x56)
#define	R_SEQ_ADD_S2A3		PHY_TO_OFF_PM_MASTER(0x57)
#define	R_SEQ_ADD_WARM		PHY_TO_OFF_PM_MASTER(0x58)
#define R_MEMORY_ADDRESS	PHY_TO_OFF_PM_MASTER(0x59)
#define R_MEMORY_DATA		PHY_TO_OFF_PM_MASTER(0x5a)

/* Smartreflex Control */
#define R_DCDC_GLOBAL_CFG     PHY_TO_OFF_PM_RECEIVER(0x61)
#define CFG_ENABLE_SRFLX      0x08

#define R_VDD1_OSC		0x5C
#define R_VDD2_OSC		0x6A
#define R_VIO_OSC		0x52
#define EXT_FS_CLK_EN		BIT(6)

#define R_WDT_CFG		0x03
#define WDT_WRK_TIMEOUT		0x03

/* resource configuration registers
   <RESOURCE>_DEV_GRP   at address 'n+0'
   <RESOURCE>_TYPE      at address 'n+1'
   <RESOURCE>_REMAP     at address 'n+2'
   <RESOURCE>_DEDICATED at address 'n+3'
*/
#define DEV_GRP_OFFSET		0
#define TYPE_OFFSET		1
#define REMAP_OFFSET		2
#define DEDICATED_OFFSET	3

/* Bit positions in the registers */

/* <RESOURCE>_DEV_GRP */
#define DEV_GRP_SHIFT		5
#define DEV_GRP_MASK		(7 << DEV_GRP_SHIFT)

/* <RESOURCE>_TYPE */
#define TYPE_SHIFT		0
#define TYPE_MASK		(7 << TYPE_SHIFT)
#define TYPE2_SHIFT		3
#define TYPE2_MASK		(3 << TYPE2_SHIFT)

/* <RESOURCE>_REMAP */
#define SLEEP_STATE_SHIFT	0
#define SLEEP_STATE_MASK	(0xf << SLEEP_STATE_SHIFT)
#define OFF_STATE_SHIFT		4
#define OFF_STATE_MASK		(0xf << OFF_STATE_SHIFT)

static u8 res_config_addrs[] = {
	[RES_VAUX1]	= 0x17,
	[RES_VAUX2]	= 0x1b,
	[RES_VAUX3]	= 0x1f,
	[RES_VAUX4]	= 0x23,
	[RES_VMMC1]	= 0x27,
	[RES_VMMC2]	= 0x2b,
	[RES_VPLL1]	= 0x2f,
	[RES_VPLL2]	= 0x33,
	[RES_VSIM]	= 0x37,
	[RES_VDAC]	= 0x3b,
	[RES_VINTANA1]	= 0x3f,
	[RES_VINTANA2]	= 0x43,
	[RES_VINTDIG]	= 0x47,
	[RES_VIO]	= 0x4b,
	[RES_VDD1]	= 0x55,
	[RES_VDD2]	= 0x63,
	[RES_VUSB_1V5]	= 0x71,
	[RES_VUSB_1V8]	= 0x74,
	[RES_VUSB_3V1]	= 0x77,
	[RES_VUSBCP]	= 0x7a,
	[RES_REGEN]	= 0x7f,
	[RES_NRES_PWRON] = 0x82,
	[RES_CLKEN]	= 0x85,
	[RES_SYSEN]	= 0x88,
	[RES_HFCLKOUT]	= 0x8b,
	[RES_32KCLKOUT]	= 0x8e,
	[RES_RESET]	= 0x91,
	[RES_MAIN_REF]	= 0x94,
};

static int twl4030_write_script_byte(u8 address, u8 byte)
{
	int err;

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				R_MEMORY_ADDRESS);
	if (err)
		goto out;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, byte,
				R_MEMORY_DATA);
out:
	return err;
}

static int twl4030_write_script_ins(u8 address, u16 pmb_message,
					   u8 delay, u8 next)
{
	int err;

	address *= 4;
	err = twl4030_write_script_byte(address++, pmb_message >> 8);
	if (err)
		goto out;
	err = twl4030_write_script_byte(address++, pmb_message & 0xff);
	if (err)
		goto out;
	err = twl4030_write_script_byte(address++, delay);
	if (err)
		goto out;
	err = twl4030_write_script_byte(address++, next);
out:
	return err;
}

static int twl4030_write_script(u8 address, struct twl4030_ins *script,
				       int len)
{
	int err;

	for (; len; len--, address++, script++) {
		if (len == 1) {
			err = twl4030_write_script_ins(address,
						script->pmb_message,
						script->delay,
						END_OF_SCRIPT);
			if (err)
				break;
		} else {
			err = twl4030_write_script_ins(address,
						script->pmb_message,
						script->delay,
						address + 1);
			if (err)
				break;
		}
	}
	return err;
}

static int twl4030_config_wakeup3_sequence(u8 address)
{
	int err;
	u8 data;

	/* Set SLEEP to ACTIVE SEQ address for P3 */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				R_SEQ_ADD_S2A3);
	if (err)
		goto out;

	/* P3 LVL_WAKEUP should be on LEVEL */
	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &data,
				R_P3_SW_EVENTS);
	if (err)
		goto out;
	data |= LVL_WAKEUP;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, data,
				R_P3_SW_EVENTS);
out:
	if (err)
		pr_err("TWL4030 wakeup sequence for P3 config error\n");
	return err;
}

static int twl4030_config_wakeup12_sequence(u8 address)
{
	int err = 0;
	u8 data;

	/* Set SLEEP to ACTIVE SEQ address for P1 and P2 */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				R_SEQ_ADD_S2A12);
	if (err)
		goto out;

	/* P1/P2 LVL_WAKEUP should be on LEVEL */
	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &data,
				R_P1_SW_EVENTS);
	if (err)
		goto out;

	data |= LVL_WAKEUP;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, data,
				R_P1_SW_EVENTS);
	if (err)
		goto out;

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &data,
				R_P2_SW_EVENTS);
	if (err)
		goto out;

	data |= LVL_WAKEUP;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, data,
				R_P2_SW_EVENTS);
	if (err)
		goto out;

	if (machine_is_omap_3430sdp() || machine_is_omap_ldp()) {
		/* Disabling AC charger effect on sleep-active transitions */
		err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &data,
					R_CFG_P1_TRANSITION);
		if (err)
			goto out;
		data &= ~(1<<1);
		err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, data ,
					R_CFG_P1_TRANSITION);
		if (err)
			goto out;
	}

out:
	if (err)
		pr_err("TWL4030 wakeup sequence for P1 and P2" \
			"config error\n");
	return err;
}

static int twl4030_config_sleep_sequence(u8 address)
{
	int err;

	/* Set ACTIVE to SLEEP SEQ address in T2 memory*/
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				R_SEQ_ADD_A2S);

	if (err)
		pr_err("TWL4030 sleep sequence config error\n");

	return err;
}

static int twl4030_config_warmreset_sequence(u8 address)
{
	int err;
	u8 rd_data;

	/* Set WARM RESET SEQ address for P1 */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				R_SEQ_ADD_WARM);
	if (err)
		goto out;

	/* P1/P2/P3 enable WARMRESET */
	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
				R_P1_SW_EVENTS);
	if (err)
		goto out;

	rd_data |= ENABLE_WARMRESET;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
				R_P1_SW_EVENTS);
	if (err)
		goto out;

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
				R_P2_SW_EVENTS);
	if (err)
		goto out;

	rd_data |= ENABLE_WARMRESET;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
				R_P2_SW_EVENTS);
	if (err)
		goto out;

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
				R_P3_SW_EVENTS);
	if (err)
		goto out;

	rd_data |= ENABLE_WARMRESET;
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
				R_P3_SW_EVENTS);
out:
	if (err)
		pr_err("TWL4030 warmreset seq config error\n");
	return err;
}

static int twl4030_configure_resource(struct twl4030_resconfig *rconfig)
{
	int rconfig_addr;
	int err;
	u8 type, type_value;
	u8 grp, grp_value;
	u8 remap, remap_value;

	if (rconfig->resource > TOTAL_RESOURCES) {
		pr_err("TWL4030 Resource %d does not exist\n",
			rconfig->resource);
		return -EINVAL;
	}

	rconfig_addr = res_config_addrs[rconfig->resource];

	/* Set resource group */
	if (rconfig->devgroup != TWL4030_RESCONFIG_UNDEF) {
		err = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &grp,
			      rconfig_addr + DEV_GRP_OFFSET);
		if (err) {
			pr_err("TWL4030 Resource %d group could not be read\n",
				rconfig->resource);
			return err;
		}

		grp_value = (grp & DEV_GRP_MASK) >> DEV_GRP_SHIFT;

		if (rconfig->devgroup != grp_value) {
			grp &= ~DEV_GRP_MASK;
			grp |= rconfig->devgroup << DEV_GRP_SHIFT;
			err = twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				       grp, rconfig_addr + DEV_GRP_OFFSET);
			if (err < 0) {
				pr_err("TWL4030 failed to program devgroup\n");
				return err;
			}
		}
	}

	/* Set resource types */
	if ((rconfig->type != TWL4030_RESCONFIG_UNDEF) ||
		(rconfig->type2 != TWL4030_RESCONFIG_UNDEF)) {

		err = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &type,
				rconfig_addr + TYPE_OFFSET);
		if (err < 0) {
			pr_err("TWL4030 Resource %d type could not be read\n",
				rconfig->resource);
			return err;
		}

		type_value = type;

		if (rconfig->type != TWL4030_RESCONFIG_UNDEF) {
			type &= ~TYPE_MASK;
			type |= rconfig->type << TYPE_SHIFT;
		}

		if (rconfig->type2 != TWL4030_RESCONFIG_UNDEF) {
			type &= ~TYPE2_MASK;
			type |= rconfig->type2 << TYPE2_SHIFT;
		}

		if (type != type_value) {
			err = twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				type, rconfig_addr + TYPE_OFFSET);
			if (err < 0) {
				pr_err("TWL4030 failed to program resource type\n");
				return err;
			}
		}
	}

	/* Set remap states */
	if ((rconfig->remap_off != TWL4030_RESCONFIG_UNDEF) ||
		(rconfig->remap_sleep != TWL4030_RESCONFIG_UNDEF)) {
		err = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &remap,
			      rconfig_addr + REMAP_OFFSET);
		if (err < 0) {
			pr_err("TWL4030 Resource %d remap could not be read\n",
				rconfig->resource);
			return err;
		}

		remap_value = remap;

		if (rconfig->remap_off != TWL4030_RESCONFIG_UNDEF) {
			remap &= ~OFF_STATE_MASK;
			remap |= rconfig->remap_off << OFF_STATE_SHIFT;
		}

		if (rconfig->remap_sleep != TWL4030_RESCONFIG_UNDEF) {
			remap &= ~SLEEP_STATE_MASK;
			remap |= rconfig->remap_sleep << SLEEP_STATE_SHIFT;
		}

		if (remap != remap_value) {
			err = twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			       remap, rconfig_addr + REMAP_OFFSET);
			if (err < 0) {
				pr_err("TWL4030 failed to program remap\n");
				return err;
			}
		}
	}

	return 0;
}

static int load_twl4030_script(struct twl4030_script *tscript,
	       u8 address)
{
	int err;
	static int order;

	/* Make sure the script isn't going beyond last valid address (0x3f) */
	if ((address + tscript->size) > END_OF_SCRIPT) {
		pr_err("TWL4030 scripts too big error\n");
		return -EINVAL;
	}

	err = twl4030_write_script(address, tscript->script, tscript->size);
	if (err)
		goto out;

	if (tscript->flags & TWL4030_WRST_SCRIPT) {
		err = twl4030_config_warmreset_sequence(address);
		if (err)
			goto out;
	}
	if (tscript->flags & TWL4030_WAKEUP12_SCRIPT) {
		err = twl4030_config_wakeup12_sequence(address);
		if (err)
			goto out;
		order = 1;
	}
	if (tscript->flags & TWL4030_WAKEUP3_SCRIPT) {
		err = twl4030_config_wakeup3_sequence(address);
		if (err)
			goto out;
	}
	if (tscript->flags & TWL4030_SLEEP_SCRIPT) {
		if (!order)
			pr_warning("TWL4030: Bad order of scripts (sleep "\
					"script before wakeup) Leads to boot"\
					"failure on some boards\n");
		err = twl4030_config_sleep_sequence(address);
	}
out:
	return err;
}

int twl4030_remove_script(u8 flags)
{
	int err = 0;

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,
			TWL4030_PM_MASTER_KEY_CFG1,
			TWL4030_PM_MASTER_PROTECT_KEY);
	if (err) {
		pr_err("twl4030: unable to unlock PROTECT_KEY\n");
		return err;
	}

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,
			TWL4030_PM_MASTER_KEY_CFG2,
			TWL4030_PM_MASTER_PROTECT_KEY);
	if (err) {
		pr_err("twl4030: unable to unlock PROTECT_KEY\n");
		return err;
	}

	if (flags & TWL4030_WRST_SCRIPT) {
		err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, END_OF_SCRIPT,
				R_SEQ_ADD_WARM);
		if (err)
			return err;
	}
	if (flags & TWL4030_WAKEUP12_SCRIPT) {
		err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, END_OF_SCRIPT,
				R_SEQ_ADD_S2A12);
		if (err)
			return err;
	}
	if (flags & TWL4030_WAKEUP3_SCRIPT) {
		err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, END_OF_SCRIPT,
				R_SEQ_ADD_S2A3);
		if (err)
			return err;
	}
	if (flags & TWL4030_SLEEP_SCRIPT) {
		err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, END_OF_SCRIPT,
				R_SEQ_ADD_A2S);
		if (err)
			return err;
	}

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0,
			TWL4030_PM_MASTER_PROTECT_KEY);
	if (err)
		pr_err("TWL4030 Unable to relock registers\n");

	return err;
}

/* API to enable smrtreflex on Triton side */
static void twl4030_smartreflex_init(void)
{
	int ret = 0;
	u8 read_val;

	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &read_val,
			R_DCDC_GLOBAL_CFG);
	read_val |= CFG_ENABLE_SRFLX;
	ret |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, read_val,
			R_DCDC_GLOBAL_CFG);
}

struct omap_sr_pmic_data twl4030_sr_data = {
	.sr_pmic_init   = twl4030_smartreflex_init,
};

void __init twl4030_power_sr_init()
{
	/* Register the SR init API with the Smartreflex driver */
	omap_sr_register_pmic(&twl4030_sr_data);
}
EXPORT_SYMBOL_GPL(twl4030_remove_script);

/**
 * twl_dcdc_use_hfclk - API to use HFCLK for TWL DCDCs
 *
 * TWL DCDCs switching to HFCLK instead of using internal RC oscillator.
 */
static int twl_dcdc_use_hfclk(void)
{
	u8 val;
	u8 smps_osc_reg[] = {R_VDD1_OSC, R_VDD2_OSC, R_VIO_OSC};
	int i;
	int err;

	for (i = 0; i < sizeof(smps_osc_reg); i++) {
		err = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &val,
							smps_osc_reg[i]);
		val |= EXT_FS_CLK_EN;
		err |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, val,
							smps_osc_reg[i]);
	}
	return err;
}

/**
 * twl_erratum27_workaround - Workaround for TWL5030 Silicon Erratum 27
 * 27 - VDD1, VDD2, may have glitches when their output value is updated.
 * 28 - VDD1 and / or VDD2 DCDC clock may stop working when internal clock is
 * switched from internal to external.
 *
 * Workaround requires the TWL DCDCs to use HFCLK instead of
 * internal oscillator. Also enable TWL watchdog before switching the osc
 * to recover if the VDD1/VDD2 stop working.
 */
static void twl_erratum27_workaround(void)
{
	u8 wdt_counter_val = 0;
	int err;

	/* Setup the twl wdt to take care of borderline failure case */
	err = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &wdt_counter_val,
			R_WDT_CFG);
	err |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, WDT_WRK_TIMEOUT,
			R_WDT_CFG);

	/* TWL DCDC switching to HFCLK */
	err |= twl_dcdc_use_hfclk();

	/* restore the original value */
	err |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, wdt_counter_val,
			R_WDT_CFG);
	if (err)
		pr_warning("TWL4030: workaround setup failed!\n");
}

static bool is_twl5030_erratum27wa_required(void)
{
	if (twl_get_type() == TWL_SIL_5030)
		return (twl_get_version() < TWL5030_REV_1_2);

	return 0;
}

/**
 * PMIC initialization specific for the OMAP3EVM
 */
static int twl4030_omap3evm_init(void)
{
	int err = 0;

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0A,
					TWL4030_PM_MASTER_CFG_BOOT);

	if (unlikely(err))
		pr_err("err: CFG_BOOT\n");

	/* PWR_EDR1 */
	err = twl_i2c_write_u8(TWL4030_MODULE_INT, 0x00,
					TWL4030_INT_PWR_EDR1);

	if (unlikely(err))
		pr_err("err: PWR_EDR1\n");

	/* PWR_EDR2 */
	err = twl_i2c_write_u8(TWL4030_MODULE_INT, 0x00,
					TWL4030_INT_PWR_EDR2);

	if (unlikely(err))
		pr_err("err: PWR_EDR2\n");

	/* PWR_SIH_CTRL */
	err = twl_i2c_write_u8(TWL4030_MODULE_INT, 0x05,
					TWL4030_INT_PWR_SIH_CTRL);

	if (unlikely(err))
		pr_err("err: PWR_EDR2\n");

	/* CFG_P1_TRANSITION */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x00,
					TWL4030_PM_MASTER_CFG_P1_TRANSITION);

	if (unlikely(err))
		pr_err("err: CFG_P1_TRANSITION\n");

	/* CFG_P2_TRANSITION */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x00,
					TWL4030_PM_MASTER_CFG_P2_TRANSITION);

	if (unlikely(err))
		pr_err("err: CFG_P2_TRANSITION\n");

	/* CFG_P3_TRANSITION */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x00,
					TWL4030_PM_MASTER_CFG_P3_TRANSITION);

	if (unlikely(err))
		pr_err("err: CFG_P3_TRANSITION\n");

	return err;
}

int twl4030_wd_enable(int sec)
{
	return twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, sec, R_WATCHDOG_CFG);
}
EXPORT_SYMBOL_GPL(twl4030_wd_enable);

int twl4030_power_init(struct twl4030_power_data *twl4030_scripts)
{
	int err = 0;
	int i;
	struct twl4030_resconfig *resconfig;
	u8 address = twl4030_start_script_address;

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,
			TWL4030_PM_MASTER_KEY_CFG1,
			TWL4030_PM_MASTER_PROTECT_KEY);
	if (err)
		goto unlock;

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,
			TWL4030_PM_MASTER_KEY_CFG2,
			TWL4030_PM_MASTER_PROTECT_KEY);
	if (err)
		goto unlock;
#if defined (CONFIG_MACH_A300) || defined (CONFIG_MACH_A317)
	// reconfigure HW states transition
	printk("%s: reconfigure HW states transition WAIT_ON -> ACTIVE by PWRON and RTC events only\n", __func__);
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x49, R_CFG_P1_TRANSITION);
	if (unlikely(err))
		pr_err("err: CFG_P1_TRANSITION\n");

	/* CFG_P2_TRANSITION */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x49, R_CFG_P2_TRANSITION);
	if (unlikely(err))
		pr_err("err: CFG_P2_TRANSITION\n");

	/* CFG_P3_TRANSITION */
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x49, R_CFG_P3_TRANSITION);
	if (unlikely(err))
		pr_err("err: CFG_P3_TRANSITION\n");
#else
	/* Applying TWL5030 Erratum 27 WA based on Si revision &
	 * flag updated from board file*/
	if (is_twl5030_erratum27wa_required()) {
		pr_info("TWL5030: Enabling workaround for Si Erratum 27\n");
		twl_erratum27_workaround();
		if (twl4030_scripts->twl5030_erratum27wa_script)
			twl4030_scripts->twl5030_erratum27wa_script();
	}

	for (i = 0; i < twl4030_scripts->num; i++) {
		err = load_twl4030_script(twl4030_scripts->scripts[i], address);
		if (err)
			goto load;
		address += twl4030_scripts->scripts[i]->size;
	}

	resconfig = twl4030_scripts->resource_config;
	if (resconfig) {
		while (resconfig->resource) {
			err = twl4030_configure_resource(resconfig);
			if (err)
				goto resource;
			resconfig++;

		}
	}
	/*
	 * TODO: Workaround until we get better way to identify that
	 *       we are running on OMAP3EVM. May not be necessary - but
	 *       being prudent to ensure we don't break any other board.
	 */
	if (machine_is_omap3evm()) {
		err = twl4030_omap3evm_init() ;
		if (err)
			goto unlock;
	}
#endif
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0,
			TWL4030_PM_MASTER_PROTECT_KEY);
	if (err)
		pr_err("TWL4030 Unable to relock registers\n");
	return err;

unlock:
	if (err)
		pr_err("TWL4030 Unable to unlock registers\n");
	return err;
load:
	if (err)
		pr_err("TWL4030 failed to load scripts\n");
	return err;
resource:
	if (err)
		pr_err("TWL4030 failed to configure resource\n");
	return err;
}
EXPORT_SYMBOL_GPL(twl4030_power_init);
