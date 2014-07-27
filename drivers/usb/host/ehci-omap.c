/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Copyright (C) 2009 Nokia Corporation
 *	Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * TODO (last updated Feb 12, 2010):
 *	- add kernel-doc
 *	- enable AUTOIDLE
 *	- add suspend/resume
 *	- move workarounds to board-files
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/ulpi.h>
#include <plat/usb.h>


#include <plat/clock.h>

#include "../../../arch/arm/mach-omap2/clock.h"
#include "../../../arch/arm/mach-omap2/prcm-common.h"
#include "../../../arch/arm/mach-omap2/cm2xxx_3xxx.h"

#include "../../../arch/arm/mach-omap2/prm-regbits-24xx.h"
#include "../../../arch/arm/mach-omap2/prm.h"

struct usb_hcd *ghcd;

/*
 * OMAP USBHOST Register addresses: VIRTUAL ADDRESSES
 *	Use ehci_omap_readl()/ehci_omap_writel() functions
 */

/* TLL Register Set */
#define	OMAP_USBTLL_REVISION				(0x00)
#define	OMAP_USBTLL_SYSCONFIG				(0x10)
#define	OMAP_USBTLL_SYSCONFIG_CACTIVITY			(1 << 8)
#define	OMAP_USBTLL_SYSCONFIG_SMARTIDLE		(2 << 3)
#define	OMAP_USBTLL_SYSCONFIG_NOIDLE			(1 << 3)
#define	OMAP_USBTLL_SYSCONFIG_FORCEIDLE		(0 << 3)
#define	OMAP_USBTLL_SYSCONFIG_SIDLEMASK			(3 << 3)
#define	OMAP_USBTLL_SYSCONFIG_ENAWAKEUP			(1 << 2)
#define	OMAP_USBTLL_SYSCONFIG_SOFTRESET			(1 << 1)
#define	OMAP_USBTLL_SYSCONFIG_AUTOIDLE			(1 << 0)

#define	OMAP_USBTLL_SYSSTATUS				(0x14)
#define	OMAP_USBTLL_SYSSTATUS_RESETDONE			(1 << 0)

#define	OMAP_USBTLL_IRQSTATUS				(0x18)
#define	OMAP_USBTLL_IRQENABLE				(0x1C)

#define	OMAP_TLL_SHARED_CONF				(0x30)
#define	OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN		(1 << 6)
#define	OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN		(1 << 5)
#define	OMAP_TLL_SHARED_CONF_USB_DIVRATION		(1 << 2)
#define	OMAP_TLL_SHARED_CONF_FCLK_REQ			(1 << 1)
#define	OMAP_TLL_SHARED_CONF_FCLK_IS_ON			(1 << 0)

#define	OMAP_TLL_CHANNEL_CONF(num)			(0x040 + 0x004 * num)
#define	OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF		(1 << 11)
#define	OMAP_TLL_CHANNEL_CONF_ULPI_ULPIAUTOIDLE		(1 << 10)
#define	OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE		(1 << 9)
#define	OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE		(1 << 8)
#define	OMAP_TLL_CHANNEL_CONF_CHANEN			(1 << 0)

#define	OMAP_TLL_ULPI_FUNCTION_CTRL(num)		(0x804 + 0x100 * num)
#define	OMAP_TLL_ULPI_INTERFACE_CTRL(num)		(0x807 + 0x100 * num)
#define	OMAP_TLL_ULPI_OTG_CTRL(num)			(0x80A + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_EN_RISE(num)			(0x80D + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_EN_FALL(num)			(0x810 + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_STATUS(num)			(0x813 + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_LATCH(num)			(0x814 + 0x100 * num)
#define	OMAP_TLL_ULPI_DEBUG(num)			(0x815 + 0x100 * num)
#define	OMAP_TLL_ULPI_SCRATCH_REGISTER(num)		(0x816 + 0x100 * num)

#define OMAP_TLL_CHANNEL_COUNT				3
#define OMAP_TLL_CHANNEL_1_EN_MASK			(1 << 1)
#define OMAP_TLL_CHANNEL_2_EN_MASK			(1 << 2)
#define OMAP_TLL_CHANNEL_3_EN_MASK			(1 << 4)

/* UHH Register Set */
#define	OMAP_UHH_REVISION				(0x00)
#define	OMAP_UHH_SYSCONFIG				(0x10)
#define	OMAP_UHH_SYSCONFIG_SMARTSTDBY			(2 << 12)
#define	OMAP_UHH_SYSCONFIG_NOSTDBY			(1 << 12)
#define	OMAP_UHH_SYSCONFIG_FORCESTDBY			(0 << 12)
#define	OMAP_UHH_SYSCONFIG_MIDLEMASK			(3 << 12)
#define	OMAP_UHH_SYSCONFIG_CACTIVITY			(1 << 8)
#define	OMAP_UHH_SYSCONFIG_SMARTIDLE			(2 << 3)
#define	OMAP_UHH_SYSCONFIG_NOIDLE			(1 << 3)
#define	OMAP_UHH_SYSCONFIG_FORCEIDLE			(0 << 3)
#define	OMAP_UHH_SYSCONFIG_SIDLEMASK			(3 << 3)
#define	OMAP_UHH_SYSCONFIG_ENAWAKEUP			(1 << 2)
#define	OMAP_UHH_SYSCONFIG_SOFTRESET			(1 << 1)
#define	OMAP_UHH_SYSCONFIG_AUTOIDLE			(1 << 0)

#define	OMAP_UHH_SYSSTATUS				(0x14)
#define	OMAP_UHH_HOSTCONFIG				(0x40)
#define	OMAP_UHH_HOSTCONFIG_ULPI_BYPASS			(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS		(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS		(1 << 11)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS		(1 << 12)
#define OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN		(1 << 2)
#define OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN		(1 << 3)
#define OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN		(1 << 4)
#define OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN		(1 << 5)
#define OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS		(1 << 8)
#define OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS		(1 << 9)
#define OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS		(1 << 10)

#define	OMAP_UHH_DEBUG_CSR				(0x44)

/* EHCI Register Set */
#define EHCI_INSNREG04					(0xA0)
#define EHCI_INSNREG04_DISABLE_UNSUSPEND		(1 << 5)
#define	EHCI_INSNREG05_ULPI				(0xA4)
#define	EHCI_INSNREG05_ULPI_CONTROL_SHIFT		31
#define	EHCI_INSNREG05_ULPI_PORTSEL_SHIFT		24
#define	EHCI_INSNREG05_ULPI_OPSEL_SHIFT			22
#define	EHCI_INSNREG05_ULPI_REGADD_SHIFT		16
#define	EHCI_INSNREG05_ULPI_EXTREGADD_SHIFT		8
#define	EHCI_INSNREG05_ULPI_WRDATA_SHIFT		0

#define EHCI_FRINDEX					(0x1C)
#define EHCI_PORTSC_0					(0x54)
#define EHCI_PORTSC_1					(0x58)
#define EHCI_PORTSC_2					(0x5C)
#define EHCI_PORTSC_SUSPEND_BIT         (1 << 7)          
#define EHCI_PORTSC_FORCE_RESUME_BIT    (1 << 6) 



/*-------------------------------------------------------------------------*/

static inline void ehci_omap_writel(void __iomem *base, u32 reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 ehci_omap_readl(void __iomem *base, u32 reg)
{
	return __raw_readl(base + reg);
}

static inline void ehci_omap_writeb(void __iomem *base, u8 reg, u8 val)
{
	__raw_writeb(val, base + reg);
}

static inline u8 ehci_omap_readb(void __iomem *base, u8 reg)
{
	return __raw_readb(base + reg);
}

/*-------------------------------------------------------------------------*/

struct ehci_hcd_omap {
	struct ehci_hcd		*ehci;
	struct device		*dev;

	struct clk		*usbhost_ick;
	struct clk		*usbhost2_120m_fck;
	struct clk		*usbhost1_48m_fck;
	struct clk		*usbtll_fck;
	struct clk		*usbtll_ick;
	unsigned		suspended:1;

	/* FIXME the following two workarounds are
	 * board specific not silicon-specific so these
	 * should be moved to board-file instead.
	 *
	 * Maybe someone from TI will know better which
	 * board is affected and needs the workarounds
	 * to be applied
	 */

	/* gpio for resetting phy */
	int			reset_gpio_port[OMAP3_HS_USB_PORTS];

	/* phy reset workaround */
	int			phy_reset;

	/* desired phy_mode: TLL, PHY */
	enum ehci_hcd_omap_mode	port_mode[OMAP3_HS_USB_PORTS];

	void __iomem		*uhh_base;
	void __iomem		*tll_base;
	void __iomem		*ehci_base;

	/* Regulators for USB PHYs.
	 * Each PHY can have a separate regulator.
	 */
	struct regulator        *regulator[OMAP3_HS_USB_PORTS];
};

/*-------------------------------------------------------------------------*/


/*workaround for suspend resume*/
static void ehci_hcd_omap_workaround1(struct ehci_hcd_omap *omap, struct usb_hcd *hcd)
{
	u32 frame_index,frame_index1;
	u32 portsc;
	u32 portsc_reg ; 
	int i;


	/**read frindex register***/
	frame_index = ehci_omap_readl(omap->ehci_base, EHCI_FRINDEX);
	frame_index1 = frame_index;

	while (frame_index< frame_index1) {
		frame_index = ehci_omap_readl(omap->ehci_base, EHCI_FRINDEX);
	}

	/***set suspend bit***/

	portsc_reg = EHCI_PORTSC_0 ;

	for (i=0; i<2; i++) {
		portsc = ehci_omap_readl(omap->ehci_base, portsc_reg);
		portsc |= EHCI_PORTSC_SUSPEND_BIT;
		ehci_omap_writel(omap->ehci_base, portsc_reg, portsc);
		portsc += 0x04;
	}

	/***wait 20 msec***/
	mdelay(20);


	/**read frindex register***/
	frame_index = ehci_omap_readl(omap->ehci_base, EHCI_FRINDEX);
	frame_index1 = frame_index;

	while (frame_index< frame_index1) {
		frame_index = ehci_omap_readl(omap->ehci_base, EHCI_FRINDEX);
	}

	/**set forced resume***/ 

	portsc_reg = EHCI_PORTSC_0 ;

	for (i=0; i<2; i++) {
		portsc = ehci_omap_readl(omap->ehci_base, portsc_reg);
		portsc |= EHCI_PORTSC_FORCE_RESUME_BIT;
		ehci_omap_writel(omap->ehci_base, portsc_reg, portsc);
		portsc += 0x04;
	}

	/***wait 20 msec***/
	mdelay(20);


	/**read frindex register***/
	frame_index = ehci_omap_readl(omap->ehci_base, EHCI_FRINDEX);
	frame_index1 = frame_index;

	while (frame_index< frame_index1) {
		frame_index = ehci_omap_readl(omap->ehci_base, EHCI_FRINDEX);
	}

	/**clear forced bit***/ 

	portsc_reg = EHCI_PORTSC_0 ;

	for (i=0; i<2; i++) {
		portsc = ehci_omap_readl(omap->ehci_base, portsc_reg);
		portsc &= 0xffffffaf;
		ehci_omap_writel(omap->ehci_base, portsc_reg, portsc);
		portsc += 0x04;
	}

}


/*-------------------------------------------------------------------------*/
static void ehci_omap_clock_power(struct ehci_hcd_omap *omap, int on)
{
	if (on) {
		clk_enable(omap->usbtll_ick);
		clk_enable(omap->usbtll_fck);
		clk_enable(omap->usbhost_ick);
		clk_enable(omap->usbhost1_48m_fck);
		clk_enable(omap->usbhost2_120m_fck);
	} else {
		clk_disable(omap->usbhost2_120m_fck);
		clk_disable(omap->usbhost1_48m_fck);
		clk_disable(omap->usbhost_ick);
		clk_disable(omap->usbtll_fck);
		clk_disable(omap->usbtll_ick);
	}
}

#ifdef EHCI_OMAP_RECOVERY_DEBUG

static void ehci_omap_get_clock_regs(struct ehci_hcd_omap *omap)
{
	
	u32 val;


	/**core module**/


	val = omap2_cm_read_mod_reg(CORE_MOD, CM_FCLKEN);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_FCLKEN1 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, 0x008);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_FCLKEN2 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_ICLKEN3);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_FCLKEN3 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_ICLKEN1);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_ICLKEN1 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_ICLKEN2);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_ICLKEN2 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_ICLKEN3);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_ICLKEN3 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_IDLEST1);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_IDLEST1 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_IDLEST2);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_IDLEST2 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, 0x0028);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_IDLEST3 reg =0x%x\n", val);


	val = omap2_cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE1);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD CM_AUTOIDLE1 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE2);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD  CM_AUTOIDLE2 reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE3);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD  M_AUTOIDLE3 reg =0x%x\n", val);


	val = omap2_cm_read_mod_reg(CORE_MOD, CM_CLKSEL);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD  CM_CLKSEL reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, OMAP2_CM_CLKSTCTRL);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD  OMAP2_CM_CLKSTCTRL reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(CORE_MOD, OMAP3430_CM_CLKSTST);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD OMAP3430_CM_CLKSTST reg =0x%x\n", val);



	/**WKUP_MOD module**/ 

	val = omap2_cm_read_mod_reg(WKUP_MOD, CM_FCLKEN);

	printk(KERN_INFO "ehci_omap_get_clock_regs WKUP_MOD  CM_FCLKEN reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(WKUP_MOD, CM_ICLKEN);

	printk(KERN_INFO "ehci_omap_get_clock_regs WKUP_MOD  CM_ICLKEN reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(WKUP_MOD, CM_IDLEST);

	printk(KERN_INFO "ehci_omap_get_clock_regs WKUP_MOD CM_IDLEST  reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(WKUP_MOD, CM_AUTOIDLE);

	printk(KERN_INFO "ehci_omap_get_clock_regs WKUP_MOD  CM_AUTOIDLE reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(WKUP_MOD, CM_CLKSEL);

	printk(KERN_INFO "ehci_omap_get_clock_regs WKUP_MOD  CM_CLKSEL reg =0x%x\n", val);
	

	val = omap2_cm_read_mod_reg(WKUP_MOD, OMAP2_CM_CLKSTCTRL);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD  OMAP2_CM_CLKSTCTRL reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(WKUP_MOD, OMAP3430_CM_CLKSTST);

	printk(KERN_INFO "ehci_omap_get_clock_regs CORE_MOD OMAP3430_CM_CLKSTST reg =0x%x\n", val);





	/** OMAP3430ES2_USBHOST_MOD **/

	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, CM_FCLKEN);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430ES2_USBHOST_MOD  CM_FCLKEN reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, CM_ICLKEN);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430ES2_USBHOST_MOD CM_ICLKEN reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, CM_IDLEST);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430ES2_USBHOST_MOD  CM_IDLEST  reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, OMAP3430_CM_SLEEPDEP);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430ES2_USBHOST_MOD  OMAP3430_CM_SLEEPDEP reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, OMAP2_CM_CLKSTCTRL);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430ES2_USBHOST_MOD  OMAP2_CM_CLKSTCTRL reg =0x%x\n", val);

	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, OMAP3430_CM_CLKSTST);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430ES2_USBHOST_MOD OMAP3430_CM_CLKSTST reg =0x%x\n", val);


}


static void ehci_omap_get_PRM_regs(struct ehci_hcd_omap *omap)
{
	
	u32 val;


	/**HOST USB module**/ 

	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, OMAP2_RM_RSTST);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP2_RM_RSTST reg =0x%x\n", val);

	
	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, PM_WKEN	);

	printk(KERN_INFO "ehci_omap_get_clock_regs PM_WKEN reg =0x%x\n", val);


	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, OMAP3430_PM_MPUGRPSEL);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430_PM_MPUGRPSEL reg =0x%x\n", val);

	
	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, 	OMAP3430_PM_IVAGRPSEL);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430_PM_IVAGRPSEL reg =0x%x\n", val);

	
	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, 	PM_WKST	);

	printk(KERN_INFO "ehci_omap_get_clock_regs PM_WKST reg =0x%x\n", val);


	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, 	PM_WKDEP );

	printk(KERN_INFO "ehci_omap_get_clock_regs PM_WKDEP reg =0x%x\n", val);

	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, 	OMAP2_PM_PWSTCTRL);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP2_PM_PWSTCTRL reg =0x%x\n", val);
	
	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, 	OMAP2_PM_PWSTST);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP2_PM_PWSTST reg =0x%x\n", val);

	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, OMAP3430_PM_PREPWSTST	);

	printk(KERN_INFO "ehci_omap_get_clock_regs OMAP3430_PM_PREPWSTST  reg =0x%x\n", val);

}

#endif



static void ehci_omap_enable(struct ehci_hcd_omap *omap, int enable)
{
	u32 reg;

	if (enable) {
		ehci_omap_clock_power(omap, 1);

		/* Enable NoIdle/NoStandby mode */
		reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		reg &= ~(OMAP_UHH_SYSCONFIG_SIDLEMASK
				| OMAP_UHH_SYSCONFIG_MIDLEMASK);
		reg |= OMAP_UHH_SYSCONFIG_NOIDLE
				| OMAP_UHH_SYSCONFIG_NOSTDBY;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, reg);
		omap->suspended = 0;
	} else {
		/* Enable ForceIdle/ForceStandby mode */
		reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		reg &= ~(OMAP_UHH_SYSCONFIG_SIDLEMASK
				| OMAP_UHH_SYSCONFIG_MIDLEMASK);
		reg |= OMAP_UHH_SYSCONFIG_FORCEIDLE
				| OMAP_UHH_SYSCONFIG_FORCESTDBY;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, reg);

		ehci_omap_clock_power(omap, 0);
		omap->suspended = 1;
	}
}

/* Electrical test support */
static void host_write_port(u8 port, const char *buf)
{
	struct usb_bus *bus;
	struct ehci_hcd *ehci = hcd_to_ehci(ghcd);
	u32 port_status;
	u32 cmd;

	/* Reset Device */
	if (!strncmp(buf, "reset", 5)) {
		printk(KERN_INFO "\n RESET PORT\n");
		bus = hcd_to_bus(ghcd);
		if (bus->root_hub->children[port])
			usb_reset_device(bus->root_hub->children[port]);
	}

	if (!strncmp(buf, "t-j", 3)) {
		printk(KERN_INFO "\n TEST_J\n");

#ifdef CONFIG_PM
		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
#endif
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 1<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	if (!strncmp(buf, "t-k", 3)) {
		printk(KERN_INFO "\n TEST_K\n");

#ifdef CONFIG_PM
		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
#endif
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 2<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	if (!strncmp(buf, "t-se0", 5)) {
		printk(KERN_INFO "\n TEST_SE0_NAK\n");

#ifdef CONFIG_PM
		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
#endif
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 3<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	/* Send test packet on suspended port */
	if (!strncmp(buf, "t-pkt", 5)) {
		printk(KERN_INFO "\n TEST_PACKET\n");

#ifdef CONFIG_PM
		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
#endif
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		/* Set Test packet bit */
		port_status |= 4<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	if (!strncmp(buf, "t-force", 7)) {
		printk(KERN_INFO "\n TEST_FORCE\n");

#ifdef CONFIG_PM
		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
#endif
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 5<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

}

static ssize_t
host_show_port(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "\nOptions\t--> Description\n"
			"\nreset\t-->Reset Device"
			"\nt-j\t-->Send TEST_J on suspended port"
			"\nt-k\t-->Send TEST_K on suspended port"
			"\nt-pkt\t-->Send TEST_PACKET[53] on suspended port"
			"\nt-force\t-->Send TEST_FORCE_ENABLE on suspended port"
			"\nt-se0\t-->Send TEST_SE0_NAK on suspended port\n\n"
			);
}

/* Port 1 */
static ssize_t
host_write_port1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct usb_device       *udev = to_usb_device(dev);

	usb_lock_device(udev);
	host_write_port(0, buf);
	usb_unlock_device(udev);
	return n;
}
static DEVICE_ATTR(port1, S_IRUGO | S_IWUSR, host_show_port, host_write_port1);

/* Port 2 */
static ssize_t
host_write_port2(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct usb_device       *udev = to_usb_device(dev);

	usb_lock_device(udev);
	host_write_port(1, buf);
	usb_unlock_device(udev);
	return n;
}
static DEVICE_ATTR(port2, S_IRUGO | S_IWUSR, host_show_port, host_write_port2);

/* Port 3 */
static ssize_t
host_write_port3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct usb_device       *udev = to_usb_device(dev);

	usb_lock_device(udev);
	host_write_port(2, buf);
	usb_unlock_device(udev);
	return n;
}
static DEVICE_ATTR(port3, S_IRUGO | S_IWUSR, host_show_port, host_write_port3);

static void omap_usb_utmi_init(struct ehci_hcd_omap *omap, u8 tll_channel_mask)
{
	unsigned reg;
	int i;

	/* Program the 3 TLL channels upfront */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		reg = ehci_omap_readl(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i));

		/* Disable AutoIdle, BitStuffing and use SDR Mode */
		reg &= ~(OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE
				| OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF
				| OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE);
		ehci_omap_writel(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i), reg);
	}

	/* Program Common TLL register */
	reg = ehci_omap_readl(omap->tll_base, OMAP_TLL_SHARED_CONF);
	reg |= (OMAP_TLL_SHARED_CONF_FCLK_IS_ON
			| OMAP_TLL_SHARED_CONF_USB_DIVRATION
			| OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN);
	reg &= ~OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN;

	ehci_omap_writel(omap->tll_base, OMAP_TLL_SHARED_CONF, reg);

	/* Enable channels now */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		reg = ehci_omap_readl(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i));

		/* Enable only the reg that is needed */
		if (!(tll_channel_mask & 1<<i))
			continue;

		reg |= OMAP_TLL_CHANNEL_CONF_CHANEN;
		ehci_omap_writel(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i), reg);

		ehci_omap_writeb(omap->tll_base,
				OMAP_TLL_ULPI_SCRATCH_REGISTER(i), 0xbe);
		dev_dbg(omap->dev, "ULPI_SCRATCH_REG[ch=%d]= 0x%02x\n",
				i+1, ehci_omap_readb(omap->tll_base,
				OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}

/*-------------------------------------------------------------------------*/

static void omap_ehci_soft_phy_reset(struct ehci_hcd_omap *omap, u8 port)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	unsigned reg = 0;

	reg = ULPI_FUNC_CTRL_RESET
		/* FUNCTION_CTRL_SET register */
		| (ULPI_SET(ULPI_FUNC_CTRL) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
		/* Write */
		| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
		/* PORTn */
		| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
		/* start ULPI access*/
		| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_omap_writel(omap->ehci_base, EHCI_INSNREG05_ULPI, reg);

	/* Wait for ULPI access completion */
	while ((ehci_omap_readl(omap->ehci_base, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_dbg(omap->dev, "phy reset operation timed out\n");
			break;
		}
	}
}

/* omap_start_ehc
 *	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct ehci_hcd_omap *omap, struct usb_hcd *hcd)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	u8 tll_ch_mask = 0;
	unsigned reg = 0;
	int ret = 0;

	dev_dbg(omap->dev, "starting TI EHCI USB Controller\n");

	printk(KERN_INFO "omap_start_ehc \n");

	
	
	/* Get all the clock handles we need */
	omap->usbhost_ick = clk_get(omap->dev, "usbhost_ick");
	if (IS_ERR(omap->usbhost_ick)) {
		dev_err(omap->dev, "could not get usbhost_ick\n");
		ret =  PTR_ERR(omap->usbhost_ick);
		goto err_host_ick;
	}

	omap->usbhost2_120m_fck = clk_get(omap->dev, "usbhost_120m_fck");
	if (IS_ERR(omap->usbhost2_120m_fck)) {
		dev_err(omap->dev, "could not get usbhost_120m_fck\n");
		ret = PTR_ERR(omap->usbhost2_120m_fck);
		goto err_host_120m_fck;
	}

	omap->usbhost1_48m_fck = clk_get(omap->dev, "usbhost_48m_fck");
	if (IS_ERR(omap->usbhost1_48m_fck)) {
		dev_err(omap->dev, "could not get usbhost_48m_fck\n");
		ret = PTR_ERR(omap->usbhost1_48m_fck);
		goto err_host_48m_fck;
	}

	if (omap->phy_reset) {
		/* Refer: ISSUE1 */
		if (gpio_is_valid(omap->reset_gpio_port[0])) {
			gpio_request(omap->reset_gpio_port[0],
						"USB1 PHY reset");
			gpio_direction_output(omap->reset_gpio_port[0], 0);
		}

		if (gpio_is_valid(omap->reset_gpio_port[1])) {
			gpio_request(omap->reset_gpio_port[1],
						"USB2 PHY reset");
			gpio_direction_output(omap->reset_gpio_port[1], 0);
		}

		/* Hold the PHY in RESET for enough time till DIR is high */
		udelay(10);
	}

	omap->usbtll_fck = clk_get(omap->dev, "usbtll_fck");
	if (IS_ERR(omap->usbtll_fck)) {
		dev_err(omap->dev, "could not get usbtll_fck\n");
		ret = PTR_ERR(omap->usbtll_fck);
		goto err_tll_fck;
	}

	omap->usbtll_ick = clk_get(omap->dev, "usbtll_ick");
	if (IS_ERR(omap->usbtll_ick)) {
		dev_err(omap->dev, "could not get usbtll_ick\n");
		ret = PTR_ERR(omap->usbtll_ick);
		goto err_tll_ick;
	}

	/* Now enable all the clocks in the correct order */
	ehci_omap_clock_power(omap, 1);

	/* perform TLL soft reset, and wait until reset is complete */
	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG,
			OMAP_USBTLL_SYSCONFIG_SOFTRESET);

	/* Wait for TLL reset to complete */
	while (!(ehci_omap_readl(omap->tll_base, OMAP_USBTLL_SYSSTATUS)
			& OMAP_USBTLL_SYSSTATUS_RESETDONE)) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_dbg(omap->dev, "operation timed out\n");
			ret = -EINVAL;
			goto err_sys_status;
		}
	}

	dev_dbg(omap->dev, "TLL RESET DONE\n");

	/* Enable smart-idle, wakeup */
	reg = OMAP_USBTLL_SYSCONFIG_CACTIVITY
			| OMAP_USBTLL_SYSCONFIG_AUTOIDLE
			| OMAP_USBTLL_SYSCONFIG_ENAWAKEUP
			| OMAP_USBTLL_SYSCONFIG_SMARTIDLE;
	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG, reg);

	/* Put UHH in NoIdle/NoStandby mode */
	reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
	reg |= OMAP_UHH_SYSCONFIG_CACTIVITY
			| OMAP_UHH_SYSCONFIG_AUTOIDLE
			| OMAP_UHH_SYSCONFIG_ENAWAKEUP;
	reg &= ~(OMAP_UHH_SYSCONFIG_SIDLEMASK | OMAP_UHH_SYSCONFIG_MIDLEMASK);
	reg |= OMAP_UHH_SYSCONFIG_NOIDLE
			| OMAP_UHH_SYSCONFIG_NOSTDBY;

	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, reg);

	reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_HOSTCONFIG);

	/* setup ULPI bypass and burst configurations */
	reg |= (OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN);
	reg &= ~OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN;

	if (omap->port_mode[0] == EHCI_HCD_OMAP_MODE_UNKNOWN)
		reg &= ~OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS;
	if (omap->port_mode[1] == EHCI_HCD_OMAP_MODE_UNKNOWN)
		reg &= ~OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS;
	if (omap->port_mode[2] == EHCI_HCD_OMAP_MODE_UNKNOWN)
		reg &= ~OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS;

	/* Bypass the TLL module for PHY mode operation */
	if (cpu_is_omap3430() && (omap_rev() <= OMAP3430_REV_ES2_1)) {
		dev_dbg(omap->dev, "OMAP3 ES version <= ES2.1\n");
		if ((omap->port_mode[0] == EHCI_HCD_OMAP_MODE_PHY) ||
			(omap->port_mode[1] == EHCI_HCD_OMAP_MODE_PHY) ||
				(omap->port_mode[2] == EHCI_HCD_OMAP_MODE_PHY))
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_BYPASS;
		else
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_BYPASS;
	} else {
		dev_dbg(omap->dev, "OMAP3 ES version > ES2.1\n");
		if (omap->port_mode[0] == EHCI_HCD_OMAP_MODE_PHY)
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS;
		else if (omap->port_mode[0] == EHCI_HCD_OMAP_MODE_TLL)
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS;

		if (omap->port_mode[1] == EHCI_HCD_OMAP_MODE_PHY)
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;
		else if (omap->port_mode[1] == EHCI_HCD_OMAP_MODE_TLL)
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;

		if (omap->port_mode[2] == EHCI_HCD_OMAP_MODE_PHY)
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS;
		else if (omap->port_mode[2] == EHCI_HCD_OMAP_MODE_TLL)
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS;

	}
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_HOSTCONFIG, reg);
	dev_dbg(omap->dev, "UHH setup done, uhh_hostconfig=%x\n", reg);


	/*
	 * An undocumented "feature" in the OMAP3 EHCI controller,
	 * causes suspended ports to be taken out of suspend when
	 * the USBCMD.Run/Stop bit is cleared (for example when
	 * we do ehci_bus_suspend).
	 * This breaks suspend-resume if the root-hub is allowed
	 * to suspend. Writing 1 to this undocumented register bit
	 * disables this feature and restores normal behavior.
	 */
	ehci_omap_writel(omap->ehci_base, EHCI_INSNREG04,
				EHCI_INSNREG04_DISABLE_UNSUSPEND);

	if ((omap->port_mode[0] == EHCI_HCD_OMAP_MODE_TLL) ||
		(omap->port_mode[1] == EHCI_HCD_OMAP_MODE_TLL) ||
			(omap->port_mode[2] == EHCI_HCD_OMAP_MODE_TLL)) {

		if (omap->port_mode[0] == EHCI_HCD_OMAP_MODE_TLL)
			tll_ch_mask |= OMAP_TLL_CHANNEL_1_EN_MASK;
		if (omap->port_mode[1] == EHCI_HCD_OMAP_MODE_TLL)
			tll_ch_mask |= OMAP_TLL_CHANNEL_2_EN_MASK;
		if (omap->port_mode[2] == EHCI_HCD_OMAP_MODE_TLL)
			tll_ch_mask |= OMAP_TLL_CHANNEL_3_EN_MASK;

		/* Enable UTMI mode for required TLL channels */
		omap_usb_utmi_init(omap, tll_ch_mask);
	}

	if (omap->phy_reset) {
		/* Refer ISSUE1:
		 * Hold the PHY in RESET for enough time till
		 * PHY is settled and ready
		 */
		udelay(10);

		if (gpio_is_valid(omap->reset_gpio_port[0]))
			gpio_set_value(omap->reset_gpio_port[0], 1);

		if (gpio_is_valid(omap->reset_gpio_port[1]))
			gpio_set_value(omap->reset_gpio_port[1], 1);
	}

	/* Soft reset the PHY using PHY reset command over ULPI */
	if (omap->port_mode[0] == EHCI_HCD_OMAP_MODE_PHY)
		omap_ehci_soft_phy_reset(omap, 0);
	if (omap->port_mode[1] == EHCI_HCD_OMAP_MODE_PHY)
		omap_ehci_soft_phy_reset(omap, 1);

	return 0;

err_sys_status:
	ehci_omap_clock_power(omap, 0);
	clk_put(omap->usbtll_ick);

err_tll_ick:
	clk_put(omap->usbtll_fck);

err_tll_fck:
	clk_put(omap->usbhost1_48m_fck);

	if (omap->phy_reset) {
		if (gpio_is_valid(omap->reset_gpio_port[0]))
			gpio_free(omap->reset_gpio_port[0]);

		if (gpio_is_valid(omap->reset_gpio_port[1]))
			gpio_free(omap->reset_gpio_port[1]);
	}

err_host_48m_fck:
	clk_put(omap->usbhost2_120m_fck);

err_host_120m_fck:
	clk_put(omap->usbhost_ick);

err_host_ick:
	return ret;
}

static void omap_stop_ehc(struct ehci_hcd_omap *omap, struct usb_hcd *hcd)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(100);
	unsigned reg = 0;
	u32 val;
	

	printk(KERN_INFO "omap_stop_ehc \n", val);


	val = omap2_cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD	, OMAP2_CM_CLKSTCTRL);
	if (0x03 == val) {
		omap2_cm_write_mod_reg(0x2, OMAP3430ES2_USBHOST_MOD, OMAP2_CM_CLKSTCTRL);

	}

#ifdef EHCI_OMAP_RECOVERY_DEBUG
	
	ehci_omap_get_clock_regs(omap);

	ehci_omap_get_PRM_regs(omap);
#endif


	dev_dbg(omap->dev, "stopping TI EHCI USB Controller\n");


	reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS);

	reg |= OMAP_UHH_SYSCONFIG_SOFTRESET;


	/* Reset OMAP modules for insmod/rmmod to work */
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG,reg);
	
	schedule_timeout(msecs_to_jiffies(1000));

	while (!(ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS)
				& (1 << 0))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(omap->dev, "operation timed out\n");
	}

	while (!(ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS)
				& (1 << 1))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(omap->dev, "operation timed out\n");
	}

	while (!(ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS)
				& (1 << 2))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(omap->dev, "operation timed out\n");
	}

	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG, (1 << 1));

	while (!(ehci_omap_readl(omap->tll_base, OMAP_USBTLL_SYSSTATUS)
				& (1 << 0))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(omap->dev, "operation timed out\n");
	}

	ehci_omap_clock_power(omap, 0);

	if (omap->usbtll_fck != NULL) {
		clk_put(omap->usbtll_fck);
		omap->usbtll_fck = NULL;
	}

	if (omap->usbhost_ick != NULL) {
		clk_put(omap->usbhost_ick);
		omap->usbhost_ick = NULL;
	}

	if (omap->usbhost1_48m_fck != NULL) {
		clk_put(omap->usbhost1_48m_fck);
		omap->usbhost1_48m_fck = NULL;
	}

	if (omap->usbhost2_120m_fck != NULL) {
		clk_put(omap->usbhost2_120m_fck);
		omap->usbhost2_120m_fck = NULL;
	}

	if (omap->usbtll_ick != NULL) {
		clk_put(omap->usbtll_ick);
		omap->usbtll_ick = NULL;
	}

	if (omap->phy_reset) {
		if (gpio_is_valid(omap->reset_gpio_port[0]))
			gpio_free(omap->reset_gpio_port[0]);

		if (gpio_is_valid(omap->reset_gpio_port[1]))
			gpio_free(omap->reset_gpio_port[1]);
	}

	dev_dbg(omap->dev, "Clock to USB host has been disabled\n");
}

#ifdef CONFIG_PM
/*-------------------------------------------------------------------------*/
static int ehci_omap_dev_suspend(struct device *dev)
{
	struct ehci_hcd_omap *omap = dev_get_drvdata(dev);

	if (!omap->suspended)
		ehci_omap_enable(omap, 0);
	return 0;
}

static int ehci_omap_dev_resume(struct device *dev)
{
	struct ehci_hcd_omap *omap = dev_get_drvdata(dev);

	if (omap->suspended)
		ehci_omap_enable(omap, 1);
	return 0;
}

static int ehci_omap_bus_suspend(struct usb_hcd *hcd)
{
	struct usb_bus *bus = hcd_to_bus(hcd);
	int ret;

	ret = ehci_bus_suspend(hcd);

	return ret;
}
static int ehci_omap_bus_resume(struct usb_hcd *hcd)
{
	struct usb_bus *bus = hcd_to_bus(hcd);
	int ret;

	ret = ehci_bus_resume(hcd);

	return ret;
}
static const struct dev_pm_ops ehci_omap_dev_pm_ops = {
	.suspend_noirq	= ehci_omap_dev_suspend,
	.resume_noirq	= ehci_omap_dev_resume,
};
#define EHCI_OMAP_DEV_PM_OPS (&ehci_omap_dev_pm_ops)
#else
#define EHCI_OMAP_DEV_PM_OPS NULL
#endif

/**
 * ehci_omap_port_handed_over - hand the port out if failed to enable it
 * @hcd:	Pointer to the usb_hcd device to which the host controller bound
 * @portnum:	Port number to which the device is attached.
 *
 * This function is used as a place to tell the user that the OMAP USB host
 * controller does support FS/LS devices via HS USB hubs only.
 */
static int ehci_omap_port_handed_over(struct usb_hcd *hcd, int portnum)
{
	dev_warn(hcd->self.controller, "port %d cannot be enabled\n", portnum);

	dev_warn(hcd->self.controller,
		"Maybe your device is not a high speed device?\n");
	dev_warn(hcd->self.controller,
		"USB host (EHCI) controller does not support full speed "
		"or low speed device on it's root port.\n");
	dev_warn(hcd->self.controller,
		"Please connect full/low speed device via a high speed hub.\n");

	return 0;
}

static const struct hc_driver ehci_omap_hc_driver;

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_probe - initialize TI-based HCDs
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int ehci_hcd_omap_probe(struct platform_device *pdev)
{
	struct ehci_hcd_omap_platform_data *pdata = pdev->dev.platform_data;
	struct ehci_hcd_omap *omap;
	struct resource *res;
	struct usb_hcd *hcd;

	int irq = platform_get_irq(pdev, 0);
	int ret = -ENODEV;
	int i;
	char supply[7];

	if (!pdata) {
		dev_dbg(&pdev->dev, "missing platform_data\n");
		goto err_pdata;
	}

	if (usb_disabled())
		goto err_disabled;

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		ret = -ENOMEM;
		goto err_disabled;
	}

	/* Electrical test support
	 * Interface is /sys/devices/platform/ehci-omap.0/portn
	 */
	if (pdata->port_mode[0] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		ret = device_create_file(&pdev->dev, &dev_attr_port1);
	if (pdata->port_mode[1] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		ret = device_create_file(&pdev->dev, &dev_attr_port2);
	if (pdata->port_mode[2] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		ret = device_create_file(&pdev->dev, &dev_attr_port3);

	ghcd = hcd = usb_create_hcd(&ehci_omap_hc_driver, &pdev->dev,
			dev_name(&pdev->dev));
	if (!hcd) {
		dev_dbg(&pdev->dev, "failed to create hcd with err %d\n", ret);
		ret = -ENOMEM;
		goto err_create_hcd;
	}

	platform_set_drvdata(pdev, omap);
	omap->dev		= &pdev->dev;
	omap->phy_reset		= pdata->phy_reset;
	omap->reset_gpio_port[0]	= pdata->reset_gpio_port[0];
	omap->reset_gpio_port[1]	= pdata->reset_gpio_port[1];
	omap->reset_gpio_port[2]	= pdata->reset_gpio_port[2];
	omap->port_mode[0]		= pdata->port_mode[0];
	omap->port_mode[1]		= pdata->port_mode[1];
	omap->port_mode[2]		= pdata->port_mode[2];
	omap->ehci		= hcd_to_ehci(hcd);
	omap->ehci->sbrn	= 0x20;
	omap->suspended = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "EHCI ioremap failed\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	/* we know this is the memory we want, no need to ioremap again */
	omap->ehci->caps = hcd->regs;
	omap->ehci_base = hcd->regs;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	omap->uhh_base = ioremap(res->start, resource_size(res));
	if (!omap->uhh_base) {
		dev_err(&pdev->dev, "UHH ioremap failed\n");
		ret = -ENOMEM;
		goto err_uhh_ioremap;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	omap->tll_base = ioremap(res->start, resource_size(res));
	if (!omap->tll_base) {
		dev_err(&pdev->dev, "TLL ioremap failed\n");
		ret = -ENOMEM;
		goto err_tll_ioremap;
	}

	/* get ehci regulator and enable */
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		if (omap->port_mode[i] != EHCI_HCD_OMAP_MODE_PHY) {
			omap->regulator[i] = NULL;
			continue;
		}
		snprintf(supply, sizeof(supply), "hsusb%d", i);
		omap->regulator[i] = regulator_get(omap->dev, supply);
		if (IS_ERR(omap->regulator[i])) {
			omap->regulator[i] = NULL;
			dev_dbg(&pdev->dev,
			"failed to get ehci port%d regulator\n", i);
		} else {
			regulator_enable(omap->regulator[i]);
		}
	}

	ret = omap_start_ehc(omap, hcd);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to start ehci\n");
		goto err_start;
	}

	omap->ehci->regs = hcd->regs
		+ HC_LENGTH(readl(&omap->ehci->caps->hc_capbase));

	dbg_hcs_params(omap->ehci, "reset");
	dbg_hcc_params(omap->ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	omap->ehci->hcs_params = readl(&omap->ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to add hcd with err %d\n", ret);
		goto err_add_hcd;
	}

	/* root ports should always stay powered */
	ehci_port_power(omap->ehci, 1);

	return 0;

err_add_hcd:
	omap_stop_ehc(omap, hcd);

err_start:
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		if (omap->regulator[i]) {
			regulator_disable(omap->regulator[i]);
			regulator_put(omap->regulator[i]);
		}
	}
	iounmap(omap->tll_base);

err_tll_ioremap:
	iounmap(omap->uhh_base);

err_uhh_ioremap:
	iounmap(hcd->regs);

err_ioremap:
	usb_put_hcd(hcd);

err_create_hcd:
	kfree(omap);
err_disabled:
err_pdata:
	return ret;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ehci_hcd_omap_remove - shutdown processing for EHCI HCDs
 * @pdev: USB Host Controller being removed
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static int ehci_hcd_omap_remove(struct platform_device *pdev)
{
	struct ehci_hcd_omap *omap = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);
	int i;

	if (omap->suspended)
		ehci_omap_enable(omap, 1);

	if (omap->port_mode[0] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		device_remove_file(&pdev->dev, &dev_attr_port1);
	if (omap->port_mode[1] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		device_remove_file(&pdev->dev, &dev_attr_port2);
	if (omap->port_mode[2] != EHCI_HCD_OMAP_MODE_UNKNOWN)
		device_remove_file(&pdev->dev, &dev_attr_port3);

	usb_remove_hcd(hcd);
	omap_stop_ehc(omap, hcd);
	iounmap(hcd->regs);
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		if (omap->regulator[i]) {
			regulator_disable(omap->regulator[i]);
			regulator_put(omap->regulator[i]);
		}
	}
	iounmap(omap->tll_base);
	iounmap(omap->uhh_base);
	usb_put_hcd(hcd);
	kfree(omap);

	return 0;
}

static void ehci_hcd_omap_shutdown(struct platform_device *pdev)
{
	struct ehci_hcd_omap *omap = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);

	if (omap->suspended)
		ehci_omap_enable(omap, 1);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct platform_driver ehci_hcd_omap_driver = {
	.probe			= ehci_hcd_omap_probe,
	.remove			= ehci_hcd_omap_remove,
	.shutdown		= ehci_hcd_omap_shutdown,
	.driver = {
		.name		= "ehci-omap",
		.pm		= EHCI_OMAP_DEV_PM_OPS,
	}
};

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "OMAP-EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#ifdef CONFIG_PM
	.bus_suspend		= ehci_omap_bus_suspend,
	.bus_resume		= ehci_omap_bus_resume,
#endif
	.relinquish_port        = NULL,
	.port_handed_over       = ehci_omap_port_handed_over,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
	.recover_hcd		= ehci_omap_recover_work,
};

MODULE_ALIAS("platform:omap-ehci");
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

