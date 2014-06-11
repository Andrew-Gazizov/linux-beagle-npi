    /*
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2006-2008 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Jouni Hogander
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <trace/events/power.h>

#include <plat/sram.h>
#include "clockdomain.h"
#include "powerdomain.h"
#include <plat/serial.h>
#include <plat/sdrc.h>
#include <plat/prcm.h>
#include <plat/gpmc.h>
#include <plat/dma.h>

#include <asm/tlbflush.h>

#include "cm2xxx_3xxx.h"
#include "cm-regbits-34xx.h"
#include "prm-regbits-34xx.h"

#include "prm2xxx_3xxx.h"
#include "pm.h"
#include "sdrc.h"
#include "control.h"
#include <linux/i2c/twl.h>

#include "../../../drivers/video/omap2/dss/dss.h"

#ifdef CONFIG_SUSPEND
static suspend_state_t suspend_state = PM_SUSPEND_ON;
static inline bool is_suspending(void)
{
	return (suspend_state != PM_SUSPEND_ON);
}
#else
static inline bool is_suspending(void)
{
	return false;
}
#endif

/* Scratchpad offsets */
#define OMAP343X_TABLE_ADDRESS_OFFSET	   0xc4
#define OMAP343X_TABLE_VALUE_OFFSET	   0xc0
#define OMAP343X_CONTROL_REG_VALUE_OFFSET  0xc8

/* pm34xx errata defined in pm.h */
u16 pm34xx_errata;

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
#endif
	struct list_head node;
};

static LIST_HEAD(pwrst_list);

static void (*_omap_sram_idle)(u32 *addr, int save_state);

static int (*_omap_save_secure_sram)(u32 *addr);

static struct powerdomain *mpu_pwrdm, *neon_pwrdm;
static struct powerdomain *core_pwrdm, *per_pwrdm;
static struct powerdomain *cam_pwrdm;
static struct powerdomain *sgx_pwrdm,*dss_pwrdm, *iva2_pwrdm, *usbhost_pwrdm, *emu_pwrdm;


static inline void omap3_per_save_context(void)
{
	omap_gpio_save_context();
}

static inline void omap3_per_restore_context(void)
{
	omap_gpio_restore_context();
}

static void omap3_enable_io_chain(void)
{
	int timeout = 0;

	if (omap_rev() >= OMAP3430_REV_ES3_1) {
		omap2_prm_set_mod_reg_bits(OMAP3430_EN_IO_CHAIN_MASK, WKUP_MOD,
				     PM_WKEN);
		/* Do a readback to assure write has been done */
        omap2_prm_read_mod_reg(WKUP_MOD, PM_WKEN);

		while (!(omap2_prm_read_mod_reg(WKUP_MOD, PM_WKEN) &
			 OMAP3430_ST_IO_CHAIN_MASK)) {
			timeout++;
			if (timeout > 1000) {
				printk(KERN_ERR "Wake up daisy chain "
				       "activation failed.\n");
				return;
			}
		}
        omap2_prm_set_mod_reg_bits(OMAP3430_ST_IO_CHAIN_MASK,
                     WKUP_MOD, PM_WKEN);
	}
}

static void omap3_disable_io_chain(void)
{
	if (omap_rev() >= OMAP3430_REV_ES3_1)
		omap2_prm_clear_mod_reg_bits(OMAP3430_EN_IO_CHAIN_MASK, WKUP_MOD,
				       PM_WKEN);
}

static void omap3_core_save_context(void)
{
	omap3_ctrl_save_padconf();

	/*
	 * Force write last pad into memory, as this can fail in some
	 * cases according to errata 1.157, 1.185
	 */
	omap_ctrl_writel(omap_ctrl_readl(OMAP343X_PADCONF_ETK_D14),
		OMAP343X_CONTROL_MEM_WKUP + 0x2a0);

	/* Save the Interrupt controller context */
	omap_intc_save_context();
	/* Save the GPMC context */
	omap3_gpmc_save_context();
	/* Save the system control module context, padconf already save above*/
	omap3_control_save_context();
	omap_dma_global_context_save();
}

static void omap3_core_restore_context(void)
{
	/* Restore the control module context, padconf restored by h/w */
	omap3_control_restore_context();
	/* Restore the GPMC context */
	omap3_gpmc_restore_context();
	/* Restore the interrupt controller context */
	omap_intc_restore_context();
	omap_dma_global_context_restore();
}

/*
 * FIXME: This function should be called before entering off-mode after
 * OMAP3 secure services have been accessed. Currently it is only called
 * once during boot sequence, but this works as we are not using secure
 * services.
 */
static void omap3_save_secure_ram_context(void)
{
	u32 ret;
	int mpu_next_state = pwrdm_read_next_pwrst(mpu_pwrdm);

	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		/*
		 * MPU next state must be set to POWER_ON temporarily,
		 * otherwise the WFI executed inside the ROM code
		 * will hang the system.
		 */
		pwrdm_set_next_pwrst(mpu_pwrdm, PWRDM_POWER_ON);
		ret = _omap_save_secure_sram((u32 *)
				__pa(omap3_secure_ram_storage));
		pwrdm_set_next_pwrst(mpu_pwrdm, mpu_next_state);
		/* Following is for error tracking, it should not happen */
		if (ret) {
			printk(KERN_ERR "save_secure_sram() returns %08x\n",
				ret);
			while (1)
				;
		}
	}
}

/*
 * PRCM Interrupt Handler Helper Function
 *
 * The purpose of this function is to clear any wake-up events latched
 * in the PRCM PM_WKST_x registers. It is possible that a wake-up event
 * may occur whilst attempting to clear a PM_WKST_x register and thus
 * set another bit in this register. A while loop is used to ensure
 * that any peripheral wake-up events occurring while attempting to
 * clear the PM_WKST_x are detected and cleared.
 */
static int prcm_clear_mod_irqs(s16 module, u8 regs)
{
	u32 wkst, fclk, iclk, clken;
	u16 wkst_off = (regs == 3) ? OMAP3430ES2_PM_WKST3 : PM_WKST1;
	u16 fclk_off = (regs == 3) ? OMAP3430ES2_CM_FCLKEN3 : CM_FCLKEN1;
	u16 iclk_off = (regs == 3) ? CM_ICLKEN3 : CM_ICLKEN1;
	u16 grpsel_off = (regs == 3) ?
		OMAP3430ES2_PM_MPUGRPSEL3 : OMAP3430_PM_MPUGRPSEL;
	int c = 0;

	wkst = omap2_prm_read_mod_reg(module, wkst_off);
	wkst &= omap2_prm_read_mod_reg(module, grpsel_off);
	if (wkst) {
		iclk = omap2_cm_read_mod_reg(module, iclk_off);
		fclk = omap2_cm_read_mod_reg(module, fclk_off);
		while (wkst) {
			clken = wkst;
			omap2_cm_set_mod_reg_bits(clken, module, iclk_off);
			/*
			 * For USBHOST, we don't know whether HOST1 or
			 * HOST2 woke us up, so enable both f-clocks
			 */
			if (module == OMAP3430ES2_USBHOST_MOD)
				clken |= 1 << OMAP3430ES2_EN_USBHOST2_SHIFT;
			omap2_cm_set_mod_reg_bits(clken, module, fclk_off);
			omap2_prm_write_mod_reg(wkst, module, wkst_off);
			wkst = omap2_prm_read_mod_reg(module, wkst_off);
			c++;
		}
		omap2_cm_write_mod_reg(iclk, module, iclk_off);
		omap2_cm_write_mod_reg(fclk, module, fclk_off);
	}

	return c;
}

static int _prcm_int_handle_wakeup(void)
{
	int c;

	c = prcm_clear_mod_irqs(WKUP_MOD, 1);
	c += prcm_clear_mod_irqs(CORE_MOD, 1);
	c += prcm_clear_mod_irqs(OMAP3430_PER_MOD, 1);
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		c += prcm_clear_mod_irqs(CORE_MOD, 3);
		c += prcm_clear_mod_irqs(OMAP3430ES2_USBHOST_MOD, 1);
	}

	return c;
}

//----------------------------------------
//  Clocks User Configuration
//----------------------------------------
void clockConf() {
    u32 val;

    // Disabling CAM_PRM
    omap_writel(0x0, 0x48004F00);
    omap_writel(0x0, 0x48004F10);
    omap_writel(0x0, 0x48306FE0);
    // Put DPLL3 in LP bypass mode
//    val = omap_readl(0x48004D00);
//    val &= ~0x7;
//    val |= 0x5;
//    omap_writel(val, 0x48004D00);

    // Put DPLL2 in LP stop mode
    omap_writel(0x1, 0x48004004);

    // Disabling EMU & CAM clocks
    val = omap_readl(0x48004D00);
    val |= (3 << 30);
    omap_writel(val, 0x48004D00);

    // Put DPLL5 in LP stop mode
    omap_writel(0x1, 0x48004D04);
    //----------------------------------------

}

/*
 * PRCM Interrupt Handler
 *
 * The PRM_IRQSTATUS_MPU register indicates if there are any pending
 * interrupts from the PRCM for the MPU. These bits must be cleared in
 * order to clear the PRCM interrupt. The PRCM interrupt handler is
 * implemented to simply clear the PRM_IRQSTATUS_MPU in order to clear
 * the PRCM interrupt. Please note that bit 0 of the PRM_IRQSTATUS_MPU
 * register indicates that a wake-up event is pending for the MPU and
 * this bit can only be cleared if the all the wake-up events latched
 * in the various PM_WKST_x registers have been cleared. The interrupt
 * handler is implemented using a do-while loop so that if a wake-up
 * event occurred during the processing of the prcm interrupt handler
 * (setting a bit in the corresponding PM_WKST_x register and thus
 * preventing us from clearing bit 0 of the PRM_IRQSTATUS_MPU register)
 * this would be handled.
 */
static irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 irqenable_mpu, irqstatus_mpu;
	int c = 0;

	irqenable_mpu = omap2_prm_read_mod_reg(OCP_MOD,
					 OMAP3_PRM_IRQENABLE_MPU_OFFSET);
	irqstatus_mpu = omap2_prm_read_mod_reg(OCP_MOD,
					 OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
	irqstatus_mpu &= irqenable_mpu;

	do {
		if (irqstatus_mpu & (OMAP3430_WKUP_ST_MASK |
				     OMAP3430_IO_ST_MASK)) {
			c = _prcm_int_handle_wakeup();

			/*
			 * Is the MPU PRCM interrupt handler racing with the
			 * IVA2 PRCM interrupt handler ?
			 */
			WARN(c == 0, "prcm: WARNING: PRCM indicated MPU wakeup "
			     "but no wakeup sources are marked\n");
		} else {
			/* XXX we need to expand our PRCM interrupt handler */
			WARN(1, "prcm: WARNING: PRCM interrupt received, but "
			     "no code to handle it (%08x)\n", irqstatus_mpu);
		}

		omap2_prm_write_mod_reg(irqstatus_mpu, OCP_MOD,
					OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

		irqstatus_mpu = omap2_prm_read_mod_reg(OCP_MOD,
					OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
		irqstatus_mpu &= irqenable_mpu;

	} while (irqstatus_mpu);

	return IRQ_HANDLED;
}

/* Function to restore the table entry that was modified for enabling MMU */
static void restore_table_entry(void)
{
	void __iomem *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;

	scratchpad_address = OMAP2_L4_IO_ADDRESS(OMAP343X_SCRATCHPAD);

	/* Get address of entry that was modified */
	address = (u32 *)__raw_readl(scratchpad_address +
				     OMAP343X_TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = __raw_readl(scratchpad_address +
				     OMAP343X_TABLE_VALUE_OFFSET);
	address = __va(address);
	*address = previous_value;
	flush_tlb_all();
	control_reg_value = __raw_readl(scratchpad_address
					+ OMAP343X_CONTROL_REG_VALUE_OFFSET);
	/* This will enable caches and prediction */
	set_cr(control_reg_value);
}

void reg_padconf() {
    u32 i;
    for  (i = 0x48002030; i <= 0x480025F8; i += 4) {
        if (i == 0x48002150) i = 0x48002158;
        if (i == 0x480021E4) i = 0x48002260;
        if (i == 0x48002268) i = 0x480025A0;
        printk(KERN_INFO "0x%08x: 0x%08x\n", i, omap_readl(i));
    }

      printk(KERN_INFO "\n");

    for  (i = 0x48002A00; i <= 0x48002A58; i += 4) {
        printk(KERN_INFO "0x%08x: 0x%08x\n", i, omap_readl(i));
    }
}

void HSUSBOTG_snapshot() {
    printk(KERN_INFO "\n\nHSUSBOTG REGISTER SNAPSHOT\n");
    printk(KERN_INFO "---------------------------------------\n");

    printk(KERN_INFO "OTG_REVISION: %08x\n", 			omap_readl(0x480AB400));
    printk(KERN_INFO "OTG_SYSCONFIG: %08x\n",           omap_readl(0x480AB404));
    printk(KERN_INFO "OTG_SYSSTATUS: %08x\n", 			omap_readl(0x480AB408));
    printk(KERN_INFO "OTG_INTERFSEL: %08x\n",       	omap_readl(0x480AB40C));
    printk(KERN_INFO "OTG_SIMENABLE: %08x\n",        	omap_readl(0x480AB410));
    printk(KERN_INFO "OTG_FORCESTDBY: %08x\n",      	omap_readl(0x480AB414));
    printk(KERN_INFO "OTG_BIGENDIAN: %08x\n",           omap_readl(0x480AB418));
}

void gpio4_snapshot() {
    printk(KERN_INFO "\n\nGPIO4 REGISTER SNAPSHOT\n");
    printk(KERN_INFO "---------------------------------------\n");

    printk(KERN_INFO "GPIO_REVISION: %08x\n", 			omap_readl(0x49054000));
    printk(KERN_INFO "GPIO_SYSCONFIG: %08x\n",          omap_readl(0x49054010));
    printk(KERN_INFO "GPIO_SYSSTATUS: %08x\n", 			omap_readl(0x49054014));
    printk(KERN_INFO "GPIO_IRQSTATUS1: %08x\n", 		omap_readl(0x49054018));
    printk(KERN_INFO "GPIO_IRQENABLE1: %08x\n",     	omap_readl(0x4905401C));
    printk(KERN_INFO "GPIO_WAKEUPENABLE: %08x\n",   	omap_readl(0x49054020));
    printk(KERN_INFO "GPIO_IRQSTATUS2: %08x\n",         omap_readl(0x49054028));
    printk(KERN_INFO "GPIO_IRQENABLE2: %08x\n", 		omap_readl(0x4905402C));
    printk(KERN_INFO "GPIO_CTRL: %08x\n",               omap_readl(0x49054030));
    printk(KERN_INFO "GPIO_OE: %08x\n",                 omap_readl(0x49054034));
    printk(KERN_INFO "GPIO_DATAIN: %08x\n", 			omap_readl(0x49054038));
    printk(KERN_INFO "GPIO_DATAOUT: %08x\n",            omap_readl(0x4905403C));
    printk(KERN_INFO "GPIO_LEVELDETECT0: %08x\n", 		omap_readl(0x49054040));
    printk(KERN_INFO "GPIO_LEVELDETECT1: %08x\n", 		omap_readl(0x49054044));
    printk(KERN_INFO "GPIO_RISINGDETECT: %08x\n",       omap_readl(0x49054048));
    printk(KERN_INFO "GPIO_FALLINGDETECT: %08x\n", 		omap_readl(0x4905404C));
    printk(KERN_INFO "GPIO_DEBOUNCENABLE: %08x\n", 		omap_readl(0x49054050));
    printk(KERN_INFO "GPIO_DEBOUNCINGTIME: %08x\n", 	omap_readl(0x49054054));
    printk(KERN_INFO "GPIO_CLEARIRQENABLE1: %08x\n", 	omap_readl(0x49054060));
    printk(KERN_INFO "GPIO_SETIRQENABLE1: %08x\n", 		omap_readl(0x49054064));
    printk(KERN_INFO "GPIO_CLEARIRQENABLE2: %08x\n", 	omap_readl(0x49054070));
    printk(KERN_INFO "GPIO_SETIRQENABLE2: %08x\n", 		omap_readl(0x49054074));
    printk(KERN_INFO "GPIO_CLEARWKUENA: %08x\n", 		omap_readl(0x49054080));
    printk(KERN_INFO "GPIO_SETWKUENA: %08x\n",          omap_readl(0x49054084));
    printk(KERN_INFO "GPIO_CLEARDATAOUT: %08x\n", 		omap_readl(0x49054090));
    printk(KERN_INFO "GPIO_SETDATAOUT: %08x\n", 		omap_readl(0x49054094));
    ;
}

//void dss_snapshot() {
//    printk(KERN_INFO "\n\n DSS REGISTER SNAPSHOT\n");
//    printk(KERN_INFO "---------------------------------------\n");

//    printk(KERN_INFO "DSS_REVISIONNUMBER: %08x\n",          omap_readl(0x48050000));
//    printk(KERN_INFO "DSS_SYSCONFIG: %08x\n",               omap_readl(0x48050010));
//    printk(KERN_INFO "DSS_SYSSTATUS: %08x\n",               omap_readl(0x48050014));
//    printk(KERN_INFO "DSS_IRQSTATUS: %08x\n",               omap_readl(0x48050018));
//    printk(KERN_INFO "DSS_CONTROL: %08x\n",                 omap_readl(0x48050040));
//    printk(KERN_INFO "DSS_CLK_STATUS: %08x\n",              omap_readl(0x4805005C));
//    printk(KERN_INFO "DISPC_REVISION: %08x\n",              omap_readl(0x48050400));
//    printk(KERN_INFO "DISPC_SYSCONFIG: %08x\n",             omap_readl(0x48050410));
//    printk(KERN_INFO "DISPC_SYSSTATUS: %08x\n",             omap_readl(0x48050414));
//    printk(KERN_INFO "DISPC_IRQSTATUS: %08x\n",             omap_readl(0x48050418));
//    printk(KERN_INFO "DISPC_IRQENABLE: %08x\n",             omap_readl(0x4805041C));
//    printk(KERN_INFO "DISPC_CONTROL: %08x\n",               omap_readl(0x48050440));
//    printk(KERN_INFO "DISPC_CONFIG: %08x\n",                omap_readl(0x48050444));
//    printk(KERN_INFO "DISPC_LINE_STATUS: %08x\n",           omap_readl(0x4805045C));
//    printk(KERN_INFO "DISPC_LINE_NUMBER: %08x\n",           omap_readl(0x48050460));
//    printk(KERN_INFO "DISPC_TIMING_H: %08x\n",              omap_readl(0x48050464));
//    printk(KERN_INFO "DISPC_TIMING_V: %08x\n",              omap_readl(0x48050468));
//    printk(KERN_INFO "DISPC_POL_FREQ: %08x\n",              omap_readl(0x4805046C));
//    printk(KERN_INFO "DISPC_DIVISOR: %08x\n",               omap_readl(0x48050470));
//    printk(KERN_INFO "DISPC_GLOBAL_ALPHA: %08x\n",          omap_readl(0x48050474));
//    printk(KERN_INFO "DISPC_SIZE_DIG: %08x\n",              omap_readl(0x48050478));
//    printk(KERN_INFO "DISPC_SIZE_LCD: %08x\n",              omap_readl(0x4805047C));
//    printk(KERN_INFO "DISPC_GFX_POSITION: %08x\n",          omap_readl(0x48050488));
//    printk(KERN_INFO "DISPC_GFX_SIZE: %08x\n",              omap_readl(0x4805048C));
//    printk(KERN_INFO "DISPC_GFX_ATTRIBUTES: %08x\n", 		omap_readl(0x480504A0));
//    printk(KERN_INFO "DISPC_GFX_FIFO_THRESHOLD: %08x\n", 	omap_readl(0x480504A4));
//    printk(KERN_INFO "DISPC_GFX_FIFO_SIZE_STATUS: %08x\n", 	omap_readl(0x480504A8));
//    printk(KERN_INFO "DISPC_GFX_ROW_INC: %08x\n",           omap_readl(0x480504AC));
//    printk(KERN_INFO "DISPC_GFX_PIXEL_INC: %08x\n", 		omap_readl(0x480504B0));
//    printk(KERN_INFO "DISPC_GFX_WINDOW_SKIP: %08x\n", 		omap_readl(0x480504B4));
//    printk(KERN_INFO "DISPC_GFX_TABLE_BA: %08x\n",          omap_readl(0x480504B8));
//    printk(KERN_INFO "DISPC_CPR_COEF_R: %08x\n",            omap_readl(0x48050620));
//    printk(KERN_INFO "DISPC_CPR_COEF_G: %08x\n",            omap_readl(0x48050624));
//    printk(KERN_INFO "DISPC_CPR_COEF_B: %08x\n", 			omap_readl(0x48050628));
//    printk(KERN_INFO "DISPC_GFX_PRELOAD: %08x\n",           omap_readl(0x4805062C));
//}

void reg_snapshot() {
    printk(KERN_INFO "\n\nREGISTER SNAPSHOT\n");
    printk(KERN_INFO "---------------------------------------\n");

    printk(KERN_INFO "\nIVA2_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_IVA2: %08x\n", 			omap_readl(0x48004000));
    printk(KERN_INFO "CM_CLKEN_PLL_IVA2: %08x\n", 		omap_readl(0x48004004));
    printk(KERN_INFO "CM_IDLEST_IVA2: %08x\n", 			omap_readl(0x48004020));
    printk(KERN_INFO "CM_IDLEST_PLL_IVA2: %08x\n", 		omap_readl(0x48004024));
    printk(KERN_INFO "CM_AUTOIDLE_PLL_IVA2: %08x\n", 	omap_readl(0x48004034));
    printk(KERN_INFO "CM_CLKSEL1_PLL_IVA2: %08x\n", 	omap_readl(0x48004040));
    printk(KERN_INFO "CM_CLKSEL2_PLL_IVA2: %08x\n", 	omap_readl(0x48004044));
    printk(KERN_INFO "CM_CLKSTCTRL_IVA2: %08x\n", 		omap_readl(0x48004048));
    printk(KERN_INFO "CM_CLKSTST_IVA2: %08x\n", 		omap_readl(0x4800404C));

    printk(KERN_INFO "\nOCP_System_Reg_CM:\n");
    printk(KERN_INFO "CM_REVISION: %08x\n", 			omap_readl(0x48004800));
    printk(KERN_INFO "CM_SYSCONFIG: %08x\n", 			omap_readl(0x48004810));

    printk(KERN_INFO "\nMPU_CM:\n");
    printk(KERN_INFO "CM_CLKEN_PLL_MPU: %08x\n", 		omap_readl(0x48004904));
    printk(KERN_INFO "CM_IDLEST_MPU: %08x\n", 			omap_readl(0x48004920));
    printk(KERN_INFO "CM_IDLEST_PLL_MPU: %08x\n", 		omap_readl(0x48004924));
    printk(KERN_INFO "CM_AUTOIDLE_PLL_MPU: %08x\n", 	omap_readl(0x48004934));
    printk(KERN_INFO "CM_CLKSEL1_PLL_MPU: %08x\n", 		omap_readl(0x48004940));
    printk(KERN_INFO "CM_CLKSEL2_PLL_MPU: %08x\n", 		omap_readl(0x48004944));
    printk(KERN_INFO "CM_CLKSTCTRL_MPU: %08x\n", 		omap_readl(0x48004948));
    printk(KERN_INFO "CM_CLKSTST_MPU: %08x\n", 			omap_readl(0x4800494C));

    printk(KERN_INFO "\nCORE_CM:\n");
    printk(KERN_INFO "CM_FCLKEN1_CORE: %08x\n", 		omap_readl(0x48004A00));
    printk(KERN_INFO "CM_FCLKEN3_CORE: %08x\n", 		omap_readl(0x48004A08));
    printk(KERN_INFO "CM_ICLKEN1_CORE: %08x\n", 		omap_readl(0x48004A10));
    printk(KERN_INFO "CM_ICLKEN3_CORE: %08x\n", 		omap_readl(0x48004A18));
    printk(KERN_INFO "CM_IDLEST1_CORE: %08x\n", 		omap_readl(0x48004A20));
    printk(KERN_INFO "CM_IDLEST3_CORE: %08x\n", 		omap_readl(0x48004A28));
    printk(KERN_INFO "CM_AUTOIDLE1_CORE: %08x\n", 		omap_readl(0x48004A30));
    printk(KERN_INFO "CM_AUTOIDLE3_CORE: %08x\n", 		omap_readl(0x48004A38));
    printk(KERN_INFO "CM_CLKSEL_CORE: %08x\n", 			omap_readl(0x48004A40));
    printk(KERN_INFO "CM_CLKSTCTRL_CORE: %08x\n", 		omap_readl(0x48004A48));
    printk(KERN_INFO "CM_CLKSTST_CORE: %08x\n", 		omap_readl(0x48004A4C));

    printk(KERN_INFO "\nSGX_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_SGX: %08x\n", 			omap_readl(0x48004B00));
    printk(KERN_INFO "CM_ICLKEN_SGX: %08x\n", 			omap_readl(0x48004B10));
    printk(KERN_INFO "CM_IDLEST_SGX: %08x\n", 			omap_readl(0x48004B20));
    printk(KERN_INFO "CM_CLKSEL_SGX: %08x\n", 			omap_readl(0x48004B40));
    printk(KERN_INFO "CM_SLEEPDEP_SGX: %08x\n", 		omap_readl(0x48004B44));
    printk(KERN_INFO "CM_CLKSTCTRL_SGX: %08x\n", 		omap_readl(0x48004B48));
    printk(KERN_INFO "CM_CLKSTST_SGX: %08x\n", 			omap_readl(0x48004B4C));

    printk(KERN_INFO "\nWKUP_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_WKUP: %08x\n", 			omap_readl(0x48004C00));
    printk(KERN_INFO "CM_ICLKEN_WKUP: %08x\n", 			omap_readl(0x48004C10));
    printk(KERN_INFO "CM_IDLEST_WKUP: %08x\n", 			omap_readl(0x48004C20));
    printk(KERN_INFO "CM_AUTOIDLE_WKUP: %08x\n", 		omap_readl(0x48004C30));
    printk(KERN_INFO "CM_CLKSEL_WKUP: %08x\n", 			omap_readl(0x48004C40));

    printk(KERN_INFO "\nClock_Control_Reg_CM:\n");
    printk(KERN_INFO "CM_CLKEN_PLL: %08x\n", 			omap_readl(0x48004D00));
    printk(KERN_INFO "CM_CLKEN2_PLL: %08x\n", 			omap_readl(0x48004D04));
    printk(KERN_INFO "CM_IDLEST_CKGEN: %08x\n", 		omap_readl(0x48004D20));
    printk(KERN_INFO "CM_IDLEST2_CKGEN: %08x\n", 		omap_readl(0x48004D24));
    printk(KERN_INFO "CM_AUTOIDLE_PLL: %08x\n", 		omap_readl(0x48004D30));
    printk(KERN_INFO "CM_AUTOIDLE2_PLL: %08x\n", 		omap_readl(0x48004D34));
    printk(KERN_INFO "CM_CLKSEL1_PLL: %08x\n", 			omap_readl(0x48004D40));
    printk(KERN_INFO "CM_CLKSEL2_PLL: %08x\n", 			omap_readl(0x48004D44));
    printk(KERN_INFO "CM_CLKSEL3_PLL: %08x\n", 			omap_readl(0x48004D48));
    printk(KERN_INFO "CM_CLKSEL4_PLL: %08x\n", 			omap_readl(0x48004D4C));
    printk(KERN_INFO "CM_CLKSEL5_PLL: %08x\n", 			omap_readl(0x48004D50));
    printk(KERN_INFO "CM_CLKOUT_CTRL: %08x\n", 			omap_readl(0x48004D70));

    printk(KERN_INFO "\nDSS_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_DSS: %08x\n", 			omap_readl(0x48004E00));
    printk(KERN_INFO "CM_ICLKEN_DSS: %08x\n", 			omap_readl(0x48004E10));
    printk(KERN_INFO "CM_IDLEST_DSS: %08x\n", 			omap_readl(0x48004E20));
    printk(KERN_INFO "CM_AUTOIDLE_DSS: %08x\n", 		omap_readl(0x48004E30));
    printk(KERN_INFO "CM_CLKSEL_DSS: %08x\n", 			omap_readl(0x48004E40));
    printk(KERN_INFO "CM_SLEEPDEP_DSS: %08x\n", 		omap_readl(0x48004E44));
    printk(KERN_INFO "CM_CLKSTCTRL_DSS: %08x\n", 		omap_readl(0x48004E48));
    printk(KERN_INFO "CM_CLKSTST_DSS: %08x\n", 			omap_readl(0x48004E4C));

    printk(KERN_INFO "\nCAM_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_CAM: %08x\n", 			omap_readl(0x48004F00));
    printk(KERN_INFO "CM_ICLKEN_CAM: %08x\n", 			omap_readl(0x48004F10));
    printk(KERN_INFO "CM_IDLEST_CAM: %08x\n", 			omap_readl(0x48004F20));
    printk(KERN_INFO "CM_AUTOIDLE_CAM: %08x\n", 		omap_readl(0x48004F30));
    printk(KERN_INFO "CM_CLKSEL_CAM: %08x\n", 			omap_readl(0x48004F40));
    printk(KERN_INFO "CM_SLEEPDEP_CAM: %08x\n", 		omap_readl(0x48004F44));
    printk(KERN_INFO "CM_CLKSTCTRL_CAM: %08x\n", 		omap_readl(0x48004F48));
    printk(KERN_INFO "CM_CLKSTST_CAM: %08x\n", 			omap_readl(0x48004F4C));

    printk(KERN_INFO "\nPER_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_PER: %08x\n", 			omap_readl(0x48005000));
    printk(KERN_INFO "CM_ICLKEN_PER: %08x\n", 			omap_readl(0x48005010));
    printk(KERN_INFO "CM_IDLEST_PER: %08x\n", 			omap_readl(0x48005020));
    printk(KERN_INFO "CM_AUTOIDLE_PER: %08x\n", 		omap_readl(0x48005030));
    printk(KERN_INFO "CM_CLKSEL_PER: %08x\n", 			omap_readl(0x48005040));
    printk(KERN_INFO "CM_SLEEPDEP_PER: %08x\n", 		omap_readl(0x48005044));
    printk(KERN_INFO "CM_CLKSTCTRL_PER: %08x\n", 		omap_readl(0x48005048));
    printk(KERN_INFO "CM_CLKSTST_PER: %08x\n", 			omap_readl(0x4800504C));

    printk(KERN_INFO "\nEMU_CM:\n");
    printk(KERN_INFO "CM_CLKSEL1_EMU: %08x\n", 			omap_readl(0x48005140));
    printk(KERN_INFO "CM_CLKSTCTRL_EMU: %08x\n", 		omap_readl(0x48005148));
    printk(KERN_INFO "CM_CLKSTST_EMU: %08x\n", 			omap_readl(0x4800514C));
    printk(KERN_INFO "CM_CLKSEL2_EMU: %08x\n", 			omap_readl(0x48005150));
    printk(KERN_INFO "CM_CLKSEL3_EMU: %08x\n", 			omap_readl(0x48005154));

    printk(KERN_INFO "\Global_Reg_CM:\n");
    printk(KERN_INFO "CM_POLCTRL: %08x\n", 				omap_readl(0x4800529C));

    printk(KERN_INFO "\NEON_CM:\n");
    printk(KERN_INFO "CM_IDLEST_NEON: %08x\n", 			omap_readl(0x48005320));
    printk(KERN_INFO "CM_CLKSTCTRL_NEON: %08x\n", 		omap_readl(0x48005348));

    printk(KERN_INFO "\nUSBHOST_CM:\n");
    printk(KERN_INFO "CM_FCLKEN_USBHOST: %08x\n", 		omap_readl(0x48005400));
    printk(KERN_INFO "CM_ICLKEN_USBHOST: %08x\n", 		omap_readl(0x48005410));
    printk(KERN_INFO "CM_IDLEST_USBHOST: %08x\n", 		omap_readl(0x48005420));
    printk(KERN_INFO "CM_AUTOIDLE_USBHOST: %08x\n", 	omap_readl(0x48005430));
    printk(KERN_INFO "CM_CLKSEL_USBHOST: %08x\n", 		omap_readl(0x48005440));
    printk(KERN_INFO "CM_SLEEPDEP_USBHOST: %08x\n", 	omap_readl(0x48005444));
    printk(KERN_INFO "CM_CLKSTCTRL_USBHOST: %08x\n", 	omap_readl(0x48005448));
    printk(KERN_INFO "CM_CLKSTST_USBHOST: %08x\n", 		omap_readl(0x4800544C));

    printk(KERN_INFO "\nIVA2_PRM:\n");
    printk(KERN_INFO "RM_RSTCTRL_IVA2: %08x\n", 		omap_readl(0x48306050));
    printk(KERN_INFO "RM_RSTST_IVA2: %08x\n", 			omap_readl(0x48306058));
    printk(KERN_INFO "PM_WKDEP_IVA2: %08x\n", 			omap_readl(0x483060C8));
    printk(KERN_INFO "PM_PWSTCTRL_IVA2: %08x\n", 		omap_readl(0x483060E0));
    printk(KERN_INFO "PM_PWSTST_IVA2: %08x\n", 			omap_readl(0x483060E4));
    printk(KERN_INFO "PM_PREPWSTST_IVA2: %08x\n", 		omap_readl(0x483060E8));
    printk(KERN_INFO "PRM_IRQSTATUS_IVA2: %08x\n", 		omap_readl(0x483060F8));
    printk(KERN_INFO "PRM_IRQENABLE_IVA2: %08x\n", 		omap_readl(0x483060FC));

    printk(KERN_INFO "\nOCP_System_Reg_PRM:\n");
    printk(KERN_INFO "PRM_REVISION: %08x\n", 			omap_readl(0x48306804));
    printk(KERN_INFO "PRM_SYSCONFIG: %08x\n", 			omap_readl(0x48306814));
    printk(KERN_INFO "PRM_IRQSTATUS_MPU: %08x\n", 		omap_readl(0x48306818));
    printk(KERN_INFO "PRM_IRQENABLE_MPU : %08x\n", 		omap_readl(0x4830681C));

    printk(KERN_INFO "\nMPU_PRM:\n");
    printk(KERN_INFO "RM_RSTST_MPU: %08x\n", 			omap_readl(0x48306958));
    printk(KERN_INFO "PM_WKDEP_MPU: %08x\n", 			omap_readl(0x483069C8));
    printk(KERN_INFO "PM_EVGENCTRL_MPU: %08x\n", 		omap_readl(0x483069D4));
    printk(KERN_INFO "PM_EVGENONTIM_MPU: %08x\n", 		omap_readl(0x483069D8));
    printk(KERN_INFO "PM_EVGENOFFTIM_MPU : %08x\n", 	omap_readl(0x483069DC));
    printk(KERN_INFO "PM_PWSTCTRL_MPU: %08x\n", 		omap_readl(0x483069E0));
    printk(KERN_INFO "PM_PWSTST_MPU: %08x\n", 			omap_readl(0x483069E4));
    printk(KERN_INFO "PM_PREPWSTST_MPU: %08x\n", 		omap_readl(0x483069E8));

    printk(KERN_INFO "\nCORE_PRM:\n");
    printk(KERN_INFO "RM_RSTST_CORE: %08x\n", 			omap_readl(0x48306A58));
    printk(KERN_INFO "PM_WKEN1_CORE: %08x\n", 			omap_readl(0x48306AA0));
    printk(KERN_INFO "PM_MPUGRPSEL1_CORE: %08x\n", 		omap_readl(0x48306AA4));
    printk(KERN_INFO "PM_IVA2GRPSEL1_CORE: %08x\n", 	omap_readl(0x48306AA8));
    printk(KERN_INFO "PM_WKST1_CORE: %08x\n", 			omap_readl(0x48306AB0));
    printk(KERN_INFO "PM_WKST3_CORE: %08x\n", 			omap_readl(0x48306AB8));
    printk(KERN_INFO "PM_PWSTCTRL_CORE: %08x\n", 		omap_readl(0x48306AE0));
    printk(KERN_INFO "PM_PWSTST_CORE: %08x\n", 			omap_readl(0x48306AE4));
    printk(KERN_INFO "PM_PREPWSTST_CORE: %08x\n", 		omap_readl(0x48306AE8));
    printk(KERN_INFO "PM_WKEN3_CORE: %08x\n", 			omap_readl(0x48306AF0));
    printk(KERN_INFO "PM_IVA2GRPSEL3_CORE: %08x\n", 	omap_readl(0x48306AF4));
    printk(KERN_INFO "PM_MPUGRPSEL3_CORE: %08x\n", 		omap_readl(0x48306AF8));

    printk(KERN_INFO "\nSGX_PRM:\n");
    printk(KERN_INFO "RM_RSTST_SGX: %08x\n", 			omap_readl(0x48306B58));
    printk(KERN_INFO "PM_WKDEP_SGX: %08x\n", 			omap_readl(0x48306BC8));
    printk(KERN_INFO "PM_PWSTCTRL_SGX: %08x\n", 		omap_readl(0x48306BE0));
    printk(KERN_INFO "PM_PWSTST_SGX: %08x\n", 			omap_readl(0x48306BE4));
    printk(KERN_INFO "PM_PREPWSTST_SGX: %08x\n", 		omap_readl(0x48306BE8));

    printk(KERN_INFO "\nWKUP_PRM:\n");
    printk(KERN_INFO "PM_WKEN_WKUP: %08x\n", 			omap_readl(0x48306CA0));
    printk(KERN_INFO "PM_MPUGRPSEL_WKUP: %08x\n", 		omap_readl(0x48306CA4));
    printk(KERN_INFO "PM_IVA2GRPSEL_WKUP: %08x\n", 		omap_readl(0x48306CA8));
    printk(KERN_INFO "PM_WKST_WKUP: %08x\n", 			omap_readl(0x48306CB0));

    printk(KERN_INFO "\Clock_Control_Reg_PRM:\n");
    printk(KERN_INFO "PRM_CLKSEL: %08x\n", 				omap_readl(0x48306D40));
    printk(KERN_INFO "PRM_CLKOUT_CTRL: %08x\n", 		omap_readl(0x48306D70));

    printk(KERN_INFO "\nDSS_PRM:\n");
    printk(KERN_INFO "RM_RSTST_DSS: %08x\n", 			omap_readl(0x48306E58));
    printk(KERN_INFO "PM_WKEN_DSS: %08x\n", 			omap_readl(0x48306EA0));
    printk(KERN_INFO "PM_WKDEP_DSS: %08x\n", 			omap_readl(0x48306EC8));
    printk(KERN_INFO "PM_PWSTCTRL_DSS: %08x\n", 		omap_readl(0x48306EE0));
    printk(KERN_INFO "PM_PWSTST_DSS: %08x\n", 			omap_readl(0x48306EE4));
    printk(KERN_INFO "PM_PREPWSTST_DSS: %08x\n", 		omap_readl(0x48306EE8));

    printk(KERN_INFO "\nCAM_PRM:\n");
    printk(KERN_INFO "RM_RSTST_CAM: %08x\n", 			omap_readl(0x48306F58));
    printk(KERN_INFO "PM_WKDEP_CAM: %08x\n", 			omap_readl(0x48306FC8));
    printk(KERN_INFO "PM_PWSTCTRL_CAM: %08x\n", 		omap_readl(0x48306FE0));
    printk(KERN_INFO "PM_PWSTST_CAM: %08x\n", 			omap_readl(0x48306FE4));
    printk(KERN_INFO "PM_PREPWSTST_CAM: %08x\n", 		omap_readl(0x48306FE8));

    printk(KERN_INFO "\nPER_PRM:\n");
    printk(KERN_INFO "RM_RSTST_PER: %08x\n", 			omap_readl(0x48307058));
    printk(KERN_INFO "PM_WKEN_PER: %08x\n", 			omap_readl(0x483070A0));
    printk(KERN_INFO "PM_MPUGRPSEL_PER: %08x\n", 		omap_readl(0x483070A4));
    printk(KERN_INFO "PM_IVA2GRPSEL_PER: %08x\n", 		omap_readl(0x483070A8));
    printk(KERN_INFO "PM_WKST_PER: %08x\n", 			omap_readl(0x483070B0));
    printk(KERN_INFO "PM_WKDEP_PER: %08x\n", 			omap_readl(0x483070C8));
    printk(KERN_INFO "PM_PWSTCTRL_PER: %08x\n", 		omap_readl(0x483070E0));
    printk(KERN_INFO "PM_PWSTST_PER: %08x\n", 			omap_readl(0x483070E4));
    printk(KERN_INFO "PM_PREPWSTST_PER: %08x\n", 		omap_readl(0x483070E8));

    printk(KERN_INFO "\nEMU_PRM:\n");
    printk(KERN_INFO "RM_RSTST_EMU: %08x\n", 			omap_readl(0x48307158));
    printk(KERN_INFO "PM_PWSTST_EMU: %08x\n", 			omap_readl(0x483071E4));

    printk(KERN_INFO "\nGlobal_Reg_PRM:\n");
    printk(KERN_INFO "PRM_VC_SMPS_SA: %08x\n", 			omap_readl(0x48307220));
    printk(KERN_INFO "PRM_VC_SMPS_VOL_RA: %08x\n", 		omap_readl(0x48307224));
    printk(KERN_INFO "PRM_VC_SMPS_CMD_RA: %08x\n", 		omap_readl(0x48307228));
    printk(KERN_INFO "PRM_VC_CMD_VAL_0: %08x\n", 		omap_readl(0x4830722C));
    printk(KERN_INFO "PRM_VC_CMD_VAL_1: %08x\n", 		omap_readl(0x48307230));
    printk(KERN_INFO "PRM_VC_CH_CONF: %08x\n", 			omap_readl(0x48307234));
    printk(KERN_INFO "PRM_VC_I2C_CFG: %08x\n", 			omap_readl(0x48307238));
    printk(KERN_INFO "PRM_VC_BYPASS_VAL: %08x\n", 		omap_readl(0x4830723C));
    printk(KERN_INFO "PRM_RSTCTRL: %08x\n", 			omap_readl(0x48307250));
    printk(KERN_INFO "PRM_RSTTIME: %08x\n", 			omap_readl(0x48307254));
    printk(KERN_INFO "PRM_RSTST: %08x\n", 				omap_readl(0x48307258));
    printk(KERN_INFO "PRM_VOLTCTRL: %08x\n", 			omap_readl(0x48307260));
    printk(KERN_INFO "PRM_SRAM_PCHARGE: %08x\n", 		omap_readl(0x48307264));
    printk(KERN_INFO "PRM_CLKSRC_CTRL: %08x\n", 		omap_readl(0x48307270));
    printk(KERN_INFO "PRM_OBS: %08x\n", 				omap_readl(0x48307280));
    printk(KERN_INFO "PRM_VOLTSETUP1: %08x\n", 			omap_readl(0x48307290));
    printk(KERN_INFO "PRM_VOLTOFFSET: %08x\n", 			omap_readl(0x48307294));
    printk(KERN_INFO "PRM_CLKSETUP: %08x\n", 			omap_readl(0x48307298));
    printk(KERN_INFO "PRM_POLCTRL: %08x\n", 			omap_readl(0x4830729C));
    printk(KERN_INFO "PRM_VOLTSETUP2: %08x\n", 			omap_readl(0x483072A0));
    printk(KERN_INFO "PRM_VP1_CONFIG: %08x\n", 			omap_readl(0x483072B0));
    printk(KERN_INFO "PRM_VP1_VSTEPMIN: %08x\n", 		omap_readl(0x483072B4));
    printk(KERN_INFO "PRM_VP1_VSTEPMAX: %08x\n", 		omap_readl(0x483072B8));
    printk(KERN_INFO "PRM_VP1_VLIMITTO: %08x\n", 		omap_readl(0x483072BC));
    printk(KERN_INFO "PRM_VP1_VOLTAGE: %08x\n", 		omap_readl(0x483072C0));
    printk(KERN_INFO "PRM_VP1_STATUS: %08x\n", 			omap_readl(0x483072C4));
    printk(KERN_INFO "PRM_VP2_CONFIG: %08x\n", 			omap_readl(0x483072D0));
    printk(KERN_INFO "PRM_VP2_VSTEPMIN: %08x\n", 		omap_readl(0x483072D4));
    printk(KERN_INFO "PRM_VP2_VSTEPMAX: %08x\n", 		omap_readl(0x483072D8));
    printk(KERN_INFO "PRM_VP2_VLIMITTO: %08x\n", 		omap_readl(0x483072DC));
    printk(KERN_INFO "PRM_VP2_VOLTAGE: %08x\n", 		omap_readl(0x483072E0));
    printk(KERN_INFO "PRM_VP2_STATUS: %08x\n", 			omap_readl(0x483072E4));
    printk(KERN_INFO "PRM_LDO_ABB_SETUP: %08x\n", 		omap_readl(0x483072F0));
    printk(KERN_INFO "PRM_LDO_ABB_CTRL: %08x\n", 		omap_readl(0x483072F4));

    printk(KERN_INFO "\NEON_PRM:\n");
    printk(KERN_INFO "RM_RSTST_NEON: %08x\n", 			omap_readl(0x48307358));
    printk(KERN_INFO "PM_WKDEP_NEON: %08x\n", 			omap_readl(0x483073C8));
    printk(KERN_INFO "PM_PWSTCTRL_NEON: %08x\n", 		omap_readl(0x483073E0));
    printk(KERN_INFO "PM_PWSTST_NEON: %08x\n", 			omap_readl(0x483073E4));
    printk(KERN_INFO "PM_PREPWSTST_NEON: %08x\n", 		omap_readl(0x483073E8));

    printk(KERN_INFO "\nUSBHOST_PRM:\n");
    printk(KERN_INFO "RM_RSTST_USBHOST: %08x\n", 		omap_readl(0x48307458));
    printk(KERN_INFO "PM_WKEN_USBHOST: %08x\n", 		omap_readl(0x483074A0));
    printk(KERN_INFO "PM_MPUGRPSEL_USBHOST: %08x\n", 	omap_readl(0x483074A4));
    printk(KERN_INFO "PM_IVA2GRPSEL_USBHOST: %08x\n", 	omap_readl(0x483074A8));
    printk(KERN_INFO "PM_WKST_USBHOST: %08x\n", 		omap_readl(0x483074B0));
    printk(KERN_INFO "PM_WKDEP_USBHOST: %08x\n", 		omap_readl(0x483074C8));
    printk(KERN_INFO "PM_PWSTCTRL_USBHOST: %08x\n", 	omap_readl(0x483074E0));
    printk(KERN_INFO "PM_PWSTST_USBHOST: %08x\n", 		omap_readl(0x483074E4));
    printk(KERN_INFO "PM_PREPWSTST_USBHOST: %08x\n", 	omap_readl(0x483074E8));

    printk(KERN_INFO "CONTROL_PROG_IO_WKUP1: %08x\n", 	omap_readl(0x48002A80));
    printk(KERN_INFO "CONTROL_SYSCONFIG: %08x\n",   	omap_readl(0x48002010));
    //printk(KERN_INFO "SYSC_REG_UART3: %08x\n",          omap_readl(0x49020054));
    //printk(KERN_INFO "SYSC_REG_UART4: %08x\n",          omap_readl(0x49042054));

}

void pm_nextstates_reg_dump() {
    printk(KERN_INFO "\nPOWERDOMAIN NEXT STATES:\n");
    printk(KERN_INFO "PM_PWSTCTRL_MPU: 0x%08x\n",       omap_readl(0x483069E0));
    printk(KERN_INFO "PM_PWSTCTRL_CORE: 0x%08x\n",      omap_readl(0x48306AE0));
    printk(KERN_INFO "PM_PWSTCTRL_SGX: 0x%08x\n",       omap_readl(0x48306BE0));
    printk(KERN_INFO "PM_PWSTCTRL_DSS: 0x%08x\n",       omap_readl(0x48306EE0));
    printk(KERN_INFO "PM_PWSTCTRL_CAM: 0x%08x\n",       omap_readl(0x48306FE0));
    printk(KERN_INFO "PM_PWSTCTRL_PER: 0x%08x\n",       omap_readl(0x483070E0));
    printk(KERN_INFO "PM_PWSTCTRL_NEON: 0x%08x\n",      omap_readl(0x483073E0));
    printk(KERN_INFO "PM_PWSTCTRL_IVA2: 0x%08x\n",      omap_readl(0x483060E0));
    printk(KERN_INFO "PM_PWSTCTRL_USBHOST: 0x%08x\n",   omap_readl(0x483074E0));
}

void pm_currstates_reg_dump() {
    printk(KERN_INFO "\nPOWERDOMAIN CURR STATES:\n");
    printk(KERN_INFO "PM_PWSTST_MPU: 0x%08x\n",       omap_readl(0x483069E4));
    printk(KERN_INFO "PM_PWSTST_CORE: 0x%08x\n",      omap_readl(0x48306AE4));
    printk(KERN_INFO "PM_PWSTST_SGX: 0x%08x\n",       omap_readl(0x48306BE4));
    printk(KERN_INFO "PM_PWSTST_DSS: 0x%08x\n",       omap_readl(0x48306EE4));
    printk(KERN_INFO "PM_PWSTST_CAM: 0x%08x\n",       omap_readl(0x48306FE4));
    printk(KERN_INFO "PM_PWSTST_PER: 0x%08x\n",       omap_readl(0x483070E4));
    printk(KERN_INFO "PM_PWSTST_NEON: 0x%08x\n",      omap_readl(0x483073E4));
    printk(KERN_INFO "PM_PWSTST_IVA2: 0x%08x\n",      omap_readl(0x483060E4));
    printk(KERN_INFO "PM_PWSTST_USBHOST: 0x%08x\n",   omap_readl(0x483074E4));
}

void pm_prevstates_reg_dump() {
    printk(KERN_INFO "\nPOWERDOMAIN PREV STATES:\n");
    printk(KERN_INFO "PM_PREPWSTST_MPU: 0x%08x\n",       omap_readl(0x483069E8));
    printk(KERN_INFO "PM_PREPWSTST_CORE: 0x%08x\n",      omap_readl(0x48306AE8));
    printk(KERN_INFO "PM_PREPWSTST_SGX: 0x%08x\n",       omap_readl(0x48306BE8));
    printk(KERN_INFO "PM_PREPWSTST_DSS: 0x%08x\n",       omap_readl(0x48306EE8));
    printk(KERN_INFO "PM_PREPWSTST_CAM: 0x%08x\n",       omap_readl(0x48306FE8));
    printk(KERN_INFO "PM_PREPWSTST_PER: 0x%08x\n",       omap_readl(0x483070E8));
    printk(KERN_INFO "PM_PREPWSTST_NEON: 0x%08x\n",      omap_readl(0x483073E8));
    printk(KERN_INFO "PM_PREPWSTST_IVA2: 0x%08x\n",      omap_readl(0x483060E8));
    printk(KERN_INFO "PM_PREPWSTST_USBHOST: 0x%08x\n",   omap_readl(0x483074E8));
}

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	int mpu_next_state = PWRDM_POWER_ON;
	int per_next_state = PWRDM_POWER_ON;
	int core_next_state = PWRDM_POWER_ON;
	int per_going_off;
	int core_prev_state, per_prev_state;
    u32 sdrc_pwr = 0;

    int nextstate;
    u32 val;

    omap3_pm_off_mode_enable(1);
    omap_clk_enable_autoidle_all();

    if (!_omap_sram_idle)
		return;

	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);

	mpu_next_state = pwrdm_read_next_pwrst(mpu_pwrdm);
	switch (mpu_next_state) {
	case PWRDM_POWER_ON:
	case PWRDM_POWER_RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PWRDM_POWER_OFF:
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}
	pwrdm_pre_transition();

	/* NEON control */
	if (pwrdm_read_pwrst(neon_pwrdm) == PWRDM_POWER_ON)
		pwrdm_set_next_pwrst(neon_pwrdm, mpu_next_state);

    /* Enable IO-PAD and IO-CHAIN wakeups */
    per_next_state = pwrdm_read_next_pwrst(per_pwrdm);
    core_next_state = pwrdm_read_next_pwrst(core_pwrdm);
    if (omap3_has_io_wakeup() &&
        (per_next_state < PWRDM_POWER_ON ||
         core_next_state < PWRDM_POWER_ON)) {
        omap2_prm_set_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD, PM_WKEN);
        omap3_enable_io_chain();
    }

	/* Block console output in case it is on one of the OMAP UARTs */
	if (!is_suspending())
		if (per_next_state < PWRDM_POWER_ON ||
		    core_next_state < PWRDM_POWER_ON)
			if (!console_trylock())
				goto console_still_active;

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		per_going_off = (per_next_state == PWRDM_POWER_OFF) ? 1 : 0;
		omap_uart_prepare_idle(2);
        omap_uart_prepare_idle(3);
		omap2_gpio_prepare_for_idle(per_going_off);
		if (per_next_state == PWRDM_POWER_OFF)
				omap3_per_save_context();
	}

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		omap_uart_prepare_idle(0);
		omap_uart_prepare_idle(1);
		if (core_next_state == PWRDM_POWER_OFF) {
			omap3_core_save_context();
			omap3_cm_save_context();
		}
	}

    omap3_intc_prepare_idle();

	/*
	* On EMU/HS devices ROM code restores a SRDC value
	* from scratchpad which has automatic self refresh on timeout
	* of AUTO_CNT = 1 enabled. This takes care of erratum ID i443.
	* Hence store/restore the SDRC_POWER register here.
	*/
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_pwr = sdrc_read_reg(SDRC_POWER);

	/*
	 * omap3_arm_context is the location where ARM registers
	 * get saved. The restore path then reads from this
	 * location and restores them back.
	 */

    // Disable HSOTGUSB
    val = omap_readl(0x48004A10);
    val &= ~ (1 << 4);
    omap_writel(val, 0x48004A10);

    omap_writel(0x400, 0x49054060);
    omap_writel(0, 0x480AB404);
    omap_writel(1, 0x480AB414);


//    printk(KERN_INFO "NEW CM_ICLKEN1_CORE: 0x%08x\n", val);

    val = omap_readl(0x48306AA0);
    val &= ~ (1 << 4);
    omap_writel(val, 0x48306AA0);

    val = omap_readl(0x48306AA4);
    val &= ~ (1 << 4);
    omap_writel(val, 0x48306AA4);



    gpio4_snapshot();
    HSUSBOTG_snapshot();
    //reg_snapshot();

   // dss_suspend_all_devices();
    //dss_snapshot();
	_omap_sram_idle(omap3_arm_context, save_state);
	cpu_init();

	/* Restore normal SDRC POWER settings */
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_write_reg(sdrc_pwr, SDRC_POWER);

	/* Restore table entry modified during MMU restoration */
	if (pwrdm_read_prev_pwrst(mpu_pwrdm) == PWRDM_POWER_OFF)
		restore_table_entry();

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		core_prev_state = pwrdm_read_prev_pwrst(core_pwrdm);
		if (core_prev_state == PWRDM_POWER_OFF) {
			omap3_core_restore_context();
			omap3_cm_restore_context();
			omap3_sram_restore_context();
			omap2_sms_restore_context();
		}
		omap_uart_resume_idle(0);
		omap_uart_resume_idle(1);
        // if (core_next_state == PWRDM_POWER_OFF)  omap_writel(0x8, 0x48307260);
//           omap2_prm_clear_mod_reg_bits(OMAP3430_AUTO_OFF_MASK, OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
    }
	omap3_intc_resume_idle();

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		per_prev_state = pwrdm_read_prev_pwrst(per_pwrdm);
		omap2_gpio_resume_after_idle();
		if (per_prev_state == PWRDM_POWER_OFF)
			omap3_per_restore_context();
		omap_uart_resume_idle(2);
		omap_uart_resume_idle(3);
	}

	if (!is_suspending())
		console_unlock();

    printk(KERN_INFO "\nWAKEUPSTATUS:\n");
    printk(KERN_INFO "PM_WKST1_CORE: 0x%08x\n",     omap_readl(0x48306AB0));
    printk(KERN_INFO "PM_WKST3_CORE: 0x%08x\n",     omap_readl(0x48306AB8));
    printk(KERN_INFO "PM_WKST_WKUP: 0x%08x\n",      omap_readl(0x48306CB0));
    printk(KERN_INFO "PM_WKST_PER: 0x%08x\n",       omap_readl(0x483070B0));
    printk(KERN_INFO "PM_WKST_USBHOST: 0x%08x\n",   omap_readl(0x483074B0));


   // dss_resume_all_devices();

    pm_prevstates_reg_dump();

    //dss_snapshot();
    //dss_restore_context();
    //clockConf();

console_still_active:
	/* Disable IO-PAD and IO-CHAIN wakeup */
	if (omap3_has_io_wakeup() &&
	    (per_next_state < PWRDM_POWER_ON ||
	     core_next_state < PWRDM_POWER_ON)) {
		omap2_prm_clear_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD,
					     PM_WKEN);
		omap3_disable_io_chain();
	}

	pwrdm_post_transition();

    clkdm_allow_idle(mpu_pwrdm->pwrdm_clkdms[0]);
}

int omap3_can_sleep(void)
{
	if (!sleep_while_idle)
		return 0;
	if (!omap_uart_can_sleep())
		return 0;
	return 1;
}

static void omap3_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();

	if (!omap3_can_sleep())
		goto out;

	if (omap_irq_pending() || need_resched())
		goto out;

	trace_power_start(POWER_CSTATE, 1, smp_processor_id());
	trace_cpu_idle(1, smp_processor_id());

    omap_sram_idle();

	trace_power_end(smp_processor_id());
	trace_cpu_idle(PWR_EVENT_EXIT, smp_processor_id());

out:
	local_fiq_enable();
	local_irq_enable();
}

#ifdef CONFIG_SUSPEND
static int omap3_pm_suspend(void)
{
	struct power_state *pwrst;
	int state, ret = 0;

    //disable_irq(266);

	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node)
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
	/* Set ones wanted by suspend */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (omap_set_pwrdm_state(pwrst->pwrdm, pwrst->next_state))
			goto restore;
		if (pwrdm_clear_all_prev_pwrst(pwrst->pwrdm))
			goto restore;
	}

	omap_uart_prepare_suspend();
	omap3_intc_suspend();

	omap_sram_idle();

restore:
	/* Restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		if (state > pwrst->next_state) {
			printk(KERN_INFO "Powerdomain (%s) didn't enter "
			       "target state %d\n",
			       pwrst->pwrdm->name, pwrst->next_state);
			ret = -1;
		}
		omap_set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
	}
	if (ret)
		printk(KERN_ERR "Could not enter target state in pm_suspend\n");
	else
		printk(KERN_INFO "Successfully put all powerdomains "
		       "to target state\n");


    omap_writel(0x2, 0x483060F8);
    printk(KERN_INFO "IVA_WKST: 0x%08x\n",   omap_readl(0x483060F8));
    omap_writel(0x00107d17, 0x48004940);
    printk(KERN_INFO "CM_CLKSEL1_PLL_MPU: 0x%08x\n",   omap_readl(0x48004940));
	return ret;
}

static int omap3_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:

       //reg_snapshot();
		ret = omap3_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* Hooks to enable / disable UART interrupts during suspend */
static int omap3_pm_begin(suspend_state_t state)
{
	disable_hlt();
	suspend_state = state;
	omap_uart_enable_irqs(0);
	return 0;
}

static void omap3_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
	omap_uart_enable_irqs(1);
	enable_hlt();
	return;
}

static const struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap3_pm_begin,
	.end		= omap3_pm_end,
	.enter		= omap3_pm_enter,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */


/**
 * omap3_iva_idle(): ensure IVA is in idle so it can be put into
 *                   retention
 *
 * In cases where IVA2 is activated by bootcode, it may prevent
 * full-chip retention or off-mode because it is not idle.  This
 * function forces the IVA2 into idle state so it can go
 * into retention/off and thus allow full-chip retention/off.
 *
 **/
static void __init omap3_iva_idle(void)
{
	/* ensure IVA2 clock is disabled */
	omap2_cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* if no clock activity, nothing else to do */
	if (!(omap2_cm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_CM_CLKSTST) &
	      OMAP3430_CLKACTIVITY_IVA2_MASK))
		return;

	/* Reset IVA2 */
	omap2_prm_write_mod_reg(OMAP3430_RST1_IVA2_MASK |
			  OMAP3430_RST2_IVA2_MASK |
			  OMAP3430_RST3_IVA2_MASK,
			  OMAP3430_IVA2_MOD, OMAP2_RM_RSTCTRL);

	/* Enable IVA2 clock */
	omap2_cm_write_mod_reg(OMAP3430_CM_FCLKEN_IVA2_EN_IVA2_MASK,
			 OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Set IVA2 boot mode to 'idle' */
	omap_ctrl_writel(OMAP3_IVA2_BOOTMOD_IDLE,
			 OMAP343X_CONTROL_IVA2_BOOTMOD);

	/* Un-reset IVA2 */
	omap2_prm_write_mod_reg(0, OMAP3430_IVA2_MOD, OMAP2_RM_RSTCTRL);

	/* Disable IVA2 clock */
	omap2_cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Reset IVA2 */
	omap2_prm_write_mod_reg(OMAP3430_RST1_IVA2_MASK |
			  OMAP3430_RST2_IVA2_MASK |
			  OMAP3430_RST3_IVA2_MASK,
			  OMAP3430_IVA2_MOD, OMAP2_RM_RSTCTRL);
}

static void __init omap3_d2d_idle(void)
{
	u16 mask, padconf;

	/* In a stand alone OMAP3430 where there is not a stacked
	 * modem for the D2D Idle Ack and D2D MStandby must be pulled
	 * high. S CONTROL_PADCONF_SAD2D_IDLEACK and
	 * CONTROL_PADCONF_SAD2D_MSTDBY to have a pull up. */
	mask = (1 << 4) | (1 << 3); /* pull-up, enabled */
	padconf = omap_ctrl_readw(OMAP3_PADCONF_SAD2D_MSTANDBY);
	padconf |= mask;
	omap_ctrl_writew(padconf, OMAP3_PADCONF_SAD2D_MSTANDBY);

	padconf = omap_ctrl_readw(OMAP3_PADCONF_SAD2D_IDLEACK);
	padconf |= mask;
	omap_ctrl_writew(padconf, OMAP3_PADCONF_SAD2D_IDLEACK);

	/* reset modem */
	omap2_prm_write_mod_reg(OMAP3430_RM_RSTCTRL_CORE_MODEM_SW_RSTPWRON_MASK |
			  OMAP3430_RM_RSTCTRL_CORE_MODEM_SW_RST_MASK,
			  CORE_MOD, OMAP2_RM_RSTCTRL);
	omap2_prm_write_mod_reg(0, CORE_MOD, OMAP2_RM_RSTCTRL);
}

static void __init prcm_setup_regs(void)
{
	u32 omap3630_en_uart4_mask = cpu_is_omap3630() ?
					OMAP3630_EN_UART4_MASK : 0;
	u32 omap3630_grpsel_uart4_mask = cpu_is_omap3630() ?
					OMAP3630_GRPSEL_UART4_MASK : 0;

    u32 val;

    /* XXX Reset all wkdeps. This should be done when initializing
         * powerdomains */
        omap2_prm_write_mod_reg(0, OMAP3430_IVA2_MOD, PM_WKDEP);
        omap2_prm_write_mod_reg(0, MPU_MOD, PM_WKDEP);
        omap2_prm_write_mod_reg(0, OMAP3430_DSS_MOD, PM_WKDEP);
        omap2_prm_write_mod_reg(0, OMAP3430_NEON_MOD, PM_WKDEP);
        omap2_prm_write_mod_reg(0, OMAP3430_CAM_MOD, PM_WKDEP);
        omap2_prm_write_mod_reg(0, OMAP3430_PER_MOD, PM_WKDEP);
        if (omap_rev() > OMAP3430_REV_ES1_0) {
            omap2_prm_write_mod_reg(0, OMAP3430ES2_SGX_MOD, PM_WKDEP);
            omap2_prm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD, PM_WKDEP);
        } else
            omap2_prm_write_mod_reg(0, GFX_MOD, PM_WKDEP);

	/* XXX This should be handled by hwmod code or SCM init code */
    omap_ctrl_writel(OMAP3430_AUTOIDLE_MASK, OMAP2_CONTROL_SYSCONFIG);

	/*
	 * Enable control of expternal oscillator through
	 * sys_clkreq. In the long run clock framework should
	 * take care of this.
	 */
	omap2_prm_rmw_mod_reg_bits(OMAP_AUTOEXTCLKMODE_MASK,
			     1 << OMAP_AUTOEXTCLKMODE_SHIFT,
			     OMAP3430_GR_MOD,
			     OMAP3_PRM_CLKSRC_CTRL_OFFSET);

	/* setup wakup source */
    omap2_prm_write_mod_reg(OMAP3430_EN_IO_MASK | OMAP3430_EN_GPIO1_MASK /*|
              OMAP3430_EN_GPT1_MASK | OMAP3430_EN_GPT12_MASK*/,
			  WKUP_MOD, PM_WKEN);
	/* No need to write EN_IO, that is always enabled */
    omap2_prm_write_mod_reg(OMAP3430_GRPSEL_GPIO1_MASK/* |
			  OMAP3430_GRPSEL_GPT1_MASK |
              OMAP3430_GRPSEL_GPT12_MASK*/,
			  WKUP_MOD, OMAP3430_PM_MPUGRPSEL);

    //omap_writel(0, 0x48306CA0);
    /* For some reason IO doesn't generate wakeup event even if
	 * it is selected to mpu wakeup goup */
	omap2_prm_write_mod_reg(OMAP3430_IO_EN_MASK | OMAP3430_WKUP_EN_MASK,
			  OCP_MOD, OMAP3_PRM_IRQENABLE_MPU_OFFSET);

	/* Enable PM_WKEN to support DSS LPR */
	omap2_prm_write_mod_reg(OMAP3430_PM_WKEN_DSS_EN_DSS_MASK,
				OMAP3430_DSS_MOD, PM_WKEN);

	/* Enable wakeups in PER */
	omap2_prm_write_mod_reg(omap3630_en_uart4_mask |
			  OMAP3430_EN_GPIO2_MASK | OMAP3430_EN_GPIO3_MASK |
			  OMAP3430_EN_GPIO4_MASK | OMAP3430_EN_GPIO5_MASK |
			  OMAP3430_EN_GPIO6_MASK | OMAP3430_EN_UART3_MASK |
			  OMAP3430_EN_MCBSP2_MASK | OMAP3430_EN_MCBSP3_MASK |
			  OMAP3430_EN_MCBSP4_MASK,
			  OMAP3430_PER_MOD, PM_WKEN);
	/* and allow them to wake up MPU */
	omap2_prm_write_mod_reg(omap3630_grpsel_uart4_mask |
			  OMAP3430_GRPSEL_GPIO2_MASK |
			  OMAP3430_GRPSEL_GPIO3_MASK |
			  OMAP3430_GRPSEL_GPIO4_MASK |
			  OMAP3430_GRPSEL_GPIO5_MASK |
			  OMAP3430_GRPSEL_GPIO6_MASK |
			  OMAP3430_GRPSEL_UART3_MASK |
			  OMAP3430_GRPSEL_MCBSP2_MASK |
			  OMAP3430_GRPSEL_MCBSP3_MASK |
			  OMAP3430_GRPSEL_MCBSP4_MASK,
			  OMAP3430_PER_MOD, OMAP3430_PM_MPUGRPSEL);

	/* Don't attach IVA interrupts */
	omap2_prm_write_mod_reg(0, WKUP_MOD, OMAP3430_PM_IVAGRPSEL);
	omap2_prm_write_mod_reg(0, CORE_MOD, OMAP3430_PM_IVAGRPSEL1);
	omap2_prm_write_mod_reg(0, CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3);
	omap2_prm_write_mod_reg(0, OMAP3430_PER_MOD, OMAP3430_PM_IVAGRPSEL);

	/* Clear any pending 'reset' flags */
	omap2_prm_write_mod_reg(0xffffffff, MPU_MOD, OMAP2_RM_RSTST);
	omap2_prm_write_mod_reg(0xffffffff, CORE_MOD, OMAP2_RM_RSTST);
	omap2_prm_write_mod_reg(0xffffffff, OMAP3430_PER_MOD, OMAP2_RM_RSTST);
	omap2_prm_write_mod_reg(0xffffffff, OMAP3430_EMU_MOD, OMAP2_RM_RSTST);
	omap2_prm_write_mod_reg(0xffffffff, OMAP3430_NEON_MOD, OMAP2_RM_RSTST);
	omap2_prm_write_mod_reg(0xffffffff, OMAP3430_DSS_MOD, OMAP2_RM_RSTST);
	omap2_prm_write_mod_reg(0xffffffff, OMAP3430ES2_USBHOST_MOD, OMAP2_RM_RSTST);

	/* Clear any pending PRCM interrupts */
	omap2_prm_write_mod_reg(0, OCP_MOD, OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

	omap3_iva_idle();
	omap3_d2d_idle();

}

void omap3_pm_off_mode_enable(int enable)
{
	struct power_state *pwrst;
	u32 state;

	if (enable)
		state = PWRDM_POWER_OFF;
	else
		state = PWRDM_POWER_RET;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (IS_PM34XX_ERRATUM(PM_SDRC_WAKEUP_ERRATUM_i583) &&
				pwrst->pwrdm == core_pwrdm &&
				state == PWRDM_POWER_OFF) {
			pwrst->next_state = PWRDM_POWER_RET;
			pr_warn("%s: Core OFF disabled due to errata i583\n",
				__func__);
		} else {
			pwrst->next_state = state;
		}
		omap_set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
	}
}

int omap3_pm_get_suspend_state(struct powerdomain *pwrdm)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm)
			return pwrst->next_state;
	}
	return -EINVAL;
}

int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm) {
			pwrst->next_state = state;
			return 0;
		}
	}
	return -EINVAL;
}

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_ATOMIC);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
    pwrst->next_state = PWRDM_POWER_RET;
	list_add(&pwrst->node, &pwrst_list);

	if (pwrdm_has_hdwr_sar(pwrdm))
		pwrdm_enable_hdwr_sar(pwrdm);

    return omap_set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
}

/*
 * Enable hw supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __init clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
		 atomic_read(&clkdm->usecount) == 0)
		clkdm_sleep(clkdm);
	return 0;
}

void omap_push_sram_idle(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_cpu_suspend,
					omap34xx_cpu_suspend_sz);
	if (omap_type() != OMAP2_DEVICE_TYPE_GP)
		_omap_save_secure_sram = omap_sram_push(save_secure_ram_context,
				save_secure_ram_context_sz);
}

static void __init pm_errata_configure(void)
{
	if (cpu_is_omap3630()) {
		pm34xx_errata |= PM_RTA_ERRATUM_i608;
		/* Enable the l2 cache toggling in sleep logic */
		enable_omap3630_toggle_l2_on_restore();
		if (omap_rev() < OMAP3630_REV_ES1_2)
			pm34xx_errata |= PM_SDRC_WAKEUP_ERRATUM_i583;
	}
}

static int __init omap3_pm_init(void)
{
	struct power_state *pwrst, *tmp;
	struct clockdomain *neon_clkdm, *per_clkdm, *mpu_clkdm, *core_clkdm;
	int ret;

	if (!cpu_is_omap34xx())
		return -ENODEV;

	pm_errata_configure();

	/* XXX prcm_setup_regs needs to be before enabling hw
	 * supervised mode for powerdomains */
	prcm_setup_regs();

    ret = request_irq(INT_34XX_PRCM_MPU_IRQ,
			  (irq_handler_t)prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       INT_34XX_PRCM_MPU_IRQ);
		goto err1;
	}

	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		printk(KERN_ERR "Failed to setup powerdomains\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);

	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	if (mpu_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get mpu_pwrdm\n");
		goto err2;
	}

	neon_pwrdm = pwrdm_lookup("neon_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");
	core_pwrdm = pwrdm_lookup("core_pwrdm");
	cam_pwrdm = pwrdm_lookup("cam_pwrdm");

    dss_pwrdm = pwrdm_lookup("dss_pwrdm");
    sgx_pwrdm = pwrdm_lookup("sgx_pwrdm");
    iva2_pwrdm = pwrdm_lookup("iva2_pwrdm");

    emu_pwrdm = pwrdm_lookup("emu_pwrdm");
    usbhost_pwrdm = pwrdm_lookup("usbhost_pwrdm");

	neon_clkdm = clkdm_lookup("neon_clkdm");
	mpu_clkdm = clkdm_lookup("mpu_clkdm");
	per_clkdm = clkdm_lookup("per_clkdm");
	core_clkdm = clkdm_lookup("core_clkdm");

	omap_push_sram_idle();
#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	pm_idle = omap3_pm_idle;
	omap3_idle_init();

	/*
	 * RTA is disabled during initialization as per erratum i608
	 * it is safer to disable RTA by the bootloader, but we would like
	 * to be doubly sure here and prevent any mishaps.
	 */
	if (IS_PM34XX_ERRATUM(PM_RTA_ERRATUM_i608))
		omap3630_ctrl_disable_rta();

	clkdm_add_wkdep(neon_clkdm, mpu_clkdm);
	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		omap3_secure_ram_storage =
			kmalloc(0x803F, GFP_KERNEL);
		if (!omap3_secure_ram_storage)
			printk(KERN_ERR "Memory allocation failed when"
					"allocating for secure sram context\n");

		local_irq_disable();
		local_fiq_disable();

		omap_dma_global_context_save();
		omap3_save_secure_ram_context();
		omap_dma_global_context_restore();

		local_irq_enable();
		local_fiq_enable();
	}

	omap3_save_scratchpad_contents();

    clockConf();

    pm_currstates_reg_dump();
err1:
	return ret;
err2:
	free_irq(INT_34XX_PRCM_MPU_IRQ, NULL);
	list_for_each_entry_safe(pwrst, tmp, &pwrst_list, node) {
		list_del(&pwrst->node);
		kfree(pwrst);
	}
	return ret;
}

static void omap3logic_pm_init(void)
{
    printk(KERN_INFO "\n omap3logic_pm_init \n");

    /* Using sys_offmode signal */
    omap_pm_sys_offmode_select(1);

    /* sys_clkreq - active high */
    omap_pm_sys_clkreq_pol(1);

    /* sys_offmode - active low */
    omap_pm_sys_offmode_pol(1);

    /* Automatically send OFF command */
    omap_pm_auto_off(1);

    /* Automatically send RET command */
    omap_pm_auto_ret(0);
}


/** Select whether OFF command is sent via I2C
 * 1 - Yes. Command is automatically send when the voltage
 *     domain is in the appropriate standby mode.
 * 0 - No. Command is not sent
 */
void omap_pm_auto_off(int flag)
{
    if (flag)
        omap2_prm_set_mod_reg_bits(OMAP3430_AUTO_OFF_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
    else
        omap2_prm_clear_mod_reg_bits(OMAP3430_AUTO_OFF_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
}

/**
 * Select whether RET command is sent via I2C
 * 1 - Yes. Command is automatically send when the voltage
 *     domain is in the appropriate standby mode.
 * 0 - No. Command is not sent
 */
void omap_pm_auto_ret(int flag)
{
    if (flag)
        omap2_prm_set_mod_reg_bits(OMAP3430_AUTO_RET_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
    else
        omap2_prm_clear_mod_reg_bits(OMAP3430_AUTO_RET_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
}

/**
 * Select whether sys_offmode is asserted
 * 1 - Yes. sys_offmode is asserted
 * 0 - No. OFF command is sent through I2C
 */
void omap_pm_sys_offmode_select(int flag)
{
    if (flag)
        omap2_prm_set_mod_reg_bits(OMAP3430_SEL_OFF_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
    else
        omap2_prm_clear_mod_reg_bits(OMAP3430_SEL_OFF_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);
}

/**
 * Select the polarity of sys_offmode signal
 * 1 - sys_offmode is active high
 * 0 - sys_offmode is active low
 */
void omap_pm_sys_offmode_pol(int flag)
{
    if (flag)
        omap2_prm_set_mod_reg_bits(OMAP3430_OFFMODE_POL_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_POLCTRL_OFFSET);
    else
        omap2_prm_clear_mod_reg_bits(OMAP3430_OFFMODE_POL_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_POLCTRL_OFFSET);
}

/**
 * Select the polarity of sys_clkreq signal
 * 1 - sys_clkreq is active high
 * 0 - sys_clkreq is active low
 */
void omap_pm_sys_clkreq_pol(int flag)
{
    if (flag)
        omap2_prm_set_mod_reg_bits(OMAP3430_CLKREQ_POL_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_POLCTRL_OFFSET);
    else
        omap2_prm_clear_mod_reg_bits(OMAP3430_CLKREQ_POL_MASK,
                    OMAP3430_GR_MOD, OMAP3_PRM_POLCTRL_OFFSET);
 }



late_initcall(omap3_pm_init);
