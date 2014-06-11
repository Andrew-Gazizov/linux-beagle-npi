/*
 * linux/arch/arm/mach-omap2/board-omap3npi.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-omap3beagle.c
 *
 * Initial code: ~compass group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/opp.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/i2c/tsc2007.h>
#include <linux/spi/spi.h>
//#include <linux/input/max7359.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <mach/id.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <linux/spi/ads7846.h>
#include <plat/mcspi.h>
#include <linux/i2c/ads1015.h>
#include <linux/mfd/stmpe.h>
#include <linux/platform_data/sc16is7x2.h>

#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include <plat/pwm.h>
#include <linux/pwm_backlight.h>
#include "common-board-devices.h"
#include <linux/i2c/twl.h>

#include "prm-regbits-34xx.h"

#include "sdram-micron-mt46h32m32lf-6.h"

/*
 * OMAP3 Beagle revision
 * Run time detection of Beagle revision is done by reading GPIO.
 * GPIO ID -
 *	AXBX	= GPIO173, GPIO172, GPIO171: 1 1 1
 *	C1_3	= GPIO173, GPIO172, GPIO171: 1 1 0
 *	C4	= GPIO173, GPIO172, GPIO171: 1 0 1
 *	XMA/XMB = GPIO173, GPIO172, GPIO171: 0 0 0
 *	XMC = GPIO173, GPIO172, GPIO171: 0 1 0
 */
enum {
    OMAP3BEAGLE_BOARD_UNKN = 0,
    OMAP3BEAGLE_BOARD_AXBX,
    OMAP3BEAGLE_BOARD_C1_3,
    OMAP3BEAGLE_BOARD_C4,
    OMAP3BEAGLE_BOARD_XM,
    OMAP3BEAGLE_BOARD_XMC,
};

static u8 omap3_beagle_version;

#define POWEROFF_GPIO 103
//#define	GPIO_MAX7359_IRQ	156

/*
 * Board-specific configuration
 * Defaults to NPIBoard
 */
static struct {
    int mmc1_gpio_wp;
    int usb_pwr_level;
    int reset_gpio;
    int usr_button_gpio;
    char *lcd_driver_name;
    int lcd_pwren;
} npi_config = {
    .mmc1_gpio_wp = -EINVAL,
    .usb_pwr_level = GPIOF_OUT_INIT_LOW,
    .reset_gpio = 98/*129*/,
    .usr_button_gpio = 4,
    .lcd_driver_name = "nec_2432_panel",
    .lcd_pwren = 110
};

//#define DEFINE_KEY(row, col, keycode) ((row << 24) | (col << 16) | keycode)

static struct stmpe_ts_platform_data stmpe811_ts_data = {
    .sample_time = 4,
    .mod_12b = 1,
    .ref_sel = 0,
    .adc_freq = 1,
    .ave_ctrl = 3,
    .touch_det_delay = 3,
    .settling = 3,
    .fraction_z = 7,
    .i_drive = 0,
};

static struct stmpe_platform_data stmpe811_data = {
    .blocks		= STMPE_BLOCK_TOUCHSCREEN,
    //    .irq = OMAP_GPIO_IRQ(106),
    //    .irq_trigger    = IRQF_TRIGGER_FALLING,
    .irq_base = TWL4030_IRQ_END,
    .irq_trigger = IRQF_TRIGGER_RISING,
    .irq_invert_polarity = true,
    .ts		= &stmpe811_ts_data,
    .autosleep      = false,
    .autosleep_timeout = 1024,
    //    .get_pendown_state = omap3npi_stmpe_get_pendown_state,
    .irq_trigger    = IRQF_TRIGGER_FALLING,
};

//static const uint32_t cm_t35_keymap[] = {
//    KEY(0, 0, KEY_ENTER),	KEY(0, 1, KEY_LEFT),	KEY(0, 2, KEY_LEFT),
//    KEY(1, 0, KEY_UP),	KEY(1, 1, KEY_ENTER),	KEY(1, 2, KEY_DOWN),
//    KEY(2, 0, KEY_UP),	KEY(2, 1, KEY_DOWN),	KEY(2, 2, KEY_ENTER),
//    KEY(3, 0, KEY_ENTER),	KEY(3, 1, KEY_RIGHT),	KEY(3, 2, KEY_DOWN),
//};

static uint32_t board_keymap[] = {

    KEY(0, 0, KEY_ENTER),	KEY(0, 1, KEY_UP),	KEY(0, 2, KEY_ENTER),	KEY(0, 3, KEY_6),
    KEY(1, 0, KEY_3),	KEY(1, 1, KEY_DOWN),	KEY(1, 2, KEY_LEFT),	KEY(1, 3, KEY_8),
    KEY(2, 0, KEY_RIGHT),	KEY(2, 1, KEY_1),	KEY(2, 2, KEY_1),	KEY(2, 3, KEY_7),
    KEY(3, 0, KEY_1),	KEY(3, 1, KEY_1),	KEY(3, 2, KEY_RIGHT),	KEY(3, 3, KEY_7),

    //    {{ .code = KEY_7, }, { .code = KEY_8}, { .code = KEY_ENTER},{ .code = KEY_A}},
    //    {{ .code = KEY_B, }, { .code = KEY_UP}, { .code = KEY_6},{ .code = KEY_C}},
    //    {{ .code = KEY_LEFT, }, { .code = KEY_RIGHT}, { .code = KEY_4},{ .code = KEY_D}},
    //    {{ .code = KEY_1, }, { .code = KEY_DOWN}, { .code = KEY_3},{ .code = KEY_E}}

};

static struct matrix_keymap_data board_map_data = {
    .keymap			= board_keymap,
    .keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data cm_t35_kp_data = {
    .keymap_data	= &board_map_data,
    .rows		= 4,
    .cols		= 4,
    .rep		= 1,
    //    .irq        = OMAP_GPIO_IRQ(0),
};

static struct sc16is7x2_platform_data sc16is7x2_SERIALPORT3_data = {
    .uartclk = 1843200,
    .uart_base = 0,
    .gpio_base = 212,
    .label = NULL,
    .names = NULL,
};

//static int __init lpc313x_sc16is7x2_register(void)
//{
//    struct spi_board_info info =
//    {
//            .modalias               = "sc16is7x2",
//        .platform_data          = &sc16is7x2_SERIALPORT3_data,
//        .bus_num                = 0,
//        .irq                    = gpio_to_irq(14),
//        .chip_select            = 11,
//        .max_speed_hz           = 187500,
//        .mode                   = SPI_MODE_0,
//        //.controller_data    = &sc16is7x2_mcspi_config,
//        //.modalias = "sc16is7x2",
//        //.max_speed_hz = 10000000,
//        //.bus_num = 0,
//        //.irq = IRQ_GPIO_14,//IRQ_GPIO14
//        //.chip_select = 1,
//    };

//    return spi_register_board_info(&info, 1);
//}

static struct i2c_board_info __initdata pl_i2c_devices_boardinfo[] = {
    {
        I2C_BOARD_INFO("stmpe811", (0x82>>1)),//0x88
        .irq = OMAP_GPIO_IRQ(106),
        .platform_data = &stmpe811_data,
        //        .flags = I2C_CLIENT_WAKE,
    },
    {
        I2C_BOARD_INFO("sc16is7x2-i2c", 0x4D/*(0x9A>>1)*/),
        .platform_data          = &sc16is7x2_SERIALPORT3_data,
        .irq        = OMAP_GPIO_IRQ(94),
        //        .flags = I2C_CLIENT_WAKE,
    },
};

/*
 * This device path represents the onboard USB <-> Ethernet bridge
 * on the NPIBoard-xM which needs a random or all-zeros
 * mac address replaced with a per-cpu stable generated one
 */

static const char * const xm_fixup_mac_device_paths[] = {
    "usb1/1-2/1-2.1/1-2.1:1.0",
};

void vibra_disable_leds(void)
{
        u8 reg;

        /* Disable LEDA & LEDB, cannot be used with vibra (PWM) */
        twl_i2c_read_u8(TWL4030_MODULE_LED, &reg, 0x00);
        reg = 0x00;
        twl_i2c_write_u8(TWL4030_MODULE_LED, 0x0, reg);
}


static int npi_device_path_need_mac(struct device *dev)
{
    const char **try = (const char **) xm_fixup_mac_device_paths;
    const char *path;
    int count = ARRAY_SIZE(xm_fixup_mac_device_paths);
    const char *p;
    int len;
    struct device *devn;

    while (count--) {

        p = *try + strlen(*try);
        devn = dev;

        while (devn) {

            path = dev_name(devn);
            len = strlen(path);

            if ((p - *try) < len) {
                devn = NULL;
                continue;
            }

            p -= len;

            if (strncmp(path, p, len)) {
                devn = NULL;
                continue;
            }

            devn = devn->parent;
            if (p == *try)
                return count;

            if (devn != NULL && (p - *try) < 2)
                devn = NULL;

            p--;
            if (devn != NULL && *p != '/')
                devn = NULL;
        }

        try++;
    }

    return -ENOENT;
}

static int omap_npi_netdev_event(struct notifier_block *this,
                                 unsigned long event, void *ptr)
{
    struct net_device *dev = ptr;
    struct sockaddr sa;
    int n;

    if (event != NETDEV_REGISTER)
        return NOTIFY_DONE;

    n = npi_device_path_need_mac(dev->dev.parent);
    if (n >= 0) {
        sa.sa_family = dev->type;
        omap2_die_id_to_ethernet_mac(sa.sa_data, n);
        dev->netdev_ops->ndo_set_mac_address(dev, &sa);
    }

    return NOTIFY_DONE;
}

static struct notifier_block omap_npi_netdev_notifier = {
    .notifier_call = omap_npi_netdev_event,
    .priority = 1,
};

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>

#define OMAP_NPI_WLAN_EN_GPIO    (139)
#define OMAP_NPI_BT_EN_GPIO      (138)
#define OMAP_NPI_WLAN_IRQ_GPIO   (137)
#define OMAP_NPI_FM_EN_BT_WU     (136)

struct wl12xx_platform_data omap_npi_wlan_data __initdata = {
    .irq = OMAP_GPIO_IRQ(OMAP_npi_WLAN_IRQ_GPIO),
    .board_ref_clock = 2, /* 38.4 MHz */
};

static int gpios[] = {OMAP_NPI_BT_EN_GPIO, OMAP_NPI_FM_EN_BT_WU, -1};
static struct platform_device wl12xx_device = {
    .name		= "kim",
    .id			= -1,
    .dev.platform_data = &gpios,
};

static struct omap2_hsmmc_info mmcbbt[] = {
    {
        .mmc		= 1,
        .caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
        .gpio_wp	= -EINVAL,
    },
    {
        .name		= "wl1271",
        .mmc		= 2,
        .caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
        .gpio_wp	= -EINVAL,
        .gpio_cd	= -EINVAL,
        .ocr_mask	= MMC_VDD_165_195,
        .nonremovable	= true,
    },
    {}	/* Terminator */
};

static struct regulator_consumer_supply npi_vmmc2_supply = 
        REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1");

static struct regulator_init_data npi_vmmc2 = {
    .constraints = {
        .valid_ops_mask = REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = 1,
    .consumer_supplies = &npi_vmmc2_supply,
};

static struct fixed_voltage_config npi_vwlan = {
    .supply_name = "vwl1271",
    .microvolts = 1800000,  /* 1.8V */
    .gpio = OMAP_npi_WLAN_EN_GPIO,
    .startup_delay = 70000, /* 70ms */
    .enable_high = 1,
    .enabled_at_boot = 0,
    .init_data = &npi_vmmc2,
};

static struct platform_device omap_vwlan_device = {
    .name           = "reg-fixed-voltage",
    .id             = 1,
    .dev = {
        .platform_data = &npi_vwlan,
    },
};
#endif

static struct omap2_mcspi_device_config dss_lcd_mcspi_config = {
    .turbo_mode		= 1,
    .single_channel	= 1,  /* 0: slave, 1: master */
};

/* Structure for configuing McSPI */
static struct spi_board_info board_spi3_board_info[] = {
    {
        .modalias    = "nec_2432_panel-spi",
        .max_speed_hz = 375000, /* Скорость SPI. */
        .bus_num     = 1,       /* Номер шины McSPI */
        .chip_select = 0,       /* Выбор кристалла для McSPI */
        .mode        = 0,       /* Режим SPI */
        .controller_data	= &dss_lcd_mcspi_config,
    },
};

static void __init pl_init_spi(void)
{
    spi_register_board_info(board_spi3_board_info, ARRAY_SIZE(board_spi3_board_info));
}

static struct mtd_partition omap3npi_nand_partitions[] = {
    /* All the partition sizes are listed in terms of NAND block size */
    {
        .name		= "X-Loader",
        .offset		= 0,
        .size		= 4 * NAND_BLOCK_SIZE,
        .mask_flags	= MTD_WRITEABLE,	/* force read-only */
    },
    {
        .name		= "U-Boot",
        .offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
        .size		= 15 * NAND_BLOCK_SIZE,
        .mask_flags	= MTD_WRITEABLE,	/* force read-only */
    },
    {
        .name		= "U-Boot Env",
        .offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
        .size		= 1 * NAND_BLOCK_SIZE,
    },
    {
        .name		= "Kernel",
        .offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
        .size		= 32 * NAND_BLOCK_SIZE,
    },
    {
        .name		= "File System",
        .offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
        .size		= MTDPART_SIZ_FULL,
    },
};

/* DSS */


static int npi_enable_lcd(struct omap_dss_device *dssdev)
{
    int err;
    int val;
    //printk(KERN_INFO "npi_enable_lcd\n");
    if (gpio_is_valid(npi_config.lcd_pwren)) {

        //int r = gpio_request_one(npi_config.lcd_pwren, GPIOF_OUT_INIT_HIGH,
        //                         "LCD power");
        //if (r < 0)
        //    printk(KERN_ERR "E: Unable to get LCD power enable GPIO\n");

        gpio_set_value(npi_config.lcd_pwren, 1);
      //  printk(KERN_INFO "%s: Enabling LCD\n", __FUNCTION__);
    } else {
        printk(KERN_INFO "%s: Invalid LCD enable GPIO: %d\n",
               __FUNCTION__, npi_config.lcd_pwren);
    }

     //printk(KERN_INFO "\nDisabling keyboard backlight....from npi");

     gpio_direction_output(109, 1);
     gpio_set_value(109, 1);

     vibra_disable_leds();

     //printk(KERN_INFO "\n\n\nStart conf_sleep()\n\n\n");
     //conf_sleep();
  //  udelay(1000);
   /* printk(KERN_INFO "\nSoftsleep....\n");
    err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x02,TWL4030_PM_MASTER_P1_SW_EVENTS);
    printk(KERN_INFO "Result: %d\n", err);
    err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x02,TWL4030_PM_MASTER_P2_SW_EVENTS);
    printk(KERN_INFO "Result: %d\n", err);
    err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x02,TWL4030_PM_MASTER_P3_SW_EVENTS);
    printk(KERN_INFO "Result: %d\n", err);
*/
    //err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, val,TWL4030_PM_MASTER_STS_P123_STATE);
    //printk(KERN_INFO "Result: %d\n", err);
    //printk(KERN_INFO "Val: %x\n", val);*/
    //err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &val,TWL4030_PM_MASTER_P1_SW_EVENTS);
    //printk(KERN_INFO "Result: %d\n", err);
    //printk(KERN_INFO "Val: %x\n", val);

    return 0;
}

static void npi_disable_lcd(struct omap_dss_device *dssdev)
{
 //   printk(KERN_INFO "npi_disable_lcd\n");
    if (gpio_is_valid(npi_config.lcd_pwren)) {

        //int r = gpio_request_one(npi_config.lcd_pwren, GPIOF_OUT_INIT_HIGH,
         //                        "LCD power");
        //if (r < 0)
        //    printk(KERN_ERR "D: Unable to get LCD power enable GPIO\n");

        //printk(KERN_INFO "%s: Disabling LCD\n", __FUNCTION__);
        gpio_set_value(npi_config.lcd_pwren, 0);
    } else {
        printk(KERN_INFO "%s: Invalid LCD enable GPIO: %d\n",
               __FUNCTION__, npi_config.lcd_pwren);
    }


//     printk(KERN_INFO "\nDisabling keyboard backlight....");
     gpio_direction_output(109, 1);
     gpio_set_value(109, 0);

    vibra_disable_leds();

    return;
}

static struct panel_generic_dpi_data lcd_panel = {
    .name = "lgphilips_lb035q02_panel",
    .platform_enable = npi_enable_lcd,
    .platform_disable = npi_disable_lcd,
};

static struct omap_dss_device npi_lcd_device = {
    .type                   = OMAP_DISPLAY_TYPE_DPI,
    .name                   = "lcd 2.7 inch nec-nl2432hc17-07b",
    .driver_name            = "nec_2432_panel",
    .phy.dpi.data_lines     = 18,
    //    .phy.dpi.data_lines     = 16,
    .platform_enable        = npi_enable_lcd,
    .platform_disable       = npi_disable_lcd,
    .reset_gpio 		= -EINVAL,
    .data                   = &lcd_panel,
    .max_backlight_level	= 100,
};

static struct omap_dss_device *npi_dss_devices[] = {
    &npi_lcd_device,
};

static struct omap_dss_board_info npi_dss_data = {
    .num_devices = ARRAY_SIZE(npi_dss_devices),
    .devices = npi_dss_devices,
    .default_device = &npi_lcd_device,
};

static void __init npi_display_init(void)
{
    int r;

    r = gpio_request_one(npi_config.lcd_pwren, GPIOF_OUT_INIT_HIGH,
                         "LCD power");
    if (r < 0)
        printk(KERN_ERR "I: Unable to get LCD power enable GPIO\n");
    // Enable lcd device
    npi_enable_lcd(&npi_lcd_device);
}

static struct omap2_hsmmc_info mmc[] = {
    {
        .mmc		= 1,
        .caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
        .gpio_wp	= -EINVAL,
    },
    {
        .mmc		= 2,
        .caps       = MMC_CAP_4_BIT_DATA,
        .transceiver	= true,
        .ocr_mask	= 0x00100000,	/* 3.3V */
    },
    {}	/* Terminator */
};

static struct regulator_consumer_supply npi_vmmc1_supply[] = {
    REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

static struct regulator_consumer_supply npi_vsim_supply[] = {
    REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.0"),
};

static int npi_twl_gpio_setup(struct device *dev,
                              unsigned gpio, unsigned ngpio)
{
    int r;

    if (npi_config.mmc1_gpio_wp != -EINVAL)
        omap_mux_init_gpio(npi_config.mmc1_gpio_wp, OMAP_PIN_INPUT);
    mmc[0].gpio_wp = npi_config.mmc1_gpio_wp;
    /* gpio + 0 is "mmc0_cd" (input/IRQ) */
    mmc[0].gpio_cd = gpio + 0;
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
    if(!strcmp(expansionboard_name, "bbtoys-wifi")) {
        omap2_hsmmc_init(mmcbbt);
    } else {
        omap2_hsmmc_init(mmc);
    }
#else
    omap2_hsmmc_init(mmc);
#endif

    /*
     * TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, XM active
     * high / others active low)
     * DVI reset GPIO is different between npi revisions
     */
    /* Valid for all -xM revisions */
    if (cpu_is_omap3630()) {
        /*
         * gpio + 1 on Xm controls the TFP410's enable line (active low)
         * gpio + 2 control varies depending on the board rev as below:
         * P7/P8 revisions(prototype): Camera EN
         * A2+ revisions (production): LDO (DVI, serial, led blocks)
         */
        r = gpio_request_one(gpio + 1, GPIOF_OUT_INIT_LOW,
                             "nDVI_PWR_EN");
        if (r)
            pr_err("%s: unable to configure nDVI_PWR_EN\n",
                   __func__);
        r = gpio_request_one(gpio + 2, GPIOF_OUT_INIT_HIGH,
                             "DVI_LDO_EN");
        if (r)
            pr_err("%s: unable to configure DVI_LDO_EN\n",
                   __func__);
    } else {
        /*
         * REVISIT: need ehci-omap hooks for external VBUS
         * power switch and overcurrent detect
         */
        if (gpio_request_one(gpio + 1, GPIOF_IN, "EHCI_nOC"))
            pr_err("%s: unable to configure EHCI_nOC\n", __func__);
    }
    //npi_dvi_device.reset_gpio = npi_config.reset_gpio;

    gpio_request_one(gpio + TWL4030_GPIO_MAX, npi_config.usb_pwr_level,
                     "nEN_USB_PWR");

    /* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
    //gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

    return 0;
}

static struct twl4030_gpio_platform_data npi_gpio_data = {
    .gpio_base	= OMAP_MAX_GPIO_LINES,
    .irq_base	= TWL4030_GPIO_IRQ_BASE,
    .irq_end	= TWL4030_GPIO_IRQ_END,
    .use_leds	= true,
    .pullups	= BIT(1),
    .pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
    | BIT(15) | BIT(16) | BIT(17),
    .setup		= npi_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data npi_vmmc1 = {
    .constraints = {
        .min_uV			= 1850000,
        .max_uV			= 3150000,
        .valid_modes_mask	= REGULATOR_MODE_NORMAL
        | REGULATOR_MODE_STANDBY,
        .valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
        | REGULATOR_CHANGE_MODE
        | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies	= ARRAY_SIZE(npi_vmmc1_supply),
    .consumer_supplies	= npi_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data npi_vsim = {
    .constraints = {
        .min_uV			= 1800000,
        .max_uV			= 3000000,
        .valid_modes_mask	= REGULATOR_MODE_NORMAL
        | REGULATOR_MODE_STANDBY,
        .valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
        | REGULATOR_CHANGE_MODE
        | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies	= ARRAY_SIZE(npi_vsim_supply),
    .consumer_supplies	= npi_vsim_supply,
};

static struct twl4030_bci_platform_data npi_bci_data;
static int npi_batt_table[] = {
    /* 0 C*/
    30800, 29500, 28300, 27100,
    26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
    17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
    11600, 11200, 10800, 10400, 10000, 9630,  9280,  8950,  8620,  8310,
    8020,  7730,  7460,  7200,  6950,  6710,  6470,  6250,  6040,  5830,
    5640,  5450,  5260,  5090,  4920,  4760,  4600,  4450,  4310,  4170,
    4040,  3910,  3790,  3670,  3550
};

static struct twl4030_bci_platform_data npi_bci_data = {
    .battery_tmp_tbl    = npi_batt_table,
    .tblsize        = ARRAY_SIZE(npi_batt_table),
};

static struct twl4030_platform_data npi_twldata = {
    /* platform_data for children goes here */
    //    .irq = OMAP_GPIO_IRQ(0),
    .gpio		= &npi_gpio_data,
    .vmmc1		= &npi_vmmc1,
    .vsim		= &npi_vsim,
    .keypad		= &cm_t35_kp_data,
    .bci		= &npi_bci_data,
};

//static struct twl4030_platform_data cm_t35_twldata = {
//    .irq_base	= TWL4030_IRQ_BASE,
//    .irq_end	= TWL4030_IRQ_END,

//    /* platform_data for children goes here */
////    .keypad		= &cm_t35_kp_data,
////    .usb		= &cm_t35_usb_data,
//    .gpio		= &npi_gpio_data,
//    .vmmc1		= &npi_vmmc1,
//    .vsim		= &npi_vsim,
//};

//static struct i2c_board_info __initdata cm_t35_i2c_boardinfo[] = {
//    {
//        I2C_BOARD_INFO("tps65930", 0x47),
//        .flags		= I2C_CLIENT_WAKE,
//        .irq		= INT_34XX_SYS_NIRQ,
//        .platform_data	= &cm_t35_twldata,
//    },
//};

static int __init omap3_npi_i2c_init(void)
{
    omap3_pmic_get_config(&npi_twldata,
                         0 /*| TWL_COMMON_PDATA_BCI | TWL_COMMON_PDATA_AUDIO*/,
                         /*TWL_COMMON_REGULATOR_VDAC |*/ TWL_COMMON_REGULATOR_VPLL2);

    npi_twldata.vpll2->constraints.name = "VDVI";

   omap3_pmic_init("twl4030", &npi_twldata);

    //    omap3_pmic_get_config(&npi_twldata,
    //                          TWL_COMMON_PDATA_USB/* | TWL_COMMON_PDATA_AUDIO*/,
    //                          TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);
    //    omap3_pmic_init("twl4030", &npi_twldata);
    //    omap3_pmic_init("tps65930", &npi_twldata);

    /* Bus 3 is attached to the DVI port where devices like the pico DLP
     * projector don't work reliably with 400kHz */
    //	omap_register_i2c_bus(3, 100, npi_i2c_eeprom, ARRAY_SIZE(npi_i2c_eeprom));

    //    board_map_data.keymap = cm_t35_keymap;
    //    board_map_data.keymap_size = ARRAY_SIZE(cm_t35_keymap);
    //    blueshark_max7359data.debounce_reg_val = 0x5F;

    /* Bus 3 is attached to the DVI port where devices like the pico DLP
     * projector don't work reliably with 400kHz */
    //    omap_register_i2c_bus(1, 2600, cm_t35_i2c_boardinfo,
    //                  ARRAY_SIZE(cm_t35_i2c_boardinfo));

    omap_register_i2c_bus(2, 400, pl_i2c_devices_boardinfo, ARRAY_SIZE(pl_i2c_devices_boardinfo));
    //omap3npi_stmpe_init();
    //    omap3npi_twl_init();
    return 0;
}

static void __init omap3_npi_init_early(void)
{
    omap2_init_common_infrastructure();
    omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
                              mt46h32m32lf6_sdrc_params);
}

static void __init omap3_npi_init_irq(void)
{
    omap3_init_irq();
}

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

    .port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
    .port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
    .port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

    .phy_reset  = true,
    .reset_gpio_port[0]  = -EINVAL,
    .reset_gpio_port[1]  = 147,
    .reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
    { .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static void __init npiOppInit(void)
{
    int r = 0;

    /* Initialize the omap3 opp table */
    if (omap3_opp_init()) {
        pr_err("%s: opp default init failed\n", __func__);
        return;
    }

    /* Custom OPP enabled for all xM versions */
    if (cpu_is_omap3630()) {
        struct omap_hwmod *mh = omap_hwmod_lookup("mpu");
        struct omap_hwmod *dh = omap_hwmod_lookup("iva");
        struct device *dev;

        if (!mh || !dh) {
            pr_err("%s: Aiee.. no mpu/dsp devices? %p %p\n",
                   __func__, mh, dh);
            return;
        }
        /* Enable MPU 1GHz and lower opps */
        dev = &mh->od->pdev.dev;
        r = opp_enable(dev, 800000000);
        //r |= opp_enable(dev, 1000000000);
        /* TODO: MPU 1GHz needs SR and ABB */

        /* Enable IVA 800MHz and lower opps */
        dev = &dh->od->pdev.dev;
        r |= opp_enable(dev, 660000000);
        /* TODO: DSP 800MHz needs SR and ABB */
        if (r) {
            pr_err("%s: failed to enable higher opp %d\n",
                   __func__, r);
            /*
             * Cleanup - disable the higher freqs - we dont care
             * about the results
             */
            dev = &mh->od->pdev.dev;
            opp_disable(dev, 800000000);
            dev = &dh->od->pdev.dev;
            opp_disable(dev, 660000000);
        }
    }
    return;
}

static void omap3_pl_poweroff(void)
{
    //! Not work
    /*gpio_request(POWEROFF_GPIO, "POWEROFF");
    gpio_direction_output(POWEROFF_GPIO, 0);
    gpio_set_value(POWEROFF_GPIO, 1);
    udelay(1000);
    gpio_set_value(POWEROFF_GPIO, 0);*/
}

/// PWM BACKLITE
#if defined(CONFIG_HAVE_PWM)

static struct omap2_pwm_platform_config pwm1_config = {
    .timer_id           = 11,   // GPT10_PWM_EVT
    .polarity           = 1     // Active-high
};

static struct platform_device npi_pwm1_device = {
    .name               = "omap-pwm",
    .id                 = 0,
    .dev                =
    {
        .platform_data  = &pwm1_config
    }
};

static struct platform_pwm_backlight_data npi_backlight_data = {
    .pwm_id		= 0,
    .max_brightness	= 100,
    .dft_brightness	= 50,
    .pwm_period_ns	= 900, // ~1Mhz
};

static struct platform_device npi_backlight_device = {
    .name		= "pwm-backlight",
    .dev		= {
        .parent = &npi_pwm1_device.dev,
        .platform_data = &npi_backlight_data,
    },
};

static struct platform_device *pwm_devices[] __initdata = {
    &npi_pwm1_device
};

static inline void pwm_init(void)
{
    platform_add_devices(pwm_devices, ARRAY_SIZE(pwm_devices));
}

static struct omap_board_mux npi_as_pwm_mux[] = {
    OMAP3_MUX(MCSPI2_CS0, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
    { .reg_offset = OMAP_MUX_TERMINATOR },
};

#endif // CONFIG_HAVE_PWM

static void __init omap3_npi_init(void)
{
    /*
    pm_power_off = omap3_pl_poweroff;
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
    //omap3_npi_init_rev();
    //    omap_serial_init();
    omap3_npi_i2c_init();

    //gpio_buttons[0].gpio = npi_config.usr_button_gpio;
    omap_serial_init();

    omap_mux_init_gpio(170, OMAP_PIN_INPUT);
    // REVISIT leave DVI powered down until it's needed ...
    gpio_request_one(170, GPIOF_OUT_INIT_HIGH, "DVI_nPD");

    gpio_direction_output(109, 1);
    gpio_set_value(109, 1);

    //gpio_direction_output(174, 1);

    //    gpio_request(110, "LCD_ON");
    //    gpio_direction_output(110, 0);
    //    gpio_set_value(110, 0);
    //    mdelay(1);
    //    gpio_set_value(110, 1);
    ////    gpio_export(110, 1);
    //    mdelay(1);

    //    gpio_request(98, "LCD_RESET");
    ////    gpio_direction_input(98);
    //    printk(KERN_INFO "LCD_RESET %d\n", gpio_get_value(98));
    //    mdelay(1);

    //    gpio_direction_output(98, 1);
    //    gpio_set_value(98, 1);
    //    mdelay(1);
    //    gpio_set_value(98, 0);
    ////    gpio_export(98, 0);
    //    mdelay(1);

*/   
#if defined(CONFIG_HAVE_PWM)
    struct omap_mux_partition *mux_partition;
#endif

    pm_power_off = omap3_pl_poweroff;                   // Not work
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);        // Need, don't work
    omap3_npi_i2c_init();
    // KEYPABOARD
    //    omap3_pmic_init("twl4030", &npi_twldata);
    // TOUCHSCEEN
    //    omap_register_i2c_bus(2, 100, pl_i2c_devices_boardinfo, ARRAY_SIZE(pl_i2c_devices_boardinfo));

    //    zkpk_init();

    omap_serial_init();

    // PowerDown DVI
    omap_mux_init_gpio(170, OMAP_PIN_INPUT);
    /* REVISIT leave DVI powered down until it's needed ... */
    gpio_request_one(170, GPIOF_OUT_INIT_HIGH, "DVI_nPD");

    gpio_direction_output(109, 1);
    gpio_set_value(109, 1);

    // SPI
    pl_init_spi();

    // USB
    //usb_musb_init(NULL);
    //usbhs_init(&usbhs_bdata);
    //omap_nand_flash_init(NAND_BUSWIDTH_16, omap3npi_nand_partitions,
                        // ARRAY_SIZE(omap3npi_nand_partitions));

    /* Ensure msecure is mux'd to be able to set the RTC. */
    omap_mux_init_signal("sys_drm_msecure", OMAP_PIN_OFF_OUTPUT_HIGH);

    /* Ensure SDRC pins are mux'd for self-refresh */
    omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
    omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

    // Display
    omap_display_init(&npi_dss_data);
    npi_display_init();

    // Register PWM device
#if defined(CONFIG_HAVE_PWM)
    mux_partition = omap_mux_get("core");
    omap_mux_write_array(mux_partition, npi_as_pwm_mux);
    //OMAP3_MUX(MCSPI2_CS0, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT);
    pwm_init();
    // PWM backlite device

    platform_device_register(&npi_backlight_device);
#endif

    // OPP table
    npiOppInit();

}

MACHINE_START(OMAP3_NPI, "OMAP3 NPI Board")
/* Maintainer: Syed Mohammed Khasim - http://npiboard.org */
.boot_params	= 0x80000100,
.reserve	= omap_reserve,
.map_io		= omap3_map_io,
.init_early	= omap3_npi_init_early,
.init_irq	= omap3_npi_init_irq,
.init_machine	= omap3_npi_init,
.timer		= &omap3_timer,
MACHINE_END
