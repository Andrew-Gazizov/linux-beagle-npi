/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
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
#include <linux/input/max7359.h>

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

#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include "common-board-devices.h"

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

#define BLUESHARK_GPIO_PENDOWN	167
#define POWEROFF_GPIO 103
#define	GPIO_MAX7359_IRQ	156

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(BLUESHARK_GPIO_PENDOWN);
}

static struct ads7846_platform_data ads7846_config_zkpk = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 550,
	.y_plate_ohms		= 300,
	.pressure_max		= 255,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.model			= 7845,
	.vref_mv		= 3300,
	.penirq_recheck_delay_usecs = 3000,
	.settle_delay_usecs = 150,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 2,
	.swap_xy		= true, /// актуально для ЗКПК и xf86-input-evdev, поскольку xinput-calibrator глючит с перепутанными осями
};

static struct ads7846_platform_data ads7846_config_pkk = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 550,
	.y_plate_ohms		= 300,
	.pressure_max		= 4096, /// подбирается эмпирически, зависит от x_plate_ohms (вычисляется), также зависит от места нажатия на тачскрине
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.model			= 7846,
	.penirq_recheck_delay_usecs = 3000,
	.settle_delay_usecs	= 150,
	.debounce_max		= 10,
	.debounce_tol		= 5,
	.debounce_rep		= 1,
	.swap_xy		= true,
};


static void __init blueshark_ads7846_init(void)
{
	if ((gpio_request(BLUESHARK_GPIO_PENDOWN, "ADS7846_PENDOWN") == 0) &&
	    (gpio_direction_input(BLUESHARK_GPIO_PENDOWN) == 0)) {
		gpio_export(BLUESHARK_GPIO_PENDOWN, 0);
	} else {
		printk(KERN_ERR "could not obtain gpio for ADS7846_PENDOWN\n");
		return;
	}

	/// особых изменений от добавления следующих строк не заметил, но пока оставлю
	/// поскольку на ЗКПК с прерыванием от тачскрина дела обстоят не очень(шум на осцилле)
	gpio_set_debounce(BLUESHARK_GPIO_PENDOWN, 0xa);
}

/*
 * Board-specific configuration
 * Defaults to BeagleBoard-xMC
 */
static struct {
	int mmc1_gpio_wp;
	int usb_pwr_level;
	int reset_gpio;
	int usr_button_gpio;
	char *lcd_driver_name;
	int lcd_pwren;
} beagle_config = {
	.mmc1_gpio_wp = -EINVAL,
	.usb_pwr_level = GPIOF_OUT_INIT_LOW,
	.reset_gpio = 129,
	.usr_button_gpio = 4,
	.lcd_driver_name = "",
	.lcd_pwren = 0
};


static int max7359_init_irq(void)
{
	int ret = 0;

	ret = gpio_request(GPIO_MAX7359_IRQ, "max7359-irq");
	if (ret < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d: %d\n",
				GPIO_MAX7359_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_MAX7359_IRQ)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_MAX7359_IRQ);
		return -ENXIO;
	}

	return ret;
}

static void max7359_exit_irq(void)
{
	gpio_free(GPIO_MAX7359_IRQ);
}

#define DEFINE_KEY(row, col, keycode) ((row << 24) | (col << 16) | keycode)

static const uint32_t pl_keys_zkpk[] = {
	DEFINE_KEY(0, 0, KEY_RESERVED), /// row 0 not used in zkpk2
	DEFINE_KEY(0, 1, KEY_RESERVED),
	DEFINE_KEY(0, 2, KEY_RESERVED),
	DEFINE_KEY(0, 3, KEY_RESERVED),
	DEFINE_KEY(0, 4, KEY_RESERVED),
	DEFINE_KEY(0, 5, KEY_RESERVED),
	DEFINE_KEY(0, 6, KEY_RESERVED),
	DEFINE_KEY(0, 7, KEY_RESERVED),

	DEFINE_KEY(1, 0, KEY_RESERVED), /// col 0 not used in zkpk2
	DEFINE_KEY(1, 1, KEY_ESC),
	DEFINE_KEY(1, 2, KEY_UP),
	DEFINE_KEY(1, 3, KEY_ENTER),
	DEFINE_KEY(1, 4, KEY_LEFTSHIFT),//f1
	DEFINE_KEY(1, 5, KEY_RESERVED), /// col 5 used as GPO port in zkpk2
	DEFINE_KEY(1, 6, KEY_RESERVED), /// col 6 used as GPO port in zkpk2
	DEFINE_KEY(1, 7, KEY_RESERVED), /// col 7 used as GPO port in zkpk2

	DEFINE_KEY(2, 0, KEY_RESERVED), /// col 0 not used in zkpk2
	DEFINE_KEY(2, 1, KEY_RIGHT),
	DEFINE_KEY(2, 2, KEY_DOWN),
	DEFINE_KEY(2, 3, KEY_LEFT),
	DEFINE_KEY(2, 4, KEY_LEFTCTRL),//f2
	DEFINE_KEY(2, 5, KEY_RESERVED), /// col 5 used as GPO port in zkpk2
	DEFINE_KEY(2, 6, KEY_RESERVED), /// col 6 used as GPO port in zkpk2
	DEFINE_KEY(2, 7, KEY_RESERVED), /// col 7 used as GPO port in zkpk2
	
	DEFINE_KEY(3, 0, KEY_RESERVED), /// col 0 not used in zkpk2
	DEFINE_KEY(3, 1, KEY_3),
	DEFINE_KEY(3, 2, KEY_2),
	DEFINE_KEY(3, 3, KEY_1),
	DEFINE_KEY(3, 4, KEY_LEFTALT),//f3
	DEFINE_KEY(3, 5, KEY_RESERVED), /// col 5 used as GPO port in zkpk2
	DEFINE_KEY(3, 6, KEY_RESERVED), /// col 6 used as GPO port in zkpk2
	DEFINE_KEY(3, 7, KEY_RESERVED), /// col 7 used as GPO port in zkpk2
	
	DEFINE_KEY(4, 0, KEY_RESERVED), /// col 0 not used in zkpk2
	DEFINE_KEY(4, 1, KEY_6),
	DEFINE_KEY(4, 2, KEY_5),
	DEFINE_KEY(4, 3, KEY_4),
	DEFINE_KEY(4, 4, KEY_F4),//f4
	DEFINE_KEY(4, 5, KEY_RESERVED), /// col 5 used as GPO port in zkpk2
	DEFINE_KEY(4, 6, KEY_RESERVED), /// col 6 used as GPO port in zkpk2
	DEFINE_KEY(4, 7, KEY_RESERVED), /// col 7 used as GPO port in zkpk2

	DEFINE_KEY(5, 0, KEY_RESERVED), /// col 0 not used in zkpk2
	DEFINE_KEY(5, 1, KEY_9),
	DEFINE_KEY(5, 2, KEY_8),
	DEFINE_KEY(5, 3, KEY_7),
	DEFINE_KEY(5, 4, KEY_RESERVED), /// NC
	DEFINE_KEY(5, 5, KEY_RESERVED), /// col 5 used as GPO port in zkpk2
	DEFINE_KEY(5, 6, KEY_RESERVED), /// col 6 used as GPO port in zkpk2
	DEFINE_KEY(5, 7, KEY_RESERVED), /// col 7 used as GPO port in zkpk2

	DEFINE_KEY(6, 0, KEY_RESERVED), /// col 0 not used in zkpk2
	DEFINE_KEY(6, 1, KEY_KPMINUS),
	DEFINE_KEY(6, 2, KEY_0),
	DEFINE_KEY(6, 3, KEY_KPPLUS),
	DEFINE_KEY(6, 4, KEY_RESERVED), /// NC
	DEFINE_KEY(6, 5, KEY_RESERVED), /// col 5 used as GPO port in zkpk2
	DEFINE_KEY(6, 6, KEY_RESERVED), /// col 6 used as GPO port in zkpk2
	DEFINE_KEY(6, 7, KEY_RESERVED), /// col 7 used as GPO port in zkpk2
	
	DEFINE_KEY(7, 0, KEY_RESERVED), /// row 7 not used in zkpk2
	DEFINE_KEY(7, 1, KEY_RESERVED),
	DEFINE_KEY(7, 2, KEY_RESERVED),
	DEFINE_KEY(7, 3, KEY_RESERVED),
	DEFINE_KEY(7, 4, KEY_RESERVED),
	DEFINE_KEY(7, 5, KEY_RESERVED),
	DEFINE_KEY(7, 6, KEY_RESERVED),
	DEFINE_KEY(7, 7, KEY_RESERVED),
};

static const uint32_t pl_keys_pkk[] = {
	KEY(0, 0, KEY_RESERVED), /// row 0 not used in zkpk2
	KEY(0, 1, KEY_RESERVED),
	KEY(0, 2, KEY_RESERVED),
	KEY(0, 3, KEY_RESERVED),
	KEY(0, 4, KEY_RESERVED),
	KEY(0, 5, KEY_RESERVED),
	KEY(0, 6, KEY_RESERVED),
	KEY(0, 7, KEY_RESERVED),

	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_ENTER),
	KEY(1, 2, KEY_ESC),
	KEY(1, 3, KEY_KPPLUS),
	KEY(1, 4, KEY_2),
	KEY(1, 5, KEY_1),
	KEY(1, 6, KEY_RESERVED), /// col 6 used as GPO port in pkkb
	KEY(1, 7, KEY_RESERVED), /// col 7 used as GPO port in pkkb

	KEY(2, 0, KEY_DOWN),
	KEY(2, 1, KEY_LEFTSHIFT),
	KEY(2, 2, KEY_LEFTCTRL),
	KEY(2, 3, KEY_KPMINUS),
	KEY(2, 4, KEY_3),
	KEY(2, 5, KEY_4),
	KEY(2, 6, KEY_RESERVED), /// col 6 used as GPO port in pkkb
	KEY(2, 7, KEY_RESERVED), /// col 7 used as GPO port in pkkb
	
	KEY(3, 0, KEY_LEFT),
	KEY(3, 1, KEY_LEFTALT),
	KEY(3, 2, KEY_F4),
	KEY(3, 3, KEY_5),
	KEY(3, 4, KEY_6),
	KEY(3, 5, KEY_7),
	KEY(3, 6, KEY_RESERVED), /// col 6 used as GPO port in pkkb
	KEY(3, 7, KEY_RESERVED), /// col 7 used as GPO port in pkkb
	
	KEY(4, 0, KEY_UP),
	KEY(4, 1, KEY_RESERVED),
	KEY(4, 2, KEY_RESERVED),
	KEY(4, 3, KEY_8),
	KEY(4, 4, KEY_9),
	KEY(4, 5, KEY_0),
	KEY(4, 6, KEY_RESERVED), /// col 6 used as GPO port in pkkb
	KEY(4, 7, KEY_RESERVED), /// col 7 used as GPO port in pkkb

	KEY(5, 0, KEY_RESERVED),
	KEY(5, 1, KEY_RESERVED),
	KEY(5, 2, KEY_RESERVED),
	KEY(5, 3, KEY_RESERVED),
	KEY(5, 4, KEY_RESERVED),
	KEY(5, 5, KEY_RESERVED),
	KEY(5, 6, KEY_RESERVED), /// col 6 used as GPO port in pkkb
	KEY(5, 7, KEY_RESERVED), /// col 7 used as GPO port in pkkb

	KEY(6, 0, KEY_RESERVED),
	KEY(6, 1, KEY_RESERVED),
	KEY(6, 2, KEY_RESERVED),
	KEY(6, 3, KEY_RESERVED),
	KEY(6, 4, KEY_RESERVED),
	KEY(6, 5, KEY_RESERVED),
	KEY(6, 6, KEY_RESERVED), /// col 6 used as GPO port in pkkb
	KEY(6, 7, KEY_RESERVED), /// col 7 used as GPO port in pkkb
	
	KEY(7, 0, KEY_RESERVED), /// row 7 not used in pkkb
	KEY(7, 1, KEY_RESERVED),
	KEY(7, 2, KEY_RESERVED),
	KEY(7, 3, KEY_RESERVED),
	KEY(7, 4, KEY_RESERVED),
	KEY(7, 5, KEY_RESERVED),
	KEY(7, 6, KEY_RESERVED),
	KEY(7, 7, KEY_RESERVED),
};

static struct matrix_keymap_data board_map_data;

static struct max7359_platform_data blueshark_max7359data =  {
	.keymap_data = &board_map_data,
	.init_platform_hw = max7359_init_irq,
	.exit_platform_hw = max7359_exit_irq,
};

static struct i2c_board_info __initdata pl_i2c_devices_boardinfo[] = {
	{
		I2C_BOARD_INFO("max7359", 0x38),
		.platform_data = &blueshark_max7359data,
		.irq = OMAP_GPIO_IRQ(GPIO_MAX7359_IRQ),
	},
	{
		I2C_BOARD_INFO("hmc5843", 0x1e),
	},
  {
    I2C_BOARD_INFO("ds1307", 0x68),
  }
};
/*
 * This device path represents the onboard USB <-> Ethernet bridge
 * on the BeagleBoard-xM which needs a random or all-zeros
 * mac address replaced with a per-cpu stable generated one
 */

static const char * const xm_fixup_mac_device_paths[] = {
	"usb1/1-2/1-2.1/1-2.1:1.0",
};

static int beagle_device_path_need_mac(struct device *dev)
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

static int omap_beagle_netdev_event(struct notifier_block *this,
						 unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	struct sockaddr sa;
	int n;

	if (event != NETDEV_REGISTER)
		return NOTIFY_DONE;

	n = beagle_device_path_need_mac(dev->dev.parent);
	if (n >= 0) {
		sa.sa_family = dev->type;
		omap2_die_id_to_ethernet_mac(sa.sa_data, n);
		dev->netdev_ops->ndo_set_mac_address(dev, &sa);
	}

	return NOTIFY_DONE;
}

static struct notifier_block omap_beagle_netdev_notifier = {
	.notifier_call = omap_beagle_netdev_event,
	.priority = 1,
};

static struct gpio omap3_beagle_rev_gpios[] __initdata = {
	{ 171, GPIOF_IN, "rev_id_0"    },
	{ 172, GPIOF_IN, "rev_id_1" },
	{ 173, GPIOF_IN, "rev_id_2"    },
};

static void __init omap3_beagle_init_rev(void)
{
	int ret;
	u16 beagle_rev = 0;
#ifndef CONFIG_BLUE_BRYN
	omap_mux_init_gpio(171, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(172, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(173, OMAP_PIN_INPUT_PULLUP);

	ret = gpio_request_array(omap3_beagle_rev_gpios,
				 ARRAY_SIZE(omap3_beagle_rev_gpios));
	if (ret < 0) {
		printk(KERN_ERR "Unable to get revision detection GPIO pins\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
		return;
	}

	beagle_rev = gpio_get_value(171) | (gpio_get_value(172) << 1)
			| (gpio_get_value(173) << 2);

	gpio_free_array(omap3_beagle_rev_gpios,
			ARRAY_SIZE(omap3_beagle_rev_gpios));
#endif
	beagle_rev = 0;
	switch (beagle_rev) {
	case 7:
		printk(KERN_INFO "OMAP3 Beagle Rev: Ax/Bx\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_AXBX;
		beagle_config.mmc1_gpio_wp = 29;
		beagle_config.reset_gpio = 170;
		beagle_config.usr_button_gpio = 7;
		break;
	case 6:
		printk(KERN_INFO "OMAP3 Beagle Rev: C1/C2/C3\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C1_3;
		beagle_config.mmc1_gpio_wp = 23;
		beagle_config.reset_gpio = 170;
		beagle_config.usr_button_gpio = 7;
		break;
	case 5:
		printk(KERN_INFO "OMAP3 Beagle Rev: C4\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C4;
		beagle_config.mmc1_gpio_wp = 23;
		beagle_config.reset_gpio = 170;
		beagle_config.usr_button_gpio = 7;
		break;
	case 0:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM Ax/Bx\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XM;
		beagle_config.usb_pwr_level = GPIOF_OUT_INIT_HIGH;
		register_netdevice_notifier(&omap_beagle_netdev_notifier);
		break;
	case 2:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM C\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XMC;
		register_netdevice_notifier(&omap_beagle_netdev_notifier);
		break;
	default:
		printk(KERN_INFO "OMAP3 Beagle Rev: unknown %hd\n", beagle_rev);
		omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
		register_netdevice_notifier(&omap_beagle_netdev_notifier);
	}
}

char expansionboard_name[16];
char expansionboard2_name[16];
char motherb_name[16];

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>

#define OMAP_BEAGLE_WLAN_EN_GPIO    (139)
#define OMAP_BEAGLE_BT_EN_GPIO      (138)
#define OMAP_BEAGLE_WLAN_IRQ_GPIO   (137)
#define OMAP_BEAGLE_FM_EN_BT_WU     (136)

struct wl12xx_platform_data omap_beagle_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(OMAP_BEAGLE_WLAN_IRQ_GPIO),
	.board_ref_clock = 2, /* 38.4 MHz */
};

static int gpios[] = {OMAP_BEAGLE_BT_EN_GPIO, OMAP_BEAGLE_FM_EN_BT_WU, -1};
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

static struct regulator_consumer_supply beagle_vmmc2_supply = 
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1");

static struct regulator_init_data beagle_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &beagle_vmmc2_supply,
};

static struct fixed_voltage_config beagle_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000,  /* 1.8V */
	.gpio = OMAP_BEAGLE_WLAN_EN_GPIO,
	.startup_delay = 70000, /* 70ms */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &beagle_vmmc2,
};

static struct platform_device omap_vwlan_device = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data = &beagle_vwlan,
	},
};
#endif

#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_ENC28J60_IRQ 157

static struct omap2_mcspi_device_config enc28j60_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy_spi_board_info[] __initdata = {
	{
		.modalias		= "enc28j60",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 20000000,
		.controller_data	= &enc28j60_spi_chip_info,
	},
};

static void __init omap3beagle_enc28j60_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, "ENC28J60_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_ENC28J60_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, 0);
		omap3beagle_zippy_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_ENC28J60_IRQ);
		irq_set_irq_type(omap3beagle_zippy_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for ENC28J60_IRQ\n");
		return;
	}
#error "ooo"
	spi_register_board_info(omap3beagle_zippy_spi_board_info,
			ARRAY_SIZE(omap3beagle_zippy_spi_board_info));
}

#else
static inline void __init omap3beagle_enc28j60_init(void) { return; }
#endif

static struct spi_board_info pl_spi_board_info[] = {
#if 0
	{
		.modalias	= "spidev",
		.max_speed_hz	= 3000000, //48 Mbps
		.bus_num	= 3,
		.chip_select	= 1,
		.mode = 0,
	},
#endif
	{
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(BLUESHARK_GPIO_PENDOWN),
    //		.platform_data		= &ads7846_config,
	},
};


static void __init pl_init_spi(void)
{
	if (strcmp(expansionboard_name, "pkk") == 0)
    pl_spi_board_info[0].platform_data = &ads7846_config_pkk;
  else if (strcmp(expansionboard_name, "zkpk") == 0)
    pl_spi_board_info[0].platform_data = &ads7846_config_zkpk;
  else
    return;
	blueshark_ads7846_init();
	spi_register_board_info(pl_spi_board_info,
				ARRAY_SIZE(pl_spi_board_info));
}

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_KS8851_IRQ 157

static struct omap2_mcspi_device_config ks8851_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy2_spi_board_info[] __initdata = {
	{
		.modalias		= "ks8851",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 36000000,
		.controller_data	= &ks8851_spi_chip_info,
	},
};

static void __init omap3beagle_ks8851_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_KS8851_IRQ, "KS8851_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_KS8851_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_KS8851_IRQ, 0);
		omap3beagle_zippy2_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_KS8851_IRQ);
		irq_set_irq_type(omap3beagle_zippy2_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for KS8851_IRQ\n");
		return;
	}
#error "boo"
	spi_register_board_info(omap3beagle_zippy2_spi_board_info,
							ARRAY_SIZE(omap3beagle_zippy2_spi_board_info));
}

#else
static inline void __init omap3beagle_ks8851_init(void) { return; }
#endif

static struct mtd_partition omap3beagle_nand_partitions[] = {
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

static int beagle_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void beagle_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct panel_generic_dpi_data dvi_panel = {
	.name = "generic",
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
};

static struct omap_dss_device beagle_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_dpi_panel",
	.data = &dvi_panel,
	.phy.dpi.data_lines = 24,
	.reset_gpio = -EINVAL,
};

static struct omap_dss_device beagle_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static int beagle_enable_lcd(struct omap_dss_device *dssdev)
{
#if 0
       if (gpio_is_valid(beagle_config.lcd_pwren)) {
               printk(KERN_INFO "%s: Enabling LCD\n", __FUNCTION__);
               gpio_set_value(beagle_config.lcd_pwren, 0);
       } else {
               printk(KERN_INFO "%s: Invalid LCD enable GPIO: %d\n",
                       __FUNCTION__, beagle_config.lcd_pwren);
       }
#endif
       return 0;
}

static void beagle_disable_lcd(struct omap_dss_device *dssdev)
{
#if 0
       if (gpio_is_valid(beagle_config.lcd_pwren)) {
               printk(KERN_INFO "%s: Disabling LCD\n", __FUNCTION__);
               gpio_set_value(beagle_config.lcd_pwren, 1);
       } else {
               printk(KERN_INFO "%s: Invalid LCD enable GPIO: %d\n",
                       __FUNCTION__, beagle_config.lcd_pwren);
       }
#endif
       return;
}

static struct panel_generic_dpi_data lcd_panel = {
	.name = "generic",
	.platform_enable = beagle_enable_lcd,
	.platform_disable = beagle_disable_lcd,
};

static struct omap_dss_device beagle_lcd_device = {
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.name                   = "lcd",
	.driver_name		= "generic_dpi_panel",
	.phy.dpi.data_lines     = 18,
	.platform_enable        = beagle_enable_lcd,
	.platform_disable       = beagle_disable_lcd,
	.reset_gpio 		= -EINVAL,
	.data			= &lcd_panel,
};

static struct omap_dss_device *beagle_dss_devices[] = {
	&beagle_dvi_device,
	&beagle_tv_device,
	&beagle_lcd_device,
};

static struct omap_dss_board_info beagle_dss_data = {
	.num_devices = ARRAY_SIZE(beagle_dss_devices),
	.devices = beagle_dss_devices,
	.default_device = &beagle_lcd_device,
};

static void __init beagle_display_init(void)
{
	int r;

	r = gpio_request_one(beagle_dvi_device.reset_gpio, GPIOF_OUT_INIT_LOW,
			     "DVI reset");
	if (r < 0)
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
#if 0
       r = gpio_request_one(beagle_config.lcd_pwren, GPIOF_OUT_INIT_LOW,
                            "LCD power");
       if (r < 0)
               printk(KERN_ERR "Unable to get LCD power enable GPIO\n");
#endif
}

#include "sdram-micron-mt46h32m32lf-6.h"

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

static struct regulator_consumer_supply beagle_vmmc1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

static struct regulator_consumer_supply beagle_vsim_supply[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.0"),
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	int r;

	if (beagle_config.mmc1_gpio_wp != -EINVAL)
		omap_mux_init_gpio(beagle_config.mmc1_gpio_wp, OMAP_PIN_INPUT);
	mmc[0].gpio_wp = beagle_config.mmc1_gpio_wp;
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
	 * DVI reset GPIO is different between beagle revisions
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
	beagle_dvi_device.reset_gpio = beagle_config.reset_gpio;

	gpio_request_one(gpio + TWL4030_GPIO_MAX, beagle_config.usb_pwr_level,
			"nEN_USB_PWR");

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(beagle_vmmc1_supply),
	.consumer_supplies	= beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(beagle_vsim_supply),
	.consumer_supplies	= beagle_vsim_supply,
};

static struct twl4030_platform_data beagle_twldata = {
	/* platform_data for children goes here */
	.gpio		= &beagle_gpio_data,
	.vmmc1		= &beagle_vmmc1,
	.vsim		= &beagle_vsim,
};

static struct i2c_board_info __initdata beagle_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("eeprom", 0x50),
       },
};

#if defined(CONFIG_RTC_DRV_DS1307) || \
	defined(CONFIG_RTC_DRV_DS1307_MODULE)

static struct i2c_board_info __initdata beagle_i2c2_zippy[] = {
	{
		I2C_BOARD_INFO("eeprom", 0x50),
		I2C_BOARD_INFO("ds1307", 0x68),
	},
};
#else
static struct i2c_board_info __initdata beagle_i2c2_zippy[] = {};
#endif

#if defined(CONFIG_INPUT_TOUCHSCREEN) && \
	defined(CONFIG_TOUCHSCREEN_TSC2007)
/* Touchscreen */
#define OMAP3BEAGLE_TSC2007_GPIO 157
static int omap3beagle_tsc2007_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3BEAGLE_TSC2007_GPIO);
}

static int omap3beagle_tsc2007_init(void)
{
	int gpio = OMAP3BEAGLE_TSC2007_GPIO;
	int ret = 0;
	printk(KERN_WARNING "TSC2007_init started");
	ret = gpio_request(gpio, "tsc2007_pen_down");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for "
		"tsc2007 pen down IRQ\n", gpio);
		return ret;
	}

	omap_mux_init_gpio(OMAP3BEAGLE_TSC2007_GPIO, OMAP_PIN_INPUT_PULLUP);
	gpio_direction_input(gpio);

	irq_set_irq_type(OMAP_GPIO_IRQ(OMAP3BEAGLE_TSC2007_GPIO), IRQ_TYPE_EDGE_FALLING);

	return ret;
}

static struct tsc2007_platform_data tsc2007_info = {
	.model = 2007,
	.x_plate_ohms = 180,
	.get_pendown_state = omap3beagle_tsc2007_get_pendown_state,
	.init_platform_hw = omap3beagle_tsc2007_init,
};

static struct i2c_board_info __initdata beagle_i2c2_bbtoys_ulcd[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq = OMAP_GPIO_IRQ(OMAP3BEAGLE_TSC2007_GPIO),
		.platform_data = &tsc2007_info,
	},
};
#else
static struct i2c_board_info __initdata beagle_i2c2_bbtoys_ulcd[] = {};
#endif

static int __init omap3_beagle_i2c_init(void)
{
	omap3_pmic_get_config(&beagle_twldata,
			TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_AUDIO,
			TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);

	beagle_twldata.vpll2->constraints.name = "VDVI";

	omap3_pmic_init("twl4030", &beagle_twldata);
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
//	omap_register_i2c_bus(3, 100, beagle_i2c_eeprom, ARRAY_SIZE(beagle_i2c_eeprom));

	if (strcmp(expansionboard_name, "pkk") == 0) {
		printk(KERN_INFO "Beagle expansionboard: PKK\n");
    board_map_data.keymap = pl_keys_pkk;
    board_map_data.keymap_size = ARRAY_SIZE(pl_keys_pkk);
    blueshark_max7359data.debounce_reg_val = 0x5F;
  } else if (strcmp(expansionboard_name, "zkpk") == 0) {
    board_map_data.keymap = pl_keys_zkpk;
    board_map_data.keymap_size = ARRAY_SIZE(pl_keys_zkpk);
    blueshark_max7359data.debounce_reg_val = 0x7F;
  }
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(2, 100, pl_i2c_devices_boardinfo, ARRAY_SIZE(pl_i2c_devices_boardinfo));
	

	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct platform_device ws2801_leds = {
	.name	= "ws2801-leds",
	.id	= -1,
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		/* Dynamically assigned depending on board */
		.gpio			= -EINVAL,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static void __init omap3_beagle_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
}

static void __init omap3_beagle_init_irq(void)
{
	omap3_init_irq();
}

static struct platform_device *omap3_beagle_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
};

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

static int __init motherb_setup(char *str)
{
	if (!str)
		return -EINVAL;
  strncpy(motherb_name, str, 15);
  motherb_name[15] = '\0';
  printk(KERN_INFO "Mother board name: %s\n", motherb_name);
  return 0;
}

static int __init expansionboard_setup(char *str)
{
	if (!str)
		return -EINVAL;
	strncpy(expansionboard_name, str, 16);
	printk(KERN_INFO "Beagle expansionboard: %s\n", expansionboard_name);
	return 0;
}

static int __init expansionboard2_setup(char *str)
{
	if (!str)
		return -EINVAL;
	strncpy(expansionboard2_name, str, 16);
	printk(KERN_INFO "Beagle second expansionboard: %s\n", expansionboard2_name);
	return 0;
}

static void __init beagle_opp_init(void)
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
		//r |= opp_enable(dev, 800000000);
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
	gpio_request(POWEROFF_GPIO, "POWEROFF");
	gpio_direction_output(POWEROFF_GPIO, 0);
	gpio_set_value(POWEROFF_GPIO, 1);
	udelay(1000);
	gpio_set_value(POWEROFF_GPIO, 0);
}

static void __init zkpk_init(void)
{
	static const int mygpio[] = {
#if 0
		98, 111, 162, 109, 15, 104, 106, 96, 103, 145, 97, 197,
		108, 105, 136, 130, 139, 132, 133, 131, 101, 156, 95, 129, 137, 135, 134
#else
		145, 96, 97, 108, 110, 111
#endif
	};
	int i;

	for (i = 0; i < sizeof(mygpio) / sizeof(mygpio[0]); ++i) {
		gpio_request(mygpio[i], "sysfs");
		gpio_export(mygpio[i], 1);
	}

  gpio_request(101, "SW_OSW_ZARA");
  gpio_direction_output(101, 0);
  gpio_set_value(101, 0);
  gpio_export(101, 1);

  gpio_request(159, "DPSV");
  gpio_direction_output(159, 0);

  if (strcmp(motherb_name, "v4") == 0) {
    gpio_set_value(159, 1);

    gpio_request(97, "LCD_ON");
    gpio_direction_output(97, 0);
    gpio_set_value(97, 0);
    gpio_export(97, 1);

    gpio_request(145, "LCD_SHDN#");
    gpio_direction_output(145, 0);
    gpio_set_value(145, 1);
    gpio_export(145, 1);

    gpio_request(139, "USB_RS_ON");
    gpio_direction_output(139, 1);
    gpio_set_value(139, 1);
    gpio_export(139, 1);

    gpio_request(110, "X_BEE_WIFI_ON");
    gpio_direction_output(110, 1);
    gpio_set_value(110, 1);

    gpio_request(109, "XBEE_DTR");
    gpio_direction_output(109, 0);
    gpio_set_value(109, 0);
    gpio_export(109, 1);

    gpio_set_value(110, 0);
    gpio_export(110, 1);

    gpio_request(126, "XBEE_RES");
    gpio_direction_output(126, 0);
    gpio_set_value(126, 0);
    gpio_export(126, 1);

    gpio_request(157, "DPSH");
    gpio_direction_output(157, 0);
    gpio_export(157, 1);

    gpio_request(111, "USB_OTR_HOST_EN");
    gpio_direction_output(111, 0);
    gpio_export(111, 1);
  } else {
    gpio_set_value(159, 0);
    
    gpio_request(139, "USB_RS_ON");
    gpio_direction_output(139, 0);
    gpio_set_value(139, 1);
    gpio_export(139, 1);

    gpio_request(110, "X_BEE_WIFI_ON");
    gpio_direction_output(110, 0);
    gpio_set_value(110, 0);    

    gpio_request(109, "XBEE_DTR");
    gpio_direction_output(109, 0);
    gpio_set_value(109, 0);
    gpio_export(109, 1);

    gpio_set_value(110, 1);
    gpio_export(110, 1);

    gpio_request(126, "XBEE_RES");
    gpio_direction_output(126, 0);
    gpio_set_value(126, 0);
    gpio_export(126, 1);
  }
}

static void __init omap3_beagle_init(void)
{
	pm_power_off = omap3_pl_poweroff;
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_init_rev();
	omap3_beagle_i2c_init();

	gpio_buttons[0].gpio = beagle_config.usr_button_gpio;

	/* TODO: set lcd_driver_name by command line or device tree */
	beagle_config.lcd_driver_name = "tfc_s9700rtwv35tr-01b",
	lcd_panel.name = beagle_config.lcd_driver_name;

	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_display_init(&beagle_dss_data);
	omap_serial_init();

	omap_mux_init_gpio(170, OMAP_PIN_INPUT);
	/* REVISIT leave DVI powered down until it's needed ... */
	gpio_request_one(170, GPIOF_OUT_INIT_HIGH, "DVI_nPD");

	if(!strcmp(expansionboard_name, "zippy")) 
	{
		printk(KERN_INFO "Beagle expansionboard: initializing enc28j60\n");
		omap3beagle_enc28j60_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
		printk(KERN_INFO "Beagle expansionboard: registering I2C2 for zippy board\n");
		omap_register_i2c_bus(2, 400,  beagle_i2c2_zippy,
							ARRAY_SIZE(beagle_i2c2_zippy));

	}
	
	if(!strcmp(expansionboard_name, "zippy2")) 
	{
		printk(KERN_INFO "Beagle expansionboard: initializing ks_8851\n");
		omap3beagle_ks8851_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
		printk(KERN_INFO "Beagle expansionboard: registering I2C2 for zippy2 board\n");
		omap_register_i2c_bus(2, 400,  beagle_i2c2_zippy,
							ARRAY_SIZE(beagle_i2c2_zippy));		
	}

	if(!strcmp(expansionboard_name, "trainer"))
	{
		printk(KERN_INFO "Beagle expansionboard: exporting GPIOs 130-141,162 to userspace\n");
		gpio_request(130, "sysfs");
		gpio_export(130, 1);
		gpio_request(131, "sysfs");
		gpio_export(131, 1);
		gpio_request(132, "sysfs");
		gpio_export(132, 1);
		gpio_request(133, "sysfs");
		gpio_export(133, 1);
		gpio_request(134, "sysfs");
		gpio_export(134, 1);
		gpio_request(135, "sysfs");
		gpio_export(135, 1);
		gpio_request(136, "sysfs");
		gpio_export(136, 1);
		gpio_request(137, "sysfs");
		gpio_export(137, 1);
		gpio_request(138, "sysfs");
		gpio_export(138, 1);
		gpio_request(139, "sysfs");
		gpio_export(139, 1);
		gpio_request(140, "sysfs");
		gpio_export(140, 1);
		gpio_request(141, "sysfs");
		gpio_export(141, 1);
		gpio_request(162, "sysfs");
		gpio_export(162, 1);
	}

	if(!strcmp(expansionboard_name, "bbtoys-wifi"))
	{
#ifdef CONFIG_WL12XX_PLATFORM_DATA
		if (wl12xx_set_platform_data(&omap_beagle_wlan_data))
			pr_err("error setting wl12xx data\n");
		printk(KERN_INFO "Beagle expansionboard: registering wl12xx bt platform device\n");
		platform_device_register(&wl12xx_device);
		printk(KERN_INFO "Beagle expansionboard: registering wl12xx wifi platform device\n");
		platform_device_register(&omap_vwlan_device);
#else
	pr_err("wl12xx disabled in config\n");
#endif
	}

	if(!strcmp(expansionboard2_name, "bbtoys-ulcd"))
	{
		printk(KERN_INFO "Beagle second expansionboard: registering bbtoys-ulcd\n");
		omap_register_i2c_bus(2, 400,  beagle_i2c2_bbtoys_ulcd,
							ARRAY_SIZE(beagle_i2c2_bbtoys_ulcd));
	}

	if(!strcmp(expansionboard_name, "beacon"))
	{
		printk(KERN_INFO "Beagle expansionboard: registering TinCanTools Beacon LED driver\n");
		platform_device_register(&ws2801_leds);
	}

	pl_init_spi();

	zkpk_init();

	usb_musb_init(NULL);
	usbhs_init(&usbhs_bdata);
	omap_nand_flash_init(NAND_BUSWIDTH_16, omap3beagle_nand_partitions,
			     ARRAY_SIZE(omap3beagle_nand_partitions));

	/* Ensure msecure is mux'd to be able to set the RTC. */
	omap_mux_init_signal("sys_drm_msecure", OMAP_PIN_OFF_OUTPUT_HIGH);

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	beagle_display_init();
	beagle_opp_init();
}

early_param("buddy", expansionboard_setup);
early_param("motherb", motherb_setup);
early_param("buddy2", expansionboard2_setup);

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3_beagle_init_early,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap3_timer,
MACHINE_END
