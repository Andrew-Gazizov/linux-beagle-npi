/*
 * Support for NEC-nl2432hc17-07b panel driver
 *
 * Copyright (C) 2010 Texas Instruments Inc.
 * Author: Erik Gilling <konkers@android.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <video/omapdss.h>

struct drv_data {
    struct omap_dss_device *dssdev;
};

//#define BBL

//struct nec_2432_data {
//	struct backlight_device *bl;
//};
/*
    VALUES FROM DATASHEAT NL2432HC17-07B DEVICES
*/

static const struct {
    unsigned char addr;
    unsigned short dat;
} nec_2432_init_seq[] = {

    //    { 3, 0x01 }, { 0, 0x00 }, { 1, 0x01 }, { 4, 0x00 }, { 5, 0x14 },
    //	{ 6, 0x24 }, { 16, 0xD7 }, { 17, 0x00 }, { 18, 0x00 }, { 19, 0x55 },
    //	{ 20, 0x01 }, { 21, 0x70 }, { 22, 0x1E }, { 23, 0x25 },	{ 24, 0x25 },
    { 3,   0x01 }, { 1,   0x00 }, { 100, 0x0F }, { 101, 0x37 }, { 102, 0x3D },
    { 103, 0x04 }, { 104, 0x00 }, { 105, 0x30 }, { 106, 0x84 }, { 107, 0x05 },
    { 108, 0x17 }, { 109, 0x62 }, { 110, 0x50 }, { 111, 0x30 }, { 112, 0x73 },
    { 113, 0x07 }, { 114, 0x66 }, { 115, 0x51 }, { 116, 0x50 }, { 2,   0x40 },
    { 75,  0x04 }, { 76,  0x01 }, { 77,  0x01 }, { 80,  0x00 },	{ 81,  0x00 },
    { 82,  0x24 }, { 83,  0xA1 }, { 86,  0x15 }, { 87,  0xF0 }, { 95,  0x3F },
    { 96,  0x22 }, { 25,  0x76 }, { 26,  0x54 }, { 27,  0x6B }, { 28,  0x60 },
    { 29,  0x04 }, { 30,  0x1C }, { 31,  0xA1 }, { 32,  0x00 }, { 33,  0x20 },
    { 24,  0x77 },
    //	{ 25, 0x02 }, { 26, 0x02 }, { 27, 0xA0 }, { 32, 0x2F }, { 33, 0x0F },
    { 59,  0x01 },
    //	{ 34, 0x0F }, { 35, 0x0F }, { 36, 0x0F }, { 37, 0x0F },	{ 38, 0x0F },
    //	{ 39, 0x00 }, { 40, 0x02 }, { 41, 0x02 }, { 42, 0x02 },	{ 43, 0x0F },
    //	{ 44, 0x0F }, { 45, 0x0F }, { 46, 0x0F }, { 47, 0x0F },	{ 48, 0x0F },
    //	{ 49, 0x0F }, { 50, 0x00 }, { 51, 0x02 }, { 52, 0x02 }, { 53, 0x02 },
    //	{ 80, 0x0C }, { 83, 0x42 }, { 84, 0x42 }, { 85, 0x41 },	{ 86, 0x14 },
    //	{ 89, 0x88 }, { 90, 0x01 }, { 91, 0x00 }, { 92, 0x02 },	{ 93, 0x0C },
    //	{ 94, 0x1C }, { 95, 0x27 }, { 98, 0x49 }, { 99, 0x27 }, { 102, 0x76 },
    //	{ 103, 0x27 }, { 112, 0x01 }, { 113, 0x0E }, { 114, 0x02 },
    //	{ 115, 0x0C }, { 118, 0x0C }, { 121, 0x30 }, { 130, 0x00 },
    //	{ 131, 0x00 }, { 132, 0xFC }, { 134, 0x00 }, { 136, 0x00 },
    //	{ 138, 0x00 }, { 139, 0x00 }, { 140, 0x00 }, { 141, 0xFC },
    //	{ 143, 0x00 }, { 145, 0x00 }, { 147, 0x00 }, { 148, 0x00 },
    //	{ 149, 0x00 }, { 150, 0xFC }, { 152, 0x00 }, { 154, 0x00 },
    //	{ 156, 0x00 }, { 157, 0x00 }, { 2, 0x00 },
    { 0,   0x0 }
};

/*
 * NEC NL2432HC17-07B  Manual
 * defines HFB, HSW, HBP, VFP, VSW, VBP as shown below
 */

static struct omap_video_timings nec_2432_panel_timings = {
    /* 240 x 320 @ 60 Hz  Reduced blanking VESA CVT 0.31M3-R */
    .x_res = 240,
    .y_res = 320,

    .pixel_clock	= 4965,
    .hfp		= 4,
    .hbp		= 4,
    .hsw		= 8,
    .vfp		= 1,
    .vbp		= 3,
    .vsw		= 2,
};
/*static int nl2432_write_reg(struct spi_device *spi, u8 reg, u8 val)
{
    struct spi_message msg;
    struct spi_transfer index_xfer = {
        .len		= 4,
//		.cs_change	= 1,
    };
    u8	buffer[4];
    int r;

    spi_message_init(&msg);

    // register index
    buffer[0] = 0x00;
    buffer[1] = reg;
    buffer[2] = 0x01;
    buffer[3] = val;
    index_xfer.tx_buf = buffer;
    spi_message_add_tail(&index_xfer, &msg);

    gpio_set_value(174, 0);
    r=spi_sync(spi, &msg);
    gpio_set_value(174, 1);

    return r;
}*/

static int nec_2432_bl_update_status(struct backlight_device *bl)
{
    struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
    int level;

    if (!dssdev->set_backlight)
        return -EINVAL;

    if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
            bl->props.power == FB_BLANK_UNBLANK)
        level = bl->props.brightness;
    else
        level = 0;

#ifdef DEBUG
    printk("brightness_level %d\n", level);

#endif
    return -1;//dssdev->set_backlight(dssdev, level);
}

static int nec_2432_bl_get_brightness(struct backlight_device *bl)
{
    int level;
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_bl_get_brightness \n");
#endif
    if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
            bl->props.power == FB_BLANK_UNBLANK)
    {
        level = bl->props.brightness;

#ifdef DEBUG
        printk("brightness_level %d\n", level);
#endif
        return level;
    }

    return 0;
}

static const struct backlight_ops nec_2432_bl_ops = {
    .get_brightness	= nec_2432_bl_get_brightness,
    .update_status	= nec_2432_bl_update_status,
};

static int nec_2432_panel_probe(struct omap_dss_device *dssdev)
{
    struct drv_data *necd = NULL;
    struct backlight_properties props;

#ifdef DEBUG
    printk(KERN_INFO "nec_2432_panel_probe \n");
#endif
    dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
            OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF |
            OMAP_DSS_LCD_ONOFF;
    dssdev->panel.timings = nec_2432_panel_timings;
    dssdev->panel.acb = 0x0;
    dssdev->panel.acbi = 0x0;

    necd = kzalloc(sizeof(*necd), GFP_KERNEL);
    if (!necd)
        return -ENOMEM;

    necd->dssdev = dssdev;

    dev_set_drvdata(&dssdev->dev, necd);

    memset(&props, 0, sizeof(struct backlight_properties));
    props.max_brightness = 100;

#ifdef BBL
    struct backlight_device *bl;
    bl = backlight_device_register("nec-2432", &dssdev->dev, dssdev,
                                   &nec_2432_bl_ops, &props);
    if (IS_ERR(bl)) {
        r = PTR_ERR(bl);
        kfree(necd);
        return r;
    }
    necd->bl = bl;

    bl->props.fb_blank = FB_BLANK_UNBLANK;
    bl->props.power = FB_BLANK_UNBLANK;
    bl->props.max_brightness = dssdev->max_backlight_level;
    bl->props.brightness = dssdev->max_backlight_level;

    r = nec_2432_bl_update_status(bl);
    if (r < 0)
        dev_err(&dssdev->dev, "failed to set lcd brightness\n");
#endif

#ifdef DEBUG
    printk(KERN_INFO "nec_2432_panel_probe OK \n");
#endif
    return 0;
}

static void nec_2432_panel_remove(struct omap_dss_device *dssdev)
{
    struct nec_2432_data *necd = dev_get_drvdata(&dssdev->dev);
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_panel_remove \n");
#endif
#ifdef BBL
    struct backlight_device *bl = necd->bl;

    bl->props.power = FB_BLANK_POWERDOWN;
    nec_2432_bl_update_status(bl);
    backlight_device_unregister(bl);
#endif

    kfree(necd);
}

static int nec_2432_panel_enable(struct omap_dss_device *dssdev)
{
    int r = 0;

    printk(KERN_INFO "nec_2432_panel_enable \n");
    if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
        return 0;

#ifdef BBL
    struct drv_data *necd = dev_get_drvdata(&dssdev->dev);

    struct backlight_device *bl = necd->bl;

    r = nec_2432_bl_update_status(bl);
    if (r < 0)
        dev_err(&dssdev->dev, "failed to set lcd brightness\n");
#endif

    if (dssdev->platform_enable) {
        r = dssdev->platform_enable(dssdev);
        if (r)
            return r;
    }

    r = omapdss_dpi_display_enable(dssdev);

    if (r)
        return r;

    dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

    return r;
}

static void nec_2432_panel_disable(struct omap_dss_device *dssdev)
{
    printk(KERN_INFO "nec_2432_panel_disable \n");

    if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED)
        return;

    omapdss_dpi_display_disable(dssdev);

#ifdef BBL
    struct drv_data *necd = dev_get_drvdata(&dssdev->dev);

    struct backlight_device *bl = necd->bl;
    bl->props.brightness = 0;
    nec_2432_bl_update_status(bl);
#endif

    if (dssdev->platform_disable)
        dssdev->platform_disable(dssdev);

    dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int nec_2432_panel_suspend(struct omap_dss_device *dssdev)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_panel_suspend \n");
#endif
    nec_2432_panel_disable(dssdev);
    return 0;
}

static int nec_2432_panel_resume(struct omap_dss_device *dssdev)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_panel_resume \n");
#endif
    return nec_2432_panel_enable(dssdev);
}

static int nec_2432_recommended_bpp(struct omap_dss_device *dssdev)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_recommended_bpp \n");
#endif
    return 16;
}

static void nec_2432_set_timings(struct omap_dss_device *dssdev,
                                 struct omap_video_timings *timings)
{
    printk(KERN_INFO "nec_2432_set_timings \n");
    dpi_set_timings(dssdev, timings);
}

static void nec_2432_get_timings(struct omap_dss_device *dssdev,
                                 struct omap_video_timings *timings)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_get_timings\n");
#endif
    *timings = dssdev->panel.timings;
}

static int nec_2432_check_timings(struct omap_dss_device *dssdev,
        struct omap_video_timings *timings)
{
    return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver nec_2432_driver = {
    .probe			= nec_2432_panel_probe,
    .remove			= nec_2432_panel_remove,
    .enable			= nec_2432_panel_enable,
    .disable		= nec_2432_panel_disable,
    .suspend		= nec_2432_panel_suspend,
    .resume			= nec_2432_panel_resume,
    .set_timings	= nec_2432_set_timings,
    .get_timings	= nec_2432_get_timings,
    .check_timings	= nec_2432_check_timings,
    .get_recommended_bpp	= nec_2432_recommended_bpp,

    .driver		= {
        .name		= "nec_2432_panel",
        .owner		= THIS_MODULE,
    },
};

static int nec_2432_spi_send(struct spi_device *spi, unsigned char reg_addr,
                             unsigned char reg_data)
{
    int ret = 0;
    unsigned int cmd = 0, data = 0;

    cmd = 0x0000 | reg_addr; /* register address write */
    data = 0x0100 | reg_data ; /* register data write */
    data = (cmd << 16) | data;

    ret = spi_write(spi, (unsigned char *)&data, 4);
    if (ret)
        pr_err("error in spi_write %x\n", data);

    return ret;
}

static int init_nec_2432_wvga_lcd(struct spi_device *spi)
{
    unsigned int i;
    /* Initialization Sequence */
    /* nec_2432_spi_send(spi, REG, VAL) */
    for (i = 0; i < (ARRAY_SIZE(nec_2432_init_seq) - 2); i++)
        nec_2432_spi_send(spi, nec_2432_init_seq[i].addr,
                          nec_2432_init_seq[i].dat);

    udelay(30);
    nec_2432_spi_send(spi, nec_2432_init_seq[i].addr,
                      nec_2432_init_seq[i].dat);
    i++;

    mdelay(20);
    nec_2432_spi_send(spi, nec_2432_init_seq[i].addr,
                      nec_2432_init_seq[i].dat);

    return 0;
}

static int nec_2432_spi_probe(struct spi_device *spi)
{
    int r;
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_spi_probe\n");
#endif
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 32;
    spi_setup(spi);


    r = omap_dss_register_driver(&nec_2432_driver);
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_spi_probe res = %d\n", r);
#endif



    return r;
}

static int nec_2432_spi_remove(struct spi_device *spi)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_spi_remove\n");
#endif
    omap_dss_unregister_driver(&nec_2432_driver);

    return 0;
}

static int nec_2432_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_spi_suspend\n");
//    nec_2432_spi_send(spi, 0, 0x01);
#endif
    nec_2432_spi_send(spi, 0, 0x01);
    mdelay(40);

    return 0;
}

static int nec_2432_spi_resume(struct spi_device *spi)
{
#ifdef DEBUG
    printk(KERN_INFO "nec_2432_spi_resume\n");
#endif
    /* reinitialize the panel */
    spi_setup(spi);
    init_nec_2432_wvga_lcd(spi);

    return 0;
}

static struct spi_driver nec_2432_spi_driver = {
    .probe		= nec_2432_spi_probe,
    .remove		= __devexit_p(nec_2432_spi_remove),
    .suspend	= nec_2432_spi_suspend,
    .resume		= nec_2432_spi_resume,

    .driver		= {
        .name	= "nec_2432_panel-spi",
        .bus	= &spi_bus_type,
        .owner	= THIS_MODULE,
    },
};

static int __init nec_2432_lcd_init(void)
{
    return spi_register_driver(&nec_2432_spi_driver);
}

static void __exit nec_2432_lcd_exit(void)
{
    return spi_unregister_driver(&nec_2432_spi_driver);
}

module_init(nec_2432_lcd_init);
module_exit(nec_2432_lcd_exit);
MODULE_AUTHOR("Erik Gilling <konkers@android.com>");
MODULE_DESCRIPTION("NEC-nl2432hl17-07b Driver");
MODULE_LICENSE("GPL");
