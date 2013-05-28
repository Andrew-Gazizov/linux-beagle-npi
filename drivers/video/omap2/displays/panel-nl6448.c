#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <video/omapdss.h>

static struct omap_video_timings nl6448_timings = {
	.x_res = 640,
	.y_res = 480,

	.pixel_clock	= 23500,

	.hsw		= 96,
	.hfp		= 16,
	.hbp		= 48,

	.vsw		= 2,
	.vfp		= 12,
	.vbp		= 31,
};

static int nl6448_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.acb = 0;
	dssdev->panel.timings = nl6448_timings;

	return 0;
}

static void nl6448_panel_remove(struct omap_dss_device *dssdev)
{
}

static int nl6448_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void nl6448_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int nl6448_panel_suspend(struct omap_dss_device *dssdev)
{
	nl6448_panel_disable(dssdev);
	return 0;
}

static int nl6448_panel_resume(struct omap_dss_device *dssdev)
{
	return nl6448_panel_enable(dssdev);
}

static struct omap_dss_driver nl6448_driver = {
	.probe		= nl6448_panel_probe,
	.remove		= nl6448_panel_remove,

	.enable		= nl6448_panel_enable,
	.disable	= nl6448_panel_disable,
	.suspend	= nl6448_panel_suspend,
	.resume		= nl6448_panel_resume,

	.driver         = {
		.name   = "nl6448_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init nl6448_panel_drv_init(void)
{
	return omap_dss_register_driver(&nl6448_driver);
}

static void __exit nl6448_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&nl6448_driver);
}

module_init(nl6448_panel_drv_init);
module_exit(nl6448_panel_drv_exit);
MODULE_LICENSE("GPL");
