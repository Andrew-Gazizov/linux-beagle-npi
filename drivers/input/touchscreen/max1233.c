/*
 * File:        drivers/input/touchscreen/max1233.c
 *
 * Based on: 	ads7846.c, ad7877.c
 *
 *		Copyright (C) 2006-2008 Michael Hennerich, Analog Devices Inc.
 *
 * Author:	Michael Hennerich, Analog Devices Inc.
 *
 * Created:	Nov, 10th 2006
 * Description:	MAX1233 based touchscreen, sensor (ADCs), DAC and GPIO driver
 *
 *
 * Modified:
 *
 * Bugs:        Enter bugs at http://blackfin.uclinux.org/
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * History:
 * Copyright (c) 2005 David Brownell
 * Copyright (c) 2006 Nokia Corporation
 * Various changes: Imre Deak <imre.deak@nokia.com>
 *
 * Using code from:
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 */


#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/max1233.h>
#include <asm/irq.h>
#include <asm/io.h>
//#include <asm/portmux.h>



/*
#ifndef CONFIG_FLIP_TS_X 
#define CONFIG_FLIP_TS_X y
#endif

#ifndef CONFIG_FLIP_TS_Y 
#define CONFIG_FLIP_TS_Y y
#endif
*/


#define	TS_PEN_UP_TIMEOUT	msecs_to_jiffies(50)

/*--------------------------------------------------------------------------*/

#define MAX_SPI_FREQ_HZ			20000000
#define	MAX_12BIT			((1<<12)-1)


/*----------------REGISTER NAMES AND IDS ----------------------------------------------------------*/

#define MAX1233_PAGE0			0
#define MAX1233_PAGE1			1


/*---------------- PAGE 1 ----------------------------------------------------------*/

#define MAX1233_REG_X			0
#define MAX1233_REG_Y			1
#define MAX1233_REG_Z1			2
#define MAX1233_REG_Z2			3
#define MAX1233_REG_KPD			4
#define MAX1233_REG_BAT1		5
#define MAX1233_REG_BAT2		6
#define MAX1233_REG_AUX1		7
#define MAX1233_REG_AUX2		8
#define MAX1233_REG_TEMP1		9
#define MAX1233_REG_TEMP2		10
#define MAX1233_REG_DAC			11

#define MAX1233_REG_GPIO		15
#define MAX1233_REG_KPData1		16
#define MAX1233_REG_KPData2		17

/*---------------- PAGE 2 ----------------------------------------------------------*/

#define MAX1233_REG_ADCctrl			0
#define MAX1233_REG_KEYctrl			1
#define MAX1233_REG_DACctrl		2

#define MAX1233_REG_GPIO_PULLUPctrl		14
#define MAX1233_REG_GPIOctrl		15
#define MAX1233_REG_KPKeyMask		16
#define MAX1233_REG_KPColumnMask	17


/*----------------ADC Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_PENSTS(x)			((x & 0x1) << 15)	/* Touch detected		use TSOP */
#define MAX1233_ADSTS(x)			((x & 0x1) << 14)	/* conv.data avail		use TSOP */
/* both above need to be combined for writing selecting operation mode */

#define MAX1233_ADCSCANSEL(x)			((x & 0xF) << 10)	/* 		use function table */
#define MAX1233_RES(x)				((x & 0x3) << 8)	/* 		8/8/10/12 */
#define MAX1233_AVG(x)				((x & 0x3) << 6)	/* 		no/4/8/16 */
#define MAX1233_CNR(x)				((x & 0x3) << 4)	/* 		3.5/3.5/10/100 */
#define MAX1233_STLTIME(x)			((x & 0x7) << 1)	/* 		0/0.1/0.5/1/5/10/50/100 */
#define MAX1233_REFV(x)				((x & 0x1) << 0)	/* 		1V/2.5V */



#define MAX1233_TSOP_SCAN_ONE 		(MAX1233_ADSTS(0) | MAX1233_PENSTS(0))
#define MAX1233_TSOP_DET_SCAN_IRQ 	(MAX1233_ADSTS(0) | MAX1233_PENSTS(1))
#define MAX1233_TSOP_DET_IRQ 		(MAX1233_ADSTS(1) | MAX1233_PENSTS(0))
#define MAX1233_TSOP_DISABLE_DETECT 	(MAX1233_ADSTS(1) | MAX1233_PENSTS(1))

#define MAX1233_ADCSCAN_CONFRES 0
#define MAX1233_ADCSCAN_MEAS_XY 1
#define MAX1233_ADCSCAN_MEAS_XYZ 2
#define MAX1233_ADCSCAN_MEAS_X 3
#define MAX1233_ADCSCAN_MEAS_Y 4
#define MAX1233_ADCSCAN_MEAS_Z 5
#define MAX1233_ADCSCAN_MEAS_BAT1 6
#define MAX1233_ADCSCAN_MEAS_BAT2 7
#define MAX1233_ADCSCAN_MEAS_AUX1 8
#define MAX1233_ADCSCAN_MEAS_AUX2 9
#define MAX1233_ADCSCAN_MEAS_TEMP_SE A
#define MAX1233_ADCSCAN_MEAS_BATAUX B
#define MAX1233_ADCSCAN_MEAS_TEMP_DIFF C
#define MAX1233_ADCSCAN_YDRIVES_ON D
#define MAX1233_ADCSCAN_XDRIVES_ON E
#define MAX1233_ADCSCAN_YXDRIVES_ON F


/*----------------DAC Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_DAC_ENABLE(x)			((x & 0x1) << 15)	/* 		State of DAC */



/*----------------KEYPAD Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_KEYSTS1(x)			((x & 0x1) << 15)	/* PRESS detected		use KEYOP */
#define MAX1233_KEYSTS0(x)			((x & 0x1) << 14)	/* conv.data avail		use KEYOP */
/* both above need to be combined for writing selecting operation mode */

#define MAX1233_DBNTIME(x)			((x & 0x7) << 11)	/* 2/10/20/50/60/80/100/120 */
#define MAX1233_HLDTIME(x)			((x & 0x7) << 8)	/* 100us/1/2/3/4/5/6/7 DEBOUNCE TIMES */



#define MAX1233_KEYOP_SCAN_ONE 		(MAX1233_KEYSTS1(0) | MAX1233_KEYSTS0(0))
#define MAX1233_KEYOP_DET_SCAN_IRQ 	(MAX1233_KEYSTS1(0) | MAX1233_KEYSTS0(1))
#define MAX1233_KEYOP_DET_IRQ 		(MAX1233_KEYSTS1(1) | MAX1233_KEYSTS0(0))
#define MAX1233_KEYOP_DISABLE_DETECT 	(MAX1233_KEYSTS1(1) | MAX1233_KEYSTS0(1))


/*----------------KEYPAD MASK Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_KEY15(x)			((x & 0x1) << 15)	/* 		detect KEY(4,4) */
#define MAX1233_KEY14(x)			((x & 0x1) << 14)	/* 		detect KEY(3,4) */
#define MAX1233_KEY13(x)			((x & 0x1) << 13)	/* 		detect KEY(2,4) */
#define MAX1233_KEY12(x)			((x & 0x1) << 12)	/* 		detect KEY(1,4) */
#define MAX1233_KEY11(x)			((x & 0x1) << 11)	/* 		detect KEY(4,3) */
#define MAX1233_KEY10(x)			((x & 0x1) << 10)	/* 		detect KEY(3,3) */
#define MAX1233_KEY9(x)				((x & 0x1) << 9)	/* 		detect KEY(2,3) */
#define MAX1233_KEY8(x)				((x & 0x1) << 8)	/* 		detect KEY(1,3) */
#define MAX1233_KEY7(x)				((x & 0x1) << 7)	/* 		detect KEY(4,2) */
#define MAX1233_KEY6(x)				((x & 0x1) << 6)	/* 		detect KEY(3,2) */
#define MAX1233_KEY5(x)				((x & 0x1) << 5)	/* 		detect KEY(2,2) */
#define MAX1233_KEY4(x)				((x & 0x1) << 4)	/* 		detect KEY(1,2) */
#define MAX1233_KEY3(x)				((x & 0x1) << 3)	/* 		detect KEY(4,1) */
#define MAX1233_KEY2(x)				((x & 0x1) << 2)	/* 		detect KEY(3,1) */
#define MAX1233_KEY1(x)				((x & 0x1) << 1)	/* 		detect KEY(2,1) */
#define MAX1233_KEY0(x)				((x & 0x1) << 0)	/* 		detect KEY(1,1) */


/*----------------KEYPAD COLUMN MASK Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_COLMASK4(x)			((x & 0x1) << 15)	/* 		detect KEYS(*,4) */
#define MAX1233_COLMASK3(x)			((x & 0x1) << 14)	/* 		detect KEYS(*,3) */
#define MAX1233_COLMASK2(x)			((x & 0x1) << 13)	/* 		detect KEYS(*,2) */
#define MAX1233_COLMASK1(x)			((x & 0x1) << 12)	/* 		detect KEYS(*,1) */


/*----------------GPIO Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_GP7(x)			((x & 0x1) << 15)	/* 		use as GPIO /KEY */
#define MAX1233_GP6(x)			((x & 0x1) << 14)	/* 		use as GPIO /KEY */
#define MAX1233_GP5(x)			((x & 0x1) << 13)	/* 		use as GPIO /KEY */
#define MAX1233_GP4(x)			((x & 0x1) << 12)	/* 		use as GPIO /KEY */
#define MAX1233_GP3(x)			((x & 0x1) << 11)	/* 		use as GPIO /KEY */
#define MAX1233_GP2(x)			((x & 0x1) << 10)	/* 		use as GPIO /KEY */
#define MAX1233_GP1(x)			((x & 0x1) << 9)	/* 		use as GPIO /KEY */
#define MAX1233_GP0(x)			((x & 0x1) << 8)	/* 		use as GPIO /KEY */

/*----------------GPIO PULLUP Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_PU7(x)			((x & 0x1) << 15)	/* 		disable PULLUP  */
#define MAX1233_PU6(x)			((x & 0x1) << 14)	/* 		disable PULLUP */
#define MAX1233_PU5(x)			((x & 0x1) << 13)	/* 		disable PULLUP */
#define MAX1233_PU4(x)			((x & 0x1) << 12)	/* 		disable PULLUP */
#define MAX1233_PU3(x)			((x & 0x1) << 11)	/* 		disable PULLUP */
#define MAX1233_PU2(x)			((x & 0x1) << 10)	/* 		disable PULLUP */
#define MAX1233_PU1(x)			((x & 0x1) << 9)	/* 		disable PULLUP */
#define MAX1233_PU0(x)			((x & 0x1) << 8)	/* 		disable PULLUP */


/* HELPER */

#define MAX1233_PAGEADDR(page, addr) (((0x1 & page)<<6) | (0x003F & addr)) /* use this to generate a register adress from page and reg ddr*/
#define MAX1233_RDADD(x)		(0x8000 | (0x7FFF & (x))) 	/* use with PAGEADDR , MAX1233_RDADD(MAX1233_PAGEADDR(page, addr))*/
#define MAX1233_WRADD(x)		(0x0000 | (0x7FFF & (x)))	/* use with PAGEADDR */

/*for reading a whole part in PAGE0*/
enum {
	MAX1233_SEQ_SND= 0,  /*this position is ocupied during sending of command wrd*/
	MAX1233_SEQ_XPOS  = 1,
	MAX1233_SEQ_YPOS  = 2,
	MAX1233_SEQ_Z1    = 3,
	MAX1233_SEQ_Z2    = 4,
	MAX1233_SEQ_KPD   = 5,
	MAX1233_SEQ_BAT1  = 6,
	MAX1233_SEQ_BAT2  = 7,
	MAX1233_SEQ_AUX1  = 8,
	MAX1233_SEQ_AUX2  = 9,
	MAX1233_SEQ_TEMP1 = 10,
	MAX1233_SEQ_TEMP2 = 11,
	MAX1233_SEQ_DAC = 12,
	MAX1233_NR_SENSE  = 13,
};




/*
 * Non-touchscreen sensors only use single-ended conversions.
 */

/*this is only for  interaction of one word back and forth*/
struct ser_req {
	u16			txbuf[2];
	u16			rxbuf[2];
	struct spi_message	msg;
	struct spi_transfer	xfer[1];
};

struct max1233 {
	struct input_dev	*input;
	char			phys[32];

	struct spi_device	*spi;
	u16			model;
	u16			vref_delay_usecs;
	u16			x_plate_ohms;
	u16			y_plate_ohms;
	u16			xmin;
	u16			xmax;
	u16			ymin;
	u16			ymax;
	u16			pressure_max;

	u16			cmd_crtl1;
	u16			cmd_crtl2;
	u16			cmd_dummy;
	u16			dac;

	u8			stopacq_polarity;
	u8			first_conversion_delay;
	u8			acquisition_time;
	u8			averaging;
	u8			pen_down_acc_interval;

	u16 conversion_send[MAX1233_NR_SENSE];
	u16 conversion_data[MAX1233_NR_SENSE];

	struct spi_transfer	xfer[1];
	struct spi_message	msg;

	int intr_flag;

	spinlock_t		lock;
	struct timer_list	timer;		/* P: lock */
	unsigned		pendown:1;	/* P: lock */
	unsigned		pending:1;	/* P: lock */

	unsigned		irq_disabled:1;	/* P: lock */
	unsigned		disabled:1;
	unsigned		gpio3:1;
	unsigned		gpio4:1;

};

//static int gpio3;

static void max1233_enable(struct max1233 *ts);
static void max1233_disable(struct max1233 *ts);

static int device_suspended(struct device *dev)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	return dev->power.power_state.event != PM_EVENT_ON || ts->disabled;
}


/* this has been worked on and changed from 7877 */
/* reg has to carry complete page and adress description read flag will be set staticly */
/* use MAX1233_RDADD to generate */
static int max1233_read(struct device *dev, u16 reg, u16 *val)
{
	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	int			status;

	if (!req)
		return -ENOMEM;

	spi_message_init(&req->msg);

	req->txbuf[0] = (u16) MAX1233_RDADD(reg);
	req->txbuf[1] = 0; //following dummys
	req->xfer[0].tx_buf = req->txbuf;
	req->xfer[0].rx_buf = req->rxbuf;

	req->xfer[0].len = 4;

	spi_message_add_tail(&req->xfer[0], &req->msg);

	status = spi_sync(spi, &req->msg);

	//printk("max1233_read: (#%X)> %X (status: %X)...(command: %X)\n", reg, buf[1], status, (u16) MAX1233_RDADD(reg));

	if (req->msg.status)
		status = req->msg.status;

	if (!status) *val = req->rxbuf[1]; /* return if no error, else error will return*/

	kfree(req);
	return status;
}

static int max1233_write(struct device *dev, u16 reg, u16 val)
{
	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	int			status;

	if (!req)
		return -ENOMEM;

	spi_message_init(&req->msg);

	req->txbuf[0] = (u16) MAX1233_WRADD (reg);
	req->txbuf[1] = val;
	
	req->xfer[0].tx_buf = req->txbuf;
	req->xfer[0].len = 4;

	spi_message_add_tail(&req->xfer[0], &req->msg);

	status = spi_sync(spi, &req->msg);

	//printk("max1233_write: (#%X) < %X (status: %X)...(command: %X)\n", reg, val, status, (u16) MAX1233_WRADD (reg));

	if (req->msg.status)
		status = req->msg.status;

	kfree(req);

	return status;
}

#define SHOW(name) static ssize_t \
name ## _show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	ssize_t v = max1233_read(dev, \
			MAX1233_READ_CHAN(name)); \
	if (v < 0) \
		return v; \
	return sprintf(buf, "%u\n", (unsigned) v); \
} \
static DEVICE_ATTR(name, S_IRUGO, name ## _show, NULL);

//SHOW(aux1)
//SHOW(aux2)
//SHOW(aux3)
//SHOW(bat1)
//SHOW(bat2)
//SHOW(temp1)
//SHOW(temp2)

static ssize_t max1233_disable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t max1233_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;
	int i;

	i = simple_strtoul(buf, &endp, 10);

	if (i)
		max1233_disable(ts);
	else
		max1233_enable(ts);

	return count;
}

static DEVICE_ATTR(disable, 0664, max1233_disable_show, max1233_disable_store);

static ssize_t max1233_dac_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);
	u16 i;


	max1233_read(dev, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_DAC), &i); //setup

	ts->dac = i & 0xFF;

	return sprintf(buf, "%u\n", ts->dac);
}

static ssize_t max1233_dac_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;
	int i;

	i = simple_strtoul(buf, &endp, 10);

	ts->dac = i & 0xFF;

	max1233_write(dev, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_DAC), ts->dac); //setup

	return count;
}

static DEVICE_ATTR(dac, 0664, max1233_dac_show, max1233_dac_store);




static u16 regaddr; /* this stores the addresses for operation */

static ssize_t max1233_regaddr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", regaddr);
}

static ssize_t max1233_regaddr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	regaddr = simple_strtoul(buf, &endp, 10);

	return count;
}

static DEVICE_ATTR(regaddr, 0664, max1233_regaddr_show, max1233_regaddr_store);



static u16 pageaddr; /* this stores the addresses for operation */

static ssize_t max1233_pageaddr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pageaddr);
}

static ssize_t max1233_pageaddr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	pageaddr = simple_strtoul(buf, &endp, 10);

	return count;
}

static DEVICE_ATTR(pageaddr, 0664, max1233_pageaddr_show, max1233_pageaddr_store);



static ssize_t max1233_regio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);
	
	u16 result = 0;
	u16 err = 0;

	err = max1233_read(dev, MAX1233_RDADD(MAX1233_PAGEADDR(pageaddr, regaddr)), &result);
	
	return sprintf(buf, "PAGE%u:REG%u > (%u) (%d) (%X)   (error: %u)\n", pageaddr, regaddr, result, result, result, err);

}

static ssize_t max1233_regio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;
	int i;
	
	u16 err = 0;

	i = simple_strtoul(buf, &endp, 10);

	err = max1233_write(dev, (u16) MAX1233_WRADD(MAX1233_PAGEADDR(pageaddr, regaddr)), i);

	return count;
}

static DEVICE_ATTR(regio, 0664, max1233_regio_show, max1233_regio_store);







static ssize_t max1233_dumpio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	u16 			*buff = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL); /*tx*/
	u16 			*buff2 = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL);
	int status;
	size_t s;
	int i;

	if (!req | ! buff)
		return -ENOMEM;

	spi_message_init(&req->msg);

	/*the max1233 needs 32 clock cycles to start sending ahead in the page after first word (word--32 clocks--more words in page*/

	buff[0] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(pageaddr, 0));
	req->xfer[0].tx_buf = buff;
	req->xfer[0].len = (1+32)*2;
	req->xfer[0].rx_buf = buff2;

	spi_message_add_tail(&req->xfer[0], &req->msg);

	status = spi_sync(spi, &req->msg);




	printk("max1233_dumpio: (#%X)>... (status: %X)...(command: %X)\n", MAX1233_PAGEADDR(pageaddr, 0), status, buff[0]);

	s = 0;
	
	for(i = 0;i<32;i++) /*in 0 we have nothing, there was the sending word*/
	{
		
		s += sprintf(buf + s, "PAGE%u:REG%u: > (%u) (%d) (%X)\n", pageaddr, i, buff2[i+1],buff2[i+1],buff2[i+1]);

	}
	s += sprintf(buf + s, "PAGE %u:end\n", pageaddr);


	kfree(buff);
	kfree(buff2);
	kfree(req);

	return s;



}

static DEVICE_ATTR(dumpio, 0664, max1233_dumpio_show, NULL);





static u16 doirqio; /* this stores the addresses for operation */

static ssize_t max1233_doirqio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", doirqio);
}

static ssize_t max1233_doirqio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	doirqio = simple_strtoul(buf, &endp, 10);

	return count;
}

static DEVICE_ATTR(doirqio, 0664, max1233_doirqio_show, max1233_doirqio_store);



static u16 toggleio; /* this stores the addresses for operation */

static ssize_t max1233_toggleio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", toggleio);
}

static ssize_t max1233_toggleio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	toggleio = simple_strtoul(buf, &endp, 10);

		if(toggleio) spi_async(ts->spi, &ts->msg); /* toggle retrieve of PAGE0 first 10 words*/


	return count;
}

static DEVICE_ATTR(toggleio, 0664, max1233_toggleio_show, max1233_toggleio_store);






static u32 ioaddr; /* this stores the addresses for operation */

static ssize_t max1233_ioaddr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%X\n", ioaddr);
}

static ssize_t max1233_ioaddr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	ioaddr = simple_strtoul(buf, &endp, 16);

	return count;
}

static DEVICE_ATTR(ioaddr, 0664, max1233_ioaddr_show, max1233_ioaddr_store);





static u16 doio; /* this stores the addresses for operation */

static ssize_t max1233_doio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

//	doio = bfin_read16(ioaddr);
	doio = readw(ioaddr);

	return sprintf(buf, "%X\n", doio);
}

static ssize_t max1233_doio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	doio = simple_strtoul(buf, &endp, 16);
//	bfin_write16(ioaddr, doio);
	writew(doio, ioaddr);

	return count;
}

static DEVICE_ATTR(doio, 0664, max1233_doio_show, max1233_doio_store);
















static struct attribute *max1233_attributes[] = {
/*	&dev_attr_temp1.attr,
	&dev_attr_temp2.attr,
	&dev_attr_aux1.attr,
	&dev_attr_aux2.attr,
	&dev_attr_bat1.attr,
	&dev_attr_bat2.attr,*/
	&dev_attr_disable.attr,
	&dev_attr_dac.attr,
	//&dev_attr_gpio4.attr,
&dev_attr_regaddr.attr,
&dev_attr_pageaddr.attr,
&dev_attr_regio.attr,
&dev_attr_dumpio.attr,
&dev_attr_doirqio.attr,
&dev_attr_toggleio.attr,
&dev_attr_ioaddr.attr,
&dev_attr_doio.attr,

	NULL
};

static const struct attribute_group max1233_attr_group = {
	.attrs = max1233_attributes,
};

/*--------------------------------------------------------------------------*/

/*
 * /DAV Data available Interrupt only kicks the kthread.
 * The kthread kicks the timer only to issue the Pen Up Event if
 * no new data is available
 *
 */

static void max1233_rx(void *ads)
{
	struct max1233		*ts = ads;
	struct input_dev	*input_dev = ts->input;
	unsigned		Rt;
	unsigned		p2;
	u16			x, y, z1, z2;

	x = ts->conversion_data[MAX1233_SEQ_XPOS] & MAX_12BIT;
	y = ts->conversion_data[MAX1233_SEQ_YPOS]& MAX_12BIT;
	z1 = ts->conversion_data[MAX1233_SEQ_Z1] & MAX_12BIT;
	z2 = ts->conversion_data[MAX1233_SEQ_Z2] & MAX_12BIT;

	/* range filtering */
	if (x == MAX_12BIT)
		x = 0;

	if (likely(x && z1 && (y != MAX_12BIT) && !device_suspended(&ts->spi->dev))) {
		/* compute touch pressure resistance using equation #2 */
		

/* AD7877 orig, this was equation 1*/

/*
		Rt = z2;
		Rt -= z1;
		Rt *= x;
		Rt *= ts->x_plate_ohms;
		Rt /= z1;
		Rt = (Rt + 2047) >> 12;
		Rt &= MAX_12BIT;	//added, FIXME
*/


		Rt = MAX_12BIT - z1 ;
		Rt *= x;
		Rt *= ts->x_plate_ohms;
		Rt /= z1;

		p2 = MAX_12BIT - y;
		p2 *= ts->y_plate_ohms;

		Rt -= p2;
		Rt >>= 12;






	} else
		Rt = 0;

#ifdef CONFIG_FLIP_TS_X
	x = ts->xmax - x + ts->xmin;
#endif

#ifdef CONFIG_FLIP_TS_Y
	y = ts->ymax - y + ts->ymin;
#endif

Rt = 1000;


	if (Rt) {
	

		input_report_abs(input_dev, ABS_X, x);
		input_report_abs(input_dev, ABS_Y, y);
		input_report_abs(input_dev, ABS_PRESSURE, Rt);
		input_sync(input_dev);
	}

	if (doirqio) printk("Reporting from: %s: %d/%d/%d%s\n", dev_name(&ts->spi->dev), x, y, Rt, Rt ? "" : " UP");


#ifdef	VERBOSE
	if (Rt)
		pr_debug("%s: %d/%d/%d%s\n", ts->spi->dev.bus_id,
			x, y, Rt, Rt ? "" : " UP");
#endif

}



static inline void max1233_ts_event_release(struct max1233 *ts)
{
	struct input_dev *input_dev = ts->input;

	input_report_abs(input_dev, ABS_PRESSURE, 0);
	input_sync(input_dev);

	if (doirqio) printk("Reporting from(timer): %s: %d\n", dev_name(&ts->spi->dev), 0);


}

static void max1233_timer(unsigned long handle)
{
	struct max1233	*ts = (void *)handle;

	max1233_ts_event_release(ts);
}



static irqreturn_t max1233_irq(int irq, void *handle)
{
	struct max1233 *ts = handle;
	unsigned long flags;
	int status;

	spin_lock_irqsave(&ts->lock, flags);

// 	if (!ts->irq_disabled) {
// 		ts->irq_disabled = 1;
// 		disable_irq(ts->spi->irq);
// 		ts->pending = 1;
// 	}

	ts->intr_flag = 1;

	spin_unlock_irqrestore(&ts->lock, flags);

	status = spi_async(ts->spi, &ts->msg); /* toggle retrieve of PAGE0 first 10 words*/
	//printk("max1233_irq: retrieved...\n");

	if (status)
		dev_err(&ts->spi->dev, "spi_sync --> %d\n", status);


	return IRQ_HANDLED;
}


static void max1233_callback(void *_ts)
{
//this is called after the read request from isr completed and we received back some data, which is stored in ts, it is processed in max1233_rx
	struct max1233 *ts = _ts;
	unsigned long flags;

	if (ts->intr_flag) {

		max1233_rx(ts); /*compute and publish the touchscreen stuff*/

		spin_lock_irqsave(&ts->lock, flags);

		ts->intr_flag = 0;
		//ts->pending = 0;

		if (!device_suspended(&ts->spi->dev)) {
			//ts->irq_disabled = 0;
			//enable_irq(ts->spi->irq);
			mod_timer(&ts->timer, jiffies + TS_PEN_UP_TIMEOUT);
		}

		spin_unlock_irqrestore(&ts->lock, flags);
	}
}


/*--------------------------------------------------------------------------*/

/* Must be called with ts->lock held */
static void max1233_disable(struct max1233 *ts)
{
	unsigned long flags;

	if (ts->disabled)
		return;

	ts->disabled = 1;

	if (!ts->pending) {
		spin_lock_irqsave(&ts->lock, flags);
		ts->irq_disabled = 1;
		disable_irq(ts->spi->irq);
		spin_unlock_irqrestore(&ts->lock, flags);
	} else {
		/* the kthread will run at least once more, and
		 * leave everything in a clean state, IRQ disabled
		 */
		while (ts->pending)
			msleep(1);

	}

	/* we know the chip's in lowpower mode since we always
	 * leave it that way after every request
	 */

}

/* Must be called with ts->lock held */
static void max1233_enable(struct max1233 *ts)
{
	unsigned long flags;

	if (!ts->disabled)
		return;

	spin_lock_irqsave(&ts->lock, flags);
	ts->disabled = 0;
	ts->irq_disabled = 0;
	enable_irq(ts->spi->irq);
	spin_unlock_irqrestore(&ts->lock, flags);
}

static int max1233_suspend(struct spi_device *spi, pm_message_t message)
{
	struct max1233 *ts = dev_get_drvdata(&spi->dev);

	spi->dev.power.power_state = message;
	max1233_disable(ts);

	return 0;

}

static int max1233_resume(struct spi_device *spi)
{
	struct max1233 *ts = dev_get_drvdata(&spi->dev);

	spi->dev.power.power_state = PMSG_ON;
	max1233_enable(ts);

	return 0;
}

static inline void max1233_setup_ts_def_msg(struct spi_device *spi, struct max1233 *ts)
{
	
	struct spi_message	*m;

	ts->cmd_crtl2 = MAX1233_DAC_ENABLE(1); //turn on dac

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_DACctrl), ts->cmd_crtl2);


	//max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), 0x100); /*turn always on the adc reference */


	ts->cmd_crtl1 = MAX1233_TSOP_DET_SCAN_IRQ |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_XYZ) |
			MAX1233_RES(3) |
			MAX1233_AVG(1) |
			MAX1233_CNR(2) |
			MAX1233_STLTIME(3) |
			MAX1233_REFV(1);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), ts->cmd_crtl1); //setup



	/* for retrieving the measurements we need to prepare a structure for the IRS which will be called there*/

	ts->cmd_dummy = 0;

	m = &ts->msg;

	spi_message_init(m);

	m->complete = max1233_callback;
	m->context = ts; /*HANDLE to the callback*/


	ts->conversion_send[MAX1233_SEQ_SND] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_X));

	ts->xfer[0].tx_buf = ts->conversion_send;
	ts->xfer[0].rx_buf = ts->conversion_data; /*index 1 and 2 are the 32 clocks dummys*/

	ts->xfer[0].len = MAX1233_NR_SENSE*2; /*+1 word is already done in the enum*/

	spi_message_add_tail(&ts->xfer[0], m);


}

static int __devinit max1233_probe(struct spi_device *spi)
{
	struct max1233			*ts;
	struct input_dev		*input_dev;
	struct max1233_platform_data	*pdata = spi->dev.platform_data;
	int				err;
	u16				verify;

	printk("max1233_probe: tries probe...\n");

	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_dbg(&spi->dev, "SPI CLK %d Hz?\n",spi->max_speed_hz);
		return -EINVAL;
	}

	ts = kzalloc(sizeof(struct max1233), GFP_KERNEL); //create driver struct
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts); //register the loval handler struct ts with driver in kernel
	spi->dev.power.power_state = PMSG_ON;

	ts->spi = spi;
	ts->input = input_dev;
	ts->intr_flag = 0;

	setup_timer(&ts->timer, max1233_timer, (unsigned long) ts); //TIMER registers ts as HANDLER struct

	spin_lock_init(&ts->lock);

	ts->model = pdata->model ? : 1233;
	ts->vref_delay_usecs = pdata->vref_delay_usecs ? : 100;
	ts->x_plate_ohms = pdata->x_plate_ohms ? : 400;
	ts->y_plate_ohms = pdata->y_plate_ohms ? : 400;
	ts->pressure_max = pdata->pressure_max ? : ~0;

	ts->xmin = pdata->x_min;
	ts->xmax = pdata->x_max ? : MAX_12BIT;
	ts->ymin = pdata->y_min;
	ts->ymax = pdata->y_max ? : MAX_12BIT;
	

	ts->stopacq_polarity = pdata->stopacq_polarity;
	ts->first_conversion_delay = pdata->first_conversion_delay;
	ts->acquisition_time = pdata->acquisition_time;
	ts->averaging = pdata->averaging;
	ts->pen_down_acc_interval = pdata->pen_down_acc_interval;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&spi->dev));

	input_dev->name = "MAX1233 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, 
			pdata->pressure_max, 
			0, 0);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_KPKeyMask), 0xF00F); /* set keymask and read it again*/

	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_KPKeyMask), &verify);

	if (verify != 0xF00F){
	//if (false){
		printk(KERN_ERR "%s:esFailed to probe %s, (%u != 0xF00F)\n", dev_name(&spi->dev),
			 input_dev->name, verify);
		err = -ENODEV;
		goto err_free_mem;
	}

	printk("max1233_probe: probe OK....\n");



//	if (gpio3)
//		max1233_write((struct device *) spi, MAX1233_REG_EXTWRITE,
//			 MAX1233_EXTW_GPIO_3_CONF);

	max1233_setup_ts_def_msg(spi, ts); //general setup

	/* Request MAX1233 /DAV GPIO interrupt */

	if (request_irq(spi->irq, max1233_irq, IRQF_TRIGGER_RISING | IRQF_SAMPLE_RANDOM,
			spi->dev.driver->name, ts)) { //ISR will receive ts as HANDLER struct
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err_free_mem;
	}

	dev_info(&spi->dev, "touchscreen, irq %d\n", spi->irq);

	err = sysfs_create_group(&spi->dev.kobj, &max1233_attr_group);
	if (err)
		goto err_remove_attr;

//	if (gpio3)
//		err = device_create_file(&spi->dev, &dev_attr_gpio3);
//	else
//		err = device_create_file(&spi->dev, &dev_attr_aux3);

	if (err)
		goto err_remove_attr;

	err = input_register_device(input_dev);
	if (err)
		goto err_idev;

	ts->intr_flag = 0;

	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_X), &verify); /*just make sure we startet good, ping max1233 to activate penirq*/

	printk("max1233_probe: probe OK. Touchscreen enabled...\n");


	return 0;

err_idev:
	input_dev = NULL; /* so we don't try to free it later */

err_remove_attr:

	sysfs_remove_group(&spi->dev.kobj, &max1233_attr_group);

//	if (gpio3)
//		device_remove_file(&spi->dev, &dev_attr_gpio3);
//	else
//		device_remove_file(&spi->dev, &dev_attr_aux3);

	free_irq(spi->irq, ts);

err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	dev_set_drvdata(&spi->dev, NULL);
	return err;
}

static int __devexit max1233_remove(struct spi_device *spi)
{
	struct max1233		*ts = dev_get_drvdata(&spi->dev); //reference allocated driver struct in kernel

	max1233_suspend(spi, PMSG_SUSPEND);

	sysfs_remove_group(&spi->dev.kobj, &max1233_attr_group);

//	if (gpio3)
//		device_remove_file(&spi->dev, &dev_attr_gpio3);
//	else
//		device_remove_file(&spi->dev, &dev_attr_aux3);

	free_irq(ts->spi->irq, ts);

	input_unregister_device(ts->input);

	kfree(ts); //free the driver struct

	dev_dbg(&spi->dev, "unregistered touchscreen\n");
	dev_set_drvdata(&spi->dev, NULL);

	return 0;
}

static struct spi_driver max1233_driver = {
	.driver = {
		.name	= "max1233",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= max1233_probe,
	.remove		= __devexit_p(max1233_remove),
	.suspend	= max1233_suspend,
	.resume		= max1233_resume,
};

static int __init max1233_init(void)
{
	return spi_register_driver(&max1233_driver);
}
module_init(max1233_init);

static void __exit max1233_exit(void)
{
	spi_unregister_driver(&max1233_driver);
}
module_exit(max1233_exit);

//module_param(gpio3, int, 0);
//MODULE_PARM_DESC(gpio3,
//	"If gpio3 is set to 1 AUX3 acts as GPIO3");

MODULE_DESCRIPTION("max1233 TouchScreen Driver");
MODULE_LICENSE("GPL");
