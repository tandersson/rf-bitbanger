/*
 * rfbb.c
 *
 * rfbb -   Device driver that transmits and records pulse 
 *          and pause-lengths using gpio. 
 *          Previous name rf_bitbanger.c.
 *          Based on lirc_serial.c by Ralph Metzler et al
 *          Uses code parts from the lirc framework (www.lirc.org).
 *
 *  Copyright (C) 2010, 2012 Tord Andersson <tord.andersson@endian.se>
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
#error "**********************************************************"
#error " Sorry, this driver needs kernel version 2.6.32 or higher "
#error "**********************************************************"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
#include <linux/autoconf.h>
#endif

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/serial_reg.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/fcntl.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/cdev.h> 
#include <linux/kfifo.h>


#define RFBB_DRIVER_VERSION "0.05"
#define RFBB_DRIVER_NAME "rfbb"

#define MAX_RFBB_DEVS 1 /* One device only */

#define HW_MODE_POWER_DOWN 0 /* Transceiver in power down mode */
#define HW_MODE_RX 1
#define HW_MODE_TX 2

/* Borrowed LIRC definitions */
#define LIRC_MODE2_SPACE     0x00000000
#define LIRC_MODE2_PULSE     0x01000000
#define LIRC_MODE2_TIMEOUT   0x03000000

#define LIRC_VALUE_MASK      0x00FFFFFF
#define LIRC_MODE2_MASK      0xFF000000

typedef int32_t lirc_t;

/*
 * We export one rfbb device.
 */
static struct cdev rfbbDevs[MAX_RFBB_DEVS];

struct rfbb {
	int tx_pin;
	int rx_pin;
	int tx_ctrl_pin;
	int rf_enable_pin;
	void (*send_pulse)(unsigned long length);
	void (*send_space)(unsigned long length);
};

#define RFBB_GPIO	 0 /* RFBB_TYPE */
#define RFBB_NO_GPIO_PIN 	-1
#define RFBB_NO_RX_IRQ		-1

static int type = RFBB_GPIO;
static int rfbb_major = 0; /* use dynamic major number assignment */

static int share_irq = 0;
static int interrupt_enabled = 0;
static int debug = 1;
static int device_open = 0;
static int hw_mode = HW_MODE_POWER_DOWN;

static DEFINE_MUTEX(read_lock);

#define dprintk(fmt, args...)					\
	do {							\
		if (debug)					\
			printk(KERN_DEBUG RFBB_DRIVER_NAME ": "	\
			       fmt, ## args);			\
	} while (0)

/* forward declarations */
static void set_tx_mode(void); /* set up transceiver for transmission */
static void set_rx_mode(void); /* set up transceiver for reception */
static void on(void); /* TX signal on */
static void off(void); /* TX signal off */
static void send_pulse_gpio(unsigned long length);
static void send_space_gpio(unsigned long length);
static void rfbb_exit_module(void);

static struct rfbb hardware[] = {
	[RFBB_GPIO] = {
		.tx_pin = 17,
		.rx_pin = RFBB_NO_GPIO_PIN, /* only tx now */
		.tx_ctrl_pin = RFBB_NO_GPIO_PIN, /* not used */
		.rf_enable_pin = RFBB_NO_GPIO_PIN, /* not used */
		.send_pulse = send_pulse_gpio,
		.send_space = send_space_gpio,
	},
};

#define RS_ISR_PASS_LIMIT 256

/*
 * A long pulse code from a remote might take up to 300 bytes.  The
 * daemon should read the bytes as soon as they are generated, so take
 * the number of keys you think you can push before the daemon runs
 * and multiply by 300.  The driver will warn you if you overrun this
 * buffer.  If you have a slow computer or non-busmastering IDE disks,
 * maybe you will need to increase this.
 */


#define RBUF_LEN 4096
#define WBUF_LEN 4096

static int sense = 0;  /* -1 = auto, 0 = active high, 1 = active low */

static int irq = RFBB_NO_RX_IRQ;

static struct timeval lasttv = {0, 0};

/* static struct lirc_buffer rbuf; */

/* Use FIFO to store received pulses */ 
static DEFINE_KFIFO(rxfifo, int, RBUF_LEN);

static int wbuf[WBUF_LEN];

/* AUREL RTX-MID transceiver TX setup sequence
   will use rf_enable as well as tx_ctrl pins.
   Not used for simple TX modules */ 
static void set_tx_mode(void)
{
	off();
	switch(hw_mode)
	{
		case HW_MODE_POWER_DOWN:
			if(hardware[type].rf_enable_pin != RFBB_NO_GPIO_PIN)
                        {
				gpio_set_value(hardware[type].rf_enable_pin, 1);
				udelay(20);
			}
                        if(hardware[type].tx_ctrl_pin != RFBB_NO_GPIO_PIN)
                        {
				gpio_set_value(hardware[type].tx_ctrl_pin, 1);
				udelay(400); /* let it settle */
			}
		break;
		
		case HW_MODE_TX:
			/* do nothing */
		break;

		case HW_MODE_RX:
			if(hardware[type].rf_enable_pin != RFBB_NO_GPIO_PIN)
			{
				gpio_set_value(hardware[type].rf_enable_pin, 1);
			}
			if(hardware[type].tx_ctrl_pin != RFBB_NO_GPIO_PIN)
			{
				gpio_set_value(hardware[type].tx_ctrl_pin, 1);
				udelay(400); /* let it settle */
			}	    
		break;
		
		default:
			printk(KERN_ERR RFBB_DRIVER_NAME": set_tx_mode. Illegal HW mode %d\n", hw_mode);
		break;		
	} 

	hw_mode = HW_MODE_TX;
}

/* AUREL RTX-MID transceiver RX setup sequence */       
static void set_rx_mode(void)
{
	off();
	switch(hw_mode)
	{
		case HW_MODE_POWER_DOWN:
			/* Note this sequence is only needed for AUREL RTX-MID */
			if((hardware[type].rf_enable_pin != RFBB_NO_GPIO_PIN) && (hardware[type].tx_ctrl_pin != RFBB_NO_GPIO_PIN))
			{
				gpio_set_value(hardware[type].rf_enable_pin, 1);
				gpio_set_value(hardware[type].tx_ctrl_pin, 0);
				udelay(20);
				gpio_set_value(hardware[type].tx_ctrl_pin, 1);
				udelay(200);
				gpio_set_value(hardware[type].tx_ctrl_pin, 0);
				udelay(40);
				gpio_set_value(hardware[type].rf_enable_pin, 0);
				udelay(20);
				gpio_set_value(hardware[type].rf_enable_pin, 1);
				udelay(200);
			}
		break;
		
		case HW_MODE_RX:
			/* do nothing */
		break;

		case HW_MODE_TX:
			if(hardware[type].tx_ctrl_pin != RFBB_NO_GPIO_PIN)
			{
				gpio_set_value(hardware[type].tx_ctrl_pin, 0);
				udelay(40);
			}
			if(hardware[type].rf_enable_pin != RFBB_NO_GPIO_PIN)
			{			
				gpio_set_value(hardware[type].rf_enable_pin, 0);
				udelay(20);
				gpio_set_value(hardware[type].rf_enable_pin, 1);
				udelay(200);
			}
		break;
		
		default:
			printk(KERN_ERR RFBB_DRIVER_NAME
		       ": set_rx_mode. Illegal HW mode %d\n", hw_mode);
		break;		
	} 

	hw_mode = HW_MODE_RX;
}

static void on(void)
{
	gpio_set_value(hardware[type].tx_pin, 1);
}

static void off(void)
{
	gpio_set_value(hardware[type].tx_pin, 0); 
}

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static void send_pulse_gpio(unsigned long length)
{   
	on();
	/* dprintk("send_pulse_gpio %ld us\n", length); */ 
	safe_udelay(length);
}

static void send_space_gpio(unsigned long length)
{
	off();
	/* dprintk("send_space_gpio %ld us\n", length); */
	safe_udelay(length);
}

static irqreturn_t irq_handler(int i, void *blah)
{
	struct timeval tv;
	int status;
	long deltv;
	lirc_t data = 0;
	static int old_status = -1;
	static int counter = 0; /* to find burst problems */ 
	/* static int intCount = 0; */
	

	status = gpio_get_value(hardware[type].rx_pin);
	if (status != old_status) {
		counter = 0;
		
		/* get current time */
		do_gettimeofday(&tv);

		/* New mode, written by Trent Piepho
		   <xyzzy@u.washington.edu>. */

		/*
		 * The old format was not very portable.
		 * We now use an int to pass pulses
		 * and spaces to user space.
		 *
		 * If PULSE_BIT is set a pulse has been
		 * received, otherwise a space has been
		 * received.  The driver needs to know if your
		 * receiver is active high or active low, or
		 * the space/pulse sense could be
		 * inverted. The bits denoted by PULSE_MASK are
		 * the length in microseconds. Lengths greater
		 * than or equal to 16 seconds are clamped to
		 * PULSE_MASK.  All other bits are unused.
		 * This is a much simpler interface for user
		 * programs, as well as eliminating "out of
		 * phase" errors with space/pulse
		 * autodetection.
		 */

		/* calc time since last interrupt in microseconds */
		deltv = tv.tv_sec-lasttv.tv_sec;
		if (tv.tv_sec < lasttv.tv_sec ||
		    (tv.tv_sec == lasttv.tv_sec &&
		     tv.tv_usec < lasttv.tv_usec)) {
			printk(KERN_WARNING RFBB_DRIVER_NAME
			       ": AIEEEE: your clock just jumped "
			       "backwards\n");
			printk(KERN_WARNING RFBB_DRIVER_NAME
			       ": %d %lx %lx %lx %lx\n",
			       sense,
			       tv.tv_sec, lasttv.tv_sec,
			       tv.tv_usec, lasttv.tv_usec);
			data = status ? (data | LIRC_VALUE_MASK) : (data | LIRC_MODE2_PULSE | LIRC_VALUE_MASK); /* handle as too long time */
		} else if (deltv > 15) {
			data = status ? (data | LIRC_VALUE_MASK) : (data | LIRC_MODE2_PULSE | LIRC_VALUE_MASK);  /* really long time */
		} else
			data = (lirc_t) (deltv*1000000 +
				       tv.tv_usec -
				       lasttv.tv_usec);
		/* frbwrite(status ? data : (data|PULSE_BIT)); */
		lasttv = tv;
		old_status = status;
		data = status ? data : (data | LIRC_MODE2_PULSE);
		/* dprintk("irq_handler. Nr: %d. Pin: %d time: %ld\n", ++intCount, status, (long)(data & PULSE_MASK));*/
		kfifo_put(&rxfifo, &data);
		/* wake_up_interruptible(&rbuf.wait_poll); */
	}
	else /* could have been a spike */
	{
		counter++;
		if (counter > RS_ISR_PASS_LIMIT) {
			printk(KERN_WARNING RFBB_DRIVER_NAME ": AIEEEE: "
			       "We're caught!\n");
			counter = 0; /* to avoid flooding warnings */
		}
	}
	return IRQ_RETVAL(IRQ_HANDLED);
}

static int hardware_init(void)
{
	unsigned long flags;
	int err = 0;
	
	local_irq_save(flags);

	/* First of all, disable all interrupts */

	if (type == RFBB_GPIO) {
		/* Setup all pins */
		err = gpio_request_one(hardware[type].tx_pin, GPIOF_OUT_INIT_LOW, "RFBB_TX");
		if (err) {
			printk(KERN_ERR  RFBB_DRIVER_NAME
			       "Could not request RFBB TX pin, error: %d\n", err);
			return -EIO;
		}                            
		
		if(hardware[type].rx_pin != RFBB_NO_GPIO_PIN)
		{
			err = gpio_request_one(hardware[type].rx_pin, GPIOF_IN, "RFBB_RX");
			if (err) {
				printk(KERN_ERR  RFBB_DRIVER_NAME
				"Could not request RFBB RX pin, error: %d\n", err);
				return -EIO;
			}
		}

		if(hardware[type].tx_ctrl_pin != RFBB_NO_GPIO_PIN)
		{
			err = gpio_request_one(hardware[type].tx_ctrl_pin, GPIOF_OUT_INIT_LOW, "RFBB_TX_CTRL");
			if (err) {
				printk(KERN_ERR  RFBB_DRIVER_NAME
				"Could not request RFBB TX CTRL pin, error: %d\n", err);
				return -EIO;
			}
		}
		
		if(hardware[type].rf_enable_pin != RFBB_NO_GPIO_PIN)
		{
			err = gpio_request_one(hardware[type].rf_enable_pin, GPIOF_OUT_INIT_LOW, "RFBB_RF_ENABLE");
			if (err) {
				printk(KERN_ERR  RFBB_DRIVER_NAME
				"Could not request RFBB RF ENABLE pin, error: %d\n", err);
				return -EIO;
			}
		}
		
		/* start in TX mode, avoid interrupts */ 
		set_tx_mode();  	

		/* Export pins and make them able to change from sysfs for troubleshooting */
		gpio_export(hardware[type].tx_pin, 1);
		if(hardware[type].rf_enable_pin != RFBB_NO_GPIO_PIN)
		{
			gpio_export(hardware[type].rf_enable_pin, 1);
		}
		if(hardware[type].tx_ctrl_pin != RFBB_NO_GPIO_PIN)
		{
			gpio_export(hardware[type].tx_ctrl_pin, 1);
		}
		if(hardware[type].rx_pin != RFBB_NO_GPIO_PIN)
		{
			gpio_export(hardware[type].rx_pin, 0);
		
			/* Get interrupt for RX */
			irq = gpio_to_irq(hardware[type].rx_pin);
			dprintk("Interrupt %d for RX pin\n", irq);
		}
	}
	
	local_irq_restore(flags);
	return 0;
}

static int init_port(void)
{
	int err = 0;

	err = hardware_init();
    if (err)
		return err;
	else 
	    return 0;
}


static ssize_t rfbb_read(struct file *filp, char *buf, size_t length, loff_t *offset )
{
	ssize_t ret = 0;
	ssize_t copied = 0;

	set_rx_mode();
	if(!interrupt_enabled)
	{
		//enable_irq(irq);
		interrupt_enabled = 1;
	}

	/* might need mutex */
	ret = kfifo_to_user(&rxfifo, buf, length, &copied);
	
	dprintk("rfbb_read request %d bytes, result %d, copied bytes %d\n", length, ret, copied);
	
	return ret ? ret : copied;
}


static ssize_t rfbb_write(struct file *file, const char *buf,
			 size_t n, loff_t *ppos)
{
	int i, count;
	unsigned long flags;
	int result = 0;
                  
	if(interrupt_enabled)
	{
		//disable_irq(irq);
		interrupt_enabled = 0;
	}
	set_tx_mode();

	dprintk("rfbb_write %d bytes\n", n);    

	if (n % sizeof(lirc_t))
		return -EINVAL;
	count = n / sizeof(lirc_t);
	if (count > WBUF_LEN)
	{
		dprintk("Too many elements (%d) in TX buffer\n", count);
		return -EINVAL;
	}
	
	result = copy_from_user(wbuf, buf, n);	
	if (result)
	{
		dprintk("Copy_from_user returns %d\n", result);    
		return -EFAULT;
	}
	local_irq_save(flags);
	for (i = 0; i < count; i++) {
		if (wbuf[i] & LIRC_MODE2_PULSE)
			hardware[type].send_pulse(wbuf[i] & LIRC_VALUE_MASK);
		else
			hardware[type].send_space(wbuf[i] & LIRC_VALUE_MASK);
	}
	off();
	local_irq_restore(flags);
	return n;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
static int rfbb_ioctl(struct inode *node, struct file *filep, unsigned int cmd,
		      unsigned long arg)
#else
static long rfbb_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
#endif
{
	switch (cmd) {

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static int rfbb_open(struct inode *ino, struct file *filep)
{
	int result;
	unsigned long flags;

	if(device_open)
	{
		printk(KERN_ERR RFBB_DRIVER_NAME ": Already opened\n"); 
		return -EBUSY;
	}


	/* Init read buffer. */
	/* if (lirc_buffer_init(&rbuf, sizeof(lirc_t), RBUF_LEN) < 0)
		return -ENOMEM; */

	/* initialize timestamp */
	do_gettimeofday(&lasttv);

	if(irq != RFBB_NO_RX_IRQ) {
		local_irq_save(flags); 
		result = request_irq(irq, irq_handler,
			     IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			     RFBB_DRIVER_NAME, (void *)&hardware);

		switch (result) {
		case -EBUSY:
			printk(KERN_ERR RFBB_DRIVER_NAME ": IRQ %d busy\n", irq);
			/* lirc_buffer_free(&rbuf); */
			return -EBUSY;
		case -EINVAL:
			printk(KERN_ERR RFBB_DRIVER_NAME ": Bad irq number or handler\n");
			/* lirc_buffer_free(&rbuf); */
			return -EINVAL;
		default:
			dprintk("Interrupt %d obtained\n", irq);
			break;
		};   

		local_irq_restore(flags);
	}
	
	if(interrupt_enabled)
	{
		//disable_irq(irq);
		interrupt_enabled = 0;
	}

	try_module_get(THIS_MODULE);
	device_open++;      
	return 0;
}

static int rfbb_release(struct inode *node, struct file *file)
{	
	off();

	if(interrupt_enabled)
	{
		//disable_irq(irq);
		interrupt_enabled = 0;
	}
	/* remove the RX interrupt */
	if(irq != RFBB_NO_RX_IRQ) {
		free_irq(irq, (void *)&hardware);
		dprintk("Freed RX IRQ %d\n", irq);
	}

	/* lirc_buffer_free(&rbuf); */ 
  
	device_open--;          /* We're now ready for our next caller */ 
	module_put(THIS_MODULE);
	return 0;
}

static struct file_operations rfbb_fops = {
	.owner = THIS_MODULE,
	.open = rfbb_open,
	.release = rfbb_release,
	.write	= rfbb_write,
	.read = rfbb_read,
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	.ioctl	= rfbb_ioctl,
#else
	.unlocked_ioctl	= rfbb_ioctl,
#endif
};

/*
 * Set up the cdev structure for a device.
 */
static void rfbb_setup_cdev(struct cdev *dev, int minor,
			      struct file_operations *fops)
{
	int err, devno = MKDEV(rfbb_major, minor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add(dev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding rfbb %d", err, minor);
}

static int rfbb_init(void)
{

	int result;
	dev_t dev = 0;

	/*
	 * Dynamic major if not set otherwise.
	 */
      if (rfbb_major) {
              dev = MKDEV(rfbb_major, 0);
              result = register_chrdev_region(dev, 1, "rfbb");
      } else {
              result = alloc_chrdev_region(&dev, 0, 1,
                              "rfbb");
              rfbb_major = MAJOR(dev);
      }
      if (result < 0) {
              printk(KERN_WARNING "rfbb: can't get major %d\n", rfbb_major);
              return result;
      }   

	rfbb_setup_cdev(rfbbDevs, 0, &rfbb_fops);

	return 0;
}

static int rfbb_init_module(void)
{
	int result;

	result = rfbb_init();
	if (result)
		goto exit_rfbb_exit; 

	result = init_port();
	if (result < 0)
		goto exit_rfbb_exit;

	printk(KERN_INFO
	       RFBB_DRIVER_NAME " " RFBB_DRIVER_VERSION " registered\n");
	dprintk("dev major = %d\n", rfbb_major);
	dprintk("IRQ = %d\n", irq);
	dprintk("share_irq = %d\n", share_irq);

	return 0;

exit_rfbb_exit:
	rfbb_exit_module();
	return result;
}

static void rfbb_exit_module(void)
{
	cdev_del(rfbbDevs);
	unregister_chrdev_region(MKDEV(rfbb_major, 0), 1);
	dprintk("cleaned up module\n");
}


module_init(rfbb_init_module);
module_exit(rfbb_exit_module);

MODULE_DESCRIPTION("RF transmitter and receiver driver for embedded CPU:s with GPIO. Based on lirc_serial");
MODULE_AUTHOR("Tord Andersson");
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
