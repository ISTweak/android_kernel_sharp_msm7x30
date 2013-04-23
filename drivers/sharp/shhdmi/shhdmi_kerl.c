/* drivers/sharp/shhdmi/shhdmi_kerl.c (SHARP HDMI Driver)
 *
 * Copyright (c) 2010, Sharp. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

// ____________________________________________________________________________________________________________________________
/**
 *	Include
 */
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <linux/wakelock.h>

#include <linux/input.h>

#include <sharp/shhdmi_kerl.h>

#include "../../video/msm/msm_fb.h"

// ____________________________________________________________________________________________________________________________
/**
 *	Internal Define
 */
#define SH_HDMI_GPIO_INT		18
#define SH_HDMI_GPIO_HDMIRST_N	148
#define SH_HDMI_GPIO_HDMIPOW	149
#define SH_HDMI_GPIO_HDMIPD_N	150

#define SH_HDMI_GPIO_LO			0
#define SH_HDMI_GPIO_HI			1

#define SH_HDMI_IRQ_DISABLE		0
#define SH_HDMI_IRQ_ENABLE		1

#define SH_HDMI_MINORNUM_BASE	0
#define SH_HDMI_DEVICE_COUNT	1
#define SH_HDMI_DEVICE_NAME		"shhdmi"
#define SH_HDMI_CLASS_NAME		"shhdmi"
#define SH_HDMI_KEY_DEVICE_NAME	"SH_hdmi_key"

// ____________________________________________________________________________________________________________________________
/**
 *	Internal variable
 */
static int					shhdmi_major = 0;
static struct cdev			shhdmi_cdev;
static struct class			*shhdmi_kdrv_class = NULL;
static dev_t				shhdmi_kdrv_dev;

static int					shhdmi_irq_port = 0;
static wait_queue_head_t	irq_wait_queue;
static char					shhdmi_irq_state = 0;

static char					shhdmi_intr_enable = SH_HDMI_IRQ_ENABLE;	/* enable */
static struct wake_lock		shhdmi_wake_lock;

static struct msm_panel_info	*pinfo;

// ____________________________________________________________________________________________________________________________
/**
 *	Internal prototypes
 */
static irqreturn_t shhdmi_int_isr(int irq, void *ptr);
static int shhdmi_open(struct inode *inode, struct file *filp);
static int shhdmi_close(struct inode *inode, struct file *filp);
static int shhdmi_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static void shhdmi_enable_intr(void);
static void shhdmi_disable_intr(void);

static void shhdmi_dtvif_gpioconfig_set(void);
static void shhdmi_dtvif_gpioconfig_reset(void);

static int shhdmi_msmfb_dummy_power_on(struct platform_device *pdev);
static int shhdmi_msmfb_dummy_power_off(struct platform_device *pdev);

static struct msm_fb_panel_data shhdmi_panel_data = {
	.on  = shhdmi_msmfb_dummy_power_on,
	.off = shhdmi_msmfb_dummy_power_off,
};

static struct platform_device shhdmi_device = {
	.name = SH_HDMI_DEVICE_NAME ,
	.id   = 2,
	.dev  = {
		.platform_data = &shhdmi_panel_data,
		}
};

static struct file_operations shhdmi_fops = {
    .owner   = THIS_MODULE,
    .open    = shhdmi_open,
    .release = shhdmi_close,
    .ioctl   = shhdmi_ioctl,
};

static struct input_dev* shhdmi_key;

// ____________________________________________________________________________________________________________________________
/**
 *	Main Functions
 */
static irqreturn_t shhdmi_int_isr(int irq, void *ptr)
{
	irqreturn_t ret = IRQ_HANDLED;
	
	shhdmi_disable_intr();
	shhdmi_irq_state = 1;
	
	wake_up_interruptible(&irq_wait_queue);

	return ret;
}

static int shhdmi_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int shhdmi_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static int shhdmi_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg)
{
	int ret = -EFAULT;
	void __user *argp = (void __user*)arg;

	switch (cmd)
	{
		case SH_HDMI_IOCTL_START_IRQ:
			shhdmi_irq_port = MSM_GPIO_TO_INT( SH_HDMI_GPIO_INT );
			init_waitqueue_head(&irq_wait_queue);
			ret = request_irq(shhdmi_irq_port, shhdmi_int_isr, IRQF_TRIGGER_LOW, "shhdmi_irq", 0);
			shhdmi_disable_intr();
			shhdmi_irq_state = 0;
			break;
		
		case SH_HDMI_IOCTL_EXIT_IRQ:
			free_irq(shhdmi_irq_port, 0);
			shhdmi_irq_state = 1;
			wake_up_interruptible(&irq_wait_queue);
			ret = 0;
			break;
		
		case SH_HDMI_IOCTL_READ_IRQ:
			shhdmi_enable_intr();
			wait_event_interruptible(irq_wait_queue, (shhdmi_irq_state != 0));
			shhdmi_irq_state = 0;
			ret = 0;
			break;
		
		case SH_HDMI_IOCTL_PORT_HDMIRST_N_LO:
			gpio_direction_output(SH_HDMI_GPIO_HDMIRST_N, SH_HDMI_GPIO_LO);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_PORT_HDMIRST_N_HI:
			gpio_direction_output(SH_HDMI_GPIO_HDMIRST_N, SH_HDMI_GPIO_HI);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_PORT_HDMIPOW_LO:
			gpio_direction_output(SH_HDMI_GPIO_HDMIPOW, SH_HDMI_GPIO_LO);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_PORT_HDMIPOW_HI:
			gpio_direction_output(SH_HDMI_GPIO_HDMIPOW, SH_HDMI_GPIO_HI);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_PORT_HDMIPD_N_LO:
			gpio_direction_output(SH_HDMI_GPIO_HDMIPD_N, SH_HDMI_GPIO_LO);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_PORT_HDMIPD_N_HI:
			gpio_direction_output(SH_HDMI_GPIO_HDMIPD_N, SH_HDMI_GPIO_HI);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_WAKE_LOCK:
			wake_lock(&shhdmi_wake_lock);
			ret = 0;
			break;
		
		case SH_HDMI_IOCTL_WAKE_UNLOCK:
			wake_unlock(&shhdmi_wake_lock);
			ret = 0;
			break;

		case SH_HDMI_IOCTL_READ_HDMIINT_N:
			{
				int value = 0;
				value = gpio_get_value(SH_HDMI_GPIO_INT);
				ret = value;
			}
			break;
		
		case SH_HDMI_IOCTL_DTVIF_GPIOCONFIG_SET:
			shhdmi_dtvif_gpioconfig_set();
			break;
		
		case SH_HDMI_IOCTL_DTVIF_GPIOCONFIG_RESET:
			shhdmi_dtvif_gpioconfig_reset();
			break;

		case SH_HDMI_IOCTL_REPORT_INPUT_KEY_PRESS:
		{
			unsigned short code; 
	        ret = copy_from_user(&code, argp, sizeof(unsigned short));
			if (ret != 0) {
				return ret;
			}
			//printk(KERN_INFO "%s line:%d PRESS: %d\n", __func__, __LINE__,code);
			input_report_key(shhdmi_key, (unsigned short)code, 1);
			break;
		}
		case SH_HDMI_IOCTL_REPORT_INPUT_KEY_RELEASE:
		{
			unsigned short code; 
	        ret = copy_from_user(&code, argp, sizeof(unsigned short));
			if (ret != 0) {
				return ret;
			}
			//printk(KERN_INFO "%s line:%d RELEASE: %d\n", __func__, __LINE__,code);
			input_report_key(shhdmi_key, (unsigned short)code, 0);
			break;
		}
	
		default:
			ret = -EFAULT;
			break;
	}

	return ret;
}


static void shhdmi_enable_intr(void)
{
	if(shhdmi_intr_enable == SH_HDMI_IRQ_DISABLE){
		shhdmi_intr_enable = SH_HDMI_IRQ_ENABLE;
		enable_irq(shhdmi_irq_port);
		enable_irq_wake(shhdmi_irq_port);
	//	printk("%s line:%d enableIrq %d\n", __func__, __LINE__, shhdmi_irq_state);
	}else{
	//	printk("%s line:%d enableIrq Noset %d\n", __func__, __LINE__, shhdmi_irq_state);
	}
}

static void shhdmi_disable_intr(void)
{
	if(shhdmi_intr_enable == SH_HDMI_IRQ_ENABLE){
		shhdmi_intr_enable = SH_HDMI_IRQ_DISABLE;
		//disable_irq(shhdmi_irq_port);
		disable_irq_wake(shhdmi_irq_port);
		disable_irq_nosync(shhdmi_irq_port);
	//	printk("%s line:%d disableIrq %d\n", __func__, __LINE__, shhdmi_irq_state);
	}else{
	//	printk("%s line:%d disableIrq Noset %d\n", __func__, __LINE__, shhdmi_irq_state);
	}
}


static void shhdmi_dtvif_gpioconfig_set(void)
{
	gpio_tlmm_config(GPIO_CFG(124, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(126, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(127, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(128, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(129, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(130, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(131, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(132, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(160, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(161, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(162, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(163, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(164, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(165, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(166, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(167, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(168, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(169, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(170, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
}

static void shhdmi_dtvif_gpioconfig_reset(void)
{
	gpio_tlmm_config(GPIO_CFG(124, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(126, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(127, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(128, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(130, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(131, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(132, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(160, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(161, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(162, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(163, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(164, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(165, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(166, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(167, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(168, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(169, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(170, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
}

static int shhdmi_msmfb_dummy_power_on(struct platform_device *pdev)
{
	return 0;
}

static int shhdmi_msmfb_dummy_power_off(struct platform_device *pdev)
{
	return 0;
}

static int __init shhdmi_init(void)
{
	int		ret = 0;
	dev_t	dev;
	pinfo = &shhdmi_panel_data.panel_info;

	dev = MKDEV(shhdmi_major, 0);

	ret = alloc_chrdev_region(&dev, SH_HDMI_MINORNUM_BASE, SH_HDMI_DEVICE_COUNT, SH_HDMI_DEVICE_NAME);
	
	if (ret < 0) {
		printk(KERN_INFO "%s line:%d \n", __func__, __LINE__);
	}

	shhdmi_major = MAJOR(dev);

	cdev_init(&shhdmi_cdev, &shhdmi_fops);
	shhdmi_cdev.owner = THIS_MODULE;
	shhdmi_cdev.ops = &shhdmi_fops;
	
	ret = cdev_add(&shhdmi_cdev, dev, SH_HDMI_DEVICE_COUNT);

	if (ret < 0) {
		printk(KERN_INFO "%s line:%d \n", __func__, __LINE__);
	}

	shhdmi_kdrv_class = class_create( THIS_MODULE, SH_HDMI_CLASS_NAME );
	if (IS_ERR(shhdmi_kdrv_class)) {
		printk(KERN_INFO "%s line:%d \n", __func__, __LINE__);
		return 0;
	}

	shhdmi_kdrv_dev = dev;
	device_create(shhdmi_kdrv_class, NULL, shhdmi_kdrv_dev, NULL, SH_HDMI_DEVICE_NAME);

	wake_lock_init(&shhdmi_wake_lock, WAKE_LOCK_SUSPEND, "shhdmi_wake_lock");

//	printk("[shhdmi]add device fb1 start\n");

	pinfo->xres = 1280 ;
	pinfo->yres = 720 ;
	pinfo->type = DTV_PANEL;
	pinfo->pdest = DISPLAY_2;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	//pinfo->bpp = 32;
	//pinfo->fb_num = 1;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 37125000;
	pinfo->lcdc.h_back_porch = 8;
	pinfo->lcdc.h_front_porch = 8;
	pinfo->lcdc.h_pulse_width = 8;
	pinfo->lcdc.v_back_porch = 2;
	pinfo->lcdc.v_front_porch = 2;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;	/* blk */
	pinfo->lcdc.underflow_clr = 0xff;	/* blue */
	pinfo->lcdc.hsync_skew = 0;
	
	msm_fb_add_device(&shhdmi_device);
	
//	printk("[shhdmi]add device fb1 end\n");
	shhdmi_key = input_allocate_device();
	if (!shhdmi_key) {
		printk("[shhdmi]input_allocate_device error\n");
		return ret;
	}
	
	shhdmi_key->name = SH_HDMI_KEY_DEVICE_NAME;
	shhdmi_key->id.vendor	= 0x0001;
	shhdmi_key->id.product	= 1;
	shhdmi_key->id.version	= 1;

	input_set_capability(shhdmi_key, EV_KEY, KEY_UP);
	input_set_capability(shhdmi_key, EV_KEY, KEY_DOWN);
	input_set_capability(shhdmi_key, EV_KEY, KEY_LEFT);
	input_set_capability(shhdmi_key, EV_KEY, KEY_RIGHT);
	input_set_capability(shhdmi_key, EV_KEY, KEY_ENTER);
	input_set_capability(shhdmi_key, EV_KEY, KEY_BACK);
	input_set_capability(shhdmi_key, EV_KEY, KEY_HOME);
	input_set_capability(shhdmi_key, EV_KEY, KEY_MENU);
	input_set_capability(shhdmi_key, EV_KEY, KEY_0);
	input_set_capability(shhdmi_key, EV_KEY, KEY_1);
	input_set_capability(shhdmi_key, EV_KEY, KEY_2);
	input_set_capability(shhdmi_key, EV_KEY, KEY_3);
	input_set_capability(shhdmi_key, EV_KEY, KEY_4);
	input_set_capability(shhdmi_key, EV_KEY, KEY_5);
	input_set_capability(shhdmi_key, EV_KEY, KEY_6);
	input_set_capability(shhdmi_key, EV_KEY, KEY_7);
	input_set_capability(shhdmi_key, EV_KEY, KEY_8);
	input_set_capability(shhdmi_key, EV_KEY, KEY_9);
	input_set_capability(shhdmi_key, EV_KEY, KEY_NUMERIC_STAR);
	input_set_capability(shhdmi_key, EV_KEY, KEY_NUMERIC_POUND);
	input_set_capability(shhdmi_key, EV_KEY, 64/*KEY_NEXT*/);
	input_set_capability(shhdmi_key, EV_KEY, 65/*KEY_PLAYPAUSE*/);
	input_set_capability(shhdmi_key, EV_KEY, 66/*KEY_PREVIOUS*/);
	input_set_capability(shhdmi_key, EV_KEY, 67/*KEY_STOP*/);
	input_set_capability(shhdmi_key, EV_KEY, 87);
	input_set_capability(shhdmi_key, EV_KEY, 88);
	
	ret = input_register_device(shhdmi_key);
	if (ret) {
		printk("[shhdmi]input_register_device error\n");
		input_free_device(shhdmi_key);
		return 0;
	}
	
	return ret;
}

static void __exit shhdmi_exit(void)
{
	device_destroy(shhdmi_kdrv_class, shhdmi_kdrv_dev);

	class_destroy(shhdmi_kdrv_class);

	cdev_del(&shhdmi_cdev);

	unregister_chrdev_region(shhdmi_major, SH_HDMI_DEVICE_COUNT);
	
	wake_lock_destroy(&shhdmi_wake_lock);
	
	free_irq(shhdmi_irq_port, 0);
	
	input_unregister_device(shhdmi_key);
}

module_init( shhdmi_init );
module_exit( shhdmi_exit );

MODULE_DESCRIPTION("SHARP HDMI DEVICE MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
