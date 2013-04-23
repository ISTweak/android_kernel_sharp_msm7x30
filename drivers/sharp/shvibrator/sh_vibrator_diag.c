/* drivers/sharp/shvibrator/sh_vibrator_diag.c
 *
 * Copyright (C) 2010 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <mach/pmic.h>

//#define SHVIB_LOG_DEBUG_ENABLE

#if defined( SHVIB_LOG_DEBUG_ENABLE )
#define SHVIB_LOG_DEBUG(p)	p
#else
#define SHVIB_LOG_DEBUG(p)
#endif /* defined( SHVIB_LOG_DEBUG_ENABLE ) */

#define SHVIB_DRIVER_NAME		"shvibrator"

#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)
#define PMIC_VIBRATOR_ON_LEV	(3100)
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER */
#define PMIC_VIBRATOR_ON_LEV	(3000)
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER */
#define PMIC_VIBRATOR_OFF_LEV	(   0)

static dev_t 				shvib_devid;
static struct class*		shvib_class;
static struct device*		shvib_device;
struct cdev 				shvib_cdev;


static void set_pmic_vibrator(int on)
{
	if (on){
		pmic_vib_mot_set_polarity(PM_VIB_MOT_POL__ACTIVE_LOW);
		pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__DBUS2);
		pmic_vib_mot_set_volt(PMIC_VIBRATOR_ON_LEV);
	}else{
		pmic_vib_mot_set_volt(0);
	}
}

static int shvib_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t shvib_write(struct file *filp, 
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char cmd;

	SHVIB_LOG_DEBUG( printk(KERN_DEBUG "[shvib]shvib_write(%s) start\n", ubuf); );
	
	if(get_user(cmd, ubuf)){
		return -EFAULT;
	}

	SHVIB_LOG_DEBUG( printk(KERN_DEBUG "[shvib]shvib_write() cmd = %d\n", cmd); );
	
	if(cmd == '0'){
		set_pmic_vibrator(0);
	}else{
		set_pmic_vibrator(1);
	}
	
	return cnt;
}

static int shvib_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations shvib_fileops = {
	.owner   = THIS_MODULE,
	.open    = shvib_open,
	.write   = shvib_write,
	.release = shvib_release,
};

static int __init init_pmic_shvibrator(void)
{	int rc;

	rc = alloc_chrdev_region(&shvib_devid, 0, 1, SHVIB_DRIVER_NAME);
	if(rc < 0){
		printk(KERN_WARNING "shvib:alloc_chrdev_region error\n");
		return rc;
	}

	shvib_class = class_create(THIS_MODULE, SHVIB_DRIVER_NAME);
	if (IS_ERR(shvib_class)) {
		rc = PTR_ERR(shvib_class);
		printk(KERN_WARNING "shvib:class_create error\n");
		goto error_vid_class_create;
	}

	shvib_device = device_create(shvib_class, NULL, 
								 shvib_devid, &shvib_cdev, 
								 SHVIB_DRIVER_NAME);
	if (IS_ERR(shvib_device)) {
		rc = PTR_ERR(shvib_device);
		printk(KERN_WARNING "shvib:device_create error\n");
		goto error_vid_class_device_create;
	}
	
	cdev_init(&shvib_cdev, &shvib_fileops);
	shvib_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shvib_cdev, shvib_devid, 1);
	if(rc < 0){
		printk(KERN_WARNING "shvib:cdev_add error\n");
		goto err_via_cdev_add;
	}

	SHVIB_LOG_DEBUG( printk(KERN_DEBUG "[shvib]shvib_init() done\n"); );
	
	return 0;
	
err_via_cdev_add:
	cdev_del(&shvib_cdev);
error_vid_class_device_create:
	class_destroy(shvib_class);
error_vid_class_create:
	unregister_chrdev_region(shvib_devid, 1);

	return rc;
}
module_init( init_pmic_shvibrator );

static void __exit exit_pmic_shvibrator(void)
{
	cdev_del(&shvib_cdev);
	class_destroy(shvib_class);
	unregister_chrdev_region(shvib_devid, 1);
}
module_exit( exit_pmic_shvibrator );

