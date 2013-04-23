/* ami_flip.c - AMI603 compass position driver
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "sharp/ami_flip.h"
#include "sharp/shmds_driver.h" /* shmds add */

#ifdef DEBUG_PRINT
#define PRINTK(fmt, args...) printk( "[Motion Debug]" KERN_INFO fmt, ## args) /* shmds mod */
#else
#define PRINTK(fmt, args...)
#endif
#define PRINTK_E(fmt, args...) printk( "[Motion Error]" fmt, ## args) /* shmds add */

static struct semaphore g_mutex;
static int g_position = 0;

static unsigned long ami_copy_to_user(void __user * to, const void *from,
				      unsigned long n)
{
	unsigned long ret;
	PRINTK("%s\n", __func__);
	if (down_interruptible(&g_mutex)) {
		PRINTK_E("down_interruptible error.\n");
		return -ERESTARTSYS;
	}

	ret = copy_to_user(to, from, n);
	up(&g_mutex);

	return ret;
}

static unsigned long ami_copy_from_user(void *to, const void __user * from,
					unsigned long n)
{
	unsigned long ret;

	PRINTK("%s\n", __func__);
	if (down_interruptible(&g_mutex)) {
		PRINTK_E("down_interruptible error.\n");
		return -ERESTARTSYS;
	}

	ret = copy_from_user(to, from, n);
	up(&g_mutex);

	return ret;
}

/*************************************************************/
static int ami_flip_open(struct inode *inode, struct file *filp)
{
	PRINTK("%s\n", __func__);
	return 0;
}

static int ami_flip_release(struct inode *inode, struct file *filp)
{
	PRINTK("%s\n", __func__);
	return 0;
}

static int ami_flip_ioctl(struct inode *inode, struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	PRINTK("%s cmd : 0x%08x\n", __func__, cmd);

	switch (cmd) {
	case AMI_FLIP_IOCTL_READ_POSITION:
		if (ami_copy_to_user(argp, &g_position, sizeof(g_position))) {
			PRINTK_E("copy_to_user error.\n");
			return -EFAULT;
		}
		break;
	case AMI_FLIP_IOCTL_WRITE_POSITION:
		if (ami_copy_from_user(&g_position, argp, sizeof(g_position))) {
			PRINTK_E("copy_to_user error.\n");
			return -EFAULT;
		}
		break;
	default:
		PRINTK_E("%s Unsupported cmd 0x%08x\n", __func__, cmd);
		PRINTK_E("AMI_FLIP_IOCTL_READ_POSITION :  0x%08x\n",
		       AMI_FLIP_IOCTL_READ_POSITION);
		PRINTK_E("AMI_FLIP_IOCTL_WRITE_POSITION :  0x%08x\n",
		       AMI_FLIP_IOCTL_WRITE_POSITION);
		return -ENOTTY;
	}
	return 0;
}

/* shmds add -> */
void SHMDS_SetFlipInformation(unsigned char position)
{
	PRINTK("SHMDS_SetFlipInformation,%d\n", position);

	if(position == MS_POSITION_OPEN) {
		g_position = 0;
	}
	else {
		g_position = 1;
	}

}
EXPORT_SYMBOL(SHMDS_SetFlipInformation);
/* shmds add <- */

/*************************************************************/
static struct file_operations ami_flip_fops = {
	.owner = THIS_MODULE,
	.ioctl = ami_flip_ioctl,
	.open = ami_flip_open,
	.release = ami_flip_release,
};

static struct miscdevice ami_flip_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AMI_FLIP_DEVFILE,
	.fops = &ami_flip_fops,
};

/*************************************************************/
static int __init ami_flip_init(void)
{
	int ret;

	ret = misc_register(&ami_flip_dev);
	if (ret) {
		PRINTK_E("fail to misc_register (MISC_DYNAMIC_MINOR)\n");
		return ret;
	}

	init_MUTEX(&g_mutex);
	g_position = 0;

	PRINTK("%s\n", __func__);
	return 0;
}

static void __exit ami_flip_exit(void)
{
	misc_deregister(&ami_flip_dev);

	PRINTK("%s\n", __func__);
}

MODULE_DESCRIPTION("ami flip");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
module_init(ami_flip_init);
module_exit(ami_flip_exit);
