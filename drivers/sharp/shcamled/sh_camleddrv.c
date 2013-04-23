/* drivers/sharp/shcamled/sh_camleddrv.c  (Camera Driver)
 *
 * Copyright (C) 2009-2011 SHARP CORPORATION
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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <mach/gpio.h>

#include "sh_camleddrv.h"
#include "sh_camleddrv_rpc.h"

#define SH_CAMLED_MINORNUM_BASE    (0)
#define SH_CAMLED_DEVICE_COUNT     (1)
#define SH_CAMLED_DEVICE_NAME      "shcamled"
#define SH_CAMLED_NODE_NAME        "shcamled"
#define SH_CAMLED_CLASS_NAME       "cls_shcamled"



static atomic_t keyled = ATOMIC_INIT(0);
static atomic_t camled = ATOMIC_INIT(0);

static struct class  *sh_camled_class = NULL;
static struct device *sh_camled_class_dev;
static struct cdev    sh_camled_cdev;
static dev_t          sh_camled_devno;
static int            sh_camled_node = 0;

static struct mutex   sh_camled_mutex_lock;
static struct mutex   sh_camled_gpio_mutex_lock;


static int  sh_camled_open(struct inode *inode, struct file *filep);
static int  sh_camled_release(struct inode *inode, struct file *filep);
static long sh_camled_ioctl(struct file *filep,
                            unsigned int cmd, unsigned long arg);
static int  sh_camled_access_gpio(void __user *argp);
static int  sh_camled_access_redled(void __user *argp);
static int  sh_camled_access_mobileled(void __user *argp);
static int  sh_camled_set_uinfo(void __user *argp);
static int  sh_camled_get_uinfo(void __user *argp);
static int  __init sh_camled_drv_start(void);
static void __exit sh_camled_drv_remove(void);



static int sh_camled_access_gpio(void __user *argp)
{
    int value;
    int ret = -1;
    unsigned long result;

    mutex_lock(&sh_camled_mutex_lock);

    result = copy_from_user((void*)&value, argp, sizeof(int));
    if(0 != result) {
        printk(KERN_ERR "[%s]:[%d] result=%ld \n",
                __func__, __LINE__, result);
        ret = -EFAULT;
    }
    else {
        ret = sh_camled_gpio_ctrl(value, SH_CAMLED_USE_CAMERA_DRV);
    }

    mutex_unlock(&sh_camled_mutex_lock);

    return ret;
}

static int sh_camled_access_redled(void __user *argp)
{
    int value;
    int ret = -1;
    unsigned long result;

    mutex_lock(&sh_camled_mutex_lock);

    result = copy_from_user((void*)&value, argp, sizeof(int));
    if(0 != result) {
        printk(KERN_ERR "[%s]:[%d] result=%ld \n",
                __func__, __LINE__, result);
        ret = -EFAULT;
    }
    else {
        ret = shcamled_red_led_control_rpc(value);
    }

    mutex_unlock(&sh_camled_mutex_lock);

    return ret;
}

static int sh_camled_access_mobileled(void __user *argp)
{
    int value;
    int ret = -1;
    unsigned long result;

    mutex_lock(&sh_camled_mutex_lock);

    result = copy_from_user((void*)&value, argp, sizeof(int));
    if(0 != result) {
        printk(KERN_ERR "[%s]:[%d] result=%ld \n",
                __func__, __LINE__, result);
        ret = -EFAULT;
    }
    else {
        ret = shcamled_mobile_led_control_rpc(value);
    }

    mutex_unlock(&sh_camled_mutex_lock);

    return ret;
}

static int sh_camled_set_uinfo(void __user *argp)
{
    int value;
    int ret = -1;
    unsigned long result;

    mutex_lock(&sh_camled_mutex_lock);

    result = copy_from_user((void*)&value, argp, sizeof(int));
    if(0 != result) {
        printk(KERN_ERR "[%s]:[%d] result=%ld \n",
                __func__, __LINE__, result);
        ret = -EFAULT;
    }
    else {
        ret = shcamled_set_led_user_info_rpc(value);
    }

    mutex_unlock(&sh_camled_mutex_lock);

    return ret;
}

static int sh_camled_get_uinfo(void __user *argp)
{
    int ret = -1;
    unsigned long result;

    mutex_lock(&sh_camled_mutex_lock);

    ret = shcamled_get_led_user_info_rpc();
    result = copy_to_user(argp, (void*)&ret, sizeof(int));
    if(0 != result) {
        printk(KERN_ERR "[%s]:[%d] result=%ld \n",
            __func__, __LINE__, result);
        ret = -EFAULT;
    }

    mutex_unlock(&sh_camled_mutex_lock);

    return ret;
}

static const struct file_operations sh_camled_fops = {
	.owner          = THIS_MODULE,
	.open           = sh_camled_open,
	.unlocked_ioctl = sh_camled_ioctl,
	.release        = sh_camled_release,
};


static int sh_camled_open(struct inode *inode, struct file *filep)
{
    mutex_lock(&sh_camled_mutex_lock);
    mutex_unlock(&sh_camled_mutex_lock);

    return 0;
}

static int sh_camled_release(struct inode *inode, struct file *filep)
{
    mutex_lock(&sh_camled_mutex_lock);
    mutex_unlock(&sh_camled_mutex_lock);

    return 0;
}

static long sh_camled_ioctl(struct file *filep,
                            unsigned int cmd, unsigned long arg)
{
    int ret = -1;
    void __user *argp = (void __user *)arg;

    switch(cmd) {
    case SH_CAMLED_IOCTL_GPIO_CTRL:
        ret = sh_camled_access_gpio(argp);
        break;
    case SH_CAMLED_IOCTL_RED_CTRL:
        ret = sh_camled_access_redled(argp);
        break;
    case SH_CAMLED_IOCTL_MOBILE_CTRL:
        ret = sh_camled_access_mobileled(argp);
        break;
    case SH_CAMLED_IOCTL_SET_UINFO_CTRL:
        ret = sh_camled_set_uinfo(argp);
        break;
    case SH_CAMLED_IOCTL_GET_UINFO_CTRL:
        ret = sh_camled_get_uinfo(argp);
        break;
    default:
        break;
    }

    return (long)ret;
}

static int __init sh_camled_drv_start(void)
{
    int ret = -ENODEV;
    int errfunc = 0;

    do {
        if(sh_camled_node >= SH_CAMLED_DEVICE_COUNT) {
            ret = -ENODEV;
            printk(KERN_ERR "[%s]:[%d] err \n", __func__, __LINE__);
            break;
        }

        if (NULL == sh_camled_class) {
            sh_camled_class = class_create(THIS_MODULE, SH_CAMLED_CLASS_NAME);
            if(IS_ERR(sh_camled_class)) {
                ret = PTR_ERR(sh_camled_class);
                printk(KERN_ERR "[%s]:[%d] ret=%d \n",
                        __func__, __LINE__, ret);
                break;
            }
            ret = alloc_chrdev_region(&sh_camled_devno,
                                        SH_CAMLED_MINORNUM_BASE,
                                        SH_CAMLED_DEVICE_COUNT,
                                        SH_CAMLED_DEVICE_NAME);
            if(ret < 0) {
                errfunc = 1;
                printk(KERN_ERR "[%s]:[%d] ret=%d \n",
                        __func__, __LINE__, ret);
                break;
            }
        }

        sh_camled_class_dev = device_create(sh_camled_class, NULL,
                                            MKDEV(MAJOR(sh_camled_devno),
                                                        SH_CAMLED_MINORNUM_BASE),
                                            NULL, SH_CAMLED_NODE_NAME);
        if(IS_ERR(sh_camled_class_dev)) {
            ret = PTR_ERR(sh_camled_class_dev);
            errfunc = 6;
            printk(KERN_ERR "[%s]:[%d] ret=%d \n",
                    __func__, __LINE__, ret);
            break;
        }

        cdev_init(&sh_camled_cdev, &sh_camled_fops);
        sh_camled_cdev.owner = THIS_MODULE;
        ret = cdev_add(&sh_camled_cdev,
                        MKDEV(MAJOR(sh_camled_devno),
                                    SH_CAMLED_MINORNUM_BASE),
                        SH_CAMLED_DEVICE_COUNT);
        if(ret < 0) {
            errfunc = 7;
            printk(KERN_ERR "[%s]:[%d] ret=%d \n",
                    __func__, __LINE__, ret);
            break;
        }

        mutex_init(&sh_camled_mutex_lock);
        mutex_init(&sh_camled_gpio_mutex_lock);

        sh_camled_node++;

    } while(0);

    switch(errfunc) {
        case 8:
            cdev_del(&sh_camled_cdev);
        case 7:
            device_destroy(sh_camled_class, sh_camled_devno);
        case 6:
        case 5:
        case 4:
        case 3:
        case 2:
            unregister_chrdev_region(sh_camled_devno, SH_CAMLED_DEVICE_COUNT);
        case 1:
            class_destroy(sh_camled_class);
            sh_camled_class = NULL;
        case 0:
        default:
            break;
    }

    return ret;
}


static void __exit sh_camled_drv_remove(void)
{
    do {
        if(0 == sh_camled_node) {
            printk(KERN_ERR "[%s]:[%d] err \n", __func__, __LINE__);
            break;
        }

        cdev_del(&sh_camled_cdev);

        device_destroy(sh_camled_class, sh_camled_devno);

        unregister_chrdev_region(sh_camled_devno, SH_CAMLED_DEVICE_COUNT);

        class_destroy(sh_camled_class);

        sh_camled_class = NULL;

        sh_camled_node = 0;

    } while(0);
}


int sh_camled_gpio_ctrl(int ctrl, int type)
{
    int result;
    int dev_keyled;
    int dev_camled;

    if((SH_CAMLED_USE_CAMERA_DRV != type)
    && (SH_CAMLED_USE_KEYLED_DRV != type)) {
        printk(KERN_ERR "[%s] out error type=%d\n", __func__, type);
        return -ENOEXEC;
    }

    mutex_lock(&sh_camled_gpio_mutex_lock);

    result     = SH_CAMLED_NO_ERROR;
    dev_keyled = atomic_read(&keyled);
    dev_camled = atomic_read(&camled);

    switch (ctrl) {
    case SH_CAMLED_GPIO102_HI:
        if((SH_CAMLED_USE_KEYLED_DRV == type)
        && (SH_CAMLED_GPIO102_HI == dev_keyled)) {
            break;
        }

        if((SH_CAMLED_USE_CAMERA_DRV == type)
        && (SH_CAMLED_GPIO102_HI == dev_camled)) {
            break;
        }

        if ((SH_CAMLED_GPIO102_LO == dev_camled)
        && (SH_CAMLED_GPIO102_LO == dev_keyled)) {
            result = gpio_direction_output(SH_CAMLED_GPIO_WLED_EN_M,
                                            SH_CAMLED_GPIO102_HI);
            if (SH_CAMLED_NO_ERROR != result) {
                result = SH_CAMLED_GPIO_HI_ERR;
                printk(KERN_ERR "[%s] error result=%d\n", __func__, result);
            }
        }

        if(SH_CAMLED_USE_KEYLED_DRV == type) {
            atomic_inc(&keyled);
        }
        else {
            atomic_inc(&camled);
        }
        dev_keyled = atomic_read(&keyled);
        dev_camled = atomic_read(&camled);
        break;

    case SH_CAMLED_GPIO102_LO:
        if(SH_CAMLED_USE_KEYLED_DRV == type) {
            if(SH_CAMLED_GPIO102_LO < dev_keyled) {
                atomic_dec(&keyled);
            }
            else {
                break;
            }
        }
        else {
            if(SH_CAMLED_GPIO102_LO < dev_camled) {
                atomic_dec(&camled);
            }
            else {
                break;
            }
        }

        if(SH_CAMLED_NO_ERROR == result) {
            dev_keyled = atomic_read(&keyled);
            dev_camled = atomic_read(&camled);
            if((SH_CAMLED_GPIO102_LO == dev_camled)
            && (SH_CAMLED_GPIO102_LO == dev_keyled)) {
                result = gpio_direction_output(SH_CAMLED_GPIO_WLED_EN_M,
                                                SH_CAMLED_GPIO102_LO);
                if(SH_CAMLED_NO_ERROR != result) {
                    result = SH_CAMLED_GPIO_LO_ERR;
                    printk(KERN_ERR "[%s] error result=%d\n", __func__, result);
                }
            }
            else {
                if(SH_CAMLED_GPIO102_LO != dev_keyled) {
                    result = SH_CAMLED_USE_KEYLED;
                }
                if(SH_CAMLED_GPIO102_LO != dev_camled) {
                    result = SH_CAMLED_USE_CAMERA;
                }
            }
        }
        break;

    default:
        result = -ENOEXEC;
        break;
    }

    mutex_unlock(&sh_camled_gpio_mutex_lock);

    return result;
}


module_init(sh_camled_drv_start);
module_exit(sh_camled_drv_remove);

MODULE_DESCRIPTION("SHARP CAMLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.01");
