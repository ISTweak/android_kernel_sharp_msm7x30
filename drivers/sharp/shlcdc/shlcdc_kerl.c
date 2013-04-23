/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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

/*
 * SHARP LCD CONTROLLER DRIVER FOR KERNEL
 */

#ifndef CONFIG_SHLCDC_SUBDISPLAY

/*
 * INCLUDE FILES
 */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shlcdc_kerl.h>
#include <sharp/sh_smem.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <asm/io.h>
#include <asm/pgtable.h>

/*
 * MACROS
 */

//#define KERN_DBG_ENABLE_A
//#define KERN_DBG_ENABLE_B
//#define KERN_DBG_ENABLE_C

#define SHLCDC_FILE "shlcdc_kerl.c"

#ifdef KERN_DBG_ENABLE_A
#define KERN_DBG_A(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_B
#define KERN_DBG_B(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_C
#define KERN_DBG_C(fmt, args...)     \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#else /* !KERN_DBG_ENABLE_C */
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_C */
#else /* !KERN_DBG_ENABLE_B */
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_B */
#else /* !KERN_DBG_ENABLE_A */
#define KERN_DBG_A(fmt, args...)
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_A */

#define SHLCDC_IOC_MAGIC 'l'
#define SHLCDC_IOCTL_SET_VDLINK_MUTEX _IOW(SHLCDC_IOC_MAGIC, 0, int)

/*
 * TYPES
 */

struct shlcdc_event {
    int event_type[NUM_SHLCDC_EVENT_TYPE];
};

/*
 * VARIABLES
 */

static dev_t shlcdc_dev;
static dev_t shlcdc_major = 0;
static dev_t shlcdc_minor = 0;
static struct cdev shlcdc_cdev;
static struct class *shlcdc_class;
static void *shlcdc_base_addr = NULL;
static wait_queue_head_t shlcdc_wq_for_read;
static atomic_t shlcdc_atomic_for_read;
static signed long shlcdc_event_counter_for_read;
static int shlcdc_driver_is_initialized = 0;
static struct shlcdc_event shlcdc_event_info;
static struct shdisp_boot_context shdisp_boot_ctx;
static spinlock_t shlcdc_spin_lock;
DECLARE_MUTEX(shdisp_vdlink_mutex);

/*
 * PROTOTYPES
 */

void shlcdc_wake_up(int *event_type);
static int shlcdc_open(struct inode *inode, struct file *filp);
static int shlcdc_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos);
static int shlcdc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos);
static int shlcdc_mmap(struct file *filp, struct vm_area_struct *vma);
static int shlcdc_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg);
static int shlcdc_release(struct inode *inode, struct file *filp);
static void shlcdc_set_int_factor(int *event_type);
static void shlcdc_get_int_factor(int *event_type);

static struct file_operations shlcdc_fops = {
    .owner   = THIS_MODULE,
    .open    = shlcdc_open,
    .write   = shlcdc_write,
    .read    = shlcdc_read,
    .mmap    = shlcdc_mmap,
    .ioctl   = shlcdc_ioctl,
    .release = shlcdc_release,
};

/*
 * FUNCTIONS
 */

/*
 * shlcdc_wake_up
 */

void shlcdc_wake_up(int *event_type)
{
    spin_lock(&shlcdc_spin_lock);

    shlcdc_set_int_factor(event_type);

    spin_unlock(&shlcdc_spin_lock);

    atomic_inc(&shlcdc_atomic_for_read);
    wake_up_interruptible(&shlcdc_wq_for_read);

    return;
}

/*
 * shlcdc_open
 */

static int shlcdc_open(struct inode *inode, struct file *filp)
{
    if (shlcdc_driver_is_initialized == 0) {
        shlcdc_event_counter_for_read = atomic_read(&shlcdc_atomic_for_read);
    }

    shlcdc_driver_is_initialized++;

    return 0;
}

/*
 * shlcdc_write
 */

static int shlcdc_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos)
{
    return 0;
}

/*
 * shlcdc_read
 */

static int shlcdc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    DECLARE_WAITQUEUE(wait, current);
    int ret = 0;
    int event_count;
    struct shlcdc_event local_event_info;

    if (count != sizeof(local_event_info)) {
        return -EPERM;
    }

    add_wait_queue(&shlcdc_wq_for_read, &wait);

    do {
        set_current_state(TASK_INTERRUPTIBLE);

        event_count = atomic_read(&shlcdc_atomic_for_read);

        if (event_count != shlcdc_event_counter_for_read) {
            spin_lock(&shlcdc_spin_lock);
            memset(&local_event_info, 0, sizeof(local_event_info));
            shlcdc_get_int_factor(&local_event_info.event_type[0]);
            spin_unlock(&shlcdc_spin_lock);
            if (copy_to_user(buf, &local_event_info, count) != 0) {
                ret = -EFAULT;
            } else {
                shlcdc_event_counter_for_read = event_count;
                ret = count;
            }
            break;
        }

        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }

        if (signal_pending(current)) {
            ret = -ERESTARTSYS;
            break;
        }

        schedule();
    } while (1);

    __set_current_state(TASK_RUNNING);
    remove_wait_queue(&shlcdc_wq_for_read, &wait);

    return ret;
}

/*
 * shlcdc_mmap
 */

static int shlcdc_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    ret = io_remap_pfn_range(vma, vma->vm_start,
                       (unsigned long)shlcdc_base_addr >> PAGE_SHIFT,
                       vma->vm_end - vma->vm_start, vma->vm_page_prot);

    return 0;
}

/*
 * shlcdc_ioctl
 */

static int shlcdc_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg)
{
    int ret, sw;
    void __user *argp = (void __user*)arg;

    switch (cmd) {
    case SHLCDC_IOCTL_SET_VDLINK_MUTEX:
        ret = copy_from_user(&sw, argp, sizeof(int));
        if (ret != 0) {
            return ret;
        }
        if (sw == 0) {
            up(&shdisp_vdlink_mutex);
        } else {
            down(&shdisp_vdlink_mutex);
        }
        break;
    default:
        ret = 0;
        break;
    }

    return ret;
}

/*
 * shlcdc_release
 */

static int shlcdc_release(struct inode *inode, struct file *filp)
{
    if (shlcdc_driver_is_initialized > 0) {
        shlcdc_driver_is_initialized--;

        if (shlcdc_driver_is_initialized == 0) {
            atomic_set(&shlcdc_atomic_for_read, 0);
        }
    }

    return 0;
}

/*
 * shlcdc_set_int_factor
 */

static void shlcdc_set_int_factor(int *event_type)
{
    if (event_type[SHLCDC_EVENT_TYPE_IRRC] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_SDHC] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDHC] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_GPIO] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_GPIO] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_PVSYNC] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_PVSYNC] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_I2C] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_I2C] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_CAMVIEW] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CAMVIEW] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_SDDET_H] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_H] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_SDDET_L] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_L] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_CSTM] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSTM] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_CSI] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSI] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_IRRC2] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC2] = 1;
    }

    return;
}

/*
 * shlcdc_get_int_factor
 */

static void shlcdc_get_int_factor(int *event_type)
{
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC] == 1) {
        event_type[SHLCDC_EVENT_TYPE_IRRC] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDHC] == 1) {
        event_type[SHLCDC_EVENT_TYPE_SDHC] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDHC] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_GPIO] == 1) {
        event_type[SHLCDC_EVENT_TYPE_GPIO] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_GPIO] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_PVSYNC] == 1) {
        event_type[SHLCDC_EVENT_TYPE_PVSYNC] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_PVSYNC] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_I2C] == 1) {
        event_type[SHLCDC_EVENT_TYPE_I2C] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_I2C] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CAMVIEW] == 1) {
        event_type[SHLCDC_EVENT_TYPE_CAMVIEW] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CAMVIEW] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_H] == 1) {
        event_type[SHLCDC_EVENT_TYPE_SDDET_H] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_H] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_L] == 1) {
        event_type[SHLCDC_EVENT_TYPE_SDDET_L] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_L] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSTM] == 1) {
        event_type[SHLCDC_EVENT_TYPE_CSTM] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSTM] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSI] == 1) {
        event_type[SHLCDC_EVENT_TYPE_CSI] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSI] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC2] == 1) {
        event_type[SHLCDC_EVENT_TYPE_IRRC2] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC2] = 0;
    }

    return;
}

/*
 * shlcdc_init
 */

static int __init shlcdc_init(void)
{
    int ret;

    sharp_smem_common_type *sh_smem_common;

    sh_smem_common = sh_smem_get_common_address();

    memcpy(&shdisp_boot_ctx,
           &(sh_smem_common->shdisp_boot_ctx),
           sizeof(struct shdisp_boot_context));
	#if 0
    if ((shdisp_boot_ctx.hw_revision & 0x000F) == 0x0000) {
        shlcdc_base_addr = (unsigned char*)0x94000000;
    } else {
        shlcdc_base_addr = (unsigned char*)0x60000000;
    }
	#endif
	shlcdc_base_addr = (unsigned char*)0x8B000000;
	
    ret = alloc_chrdev_region(&shlcdc_dev, 0, 1, "shlcdc");

    if (!ret) {
        shlcdc_major = MAJOR(shlcdc_dev);
        shlcdc_minor = MINOR(shlcdc_dev);
    } else {
        goto shlcdc_err_1;
    }

    cdev_init(&shlcdc_cdev, &shlcdc_fops);

    shlcdc_cdev.owner = THIS_MODULE;
    shlcdc_cdev.ops = &shlcdc_fops;

    ret = cdev_add(&shlcdc_cdev, shlcdc_dev, 1);

    if (ret) {
        goto shlcdc_err_2;
    }

    shlcdc_class = class_create(THIS_MODULE, "shlcdc");

    if (IS_ERR(shlcdc_class)) {
        goto shlcdc_err_2;
    }

    device_create(shlcdc_class, NULL,
                  shlcdc_dev, &shlcdc_cdev, "shlcdc");

    init_waitqueue_head(&shlcdc_wq_for_read);

    memset(&shlcdc_event_info, 0, sizeof(shlcdc_event_info));
    spin_lock_init(&shlcdc_spin_lock);

    atomic_set(&shlcdc_atomic_for_read, 0);;

    return 0;

shlcdc_err_2:
    cdev_del(&shlcdc_cdev);

shlcdc_err_1:
    return -1;
}
module_init(shlcdc_init);

/*
 * shlcdc_exit
 */

static void shlcdc_exit(void)
{
    device_destroy(shlcdc_class, shlcdc_dev);
    class_destroy(shlcdc_class);
    cdev_del(&shlcdc_cdev);

    return;
}
module_exit(shlcdc_exit);

MODULE_DESCRIPTION("SHARP LCD CONTROLLER DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

#else /* CONFIG_SHLCDC_SUBDISPLAY */

/*
 * INCLUDE FILES
 */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shlcdc_kerl.h>
#include <sharp/shlcdc_dev.h>
#include <sharp/sh_smem.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <asm/io.h>
#include <asm/pgtable.h>

#include <linux/spi/spi.h>

/*
 * MACROS
 */
//#define SHLCDC_LOG_ERROR_ENABLE
#if defined( SHLCDC_LOG_ERROR_ENABLE )
#define SHLCDC_LOG_ERROR(p) p
#else
#define SHLCDC_LOG_ERROR(p)
#endif /* defined( SHLCDC_LOG_ERROR_ENABLE ) */
//#define SHLCDC_LOG_DEBUG_ENABLE
#if defined( SHLCDC_LOG_DEBUG_ENABLE )
#define SHLCDC_LOG_DEBUG(p) p
#else
#define SHLCDC_LOG_DEBUG(p)
#endif /* defined( SHLCDC_LOG_DEBUG_ENABLE ) */

//#define KERN_DBG_ENABLE_A
//#define KERN_DBG_ENABLE_B
//#define KERN_DBG_ENABLE_C

#define SHLCDC_FILE "shlcdc_kerl.c"

#ifdef KERN_DBG_ENABLE_A
#define KERN_DBG_A(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_B
#define KERN_DBG_B(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_C
#define KERN_DBG_C(fmt, args...)     \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#else /* !KERN_DBG_ENABLE_C */
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_C */
#else /* !KERN_DBG_ENABLE_B */
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_B */
#else /* !KERN_DBG_ENABLE_A */
#define KERN_DBG_A(fmt, args...)
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_A */

#define SHLCDC_IOC_MAGIC 'l'
#define SHLCDC_IOCTL_SET_VDLINK_MUTEX _IOW(SHLCDC_IOC_MAGIC, 0, int)
#define SHLCDC_IOCTL_EXEC_DISPON _IOR(SHLCDC_IOC_MAGIC,1, int)
#define SHLCDC_IOCTL_EXEC_OFFSEQ _IOR(SHLCDC_IOC_MAGIC,2, int)
#define SHLCDC_IOCTL_EXEC_CSLOW _IOR(SHLCDC_IOC_MAGIC,3, int)
#define SHLCDC_IOCTL_SUB_UPDATE _IOR(SHLCDC_IOC_MAGIC, 4, int)
#define SHLCDC_IOCTL_EXEC_INITCMD _IOR(SHLCDC_IOC_MAGIC,5, int)
#define SHLCDC_IOCTL_EXEC_INITCMD_CLEAR_BUF _IOR(SHLCDC_IOC_MAGIC,6, int)

#define SHLCDC_SPIRETRY_ENABLE


/*
 * TYPES
 */
struct shlcdc_sub_update {
    unsigned short start_xps;
    unsigned short start_yps;
    unsigned short width;
    unsigned short height;
    unsigned char *buf;
};

struct shlcdc_event {
    int event_type[NUM_SHLCDC_EVENT_TYPE];
};
struct el_pos {
    unsigned char xstart;
    unsigned char ystart;
    unsigned char xend;
    unsigned char yend;
};

enum {
    DCLINE_COMMAND,
    DCLINE_DATA
};

enum {
    SHLCDC_CHIPSELECT_LO,
    SHLCDC_CHIPSELECT_HI
};

enum {
    SHLCDC_INIT_DONT_CLEAR_BUF,
    SHLCDC_INIT_CLEAR_BUF
};
/*
 * VARIABLES
 */

static dev_t shlcdc_dev;
static dev_t shlcdc_major = 0;
static dev_t shlcdc_minor = 0;
static struct cdev shlcdc_cdev;
static struct class *shlcdc_class;
static void *shlcdc_base_addr = NULL;
static wait_queue_head_t shlcdc_wq_for_read;
static atomic_t shlcdc_atomic_for_read;
static signed long shlcdc_event_counter_for_read;
static int shlcdc_driver_is_initialized = 0;
static struct shlcdc_event shlcdc_event_info;
static struct shdisp_boot_context shdisp_boot_ctx;
static spinlock_t shlcdc_spin_lock;
static struct spi_device *spid;
DECLARE_MUTEX(shdisp_vdlink_mutex);


static const unsigned char OnSeqCmd[INITCMDSIZE]={
    0x01,0x02,0x09,0x10,0x12,0x13,0x14,0x16,0x17,0x18,
    0x1a,0x1c,0x1d,0x30,0x30,0x32,0x32,0x34,0x35,0x36,
    0x37,0x38,0x39,0x48,0xc3,0xc4,0xcc,0xcd,0xd0,0xd2,
    0xd4,0xd5,0xd6,0xd7,0xd9,0xdb,0xdd
};

static const unsigned char OnSeqDat[INITCMDSIZE]={
    0x00,0x00,0x00,0x05,0x31,0x00,0x00,0x00,0x00,0x04,
    0x01,0x00,0x0b,0x10,0x6f,0x01,0x18,0x00,0x0b,0x01,
    0x18,0x70,0x00,0x03,0x00,0x00,0x00,0x00,0x80,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0f,0x87
};

/*
 * PROTOTYPES
 */

void shlcdc_wake_up(int *event_type);
static int shlcdc_open(struct inode *inode, struct file *filp);
static int shlcdc_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos);
static int shlcdc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos);
static int shlcdc_mmap(struct file *filp, struct vm_area_struct *vma);
static int shlcdc_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg);
static int shlcdc_release(struct inode *inode, struct file *filp);
static void shlcdc_set_int_factor(int *event_type);
static void shlcdc_get_int_factor(int *event_type);

static int __devinit shlcdc_probe(struct spi_device *spi);
static void shlcdc_gpio_init(void);
static int __init shlcdc_drv_init(void);
static int shlcdc_suspend(struct spi_device *spi, pm_message_t mesg);
static int __devexit shlcdc_remove(struct spi_device *spi);
static int shlcdc_resume(struct spi_device *spi);

static int shlcdc_spisync(struct spi_device *spi, const unsigned char data[],int dn);
static int shlcdc_sub_disp_setpos(struct spi_device * spi,struct el_pos *pos);
static int shlcdc_sub_disp_write_rect(struct spi_device *spi,struct shlcdc_sub_update * update);
static int shlcdc_exec_init_command(struct spi_device *spi);

static int shlcdc_sub_disp_resetpos(struct spi_device * spi);
static int shlcdc_sub_disp_writedata(struct spi_device *spi,unsigned char data[]);
static int shlcdc_sub_disp_write_allblack(struct spi_device *spi);

static int shlcdc_sub_disp_onoff(unsigned char value);
static int shlcdc_init_sub_disp(unsigned char value);
static int shlcdc_sub_disp_offseq(void);

static int shlcdc_sub_disp_update(struct shlcdc_sub_update *update);

static struct file_operations shlcdc_fops = {
    .owner   = THIS_MODULE,
    .open    = shlcdc_open,
    .write   = shlcdc_write,
    .read    = shlcdc_read,
    .mmap    = shlcdc_mmap,
    .ioctl   = shlcdc_ioctl,
    .release = shlcdc_release,
};

/*
 * FUNCTIONS
 */

/*
 * shlcdc_wake_up
 */

void shlcdc_wake_up(int *event_type)
{
    spin_lock(&shlcdc_spin_lock);

    shlcdc_set_int_factor(event_type);

    spin_unlock(&shlcdc_spin_lock);

    atomic_inc(&shlcdc_atomic_for_read);
    wake_up_interruptible(&shlcdc_wq_for_read);

    return;
}

/*
 * shlcdc_open
 */

static int shlcdc_open(struct inode *inode, struct file *filp)
{
    if (shlcdc_driver_is_initialized == 0) {
        shlcdc_event_counter_for_read = atomic_read(&shlcdc_atomic_for_read);
    }

    shlcdc_driver_is_initialized++;

    return 0;
}

/*
 * shlcdc_write
 */

static int shlcdc_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos)
{
    return 0;
}

/*
 * shlcdc_read
 */

static int shlcdc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    DECLARE_WAITQUEUE(wait, current);
    int ret = 0;
    int event_count;
    struct shlcdc_event local_event_info;

    add_wait_queue(&shlcdc_wq_for_read, &wait);

    do {
        set_current_state(TASK_INTERRUPTIBLE);

        event_count = atomic_read(&shlcdc_atomic_for_read);

        if (event_count != shlcdc_event_counter_for_read) {
            spin_lock(&shlcdc_spin_lock);
            memset(&local_event_info, 0, sizeof(local_event_info));
            shlcdc_get_int_factor(&local_event_info.event_type[0]);
            spin_unlock(&shlcdc_spin_lock);
            if (copy_to_user(buf, &local_event_info, count) != 0) {
                ret = -EFAULT;
            } else {
                shlcdc_event_counter_for_read = event_count;
                ret = count;
            }
            break;
        }

        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }

        if (signal_pending(current)) {
            ret = -ERESTARTSYS;
            break;
        }

        schedule();
    } while (1);

    __set_current_state(TASK_RUNNING);
    remove_wait_queue(&shlcdc_wq_for_read, &wait);

    return ret;
}

/*
 * shlcdc_mmap
 */

static int shlcdc_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    ret = io_remap_pfn_range(vma, vma->vm_start,
                       (unsigned long)shlcdc_base_addr >> PAGE_SHIFT,
                       vma->vm_end - vma->vm_start, vma->vm_page_prot);

    return 0;
}

/*
 * shlcdc_ioctl
 */

static int shlcdc_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg)
{
    int ret, sw;
    struct shlcdc_sub_update update;
    unsigned short pos[4];
    unsigned char dt[SHLCDC_SUB_DISP_UPDATE_SIZE_MAX];
    unsigned char* cpt;
    void __user *argp = (void __user*)arg;

    cpt = (unsigned char*)argp;

    switch (cmd) {
    case SHLCDC_IOCTL_SET_VDLINK_MUTEX:
        ret = copy_from_user(&sw, argp, sizeof(int));
        if (ret != 0) {
            return ret;
        }
        if (sw == 0) {
            up(&shdisp_vdlink_mutex);
        } else {
            down(&shdisp_vdlink_mutex);
        }
        break;

    case SHLCDC_IOCTL_EXEC_INITCMD:
        ret=shlcdc_init_sub_disp(SHLCDC_INIT_DONT_CLEAR_BUF);
        break;

    case SHLCDC_IOCTL_EXEC_INITCMD_CLEAR_BUF:
        ret=shlcdc_init_sub_disp(SHLCDC_INIT_CLEAR_BUF);
        break;

    case SHLCDC_IOCTL_EXEC_DISPON:
        ret = shlcdc_sub_disp_onoff(SHLCDC_DATA_SUB_DISP_ON);
        break;

    case SHLCDC_IOCTL_EXEC_OFFSEQ:
        ret = shlcdc_sub_disp_offseq();
        break;

    case SHLCDC_IOCTL_SUB_UPDATE:
        ret = copy_from_user(pos, argp, sizeof(unsigned short)*4);
        update.start_xps=pos[0];
        update.start_yps=pos[1];
        update.width=pos[2];
        update.height=pos[3];

        ret = copy_from_user(dt, cpt+8, sizeof(unsigned char)*update.width*update.height);
        update.buf=dt;

        ret = shlcdc_sub_disp_update(&update);
        break;

    case SHLCDC_IOCTL_EXEC_CSLOW:
        SHLCDC_SET_CHIPSELECT(SHLCDC_CHIPSELECT_LO);
        ret = 0;
        break;

    default:
        ret = 0;
        break;
    }

    return ret;
}

/*
 * shlcdc_sub_disp_update
 */
static int shlcdc_sub_disp_update(struct shlcdc_sub_update *update)
{
    shlcdc_sub_disp_write_rect(spid,update);

    return 0;
}

static int shlcdc_init_sub_disp(unsigned char value){
    shlcdc_gpio_init();
    shlcdc_exec_init_command(spid);    

    if(value==SHLCDC_INIT_CLEAR_BUF){
        shlcdc_sub_disp_write_allblack(spid);
    }

    return 0;
}

/*
 * shlcdc_release
 */

static int shlcdc_release(struct inode *inode, struct file *filp)
{
    if (shlcdc_driver_is_initialized > 0) {
        shlcdc_driver_is_initialized--;

        if (shlcdc_driver_is_initialized == 0) {
            atomic_set(&shlcdc_atomic_for_read, 0);
        }
    }

    return 0;
}

/*
 * shlcdc_set_int_factor
 */

static void shlcdc_set_int_factor(int *event_type)
{
    if (event_type[SHLCDC_EVENT_TYPE_IRRC] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_SDHC] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDHC] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_GPIO] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_GPIO] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_PVSYNC] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_PVSYNC] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_I2C] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_I2C] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_CAMVIEW] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CAMVIEW] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_SDDET_H] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_H] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_SDDET_L] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_L] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_CSTM] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSTM] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_CSI] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSI] = 1;
    }
    if (event_type[SHLCDC_EVENT_TYPE_IRRC2] == 1) {
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC2] = 1;
    }

    return;
}

/*
 * shlcdc_get_int_factor
 */

static void shlcdc_get_int_factor(int *event_type)
{
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC] == 1) {
        event_type[SHLCDC_EVENT_TYPE_IRRC] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDHC] == 1) {
        event_type[SHLCDC_EVENT_TYPE_SDHC] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDHC] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_GPIO] == 1) {
        event_type[SHLCDC_EVENT_TYPE_GPIO] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_GPIO] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_PVSYNC] == 1) {
        event_type[SHLCDC_EVENT_TYPE_PVSYNC] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_PVSYNC] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_I2C] == 1) {
        event_type[SHLCDC_EVENT_TYPE_I2C] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_I2C] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CAMVIEW] == 1) {
        event_type[SHLCDC_EVENT_TYPE_CAMVIEW] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CAMVIEW] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_H] == 1) {
        event_type[SHLCDC_EVENT_TYPE_SDDET_H] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_H] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_L] == 1) {
        event_type[SHLCDC_EVENT_TYPE_SDDET_L] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_SDDET_L] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSTM] == 1) {
        event_type[SHLCDC_EVENT_TYPE_CSTM] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSTM] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSI] == 1) {
        event_type[SHLCDC_EVENT_TYPE_CSI] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_CSI] = 0;
    }
    if (shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC2] == 1) {
        event_type[SHLCDC_EVENT_TYPE_IRRC2] = 1;
        shlcdc_event_info.event_type[SHLCDC_EVENT_TYPE_IRRC2] = 0;
    }

    return;
}

/*
 * shlcdc_init
 */

static int __init shlcdc_init(void)
{
    int ret;

    sharp_smem_common_type *sh_smem_common;

    sh_smem_common = sh_smem_get_common_address();

    memcpy(&shdisp_boot_ctx,
           &(sh_smem_common->shdisp_boot_ctx),
           sizeof(struct shdisp_boot_context));
	#if 0
    if ((shdisp_boot_ctx.hw_revision & 0x000F) == 0x0000) {
        shlcdc_base_addr = (unsigned char*)0x94000000;
    } else {
        shlcdc_base_addr = (unsigned char*)0x60000000;
    }
	#endif
	shlcdc_base_addr = (unsigned char*)0x8B000000;
	
    ret = alloc_chrdev_region(&shlcdc_dev, 0, 1, "shlcdc");

    if (!ret) {
        shlcdc_major = MAJOR(shlcdc_dev);
        shlcdc_minor = MINOR(shlcdc_dev);
    } else {
        goto shlcdc_err_1;
    }

    cdev_init(&shlcdc_cdev, &shlcdc_fops);

    shlcdc_cdev.owner = THIS_MODULE;
    shlcdc_cdev.ops = &shlcdc_fops;

    ret = cdev_add(&shlcdc_cdev, shlcdc_dev, 1);

    if (ret) {
        goto shlcdc_err_2;
    }

    shlcdc_class = class_create(THIS_MODULE, "shlcdc");

    if (IS_ERR(shlcdc_class)) {
        goto shlcdc_err_2;
    }

    device_create(shlcdc_class, NULL,
                  shlcdc_dev, &shlcdc_cdev, "shlcdc");

    init_waitqueue_head(&shlcdc_wq_for_read);

    memset(&shlcdc_event_info, 0, sizeof(shlcdc_event_info));
    spin_lock_init(&shlcdc_spin_lock);

    atomic_set(&shlcdc_atomic_for_read, 0);;

    return 0;

shlcdc_err_2:
    cdev_del(&shlcdc_cdev);

shlcdc_err_1:
    return -1;
}
module_init(shlcdc_init);


static struct spi_driver shlcdc_driver = {

    .probe         = shlcdc_probe,
    .remove        = __devexit_p(shlcdc_remove),
    .suspend       = shlcdc_suspend,
    .resume        = shlcdc_resume,
    .driver        = {
        .name  = SH_SUBLCD_DEVNAME,
        .owner = THIS_MODULE,

    },
};

/*
 * shlcdc_drv_init
 */
static int __init shlcdc_drv_init(void)
{
    return spi_register_driver(&shlcdc_driver);
}
module_init(shlcdc_drv_init);


/*
 * shlcdc_exit
 */

static void shlcdc_exit(void)
{
    device_destroy(shlcdc_class, shlcdc_dev);
    class_destroy(shlcdc_class);
    cdev_del(&shlcdc_cdev);

    return;
}
module_exit(shlcdc_exit);


/*
 * shlcdc_probe
 */

static int __devinit shlcdc_probe(struct spi_device *spi)
{
    int result=0;

    spid=spi;
    shlcdc_gpio_init();

    return result;
}
/*
 * shlcdc_gpio_init
 */

static void shlcdc_gpio_init(void)
{    
#if defined(CONFIG_SHLCDC_SUBDISPLAY)
    int result;

    result = gpio_tlmm_config(GPIO_CFG(SHLCDC_GPIO_NUM_DATACOMMAND_LINE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if(result < 0){
        SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_gpio_init]spi_dcset gpio_tlmm_config() error : %d\n", result););
    }

    result = gpio_tlmm_config(GPIO_CFG(SHLCDC_GPIO_NUM_SPI_CS, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if(result < 0){
        SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_gpio_init]spi_cs1 gpio_tlmm_config() error : %d\n", result););
    }
#endif /* #if defined(CONFIG_SHLCDC_SUBDISPLAY) */
    return ;
}

/*
 * shlcdc_remove
 */

static int __devexit shlcdc_remove(struct spi_device *spi)
{
    return 0;
}

/*
 * shlcdc_suspend
 */


static int shlcdc_suspend(struct spi_device *spi, pm_message_t mesg)
{
#if 0
#if defined( SHLCDC_SUSPENDSLEEP_ENABLE )
    struct shlcdc_spi *ts = spi_get_drvdata(spi);
    SHLCDC_LOG_DEBUG( printk(KERN_DEBUG "[shlcdc]shlcdc_suspend() done\n"); );
    request_event(ts, SHLCDC_EVENT_SLEEP, 0);
#endif
#endif
#if defined(CONFIG_SHLCDC_SUBDISPLAY)
    gpio_tlmm_config(GPIO_CFG(SHLCDC_GPIO_NUM_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
    SHLCDC_LOG_DEBUG( printk(KERN_DEBUG "[shlcdc]shlcdc_suspend() done\n"); );
#endif
    return 0;
}
/*
 * shlcdc_resume
 */

static int shlcdc_resume(struct spi_device *spi)
{
#if defined(CONFIG_SHLCDC_SUBDISPLAY)
    gpio_tlmm_config(GPIO_CFG(SHLCDC_GPIO_NUM_SPI_CS, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    SHLCDC_LOG_DEBUG( printk(KERN_DEBUG "[shlcdc]shlcdc_resume() done\n"); );
#endif
    return 0;
}


/*
 * shlcdc_sub_disp_writedata
 */

static int shlcdc_sub_disp_writedata(struct spi_device *spi,unsigned char data[])
{
    int result;
    unsigned char addr;

    shlcdc_sub_disp_resetpos(spi);

    addr=SHLCDC_ADDR_DATA_WRITE;
    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&addr,1);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_writedata]spisync err\n"););

    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spi,data,SHLCDC_SUB_DISP_UPDATE_SIZE_MAX);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_writedata]spisync err\n"));;

    return result;
}

/*
 * shlcdc_spisync
 */

static int shlcdc_spisync(struct spi_device *spi, const unsigned char data[],int dn)
{
    struct spi_message   msg_dt;
    struct spi_transfer    t_dt;
    int   status=0;

    if(dn< 1){
        SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_kerl] err in spisync: data amount invalid \n"); );
        return SHLCDC_RESULT_FAILURE;
    }

    memset(&t_dt, 0, sizeof(t_dt));
    spi_message_init(&msg_dt);
    spi_message_add_tail(&t_dt, &msg_dt);
    t_dt.tx_buf         = data;
    t_dt.rx_buf         = NULL;
    t_dt.len            = dn;


#if defined( SHLCDC_SPIRETRY_ENABLE )
    {
        int retry = SHLCDC_SPIRETRY_LIMIT;
        int err;

        do{
            err = spi_sync(spi, &msg_dt);
            if(err != 0){
                spi_setup(spi);
            }
        }while(err != 0 && retry-- > 0);
        status = err;
    }
#else
    status =  spi_sync(spi, &msg_dt);
#endif 
    return status;
}

/*
 * shlcdc_sub_disp_setpos
 */

static int shlcdc_sub_disp_setpos (struct spi_device * spi,struct el_pos *pos){

    int result;
    unsigned char addr;

    addr=SHLCDC_ADDR_SET_POS_XSTART;
    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&addr,1); 

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_setpos]spisync err\n"););
    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spi,&(pos->xstart),1); 

    addr=SHLCDC_ADDR_SET_POS_XEND;
    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&addr,1); 

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_setpos]spisync err\n"););
    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spi,&(pos->xend),1); 

    addr=SHLCDC_ADDR_SET_POS_YSTART;
    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&addr,1); 

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_setpos]spisync err\n"););
    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spi,&(pos->ystart),1); 

    addr=SHLCDC_ADDR_SET_POS_YEND;
    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&addr,1); 
    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_setpos]spisync err\n"););

    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spi,&(pos->yend),1); 
    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_setpos]spisync err\n"););

    return result;
}

/*
 * shlcdc_sub_disp_resetpos
 */

static int shlcdc_sub_disp_resetpos (struct spi_device * spi)
{    
    struct el_pos pos={
        .xstart=SHLCDC_DATA_DEFAULT_POS_XSTART,
        .xend=SHLCDC_DATA_DEFAULT_POS_XEND,
        .ystart=SHLCDC_DATA_DEFAULT_POS_YSTART,
        .yend=SHLCDC_DATA_DEFAULT_POS_YEND
    };

    return shlcdc_sub_disp_setpos(spi,&pos);
}

/*
 * shlcdc_sub_disp_write_rect
 */

static int shlcdc_sub_disp_write_rect(struct spi_device *spi,struct shlcdc_sub_update * update) 
{

    int result;
    int dsize;
    unsigned char addr;

    struct el_pos pos={
        .xstart=(unsigned char)update->start_xps,
        .ystart=(unsigned char)update->start_yps,
        .xend  =(unsigned char)update->start_xps + update->width-1,
        .yend  =(unsigned char)update->start_yps + update->height-1,
    };

    dsize=update->width * update->height;

    shlcdc_sub_disp_setpos(spi,&pos); 

    addr=0x08;
    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&addr,1);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_write_rect]spisync err\n"););

    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spi,update->buf,dsize);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_write_rect]spisync err\n"););


    return result;
}
/*
 * shlcdc_exec_init_command
 */

static int shlcdc_exec_init_command(struct spi_device *spi){
    int i;
    int byte; 

    int result;

    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spi,&OnSeqCmd[0],1);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_exec_init_command]spisync err\n"););
    byte=0;

    for(i=1;i<INITCMDSIZE;i+=byte){
        SHLCDC_SET_DCLINE(DCLINE_COMMAND);
        result = shlcdc_spisync(spi,&OnSeqCmd[i],1);

        if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_exec_init_command]spisync err\n"););

        SHLCDC_SET_DCLINE(DCLINE_DATA);
        byte=1;
        while(i+byte < INITCMDSIZE && OnSeqCmd[i] == OnSeqCmd[i+byte]){
            byte++;
        }
        result = shlcdc_spisync(spi, &OnSeqDat[i], byte);

        if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_exec_init_command]spisync err\n"););
    }

    return 0;
}

/*
 * shlcdc_sub_disp_write_allblack
 */

static int shlcdc_sub_disp_write_allblack(struct spi_device *spi){
    static unsigned char data[SHLCDC_SUB_DISP_UPDATE_SIZE_MAX];

    memset(data,0x00,SHLCDC_SUB_DISP_UPDATE_SIZE_MAX);

    return shlcdc_sub_disp_writedata(spi,data);
}

/*
 * shlcdc_sub_disp_onoff
 */

static int shlcdc_sub_disp_onoff(unsigned char value){
    int result; 
    unsigned char addr=SHLCDC_ADDR_SUB_DISP_ONOFF;
    unsigned char data=value;


    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spid,&addr,1);
    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_onoff]spisync err\n"););

    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spid,&data,1);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_onoff]spisync err\n"););


    return result;
}

/*
 * shlcdc_sub_disp_offseq
 */


static int shlcdc_sub_disp_offseq(void){
    int result; 
    unsigned char addr=SHLCDC_ADDR_DISP_STB;
    unsigned char data=SHLCDC_DATA_DISP_STB_ON;

    shlcdc_sub_disp_onoff(SHLCDC_DATA_SUB_DISP_OFF);

    SHLCDC_SET_DCLINE(DCLINE_COMMAND);
    result = shlcdc_spisync(spid,&addr,1);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_offseq]spisync err\n"););

    SHLCDC_SET_DCLINE(DCLINE_DATA);
    result = shlcdc_spisync(spid,&data,1);

    if(result != 0) SHLCDC_LOG_ERROR (printk(KERN_DEBUG "[shlcdc_sub_disp_offseq]spisync err\n"););


    return result;
}
MODULE_DESCRIPTION("SHARP LCD CONTROLLER DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

#endif /* CONFIG_SHLCDC_SUBDISPLAY */
