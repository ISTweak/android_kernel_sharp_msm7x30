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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include <sharp/shlcdc_kerl.h>
#include <sharp/shsdisr_kerl.h>

/*#define DEBUG_LOG*/

#define print_error(...)  printk(KERN_ERR __VA_ARGS__)
#if defined( DEBUG_LOG )
#define print_debug(...) printk(KERN_DEBUG __VA_ARGS__)
#else
#define print_debug(...)
#endif
#define print_info(...) printk(KERN_INFO __VA_ARGS__)

#define SHSDISR_BASEMINOR (0)
#define SHSDISR_MINORCOUNT (1)
#define SHSDISR_NAME "shsdisr"
#define SHSDISR_BUFFER_SIZE (32)

static int shsdisr_open( struct inode* inode, struct file* filp );
static int shsdisr_release( struct inode* inode, struct file* filp );
static ssize_t shsdisr_read( struct file* filp, char __user *buf, size_t count, loff_t* pos );
static ssize_t shsdisr_write( struct file* filp, const char __user *buf, size_t count, loff_t* pos );
static int shsdisr_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg );
static int shsdisr_ioctl_event_subscribe( void __user *argp );
static int shsdisr_ioctl_event_unsubscribe( void __user *argp );
static void shsdisr_callback_sdhc( void );
static void shsdisr_callback_sddeth( void );
static void shsdisr_callback_sddetl( void );
/*static int shsdisr_sqe_read( char* buf, size_t count );*/
static ssize_t shsdisr_sqe_write( const char* buf, size_t count );

struct shsdisr_buffer{
    int write;
    int read;
    int count;
    char data[SHSDISR_BUFFER_SIZE];
};

static dev_t shsdisr_dev;
static struct cdev shsdisr_cdev;
static struct class *shsdisr_class;
static struct semaphore shsdisr_sem;
static wait_queue_head_t shsdisr_write_q;
static wait_queue_head_t shsdisr_read_q;
static struct file_operations shsdisr_fops = {
    .owner = THIS_MODULE,
    .open = shsdisr_open,
    .ioctl = shsdisr_ioctl,
    .release = shsdisr_release,
    .write = shsdisr_write,
    .read = shsdisr_read,
};
static struct shsdisr_buffer shsdisr_buffer = {
    .write = 0,
    .read = 0,
    .count = 0,
};

static int shsdisr_open( struct inode* inode, struct file* filp )
{
    print_debug( "shsdisr_open was called.\n" );
    return 0;
}

static int shsdisr_release( struct inode* inode, struct file* filp )
{
    print_debug( "shsdisr_release was called.\n" );
    return 0;
}

static ssize_t shsdisr_read( struct file* filp, char __user *buf, size_t count, loff_t* pos )
{
    int ret;

    print_debug( "shsdisr_read was called.\n" );

    ret = down_interruptible( &shsdisr_sem );
    if( ret != 0 ) return -ERESTARTSYS;

    while( shsdisr_buffer.count == 0 ){
        up( &shsdisr_sem );

        if( filp->f_flags & O_NONBLOCK ) return -EAGAIN;

        ret = wait_event_interruptible( shsdisr_read_q, ( shsdisr_buffer.count != 0) );
        if( ret != 0 ) return -ERESTARTSYS;

        ret = down_interruptible( &shsdisr_sem );
        if( ret != 0 ) return -ERESTARTSYS;
    }

    ret = copy_to_user( buf, &shsdisr_buffer.data[shsdisr_buffer.read], 1 );
    if( ret != 0 ){
        up( &shsdisr_sem );
        return -EFAULT;
    }

    shsdisr_buffer.read = (shsdisr_buffer.read + 1) % SHSDISR_BUFFER_SIZE;
    --shsdisr_buffer.count;

    up( &shsdisr_sem );
    /*wake_up_interruptible( &shsdisr_write_q );*/
    return 1;
}

static ssize_t shsdisr_write( struct file* filp, const char __user *buf, size_t count, loff_t* pos )
{
    print_debug( "shsdisr_write was called.\n" );
    return 0;
}

static int shsdisr_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg )
{
    int ret;
    void __user *argp = (void __user*)arg;

    print_debug( "shsdisr_ioctl was called.\n" );

    switch( cmd ){
    case SHSDISR_IOCTL_EVENT_SUBSCRIBE:
        ret =shsdisr_ioctl_event_subscribe( argp );
        break;
    case SHSDISR_IOCTL_EVENT_UNSUBSCRIBE:
        ret =shsdisr_ioctl_event_unsubscribe( argp );
        break;
    default:
        print_error( "shsdisr_ioctl invalid command.\n" );
        ret = -EFAULT;
        break;
    }
    return ret;
}

static int shsdisr_ioctl_event_subscribe( void __user *argp )
{
    int ret;
    int event_type;
    struct shlcdc_subscribe shlcdc_subscribe;

    ret = copy_from_user(  &event_type, argp, sizeof( int ) );
    if( ret != 0 ) return ret;

    print_debug( "shsdisr_ioctl_event_subscribe was called.(event_type=%d)\n", event_type );

    switch( event_type ){
    case SHLCDC_EVENT_TYPE_SDHC:
        shlcdc_subscribe.event_type = event_type;
        shlcdc_subscribe.callback = shsdisr_callback_sdhc;
        break;
    case SHLCDC_EVENT_TYPE_SDDET_H:
        shlcdc_subscribe.event_type = event_type;
        shlcdc_subscribe.callback = shsdisr_callback_sddeth;
        break;
    case SHLCDC_EVENT_TYPE_SDDET_L:
        shlcdc_subscribe.event_type = event_type;
        shlcdc_subscribe.callback = shsdisr_callback_sddetl;
        break;
    default:
        return -EINVAL;
    }
    ret = shlcdc_api_event_subscribe( &shlcdc_subscribe );
    if( ret != 0 ){
        print_error( "shlcdc_api_event_subscribe called.(ret = %d)\n", ret );
    }
    return ret;
}

static int shsdisr_ioctl_event_unsubscribe( void __user *argp )
{
    int ret;
    int event_type;

    ret = copy_from_user( &event_type, argp, sizeof( int ) );
    if( ret != 0 ) return ret;

    print_debug( "shsdisr_ioctl_event_unsubscribe was called.(event_type=%d)\n", event_type );

    switch( event_type ){
    case SHLCDC_EVENT_TYPE_SDHC:
    case SHLCDC_EVENT_TYPE_SDDET_H:
    case SHLCDC_EVENT_TYPE_SDDET_L:
        break;
    default:
        return -EINVAL;
    }
    ret = shlcdc_api_event_unsubscribe( event_type );
    if( ret != 0 ){
        print_error( "shlcdc_api_event_unsubscribe called.(ret = %d)\n", ret );
    }
    return ret;
}

static void shsdisr_callback_sdhc( void )
{
    char c = SHLCDC_EVENT_TYPE_SDHC;

    print_debug( "shsdisr_callback_sdhc was called.\n" );
    shsdisr_sqe_write( &c, sizeof( char ) );
}

static void shsdisr_callback_sddeth( void )
{
    char c = SHLCDC_EVENT_TYPE_SDDET_H;

    print_debug( "shsdisr_callback_sddeth was called.\n" );
    shsdisr_sqe_write( &c, sizeof( char ) );
}

static void shsdisr_callback_sddetl( void )
{
    char c = SHLCDC_EVENT_TYPE_SDDET_L;

    print_debug( "shsdisr_callback_sdetl was called.\n" );
    shsdisr_sqe_write( &c, sizeof( char ) );
}

/*
static int shsdisr_sqe_read( char* buf, size_t count )
{
    return 0;
}
*/

static ssize_t shsdisr_sqe_write( const char* buf, size_t count )
{
    int i;

    print_debug( "shsdisr_sqe_write was called.\n" );

    down( &shsdisr_sem );

    for( i = 0; i < count; ++i ){
        if( shsdisr_buffer.count >= SHSDISR_BUFFER_SIZE ){
            print_error( "shsdisr_sqe_write buffer overflow.\n" );
            goto exit;
        }
        shsdisr_buffer.data[shsdisr_buffer.write] = *buf++;
        shsdisr_buffer.write = (shsdisr_buffer.write + 1) % SHSDISR_BUFFER_SIZE;
        ++shsdisr_buffer.count;
    }
exit:
    up( &shsdisr_sem );
    wake_up_interruptible( &shsdisr_read_q );
    return i;
}

static int __init shsdisr_ker_init( void )
{
    int ret;
    struct device *dev;

    print_info( "shsdisr_ker_init was called.\n" );

    init_MUTEX( &shsdisr_sem );
    init_waitqueue_head( &shsdisr_write_q );
    init_waitqueue_head( &shsdisr_read_q );

    ret = alloc_chrdev_region( &shsdisr_dev, SHSDISR_BASEMINOR, SHSDISR_MINORCOUNT, SHSDISR_NAME );
    if( ret < 0 ){
        print_error( "shsdisr alloc_chrdev_region failed.\n" );
        return -1;
    }

    cdev_init( &shsdisr_cdev, &shsdisr_fops );
    shsdisr_cdev.owner = THIS_MODULE;
    shsdisr_cdev.ops = &shsdisr_fops;

    ret = cdev_add( &shsdisr_cdev, shsdisr_dev, SHSDISR_MINORCOUNT );
    if( ret < 0 ){
        print_error( "shsdisr cdev_add failed.\n" );
        unregister_chrdev_region( shsdisr_dev, SHSDISR_MINORCOUNT );
        return -1;
    }

    shsdisr_class = class_create( THIS_MODULE, SHSDISR_NAME );
    if( IS_ERR( shsdisr_class ) ){
        print_error( "shsdisr class_create failed.\n" );
        cdev_del( &shsdisr_cdev );
        unregister_chrdev_region( shsdisr_dev, SHSDISR_MINORCOUNT );
        return -1;
    }

    dev = device_create( shsdisr_class, NULL, shsdisr_dev, &shsdisr_cdev, SHSDISR_NAME );
    ret = IS_ERR( dev ) ? PTR_ERR( dev ) : 0;
    if( ret < 0 ){
        print_error( "shsdisr device_create failed.\n" );
        class_destroy( shsdisr_class );
        cdev_del( &shsdisr_cdev );
        unregister_chrdev_region( shsdisr_dev, SHSDISR_MINORCOUNT );
        return -1;
    }
    return 0;
}

static void __exit shsdisr_ker_exit( void )
{
    print_info( "shsdisr_ker_exit was called.\n" );

    device_destroy( shsdisr_class, shsdisr_dev );
    class_destroy( shsdisr_class );
    cdev_del( &shsdisr_cdev );
    unregister_chrdev_region( shsdisr_dev, SHSDISR_MINORCOUNT );
}

module_init( shsdisr_ker_init );
module_exit( shsdisr_ker_exit );

MODULE_DESCRIPTION("SHARP SD DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
