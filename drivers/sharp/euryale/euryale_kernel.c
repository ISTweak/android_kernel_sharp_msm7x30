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
#include <sharp/euryale_kernel.h>

/*#define DEBUG_LOG */
/*#define BACKWARD_COMPATIBILITY*/

#define print_error(...) printk(KERN_ERR __VA_ARGS__)
#if defined( DEBUG_LOG )
#define print_debug(...) printk(KERN_DEBUG __VA_ARGS__)
#else
#define print_debug(...)
#endif
#define print_info(...) printk(KERN_INFO __VA_ARGS__)

#define EURYALE_KER_BASEMINOR (0)
#define EURYALE_KER_MINORCOUNT (1)
#define EURYALE_KER_DRVNAME "euryale"
#define EURYALE_CLASS_NAME "cls_euryale"
#define EURYALE_SECTOR_SIZE (512)
#define EURYALE_SECTORS_PER_CLUSTER (64*2)
#define EURYALE_BUFFER_SIZE (sizeof( struct euryale_command ) + EURYALE_SECTOR_SIZE * EURYALE_SECTORS_PER_CLUSTER)

enum{
    EURYALE_OWNER_NONE = 0,
    EURYALE_OWNER_KERNEL,
    EURYALE_OWNER_USER
};

struct euryale_buffer{
    int owner;
    int write;
    int read;
    int count;
    char data[EURYALE_BUFFER_SIZE];
};

static int euryale_open( struct inode* inode, struct file* filp );
static int euryale_release( struct inode* inode, struct file* filp );
static ssize_t euryale_read( struct file* filp, char __user *buf, size_t count, loff_t* pos );
static ssize_t euryale_write( struct file* filp, const char __user *buf, size_t count, loff_t* pos );
static ssize_t euryale_sqe_read( void *ptr, size_t count );
static ssize_t euryale_sqe_write( const void *ptr, size_t count );
#if defined( BACKWARD_COMPATIBILITY )
static ssize_t euryale_sqe_write_end( void );
#endif

static dev_t euryale_dev;
static struct cdev euryale_cdev;
static struct class *euryale_class;
static struct semaphore euryale_sem;
static wait_queue_head_t euryale_user_read_q;
static wait_queue_head_t euryale_kernel_read_q;
static struct file_operations euryale_fops = {
    .owner = THIS_MODULE,
    .open = euryale_open,
    .release = euryale_release,
    .write = euryale_write,
    .read = euryale_read,
};
static struct euryale_buffer euryale_buffer = {
    .owner = EURYALE_OWNER_NONE,
    .write = 0,
    .read = 0,
    .count = 0,
};

#if defined( BACKWARD_COMPATIBILITY )
static size_t medousa_request_size = 0;
#endif

static int euryale_open( struct inode* inode, struct file* filp )
{
    print_debug( "euryale_open was called.\n" );
    return 0;
}

static int euryale_release( struct inode* inode, struct file* filp )
{
    print_debug( "euryale_release was called.\n" );
    return 0;
}

static ssize_t euryale_read( struct file* filp, char __user *buf, size_t count, loff_t* pos )
{
    int ret;
    ssize_t length;

    print_debug( "euryale_read was called.\n" );

#if defined( BACKWARD_COMPATIBILITY )
    if( medousa_request_size == 0 ){
        medousa_request_size = count;
    }
#endif

    ret = down_interruptible( &euryale_sem );
    if( ret != 0 ) return -ERESTARTSYS;

    while( !( euryale_buffer.owner == EURYALE_OWNER_KERNEL && euryale_buffer.count >= count ) ){
        up( &euryale_sem );

        if( filp->f_flags & O_NONBLOCK ) return -EAGAIN;

        ret = wait_event_interruptible( euryale_user_read_q,
                                        ( euryale_buffer.owner == EURYALE_OWNER_KERNEL && euryale_buffer.count >= count ) );
        if( ret != 0 ) return -ERESTARTSYS;

        ret = down_interruptible( &euryale_sem );
        if( ret != 0 ) return -ERESTARTSYS;
    }

    length = (count > euryale_buffer.count) ? euryale_buffer.count : count;

    ret = copy_to_user( buf, &euryale_buffer.data[euryale_buffer.read], length );
    if( ret != 0 ){
        up( &euryale_sem );
        return -EFAULT;
    }

    euryale_buffer.read += length;
    euryale_buffer.count -= length;

    up( &euryale_sem );

    return length;
}

static ssize_t euryale_write( struct file* filp, const char __user *buf, size_t count, loff_t* pos )
{
    int ret;
    ssize_t length;

    print_debug( "euryale_write was called.\n" );

    ret = down_interruptible( &euryale_sem );
    if( ret != 0 ) return -ERESTARTSYS;

    if( euryale_buffer.owner != EURYALE_OWNER_USER ){
        euryale_buffer.owner = EURYALE_OWNER_USER;
        euryale_buffer.write = 0;
        euryale_buffer.read = 0;
        euryale_buffer.count = 0;
    }

    if( euryale_buffer.count >= EURYALE_BUFFER_SIZE ){
        up( &euryale_sem );
        return -EBUSY;
    }

    length = (count > EURYALE_BUFFER_SIZE - euryale_buffer.write) ? EURYALE_BUFFER_SIZE - euryale_buffer.write : count;

    ret = copy_from_user( &euryale_buffer.data[euryale_buffer.write], buf, length );
    if( ret != 0 ){
        up( &euryale_sem );
        return -EFAULT;
    }

    euryale_buffer.write += length;
    euryale_buffer.count += length;

    up( &euryale_sem );
    wake_up_interruptible( &euryale_kernel_read_q );

    return length;
}

static ssize_t euryale_sqe_read( void *ptr, size_t count )
{
    char *buf = (char*)ptr;
    int ret;
    ssize_t length;

    print_debug( "euryale_sqe_read was called.(ptr=%lx len=%d)\n", (unsigned long)ptr, (int)count );

    ret = down_interruptible( &euryale_sem );
    if( ret != 0 ) return -ERESTARTSYS;

    while( !( euryale_buffer.owner == EURYALE_OWNER_USER && euryale_buffer.count >= count ) ){
        up( &euryale_sem );

        ret = wait_event_interruptible( euryale_kernel_read_q,
                                        ( euryale_buffer.owner == EURYALE_OWNER_USER && euryale_buffer.count >= count ) );
        if( ret != 0 ) return -ERESTARTSYS;

        ret = down_interruptible( &euryale_sem );
        if( ret != 0 ) return -ERESTARTSYS;
    }

    length = (count > euryale_buffer.count) ? euryale_buffer.count : count;

    memcpy( buf, &euryale_buffer.data[euryale_buffer.read], length );

    euryale_buffer.read += length;
    euryale_buffer.count -= length;

    up( &euryale_sem );

    return length;
}

static ssize_t euryale_sqe_write( const void *ptr, size_t count )
{
    char *buf = (char*)ptr;
    int ret;
    ssize_t length;

    print_debug( "euryale_sqe_write was called.(ptr=%lx len=%d)\n", (unsigned long)ptr, (int)count );

    ret = down_interruptible( &euryale_sem );
    if( ret != 0 ) return -ERESTARTSYS;

    if( euryale_buffer.owner != EURYALE_OWNER_KERNEL ){
        euryale_buffer.owner = EURYALE_OWNER_KERNEL;
        euryale_buffer.write = 0;
        euryale_buffer.read = 0;
        euryale_buffer.count = 0;
    }

    if( euryale_buffer.count >= EURYALE_BUFFER_SIZE ){
        up( &euryale_sem );
        return -EBUSY;
    }

    length = (count > EURYALE_BUFFER_SIZE - euryale_buffer.write) ? EURYALE_BUFFER_SIZE - euryale_buffer.write : count;

    memcpy( &euryale_buffer.data[euryale_buffer.write], buf, length );

    euryale_buffer.write += length;
    euryale_buffer.count += length;

    up( &euryale_sem );
    wake_up_interruptible( &euryale_user_read_q );

    return length;
}

#if defined( BACKWARD_COMPATIBILITY )
static ssize_t euryale_sqe_write_end( void )
{
    int ret;

    if( !euryale_need_backwardcompatibility() ) return 0;

    print_debug( "euryale_sqe_write_end was called.\n" );

    ret = down_interruptible( &euryale_sem );
    if( ret != 0 ) return -ERESTARTSYS;

    if( euryale_buffer.owner == EURYALE_OWNER_KERNEL ){
        euryale_buffer.write = EURYALE_BUFFER_SIZE;
        euryale_buffer.count = EURYALE_BUFFER_SIZE;
    }

    up( &euryale_sem );
    wake_up_interruptible( &euryale_user_read_q );

    return 0;
}
#endif

/**
 * euryale_write_process - request to write data
 * @sector: sector to write
 * @nr: number of sectors to write
 * @buffer: buffer to write
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int euryale_write_process( unsigned long sector, unsigned long nr, const char *buffer )
{
    struct euryale_command euryale_c;
    int ret;

    print_debug( "euryale_write_process was called.(sector=%d nr=%d)\n", (int)sector, (int)nr  );

    if( nr > EURYALE_SECTORS_PER_CLUSTER ){
        print_error( "euryale_write_process invalid nr.\n" );
        return -1;
    }

    euryale_c.inited = 1;
    euryale_c.cmd = EURYALE_COMMAND_WRITE;
    euryale_c.sector = sector;
    euryale_c.nr = nr;

    ret = euryale_sqe_write( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_write_process command write error.\n" );
        return -1;
    }

    ret = euryale_sqe_write( buffer, nr * EURYALE_SECTOR_SIZE  );
    if( ret != nr * EURYALE_SECTOR_SIZE ){
        print_error( "euryale_write_process data write error.\n" );
        return -1;
    }
#if defined( BACKWARD_COMPATIBILITY )
    ret = euryale_sqe_write_end();
    if( ret != 0 ){
        print_error( "euryale_write_process data write(2) error.\n" );
        return -1;
    }
#endif
    ret = euryale_sqe_read( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_write_process result read error.\n" );
        return -1;
    }

    if( euryale_c.ret != 0 ){
        print_error( "euryale_write_process failed.\n" );
        return -1;
    }
    return 0;
}

/**
 * euryale_read_process - request to read data
 * @sector: sector to read
 * @nr: number of sectors to read
 * @buffer: buffer to read
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int euryale_read_process( unsigned long sector, unsigned long nr, char *buffer )
{
    struct euryale_command euryale_c;
    int ret;

    print_debug( "euryale_read_process was called.(sector=%d nr=%d)\n", (int)sector, (int)nr );

    if( nr > EURYALE_SECTORS_PER_CLUSTER ){
        print_error( "euryale_read_process invalid nr.\n" );
        return -1;
    }

    euryale_c.inited = 1;
    euryale_c.cmd = EURYALE_COMMAND_READ;
    euryale_c.sector = sector;
    euryale_c.nr = nr;

    ret = euryale_sqe_write( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_read_process command write error.\n" );
        return -1;
    }
#if defined( BACKWARD_COMPATIBILITY )
    ret = euryale_sqe_write_end();
    if( ret != 0 ){
        print_error( "euryale_write_process data write(2) error.\n" );
        return -1;
    }
#endif
    ret = euryale_sqe_read( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_read_process result read error.\n" );
        return -1;
    }

    if( euryale_c.ret != 0 ){
        print_error( "euryale_read_process failed.\n" );
        return -1;
    }

    ret = euryale_sqe_read( buffer, nr * EURYALE_SECTOR_SIZE );
    if( ret != nr * EURYALE_SECTOR_SIZE ){
        print_error( "euryale_read_process data read error.\n" );
        return -1;
    }
    return 0;
}

/**
 * euryale_api_blockwrite - request to write data with discontinuous buffers
 * @sector: sector to write
 * @nr: number of sectors to write
 * @block: pointer of buffer lists to write
 *
 * Description:
 *  last @block->buffer must be NULL.
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int euryale_api_blockwrite( unsigned long sector, unsigned long nr, struct euryale_block *block )
{
    struct euryale_command euryale_c;
    int ret;

    print_debug( "euryale_api_blockwrite was called.(sector=%d nr=%d)\n", (int)sector, (int)nr );

    if( nr > EURYALE_SECTORS_PER_CLUSTER ){
        print_error( "euryale_api_blockwrite invalid nr.\n" );
        return -1;
    }

    euryale_c.inited = 1;
    euryale_c.cmd = EURYALE_COMMAND_WRITE;
    euryale_c.sector = sector;
    euryale_c.nr = nr;

    ret = euryale_sqe_write( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_write_process command write error.\n" );
        return -1;
    }

    while( block->buffer != NULL ){
        ret = euryale_sqe_write( block->buffer, block->length );
        if( ret != block->length ){
            print_error( "euryale_api_blockwrite data write error.\n" );
            return -1;
        }
        ++block;
    }
#if defined( BACKWARD_COMPATIBILITY )
    ret = euryale_sqe_write_end();
    if( ret != 0 ){
        print_error( "euryale_write_process data write(2) error.\n" );
        return -1;
    }
#endif
    ret = euryale_sqe_read( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_api_blockwrite result read error.\n" );
        return -1;
    }

    if( euryale_c.ret != 0 ){
        print_error( "euryale_api_blockwrite failed.\n" );
        return -1;
    }
    return 0;
}

/**
 * euryale_api_blockread - request to read data with discontinuous buffers
 * @sector: sector to read
 * @nr: number of sectors to read
 * @block: pointer of buffer lists to read
 *
 * Description:
 *  last @block->buffer must be NULL.
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int euryale_api_blockread( unsigned long sector, unsigned long nr, struct euryale_block *block )
{
    struct euryale_command euryale_c;
    int ret;

    print_debug( "euryale_api_blockread was called.(sector=%d nr=%d)\n", (int)sector, (int)nr );

    if( nr > EURYALE_SECTORS_PER_CLUSTER ){
        print_error( "euryale_api_blockread invalid nr.\n" );
        return -1;
    }

    euryale_c.inited = 1;
    euryale_c.cmd = EURYALE_COMMAND_READ;
    euryale_c.sector = sector;
    euryale_c.nr = nr;

    ret = euryale_sqe_write( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_api_blockread command write error.\n" );
        return -1;
    }
#if defined( BACKWARD_COMPATIBILITY )
    ret = euryale_sqe_write_end();
    if( ret != 0 ){
        print_error( "euryale_write_process data write(2) error.\n" );
        return -1;
    }
#endif
    ret = euryale_sqe_read( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_api_blockread result read error.\n" );
        return -1;
    }

    if( euryale_c.ret != 0 ){
        print_error( "euryale_api_blockread failed.\n" );
        return -1;
    }

    while( block->buffer != NULL ){
        ret = euryale_sqe_read( block->buffer,  block->length );
        if( ret != block->length ){
            print_error( "euryale_api_blockread data read error.\n" );
            return -1;
        }
        ++block;
    }
    return 0;
}

/**
 * euryale_api_init - request to initialize card
 *
 * Description:
 *  must be called immediately when sdcard insert.
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int euryale_api_init( void )
{
    struct euryale_command euryale_c;
    int ret;

    print_debug( "euryale_api_init was called.\n" );

#if defined( BACKWARD_COMPATIBILITY )
    if( euryale_need_backwardcompatibility() ){
        char buffer[EURYALE_SECTOR_SIZE];

        print_info( "euryale dummy read command.(inited = 0)\n" );

        euryale_c.inited = 0;
        euryale_c.cmd = EURYALE_COMMAND_READ;
        euryale_c.sector = 0;
        euryale_c.nr = 1;

        ret = euryale_sqe_write( &euryale_c, sizeof( struct euryale_command ) );
        if( ret != sizeof( struct euryale_command ) ){
            print_error( "euryale_read_process command write error.\n" );
            return -1;
        }

        ret = euryale_sqe_write_end();
        if( ret != 0 ){
            print_error( "euryale_write_process data write(2) error.\n" );
            return -1;
        }

        ret = euryale_sqe_read( &euryale_c, sizeof( struct euryale_command ) );
        if( ret != sizeof( struct euryale_command ) ){
            print_error( "euryale_read_process result read error.\n" );
            return -1;
        }

        if( euryale_c.ret != 0 ){
            print_error( "euryale_read_process failed.\n" );
            return -1;
        }

        ret = euryale_sqe_read( buffer, EURYALE_SECTOR_SIZE );
        if( ret != EURYALE_SECTOR_SIZE ){
            print_error( "euryale_read_process data read error.\n" );
            return -1;
        }
        return 0;
    }
#endif
    euryale_c.inited = 1;
    euryale_c.cmd = EURYALE_COMMAND_INIT;
    euryale_c.sector = 0;
    euryale_c.nr = 0;

    ret = euryale_sqe_write( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_api_init command write error.\n" );
        return -1;
    }
#if defined( BACKWARD_COMPATIBILITY )
    ret = euryale_sqe_write_end();
    if( ret != 0 ){
        print_error( "euryale_write_process data write(2) error.\n" );
        return -1;
    }
#endif
    ret = euryale_sqe_read( &euryale_c, sizeof( struct euryale_command ) );
    if( ret != sizeof( struct euryale_command ) ){
        print_error( "euryale_api_init result read error.\n" );
        return -1;
    }

    if( euryale_c.ret != 0 ){
        print_error( "euryale_api_init failed.\n" );
        return -1;
    }
    return 0;
}

/**
 * euryale_need_backwardcompatibility - notify to need backward compatibility
 *
 * Description:
 *  medousa needs backward compatibility or not.
 *
 * Return: 0 if no need, non 0 otherwise.
 */
int euryale_need_backwardcompatibility( void )
{
#if defined( BACKWARD_COMPATIBILITY )
    if( medousa_request_size > sizeof( struct euryale_command ) ){
        return 1;
    }
#endif
    return 0;
}

static int __init euryale_ker_init( void )
{
    int ret;
    struct device *dev;

    print_info( "euryale_ker_init was called.\n" );

    init_MUTEX( &euryale_sem );
    init_waitqueue_head( &euryale_user_read_q );
    init_waitqueue_head( &euryale_kernel_read_q );

    ret = alloc_chrdev_region( &euryale_dev, EURYALE_KER_BASEMINOR, EURYALE_KER_MINORCOUNT, EURYALE_KER_DRVNAME );
    if( ret < 0 ){
        print_error( "euryale alloc_chrdev_region failed.\n" );
        return -1;
    }

    cdev_init( &euryale_cdev, &euryale_fops );
    euryale_cdev.owner = THIS_MODULE;
    euryale_cdev.ops = &euryale_fops;

    ret = cdev_add( &euryale_cdev, euryale_dev, EURYALE_KER_MINORCOUNT );
    if( ret < 0 ){
        print_error( "euryale cdev_add failed.\n" );
        unregister_chrdev_region( euryale_dev, EURYALE_KER_MINORCOUNT );
        return -1;
    }

    euryale_class = class_create( THIS_MODULE, EURYALE_CLASS_NAME );
    if( IS_ERR( euryale_class ) ){
        print_error( "euryale class_create failed.\n" );
        cdev_del( &euryale_cdev );
        unregister_chrdev_region( euryale_dev, EURYALE_KER_MINORCOUNT );
        return -1;
    }

    dev = device_create( euryale_class, NULL, euryale_dev, &euryale_cdev, EURYALE_KER_DRVNAME );
    ret = IS_ERR( dev ) ? PTR_ERR( dev ) : 0;
    if( ret < 0 ){
        print_error( "euryale device_create failed.\n" );
        class_destroy( euryale_class );
        cdev_del( &euryale_cdev );
        unregister_chrdev_region( euryale_dev, EURYALE_KER_MINORCOUNT );
        return -1;
    }

    return 0;
}

static void __exit euryale_ker_exit( void )
{
    print_info( "euryale_ker_exit was called.\n" );

    device_destroy( euryale_class, euryale_dev );
    class_destroy( euryale_class );
    cdev_del( &euryale_cdev );
    unregister_chrdev_region( euryale_dev, EURYALE_KER_MINORCOUNT );
}

module_init( euryale_ker_init );
module_exit( euryale_ker_exit );

MODULE_DESCRIPTION("SHARP EURYALE MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
