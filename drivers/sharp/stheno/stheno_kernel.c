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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/vmalloc.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <asm/byteorder.h>
#include <sharp/stheno_kernel.h>
#include <sharp/euryale_kernel.h>

/*#define DEBUG_LOG */
/*#define BACKWARD_COMPATIBILITY*/
/*#define USE_KMALLOC*/

#define print_error(...) printk(KERN_ERR __VA_ARGS__)
#if defined( DEBUG_LOG )
#define print_debug(...) printk(KERN_DEBUG __VA_ARGS__)
#else
#define print_debug(...)
#endif
#define print_info(...) printk(KERN_INFO __VA_ARGS__)

#define STHENO_NAME "stheno"
#define STHENO_THREAD_NAME "sthenod"
#define STHENO_MINOR_COUNT (1)
#define SECTOR_SIZE (512)
#define MAX_SECTORS (64*2)
#define AMOUNT_OF_SECTORS (0x4000000)

static int stheno_request_thread( void *arg );
static void stheno_request( struct request_queue *q );
static int stheno_open( struct block_device *bdev, fmode_t fmode );
static int stheno_release( struct gendisk *gd, fmode_t fmode );
/*static int stheno_media_changed( struct gendisk *gd );
static int stheno_revalidate_disk( struct gendisk *gd );*/
static int stheno_getgeo( struct block_device *bdev, struct hd_geometry *geo );

static int stheno_major = -1;
static spinlock_t stheno_lock;
static struct request_queue *stheno_queue = NULL;
struct gendisk *stheno_gd = NULL;
static struct task_struct *stheno_thread;
static wait_queue_head_t stheno_wait_q;
static int stheno_wakeup = 0;
static int stheno_read_sector0_flag = 0;
static unsigned long stheno_lbaoffset = 0;
static int stheno_usage = 0;
static DEFINE_MUTEX( stheno_openlock );
static struct wake_lock stheno_wakelock;

static struct block_device_operations stheno_fops = {
    .owner = THIS_MODULE,
    .open = stheno_open,
    .release = stheno_release,
    /*.media_changed = stheno_media_changed,*/
    /*.revalidate_disk = stheno_revalidate_disk,*/
    .getgeo = stheno_getgeo
};

static int stheno_read_sector0( void )
{
    char boot_sector[SECTOR_SIZE];
    int ret;

    if( stheno_read_sector0_flag == 0 ) return 0;

    ret = euryale_api_init();
    if( ret != 0 ){
        print_error( "stheno euryale_api_init failed.\n" );
        return -1;
    }
#if defined( BACKWARD_COMPATIBILITY )
    if( euryale_need_backwardcompatibility() ){
        print_info( "stheno backward compatibility enable.\n" );
        stheno_lbaoffset = 0;
        goto exit;
    }
#endif
    ret = euryale_read_process( 0, 1, boot_sector );
    if( ret != 0 ){
        print_error( "stheno cannot read sector #0.\n" );
        return -1;
    }

    if( (boot_sector[0] == 0xEB && boot_sector[2] == 0x90) || boot_sector[0] == 0xE9 ){
        /* Boot Sector(FAT12/16/32) */
        stheno_lbaoffset = 0;
    }else{
        /* Master Boot Record */
        /* PT_LbaOfs in MBR_Partation1 */
        memcpy( &stheno_lbaoffset, &boot_sector[446+8], sizeof( unsigned long ) );
        stheno_lbaoffset = le32_to_cpu( stheno_lbaoffset );
    }

    print_info( "stheno boot sector = %x %x %x\n",
                 (int)boot_sector[0] & 0xFF, (int)boot_sector[1] & 0xFF, (int)boot_sector[2] & 0xFF );
    print_info( "stheno LBA offset = %ld\n", stheno_lbaoffset );
exit:
    stheno_read_sector0_flag = 0;
    return 0;
}

#if 0
static int stheno_request_thread( void *arg )
{
    struct request *req;
    int ret;

    while( 1 ){
        ret = wait_event_interruptible( stheno_wait_q, (kthread_should_stop() || stheno_wakeup == 1) );
        if( ret != 0 ) break;

        stheno_wakeup = 0;

        if( kthread_should_stop() ) break;

        while( 1 ){
            spin_lock_irq( stheno_queue->queue_lock );
            req = blk_fetch_request( stheno_queue );
            spin_unlock_irq( stheno_queue->queue_lock );
        next_segment:
            if( req == NULL ) break;

            if( !blk_fs_request( req ) ){
                /*blk_end_request_cur( req, -EIO );*/
                spin_lock_irq( stheno_queue->queue_lock );
                ret = __blk_end_request_cur( req, -EIO );
                spin_unlock_irq( stheno_queue->queue_lock );
                if( ret == true ) goto next_segment;
                continue;
            }
            if( stheno_read_sector0() != 0 ){
                spin_lock_irq( stheno_queue->queue_lock );
                ret = __blk_end_request_cur( req, -EIO );
                spin_unlock_irq( stheno_queue->queue_lock );
                if( ret == true ) goto next_segment;
                continue;
            }
            if( blk_rq_sectors( req ) == 0 || blk_rq_cur_sectors( req ) == 0 ){
                spin_lock_irq( stheno_queue->queue_lock );
                ret = __blk_end_request_cur( req, -EIO );
                spin_unlock_irq( stheno_queue->queue_lock );
                if( ret == true ) goto next_segment;
                continue;
            }
            if( rq_data_dir( req ) == 0 ){
                ret = euryale_read_process( stheno_lbaoffset + blk_rq_pos( req ), blk_rq_cur_sectors( req ), req->buffer );
            }else{
                ret = euryale_write_process( stheno_lbaoffset + blk_rq_pos( req ), blk_rq_cur_sectors( req ), req->buffer );
            }
            /*blk_end_request_cur( req, ret == 0 ? 0 : -EIO );*/
            spin_lock_irq( stheno_queue->queue_lock );
            ret = __blk_end_request_cur( req, ret == 0 ? 0 : -EIO );
            spin_unlock_irq( stheno_queue->queue_lock );
            if( ret == true ) goto next_segment;
        }
    }
    print_debug("stheno_request_thread was terminated.\n");
    return 0;
}
#else
#if !defined( USE_KMALLOC )
static struct euryale_block euryale_block[MAX_SECTORS+1];
#endif
static int stheno_request_thread( void *arg )
{
#if defined( USE_KMALLOC )
    struct euryale_block *euryale_block;
#else
    /* kernel stack size has 1024 bytes(?) */
    /*struct euryale_block euryale_block[MAX_SECTORS+1];*/
#endif
    unsigned long sector;
    unsigned long nr;
    struct request *req;
    struct bio_vec *bvec;
    struct req_iterator iter;
    int ret;
    int i;

#if defined( USE_KMALLOC )
    euryale_block = (struct euryale_block*)kmalloc( sizeof( struct euryale_block ) * (MAX_SECTORS + 1), GFP_KERNEL );
    if( euryale_block == NULL ){
        print_error( "stheno kmalloc failed.\n" );
    }
#endif
    while( 1 ){
        ret = wait_event_interruptible( stheno_wait_q, (kthread_should_stop() || stheno_wakeup == 1) );
        if( ret != 0 ) break;

        stheno_wakeup = 0;

        if( kthread_should_stop() ) break;

        wake_lock( &stheno_wakelock );

        while( 1 ){
            spin_lock_irq( stheno_queue->queue_lock );
            req = blk_fetch_request( stheno_queue );
            spin_unlock_irq( stheno_queue->queue_lock );
            if( req == NULL ) break;

            if( !blk_fs_request( req ) ){
                ret = -EIO;
                goto skip;
            }
            if( stheno_read_sector0() != 0 ){
                ret = -EIO;
                goto skip;
            }
            sector = blk_rq_pos( req );
            nr = blk_rq_sectors( req );

#if defined( USE_KMALLOC )
            if( euryale_buffer == NULL ){
                ret = -EIO;
                goto skip;
            }
#endif
            i = 0;
            rq_for_each_segment( bvec, req, iter ){
                if( i >= MAX_SECTORS ){
                    print_error( "stheno euryale_block overrun error.\n" );
                    ret = -EIO;
                    goto skip;
                }
                euryale_block[i].buffer = page_address(bvec->bv_page) + bvec->bv_offset;
                euryale_block[i].length = bvec->bv_len;
                ++i;
            }
            euryale_block[i].buffer = 0; /* end of buffer */

            if( rq_data_dir( req ) == 0 ){
                ret = euryale_api_blockread( stheno_lbaoffset + sector, nr, euryale_block );
                print_debug( "stheno euryale_api_blockread sec=%ld nr=%ld ret=%d.\n", stheno_lbaoffset + sector, nr, ret );
            }else{
                ret = euryale_api_blockwrite( stheno_lbaoffset + sector, nr, euryale_block );
                print_debug( "stheno euryale_api_blockwrite sec=%ld nr=%ld ret=%d.\n", stheno_lbaoffset + sector, nr, ret );
            }
        skip:
            spin_lock_irq( stheno_queue->queue_lock );
            __blk_end_request_all( req, ret == 0 ? 0 : -EIO );
            /*__blk_end_request( req, ret == 0 ? 0 : -EIO, blk_rq_bytes( req ) );*/
            spin_unlock_irq( stheno_queue->queue_lock );
            /*print_debug( "stheno blk_end_request called.(ret=%d)\n", ret );*/
        }
        wake_unlock( &stheno_wakelock );
        /*print_debug( "stheno end of request.\n" );*/
    }
    print_debug("stheno_request_thread was terminated.\n");
#if defined( USE_KMALLOC )
    if( euryale_block != NULL ) kfree( euryale_block );
#endif
    return 0;
}
#endif

static void stheno_request( struct request_queue *q )
{
    /* caution : should be atomic procedure */
    stheno_wakeup = 1;
    wake_up_interruptible( &stheno_wait_q );
}

static int stheno_open( struct block_device *bdev, fmode_t fmode )
{
    print_debug( "stheno_open was called.\n" );
    mutex_lock( &stheno_openlock );

    if( stheno_usage == 0 ) stheno_read_sector0_flag = 1;
    ++stheno_usage;

    mutex_unlock( &stheno_openlock );
    return 0;
}

static int stheno_release( struct gendisk *gd, fmode_t fmode )
{
    print_debug( "stheno_release was called.\n" );
    mutex_lock( &stheno_openlock );

    --stheno_usage;
    if( stheno_usage < 0 ){
        print_error( "stheno reference counter is 0.\n" );
        stheno_usage = 0;
    }

    mutex_unlock( &stheno_openlock );
    return 0;
}

#if 0
static int stheno_media_changed( struct gendisk *gd )
{
    print_debug( "stheno_media_changed was called.\n" );
    return -1; /* not changed */
}

static int stheno_revalidate_disk( struct gendisk *gd )
{
    print_debug( "stheno_revalidate_disk was called.\n" );
    return 0;
}
#endif

static int stheno_getgeo( struct block_device *bdev, struct hd_geometry *geo )
{
    print_debug( "stheno_stheno_getgeo was called.\n" );

    memset( geo, 0, sizeof( struct hd_geometry ) );

    geo->heads = 128;
    geo->sectors = 63;
    geo->cylinders = AMOUNT_OF_SECTORS / geo->heads / geo->sectors;
    geo->start = 0;

    return 0;
}

/**
 * stheno_add_card - notify to insert card
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int stheno_add_card( void )
{
    struct kobject *kobj = NULL;
    char env_major[32];
    char *env[16];
    int ret;
    int retval = -1;

    print_info( "stheno sdcard inserted.\n" );

    if( stheno_gd == NULL ){
        print_error( "stheno is not allocated.\n" );
        goto exit;
    }

    /*stheno_read_sector0_flag = 1;*/

    kobj = get_disk( stheno_gd );
    if( kobj == NULL ) {
        print_error( "stheno get_disk failed.\n" );
        goto exit;
    }

    snprintf( env_major, sizeof( env_major ), "MAJOR=%d", stheno_major );
    env[0] = env_major;
    env[1] = "MINOR=0";
    env[2] = "DEVNAME="STHENO_NAME;
    env[3] = "DEVTYPE=disk";
    env[4] = "NPARTS=1";
    env[5] = NULL;
    ret = kobject_uevent_env( kobj, KOBJ_ADD, env );
    if( ret != 0 ){
        print_error( "stheno 1st kobject_uevent(KOBJ_ADD) failed.(ret=%d)\n", ret );
        /* ignore error because kobject_uevent_env() always returns -ENOENT(-2). */
        /*goto exit;*/
    }

    env[0] = env_major;
    env[1] = "MINOR=0";
    env[2] = "DEVNAME="STHENO_NAME;
    env[3] = "DEVTYPE=";
    env[4] = "PARTN=1";
    env[5] = NULL;
    ret = kobject_uevent_env( kobj, KOBJ_ADD, env );
    if( ret != 0 ){
        print_error( "stheno 2nd kobject_uevent(KOBJ_ADD) failed.(ret=%d)\n", ret );
        /* ignore error because kobject_uevent_env() always returns -ENOENT(-2). */
        /*goto exit;*/
    }
    retval = 0;
exit:
    if( kobj != NULL ) put_disk( stheno_gd );
    return retval;
}

/**
 * stheno_remove_card - notify to remove card
 *
 * Return: 0 if successful, non 0 otherwise.
 */
int stheno_remove_card( void )
{
    struct kobject *kobj = NULL;
    static char env_major[32];
    char *env[16];
    int ret;
    int retval = -1;

    print_info( "stheno sdcard removed.\n" );

    if( stheno_gd == NULL ){
        print_error( "stheno is not allocated.\n" );
        goto exit;
    }

    kobj = get_disk( stheno_gd );
    if( kobj == NULL ){
        print_error( "stheno get_disk failed.\n" );
        goto exit;
    }

    snprintf( env_major, sizeof( env_major ), "MAJOR=%d", stheno_major );
    env[0] = env_major;
    env[1] = "MINOR=0";
    env[2] = "DEVNAME="STHENO_NAME;
    env[3] = "DEVTYPE=";
    env[4] = "PARTN=1";
    env[5] = NULL;
    ret = kobject_uevent_env( kobj, KOBJ_REMOVE, env );
    if( ret != 0 ){
        print_error( "stheno 1st kobject_uevent(KOBJ_REMOVE) failed.(ret=%d)\n", ret );
        /* ignore error because kobject_uevent_env() always returns -ENOENT(-2). */
        /*goto exit;*/
    }

    env[0] = env_major;
    env[1] = "MINOR=0";
    env[2] = "DEVNAME="STHENO_NAME;
    env[3] = "DEVTYPE=disk";
    env[4] = "NPARTS=1";
    env[5] = NULL;
    ret = kobject_uevent_env( kobj, KOBJ_REMOVE, env );
    if( ret != 0 ){
        print_error( "stheno 2nd kobject_uevent(KOBJ_REMOVE) failed.(ret=%d)\n", ret );
        /* ignore error because kobject_uevent_env() always returns -ENOENT(-2). */
        /*goto exit;*/
    }
    retval = 0;
exit:
    if( kobj != NULL ) put_disk( stheno_gd );
    return retval;
}

static int __init stheno_module_init( void )
{
    int retval;

    print_info( "stheno_module_init was called.\n" );

    init_waitqueue_head( &stheno_wait_q );

    wake_lock_init( &stheno_wakelock, WAKE_LOCK_SUSPEND, STHENO_NAME );

    stheno_major = register_blkdev( 0, STHENO_NAME );
    if( stheno_major <= 0 ){
        print_error( "stheno register_blkdev failed.\n" );
        retval = -EBUSY;
        goto error;
    }

    spin_lock_init( &stheno_lock );
    stheno_queue = blk_init_queue( stheno_request, &stheno_lock );
    if( stheno_queue == NULL ){
        print_error( "stheno blk_init_queue failed.\n" );
        retval = -ENOMEM;
        goto error;
    }

    /*blk_queue_hardsect_size( stheno_queue, SECTOR_SIZE );*/
    /*blk_queue_max_sectors( stheno_queue, MAX_SECTORS );*/
    blk_queue_logical_block_size( stheno_queue, SECTOR_SIZE );
    blk_queue_max_hw_sectors( stheno_queue, MAX_SECTORS );
#if defined( STHENO_BLK_BOUNCE_ANY )
    blk_queue_bounce_limit( stheno_queue, BLK_BOUNCE_ANY );
#else
    blk_queue_bounce_limit( stheno_queue, BLK_BOUNCE_HIGH ); /* default */
#endif

    stheno_gd = alloc_disk( STHENO_MINOR_COUNT );
    if( stheno_gd == NULL ){
        print_error( "stheno alloc_disk failed.\n" );
        retval = -ENOMEM;
        goto error;
    }

    stheno_gd->major = stheno_major;
    stheno_gd->first_minor = 0;
    stheno_gd->fops = &stheno_fops;
    stheno_gd->queue = stheno_queue;
    /*stheno_gd->flags = GENHD_FL_REMOVABLE;*/
    /*stheno_gd->private_data = NULL;*/
    snprintf( stheno_gd->disk_name, DISK_NAME_LEN, "%s", STHENO_NAME );
    set_capacity( stheno_gd, AMOUNT_OF_SECTORS );

    stheno_thread = kthread_create( stheno_request_thread, 0, STHENO_THREAD_NAME );
    if( IS_ERR( stheno_thread ) ){
        print_error( "stheno kthread_create failed.\n" );
        retval = -EBUSY;
        goto error;
    }
    wake_up_process( stheno_thread );

    add_disk( stheno_gd );

    print_debug( "stheno major = %d\n", stheno_major );
    return 0;
error:
    if( stheno_gd != NULL ) del_gendisk( stheno_gd );
    if( stheno_queue != NULL ) blk_cleanup_queue( stheno_queue );
    if( stheno_major > 0 ) unregister_blkdev( stheno_major, STHENO_NAME );
    return retval;
}

static void __exit stheno_module_exit( void )
{
    print_info( "stheno_module_exit was called.\n" );

    del_gendisk( stheno_gd );
    blk_cleanup_queue( stheno_queue );
    unregister_blkdev( stheno_major, STHENO_NAME );

    kthread_stop( stheno_thread );

    wake_lock_destroy( &stheno_wakelock );
}

module_init( stheno_module_init );
module_exit( stheno_module_exit );

MODULE_DESCRIPTION("SHARP STHENO MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
