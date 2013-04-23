/* drivers/sharp/shrmts/shrmts.c
 *
 * Copyright (C) 2011 Sharp Corporation
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

/* -------------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h> 
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <mach/msm_rpcrouter.h>
#include "sharp/sh_oncrpc_id.h"

#include <sharp/sh_smem.h>
/* -------------------------------------------------------------------- */
#define SHRMTS_KER_BASEMINOR	(0)
#define SHRMTS_KER_MINORCOUNT	(1)
#define SHRMTS_KER_DRVNAME		"shrmts"
#define SHRMTS_CLASS_NAME		"cls_shrmts"
#define SHRMTS_SECT_SIZE 		(512)
#define SHRMTS_SECT_PER_CHUNK 	(32)
#define shrmts_printk			if(0)printk

#define SHRMTS_SRV_STARTED	"SHRMTS_INITIALIZED"

/* -------------------------------------------------------------------- */
typedef struct{
	int major;
	dev_t dev;
	struct cdev shrmts_cdev;
	struct class* shrmts_classp;
	int status;
}shrmts_info_type;
shrmts_info_type shrmts_info;
/* -------------------------------------------------------------------- */
typedef struct _shrmts_command
{
	int cmd;
	int inited;
	int ret;
	unsigned long sector;
	unsigned long nr;
	char buffer[SHRMTS_SECT_SIZE * SHRMTS_SECT_PER_CHUNK];
}shrmts_command;
/* -------------------------------------------------------------------- */
static int		shrmts_open(struct inode* inode, struct file* filp);
static int		shrmts_close(struct inode* inode, struct file* filp);
static ssize_t	shrmts_read(struct file* filp, char* buf, size_t count, loff_t* pos);
static ssize_t	shrmts_write(struct file* filp, const char* buf, size_t count, loff_t* pos);
/* -------------------------------------------------------------------- */
static struct file_operations shrmts_Ops = {
	.owner   = THIS_MODULE,
	.read    = shrmts_read,
	.write   = shrmts_write,
	.open    = shrmts_open,
	.release = shrmts_close,
};
/* -------------------------------------------------------------------- */
static struct semaphore 	shrmts_sem;
static wait_queue_head_t	shrmts_process_q;
static wait_queue_head_t	shrmts_read_q;
static wait_queue_head_t	shrmts_write_q;
static shrmts_command 		shrmts_c;
static volatile int		shrmts_state = 0;
/* -------------------------------------------------------------------- */
static sharp_smem_common_type *p_sh_smem_common = NULL;

#define SHRMTS_READ_PROC	2
#define SHRMTS_WRITE_PROC	3

int shrmts_write_process(unsigned long sector, unsigned long current_nr_sectors, char* buffer)
{
	int ret;
	unsigned long i;

	shrmts_printk("[0] : shrmts_write_process start %d , %d\n", (int)sector, (int)current_nr_sectors);

	for(i = 0; i < current_nr_sectors; i += SHRMTS_SECT_PER_CHUNK)
	{
		shrmts_printk("[0] : shrmts_write_process %d < %d\n", (int)i, (int)current_nr_sectors);

		if(down_interruptible(&shrmts_sem))return -1;

		memset(&shrmts_c, 0, sizeof(shrmts_command));

		shrmts_c.inited = 1;
		shrmts_c.cmd = 0;
		shrmts_c.sector = sector + i;
		
		if(i + SHRMTS_SECT_PER_CHUNK <= current_nr_sectors)
		{
			shrmts_c.nr = SHRMTS_SECT_PER_CHUNK;
		}
		else
		{
			shrmts_c.nr = current_nr_sectors - i;
		}
		
		memcpy(shrmts_c.buffer, &buffer[i * SHRMTS_SECT_SIZE], shrmts_c.nr * SHRMTS_SECT_SIZE);

		shrmts_state = 1;

		up(&shrmts_sem);

		shrmts_printk("[0] : wake_up_interruptible(&shrmts_read_q);\n");

		wake_up_interruptible(&shrmts_read_q);

		shrmts_printk("[0] : wait_event_interruptible(shrmts_process_q, shrmts_state == 3);\n");

		wait_event_interruptible(shrmts_process_q, shrmts_state == 3);

		shrmts_printk("[5] : awake\n");

		if(down_interruptible(&shrmts_sem))return -1;

		ret = shrmts_c.ret;
		shrmts_state = 0;

		up(&shrmts_sem);

		if(ret != 0) {
			shrmts_printk("[E] %s: failed to write process. ret=%d, i=%d\n",__FUNCTION__,ret, (int)i);
			return -1;
		}
	}

	shrmts_printk("[6] : shrmts_write_process end\n");

	return 0;
}
/* -------------------------------------------------------------------- */
int shrmts_read_process(unsigned long sector, unsigned long current_nr_sectors, char* buffer)
{
	int ret;	
	unsigned long i;

	shrmts_printk("[1] : shrmts_read_process start %d , %d\n", (int)sector, (int)current_nr_sectors);

	for(i = 0; i < current_nr_sectors; i += SHRMTS_SECT_PER_CHUNK)
	{
		shrmts_printk("[1] : shrmts_read_process %d < %d\n", (int)i, (int)current_nr_sectors);

		if(down_interruptible(&shrmts_sem))return -1;

		memset(&shrmts_c, 0, sizeof(shrmts_command));

		shrmts_c.cmd = 1;
		shrmts_c.sector = sector + i;
		
		if(i + SHRMTS_SECT_PER_CHUNK <= current_nr_sectors)
		{
			shrmts_c.nr = SHRMTS_SECT_PER_CHUNK;
		}
		else
		{
			shrmts_c.nr = current_nr_sectors - i;
		}

		shrmts_state = 1;

		up(&shrmts_sem);

		shrmts_printk("[1] : wake_up_interruptible(&shrmts_poll_q);\n");

		wake_up_interruptible(&shrmts_read_q);

		shrmts_printk("[1] : wait_event_interruptible(shrmts_process_q, shrmts_state == 3);\n");

		wait_event_interruptible(shrmts_process_q, shrmts_state == 3);

		shrmts_printk("[7] : awake\n");

		if(down_interruptible(&shrmts_sem))return -1;

		memcpy(&buffer[i * SHRMTS_SECT_SIZE], shrmts_c.buffer, shrmts_c.nr * SHRMTS_SECT_SIZE);
		
		ret = shrmts_c.ret;
		shrmts_state = 0;

		up(&shrmts_sem);

		if(ret != 0) {
			shrmts_printk("[E] %s: failed to read process. ret=%d, i=%d\n",__FUNCTION__,ret, (int)i);
			return -1;
		}
	}

	shrmts_printk("[8] : shrmts_read_process end\n");

	return 0;
}


static int shrmts_handle_rpc_call( struct msm_rpc_server* svr_p,
                                       struct rpc_request_hdr* req_p,
                                       unsigned len )
{
	int result = 0;
	int ret = 0;
	char *buf = NULL;
	unsigned long start_sector;
	unsigned long sector_num;

	shrmts_printk("[S] %s: %d\n",__FUNCTION__,req_p->procedure);

	switch(req_p->procedure)
	{
		case SHRMTS_READ_PROC:
		case SHRMTS_WRITE_PROC:
		{
			if (p_sh_smem_common == NULL)
			{
				shrmts_printk("[E] %s: failed to get smem address.\n",__FUNCTION__);
				result = -1;
				break;
			}
			
			memcpy(&start_sector, &(p_sh_smem_common->shrmts_data_buf[18]), 4);
			memcpy(&sector_num, &(p_sh_smem_common->shrmts_data_buf[22]), 4);
			buf = &(p_sh_smem_common->shrmts_data_buf[30]);
			
			if (req_p->procedure == SHRMTS_READ_PROC)
			{
				ret = shrmts_read_process(start_sector, sector_num, buf);
			}
			else
			{
				ret = shrmts_write_process(start_sector, sector_num, buf);
			}

			if (ret != 0) result = -1;
			
			break;
		}

		default:
			result = -1;
			break;
	}

	memcpy(&(p_sh_smem_common->shrmts_data_buf[26]), &result, 4);
	
	shrmts_printk("[E] %s proc=%d result=%d ret=%d\n",__FUNCTION__,req_p->procedure, result, ret);

	return result;
}

static void shrmts_shutdown(struct platform_device *pdev)
{
	memset(p_sh_smem_common->shrmts_data_buf, 0, 18);
}

static struct msm_rpc_server shrmts_rpc_server = 
{
	.prog     = SHRMTS_REMOTE_M2APROG,
	.vers     = SHRMTS_REMOTE_M2AVERS,
	.rpc_call = shrmts_handle_rpc_call,
};

static struct platform_device shrmts_device = {
	.name = "shrmts",
	.id   = -1,
};

static struct platform_driver shrmts_driver = 
{
  .shutdown  = shrmts_shutdown,
  .driver = { .name = "shrmts", .owner = THIS_MODULE, },
};

static int __init shrmts_init( void )
{
	int sdResult;
	struct device* devp;

	shrmts_printk("[S] %s\n",__FUNCTION__);

	if (msm_rpc_create_server(&shrmts_rpc_server) < 0)
	{
		shrmts_printk("[E] %s: failed to create server.\n",__FUNCTION__);
		return -1;
	}

	if(platform_device_register(&shrmts_device))
	{
		shrmts_printk("[E] %s: failed to register device.\n", __FUNCTION__);
		return -1;
	}

	if(platform_driver_register(&shrmts_driver))
	{
		platform_driver_unregister(&shrmts_driver);
		shrmts_printk("[E] %s: failed to register driver.\n", __FUNCTION__);
		return -1;
	}

	shrmts_info.major = -1;
	shrmts_info.shrmts_classp = NULL;
	shrmts_info.status = 0;
	
	shrmts_info.dev = MKDEV(shrmts_info.major, 0);
	
	sdResult = alloc_chrdev_region( &shrmts_info.dev, SHRMTS_KER_BASEMINOR, SHRMTS_KER_MINORCOUNT, SHRMTS_KER_DRVNAME );
	if( sdResult < 0 ){
		shrmts_printk("[E] %s: failed to alloc region\n",__FUNCTION__);
		return -1;
	}
	shrmts_info.major = sdResult;
	

	cdev_init( &shrmts_info.shrmts_cdev, &shrmts_Ops );
	shrmts_info.shrmts_cdev.owner = THIS_MODULE;
	shrmts_info.shrmts_cdev.ops = &shrmts_Ops;

	sdResult = cdev_add(&shrmts_info.shrmts_cdev, shrmts_info.dev, SHRMTS_KER_MINORCOUNT);
	if( sdResult < 0 ){
		shrmts_printk("[E] %s: failed to add device\n",__FUNCTION__);
		return -1;
	}

	
	shrmts_info.shrmts_classp = class_create( THIS_MODULE, SHRMTS_CLASS_NAME );
	if (IS_ERR(shrmts_info.shrmts_classp)){
		shrmts_printk("[E] %s: failed to create class\n",__FUNCTION__);
		return -1;
	}

	devp = device_create( shrmts_info.shrmts_classp, NULL, shrmts_info.dev, NULL, SHRMTS_KER_DRVNAME );
	sdResult = IS_ERR(devp) ? PTR_ERR(devp) : 0;
	if ( sdResult < 0 ){
		shrmts_printk("[E] %s: failed to create device\n",__FUNCTION__);
		return -1;
	}
	
	sema_init(&shrmts_sem, 1);

	init_waitqueue_head(&shrmts_process_q);
	init_waitqueue_head(&shrmts_read_q);
	init_waitqueue_head(&shrmts_write_q);

	p_sh_smem_common = sh_smem_get_common_address();
	if (p_sh_smem_common == NULL)
	{
		shrmts_printk("[E] %s: failed to get smem address.\n",__FUNCTION__);
		return -1;
	}

	shrmts_printk("[E] %s: success.\n",__FUNCTION__);

	return 0;
}

static void __exit shrmts_term( void )
{
	if( shrmts_info.major < 0){
		return;
	}
	
	device_destroy( shrmts_info.shrmts_classp, shrmts_info.dev );
	class_destroy( shrmts_info.shrmts_classp );
	shrmts_info.shrmts_classp = NULL;

	cdev_del( &shrmts_info.shrmts_cdev );
	unregister_chrdev_region( shrmts_info.dev, SHRMTS_KER_MINORCOUNT );
	shrmts_info.major = -1;

	return;
}
/* -------------------------------------------------------------------- */
/*  */
/* -------------------------------------------------------------------- */
static int shrmts_open(struct inode* inode, struct file* filp)
{
	memcpy(p_sh_smem_common->shrmts_data_buf, SHRMTS_SRV_STARTED, 18);

	return 0;
}
/* -------------------------------------------------------------------- */
static int shrmts_close(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static ssize_t shrmts_read(struct file* filp, char* buf, size_t count, loff_t* pos)
{
	ssize_t ret = 0;
	int r;

	shrmts_printk("[3] : shrmts_read\n");

	r = wait_event_interruptible(shrmts_read_q, shrmts_state == 1);

	if(r != 0)
	{
		return 0;
	}

	shrmts_printk("[3] : awake\n");
	shrmts_printk("[3] : %02x%02x%02x%02x%02x%02x%02x%02x\n", shrmts_c.buffer[0], shrmts_c.buffer[1], shrmts_c.buffer[2], shrmts_c.buffer[3], shrmts_c.buffer[4], shrmts_c.buffer[5], shrmts_c.buffer[6], shrmts_c.buffer[7]);

	do
	{
		if(down_interruptible(&shrmts_sem))return 0;

		if(copy_to_user(buf, &shrmts_c, sizeof(shrmts_command)))break;

		ret = sizeof(shrmts_command);
	}
	while(0);

	shrmts_state = 2;

	up(&shrmts_sem);

	shrmts_printk("[3] : wake_up_interruptible(&shrmts_write_q);\n");

	wake_up_interruptible(&shrmts_write_q);

	return ret;
}
/* -------------------------------------------------------------------- */
static ssize_t shrmts_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	ssize_t ret = 0;

	shrmts_printk("[4] : shrmts_write\n");

	wait_event_interruptible(shrmts_write_q, shrmts_state == 2);

	shrmts_printk("[4] : awake\n");

	do
	{
		if(down_interruptible(&shrmts_sem))return 0;

		if(copy_from_user(&shrmts_c, buf, sizeof(shrmts_command)))break;

		ret = sizeof(shrmts_command);
	}
	while(0);

	shrmts_state = 3;

	up(&shrmts_sem);

	shrmts_printk("[4] : wake_up_interruptible(&shrmts_rw_q);\n");

	wake_up_interruptible(&shrmts_process_q);

	return ret;
}
/* -------------------------------------------------------------------- */

module_init(shrmts_init);
module_exit(shrmts_term);

MODULE_DESCRIPTION("SH RMTS Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

