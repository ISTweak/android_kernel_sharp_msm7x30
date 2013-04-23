/* drivers/sharp/shdiag/smd_read.c
 *
 * Copyright (C) 2010 Sharp Corporation
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
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>

#include <sharp/sh_smem.h>

#include <sharp/shdiag_smd.h>

/* Soft Update Flag */
#define D_SHSOFTUP_F_MASK	0x00000010		/* bit4 */
#define D_SHSOFTUP_F_SHIFT	4

static int smd_mode_open(struct inode *inode, struct file *filp)
{
/*	printk("%s\n", __func__);*/
	return 0;
}


static ssize_t smd_mode_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	struct smem_comm_mode  smem_comm_data;
	unsigned long UpDateFlgStatus;

/*	printk("%s\n", __func__);*/

	p_sh_smem_common_type = sh_smem_get_common_address();
	if( p_sh_smem_common_type != NULL){
		smem_comm_data.BootMode  = p_sh_smem_common_type->sh_boot_mode;

		UpDateFlgStatus = p_sh_smem_common_type->shdiag_FlagData;
		smem_comm_data.UpDateFlg = ( UpDateFlgStatus & D_SHSOFTUP_F_MASK )>>D_SHSOFTUP_F_SHIFT;

		/* user aera */
		if( copy_to_user( buf, (void *)&smem_comm_data, sizeof(smem_comm_data) ) ){
			printk( "copy_to_user failed\n" );
			return -EFAULT;
		}
	} else {
		printk("[SH]smd_read_probe: smem_alloc FAILE\n");
	}
	return count;
}

static ssize_t smd_mode_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
/*	printk("%s\n", __func__);*/
	return count;
}

static int smd_mode_release(struct inode *inode, struct file *filp)
{
/*	printk("%s\n", __func__);*/
	return 0;
}

static struct file_operations smd_mode_fops = {
	.owner		= THIS_MODULE,
	.read		= smd_mode_read,
	.write		= smd_mode_write,
	.open		= smd_mode_open,
	.release	= smd_mode_release,
};

static struct miscdevice smd_mode_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smd_read",
	.fops = &smd_mode_fops,
};

static int __init smd_mode_init( void )
{
	int ret;

	ret = misc_register(&smd_mode_dev);
	if (ret) {
		printk("fail to misc_register (smd_mode_dev)\n");
		return ret;
	}
	printk("smd_mode loaded.\n");
	return 0;
}

module_init(smd_mode_init);

MODULE_DESCRIPTION("smd_read");
MODULE_LICENSE("GPL v2");

