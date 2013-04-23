/* include/sharp/shswic_kerl.c
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

#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/err.h>
#include <asm/uaccess.h>
#include <sharp/sh_oncrpc_id.h>
#include <mach/msm_rpcrouter.h>

#include <sharp/shswic_kerl.h>

#if defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR )
#include <sharp/shtps_dev.h>
#endif	/* defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR ) */

#define SHSWIC_LOG_TAG "SHSWICkerl"
#define  SHSWIC_DEBUG_LOG_ENABLE_1
//#define  SHSWIC_DEBUG_LOG_ENABLE_2

#ifdef SHSWIC_DEBUG_LOG_ENABLE_1
#define SHSWIC_DEBUG_LOG_1(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHSWIC_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHSWIC_DEBUG_LOG_1(fmt, args...)
#endif

#ifdef SHSWIC_DEBUG_LOG_ENABLE_2
#define SHSWIC_DEBUG_LOG_2(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHSWIC_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHSWIC_DEBUG_LOG_2(fmt, args...)
#endif

#define SHSWIC_REMOTE_GET_ID_STATUS_PROC		3
#define SHSWIC_REMOTE_DIAG_GET_ID_PROC			4
#define SHSWIC_REMOTE_DIAG_GET_DETECT_ID_PROC	5

#define SHSWIC_ID_CB_PROC 1

#define SHSWIC_ID_MASK 0xFFFFFFC0

static void shswic_detect_cb_call(uint32_t detect);
static shswic_result_t shswic_diag_get_id_data( uint8_t* id_data );

typedef void (*shswic_cb_func_t)(uint8_t device, void* user_data);

struct rpc_shswic_data_args
{
	uint32_t detect;
};

static struct msm_rpc_endpoint* ep_shswic_p = NULL;
static shswic_cb_func_t shswic_vbus_cb_func = NULL;
static uint32_t vbus_cb_irq = 0;
static void* shswic_userdata_vbus;
static uint32_t shswic_last_detect = SHSWIC_ID_NONE;

shswic_result_t shswic_detect_cb_regist( uint8_t cb_dev, uint32_t cb_irq, void* cb_func, void* user_data )
{
	shswic_result_t ret = SHSWIC_SUCCESS;
	
	SHSWIC_DEBUG_LOG_2();
	
	if((cb_irq & SHSWIC_ID_MASK) != 0)
	{
		SHSWIC_DEBUG_LOG_1("param error");
		return SHSWIC_PARAM_ERROR;
	}
	
	switch(cb_dev)
	{
		case SHSWIC_VBUS_DEVICE:
			shswic_vbus_cb_func = (shswic_cb_func_t)cb_func;
			vbus_cb_irq = cb_irq;
			shswic_userdata_vbus = user_data;
			break;
			
		default:
			SHSWIC_DEBUG_LOG_1("param error");
			ret =  SHSWIC_PARAM_ERROR;
			break;
	}
	
	return ret;
}

shswic_result_t shswic_get_usb_port_status( uint8_t* device )
{
	int rc = 0;
	struct shswic_data_req {
		struct rpc_request_hdr hdr;
	} req;
	struct shswic_data_rep {
		struct rpc_reply_hdr hdr;
		uint32_t receive_data;
	} rep;
	
	SHSWIC_DEBUG_LOG_2();
	
	if(ep_shswic_p == NULL)
	{
		SHSWIC_DEBUG_LOG_1("ep_shswic_p NULL");
		return SHSWIC_FAILURE;
	}
	
	if(IS_ERR(ep_shswic_p))
	{
		SHSWIC_DEBUG_LOG_1("rpc connect failed");
		return SHSWIC_FAILURE;
		
	}
	
	rc = msm_rpc_call_reply(ep_shswic_p, SHSWIC_REMOTE_GET_ID_STATUS_PROC, 
														&req, sizeof(req), 
														&rep, sizeof(rep),
														5 * HZ);
	if (rc < 0) 
	{
		SHSWIC_DEBUG_LOG_1("shswic_get_id_status ERROR : %d", rc);
		return SHSWIC_FAILURE;
	}
	
	shswic_last_detect = be32_to_cpu(rep.receive_data);
	*device = (uint8_t)shswic_last_detect;
	
	SHSWIC_DEBUG_LOG_2("A->M rpc data = %d", *device);
	
	return SHSWIC_SUCCESS;

}

static shswic_result_t shswic_diag_get_id_data( uint8_t* id_data )
{
	int rc = 0;
	struct shswic_data_req {
		struct rpc_request_hdr hdr;
	} req;
	struct shswic_data_rep {
		struct rpc_reply_hdr hdr;
		uint32_t receive_data;
	} rep;
	
	SHSWIC_DEBUG_LOG_2();
	
	if(ep_shswic_p == NULL)
	{
		SHSWIC_DEBUG_LOG_1("ep_shswic_p NULL");
		return SHSWIC_FAILURE;
	}
	
	if(IS_ERR(ep_shswic_p))
	{
		SHSWIC_DEBUG_LOG_1("rpc connect failed");
		return SHSWIC_FAILURE;
	}
	
	rc = msm_rpc_call_reply(ep_shswic_p, SHSWIC_REMOTE_DIAG_GET_ID_PROC, 
														&req, sizeof(req), 
														&rep, sizeof(rep),
														5 * HZ);
	if (rc < 0) 
	{
		SHSWIC_DEBUG_LOG_1("shswic_diag_get_id ERROR : %d", rc);
		return SHSWIC_FAILURE;
	}
	
	*id_data = (uint8_t)be32_to_cpu(rep.receive_data);
	
	SHSWIC_DEBUG_LOG_2("A->M rpc data = %d", *id_data);
	
	return SHSWIC_SUCCESS;

}

static shswic_result_t shswic_diag_get_detect_id_data( uint8_t* detect_id )
{
	int rc = 0;
	struct shswic_data_req {
		struct rpc_request_hdr hdr;
	} req;
	struct shswic_data_rep {
		struct rpc_reply_hdr hdr;
		uint32_t receive_data;
	} rep;
	
	SHSWIC_DEBUG_LOG_2();
	
	if(ep_shswic_p == NULL)
	{
		SHSWIC_DEBUG_LOG_1("ep_shswic_p NULL");
		return SHSWIC_FAILURE;
	}
	
	if(IS_ERR(ep_shswic_p))
	{
		SHSWIC_DEBUG_LOG_1("rpc connect failed");
		return SHSWIC_FAILURE;
	}
	
	rc = msm_rpc_call_reply(ep_shswic_p, SHSWIC_REMOTE_DIAG_GET_DETECT_ID_PROC, 
														&req, sizeof(req), 
														&rep, sizeof(rep),
														5 * HZ);
	if (rc < 0) 
	{
		SHSWIC_DEBUG_LOG_1("shswic_diag_get_detect_id_data ERROR : %d", rc);
		return SHSWIC_FAILURE;
	}
	
	*detect_id = (uint8_t)be32_to_cpu(rep.receive_data);
	
	SHSWIC_DEBUG_LOG_2("A->M rpc data = %d", *detect_id);
	
	return SHSWIC_SUCCESS;

}

static int shswic_handle_rpc_call( struct msm_rpc_server* svr_p,
                                   struct rpc_request_hdr* req_p,
                                   unsigned len )
{
	int result = 0;
	
	struct rpc_shswic_data_args* args;
	
	SHSWIC_DEBUG_LOG_2();

	switch(req_p->procedure)
	{
		case SHSWIC_ID_CB_PROC:
			args = (struct rpc_shswic_data_args *)(req_p + 1);
			args->detect = be32_to_cpu(args->detect);
			SHSWIC_DEBUG_LOG_2("M->A rpc detect = %d\n", args->detect);
			shswic_detect_cb_call(args->detect);
			break;
			
		default:
			result = -1;
			SHSWIC_DEBUG_LOG_1("shswic_handle_rpc_call default %d", req_p->procedure);
			break;
	}

	return result;
}

static void shswic_detect_cb_call(uint32_t detect)
{
	SHSWIC_DEBUG_LOG_2("detect = %d", detect);
	
	if(detect == SHSWIC_ID_USB_CABLE)
	{
		if(shswic_vbus_cb_func != NULL)
		{
			if((vbus_cb_irq & SHSWIC_ID_USB_CABLE) != 0)
			{
				shswic_vbus_cb_func((uint8_t)detect, shswic_userdata_vbus);
			}
		}
#if defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR )
		msm_tps_set_chargerarmor(1);
#endif	/* defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR ) */
	}
	else if(detect == SHSWIC_ID_AC_ADAPTER)
	{
		if(shswic_vbus_cb_func != NULL)
		{
			if((vbus_cb_irq & SHSWIC_ID_AC_ADAPTER) != 0)
			{
				shswic_vbus_cb_func((uint8_t)detect, shswic_userdata_vbus);
			}
		}
#if defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR )
		msm_tps_set_chargerarmor(1);
#endif	/* defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR ) */
	}
	else if(detect == SHSWIC_ID_UNKNOWN)
	{
		if(shswic_vbus_cb_func != NULL)
		{
			if((vbus_cb_irq & SHSWIC_ID_UNKNOWN) != 0)
			{
				shswic_vbus_cb_func((uint8_t)detect, shswic_userdata_vbus);
			}
		}
	}
	else if(detect == SHSWIC_ID_NONE)
	{
		if(shswic_vbus_cb_func != NULL)
		{
			if((vbus_cb_irq & SHSWIC_ID_USB_CABLE) != 0)
			{
				if(shswic_last_detect == SHSWIC_ID_USB_CABLE)
				{
					shswic_vbus_cb_func((uint8_t)detect, shswic_userdata_vbus);
				}
			}
			if((vbus_cb_irq & SHSWIC_ID_AC_ADAPTER) != 0)
			{
				if(shswic_last_detect == SHSWIC_ID_AC_ADAPTER)
				{
					shswic_vbus_cb_func((uint8_t)detect, shswic_userdata_vbus);
				}
			}
			if((vbus_cb_irq & SHSWIC_ID_UNKNOWN) != 0)
			{
				if(shswic_last_detect == SHSWIC_ID_UNKNOWN)
				{
					shswic_vbus_cb_func((uint8_t)detect, shswic_userdata_vbus);
				}
			}
		}
#if defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR )
		msm_tps_set_chargerarmor(0);
#endif	/* defined( CONFIG_SHTPS_TMA3XX_CHARGER_ARMOR ) */
	}
	
	shswic_last_detect = detect;
}

static int shswic_diag_open(struct inode *inode, struct file *file)
{
	SHSWIC_DEBUG_LOG_2();
	return 0;
}

static int shswic_diag_release(struct inode *inode, struct file *file)
{
	SHSWIC_DEBUG_LOG_2();
	return 0;
}

static ssize_t shswic_diag_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t id = 0x00;
	
	SHSWIC_DEBUG_LOG_2();
	
	shswic_diag_get_id_data(&id);
	
	if(id != 0x00)
	{
		if (copy_to_user(buf, &id, sizeof(id)))
		{
			SHSWIC_DEBUG_LOG_1("copy_to_user Error");
			return -EFAULT;
		}
	}
	else
	{
		return 0;
	}
	
	return 1;
}

static int shswic_diag_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	uint8_t detect_id;

	SHSWIC_DEBUG_LOG_2();

	if(argp == NULL)
	{
		SHSWIC_DEBUG_LOG_1("SHSWIC_PARAM_ERROR");
		return -EIO;
	}

	switch(cmd)
	{
		case SHSWIC_IOCTL_ID_READ:
			SHSWIC_DEBUG_LOG_2("SHSWIC_IOCTL_ID_READ");
			if(shswic_diag_get_detect_id_data(&detect_id) != SHSWIC_SUCCESS)
			{
				return -EIO;
			}
			break;
		default:
			break;
	}

	switch(cmd)
	{
		case SHSWIC_IOCTL_ID_READ:
			if (copy_to_user(argp, &detect_id, sizeof(detect_id)))
			{
				SHSWIC_DEBUG_LOG_1("copy_to_user Error");
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	return 0;
}

static struct msm_rpc_server shswic_rpc_server = 
{
	.prog     = SHSWIC_REMOTE_M2APROG,
	.vers     = SHSWIC_REMOTE_M2AVERS,
	.rpc_call = shswic_handle_rpc_call,
};

static struct file_operations shswic_diag_fops = {
	.owner = THIS_MODULE,
	.open    = shswic_diag_open,
	.release = shswic_diag_release,
	.read    = shswic_diag_read,
	.ioctl   = shswic_diag_ioctl,
};

static struct miscdevice shswic_diag_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "shswic_diag",
	.fops = &shswic_diag_fops,
};

static int __init shswic_init( void )
{
	int err = -ENODEV;
	
	msm_rpc_create_server(&shswic_rpc_server);
	
	err = misc_register(&shswic_diag_device);
	if (err)
	{
		misc_deregister(&shswic_diag_device);
		SHSWIC_DEBUG_LOG_1("shswic_diag_device: register failed\n");
	}
	
	ep_shswic_p = msm_rpc_connect(SHSWIC_REMOTE_A2MPROG, SHSWIC_REMOTE_A2MVERS, 0);
	if (IS_ERR(ep_shswic_p))
	{
		SHSWIC_DEBUG_LOG_1("rpc connect failed");
		return -1;
	}
	return 0;
}

module_init(shswic_init);

MODULE_DESCRIPTION("SH Swic Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
