/* drivers/sharp/shcamsensor/tools/sh_camera_tools.c (Camera Driver)
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <mach/camera.h>
#include <sharp/sh_smem.h>

#include "sh_camera_tools.h"

#define AF_TABLE_DEFAULT        0
#define AF_TABLE_USER           1

static int shcam_get_sensor_camcon(struct msm_sync *sync);
static int shcam_get_sensor_data(struct msm_sync *sync, void __user *argp);
static int shcam_set_sensor_data(struct msm_sync *sync, void __user *argp);
static int shcam_get_sensor_otp(struct msm_sync *sync, void __user *argp);
static int shcam_ctrl_power(struct msm_sync *sync, void __user *argp);
static int shcam_get_smem_otp(void __user *argp);
static int shcam_set_af_table(struct msm_sync *sync, void __user *argp);

static int shcam_get_sensor_camcon(struct msm_sync *sync)
{
    int ret = 0;

    ret = sync->sctrl.s_camcon();

    return ret;
}

static int shcam_get_sensor_data(struct msm_sync *sync, void __user *argp)
{
    unsigned short temp = 0;
    unsigned short addr = 0;
    int ret = 1;

    struct sh_sensor_dspr {
        unsigned short  addr;
        unsigned short* data;
    } *pData = (struct sh_sensor_dspr*)argp;

    if(copy_from_user((void*)&addr, (void*)&pData->addr, sizeof(short))) {
        printk(KERN_ERR "[%d]:copy_from_user()\n", __LINE__);
        ret = 0;
    }
    else {
        ret = sync->sctrl.s_reg_read(addr, &temp);

        if(copy_to_user((void*)pData->data, (void*)&temp, sizeof(short))) {
            printk(KERN_ERR "[%d]:copy_to_user()\n", __LINE__);
            ret = 0;
        }
    }

    return ret;
}

static int shcam_set_sensor_data(struct msm_sync *sync, void __user *argp)
{
    int ret;

    struct sh_sensor_dspw {
        unsigned short  addr;
        unsigned short  data;
    } udata;

    if(copy_from_user((void*)&udata, argp, sizeof(struct sh_sensor_dspw))) {
        printk(KERN_ERR "[%d]:copy_from_user()\n", __LINE__);
        ret = 0;
    }
    else {
        ret = sync->sctrl.s_reg_write(udata.addr, udata.data);
    }

    return ret;
}

static int shcam_get_sensor_otp(struct msm_sync *sync, void __user *argp)
{
    int ret = 0;
    short len = 0;
    unsigned char* temp = NULL;
    struct sh_sensor_otp udata;
    struct sh_sensor_otp* pData = (struct sh_sensor_otp*)argp;

    if (copy_from_user((void*)&udata, argp, sizeof(struct sh_sensor_otp))) {
        printk(KERN_ERR "[%d]:copy_from_user()\n", __LINE__);
        ret = 0;
    }
    else {
        len = udata.len;
        temp = kmalloc(len, GFP_ATOMIC);
        if(NULL != temp) {
            ret = sync->sctrl.s_otp_read(udata.page, udata.offset, len, temp);

            if (copy_to_user((void*)pData->buf, (void*)&temp[0], len)) {
                printk(KERN_ERR "[%d]:copy_to_user()\n", __LINE__);
                ret = 0;
            }
            kfree(temp);
        }
    }

    return ret;
}

static int shcam_ctrl_power(struct msm_sync *sync, void __user *argp)
{
    struct platform_device *pdev = sync->pdev;
    int8_t power = 0;
    int ret = 1;

    if (copy_from_user((void*)&power, argp, sizeof(int8_t))) {
        printk(KERN_ERR "[%d]:copy_from_user()\n", __LINE__);
        ret = 0;
    }
    else {
        ret = sync->sctrl.s_power_ctrl(pdev, power);
    }

    return ret;
}

static int shcam_set_af_table(struct msm_sync *sync, void __user *argp)
{
    int ret = 1;

    struct af_table_info {
        unsigned char  type;
        unsigned short start;
        unsigned short end;
        unsigned short step;
    } udata;

    if(copy_from_user((void*)&udata, argp, sizeof(struct af_table_info))) {
        printk(KERN_ERR "[%d]:copy_from_user()\n", __LINE__);
        ret = 0;
    }
    else {
        if(udata.type == AF_TABLE_USER) {
            ret = sync->sctrl.s_set_af_table(udata.start, udata.end, udata.step);
        }
        else if(udata.type == AF_TABLE_DEFAULT) {
            ret = sync->sctrl.s_reset_af_table();
        }
        else {
            ret = sync->sctrl.s_reset_af_table();
        }
    }

    return ret;
}

static int shcam_get_smem_otp(void __user *argp)
{
    int ret = 1;
    char* pData = (char*)argp;
    sharp_smem_common_type  *sh_smem_common = NULL;

    do {
        sh_smem_common = sh_smem_get_common_address();
        if(sh_smem_common != NULL) {
            if(copy_to_user((void*)pData,
                (void*)sh_smem_common->sh_camOtpData, 2048)) {
                printk(KERN_ERR "[%d]:copy_to_user()\n", __LINE__);
                ret = -EFAULT;
            }
        }
        else {
            ret = -EFAULT;
            break;
        }
    }while(0);

    return ret;
}


int sh_camera_ioctl(struct msm_sync *sync, void __user *argp)
{
    int ret = -EINVAL;
    struct sh_sensor_clrt pctrl;

    if (copy_from_user((void*)&pctrl, argp, sizeof(struct sh_sensor_clrt))) {
        printk(KERN_ERR "[%d]:copy_from_user()\n", __LINE__);
        ret = 0;
    }

    switch(pctrl.cmd) {
    case SH_CAM_GET_SENSOR_CAMCON:
        ret = shcam_get_sensor_camcon(sync);
        break;
    case SH_CAM_GET_SENSOR_DATA:
        ret = shcam_get_sensor_data(sync, pctrl.ctrl);
        break;
    case SH_CAM_SET_SENSOR_DATA:
        ret = shcam_set_sensor_data(sync, pctrl.ctrl);
        break;
    case SH_CAM_GET_SENSOR_OTP:
        ret = shcam_get_sensor_otp(sync, pctrl.ctrl);
        break;
    case SH_CAM_CTRL_SENSOR_POWER:
        ret = shcam_ctrl_power(sync, pctrl.ctrl);
        break;
    case SH_CAM_GET_SMEM_OTP:
        ret = shcam_get_smem_otp(pctrl.ctrl);
        break;
    case SH_CAM_SET_AF_TABLE:
        ret = shcam_set_af_table(sync, pctrl.ctrl);
        break;
    default:
        break;
    }

    return ret;
}

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
