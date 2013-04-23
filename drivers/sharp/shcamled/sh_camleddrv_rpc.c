/* drivers/sharp/shcamled/sh_camleddrv_rpc.c  (CamLED Driver)
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
#include <linux/err.h>
#include <mach/msm_rpcrouter.h>
#include <sharp/sh_oncrpc_id.h>

#define SHCAMLED_RPC_A2M_PROG       0x50000040
#define SHCAMLED_RPC_A2M_VERS       0x00010001

#define SHCAMLED_RPC_A2M_ONCRPC_NULL_PROC                       0
#define SHCAMLED_RPC_A2M_REMOTE_RPC_GLUE_CODE_INFO_REMOTE_PROC  1
#define SHCAMLED_RPC_A2M_API_INITIALIZE_REMOTE_PROC             2
#define SHCAMLED_RPC_A2M_LED_RED_PROC                           3
#define SHCAMLED_RPC_A2M_LED_MOBILELIGHT_PROC                   4
#define SHCAMLED_RPC_A2M_SET_LED_USER_INFO_PROC                 5
#define SHCAMLED_RPC_A2M_GET_LED_USER_INFO_PROC                 6

#define SHCAMLED_RPC_A2M_TIMEOUT                                (5 * HZ)

static struct msm_rpc_endpoint* endpoint = NULL;

int shcamled_red_led_control_rpc(int level)
{
    int ret = -1;

    struct shcamled_req {
        struct rpc_request_hdr hdr;
        int request_data;
    } req;

    struct shcamled_rep {
        struct rpc_reply_hdr hdr;
        int receive_data;
    } rep;

    /* get rpc endpoint */
    if(NULL == endpoint) {
        endpoint = msm_rpc_connect(SHCAMLED_RPC_A2M_PROG,
                                SHCAMLED_RPC_A2M_VERS,
                                0);
    }
    if(IS_ERR(endpoint)) {
        ret = -1;
        printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
    }
    else {
        req.request_data = cpu_to_be32(level);

        /* rpc event send */
        ret = msm_rpc_call_reply(endpoint, SHCAMLED_RPC_A2M_LED_RED_PROC,
                                &req, sizeof(req), &rep, sizeof(rep),
                                SHCAMLED_RPC_A2M_TIMEOUT);
        if(0 > ret) {
            printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
        }
        else {
            ret = be32_to_cpu(rep.receive_data);
        }
    }

    return ret;
}


int shcamled_mobile_led_control_rpc(int level)
{
    int ret = -1;

    struct shcamled_req {
        struct rpc_request_hdr hdr;
        int request_data;
    } req;

    struct shcamled_rep {
        struct rpc_reply_hdr hdr;
        int receive_data;
    } rep;

    /* get rpc endpoint */
    if(NULL == endpoint) {
        endpoint = msm_rpc_connect(SHCAMLED_RPC_A2M_PROG,
                                SHCAMLED_RPC_A2M_VERS,
                                0);
    }
    if(IS_ERR(endpoint)) {
        ret = -1;
        printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
    }
    else {
        req.request_data = cpu_to_be32(level);

        /* rpc event send */
        ret = msm_rpc_call_reply(endpoint,
                                SHCAMLED_RPC_A2M_LED_MOBILELIGHT_PROC,
                                &req, sizeof(req), &rep, sizeof(rep),
                                SHCAMLED_RPC_A2M_TIMEOUT);
        if(0 > ret) {
            printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
        }
        else {
            ret = be32_to_cpu(rep.receive_data);
        }
    }

    return ret;
}

int shcamled_set_led_user_info_rpc(int user_info)
{
    int ret = -1;

    struct shcamled_req {
        struct rpc_request_hdr hdr;
        int request_data;
    } req;

    struct shcamled_rep {
        struct rpc_reply_hdr hdr;
        int receive_data;
    } rep;

    /* get rpc endpoint */
    if(NULL == endpoint) {
        endpoint = msm_rpc_connect(SHCAMLED_RPC_A2M_PROG,
                                SHCAMLED_RPC_A2M_VERS,
                                0);
    }
    if(IS_ERR(endpoint)) {
        ret = -1;
        printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
    }
    else {
        req.request_data = cpu_to_be32(user_info);

        /* rpc event send */
        ret = msm_rpc_call_reply(endpoint,
                                SHCAMLED_RPC_A2M_SET_LED_USER_INFO_PROC,
                                &req, sizeof(req), &rep, sizeof(rep),
                                SHCAMLED_RPC_A2M_TIMEOUT);
        if(0 > ret) {
            printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
        }
        else {
            ret = be32_to_cpu(rep.receive_data);
        }
    }

    return ret;
}


int shcamled_get_led_user_info_rpc(void)
{
    int ret = -1;

    struct shcamled_req {
        struct rpc_request_hdr hdr;
    } req;

    struct shcamled_rep {
        struct rpc_reply_hdr hdr;
        int receive_data;
    } rep;

    /* get rpc endpoint */
    if(NULL == endpoint) {
        endpoint = msm_rpc_connect(SHCAMLED_RPC_A2M_PROG,
                                SHCAMLED_RPC_A2M_VERS,
                                0);
    }
    if(IS_ERR(endpoint)) {
        ret = -1;
        printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
    }
    else {
        /* rpc event send */
        ret = msm_rpc_call_reply(endpoint,
                                SHCAMLED_RPC_A2M_GET_LED_USER_INFO_PROC,
                                &req, sizeof(req), &rep, sizeof(rep),
                                SHCAMLED_RPC_A2M_TIMEOUT);
        if(0 > ret) {
            printk(KERN_ERR "[%s]:[%d] ret=%d \n", __func__, __LINE__, ret);
        }
        else {
            ret = be32_to_cpu(rep.receive_data);
        }
    }

    return ret;
}

MODULE_DESCRIPTION("SHARP CAMLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
