/* drivers/sharp/shcamsensor/tools/sh_camera_tools.h (Camera Driver)
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

#ifndef __SH_CAMERA_TOOLS_H
#define __SH_CAMERA_TOOLS_H

#include <linux/platform_device.h>
#include <media/msm_camera.h>

#define MSM_CAM_IOCTL_CTRL_SENSOR_COMMAND \
    _IOR(MSM_CAM_IOCTL_MAGIC, 100, struct sh_sensor_clrt*)

#define SH_CAM_GET_SMEM_OTP             0
#define SH_CAM_GET_SENSOR_CAMCON        1
#define SH_CAM_GET_SENSOR_DATA          2
#define SH_CAM_SET_SENSOR_DATA          3
#define SH_CAM_GET_SENSOR_OTP           4
#define SH_CAM_CTRL_SENSOR_POWER        5
#define SH_CAM_SET_AF_TABLE             6

struct sh_sensor_otp {
    unsigned char page;
    unsigned char offset;
    short len;
    unsigned char* buf;
};

struct sh_sensor_clrt {
    int cmd;
    void* ctrl;
};

int sh_camera_ioctl(struct msm_sync *sync, void __user *argp);

#if defined(CONFIG_T8EV4)
    int t8ev4_power_ctrl(struct platform_device *pdev, int8_t power); /* fronm t8ev4.h */
    #define SENSOR_POWER_CTRL           t8ev4_power_ctrl
#endif /* CONFIG_T8EV4 */

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

#endif /* __SH_CAMERA_TOOLS_H */
