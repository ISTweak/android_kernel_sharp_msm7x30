/* drivers/sharp/shcamled/sh_camleddrv.h  (Camera Driver)
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

#ifndef __SH_CAMLEDDRV_H
#define __SH_CAMLEDDRV_H

#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <mach/board.h>

#include "sh_camled_gpio_ctrl.h"

#define SH_CAMLED_IOCTL_MAGIC 'c'
#define SH_CAMLED_IOCTL_GPIO_CTRL       _IOW(SH_CAMLED_IOCTL_MAGIC, 1, int)
#define SH_CAMLED_IOCTL_RED_CTRL        _IOW(SH_CAMLED_IOCTL_MAGIC, 2, int)
#define SH_CAMLED_IOCTL_MOBILE_CTRL     _IOW(SH_CAMLED_IOCTL_MAGIC, 3, int)
#define SH_CAMLED_IOCTL_SET_UINFO_CTRL  _IOW(SH_CAMLED_IOCTL_MAGIC, 4, int)
#define SH_CAMLED_IOCTL_GET_UINFO_CTRL  _IOR(SH_CAMLED_IOCTL_MAGIC, 5, int)

#define SH_CAMLED_GPIO_WLED_EN_M       102


MODULE_DESCRIPTION("SHARP CAMLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

#endif
