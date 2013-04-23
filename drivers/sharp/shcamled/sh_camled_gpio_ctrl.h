/* drivers/sharp/shcamled/sh_camled_gpio_ctrl.h  (Camera Driver)
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

#ifndef __SH_CAMLED_GPIO_CTRL_H
#define __SH_CAMLED_GPIO_CTRL_H

#define SH_CAMLED_USE_CAMERA_DRV    (0)
#define SH_CAMLED_USE_KEYLED_DRV    (1)

#define SH_CAMLED_GPIO102_LO        (0)
#define SH_CAMLED_GPIO102_HI        (1)

#define SH_CAMLED_NO_ERROR          (0)
#define SH_CAMLED_USE_CAMERA        (-201)
#define SH_CAMLED_USE_KEYLED        (-202)
#define SH_CAMLED_GPIO_HI_ERR       (-203)
#define SH_CAMLED_GPIO_LO_ERR       (-204)

int sh_camled_gpio_ctrl(int ctrl, int type);


MODULE_DESCRIPTION("SHARP CAMLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

#endif
