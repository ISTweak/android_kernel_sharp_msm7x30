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
#ifndef __SH_CAMLED_RPC_H
#define __SH_CAMLED_RPC_H

int shcamled_red_led_control_rpc(int level);
int shcamled_mobile_led_control_rpc(int level);
int shcamled_set_led_user_info_rpc(int user_info);
int shcamled_get_led_user_info_rpc(void);

MODULE_DESCRIPTION("SHARP CAMLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

#endif
