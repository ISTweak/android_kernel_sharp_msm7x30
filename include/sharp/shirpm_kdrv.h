/* include/sharp/shirpm_kdrv.h (Infrared driver Power Management)
 *
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#ifndef _SHIRPM_KDRV_H
#define _SHIRPM_KDRV_H





#define IRPOWER_DRV_RESOURCE_NOT_USE	(0)
#define IRPOWER_DRV_RESOURCE_USING	(1)
#define IRPOWER_DRV_RESOURCE_FREE	'0'
#define IRPOWER_DRV_RESOURCE_GET	'1'

#define IRPOWER_DRIVER_NAME		"shirpm"

#define IRPOWER_DEV_NAME	"/sys/devices/platform/"IRPOWER_DRIVER_NAME
#define IRPOWER_DEV_ATTR_VERSION	IRPOWER_DEV_NAME"/version"
#define IRPOWER_DEV_ATTR_IRDARESOUCE	IRPOWER_DEV_NAME"/irdaresource"
#define IRPOWER_DEV_ATTR_IRRCRESOUCE	IRPOWER_DEV_NAME"/irrcresource"

typedef struct {
	char ir_device_use[4];
	char irpm_user_drv_version[16];
	char user_drv_version[16];
	char stack_version[16];
} irpower_resource_write_info;

#endif
