/* include/sharp/shirrc_kdrv.h (Infrared driver)
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

#ifndef _IRRC_KDRV_API_H
#define _IRRC_KDRV_API_H

#include <linux/ioctl.h>
#include "shirrc_common.h"

typedef struct {
	int32 carrier_high;
	int32 carrier_low;
	int16 modulation0;
	int16 modulation1;
	int32 pulse0_high;
	int32 pulse0_low;
	int32 pulse1_high;
	int32 pulse1_low;
	int32 leader_high;
	int32 leader_low;
	int32 trailer_high;
	int32 end_to_start;
} irrc_send_info;

#define IRRC_BASE_NUM		(uint16)0x0008
#define IRRC_IRRCDIV		(uint16)0x0102

#define	IRRC_SOURCE_CLK		625

#define	IRRC_CARRIER_UNIT_CONV	10000

#define	IRRC_CARRIER_ROUND_NUM	5
#define	IRRC_CARRIER_10_TIMES	10

#define IRRC_FOPS_DEVFILE_NAME		"shirrc"
#define IRRC_FOPS_DEVFILE		"/dev/"IRRC_FOPS_DEVFILE_NAME

#define IRRC_PLFM_DRIVER_NAME		IRRC_FOPS_DEVFILE_NAME

#define IRRC_PLFM_DEV_NAME	"/sys/devices/platform/"IRRC_PLFM_DRIVER_NAME
#define IRRC_PLFM_DEV_ATTR_NAME		IRRC_PLFM_DEV_NAME"/name"
#define IRRC_PLFM_DEV_ATTR_VERSION	IRRC_PLFM_DEV_NAME"/version"
#define IRRC_PLFM_DEV_ATTR_CAPABILITY	IRRC_PLFM_DEV_NAME"/capability"
#define IRRC_PLFM_DEV_ATTR_STATUS	IRRC_PLFM_DEV_NAME"/status"

#define IRRC_DRV_IOCTL_MAGIC		'r'

#define IRRC_DRV_IOCTL_INIT \
		_IOW(IRRC_DRV_IOCTL_MAGIC, 1, irrc_drv_capa_info)

#define IRRC_DRV_IOCTL_SET_SENDPARAM \
		_IOR(IRRC_DRV_IOCTL_MAGIC, 2, irrc_send_info)

#define IRRC_DRV_IOCTL_STOP_SEND	_IO(IRRC_DRV_IOCTL_MAGIC, 3)

#endif
