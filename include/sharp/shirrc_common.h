/* include/sharp/shirrc_common.h (Infrared driver)
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

#ifndef _IRRC_COMMON_H
#define _IRRC_COMMON_H



typedef unsigned long	uint32;
typedef unsigned short	uint16;
typedef unsigned char	uint8;

typedef long		int32;
typedef short		int16;
typedef char		int8;

#define	IRRC_PPM_HIGH_LOW	(0)
#define	IRRC_PPM_LOW_HIGH	(1)

#define	IRRC_FRAME_COUNT_MAX	(int32)8

#define	IRRC_HW_BITLEN_MAX	(int32)0x1000
#define	IRRC_USER_DATA_BSIZE	8
#define	IRRC_USER_DATA_BUFSIZE	(IRRC_HW_BITLEN_MAX / IRRC_USER_DATA_BSIZE)

typedef struct {
	int32 carrier_min;
	int32 carrier_max;
	int32 pulse_min;
	int32 pulse_max;
	int32 leader_max;
	int32 trailer_max;
	int32 frame_max;
	int32 end_to_start_min;
	int32 end_to_start_max;
	int32 data_max;
	int32 frame_repeat_max;
	int32 frame_count_max;
	int32 block_repeat_max;
} irrc_drv_capa_info;

#endif
