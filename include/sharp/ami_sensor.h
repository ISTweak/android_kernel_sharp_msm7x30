/*
 * Copyright (C) 2011 SHARP CORPORATION
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

#ifndef AMI_SENSOR_H
#define AMI_SENSOR_H

#include <linux/ioctl.h>

#define AMI_DRV_NAME			"ami_sensor"

enum AMI_SENSOR_MODE {
	AMI_NONE,				/**< none */
	AMI_NORMAL_MODE,		/**< Normal state */
	AMI_FORCE_MODE			/**< Force state */
};

enum AMI_SENSOR_INTERVAL {
	AMI_INTERVAL_20MS,
	AMI_INTERVAL_40MS,
	AMI_INTERVAL_60MS,
	AMI_INTERVAL_80MS,
	AMI_INTERVAL_100MS
};

struct ami_start_sensor_param {
	/**< 0:Normal State 1:Force State */
	uint32_t mode;
	/**< 0:20ms 1:40ms 2:80ms 3:100ms */
	uint32_t interval;
	/**< window */
	struct ami_win_parameter window;
};

/* IOCTLs for AMI-Sensor middleware misc. device library */
#define AMI_IO							0x84
#define AMI_IOCTL_CHIPINFO				_IOR(AMI_IO, 0x01, struct ami_chipinfo)
#define AMI_IOCTL_DRIVERINFO			_IOR(AMI_IO, 0x02, struct ami_driverinfo)

/* geomagnetic and acceleration command */
#define AMI_IOCTL_INITIALIZE_SENSOR		_IOR(AMI_IO, 0x10, struct ami_win_parameter)
#define AMI_IOCTL_START_SENSOR			_IOW(AMI_IO, 0x11, struct ami_start_sensor_param)
#define AMI_IOCTL_STOP_SENSOR			_IO( AMI_IO, 0x12 )
#define AMI_IOCTL_READ_PARAMS			_IOR(AMI_IO, 0x13, struct ami_sensor_parameter)
#define AMI_IOCTL_GET_SENSORRAWVALUE	_IOR(AMI_IO, 0x14, struct ami_sensor_rawvalue)
#define AMI_IOCTL_SET_DELAY				_IOW(AMI_IO, 0x15, int)

/* window command */
#define AMI_IOCTL_SEARCH_OFFSET			_IOR(AMI_IO, 0x21, struct ami_win_parameter)
#define AMI_IOCTL_READ_WINPARAMS		_IOR(AMI_IO, 0x22, struct ami_win_parameter)
#define AMI_IOCTL_WRITE_WINPARAMS		_IOW(AMI_IO, 0x23, struct ami_win_parameter)

/* pedometer command */
#define AMI_IOCTL_START_PEDO			_IOW(AMI_IO, 0x30, struct ami_win_parameter)
#define AMI_IOCTL_STOP_PEDO				_IO( AMI_IO, 0x31 )
#define AMI_IOCTL_SET_PEDO_SH			_IOW(AMI_IO, 0x32, struct ami_pedo_threshold)
#define AMI_IOCTL_GET_PEDO_SH			_IOR(AMI_IO, 0x33, struct ami_pedo_threshold)
#define AMI_IOCTL_GET_PEDO_STATUS		_IOR(AMI_IO, 0x34, struct ami_pedo_status)
#define AMI_IOCTL_CLEAR_PEDO			_IO( AMI_IO, 0x35)

/* shmds add -> */
/* shmds ext command */
#define AMI_IOCTL_GET_SENSORRAWVALUE2	_IOR(AMI_IO, 0xD0, struct ami_sensor_rawvalue)
#define AMI_IOCTL_WRITE_TIMER			_IOW(AMI_IO, 0xD1, int)
#define AMI_IOCTL_WRITE_THRESH			_IOW(AMI_IO, 0xD2, int)
#define AMI_IOCTL_WRITE_BUF_SIZE		_IOW(AMI_IO, 0xD3, int)
#define AMI_IOCTL_WRITE_MD_THRESH1		_IOW(AMI_IO, 0xD4, int)
#define AMI_IOCTL_WRITE_MD_THRESH2		_IOW(AMI_IO, 0xD5, int)
/* shmds add <- */

/* for debug command */
#define AMI_IOCTL_DBG_READ				_IOR(AMI_IO, 0xF0, int)
#define AMI_IOCTL_DBG_READ_W			_IOR(AMI_IO, 0xF1, int)
#define AMI_IOCTL_DBG_READ_DW			_IOR(AMI_IO, 0xF4, int)
#define AMI_IOCTL_DBG_WRITE				_IOR(AMI_IO, 0xF2, int)
#define AMI_IOCTL_DBG_WRITE_W			_IOR(AMI_IO, 0xF3, int)
#define AMI_IOCTL_DBG_WRITE_DW			_IOR(AMI_IO, 0xF5, int)
#define AMI_IOCTL_DBG_READ_PD			_IOR(AMI_IO, 0xF6, int)
#define AMI_IOCTL_DBG_READ_PI			_IOR(AMI_IO, 0xF7, int)

/* shmds testmode command */
#define AMI_IOCTL_SHMDS_RESETSENSOR		_IO( AMI_IO, 0xE0)
#define AMI_IOCTL_SHMDS_READPARAMS		_IOR(AMI_IO, 0xE1, struct ami_sensor_parameter)
#define AMI_IOCTL_SHMDS_GETRAWVALUE		_IOR(AMI_IO, 0xE2, signed short[6])
#define AMI_IOCTL_SHMDS_GETVERSION		_IOR(AMI_IO, 0xE3, unsigned short[3])

#endif
