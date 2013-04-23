/* include/sharp/shtps_sy3000_tm1701-001.h
 *
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
#ifndef __SHTPS_SY3000_TM1701_001_H__
#define __SHTPS_SY3000_TM1701_001_H__

/* -----------------------------------------------------------------------------------
 */
#define SH_TOUCH_DEVNAME		"shtps_rmi"
#define SH_TOUCH_IF_DEVNAME 	"shtpsif"
#define SH_TOUCH_IF_DEVPATH 	"/dev/shtpsif"

#define SHTPS_TM_TXNUM_MAX		20
#define SHTPS_TM_RXNUM_MAX		12

#define TPS_IOC_MAGIC					0xE0

#define TPSDEV_ENABLE					_IO  ( TPS_IOC_MAGIC,  1)
#define TPSDEV_DISABLE					_IO  ( TPS_IOC_MAGIC,  2)
#define TPSDEV_RESET					_IO  ( TPS_IOC_MAGIC,  3)
#define TPSDEV_SOFT_RESET				_IO  ( TPS_IOC_MAGIC,  4)
#define TPSDEV_GET_FW_VERSION			_IOR ( TPS_IOC_MAGIC,  5, unsigned short)
#define TPSDEV_ENTER_BOOTLOADER			_IOR ( TPS_IOC_MAGIC,  6, struct shtps_bootloader_info)
#define TPSDEV_LOCKDOWN_BOOTLOADER		_IOW ( TPS_IOC_MAGIC,  7, struct shtps_ioctl_param)
#define TPSDEV_ERASE_FLASE				_IO  ( TPS_IOC_MAGIC,  8)
#define TPSDEV_WRITE_IMAGE				_IOW ( TPS_IOC_MAGIC,  9, struct shtps_ioctl_param)
#define TPSDEV_WRITE_CONFIG				_IOW ( TPS_IOC_MAGIC, 10, struct shtps_ioctl_param)
#define TPSDEV_GET_TOUCHINFO			_IOR ( TPS_IOC_MAGIC, 11, struct shtps_touch_info)
#define TPSDEV_GET_TOUCHINFO_UNTRANS	_IOR ( TPS_IOC_MAGIC, 12, struct shtps_touch_info)
#define TPSDEV_SET_TOUCHMONITOR_MODE	_IOW ( TPS_IOC_MAGIC, 13, unsigned char)
#define TPSDEV_READ_REG					_IOWR( TPS_IOC_MAGIC, 14, struct shtps_ioctl_param)
#define TPSDEV_READ_ALL_REG				_IOR ( TPS_IOC_MAGIC, 15, struct shtps_ioctl_param)
#define TPSDEV_WRITE_REG				_IOW ( TPS_IOC_MAGIC, 16, struct shtps_ioctl_param)
#define TPSDEV_START_TM					_IOW ( TPS_IOC_MAGIC, 17, struct shtps_ioctl_param)
#define TPSDEV_STOP_TM					_IO  ( TPS_IOC_MAGIC, 18)
#define TPSDEV_GET_BASELINE				_IOR ( TPS_IOC_MAGIC, 19, unsigned short*)
#define TPSDEV_GET_FRAMELINE			_IOR ( TPS_IOC_MAGIC, 20, unsigned char*)
#define TPSDEV_START_FACETOUCHMODE		_IO  ( TPS_IOC_MAGIC, 21)
#define TPSDEV_STOP_FACETOUCHMODE		_IO  ( TPS_IOC_MAGIC, 22)
#define TPSDEV_POLL_FACETOUCHOFF		_IO  ( TPS_IOC_MAGIC, 23)
#define TPSDEV_GET_FWSTATUS				_IOR ( TPS_IOC_MAGIC, 24, unsigned char)
#define TPSDEV_GET_FWDATE				_IOR ( TPS_IOC_MAGIC, 25, unsigned short)
#define TPSDEV_CALIBRATION_PARAM		_IOW ( TPS_IOC_MAGIC, 26, struct shtps_ioctl_param)
#define TPSDEV_DEBUG_REQEVENT			_IOW ( TPS_IOC_MAGIC, 27, int)
#define TPSDEV_SET_DRAGSTEP				_IOW ( TPS_IOC_MAGIC, 28, int)
#define TPSDEV_SET_POLLINGINTERVAL		_IOW ( TPS_IOC_MAGIC, 29, int)
#define TPSDEV_SET_FINGERFIXTIME		_IOW ( TPS_IOC_MAGIC, 30, int)
#define TPSDEV_REZERO					_IO  ( TPS_IOC_MAGIC, 31)
#define TPSDEV_ACK_FACETOUCHOFF			_IO  ( TPS_IOC_MAGIC, 32)
#define TPSDEV_START_TM_F05				_IOW ( TPS_IOC_MAGIC, 33, int)
#define TPSDEV_SET_DRAGSTEP_X			_IOW ( TPS_IOC_MAGIC, 34, int)
#define TPSDEV_SET_DRAGSTEP_Y			_IOW ( TPS_IOC_MAGIC, 35, int)
#define TPSDEV_LOGOUTPUT_ENABLE			_IOW ( TPS_IOC_MAGIC, 36, int)
#define TPSDEV_GET_BL_VERSION			_IOR ( TPS_IOC_MAGIC, 37, unsigned char*)
#define TPSDEV_WRITE_BL_CONFIG			_IOW ( TPS_IOC_MAGIC, 38, struct shtps_ioctl_param)

#define TPSDEV_FACETOUCHOFF_NOCHG	0x00
#define TPSDEV_FACETOUCHOFF_DETECT	0x01

#define TPSDEV_TOUCHINFO_MODE_LCDSIZE	0
#define TPSDEV_TOUCHINFO_MODE_DEVSIZE	1

struct shtps_ioctl_param {
	int				size;
	unsigned char*	data;
};

struct shtps_bootloader_info {
	unsigned long	block_size;
	unsigned long	program_block_num;
	unsigned long	config_block_num;
};

struct shtps_touch_info {
	struct fingers{
		unsigned char	state;
		unsigned short	x;
		unsigned short	y;
		unsigned char	wx;
		unsigned char	wy;
		unsigned char	z;
	} fingers[5];
	
	unsigned char		gs1;
	unsigned char		gs2;
	unsigned char		flick_x;
	unsigned char		flick_y;
	unsigned char		flick_time;
	unsigned char		finger_num;
};

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_F01_RMI_CTRL_ADR	0x0031
#define SHTPS_FINGER_MAX		5

enum shtps_drag_threshold_mode {
	SHTPS_DRAG_THRESHOLD_1ST,
	SHTPS_DRAG_THRESHOLD_2ND
};

enum shtps_proc_mode {
	SHTPS_IN_IDLE,
	SHTPS_IN_NORMAL,
	SHTPS_IN_BOOTLOADER,
};

enum shtps_diag_mode {
	SHTPS_DIAGMODE_NONE,
	SHTPS_DIAGMODE_TOUCHINFO,
	SHTPS_DIAGMODE_TM,
};

enum shtps_diag_tm_mode {
	SHTPS_TMMODE_NONE,
	SHTPS_TMMODE_FRAMELINE,
	SHTPS_TMMODE_BASELINE,
};

struct shtps_platform_data {
	int (*setup)(struct device *);
	void (*teardown)(struct device *);
	int gpio_rst;
};

struct rmi_pdt {
	u8	queryBaseAddr;
	u8	commandBaseAddr;
	u8	controlBaseAddr;
	u8	dataBaseAddr;
	u8	interruptSrcCount;
	u8	functionNumber;
};

struct shtps_touch_state {
	u8				numOfFingers;
	u8				fingerStatus[SHTPS_FINGER_MAX];
	u8				dragStep[SHTPS_FINGER_MAX][2];
	u8				rezeroRequest;
	unsigned long	drag_timeout[SHTPS_FINGER_MAX][2];
};

#define F01_QUERY_MANUFACTURERID(data)	data[0]
#define F01_QUERY_NONCOMPLIANT(data)	((data[1] >> 1) & 0x01)
#define F01_QUERY_CUSTOMMAP(data)		(data[1] & 0x01)
#define F01_QUERY_PRODUCTINFO0(data)	(data[2] & 0x7F)
#define F01_QUERY_PRODUCTINFO1(data)	(data[3] & 0x7F)
#define F01_QUERY_DATACODEYEAR(data)	(data[4] & 0x1F)
#define F01_QUERY_DATACODEMONTH(data)	(data[5] & 0x0F)
#define F01_QUERY_DATACODEDAY(data)		(data[6] & 0x1F)
#define F01_QUERY_TESTID_HI(data)		(data[7] & 0x7F)
#define F01_QUERY_TESTID_LO(data)		(data[8] & 0x7F)
#define F01_QUERY_SERIALNUM_HI(data)	(data[9] & 0x7F)
#define F01_QUERY_SERIALNUM_LO(data)	(data[10]& 0x7F)
#define F01_QUERY_PRODUCTID(data)		data[11]

struct rmi_f01Query {
	u8	data[21];
};

struct rmi_f01 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f01Query	query;
};

#define F05_QUERY_NUMOFRCVEL(data)		(data[0] & 0x3F)
#define F05_QUERY_NUMOFTRANSEL(data)	(data[1] & 0x3F)
#define F05_QUERY_HAS16BIT(data)		((data[3] >> 7) & 0x01)
#define F05_QUERY_IMAGEWINDOWSIZE(data)	(data[4])

struct rmi_f05Query {
	u8	data[6];
};

struct rmi_f05 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f05Query	query;
};

#define F11_QUERY_NUMOFSENSORS(data)	(data[0] & 0x07)
#define F11_QUERY_CONFIGURABLE(data)	((data[1] >> 0x07) & 0x01)
#define F11_QUERY_HASSENSADJUST(data)	((data[1] >> 0x06) & 0x01)
#define F11_QUERY_HASGESTURES(data)		((data[1] >> 0x05) & 0x01)
#define F11_QUERY_HASABS(data)			((data[1] >> 0x04) & 0x01)
#define F11_QUERY_HASREL(data)			((data[1] >> 0x03) & 0x01)
#define F11_QUERY_NUMOFFINGERS(data)	(data[1] & 0x07)
#define F11_QUERY_NUMOFXELEC(data)		(data[2] & 0x1F)
#define F11_QUERY_NUMOFYELEC(data)		(data[3] & 0x1F)
#define F11_QUERY_MAXELEC(data)			(data[4] & 0x1F)
#define F11_QUERY_HASANCHORED(data)		((data[5] >> 0x02) & 0x01)
#define F11_QUERY_ABSDATASIZE(data)		(data[5] & 0x03)
#define F11_QUERY_HASPINCH(data)		((data[6] >> 0x06) & 0x01)
#define F11_QUERY_HASPRESS(data)		((data[6] >> 0x05) & 0x01)
#define F11_QUERY_HASFLICK(data)		((data[6] >> 0x04) & 0x01)
#define F11_QUERY_HASEARLYTAP(data)		((data[6] >> 0x03) & 0x01)
#define F11_QUERY_HASDOUBLETAP(data)	((data[6] >> 0x02) & 0x01)
#define F11_QUERY_HASTAPHOST(data)		((data[6] >> 0x01) & 0x01)
#define F11_QUERY_HASSINGLETAP(data)	(data[6] & 0x01)
#define F11_QUERY_HASISCROLLZONE(data)	((data[7] >> 0x04) & 0x01)
#define F11_QUERY_HASSCROLLZONE(data)	((data[7] >> 0x03) & 0x01)
#define F11_QUERY_HASTOUCHSHAPE(data)	((data[7] >> 0x02) & 0x01)
#define F11_QUERY_HASROTATE(data)		((data[7] >> 0x01) & 0x01)
#define F11_QUERY_HASPALMDET(data)		(data[7] & 0x01)
#define F11_QUERY_HASROTATE(data)		((data[7] >> 0x01) & 0x01)
#define F11_QUERY_HASLOS(data)			((data[8] >> 0x02) & 0x01)
#define F11_QUERY_HASFPROXIMITY(data)	((data[8] >> 0x01) & 0x01)
#define F11_QUERY_HASPEN(data)			(data[8] & 0x01)
#define F11_QUERY_HASGESTURE1(data)		(data[6])
#define F11_QUERY_HASGESTURE2(data)		(data[7] & 0x1F)

struct rmi_f11Query {
	u8	data[9];
};
struct rmi_f11Ctrl {
	u16 maxXPosition;
	u16 maxYPosition;
};

struct rmi_f11 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f11Query	query;
	struct rmi_f11Ctrl	ctrl;
};

#define F34_QUERY_BOOTLOADERID0(data)		data[0]
#define F34_QUERY_BOOTLOADERID1(data)		data[1]
#define F34_QUERY_UNLOCKED(data)			((data[2] >> 0x01) & 0x01)
#define F34_QUERY_BLOCKSIZE(data)			((data[3] & 0xff) | ((data[4] << 0x08) & 0xff00))
#define F34_QUERY_FIRMBLOCKCOUNT(data)		((data[5] & 0xff) | ((data[6] << 0x08) & 0xff00))
#define F34_QUERY_CONFIGBLOCKCOUNT(data)	((data[7] & 0xff) | ((data[8] << 0x08) & 0xff00))

struct rmi_f34Query {
	u8	data[9];
};

struct rmi_f34 {
	u8					enable;
	u32					queryBase;
	u32					dataBase;
	struct rmi_f34Query	query;
};

struct rmi_map {
	struct rmi_f01	fn01;
	struct rmi_f05	fn05;
	struct rmi_f11	fn11;
	struct rmi_f34	fn34;
};

//#define F11_DATA_FINGERSTATE0(data)		data[0]
#define F11_DATA_XPOS(data)				(((data[0] << 0x04) & 0x0FF0) | (data[2] & 0x0F))
#define F11_DATA_YPOS(data)				(((data[1] << 0x04) & 0x0FF0) | ((data[2] >> 0x04) & 0x0F))
#define F11_DATA_WX(data)				(data[3] & 0x0F)
#define F11_DATA_WY(data)				((data[3] >> 0x04) & 0x0F)
#define F11_DATA_Z(data)				data[4]

struct shtps_rmi_fingerState {
	u8	data[6];
};

#endif /* __SHTPS_SY3000_TM1701_001_H__ */
