/* include/sharp/shtps_tma3xx_tma340-003.h
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
#ifndef __SHTPS_TMA3XX_TMA340_003_H__
#define __SHTPS_TMA3XX_TMA340_003_H__

struct msm_sh_spitps_platform_data {
	int  gpio_irq;
	int  gpio_hssp_clk;
	int  gpio_hssp_data;
	int  gpio_reset;
	int  gpio_standby;
	int  gpio_vcpin;
	int (*gpio_setup)(void);
	void (*gpio_teardown)(void);
};

#define	SH_TOUCH_I2C_DEVNAME	"SH_touchpanel"
#define	SH_TOUCH_SPI_DEVNAME	"SH_touchpanel"

#define	SH_TOUCH_IRQ			118
#define	SH_TOUCH_SPI_CLK		45
#define	SH_TOUCH_SPI_MOSI		47
#define	SH_TOUCH_SPI_MIS0		48
#define	SH_TOUCH_SPI_CS			87
#define	SH_TOUCH_HSSP_CLK		96
#define	SH_TOUCH_HSSP_DATA		97
#define	SH_TOUCH_RESET			55
#define	SH_TOUCH_STBY			95
#define	SH_TOUCH_VCPIN			177

#define	SH_SENSOR_MAX_X			1009
#define	SH_SENSOR_MAX_Y			2012
#define	SH_TOUCH_MAX_Z			255

#define	SH_TOUCH_MAX_DISTANCE	1101
#define	SH_TOUCH_MAX_DIAGONAL	1101
#define SH_TOUCH_LCD_MAX_X		539
#define SH_TOUCH_LCD_MAX_Y		959
#define SH_TOUCH_LCD_V_MAX_X	539
#define SH_TOUCH_LCD_V_MAX_Y	1075

#define SHTPS_TMA_TXNUM_MAX		11
#define SHTPS_TMA_RXNUM_MAX		21
#define SHTPS_TMA_OFFSET_MAX	11

#define	FIRMDATA_SIZE			32768

#define TPSIF_DEV_NAME			"shtpsif"
#define	TPSIF_DEV_FULLNAME		"/dev/shtpsif"

#define SH_TOUCH_IF_DEVNAME 	"shtpsif"
#define SH_TOUCH_IF_DEVPATH 	"/dev/shtpsif"

#define TPSDRV_IOC_MAGIC            0xE0

#define	TPSDEV_ENABLE 				_IO  (TPSDRV_IOC_MAGIC, 0)
#define	TPSDEV_DISABLE				_IO  (TPSDRV_IOC_MAGIC, 1)
#define	TPSDEV_FW_VERSION			_IO  (TPSDRV_IOC_MAGIC, 2)
#define	TPSDEV_FW_DOWNLOAD			_IO  (TPSDRV_IOC_MAGIC, 3)
#define	TPSDEV_FW_UPDATE			_IO  (TPSDRV_IOC_MAGIC, 4)
#define	TPSDEV_START_TESTMODE		_IOW (TPSDRV_IOC_MAGIC, 5, int)
#define	TPSDEV_STOP_TESTMODE		_IO  (TPSDRV_IOC_MAGIC, 6)
#define	TPSDEV_GET_SENSOR			_IOR (TPSDRV_IOC_MAGIC, 7, int)
#define	TPSDEV_SET_FIRMPARAM		_IOW (TPSDRV_IOC_MAGIC, 8, int)
#define	TPSDEV_CALIBRATION_PARAM	_IOW (TPSDRV_IOC_MAGIC, 9, int)
#define	TPSDEV_SLEEP_ON				_IO  (TPSDRV_IOC_MAGIC, 10)
#define	TPSDEV_SLEEP_OFF			_IO  (TPSDRV_IOC_MAGIC, 11)
#define	TPSDEV_FW_UPDATE2			_IO  (TPSDRV_IOC_MAGIC, 12)

#define	TPSDEV_UPDATE_START			_IOR (TPSDRV_IOC_MAGIC, 13, int)
#define	TPSDEV_UPDATE_STOP			_IOW (TPSDRV_IOC_MAGIC, 14, int)
#define	TPSDEV_GPIO_RESET			_IO  (TPSDRV_IOC_MAGIC, 15)
#define	TPSDEV_GPIO_HSSPCLK			_IOW (TPSDRV_IOC_MAGIC, 16, int)
#define	TPSDEV_GPIO_HSSPDATA		_IOW (TPSDRV_IOC_MAGIC, 17, int)
#define	TPSDEV_GPIO_CLK_INOUT		_IOW (TPSDRV_IOC_MAGIC, 18, int)
#define	TPSDEV_GPIO_DATA_INOUT		_IOW (TPSDRV_IOC_MAGIC, 19, int)
#define	TPSDEV_RUN_CLOCK			_IOW (TPSDRV_IOC_MAGIC, 20, int)
#define	TPSDEV_RECV_BYTE			_IOR (TPSDRV_IOC_MAGIC, 21, int)
#define	TPSDEV_SEND_BYTE			_IOW (TPSDRV_IOC_MAGIC, 22, int)
#define	TPSDEV_DETECT_HILO			_IOW (TPSDRV_IOC_MAGIC, 23, int)
#define	TPSDEV_RECALIBRATION_IDAC	_IO  (TPSDRV_IOC_MAGIC, 24)
#define TPSDEV_READ_REG				_IOWR(TPSDRV_IOC_MAGIC, 25, struct shtps_ioctl_param)
#define TPSDEV_WRITE_REG			_IOW (TPSDRV_IOC_MAGIC, 26, struct shtps_ioctl_param)
#define	TPSDEV_HWRESET				_IO  (TPSDRV_IOC_MAGIC, 27)
#define	TPSDEV_SOFTRESET			_IO  (TPSDRV_IOC_MAGIC, 28)
#define	TPSDEV_GET_TOUCHINFO			_IOR ( TPSDRV_IOC_MAGIC, 29, struct shtps_touch_info)
#define	TPSDEV_GET_TOUCHINFO_UNTRANS	_IOR ( TPSDRV_IOC_MAGIC, 30, struct shtps_touch_info)
#define	TPSDEV_SET_TOUCHMONITOR_MODE	_IOW ( TPSDRV_IOC_MAGIC, 31, unsigned char)
#define	TPSDEV_START_FACETOUCHMODE	_IO ( TPSDRV_IOC_MAGIC, 32)
#define	TPSDEV_STOP_FACETOUCHMODE	_IO  ( TPSDRV_IOC_MAGIC, 33)
#define	TPSDEV_POLL_FACETOUCHOFF	_IO  ( TPSDRV_IOC_MAGIC, 34)
#define	TPSDEV_ACK_FACETOUCHOFF		_IO  ( TPSDRV_IOC_MAGIC, 35)
#define	TPSDEV_TPSBLANK				_IOW (TPSDRV_IOC_MAGIC, 36, int)

#define	TPSDEV_FACETOUCHOFF_NOCHG	0x00
#define	TPSDEV_FACETOUCHOFF_DETECT	0x01

#define TPSDEV_TOUCHINFO_MODE_LCDSIZE	0
#define TPSDEV_TOUCHINFO_MODE_DEVSIZE	1

struct shtps_touch_info {
	struct fingers{
		unsigned char	id;
		unsigned short	x;
		unsigned short	y;
		unsigned short	z;
	} fingers[4];
};

typedef struct
{
	uint8_t bData;
	int nNumBits;
	uint8_t bReset;
} Tps_send_data;

typedef struct
{
	uint16_t wPosX;						/* X Pos */
	uint16_t wPosY;						/* Y Pos */
	uint8_t  bSensData[SHTPS_TMA_TXNUM_MAX*SHTPS_TMA_RXNUM_MAX];	/* Sensor data */
	uint8_t  bCounter;					/* New Data Counter */
} TpsSensorData;

struct shtps_ioctl_param {
	int				size;
	unsigned char*	data;
};

enum shtps_drag_threshold_mode {
	TPS_DRAG_THRESHOLD_1ST,
	TPS_DRAG_THRESHOLD_2ND
};

void msm_spitps_flipchange(int nFlipState);
void msm_tps_setsleep(int nIsSleep);
void msm_tps_shutdown(void);
void msm_tps_set_chargerarmor(int nMode);

#endif /* __SHTPS_TMA3XX_TMA340_003_H__ */
