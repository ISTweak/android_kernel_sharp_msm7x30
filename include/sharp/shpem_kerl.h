/* include/sharp/shpem_kerl.h (Peripheral Microprocessor Driver)
 *
 * Copyright (C) 2010 SHARP CORPORATION
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

#ifndef _SHPEM_KERL_H_
#define _SHPEM_KERL_H_

#include <linux/ioctl.h>

#define SHPEM_I2C_DEVNAME	"shpem"
#define SHPEM_I2C_SLAVE		0x1E

#define SHPEM_MISC_DEVNAME		"shpem_dev"
#define SHPEM_MISC_DEVNAME_ALL	"/dev/shpem_dev"

#define SHPEM_IO			0xA2

#define SHPEM_IOCTL_COM					_IO(SHPEM_IO, 0x01)
#define SHPEM_IOCTL_SET_SLEEP_STATE		_IO(SHPEM_IO, 0x02)
#define SHPEM_IOCTL_SET_SENSOR_STATE	_IO(SHPEM_IO, 0x03)
#define SHPEM_IOCTL_HINT				_IO(SHPEM_IO, 0x04)
#define SHPEM_IOCTL_GPIO_CTL			_IO(SHPEM_IO, 0x05)
#define SHPEM_IOCTL_I2C_WRITE			_IO(SHPEM_IO, 0x06)
#define SHPEM_IOCTL_I2C_READ			_IO(SHPEM_IO, 0x07)
#define SHPEM_IOCTL_FW_WRITE_START		_IO(SHPEM_IO, 0x08)
#define SHPEM_IOCTL_FW_WRITE_END		_IO(SHPEM_IO, 0x09)
#define SHPEM_IOCTL_RESET				_IO(SHPEM_IO, 0x0A)
#define SHPEM_IOCTL_GET_HWREV			_IO(SHPEM_IO, 0x0B)

#define SHPEM_ISR_BREAK_PORT		0x01
#define SHPEM_ISR_RESERVE1_PORT		0x02
#define SHPEM_ISR_RESERVE2_PORT		0x04
#define SHPEM_ISR_RESERVE3_PORT		0x08
#define SHPEM_ISR_RESERVE4_PORT		0x10
#define SHPEM_ISR_RESERVE5_PORT		0x20
#define SHPEM_ISR_RESERVE6_PORT		0x40
#define SHPEM_ISR_RESERVE7_PORT		0x80

#ifndef _SHPEM_USR_H_

typedef signed char shpem_result_t;

enum
{
	SHPEM_SUCCESS            = 0,
	SHPEM_NG                 = -1,
	SHPEM_COM_ERROR          = -2,
	SHPEM_PARAM_ERROR        = -3,
	SHPEM_EXCLUSIVE_ERROR    = -4,
	SHPEM_INIT_ERROR         = -5,
	SHPEM_BREAK_MICON_ERROR  = -6,
	SHPEM_BREAK_SENSOR_ERROR = -7,
	SHPEM_SEQUENCE_ERROR     = -8,
	SHPEM_SLEEP_ERROR        = -9,
	SHPEM_VERIFY_ERROR       = -10,
	SHPEM_IO_ERROR           = -11,
};

enum
{
	SHPEM_CMD_PVER,
	SHPEM_CMD_WPSN,
	SHPEM_CMD_RPSN,
	SHPEM_CMD_CLSP,
	SHPEM_CMD_RDSP,
	SHPEM_CMD_SJGM,
	SHPEM_CMD_RDMS,
	SHPEM_CMD_SMIR,
	SHPEM_CMD_RMIR,
	SHPEM_CMD_TMES,
	SHPEM_CMD_SLEP,
	SHPEM_CMD_TTES,
	SHPEM_CMD_SCMS,
	SHPEM_CMD_DSST,
	SHPEM_CMD_DSTR,
	SHPEM_CMD_DSEN,
	SHPEM_CMD_DSAR,
	SHPEM_CMD_DSAD,
	SHPEM_CMD_ADSR,
	SHPEM_CMD_BTST,
	SHPEM_CMD_SOOF,
	SHPEM_CMD_KIND,
	SHPEM_CMD_DSSD,
	SHPEM_CMD_DSLG,
	SHPEM_CMD_DSHZ,
	SHPEM_CMD_SOAR,
	SHPEM_CMD_SOAC,
	SHPEM_CMD_SHDN,
	SHPEM_CMD_INIT,
	SHPEM_CMD_SENS,
	SHPEM_CMD_SENE,
	SHPEM_CMD_DSCS,
	SHPEM_CMD_DSCE,
	SHPEM_CMD_DSFP,
	SHPEM_CMD_DSIV,
	SHPEM_CMD_DSBS,
	SHPEM_CMD_DSBE,
	SHPEM_CMD_DSPS,
	SHPEM_CMD_RESET,
	SHPEM_CMD_NUM,
};

enum
{
	SHPEM_ISR_ID_BREAK_PEDO,
	SHPEM_ISR_ID_BREAK_ACCEL,
	SHPEM_ISR_ID_BREAK_DISP,
	SHPEM_ISR_ID_NUM,
};

enum
{
	SHPEM_MICON_SLEEP_PEDO,
	SHPEM_MICON_SLEEP_ACCEL,
	SHPEM_MICON_SLEEP_DIAG,
	SHPEM_MICON_SLEEP_NUM,
};

enum
{
	SHPEM_MICON_SLEEP,
	SHPEM_MICON_WAKEUP,
};

enum
{
	SHPEM_MICON_SENSOR_ACCEL,
	SHPEM_MICON_SENSOR_DIAG,
	SHPEM_MICON_SENSOR_NUM,
};

enum
{
	SHPEM_MICON_SENSOR_ON,
	SHPEM_MICON_SENSOR_OFF,
};

#define SHPEM_FW_WRITE_LOOP 6

typedef struct shpem_com_param_tag
{
	unsigned char cmd_id;
	unsigned char* send_buf;
	unsigned char* recv_buf;
} shpem_com_param_t;

#endif /* _SHPEM_USR_H_ */

typedef struct
{
	unsigned char *send_cmd;
	unsigned char *recv_ok;
	unsigned char *recv_ng;
} shpem_command_t;

typedef struct
{
	unsigned char* buf;
	int len;
} shpem_i2c_io_t;

extern const shpem_command_t shpem_command[];

shpem_result_t shpem_com_kerl(shpem_com_param_t* param);
shpem_result_t shpem_set_sleep_state_kerl( unsigned char dev_id, unsigned char sleep );
shpem_result_t shpem_set_sensor_state_kerl( unsigned char dev_id, unsigned char sensor);
shpem_result_t shpem_shdn_send(void);

#endif /* _SHPEM_KERL_H_ */
