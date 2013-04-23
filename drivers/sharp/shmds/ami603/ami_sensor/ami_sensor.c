/* ami_sensor.c - AMI-Sensor driver
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

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		Driver Information.
 *-------+---------+---------+---------+---------+---------+---------+--------*/
#define	THIS_CODE_REMARKS "am603."	/*  Driver Information response. */
#define THIS_VER_MAJOR	0
#define THIS_VER_MIDDLE	0
#define THIS_VER_MINOR	7

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		Compile Switch
 *-------+---------+---------+---------+---------+---------+---------+--------*/
#define		SMBUS_USE	/* i2c ca use smbus or not. */
/*#define       DEBUG_PRINT *//* debug write or not. */
/*#define       DEBUG_DUMP  *//* dump write or not. */

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		Includes depend on platform
 *-------+---------+---------+---------+---------+---------+---------+--------*/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/time.h>
/* shmds add -> */
#include <linux/wait.h>
#include <linux/semaphore.h>
/* shmds add <- */

#include "sharp/ami_hw.h"
#include "sharp/ami_sensor_def.h"
#include "sharp/ami_driver_def.h"
#include "sharp/ami_sensor.h"

#include "sharp/shterm_k.h" /* shmds add */
#include "sharp/shmds_driver.h" /* shmds add */

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		define	
 *-------+---------+---------+---------+---------+---------+---------+--------*/
#define	TRUE				1
#define	FALSE				0

#define AMI_FINE_MAX			96
#define AMI_STANDARD_OFFSET		0x800
#define AMI_GAIN_COR_DEFAULT	1000

#define AMI_TICK_INTERVAL_20MS	640
#define AMI_TICK_INTERVAL_40MS	1280
#define AMI_TICK_INTERVAL_60MS	1920
#define AMI_TICK_INTERVAL_80MS	2560
#define AMI_TICK_INTERVAL_100MS	3200

#define AMI_BIT_FORCE_MODE		0x01
#define AMI_BIT_NORMAL_MODE		0x02
#define AMI_BIT_PEDO_MODE		0x04

/* shmds add -> */
#ifdef SHMDS_DETECT
#define SHMDS_DETECT_NORMAL     0 /**< mode1 */ 
#define SHMDS_DETECT_ECONOMIZE     1 /**< mode2 */

DECLARE_WAIT_QUEUE_HEAD(shmds_sleep_wq);
DECLARE_MUTEX(shmds_sem);

#define AMI_REG_MD_THRESH 				0x2C
#define AMI_REG_MD_TIMER 				0x2D

#endif
/* shmds add <- */

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		global variable definition	
 *-------+---------+---------+---------+---------+---------+---------+--------*/
static struct i2c_client *ami_i2c_client = NULL;

static struct _ami_initoffset {
	int mode;
	u8 fine[3];
} ami_sen_stat;

static struct ami_start_sensor_param k_start_param;
static struct ami_pedo_threshold g_pedoThreshold;
static struct ami_pedo_status g_pedoStatus;
static struct semaphore g_mutex;
static int k_initflag;
static s16 k_val_buff[6];
static int64_t g_time;
static uint32_t g_interval;
static struct ami_win_parameter k_win;
static struct ami_sensor_parameter k_param;
static struct ami_sensor_rawvalue k_val;
static struct ami_chipinfo k_chip;
static struct ami_driverinfo k_drv;
/* shmds add -> */
static unsigned short shmds_ver[3];
static struct semaphore shmds_pedomutex;
static unsigned char shmds_pedopauseflag = 0;
static unsigned char shmds_testmodeflag = 0;
#ifdef SHMDS_DETECT
struct shmds_ami603_data {
    struct input_dev *input_dev;
    struct work_struct work;
};

static struct work_struct shmds_work;
static s16 *shmds_detect_buf[3];
static int shmds_buf_size = 10;
static int shmds_buf_size_temp = 10;
static int shmds_md_thresh1 = 50;
static int shmds_md_thresh2 = 50;
static int shmds_timer[2] = {AMI_REG_MD_TIMER, 0x00};
static int shmds_thresh[2] = {AMI_REG_MD_THRESH, 25};
static int shmds_init_flg = 0;
static int shmds_buf_loop_flg = 0;
static int shmds_close_flg = 1;
static int shmds_count = 0;
static int shmds_wakeup = 0;
static int shmds_detect_switch_flg = 0;
static int shmds_detect_mode_flg = SHMDS_DETECT_NORMAL;
static s16 shmds_economize_buf[3];
#endif
/* shmds add <- */

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		Some routines for debug or trace
 *-------+---------+---------+---------+---------+---------+---------+--------*/
#ifdef DEBUG_PRINT
#define PRINTK(fmt, args...) printk( "[Motion Debug]" KERN_INFO fmt, ## args) /* shmds mod */
#else
#define PRINTK(fmt, args...)
#endif
#define PRINTK_E(fmt, args...) printk( "[Motion Error]" fmt, ## args) /* shmds add */

#ifdef DEBUG_DUMP
#define HEX_DUMP(a,b,c,d) Hex_dump(a,b,c,d)
#else
#define HEX_DUMP(a,b,c,d)
#endif

#ifdef DEBUG_DUMP
#define HEX_DUMP_BUFF 256
#define HEX_DUMP_LEN (HEX_DUMP_BUFF/3)
static void Hex_dump(const char *str, int cmd, const char *p, unsigned int n)
{
	char wk[HEX_DUMP_BUFF];
	int i = 0;
	for (i = 0; i < (n > HEX_DUMP_LEN ? HEX_DUMP_LEN : n); i++) {
		sprintf(wk + (i * 3), "%02x ", *(p + i));
	}
	PRINTK_E(KERN_INFO "%-10s : cmd(0x%02x) len(%d) %s\n", str, cmd, n, wk);
}
#endif

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *		Some routines using i2c interface
 *-------+---------+---------+---------+---------+---------+---------+--------*/
/*---------------------------------------------------------------------------*/
#ifndef SMBUS_USE
/* i2c_master_send */
static int AMI_i2c_send(struct i2c_client *client, u8 cmd, u8 len,
			const u8 * buf)
{
	int res = 0;
	u8 writebuf[256];	/* AMI-Sensor register size */
	int i;
	const u8 *p = buf;
	writebuf[0] = cmd;
	for (i = len; i > 0; --i) {
		writebuf[sendlen - i + 1] = *p;
		++p;
	}
	res = i2c_master_send(client, writebuf, len + 1);
	return (res < 0 ? res : 0);
}

/*---------------------------------------------------------------------------*/
/* i2c_master_read */
static int AMI_i2c_recv(struct i2c_client *client, u8 cmd, u8 len, u8 * buf)
{
	int res = 0;
	res = i2c_master_send(client, &cmd, 1);
	res = i2c_master_recv(client, buf, len);
	return (res < 0 ? res : 0);
}
#else
/*---------------------------------------------------------------------------*/
/* AMI Write Sequence using SMBus interface */
static int AMI_i2c_send(struct i2c_client *client, u8 cmd, u8 len, u8 * buf)
{
	int res = 0;
	HEX_DUMP("AMI WRITE", cmd, buf, len);
	res = i2c_smbus_write_i2c_block_data(client, cmd, len, buf);
	return (res < 0 ? res : 0);
}

/*---------------------------------------------------------------------------*/
/* AMI Read Sequence using SMBus interface */
static int AMI_i2c_recv(struct i2c_client *client, u8 cmd, u8 len, u8 * buf)
{
	int res = 0;
	res = i2c_smbus_read_i2c_block_data(client, cmd, len, buf);
	HEX_DUMP("AMI READ", cmd, buf, len);
	return (res < 0 ? res : 0);
}
#endif
/*---------------------------------------------------------------------------*/
#define AMI_i2c_send_b(client, cmd, buf) AMI_i2c_send(client, cmd, 1, &buf)
/*---------------------------------------------------------------------------*/
#define AMI_i2c_recv_b(client, cmd, buf) AMI_i2c_recv(client, cmd, 1, buf)
/*---------------------------------------------------------------------------*/
static int AMI_i2c_send_w(struct i2c_client *client, u8 cmd, const u16 buf)
{
	u8 dat[2];
	dat[0] = 0xFF & buf;
	dat[1] = 0xFF & buf >> 8;
	return AMI_i2c_send(client, cmd, sizeof(dat), dat);
}

/*---------------------------------------------------------------------------*/
static int AMI_i2c_recv_w(struct i2c_client *client, u8 cmd, u16 * buf)
{
	u8 dat[2];
	int ret = AMI_i2c_recv(client, cmd, sizeof(dat), dat);
	*buf = dat[1] << 8 | dat[0];
	return ret;
}

/*---------------------------------------------------------------------------*/
static int AMI_i2c_send_dw(struct i2c_client *client, u8 cmd, const u32 buf)
{
	u8 dat[4];
	dat[0] = 0xFF & buf;
	dat[1] = 0xFF & buf >> 8;
	dat[2] = 0xFF & buf >> 16;
	dat[3] = 0xFF & buf >> 24;
	return AMI_i2c_send(client, cmd, sizeof(dat), dat);
}

/*---------------------------------------------------------------------------*/
static int AMI_i2c_recv_dw(struct i2c_client *client, u8 cmd, u32 * buf)
{
	u8 dat[4];
	int ret = AMI_i2c_recv(client, cmd, sizeof(dat), dat);
	*buf = dat[3] << 24 | dat[2] << 16 | dat[1] << 8 | dat[0];
	return ret;
}

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *	Some routines, bit Up or bit Dn AMI register.
 *-------+---------+---------+---------+---------+---------+---------+--------*/
/*--------------------------------------------------------------*/
static int AMI_Up_RegisterBit8(struct i2c_client *client, u8 regiaddr, u8 bit)
{
	int res = 0;
	u8 buf;

	res = AMI_i2c_recv_b(client, regiaddr, &buf);
	buf |= bit;
	res = AMI_i2c_send_b(client, regiaddr, buf);
	return (res);		/* At read error, it may happen write error. */
}

/*--------------------------------------------------------------*/
static int AMI_Dn_RegisterBit8(struct i2c_client *client, u8 regiaddr,
			       u8 maskbit)
{
	int res = 0;
	u8 buf;

	res = AMI_i2c_recv_b(client, regiaddr, &buf);
	buf &= (u8) ~ (maskbit);
	res = AMI_i2c_send_b(client, regiaddr, buf);
	return (res);		/* At read error, it may happen write error. */
}

/* shmds add -> */
#ifdef SHMDS_DETECT
static void SHMDS_ami603_work_func(struct work_struct *work)
{
	u8 buf1,buf2;

	AMI_i2c_recv_b(ami_i2c_client, AMI_REG_STA1, &buf1);
	AMI_i2c_recv_b(ami_i2c_client, AMI_REG_INS2, &buf2);

	if (((buf1 & 0x08) == 0x08) && ((buf2 & 0x80) == 0x80)) {
		AMI_i2c_recv_b(ami_i2c_client, AMI_REG_INTREL, &buf1);
		shmds_wakeup = 1;
		wake_up_interruptible(&shmds_sleep_wq);
	}
}

static irqreturn_t SHMDS_ami603_interrupt(int irq, void *dev_id)
{
	schedule_work(&shmds_work);
    return IRQ_HANDLED;
}

static void SHMDS_ami603_Start_SensorInterrupt(void)
{
	u8 temp;
	shmds_close_flg = 0;
	
	temp = 0xFF & shmds_timer[1];
	AMI_i2c_send_b(ami_i2c_client, shmds_timer[0], temp);
	
	temp = 0xFF & shmds_thresh[1];
	AMI_i2c_send_b(ami_i2c_client, shmds_thresh[0], temp);
	
	temp = 0x0c;
	AMI_i2c_send_b(ami_i2c_client, AMI_REG_INC1, temp); //INT Enable
	AMI_Dn_RegisterBit8(ami_i2c_client, 0x29, 0x03);

	AMI_Up_RegisterBit8(ami_i2c_client, AMI_REG_CNTL1, 0x82); //Detection Enable

	enable_irq(ami_i2c_client->irq);
}

static int SHMDS_ami603_Stop_SensorInterrupt(void)
{
	u8 temp;
	
    shmds_close_flg = 1;
	
	disable_irq_nosync(ami_i2c_client->irq);
    
	AMI_Dn_RegisterBit8(ami_i2c_client, AMI_REG_CNTL1, 0x02);
	temp = 0x04;
	AMI_i2c_send_b(ami_i2c_client, AMI_REG_INC1, temp);

    return 0;
}
#endif
/* shmds add <- */

/*-------+---------+---------+---------+---------+---------+---------+---------+
 *	Some routines ami command
 *-------+---------+---------+---------+---------+---------+---------+--------*/
/*--------------------------------------------------------------*/
/* FS1 Force & PC1 Active */
static int AMI_ForceModeActive(struct i2c_client *client)
{
	int res = 0;
	res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x80 | 0x40);
	udelay(8);		/* 8microsec(Turn on time 2) */
	return (res);
}

/*--------------------------------------------------------------*/
/*A_CNTL A_ALWAYS = 1 */
static int AMI_AccCtrlOn(struct i2c_client *client)
{
	return AMI_Up_RegisterBit8(client, AMI_REG_A_CNTL, 0x01);
}

/*A_CNTL A_ALWAYS = 0 */
static int AMI_AccCtrlOff(struct i2c_client *client)
{
	return AMI_Dn_RegisterBit8(client, AMI_REG_A_CNTL, 0x01);
}

/*CNTL3 FRC_M = 1 & FRC_A = 1 */
static int AMI_StartMesForceState(struct i2c_client *client)
{
	u8 dat = 0x40 | 0x20;
	return AMI_i2c_send_b(client, AMI_REG_CNTL3, dat);
}

/*CNTL3 FRC_M = 0 & FRC_A = 0 */
static int AMI_StopMesForceState(struct i2c_client *client)
{
	return AMI_Dn_RegisterBit8(client, AMI_REG_CNTL3, 0x40 | 0x20);
}

/*--------------------------------------------------------------*/
/*CNTL3 NRM_M = 1 & NRM_A = 1 */
static int AMI_StartMesNormalState(struct i2c_client *client)
{
	return AMI_Up_RegisterBit8(client, AMI_REG_CNTL3, 0x08 | 0x04);
}

/*CNTL3 NRM_M = 0 & NRM_A = 0 */
static int AMI_StopMesNormalState(struct i2c_client *client)
{
	return AMI_Dn_RegisterBit8(client, AMI_REG_CNTL3, 0x08 | 0x04);
}

/*--------------------------------------------------------------*/
/* FS1 Normal & PC1 Active */
static int AMI_NormalModeActive(struct i2c_client *client)
{
	int res;
	res = AMI_Dn_RegisterBit8(client, AMI_REG_CNTL1, 0x40);
	udelay(8);		/* 8microsec(Turn on time 2) */
	return res;
}

/*--------------------------------------------------------------*/
/* CNTL1 PC1 = 1; */
static int AMI_Active(struct i2c_client *client)
{
	int res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x80);
	udelay(8);		/* 8microsec(Turn on time 2) */
	return res;
}


static int AMI_StopMes(struct i2c_client *client)
{
	int res = 0;
	res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x40);
	return (res);
}

/* PC1 = 0 Stand-by */
static int AMI_Standby(struct i2c_client *client)
{
	int res = 0;
	res = AMI_Dn_RegisterBit8(client, AMI_REG_CNTL1, 0x80);
	udelay(30);		/* 30microsec(Turn off time 1) */
	return (res);
}

/*--------------------------------------------------------------*/
/* DRDY enable */
static int AMI_DRDY_Enable(struct i2c_client *client)
{
	return AMI_Up_RegisterBit8(client, AMI_REG_CNTL2, 0x80);
}

/*--------------------------------------------------------------*/
/* change Tick Interval */
static u16 AMI_ChangeTickinterval(uint32_t interval)
{
	u16 tick = 0;

	switch (interval) {
	case AMI_INTERVAL_20MS:
		tick = AMI_TICK_INTERVAL_20MS;
		break;
	default:
	case AMI_INTERVAL_40MS:
		tick = AMI_TICK_INTERVAL_40MS;
		break;
	case AMI_INTERVAL_60MS:
		tick = AMI_TICK_INTERVAL_60MS;
		break;
	case AMI_INTERVAL_80MS:
		tick = AMI_TICK_INTERVAL_80MS;
		break;
	case AMI_INTERVAL_100MS:
		tick = AMI_TICK_INTERVAL_100MS;
		break;
	}

	return tick;
}

/*--------------------------------------------------------------*/
/* set Tick Interval */
static int AMI_SetTickInterval(struct i2c_client *client, uint32_t interval)
{
	return AMI_i2c_send_w(client, AMI_REG_TICK_INTERVAL,
			      AMI_ChangeTickinterval(interval));
}

/*--------------------------------------------------------------*/
/* SoftReset */
static int AMI_SRSTset(struct i2c_client *client)
{
	int res = 0;
	res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL3, 0x80);
	udelay(300);		/* 250microsec(Turn on time 1) */
	return (res);
}

/*--------------------------------------------------------------*/
/* DATA X,Y,Z read */
static int AMI_Read_RawData(struct i2c_client *client, s16 dat[6])
{
	int ret = 0;
	u8 buf[12];
	ret = AMI_i2c_recv(client, AMI_REG_AXOUT, sizeof(buf), buf);
	/*mag */
	dat[0] = buf[7] << 8 | buf[6];
	dat[1] = buf[9] << 8 | buf[8];
	dat[2] = buf[11] << 8 | buf[10];
	/*acc */
	dat[3] = buf[1] << 8 | buf[0];
	dat[4] = buf[3] << 8 | buf[2];
	dat[5] = buf[5] << 8 | buf[4];

	return ret;
}

/*--------------------------------------------------------------*/
/* Temperature */
static int AMI_Read_Temperature(struct i2c_client *client, u16 * dat)
{
	return AMI_i2c_recv_w(client, AMI_REG_TEMP, dat);
}

/*--------------------------------------------------------------*/
/* read fine_output */
static int AMI_FineOutput(struct i2c_client *client, u16 dat[3])
{
	int ret = 0;
	u8 page = 0x0f;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	ret = AMI_i2c_recv_w(client, AMI_OTP_FINE_OUTPUTX, &dat[0]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_FINE_OUTPUTY, &dat[1]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_FINE_OUTPUTZ, &dat[2]);
	page = 0x00;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	return ret;
}

/*--------------------------------------------------------------*/
/* OFF X,Y,Z write */
static int AMI_Write_Offset(struct i2c_client *client, u8 fine[3])
{
	int ret = 0;
	/* shmds mod -> */
	if((fine[0] != 0) && (fine[1] != 0) && (fine[2] != 0)){
		ret = AMI_i2c_send_b(client, AMI_REG_OFFX, fine[0]);
		ret = AMI_i2c_send_b(client, AMI_REG_OFFY, fine[1]);
		ret = AMI_i2c_send_b(client, AMI_REG_OFFZ, fine[2]);
	}
	/* shmds mod <- */
	return ret;
}

/*--------------------------------------------------------------*/
/* OFF X,Y,Z read */
static int AMI_Read_Offset(struct i2c_client *client, u8 fine[3])
{
	int ret = 0;
	ret = AMI_i2c_recv_b(client, AMI_REG_OFFX, &fine[0]);
	ret = AMI_i2c_recv_b(client, AMI_REG_OFFY, &fine[1]);
	ret = AMI_i2c_recv_b(client, AMI_REG_OFFZ, &fine[2]);
	return ret;
}

/*--------------------------------------------------------------*/
/* OFFOTP X,Y,Z read */
static int AMI_Read_OffsetOTP(struct i2c_client *client, u8 fine[3])
{
	int ret = 0;
	u8 dat[3];
	u8 page = 0x0f;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	ret = AMI_i2c_recv_b(client, AMI_OTP_0GAUSS_FINEX, &dat[0]);
	ret = AMI_i2c_recv_b(client, AMI_OTP_0GAUSS_FINEY, &dat[1]);
	ret = AMI_i2c_recv_b(client, AMI_OTP_0GAUSS_FINEZ, &dat[2]);
	page = 0x00;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);

	fine[0] = (u8) (dat[0] & 0x007f);
	fine[1] = (u8) (dat[1] & 0x007f);
	fine[2] = (u8) (dat[2] & 0x007f);
	return ret;
}

/*--------------------------------------------------------------*/
/* SENS X,Y,Z read */
static int AMI_Read_Sense(struct i2c_client *client, u16 dat[6])
{
	int ret = 0;
	u8 page = 0x0f;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	ret = AMI_i2c_recv_w(client, AMI_OTP_SENSMX, &dat[0]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_SENSMY, &dat[1]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_SENSMZ, &dat[2]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_SENSAX, &dat[3]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_SENSAY, &dat[4]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_SENSAZ, &dat[5]);
	page = 0x00;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	return ret;
}

/*--------------------------------------------------------------*/
/* GAIN PARA X,Y,Z read */
static int AMI_Read_GainPara(struct i2c_client *client, u8 interference[6])
{
	int ret = 0;
	u16 dat[3];
	u8 page = 0x0f;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	ret = AMI_i2c_recv_w(client, AMI_OTP_GAIN_PARA_MX, &dat[0]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_GAIN_PARA_MY, &dat[1]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_GAIN_PARA_MZ, &dat[2]);
	page = 0x00;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	interference[0] = 0xFF & (dat[0] >> 8);	/*xy */
	interference[1] = 0xFF & dat[0];	/*xz */
	interference[2] = 0xFF & (dat[1] >> 8);	/*yz */
	interference[3] = 0xFF & dat[1];	/*yx */
	interference[4] = 0xFF & (dat[2] >> 8);	/*zy */
	interference[5] = 0xFF & dat[2];	/*zx */

	return ret;
}

#if 0 /* shmds del */
static int AMI_Read_Deviation(struct i2c_client *client, u16 * dev)
{
	int ret = 0;
	u16 dat;
	u8 page = 0x0f;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	ret = AMI_i2c_recv_w(client, AMI_OTP_GAIN_PARA_AX, &dat);
	page = 0x00;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	*dev = dat;
	return ret;
}
#endif /* shmds del */

/*--------------------------------------------------------------*/
static int64_t currentTimeInNanoSec(void)
{
	struct timespec sTime;
	int64_t time;

	sTime = current_kernel_time();
	time = (int64_t) sTime.tv_sec * 1000000000LL + (int64_t) sTime.tv_nsec;
	return time;
}

/*--------------------------------------------------------------*/
/* Wait DataReaDY */
static int AMI_Wait_DataReady(struct i2c_client *client, unsigned long msecs,
			      unsigned long times)
{
	unsigned long i;

	for (i = 0; i < times; i++) {
		if (gpio_get_value(AMI_GPIO_DRDY) > 0) {
			PRINTK("DRDY ON (%ld)\n", i);
			return 0;
		}
#if 0 /* shmds mod */
		msleep(msecs);
#else /* shmds mod */
		if (shmds_testmodeflag == 0) {
			msleep(msecs);
		} else {
			udelay(msecs * 1000);
		}
#endif /* shmds mod */
	}
	return (-EAGAIN);
}

#if 0 /* shmds del */
static int AMI_Read_Interrupt(struct i2c_client *client, u8 * dat)
{
	*dat = gpio_get_value(AMI_GPIO_INT);
	return 0;
}
#endif /* shmds del */

/*--------------------------------------------------------------*/
/* read WIA */
static int AMI_WhoIam(struct i2c_client *client, u8 * dat)
{
	return AMI_i2c_recv_b(client, AMI_REG_WIA, dat);
}

/*--------------------------------------------------------------*/
/* read VERSION */
static int AMI_GetVersion(struct i2c_client *client, u16 * dat)
{
	int ret = AMI_i2c_recv_w(client, AMI_REG_VER, dat);
	*dat &= 0x7f;
	return ret;
}

/*--------------------------------------------------------------*/
/* read S/N */
static int AMI_SerialNumber(struct i2c_client *client, u16 * dat)
{
	return AMI_i2c_recv_w(client, AMI_REG_SN, dat);
}

/*--------------------------------------------------------------*/
/* read MoreInfo */
static int AMI_MoreInfo(struct i2c_client *client, u16 * dat)
{
	return AMI_i2c_recv_w(client, AMI_REG_INFO, dat);
}

/*--------------------------------------------------------------*/
static void AMI_Up_Mode(int mode)
{
	ami_sen_stat.mode |= mode;
}

/*--------------------------------------------------------------*/
static void AMI_Dn_Mode(int mode)
{
	ami_sen_stat.mode &= ~mode;
}

/*============================================================================*/
/*======			Get Sensor Raw Value			====*/
/*============================================================================*/
/*--------------------------------------------------------------*/
#define AMI_WAIT_DATAREADY_RETRY		3	/* retry times */
#define AMI_DRDYWAIT				1	/* u(micro) sec */
static int AMI_ForceMesurement(struct i2c_client *client, s16 ver[6])
{
	int res;
	/* Write CNTL3:FORCE = 1 */
	res = AMI_StartMesForceState(client);
	if (0 > res) {
		PRINTK_E("AMI_StartMesForceState error : %d\n", res);
		return res;
	}
	/* Wait DRDY high */
	res =
	    AMI_Wait_DataReady(client, AMI_DRDYWAIT, AMI_WAIT_DATAREADY_RETRY);
	if (0 > res) {
		PRINTK_E("AMI_Wait_DataReady error : %d\n", res);
		return res;
	}
	/* READ DATA X,Y,Z */
	res = AMI_Read_RawData(client, ver);
	if (0 > res) {
		PRINTK_E("AMI_Read_RawData error : %d\n", res);
		return res;
	}
	return 0;
}

static int AMI_ChangeIntervalToMS(uint32_t interval)
{
	if (interval == AMI_INTERVAL_20MS) {
		return 20;
	} else if (interval == AMI_INTERVAL_40MS) {
		return 40;
	} else if (interval == AMI_INTERVAL_60MS) {
		return 60;
	} else if (interval == AMI_INTERVAL_80MS) {
		return 80;
	} else if (interval == AMI_INTERVAL_100MS) {
		return 100;
	} else {
		return 40;
	}
}

/*--------------------------------------------------------------*/
static int AMI_IsInterval(int64_t old)
{
	int64_t now = currentTimeInNanoSec();
	int64_t diff = now - old;
	int64_t interval = AMI_ChangeIntervalToMS(g_interval) * 1000000;

	if (diff >= interval) {
		PRINTK("%s ok diff : %lld in : %lld\n", __func__, diff,
		       interval);
		return 1;
	}
	PRINTK("%s ng diff : %lld in : %lld\n", __func__, diff, interval);
	return 0;
}

/*--------------------------------------------------------------*/
#define AMI_RETRY_BUFF_DRDY 2
#define AMI_WAIT_BUFF_DRDY_TIME 1
static int AMI_NormalMesurement(struct i2c_client *client, s16 ver[6], int flag)
{
	int res;

/* shmds add -> */
#ifdef SHMDS_DETECT
	s16 sub[3] = {0,0,0};
#endif
/* shmds add <- */

	/* Wait DRDY high */

	if (k_initflag == 1 || AMI_IsInterval(g_time) == 1 || flag == 1 ) {
		PRINTK("%s init.\n", __func__);
		k_initflag = 0;
		res =
		    AMI_Wait_DataReady(client,
				       1,
				       AMI_ChangeIntervalToMS
				       (k_start_param.interval) + 1);
		if (0 > res) {
			PRINTK_E("AMI_Wait_DataReady error : %d\n", res);
			return res;
		}
	} else {
		PRINTK("%s no init.\n", __func__);
		res =
		    AMI_Wait_DataReady(client, AMI_WAIT_BUFF_DRDY_TIME,
				       AMI_RETRY_BUFF_DRDY);
		if (0 > res) {
			PRINTK("%s buff copy and return.\n", __func__);
			ver[0] = k_val_buff[0];
			ver[1] = k_val_buff[1];
			ver[2] = k_val_buff[2];
			ver[3] = k_val_buff[3];
			ver[4] = k_val_buff[4];
			ver[5] = k_val_buff[5];
			return 0;
		}
	}

	g_time = currentTimeInNanoSec();

/* shmds mod -> */
#ifdef SHMDS_DETECT
	if ((shmds_detect_switch_flg == 1) && ((ami_sen_stat.mode & AMI_BIT_PEDO_MODE) == 0)) {
		if (shmds_init_flg == 0) {
			int i;
			shmds_init_flg = 1;
			shmds_buf_loop_flg = 0;
			shmds_count = 0;
			shmds_wakeup = 0;
			shmds_detect_mode_flg = SHMDS_DETECT_NORMAL;
			memset(shmds_economize_buf, 0, sizeof(shmds_economize_buf));
			
			shmds_buf_size = shmds_buf_size_temp;
			
			for (i = 0; i < 3; i++) {
				kfree(shmds_detect_buf[i]);
				shmds_detect_buf[i] = NULL;
			}
			
			for (i = 0; i < 3; i++) {
				shmds_detect_buf[i] = (s16 *)kmalloc(shmds_buf_size * sizeof(s16), GFP_KERNEL);
				if (NULL == shmds_detect_buf[i]) {
					return -ENOMEM;
				}
				memset(shmds_detect_buf[i], 0, shmds_buf_size * sizeof(s16));
			}
		}

		if (shmds_detect_mode_flg == SHMDS_DETECT_NORMAL) {
			/* READ DATA X,Y,Z */
			res = AMI_Read_RawData(client, ver);
			if (0 > res) {
				PRINTK_E("AMI_Read_RawData error : %d\n", res);
				return res;
			}
			
			if (shmds_buf_loop_flg == 0) {
				shmds_detect_buf[0][shmds_count] = ver[3];
				shmds_detect_buf[1][shmds_count] = ver[4];
				shmds_detect_buf[2][shmds_count] = ver[5];
				shmds_count++;
				
				if (shmds_count == (shmds_buf_size - 1)) {
					shmds_buf_loop_flg = 1;
				}
			}
			else {
				s16 max,min;
				int i,j;
				
				shmds_detect_buf[0][shmds_count] = ver[3];
				shmds_detect_buf[1][shmds_count] = ver[4];
				shmds_detect_buf[2][shmds_count] = ver[5];
				
				for (i = 0; i < 3; i++) {
					max = shmds_detect_buf[i][0];
					min = shmds_detect_buf[i][0];
					
					for (j = 1; j < shmds_buf_size; j++) {
						if (shmds_detect_buf[i][j] > max)
							max = shmds_detect_buf[i][j];
						if (shmds_detect_buf[i][j] < min)
							min = shmds_detect_buf[i][j];
					}
					sub[i] = max - min;
				}
				
				if ((sub[0] <= shmds_md_thresh1) && (sub[1] <= shmds_md_thresh1) && (sub[2] <= shmds_md_thresh1)) {
					if (down_interruptible(&shmds_sem))
						return -ERESTARTSYS;
					shmds_detect_mode_flg = SHMDS_DETECT_ECONOMIZE;
					up(&shmds_sem);
						
					shmds_economize_buf[0] = shmds_detect_buf[0][shmds_count];
					shmds_economize_buf[1] = shmds_detect_buf[1][shmds_count];
					shmds_economize_buf[2] = shmds_detect_buf[2][shmds_count];
					
					SHMDS_ami603_Start_SensorInterrupt();
				}
				
				shmds_count++;
				if (shmds_count == shmds_buf_size) {
					shmds_count = 0;
				}
			}
			
			k_val_buff[0] = ver[0];
			k_val_buff[1] = ver[1];
			k_val_buff[2] = ver[2];
			k_val_buff[3] = ver[3];
			k_val_buff[4] = ver[4];
			k_val_buff[5] = ver[5];
		}
		else if (shmds_detect_mode_flg == SHMDS_DETECT_ECONOMIZE) {
			int i;
			
			res = wait_event_interruptible_timeout(shmds_sleep_wq, shmds_wakeup, ((9*HZ)/5));
			
			if (shmds_wakeup == 1) {
				if (down_interruptible(&shmds_sem))
					return -ERESTARTSYS;
				shmds_wakeup = 0;
				shmds_detect_mode_flg = SHMDS_DETECT_NORMAL;
				up(&shmds_sem);
				
				if (shmds_close_flg == 0)
					SHMDS_ami603_Stop_SensorInterrupt();
				
				res = AMI_Read_RawData(client, ver);
				if (0 > res) {
					PRINTK_E("AMI_Read_RawData error : %d\n", res);
					return res;
				}
				
				shmds_detect_buf[0][shmds_count] = ver[3];
				shmds_detect_buf[1][shmds_count] = ver[4];
				shmds_detect_buf[2][shmds_count] = ver[5];
				shmds_count++;
				if (shmds_count == shmds_buf_size) {
					shmds_count = 0;
				}
			}
			else if (res == 0) {
				res = AMI_Read_RawData(client, ver);
				if (0 > res) {
					PRINTK_E("AMI_Read_RawData error : %d\n", res);
					return res;
				}
				
				shmds_detect_buf[0][shmds_count] = ver[3];
				shmds_detect_buf[1][shmds_count] = ver[4];
				shmds_detect_buf[2][shmds_count] = ver[5];
				shmds_count++;
				if (shmds_count == shmds_buf_size) {
					shmds_count = 0;
				}
				
				for (i = 3; i < 6; i++) {
					if (ver[i] > shmds_economize_buf[i-3]) {
						sub[i-3] = ver[i] - shmds_economize_buf[i-3];
					} 
					else {
						sub[i-3] = shmds_economize_buf[i-3] - ver[i];
					}
				}
				
				if ((sub[0] > shmds_md_thresh2) || (sub[1] > shmds_md_thresh2) || (sub[2] > shmds_md_thresh2)) {
					if (down_interruptible(&shmds_sem))
						return -ERESTARTSYS;
					shmds_detect_mode_flg = SHMDS_DETECT_NORMAL;
					up(&shmds_sem);
					
					if (shmds_close_flg == 0)
						SHMDS_ami603_Stop_SensorInterrupt();
				}
			}
				
			k_val_buff[0] = ver[0];
			k_val_buff[1] = ver[1];
			k_val_buff[2] = ver[2];
			k_val_buff[3] = ver[3];
			k_val_buff[4] = ver[4];
			k_val_buff[5] = ver[5];
		}
	}
	else {
		if (shmds_close_flg == 0)
			SHMDS_ami603_Stop_SensorInterrupt();
		
		if (shmds_init_flg == 1)
			shmds_init_flg = 0;
		
		/* READ DATA X,Y,Z */
		res = AMI_Read_RawData(client, ver);
		if (0 > res) {
			PRINTK_E("AMI_Read_RawData error : %d\n", res);
			return res;
		}

		PRINTK("%s copy to buffer.\n", __func__);

		k_val_buff[0] = ver[0];
		k_val_buff[1] = ver[1];
		k_val_buff[2] = ver[2];
		k_val_buff[3] = ver[3];
		k_val_buff[4] = ver[4];
		k_val_buff[5] = ver[5];
	}
#else /* SHMDS_DETECT */
	/* READ DATA X,Y,Z */
	res = AMI_Read_RawData(client, ver);
	if (0 > res) {
		PRINTK_E("AMI_Read_RawData error : %d\n", res);
		return res;
	}

	PRINTK("%s copy to buffer.\n", __func__);

	k_val_buff[0] = ver[0];
	k_val_buff[1] = ver[1];
	k_val_buff[2] = ver[2];
	k_val_buff[3] = ver[3];
	k_val_buff[4] = ver[4];
	k_val_buff[5] = ver[5];
#endif /* SHMDS_DETECT */
/* shmds mod <- */

	return 0;
}

/*--------------------------------------------------------------*/
static int AMI_Mea(s16 val[6], u8 mode, int flag)
{
	int res;

	PRINTK("%s mode : %d\n", __FUNCTION__, mode);

	if (mode == AMI_FORCE_MODE) {
		res = AMI_ForceMesurement(ami_i2c_client, val);
	} else {
		res = AMI_NormalMesurement(ami_i2c_client, val, flag);
	}
	if (0 > res) {
		if (mode == AMI_FORCE_MODE) {
			PRINTK("AMI_ForceMesurement error : %d\n", res);
		} else {
			PRINTK("AMI_NormalMesurement error : %d\n", res);
		}
		return res;
	}
	val[0] += AMI_STANDARD_OFFSET;
	val[1] += AMI_STANDARD_OFFSET;
	val[2] += AMI_STANDARD_OFFSET;
	val[3] += AMI_STANDARD_OFFSET;
	val[4] += AMI_STANDARD_OFFSET;
	val[5] += AMI_STANDARD_OFFSET;
	return res;
}

/*--------------------------------------------------------------*/
static int AMI_ReadSensorData(struct i2c_client *client,
			      struct ami_sensor_rawvalue *val, u8 mode)
{
	int res = 0;
	s16 m_val[6];

	if (FALSE == ami_sen_stat.mode)
		return -EFAULT;

	res = AMI_Mea(m_val, mode, 0);
	if (0 > res) {
		PRINTK_E("AMI_Mea error : %d\n", res);
		return res;
	}

	val->mx = 0x0FFF & ~m_val[0];
	val->my = 0x0FFF & ~m_val[1];
	val->mz = 0x0FFF & ~m_val[2];
	val->ax = m_val[3];
	val->ay = m_val[4];
	val->az = m_val[5];

	return AMI_Read_Temperature(ami_i2c_client, &val->temperature);
}

/*--------------------------------------------------------------*/
static int AMI_WritePedoDivision(struct i2c_client *client, uint32_t interval)
{
	int res = 0;

	u8 ctrl3;
	u8 ctrl4;

	if (interval == AMI_INTERVAL_20MS) {
		ctrl3 = 0x10;
		ctrl4 = 0x10;
	} else {
		ctrl3 = 0x50;
		ctrl4 = 0x00;
	}

	if (ami_sen_stat.mode & AMI_BIT_NORMAL_MODE
	    && interval == AMI_INTERVAL_40MS) {
		ctrl3 = 0x00;
		ctrl4 = 0x00;
	}

	res = AMI_i2c_send_b(client, AMI_DR_CTRL3, ctrl3);
	if (0 > res)
		return res;

	res = AMI_i2c_send_b(client, AMI_DR_CTRL4, ctrl4);
	if (0 > res)
		return res;

	return res;
}

/*--------------------------------------------------------------*/
static int AMI_ReadAccOffsetOPT(struct i2c_client *client, u16 dat[3])
{
	int ret = 0;
	u8 page = 0x0f;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	ret = AMI_i2c_recv_w(client, AMI_OTP_ORGAX, &dat[0]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_ORGAY, &dat[1]);
	ret = AMI_i2c_recv_w(client, AMI_OTP_ORGAZ, &dat[2]);
	page = 0x00;
	ret = AMI_i2c_send_b(client, AMI_REG_I2C_PAGE_NO, page);
	return ret;
}

/*--------------------------------------------------------------*/
static int AMI_StopSensor(struct i2c_client *client, int mode)
{
	int res;
	PRINTK("%s\n", __FUNCTION__);
	if (mode == AMI_FORCE_MODE) {
		res = AMI_AccCtrlOff(client);
		if (0 > res) {
			PRINTK_E("AMI_AccCtrlOff ERROR(%d)\n", res);
			return -EFAULT;
		}
		res = AMI_StopMesForceState(client);
		if (0 > res) {
			PRINTK_E("AMI_StopMesForceState ERROR(%d)\n", res);
			return -EFAULT;
		}
	} else {
		res = AMI_StopMesNormalState(client);
		if (0 > res) {
			PRINTK_E("AMI_StopMesNormalState ERROR(%d)\n", res);
			return -EFAULT;
		}
	}
	res = AMI_StopMes(client);
	if (0 > res) {
		PRINTK_E("AMI_StopMes ERROR(%d)\n", res);
		return -EFAULT;
	}
	
	res = AMI_Standby(client);
	if (0 > res) {
		PRINTK_E("AMI_Standby ERROR(%d)\n", res);
		return -EFAULT;
	}
	return 0;
}

/*============================================================================*/
/*======			   StartSensor    			====*/
/*============================================================================*/
/* 13-2.sequence */
#define AMI_FORCE_STATE_RETRY 	3
static int AMI_StartSensor(struct i2c_client *client, int mode,
			   uint32_t interval, int init)
{
	int res = 0;
	int i;

	PRINTK("%s mode : %d interval : %d\n", __FUNCTION__, mode, interval);

	if (mode == AMI_FORCE_MODE) {
		for (i = 0; i < AMI_FORCE_STATE_RETRY; i++) {
			/* Step 1 */
			res = AMI_ForceModeActive(client);
			if (0 > res) {
				PRINTK_E(KERN_ERR
				       "AMI_ForceModeActive ERROR(%d)\n", res);
				return res;
			}
			/* Step 2 */
			res = AMI_DRDY_Enable(client);
			if (0 > res) {
				PRINTK_E(KERN_ERR "AMI_DRDY_Enable ERROR(%d)\n",
				       res);
				return res;
			}
			/* Step 3 */
			if (init == 0) {
				res =
				    AMI_Write_Offset(client, ami_sen_stat.fine);
				if (0 > res) {
					PRINTK_E(KERN_ERR
					       "AMI_Write_Offset ERROR(%d)\n",
					       res);
					return res;
				}
			}
			/* Step 4 */
			res = AMI_AccCtrlOn(client);
			if (0 > res) {
				PRINTK_E(KERN_ERR "AMI_AccCtrlOn ERROR(%d)\n",
				       res);
				return res;
			}

			msleep(10);

			/* Write CNTL3:FORCE = 1 */
			res = AMI_StartMesForceState(client);
			if (0 > res) {
				PRINTK_E("AMI_StartMesForceState error : %d\n",
				       res);
				return res;
			}
			/* Wait DRDY high */
			res =
			    AMI_Wait_DataReady(client, AMI_DRDYWAIT,
					       AMI_WAIT_DATAREADY_RETRY);
			if (0 > res) {
				AMI_StopSensor(client, mode);
				PRINTK("Retry Force state count : %d\n", i + 1);
			} else {
				s16 buf[6];
				res = AMI_Read_RawData(client, buf);
				PRINTK("Force state OK count : %d\n", i + 1);
				break;
			}
		}
	} else if (mode == AMI_NORMAL_MODE) {
		/* Step 1 */
		res = AMI_ForceModeActive(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_ForceModeActive ERROR(%d)\n", res);
			return res;
		}
		/* Step 2 */
		res = AMI_DRDY_Enable(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_DRDY_Enable ERROR(%d)\n", res);
			return res;
		}
		/* Step 3 */
		res = AMI_Write_Offset(client, ami_sen_stat.fine);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_Write_Offset ERROR(%d)\n", res);
			return res;
		}
		/* Step 4 */
		res = AMI_StartMesNormalState(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_StartMesNormalState ERROR(%d)\n",
			       res);
			return res;
		}
		/* Step 5 */
		AMI_SetTickInterval(client, interval);
		/* Step 6 */
		res = AMI_NormalModeActive(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_NormalModeActive ERROR(%d)\n",
			       res);
			return res;
		}
	} else {
		return -EINVAL;
	}
	return 0;
}

/*--------------------------------------------------------------*/
static int AMI_SetDelay(struct i2c_client *client, int interval)
{
	int res = 0;
	k_start_param.interval = interval;
	if (ami_sen_stat.mode & AMI_BIT_PEDO_MODE) {
		if (interval == AMI_INTERVAL_80MS) {
			interval = AMI_INTERVAL_40MS;
		} else if (interval == AMI_INTERVAL_100MS
			   || interval == AMI_INTERVAL_60MS) {
			interval = AMI_INTERVAL_20MS;
		}

		res = AMI_WritePedoDivision(client, interval);
		if (0 > res) {
			PRINTK_E("AMI_WritePedoDivision error : %d\n", res);
			return -EFAULT;
		}
	}

	AMI_SetTickInterval(client, interval);
	g_interval = interval;
	return 0;
}

/*--------------------------------------------------------------*/
static int AMI_StartPedo(struct i2c_client *client)
{
	int res = 0;
	uint32_t interval;

	if (ami_sen_stat.mode & AMI_BIT_FORCE_MODE
	    || ami_sen_stat.mode & AMI_BIT_PEDO_MODE) {
		PRINTK_E("sensor mode error. (mode:force state) \n");
		return -EFAULT;
	}

	if (FALSE == ami_sen_stat.mode) {
		/* Step 1 */
		res = AMI_ForceModeActive(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_ForceModeActive ERROR(%d)\n", res);
			return res;
		}
		/* Step 2 */
		res = AMI_DRDY_Enable(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_DRDY_Enable ERROR(%d)\n", res);
			return res;
		}
		/* Step 3 */
		res = AMI_Write_Offset(client, ami_sen_stat.fine);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_Write_Offset ERROR(%d)\n", res);
			return res;
		}
		/* Step 6 */
		res = AMI_NormalModeActive(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_NormalModeActive ERROR(%d)\n",
			       res);
			return res;
		}
		k_start_param.interval = AMI_INTERVAL_40MS;
	}

	interval = k_start_param.interval;
	if (interval == AMI_INTERVAL_80MS) {
		interval = AMI_INTERVAL_40MS;
	} else if (interval == AMI_INTERVAL_100MS
		   || interval == AMI_INTERVAL_60MS) {
		interval = AMI_INTERVAL_20MS;
	}
	AMI_SetTickInterval(client, interval);

	g_interval = interval;
	res = AMI_WritePedoDivision(ami_i2c_client, interval);
	if (0 > res) {
		PRINTK_E("AMI_WritePedoDivision error : %d\n", res);
		return -EFAULT;
	}

	res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x10);
	if (0 > res) {
		PRINTK_E
		    ("AMI_Up_RegisterBit8 AMI_REG_CNTL1 PEDO = 0 error : %d\n",
		     res);
		return -EFAULT;
	}

	return 0;
}

/*--------------------------------------------------------------*/
static int AMI_ResetPedoStatus(struct i2c_client *client)
{
	u8 dat;
	return AMI_i2c_recv_b(client, AMI_RST_STATUS, &dat);
}

/*--------------------------------------------------------------*/
static int AMI_StopPedo(struct i2c_client *client)
{
	int res = 0;
	res = AMI_ResetPedoStatus(client);
	res = AMI_Dn_RegisterBit8(client, AMI_REG_CNTL1, 0x10);

	return res;
}

/*--------------------------------------------------------------*/
static int AMI_Read_PedoThreshold(struct i2c_client *client,
				  struct ami_pedo_threshold *pedo)
{
	int res = 0;
	u8 dat8;

	res = AMI_i2c_recv_b(client, AMI_REST_OUT_TH, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->rest_out_th = dat8;
	res = AMI_i2c_recv_b(client, AMI_REST_IN_TIME, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->rest_in_time = dat8;
	res = AMI_i2c_recv_b(client, AMI_REST_IN_CNT, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->rest_in_cnt = dat8;
	res = AMI_i2c_recv_b(client, AMI_REST_IN_TH, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->rest_in_th = dat8;
	res = AMI_i2c_recv_b(client, AMI_STEP_UP_TH, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->step_up_th = dat8;
	res = AMI_i2c_recv_b(client, AMI_STEP_DW_TH, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->step_dw_th = dat8;
	res = AMI_i2c_recv_b(client, AMI_STEP_MS, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->step_min_time = dat8;
	res = AMI_i2c_recv_b(client, AMI_STEP_STOP_MS, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->step_stop_time = dat8;
	res = AMI_i2c_recv_b(client, AMI_IIR_WEIGHT1, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->iir_weight1 = dat8;
	res = AMI_i2c_recv_b(client, AMI_IIR_WEIGHT2, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->iir_weight2 = dat8;
	res = AMI_i2c_recv_b(client, AMI_IIR_WEIGHT3, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->iir_weight3 = dat8;
	res = AMI_i2c_recv_b(client, AMI_ZERO_CROSS_CHK, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->axis_chk_flg = dat8;
	res = AMI_i2c_recv_b(client, AMI_TWO_STEP_MIN, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->two_step_min = dat8;
	res = AMI_i2c_recv_b(client, AMI_TWO_STEP_MAX, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->two_step_max = dat8;
	res = AMI_i2c_recv_b(client, AMI_TWO_STEP_DEF, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->two_step_dif = dat8;
	res = AMI_i2c_recv_b(client, AMI_TWO_STEP_CHK, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->two_step_chk = dat8;

	return 0;
 errreturn:
	PRINTK_E("AMI_Read_PedoThreshold error : %d\n", res);
	return -EFAULT;
}

/*--------------------------------------------------------------*/
static int AMI_Write_PedoThreshold(struct i2c_client *client,
				   struct ami_pedo_threshold *pedo)
{
	int res = 0;
	u8 dat8;

	PRINTK("rest_out_th : %d\n", pedo->rest_out_th);
	PRINTK("rest_in_time : %d\n", pedo->rest_in_time);
	PRINTK("rest_in_cnt : %d\n", pedo->rest_in_cnt);
	PRINTK("rest_in_th : %d\n", pedo->rest_in_th);
	PRINTK("step_up_th : %d\n", pedo->step_up_th);
	PRINTK("step_dw_th : %d\n", pedo->step_dw_th);
	PRINTK("step_min_time : %d\n", pedo->step_min_time);
	PRINTK("step_stop_time : %d\n", pedo->step_stop_time);
	PRINTK("iir_weight1 : %d\n", pedo->iir_weight1);
	PRINTK("iir_weight2 : %d\n", pedo->iir_weight2);
	PRINTK("iir_weight3 : %d\n", pedo->iir_weight3);
	PRINTK("axis_chk_flg : %d\n", pedo->axis_chk_flg);
	PRINTK("two_step_min : %d\n", pedo->two_step_min);
	PRINTK("two_step_max : %d\n", pedo->two_step_max);
	PRINTK("two_step_dif : %d\n", pedo->two_step_dif);
	PRINTK("two_step_chk : %d\n", pedo->two_step_chk);

	dat8 = pedo->rest_out_th;
	res = AMI_i2c_send_b(client, AMI_REST_OUT_TH, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->rest_in_time;
	res = AMI_i2c_send_b(client, AMI_REST_IN_TIME, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->rest_in_cnt;
	res = AMI_i2c_send_b(client, AMI_REST_IN_CNT, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->rest_in_th;
	res = AMI_i2c_send_b(client, AMI_REST_IN_TH, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->step_up_th;
	res = AMI_i2c_send_b(client, AMI_STEP_UP_TH, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->step_dw_th;
	res = AMI_i2c_send_b(client, AMI_STEP_DW_TH, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->step_min_time;
	res = AMI_i2c_send_b(client, AMI_STEP_MS, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->step_stop_time;
	res = AMI_i2c_send_b(client, AMI_STEP_STOP_MS, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->iir_weight1;
	res = AMI_i2c_send_b(client, AMI_IIR_WEIGHT1, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->iir_weight2;
	res = AMI_i2c_send_b(client, AMI_IIR_WEIGHT2, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->iir_weight3;
	res = AMI_i2c_send_b(client, AMI_IIR_WEIGHT3, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->axis_chk_flg;
	res = AMI_i2c_send_b(client, AMI_ZERO_CROSS_CHK, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->two_step_min;
	res = AMI_i2c_send_b(client, AMI_TWO_STEP_MIN, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->two_step_max;
	res = AMI_i2c_send_b(client, AMI_TWO_STEP_MAX, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->two_step_dif;
	res = AMI_i2c_send_b(client, AMI_TWO_STEP_DEF, dat8);
	if (0 > res)
		goto errreturn;
	dat8 = pedo->two_step_chk;
	res = AMI_i2c_send_b(client, AMI_TWO_STEP_CHK, dat8);
	if (0 > res)
		goto errreturn;

	return 0;
 errreturn:
	PRINTK_E("AMI_Write_PedoThreshold error : %d\n", res);
	return -EFAULT;
}

/*--------------------------------------------------------------*/
static int AMI_Read_PedoStatus(struct i2c_client *client,
			       struct ami_pedo_status *pedo)
{
	int res = 0;
	int dat;
	u8 dat8;
	res = AMI_i2c_recv_dw(client, AMI_STATUS_CNT, &dat);
	if (0 > res)
		goto errreturn;
	pedo->count = dat;
	res = AMI_i2c_recv_dw(client, AMI_STATUS_TIME, &dat);
	if (0 > res)
		goto errreturn;
	pedo->time = dat;
	res = AMI_i2c_recv_b(client, AMI_STATUS_STAT, &dat8);
	if (0 > res)
		goto errreturn;
	pedo->stat = dat8;

	return 0;
 errreturn:
	PRINTK_E("AMI_Read_PedoStatus error : %d\n", res);
	return -EFAULT;
}

/*--------------------------------------------------------------*/

/*============================================================================*/
/*======			   B0 Adjustment    			====*/
/*============================================================================*/
/*---------------------------------------------------------------------------*/
static int AMI_InitialAdjustFine(struct i2c_client *client)
{
	int res = 0;
	u8 fine[3] = { 0 };
	s16 data[6];
	int diff[3] = { 0x7fff, 0x7fff, 0x7fff };
	int fn = 0, ax = 0;

	for (fn = 1; fn < AMI_FINE_MAX; ++fn) {	/* fine 0 -> 95 */
		fine[0] = fine[1] = fine[2] = fn;
		res = AMI_Write_Offset(client, fine);
		if (0 > res) {
			PRINTK_E("AMI_Write_Offset error. (%d)\n", res);
			return res;
		}

		res = AMI_Mea(data, k_start_param.mode, 0);

		data[0] -= AMI_STANDARD_OFFSET;
		data[1] -= AMI_STANDARD_OFFSET;
		data[2] -= AMI_STANDARD_OFFSET;

		if (0 > res) {
			PRINTK_E("AMI_Mea error. (%d) mode : %d\n", res,
			       k_start_param.mode);
			return res;
		}
		PRINTK("[%d] x:%-5d y:%-5d z:%-5d\n", fn, data[0], data[1],
		       data[2]);

		for (ax = 0; ax < 3; ax++) {
			/* search point most close to zero. */
			if (diff[ax] > abs(data[ax])) {
				ami_sen_stat.fine[ax] = fn;
				diff[ax] = abs(data[ax]);
			}
		}
	}
	PRINTK("fine x:%-5d y:%-5d z:%-5d\n", ami_sen_stat.fine[0],
	       ami_sen_stat.fine[1], ami_sen_stat.fine[2]);
	res = AMI_Write_Offset(client, ami_sen_stat.fine);
	if (0 > res)
		return res;

	return 0;
}

/*============================================================================*/
/*======			   Read window parameter		====*/
/*============================================================================*/
/*--------------------------------------------------------------*/
static int AMI_ReadWindowValue(struct i2c_client *client,
			       struct ami_win_parameter *win)
{
	int res = 0;
	u16 m_fineout[3];
	u8 fine[3];

	res = AMI_Read_OffsetOTP(client, fine);	/* OFFOTP */
	if (0 > res) {
		PRINTK_E("AMI_Read_OffsetOTP error : %d\n", res);
		return res;
	}
	win->m_0Gauss_fine.x = fine[0];
	win->m_0Gauss_fine.y = fine[1];
	win->m_0Gauss_fine.z = fine[2];

	res = AMI_Read_Offset(client, fine);	/*OFF */
	if (0 > res) {
		PRINTK_E("AMI_Read_Offset error : %d\n", res);
		return res;
	}
	win->m_fine.x = fine[0];
	win->m_fine.y = fine[1];
	win->m_fine.z = fine[2];

	res = AMI_FineOutput(client, m_fineout);	/*fine output */
	if (0 > res) {
		PRINTK_E("AMI_FineOutput error : %d\n", res);
		return res;
	}
	win->m_fine_output.x = m_fineout[0];
	win->m_fine_output.y = m_fineout[1];
	win->m_fine_output.z = m_fineout[2];

	return res;
}

/*============================================================================*/
/*======			   Write window parameter		====*/
/*============================================================================*/
static int AMI_WriteWindowValue(struct i2c_client *client,
				struct ami_win_parameter *win)
{
	/* fine set */
	ami_sen_stat.fine[0] = win->m_fine.x;
	ami_sen_stat.fine[1] = win->m_fine.y;
	ami_sen_stat.fine[2] = win->m_fine.z;
	return 0;
}

/*============================================================================*/
/*======			   Read SENSOR parameter		====*/
/*============================================================================*/
static int AMI_ReadParameters(struct i2c_client *client,
			      struct ami_sensor_parameter *param)
{
	int res = 0;
	u16 m_gainbuf[6];
	u8 m_intebuf[6];
	u16 offset[3];
/*	u16 dev; */

	res = AMI_Read_Sense(client, m_gainbuf);
	if (0 > res)
		return res;

	param->m_gain.x = m_gainbuf[0];
	param->m_gain.y = m_gainbuf[1];
	param->m_gain.z = m_gainbuf[2];
	param->a_gain.x = m_gainbuf[3] / 2;
	param->a_gain.y = m_gainbuf[4] / 2;
	param->a_gain.z = m_gainbuf[5] / 2;

	res = AMI_Read_GainPara(client, m_intebuf);
	if (0 > res)
		return res;

	param->m_interference.xy = m_intebuf[0];
	param->m_interference.xz = m_intebuf[1];
	param->m_interference.yx = m_intebuf[2];
	param->m_interference.yz = m_intebuf[3];
	param->m_interference.zx = m_intebuf[4];
	param->m_interference.zy = m_intebuf[5];

	param->m_offset.x = AMI_STANDARD_OFFSET;
	param->m_offset.y = AMI_STANDARD_OFFSET;
	param->m_offset.z = AMI_STANDARD_OFFSET;

	res = AMI_ReadAccOffsetOPT(client, offset);
	if (0 > res)
		return res;

	param->a_offset.x = offset[0] + AMI_STANDARD_OFFSET;
	param->a_offset.y = offset[1] + AMI_STANDARD_OFFSET;
	param->a_offset.z = offset[2] + AMI_STANDARD_OFFSET;

/*  TBD
	res = AMI_Read_Deviation(client, &dev);
	if (0 > res)
		return res;
	param->a_deviation = dev;
*/

	param->m_gain_cor.x = AMI_GAIN_COR_DEFAULT;
	param->m_gain_cor.y = AMI_GAIN_COR_DEFAULT;
	param->m_gain_cor.z = AMI_GAIN_COR_DEFAULT;

	return res;
}

/*============================================================================*/
/*======		  	         SearchOffset 	  		  ====*/
/*============================================================================*/
static int AMI_ChangeForceMode(struct i2c_client *client)
{
	int res;
	if( ami_sen_stat.mode & AMI_BIT_NORMAL_MODE ){
		res = AMI_StopMesNormalState(client);
		if (0 > res) {
			PRINTK_E("AMI_StopMesNormalState ERROR(%d)\n", res);
			goto ERROR;
		}
		res = AMI_StopMes(client);
		if (0 > res) {
			PRINTK_E("AMI_StopMes ERROR(%d)\n", res);
			goto ERROR;
		}
	}

	if( ami_sen_stat.mode & AMI_BIT_PEDO_MODE ){
		res = AMI_Dn_RegisterBit8(client, AMI_REG_CNTL1, 0x10);
		if (0 > res) {
			PRINTK_E("AMI_REG_CNTL1 ERROR(%d)\n", res);
			goto ERROR;
		}
	}

	if( !(ami_sen_stat.mode & AMI_BIT_FORCE_MODE) ){
		res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x40);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_REG_CNTL1 ERROR(%d)\n",
			       res);
			goto ERROR;
		}
		
		res = AMI_AccCtrlOn(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_AccCtrlOn ERROR(%d)\n",
			       res);
			goto ERROR;
		}

		msleep(10);
	}
	return 0;
 ERROR:
	return -1;
}

static int AMI_ReturnMode(struct i2c_client *client)
{
	int res;
	if( !(ami_sen_stat.mode & AMI_BIT_FORCE_MODE) ){
		res = AMI_AccCtrlOff(client);
		if (0 > res) {
			PRINTK_E("AMI_AccCtrlOff ERROR(%d)\n", res);
			return -EFAULT;
		}
		res = AMI_StopMesForceState(client);
		if (0 > res) {
			PRINTK_E("AMI_StopMesForceState ERROR(%d)\n", res);
			return -EFAULT;
		}
	}

	if( ami_sen_stat.mode & AMI_BIT_NORMAL_MODE ){
		res = AMI_StartMesNormalState(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_StartMesNormalState ERROR(%d)\n",
			       res);
			return res;
		}
		res = AMI_NormalModeActive(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_NormalModeActive ERROR(%d)\n",
			       res);
			return res;
		}
	}

	if( ami_sen_stat.mode & AMI_BIT_PEDO_MODE ){
		res = AMI_NormalModeActive(client);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI_NormalModeActive ERROR(%d)\n",
			       res);
			return res;
		}
#if 0 /* shmds mod */
		res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x10);
		if (0 > res) {
			PRINTK_E
			    ("AMI_Up_RegisterBit8 AMI_REG_CNTL1 PEDO = 0 error : %d\n",
			     res);
			return -EFAULT;
		}
#else /* shmds mod */
		if (shmds_pedopauseflag == 0) {
			res = AMI_Up_RegisterBit8(client, AMI_REG_CNTL1, 0x10);
			if (0 > res) {
				PRINTK_E
				    ("AMI_Up_RegisterBit8 AMI_REG_CNTL1 PEDO = 0 error : %d\n",
				     res);
				return -EFAULT;
			}
		}
#endif /* shmds mod */
	}
	return 0;
}

#define SEH_RANGE_MIN 100
#define SEH_RANGE_MAX 3950
static int AMI_SearchOffset(struct i2c_client *client)
{
	int res;
	int axis;
	u8 run_flg[3] = { 1, 1, 1 };
	u8 fine[3];
	u8 win_range_fine[3];
	u16 fine_output[3];
	s16 val[6];
	u16 cnt[3] = { 0 };

	if (FALSE == ami_sen_stat.mode)
		return -EFAULT;


	res = AMI_ChangeForceMode(client);
	if (0 > res)
		goto ERROR;

	res = AMI_FineOutput(client, fine_output);
	if (0 > res) {
		PRINTK_E("error AMI_FineOutput %d \n", __LINE__);
		goto ERROR;
	}

	for (axis = 0; axis < 3; ++axis) {
		if (fine_output[axis] == 0) {
			PRINTK("error fine_output %d  axis:%d \n", __LINE__,
			       axis);
			goto ERROR;
		}
		/* fines per a window */
		win_range_fine[axis] =
		    (SEH_RANGE_MAX - SEH_RANGE_MIN) / fine_output[axis];
	}

	/* get current fine */
	res = AMI_Read_Offset(client, fine);
	if (0 > res) {
		PRINTK_E("error fine_output %d \n", __LINE__);
		goto ERROR;
	}

	while (run_flg[0] == 1 || run_flg[1] == 1 || run_flg[2] == 1) {

		res = AMI_Mea(val, AMI_FORCE_MODE, 1);
		if (0 > res) {
			PRINTK_E("error fine_output %d \n", __LINE__);
			goto ERROR;
		}
		PRINTK("val  x:%-5d y:%-5d z:%-5d\n", val[0], val[1], val[2]);
		PRINTK("now fine x:%-5d y:%-5d z:%-5d\n", fine[0], fine[1],
		       fine[2]);

		for (axis = 0; axis < 3; ++axis) {
			if (axis == 2)	/* Z-axis is reversed */
				val[axis] = 0x0FFF & ~val[axis];

			/* At the case of less low limmit. */
			if (val[axis] < SEH_RANGE_MIN) {
				fine[axis] -= win_range_fine[axis];
				PRINTK("min : fine=%d diff=%d\n", fine[axis],
				       win_range_fine[axis]);
			}
			/* At the case of over high limmit. */
			if (val[axis] > SEH_RANGE_MAX) {
				fine[axis] += win_range_fine[axis];
				PRINTK("max : fine=%d diff=%d\n", fine[axis],
				       win_range_fine[axis]);
			}
			/* In the current window. */
			if (SEH_RANGE_MIN <= val[axis]
			    && val[axis] <= SEH_RANGE_MAX) {
				int diff_fine =
				    (val[axis] -
				     AMI_STANDARD_OFFSET) / fine_output[axis];
				fine[axis] += diff_fine;
				run_flg[axis] = 0;
				PRINTK("mid : fine=%d diff=%d\n", fine[axis],
				       diff_fine);
			}

			if (!(0 <= fine[axis] && fine[axis] < AMI_FINE_MAX)) {
				PRINTK_E(KERN_ERR "fine err :%d\n", cnt[axis]);
				goto ERROR;
			}
			if (cnt[axis] > 3) {
				PRINTK_E(KERN_ERR "cnt err :%d\n", cnt[axis]);
				goto ERROR;
			}
			cnt[axis]++;
		}
		PRINTK("new fine x:%-5d y:%-5d z:%-5d\n", fine[0], fine[1],
		       fine[2]);

		/* set current fine */
		res = AMI_Write_Offset(client, fine);
		if (0 > res) {
			PRINTK_E("AMI_Write_Offset error : %d\n", res);
			goto ERROR;
		}
	}
	memcpy(ami_sen_stat.fine, fine, sizeof(fine));

	res = AMI_ReturnMode(client);
	if (0 > res)
		return -EFAULT;

	return 0;
 ERROR:
	AMI_ReturnMode(client);
	return -EFAULT;
}

/*============================================================================*/
/*============================================================================*/
/*======                                Chip Information                  ====*/
/*============================================================================*/
/*============================================================================*/
/*---------------------------------------------------------------------------*/
static int AMI_ChipInformation(struct i2c_client *client,
			       struct ami_chipinfo *k_chip)
{
	int res;
	res = AMI_WhoIam(client, &k_chip->wia);
	if (res < 0)
		return res;

	res = AMI_GetVersion(client, &k_chip->ver);
	if (res < 0)
		return res;

	res = AMI_SerialNumber(client, &k_chip->sn);
	if (res < 0)
		return res;

	res = AMI_MoreInfo(client, &k_chip->info);
	if (res < 0)
		return res;

	return 0;
}

/*============================================================================*/
/*============================================================================*/
/*======                              Driver Information                  ====*/
/*============================================================================*/
/*============================================================================*/
/*---------------------------------------------------------------------------*/
static int AMI_DriverInformation(struct i2c_client *client,
				 struct ami_driverinfo *k_drv)
{
	k_drv->ver_major = THIS_VER_MAJOR;
	k_drv->ver_middle = THIS_VER_MIDDLE;
	k_drv->ver_minor = THIS_VER_MINOR;
	sprintf(k_drv->datetime, "%s %s", __DATE__, __TIME__);
	sprintf(k_drv->remarks, "%s", THIS_CODE_REMARKS);
	return 0;
}

/*-===========================================================================
 *
 *		M  I  D  D  L  E     &     D  A  E  M  O  N
 * 
 *-=========================================================================-*/

/*---------------------------------------------------------------------------*/
static int ami_open(struct inode *inode, struct file *file)
{
	PRINTK("AMI : Open device \n");

	init_MUTEX(&g_mutex);

	return 0;		/* force zero */
}

/*---------------------------------------------------------------------------*/
static int ami_release(struct inode *inode, struct file *file)
{
	PRINTK("AMI : Release device\n");
	return 0;		/* force zero */
}

/*---------------------------------------------------------------------------*/

static void print_win(void)
{
	PRINTK("Msensor fine     value mx = %d my = %d mz = %d\n",
	       k_win.m_fine.x, k_win.m_fine.y, k_win.m_fine.z);
	PRINTK("Msensor fine_out value mx = %d my = %d mz = %d\n",
	       k_win.m_fine_output.x, k_win.m_fine_output.y,
	       k_win.m_fine_output.z);
	PRINTK("Msensor 0G_fine  value mx = %d my = %d mz = %d\n",
	       k_win.m_0Gauss_fine.x, k_win.m_0Gauss_fine.y,
	       k_win.m_0Gauss_fine.z);
}

static void print_param(void)
{
	PRINTK("Msensor gain    value mx = %d my = %d mz = %d\n",
	       k_param.m_gain.x, k_param.m_gain.y, k_param.m_gain.z);
	PRINTK("Msensor offset  value mx = %d my = %d mz = %d\n",
	       k_param.m_offset.x, k_param.m_offset.y, k_param.m_offset.z);
	PRINTK
	    ("Msensor interference"
	    "  value xy = %d xz = %d yx = %d yz = %d zx = %d zy = %d\n",
	     k_param.m_interference.xy, k_param.m_interference.xz,
	     k_param.m_interference.yx, k_param.m_interference.yz,
	     k_param.m_interference.zx, k_param.m_interference.zy);
}

static void print_val(void)
{
	PRINTK("Msensor raw value mx = %d my = %d mz = %d\n", k_val.mx,
	       k_val.my, k_val.mz);
}

static void print_chip(void)
{
	PRINTK
	    ("Chip Information WhoIam = %02x VER = %d S/N = %d MoreInfo = %d\n",
	     k_chip.wia, k_chip.ver, k_chip.sn, k_chip.info);
}

static void print_drv(void)
{
	PRINTK
	    ("Driver Information "
	    "Ver = %d.%d.%d  Compliled = %s  Remarks = %s\n",
	     k_drv.ver_major, k_drv.ver_middle, k_drv.ver_minor, k_drv.datetime,
	     k_drv.remarks);
}

/* shmds add -> */
static int SHMDS_PedoMutexDown(void)
{
	if (down_interruptible(&shmds_pedomutex)) {
		PRINTK("pedo:down_interruptible\n");
		return -EFAULT;
	}
	return 0;
}

static void SHMDS_pedoMutexUp(void)
{
	up(&shmds_pedomutex);
}

void SHMDS_Pedometer_ReStart(void)
{
	if (SHMDS_PedoMutexDown()) {
		return;
	}
	if ((ami_sen_stat.mode & AMI_BIT_PEDO_MODE) && (shmds_pedopauseflag == 1)) {
		AMI_Up_RegisterBit8(ami_i2c_client, AMI_REG_CNTL1, 0x10);
	}
	shmds_pedopauseflag = 0;
	SHMDS_pedoMutexUp();
}
EXPORT_SYMBOL(SHMDS_Pedometer_ReStart);

void SHMDS_Pedometer_Pause(void)
{
	if (SHMDS_PedoMutexDown()) {
		return;
	}
	if(ami_sen_stat.mode == AMI_BIT_PEDO_MODE){
		AMI_Dn_RegisterBit8(ami_i2c_client, AMI_REG_CNTL1, 0x10);
	} else if (ami_sen_stat.mode & AMI_BIT_PEDO_MODE) {
		AMI_StopMesNormalState(ami_i2c_client);
		AMI_Standby(ami_i2c_client);
		AMI_Dn_RegisterBit8(ami_i2c_client, AMI_REG_CNTL1, 0x10);
		AMI_Active(ami_i2c_client);
		AMI_StartMesNormalState(ami_i2c_client);
		AMI_NormalModeActive(ami_i2c_client);
	}
	shmds_pedopauseflag = 1;
	SHMDS_pedoMutexUp();
}
EXPORT_SYMBOL(SHMDS_Pedometer_Pause);
/* shmds add <- */

static int debugRead(unsigned int cmd, void __user * argp);
static int debugWrite(unsigned int cmd, void __user * argp);
static int ami_private_ioctl(struct inode *inode, struct file *file,
			     unsigned int cmd, unsigned long arg)
{
	int res = 0;
	u8 fine[3];

	void __user *argp = (void __user *)arg;
	u8 i; /* shmds add */

	switch (cmd) {
	case AMI_IOCTL_INITIALIZE_SENSOR:
#if 0 /* shmds mod */
		if (FALSE == ami_sen_stat.mode) {
			res =
			    AMI_StartSensor(ami_i2c_client, AMI_FORCE_MODE,
					    AMI_INTERVAL_40MS, 1);
			if (0 > res) {
				PRINTK_E(KERN_ERR "AMI_StartSensor ERROR(%d)\n",
				       res);
				return res;
			}
			k_start_param.mode = AMI_FORCE_MODE;
		} else {
			return -EFAULT;
		}
		res = AMI_InitialAdjustFine(ami_i2c_client);
		if (0 > res)
			return -EFAULT;
#else /* shmds mod */
		for (i=0; i<3; i++) {
			if (FALSE == ami_sen_stat.mode) {
				res =
				    AMI_StartSensor(ami_i2c_client, AMI_FORCE_MODE,
					    AMI_INTERVAL_40MS, 1);
				if (0 > res) {
					PRINTK_E(KERN_ERR "AMI_StartSensor ERROR(%d)\n",
					       res);
					return res;
				}
				k_start_param.mode = AMI_FORCE_MODE;
			}
			res = AMI_InitialAdjustFine(ami_i2c_client);
			if (0 == res) {
				break;
			}
			AMI_Standby(ami_i2c_client);
		}
		if (0 > res) {
			AMI_SRSTset(ami_i2c_client);
		}
#endif /* shmds mod */

		res = AMI_ReadWindowValue(ami_i2c_client, &k_win);
		if (0 > res)
			return -EFAULT;

		print_win();
		if (copy_to_user(argp, &k_win, sizeof k_win))
			return -EFAULT;

		if (FALSE == ami_sen_stat.mode)
			AMI_StopSensor(ami_i2c_client, k_start_param.mode);

		break;
	case AMI_IOCTL_READ_WINPARAMS:

		if (FALSE == ami_sen_stat.mode)
			AMI_Active(ami_i2c_client);

		res = AMI_ReadWindowValue(ami_i2c_client, &k_win);
		if (0 > res)
			return -EFAULT;

		print_win();
		if (copy_to_user(argp, &k_win, sizeof k_win))
			return -EFAULT;

		if (FALSE == ami_sen_stat.mode)
			AMI_Standby(ami_i2c_client);

		break;
	case AMI_IOCTL_WRITE_WINPARAMS:
		if (FALSE == ami_sen_stat.mode)
			AMI_Active(ami_i2c_client);

		if (copy_from_user
		    (&k_win, argp, sizeof k_win)) {
			PRINTK("copy error.\n");
			return -EFAULT;
		}
		
		fine[0] = k_win.m_fine.x;
		fine[1] = k_win.m_fine.y;
		fine[2] = k_win.m_fine.z;
		res = AMI_Write_Offset(ami_i2c_client, fine);
		if (0 > res) {
			PRINTK_E("AMI_Write_Offset error : %d\n", res);
			return -EFAULT;
		}

		if (FALSE == ami_sen_stat.mode)
			AMI_Standby(ami_i2c_client);
		
		break;

	case AMI_IOCTL_START_SENSOR:
		{
			uint32_t interval;
			k_initflag = 1;
			if (copy_from_user
			    (&k_start_param, argp, sizeof k_start_param)) {
				PRINTK("copy error.\n");
				return -EFAULT;
			}

			if (k_start_param.mode != AMI_FORCE_MODE
			    && k_start_param.mode != AMI_NORMAL_MODE) {
				PRINTK("is stating error.\n");
				return -EFAULT;
			}

			if (ami_sen_stat.mode & AMI_BIT_FORCE_MODE
			    || ami_sen_stat.mode & AMI_BIT_NORMAL_MODE) {
				PRINTK("modo error. %x(%d)\n",
				       ami_sen_stat.mode, ami_sen_stat.mode);
				return -EFAULT;
			}

			if (k_start_param.mode == AMI_FORCE_MODE
			    && ami_sen_stat.mode & AMI_BIT_PEDO_MODE) {
				PRINTK("force pedo error.\n");
				return -EFAULT;
			}

			k_win = k_start_param.window;
			print_win();
			res = AMI_WriteWindowValue(ami_i2c_client, &k_win);
			if (0 > res)
				return -EFAULT;

			if (k_start_param.mode == AMI_FORCE_MODE) {
				AMI_Up_Mode(AMI_BIT_FORCE_MODE);
			} else if (k_start_param.mode == AMI_NORMAL_MODE) {
				AMI_Up_Mode(AMI_BIT_NORMAL_MODE);
			}

			interval = k_start_param.interval;
			if (ami_sen_stat.mode & AMI_BIT_PEDO_MODE) {
				if (interval == AMI_INTERVAL_80MS) {
					interval = AMI_INTERVAL_40MS;
				} else if (interval == AMI_INTERVAL_100MS
					   || interval == AMI_INTERVAL_60MS) {
					interval = AMI_INTERVAL_20MS;
				}
				AMI_SetTickInterval(ami_i2c_client, interval);
				res =
				    AMI_WritePedoDivision(ami_i2c_client,
							  interval);
				if (0 > res) {
					PRINTK_E
					    ("AMI_WritePedoDivision error : %d\n",
					     res);
					return -EFAULT;
				}
			}

			g_interval = interval;
			res =
			    AMI_StartSensor(ami_i2c_client, k_start_param.mode,
					    interval, 0);
			if (0 > res)
				return -EFAULT;

/* shmds add -> */
			if (!(ami_sen_stat.mode & AMI_BIT_PEDO_MODE)) {
				shterm_k_set_info(SHTERM_INFO_ACCELE, 1);
			}
			shterm_k_set_info(SHTERM_INFO_COMPS, 1);
/* shmds add <- */
		}
		break;

	case AMI_IOCTL_STOP_SENSOR:
		/* stop sensor command has no inputs/outputs.  */
		AMI_Dn_Mode(AMI_BIT_FORCE_MODE | AMI_BIT_NORMAL_MODE);
		if (ami_sen_stat.mode & AMI_BIT_PEDO_MODE) {
			res =
			    AMI_SetTickInterval(ami_i2c_client,
						AMI_INTERVAL_40MS);
			if (0 > res)
				return -EFAULT;

			res =
			    AMI_WritePedoDivision(ami_i2c_client,
						  AMI_INTERVAL_40MS);

			g_interval = AMI_INTERVAL_40MS;
			if (0 > res) {
				PRINTK("AMI_WritePedoDivision error : %d\n", res);
				return -EFAULT;
			}
			res = AMI_StopMesNormalState(ami_i2c_client);
			if (0 > res) {
				PRINTK_E("AMI_StopMesNormalState ERROR(%d)\n", res);
				return -EFAULT;
			}
		} else {
			res =
			    AMI_StopSensor(ami_i2c_client, k_start_param.mode);
			if (0 > res)
				return -EFAULT;

		}
		PRINTK("mode is %x(%d)\n", ami_sen_stat.mode,
		       ami_sen_stat.mode);

/* shmds add -> */
		if(FALSE == ami_sen_stat.mode) {
			shterm_k_set_info(SHTERM_INFO_ACCELE, 0);
		}
		shterm_k_set_info(SHTERM_INFO_COMPS, 0);
#ifdef SHMDS_DETECT
		if (shmds_close_flg == 0)
			SHMDS_ami603_Stop_SensorInterrupt();
		
		if (shmds_init_flg == 1)
			shmds_init_flg = 0;
#endif
/* shmds add <- */
		break;

	case AMI_IOCTL_SEARCH_OFFSET:

		/*temporarily search offset is user trigger. */
		res = AMI_SearchOffset(ami_i2c_client);
		if (0 > res)
			return -EFAULT;

		res = AMI_ReadWindowValue(ami_i2c_client, &k_win);
		if (0 > res)
			return -EFAULT;

		print_win();
		if (copy_to_user(argp, &k_win, sizeof k_win))
			return -EFAULT;

		break;

	case AMI_IOCTL_READ_PARAMS:

		if (FALSE == ami_sen_stat.mode)
			AMI_Active(ami_i2c_client);

		res = AMI_ReadParameters(ami_i2c_client, &k_param);
		if (0 > res)
			return -EFAULT;

		print_param();
		if (FALSE == ami_sen_stat.mode)
			AMI_Standby(ami_i2c_client);

		if (copy_to_user(argp, &k_param, sizeof k_param))
			return -EFAULT;

		break;

	case AMI_IOCTL_GET_SENSORRAWVALUE:
/* shmds add -> */
#ifdef SHMDS_DETECT
		shmds_detect_switch_flg = 0;
#endif
/* shmds add <- */

		res =
		    AMI_ReadSensorData(ami_i2c_client, &k_val,
				       k_start_param.mode);
		if (0 > res)
			return -EFAULT;

		print_val();
		if (copy_to_user(argp, &k_val, sizeof k_val))
			return -EFAULT;

		break;

/* shmds add -> */
#ifdef SHMDS_DETECT
	case AMI_IOCTL_GET_SENSORRAWVALUE2:
		shmds_detect_switch_flg = 1;

		res =
		    AMI_ReadSensorData(ami_i2c_client, &k_val,
				       k_start_param.mode);
		if (0 > res)
			return -EFAULT;

		print_val();
		if (copy_to_user(argp, &k_val, sizeof k_val))
			return -EFAULT;

		break;
		
	case AMI_IOCTL_WRITE_TIMER:
		if (copy_from_user(&shmds_timer, argp, sizeof(shmds_timer))) {
				PRINTK("copy error.\n");
				return -EFAULT;
		}
		break;
	case AMI_IOCTL_WRITE_THRESH:
		if (copy_from_user(&shmds_thresh, argp, sizeof(shmds_thresh))) {
				PRINTK("copy error.\n");
				return -EFAULT;
		}
		break;
	case AMI_IOCTL_WRITE_BUF_SIZE:
		if (copy_from_user(&shmds_buf_size_temp, argp, sizeof(shmds_buf_size_temp))) {
				PRINTK("copy error.\n");
				return -EFAULT;
		}
		if (shmds_buf_size_temp <= 0) {
			shmds_buf_size_temp = 10;
		}
		break;
	case AMI_IOCTL_WRITE_MD_THRESH1:
		if (copy_from_user(&shmds_md_thresh1, argp, sizeof(shmds_md_thresh1))) {
				PRINTK("copy error.\n");
				return -EFAULT;
		}
		break;
	case AMI_IOCTL_WRITE_MD_THRESH2:
		if (copy_from_user(&shmds_md_thresh2, argp, sizeof(shmds_md_thresh2))) {
				PRINTK("copy error.\n");
				return -EFAULT;
		}
		break;
#endif
/* shmds add <- */

	case AMI_IOCTL_SET_DELAY:
		{
			int interval = 0;
			if (copy_from_user(&interval, argp, sizeof interval)) {
				PRINTK("copy error.\n");
				return -EFAULT;
			}

			res = AMI_SetDelay(ami_i2c_client, interval);
			if (0 > res)
				return -EFAULT;
		}
		break;

	case AMI_IOCTL_CHIPINFO:

		if (FALSE == ami_sen_stat.mode)
			AMI_Active(ami_i2c_client);

		res = AMI_ChipInformation(ami_i2c_client, &k_chip);
		if (0 > res)
			return -EFAULT;

		print_chip();
		if (FALSE == ami_sen_stat.mode)
			AMI_Standby(ami_i2c_client);

		if (copy_to_user(argp, &k_chip, sizeof k_chip))
			return -EFAULT;

		break;

	case AMI_IOCTL_DRIVERINFO:

		res = AMI_DriverInformation(ami_i2c_client, &k_drv);
		if (0 > res)
			return -EFAULT;

		print_drv();
		if (copy_to_user(argp, &k_drv, sizeof k_drv))
			return -EFAULT;

		break;
	case AMI_IOCTL_START_PEDO:
		if (FALSE == ami_sen_stat.mode) {
			if (copy_from_user(&k_win, argp, sizeof k_win))
				return -EFAULT;

			print_win();
			res = AMI_WriteWindowValue(ami_i2c_client, &k_win);
			if (0 > res)
				return -EFAULT;
		}

		res = AMI_StartPedo(ami_i2c_client);
		if (0 > res)
			return -EFAULT;

		AMI_Up_Mode(AMI_BIT_PEDO_MODE);
		PRINTK("mode is %x(%d)\n", ami_sen_stat.mode,
		       ami_sen_stat.mode);

/* shmds add -> */
		shmds_pedopauseflag = 0;

		if (!(ami_sen_stat.mode & AMI_BIT_FORCE_MODE
		    || ami_sen_stat.mode & AMI_BIT_NORMAL_MODE)) {
			shterm_k_set_info(SHTERM_INFO_ACCELE, 1);
		}
/* shmds add <- */

		break;
	case AMI_IOCTL_STOP_PEDO:
		if (ami_sen_stat.mode == AMI_BIT_PEDO_MODE) {
			res = AMI_StopPedo(ami_i2c_client);
			if (0 > res)
				return -EFAULT;

			res = AMI_StopMes(ami_i2c_client);
			if (0 > res)
				return -EFAULT;
			
			res = AMI_Standby(ami_i2c_client);
			if (0 > res)
				return -EFAULT;
		} else {
			res = AMI_Standby(ami_i2c_client);
			if (0 > res)
				return -EFAULT;
			
			res = AMI_StopPedo(ami_i2c_client);
			if (0 > res)
				return -EFAULT;

			res = AMI_Active(ami_i2c_client);
			if (0 > res)
				return -EFAULT;

			res = AMI_NormalModeActive(ami_i2c_client);
			if (0 > res)
				return -EFAULT;

			res =
			    AMI_SetTickInterval(ami_i2c_client,
						k_start_param.interval);
			g_interval = k_start_param.interval;

			if (0 > res)
				return -EFAULT;
		}
		AMI_Dn_Mode(AMI_BIT_PEDO_MODE);
		PRINTK("mode is %x(%d)\n", ami_sen_stat.mode,
		       ami_sen_stat.mode);

/* shmds add -> */
		if(FALSE == ami_sen_stat.mode) {
			shterm_k_set_info(SHTERM_INFO_ACCELE, 0);
		}
/* shmds add <- */

		break;
	case AMI_IOCTL_SET_PEDO_SH:
		{
			if (copy_from_user
			    (&g_pedoThreshold, argp, sizeof g_pedoThreshold))
				return -EFAULT;

			if (FALSE == ami_sen_stat.mode)
				AMI_Active(ami_i2c_client);

			res =
			    AMI_Write_PedoThreshold(ami_i2c_client,
						    &g_pedoThreshold);
			if (0 > res)
				return -EFAULT;

			if (FALSE == ami_sen_stat.mode)
				AMI_Standby(ami_i2c_client);
		}
		break;
	case AMI_IOCTL_GET_PEDO_SH:
		{
			if (FALSE == ami_sen_stat.mode)
				AMI_Active(ami_i2c_client);

			res =
			    AMI_Read_PedoThreshold(ami_i2c_client,
						   &g_pedoThreshold);
			if (0 > res)
				return -EFAULT;

			if (FALSE == ami_sen_stat.mode)
				AMI_Standby(ami_i2c_client);

			if (copy_to_user
			    (argp, &g_pedoThreshold, sizeof g_pedoThreshold))
				return -EFAULT;
		}
		break;
	case AMI_IOCTL_GET_PEDO_STATUS:
		{
			res =
			    AMI_Read_PedoStatus(ami_i2c_client, &g_pedoStatus);
			if (0 > res)
				return -EFAULT;

			if (copy_to_user
			    (argp, &g_pedoStatus, sizeof g_pedoStatus))
				return -EFAULT;
		}
		break;

	case AMI_IOCTL_CLEAR_PEDO:
		{
			res = AMI_ResetPedoStatus(ami_i2c_client);
			if (0 > res)
				return -EFAULT;
		}
		break;
	case AMI_IOCTL_DBG_READ:
	case AMI_IOCTL_DBG_READ_W:
	case AMI_IOCTL_DBG_READ_DW:
	case AMI_IOCTL_DBG_READ_PD:
	case AMI_IOCTL_DBG_READ_PI:
		return debugRead(cmd, argp);

	case AMI_IOCTL_DBG_WRITE:
	case AMI_IOCTL_DBG_WRITE_W:
	case AMI_IOCTL_DBG_WRITE_DW:
		return debugWrite(cmd, argp);

/* shmds add -> */
	/* shmds test mode */
	case AMI_IOCTL_SHMDS_RESETSENSOR:
		res = AMI_SRSTset(ami_i2c_client);
		if (0 > res)
			return -EFAULT;

		memset(&k_start_param, 0, sizeof(k_start_param));
		ami_sen_stat.mode = 0;
		memset(&g_pedoStatus, 0, sizeof(g_pedoStatus));
		break;

	case AMI_IOCTL_SHMDS_READPARAMS:
		{
			u16 m_gainbuf[6];
			u8 m_intebuf[6];
			u16 offset[3];
			struct ami_sensor_parameter param;

			if (shmds_testmodeflag == 0) {
				shmds_testmodeflag = 1;
			}

			if (FALSE == ami_sen_stat.mode)
				AMI_Active(ami_i2c_client);

			res = AMI_Read_Sense(ami_i2c_client, m_gainbuf);
			if (0 > res)
				return -EFAULT;

			param.m_gain.x = m_gainbuf[0];
			param.m_gain.y = m_gainbuf[1];
			param.m_gain.z = m_gainbuf[2];
			param.a_gain.x = m_gainbuf[3];
			param.a_gain.y = m_gainbuf[4];
			param.a_gain.z = m_gainbuf[5];

			res = AMI_Read_GainPara(ami_i2c_client, m_intebuf);
			if (0 > res)
				return -EFAULT;

			param.m_interference.xy = m_intebuf[0];
			param.m_interference.xz = m_intebuf[1];
			param.m_interference.yx = m_intebuf[2];
			param.m_interference.yz = m_intebuf[3];
			param.m_interference.zx = m_intebuf[4];
			param.m_interference.zy = m_intebuf[5];

			param.m_offset.x = 0;
			param.m_offset.y = 0;
			param.m_offset.z = 0;

			res = AMI_ReadAccOffsetOPT(ami_i2c_client, offset);
			if (0 > res)
				return -EFAULT;

			param.a_offset.x = offset[0];
			param.a_offset.y = offset[1];
			param.a_offset.z = offset[2];

			param.m_gain_cor.x = AMI_GAIN_COR_DEFAULT;
			param.m_gain_cor.y = AMI_GAIN_COR_DEFAULT;
			param.m_gain_cor.z = AMI_GAIN_COR_DEFAULT;

			if (FALSE == ami_sen_stat.mode)
				AMI_Standby(ami_i2c_client);

			if (copy_to_user(argp, &param, sizeof param))
				return -EFAULT;
		}
		break;

	case AMI_IOCTL_SHMDS_GETRAWVALUE:
		{
			signed short val[6];

			if (FALSE == ami_sen_stat.mode)
				return -EFAULT;

			if (k_start_param.mode == AMI_FORCE_MODE) {
				res = AMI_ForceMesurement(ami_i2c_client, val);
			} else {
				res = AMI_NormalMesurement(ami_i2c_client, val, 0);
			}
			if (0 > res) {
				if (k_start_param.mode == AMI_FORCE_MODE) {
					PRINTK("AMI_ForceMesurement error : %d\n", res);
				} else {
					PRINTK("AMI_NormalMesurement error : %d\n", res);
				}
				return -EFAULT;
			}

			if (copy_to_user(argp, val, sizeof val))
				return -EFAULT;
		}
		break;

	case AMI_IOCTL_SHMDS_GETVERSION:
		if (copy_to_user(argp, shmds_ver, sizeof shmds_ver))
			return -EFAULT;
		break;
/* shmds add <- */

	default:
		PRINTK_E(KERN_ERR "%s not supported = 0x%08x\n", __FUNCTION__, cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}

static int ami_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&g_mutex)) {
		PRINTK("down_interruptible\n");
		return -ERESTARTSYS;
	}

#if 0 /* shmds mod */
	ret = ami_private_ioctl(inode, file, cmd, arg);
#else /* shmds mod */
	if (SHMDS_PedoMutexDown()) {
		ret = ami_private_ioctl(inode, file, cmd, arg);
	} else {
		ret = ami_private_ioctl(inode, file, cmd, arg);
		SHMDS_pedoMutexUp();
	}
#endif /* shmds mod */
	up(&g_mutex);

	return ret;
}

static int debugRead(unsigned int cmd, void __user * argp)
{
	int res = 0;
	int val[2];
	if (copy_from_user(val, argp, sizeof(val)))
		return -EFAULT;

	if (cmd == AMI_IOCTL_DBG_READ) {
		u8 dat;
		res = AMI_i2c_recv_b(ami_i2c_client, val[0], &dat);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI DBG : AMI_i2c_recv error(%d)\n",
			       res);
			return -EFAULT;
		}
		val[1] = dat;
	} else if (cmd == AMI_IOCTL_DBG_READ_W) {
		u16 dat;
		res = AMI_i2c_recv_w(ami_i2c_client, val[0], &dat);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI DBG : AMI_i2c_recv error(%d)\n",
			       res);
			return -EFAULT;
		}
		val[1] = dat;
	} else if (cmd == AMI_IOCTL_DBG_READ_DW) {
		int dat;
		res = AMI_i2c_recv_dw(ami_i2c_client, val[0], &dat);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI DBG : AMI_i2c_recv error(%d)\n",
			       res);
			return -EFAULT;
		}
		val[1] = dat;
	} else if (cmd == AMI_IOCTL_DBG_READ_PD) {
		val[1] = gpio_get_value(AMI_GPIO_DRDY);
	} else if (cmd == AMI_IOCTL_DBG_READ_PI) {
		val[1] = gpio_get_value(AMI_GPIO_INT);
	}
	if (copy_to_user(argp, val, sizeof(val)))
		return -EFAULT;

	return 0;
}

static int debugWrite(unsigned int cmd, void __user * argp)
{
	int res = 0;
	int val[2];
	if (copy_from_user(val, argp, sizeof(val)))
		return -EFAULT;

	if (cmd == AMI_IOCTL_DBG_WRITE) {
		u8 dat = 0xFF & val[1];
		res = AMI_i2c_send_b(ami_i2c_client, val[0], dat);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI DBG : AMI_i2c_recv error(%d)\n",
			       res);
			return -EFAULT;
		}
	} else if (cmd == AMI_IOCTL_DBG_WRITE_W) {
		res = AMI_i2c_send_w(ami_i2c_client, val[0], val[1]);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI DBG : AMI_i2c_recv error(%d)\n",
			       res);
			return -EFAULT;
		}
	} else if (cmd == AMI_IOCTL_DBG_WRITE_DW) {
		res = AMI_i2c_send_dw(ami_i2c_client, val[0], val[1]);
		if (0 > res) {
			PRINTK_E(KERN_ERR "AMI DBG : AMI_i2c_recv error(%d)\n",
			       res);
			return -EFAULT;
		}
	}
	return 0;
}

/*-===========================================================================	
 *
 *			R E G U R A R   S K E L E T O N
 * 
 *-=========================================================================-*/
/*---------------------------------------------------------------------------*/
static struct file_operations ami_fops = {
	.owner = THIS_MODULE,
	.open = ami_open,
	.release = ami_release,
	.ioctl = ami_ioctl,
};

static struct miscdevice ami_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AMI_DRV_NAME,
	.fops = &ami_fops,
};

/*---------------------------------------------------------------------------*/
static int __devinit ami_probe(struct i2c_client *client,
			       const struct i2c_device_id *devid)
{
/* shmds add -> */
	struct shmds_ami603_data *pdata;
	int ret;
	u8 temp;
/* shmds add <- */

	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "start");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PRINTK_E(KERN_ERR
		       "AMI : adapter can NOT support I2C_FUNC_I2C.\n");
		return -ENODEV;
	}
	ami_i2c_client = client;
/* shmds add -> */
	init_MUTEX(&shmds_pedomutex);
	{
		u16 ver = 0;
		u16 asc = 0;
		u8 page = 0x0f;
		memset(shmds_ver, 0, sizeof(shmds_ver));
		if (0 > AMI_i2c_recv_w(ami_i2c_client, AMI_REG_VER, &ver)) {
			ami_i2c_client->addr = (0x1C >> 1);
			if (0 > AMI_i2c_recv_w(ami_i2c_client, AMI_REG_VER, &ver)) {
				ami_i2c_client->addr = (0x1E >> 1);
				shmds_ver[2] = 3;
			}
			else {
				shmds_ver[2] = 2;
			}
		}
		else {
			shmds_ver[2] = 1;
		}
		AMI_i2c_send_b(ami_i2c_client, AMI_REG_I2C_PAGE_NO, page);
		AMI_i2c_recv_w(ami_i2c_client, 0xA4, &asc);
		page = 0x00;
		AMI_i2c_send_b(ami_i2c_client, AMI_REG_I2C_PAGE_NO, page);
		shmds_ver[0] = ver;
		shmds_ver[1] = asc;
	}
/* shmds add <- */

/* shmds add -> */
#ifdef SHMDS_DETECT
	pdata = i2c_get_clientdata(ami_i2c_client);
	INIT_WORK(&shmds_work, SHMDS_ami603_work_func);
	ret = request_irq(ami_i2c_client->irq, SHMDS_ami603_interrupt, IRQF_TRIGGER_RISING, "ami_sensor", pdata);
	disable_irq_nosync(ami_i2c_client->irq);
	if (ret < 0) {
		printk(KERN_ERR "ami603 request irq failed\n");
		return ret;
	}
	
	temp = 0x00;
	AMI_i2c_send_b(ami_i2c_client, AMI_REG_MD_TIMER, temp);
	temp = 25;
	AMI_i2c_send_b(ami_i2c_client, AMI_REG_MD_THRESH, temp);
	
	for (temp = 0; temp < 3; temp++) {
		shmds_detect_buf[temp] = (s16 *)kmalloc(shmds_buf_size * sizeof(s16), GFP_KERNEL);
		if (NULL == shmds_detect_buf[temp]) {
			return -ENOMEM;
		}
		memset(shmds_detect_buf[temp], 0, shmds_buf_size * sizeof(s16));
	}
	
#endif
/* shmds add <- */

	memset(&k_start_param, 0, sizeof(k_start_param));
	ami_sen_stat.mode = 0;

	memset(&g_pedoThreshold, 0, sizeof(g_pedoThreshold));
	memset(&g_pedoStatus, 0, sizeof(g_pedoStatus));

	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "end");

	return 0;
}

/*---------------------------------------------------------------------------*/
static int __devexit ami_remove(struct i2c_client *client)
{
/* shmds add -> */
#ifdef SHMDS_DETECT
	struct shmds_ami603_data *pdata;
	int i;
	
	pdata = i2c_get_clientdata(ami_i2c_client);
	free_irq(client->irq, pdata);
	
	for (i = 0; i < 3; i++) {
		kfree(shmds_detect_buf[i]);
		shmds_detect_buf[i] = NULL;
	}
#endif
/* shmds add <- */

	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "start");
	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "end");
	return 0;
}

/*---------------------------------------------------------------------------*/
static const struct i2c_device_id ami_idtable[] = {
	{AMI_DRV_NAME, 0},
	{}
};

/*---------------------------------------------------------------------------*/
static struct i2c_driver ami_i2c_driver = {
	.driver = {
		   .name = AMI_DRV_NAME,
		   },
	.probe = ami_probe,
	.remove = __devexit_p(ami_remove),
	.id_table = ami_idtable,
};

/*---------------------------------------------------------------------------*/
static int __devinit ami_init(void)
{
	int res = 0;
#if 0	/* shmds del */
	struct i2c_board_info info;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
#endif	/* shmds del */

	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "start");
	PRINTK("%s compiled %s %s\n", AMI_DRV_NAME, __DATE__, __TIME__);
	/* THIS_CODE_REMARKS is defined 1st page of this file. */
	PRINTK("%s\n", THIS_CODE_REMARKS);

	res = misc_register(&ami_device);
	if (0 > res) {
		PRINTK_E(KERN_ERR "AMI : register failed\n");
		return res;
	}
	res = i2c_add_driver(&ami_i2c_driver);
	if (0 > res) {
		PRINTK_E(KERN_ERR "AMI : i2c_add_driver failed\n");
		misc_deregister(&ami_device);
		return res;
	}
#if 0	/* shmds del */
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = AMI_I2C_ADDRESS;
	strlcpy(info.type, AMI_DRV_NAME, I2C_NAME_SIZE);
	adapter = i2c_get_adapter(AMI_I2C_BUS_NUM);
	client = i2c_new_device(adapter, &info);
	i2c_put_adapter(adapter);
#endif	/* shmds del */

	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "end");

	/* Init gpio */
	gpio_request(AMI_GPIO_DRDY, "DRDY");
	gpio_direction_input(AMI_GPIO_DRDY);

	gpio_request(AMI_GPIO_INT, "INT");
	gpio_direction_input(AMI_GPIO_INT);

	return res;
}

module_init(ami_init);
/*---------------------------------------------------------------------------*/
static void __exit ami_exit(void)
{
	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "start");
	/* Unregist i2c slave address, or next i2c_new_device will fail. */
	i2c_unregister_device(ami_i2c_client);
	i2c_del_driver(&ami_i2c_driver);	/* delete driver */
	misc_deregister(&ami_device);
	ami_i2c_client = NULL;

	PRINTK("%s %s %s\n", AMI_DRV_NAME, __FUNCTION__, "end");
}

module_exit(ami_exit);
/*---------------------------------------------------------------------------*/
/*        R E G U L A R        F O O T E R                                   */
/*---------------------------------------------------------------------------*/
MODULE_DESCRIPTION("AMI MI sensor");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
