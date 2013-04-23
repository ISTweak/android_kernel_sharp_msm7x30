/* include/sharp/shmds_driver.h  (MotionSensor Driver)
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

#ifndef SHMDS_H
#define SHMDS_H

#include <linux/ioctl.h>

struct sh_i2c_ms_platform_data {
    int     gpio_trg;
    int     gpio_irq;
    int     gpio_rst;
    int     (*gpio_setup) (void);
    void    (*gpio_shutdown)(void);
};
#define	SH_MS_I2C_DEVNAME    "ami602"
#define SH_MS_I2C_SLAVE      0x60
#define	SH_MS_TRG            16
#define	SH_MS_IRQ            20
#define	SH_MS_RST            85

/* ami602 ioctl */
#define MS_IOC             'm'
#define MS_I2C_READ       _IOWR(MS_IOC,0x01, MS_I2CParam_t)
#define MS_I2C_WRITE      _IOWR(MS_IOC,0x02, MS_I2CParam_t)
#define MS_MISTART_GPIO   _IOW(MS_IOC,0x03, char)
#define MS_MIBUSY_GPIO    _IOR(MS_IOC,0x04, char)
#define MS_MIRST_GPIO     _IOW(MS_IOC,0x05, char)
#define MS_SETMES         _IOW(MS_IOC,0x06, unsigned char[9])
#define MS_SETPED         _IOW(MS_IOC,0x07, unsigned char[9])
#define MS_INTERRUPT      _IOW(MS_IOC,0x08, char)
#define MS_GETMES         _IOR(MS_IOC,0x09, unsigned char[9])
#define MS_GET_INIT       _IOR(MS_IOC,0x0A, MS_InitData_t)
#define MS_GETPED         _IOR(MS_IOC,0x0B, unsigned char[9])
#define MS_INITPEDOPARAM  _IOW(MS_IOC,0x0C, MS_PedoParam_t)
#define MS_GETPOSITION    _IOW(MS_IOC,0x0D, unsigned char)
#define MS_COSRFINRCHECK  _IO(MS_IOC,0x0E)

/* user data hold */
#define MS_GETSTATE       _IOR(MS_IOC,0x30, unsigned char)
#define MS_SETSTATE       _IOW(MS_IOC,0x31, unsigned char)
#define MS_GETCUSTODY     _IOR(MS_IOC,0x32, MS_CustodyData_t)
#define MS_SETCUSTODY     _IOW(MS_IOC,0x33, MS_CustodyData_t)
#define MS_GETUSERDATA    _IOR(MS_IOC,0x34, unsigned short)
#define MS_SETUSERDATA    _IOW(MS_IOC,0x35, unsigned short)
#define MS_GETPEDOPARAM   _IOR(MS_IOC,0x36, MS_PedoParam_t)
#define MS_SETPEDOPARAM   _IOW(MS_IOC,0x37, MS_PedoParam_t)
#define MS_GETPEDOTHRESH  _IOR(MS_IOC,0x38, unsigned char[17])
#define MS_SETPEDOTHRESH  _IOW(MS_IOC,0x39, unsigned char[17])
#define MS_GETMESWAIT     _IOR(MS_IOC,0x3A, unsigned short)
#define MS_SETMESWAIT     _IOW(MS_IOC,0x3B, unsigned short)
#define MS_GETBATTERYLID  _IOR(MS_IOC,0x3C, MS_BatteryLid_t)
#define MS_SETBATTERYLID  _IOW(MS_IOC,0x3D, MS_BatteryLid_t)
#define MS_GETINITFLG     _IOR(MS_IOC,0x3E, unsigned char)
#define MS_SETINITFLG     _IOW(MS_IOC,0x3F, unsigned char)
#define MS_GETCALIBRATION _IOR(MS_IOC,0x40, MS_CalibrationParam_t)
#define MS_SETCALIBRATION _IOW(MS_IOC,0x41, MS_CalibrationParam_t)
#define MS_GETACTIVE      _IOR(MS_IOC,0x42, unsigned long)
#define MS_SETACTIVE      _IOW(MS_IOC,0x43, unsigned long)
#define MS_GETDISPLAY     _IOR(MS_IOC,0x44, MS_DisplayData_t)
#define MS_SETDISPLAY     _IOW(MS_IOC,0x45, MS_DisplayData_t)
#define MS_GETTPTHREAD    _IOR(MS_IOC,0x46, MS_TPthread_t)
#define MS_SETTPTHREAD    _IOW(MS_IOC,0x47, MS_TPthread_t)
#define MS_GETPEDOPAUSE   _IOR(MS_IOC,0x48, unsigned char)
#define MS_SETPEDOPAUSE   _IOW(MS_IOC,0x49, unsigned char)
#define MS_GETTPMODE      _IOR(MS_IOC,0x4A, unsigned char)
#define MS_SETTPMODE      _IOW(MS_IOC,0x4B, unsigned char)

#define MS_FLG_OFF      0
#define MS_FLG_ON       1

#define MS_RESULT_ERR 90

#define  CAL_COSRFINR_MASK			0x01
#define  CAL_OFFSET_ACC_MASK		0x02
#define  CAL_GAIN_MAG_MASK			0x04
#define  CAL_OFFSET_OPEN_MAG_MASK	0x08
#define  CAL_OFFSET_CYCL_MAG_MASK	0x10
#define  CAL_GAIN_MAG_X_MASK		0x20
#define  CAL_GAIN_MAG_Y_MASK		0x40
#define  CAL_GAIN_MAG_Z_MASK		0x80
#define  CAL_COSRFINR2_MASK			0x100

typedef struct
{
    char cmd;
    char data[20];
    char length;
}MS_I2CParam_t;

typedef struct {
    unsigned short OriginSensitivity[5][3];
    unsigned char VersionData[6];
    unsigned short ADOutputChangeValue[3];
    signed short SerialIDValue;
    unsigned char InitErrFlg;
    unsigned char ChipCosr[6];
    unsigned char ChipFinr[6];
}MS_InitData_t;

typedef struct {
    short x;
    short y;
    short z;
} MS_Vector_t;

typedef struct {
    short xy;
    short xz;
    short yx;
    short yz;
    short zx;
    short zy;
} MS_Interference_t;

typedef struct {
    MS_Vector_t m_gain;
    MS_Vector_t m_offset;
    MS_Interference_t m_interference;
    MS_Vector_t a_gain;
    MS_Vector_t a_offset;
} MS_SensorParam_t;

typedef struct {
    MS_Vector_t m_offset;
    MS_Vector_t a_offset;
} MS_UserParam_t;

typedef struct {
    char mag_dir;
    char mag_polarity;
    char acc_dir;
    char acc_polarity;
} MS_Direction_t;

typedef struct {
    MS_SensorParam_t sensor;
    MS_UserParam_t usr;	
    MS_Direction_t dir;
} MS_CalibrationParam_t;

typedef struct {
    unsigned short OP_Offset_MagX;
    unsigned short OP_Offset_MagY;
    unsigned short OP_Offset_MagZ;
    unsigned short CY_Offset_MagX;
    unsigned short CY_Offset_MagY;
    unsigned short CY_Offset_MagZ;
    unsigned short OP_Offset_AccX;
    unsigned short OP_Offset_AccY;
    unsigned short OP_Offset_AccZ;
    unsigned short CY_Offset_AccX;
    unsigned short CY_Offset_AccY;
    unsigned short CY_Offset_AccZ;
    unsigned char OpenCosrValue[6];
    unsigned char OpenFinrValue[6];
    unsigned short Gain_MagX;
    unsigned short Gain_MagY;
    unsigned short Gain_MagZ;
    unsigned short CalibrationFlg;
    unsigned char OtherCosrValue[6];
    unsigned char OtherFinrValue[6];
}MS_CustodyData_t;

typedef	struct {
    unsigned short OP_Offset_MagX;
    unsigned short OP_Offset_MagY;
    unsigned short OP_Offset_MagZ;
    unsigned short CY_Offset_MagX;
    unsigned short CY_Offset_MagY;
    unsigned short CY_Offset_MagZ;
    unsigned char OpenCosrValue[6];
    unsigned char OpenFinrValue[6];
    unsigned short CalibrationFlg;
    unsigned char OtherCosrValue[6];
    unsigned char OtherFinrValue[6];
}MS_UserCalibrationData_t;

typedef struct {
    unsigned long CntData;
    unsigned long TimeData;
    unsigned char StatusData;
    unsigned long KeepCnt;
    unsigned long KeepTime;
    unsigned char KernelPause;
}MS_PedoParam_t;

typedef struct {
    short diff_x;
    short diff_y;
    short diff_z;
    short k11;
    short k12;
    short k13;
    short k21;
    short k22;
    short k23;
    short k31;
    short k32;
    short k33;
}MS_BT_LidParamData_t;

typedef struct {
    MS_BT_LidParamData_t Param[2];
    unsigned char Flg;
}MS_BatteryLid_t;

typedef struct {
    unsigned char direct;
    unsigned char result;
}MS_DisplayData_t;

typedef struct
{
    char poll;
    long tbl;
}MS_TPthread_t;

typedef struct{
    unsigned char CosrValue[6];
    unsigned char FinrValue[6];
    unsigned short CalibrationFlg;
}MS_CosrFinrData_t;


enum ami602_read_command
{
    GET_COSR = 0x11,
    GET_FINR = 0x12,
    GET_MES_SUSPEND = 0x28,
    GET_MES = 0x14,
    GET_DAT = 0x21,
    GET_FIRMWARE = 0x17,
    GET_MES_PED_AUTO_SUSPEND = 0x32,
    GET_MES_AVG = 0x33,
    GET_GAIN = 0x10,
    GET_PED_TH = 0x36,
    GET_MES_T = 0x15
};

enum ami602_write_command {
    REQ_MES = 0x55,
    SET_PWR_DOWN = 0x57,
    SET_SUSPEND = 0x75,
    SET_COSR = 0x5B,
    SET_FINR = 0x5C,
    SET_INDEX_DAT = 0x63,
    SET_AEN = 0x74,
    SET_MES_6CH_AUTO_START = 0x64,
    SET_MES_6CH_AUTO_STOP = 0x65,
    SET_MES_PED_AUTO_START = 0x77,
    SET_MES_PED_AUTO_STOP = 0x78,
    CLR_PED_SUSPEND = 0x79,
    CHG_MES_AVG = 0x80,
    SET_GAIN = 0x61,
    SET_PED_TH = 0x83
};

enum ami602_state {
    MEASURE_PEDOMETER_STANDBY = 0,
    MEASURE_ONLY_ACTIVE,
    PEDOMETER_ONLY_ACTIVE,
    MEASURE_PEDOMETER_ACTIVE,
    PEDOMETER_ONLY_PAUSE,
    MEASURE_ACTIVE_PEDO_PAUSE,
    ACTIVEMODE_SHDIAG,
    HOSTTRIGGER_SHDIAG,
};

enum {
    MS_INTERRUPT_OFF = 0,
    MS_INTERRUPT_MES,
    MS_INTERRUPT_MES_PEDO,
};

enum{
    MS_POSITION_OPEN = 0,
    MS_POSITION_CLOSE,
    MS_POSITION_SWIVEL,
    MS_POSITION_CYCLOID,
};

void AMI602Pedometer_ReStart(void);
void AMI602Pedometer_Pause(void);
void AMI602_SetFlipInformation(unsigned char position);

#define SHMDS_Pedometer_ReStart AMI602Pedometer_ReStart
#define SHMDS_Pedometer_Pause AMI602Pedometer_Pause
#define SHMDS_SetFlipInformation AMI602_SetFlipInformation

#endif
