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

#ifndef AMI_HW_H
#define AMI_HW_H

#define	AMI_I2C_BUS_NUM				2

#define AMI_I2C_ADDRESS 			0x0E

#define AMI_GPIO_INT				20 /* shmds mod */
#define AMI_GPIO_DRDY				16 /* shmds mod */

/* AMI-Sensor Internal Register Address  
(Please refer to AMI-Sensor Specifications) */
#define AMI_REG_AXOUT 				0x06
#define AMI_REG_AYOUT 				0x08
#define AMI_REG_AZOUT 				0x0A
#define AMI_REG_MXOUT 				0x0C
#define AMI_REG_MYOUT 				0x0E
#define AMI_REG_MZOUT 				0x10
#define AMI_REG_INS2 				0x17
#define AMI_REG_STA1 				0x18
#define AMI_REG_INTREL 				0x1A
#define AMI_REG_CNTL1 				0x1B
#define AMI_REG_CNTL2 				0x1C
#define AMI_REG_CNTL3 				0x1D
#define AMI_REG_INC1 				0x1E
#define AMI_REG_INC2 				0x1F
#define AMI_REG_I2C_PAGE_NO 		0x3F
#define AMI_REG_TICK_INTERVAL 		0x84
#define AMI_REG_OFFZ 				0x92
#define AMI_REG_OFFY 				0x95
#define AMI_REG_OFFX 				0x98
#define AMI_REG_A_CNTL				0xB4
#define AMI_REG_INFO 				0xB8
#define AMI_REG_WIA 				0xBA
#define AMI_REG_VER 				0xBC
#define AMI_REG_SN 					0xBE
#define AMI_REG_TEMP 				0xD6

/* AMI-Sensor Internal Register Address pedo 
(Please refer to AMI-Sensor Specifications) */
#define AMI_DR_CTRL3 				0x2A
#define AMI_DR_CTRL4 				0x2B
#define AMI_REST_OUT_TH 			0x42
#define AMI_REST_IN_TIME 			0x43
#define AMI_REST_IN_CNT 			0x44
#define AMI_REST_IN_TH 				0x45
#define AMI_STEP_UP_TH 				0x46
#define AMI_STEP_DW_TH 				0x47
#define AMI_STEP_MS 				0x48
#define AMI_STEP_STOP_MS 			0x49
#define AMI_IIR_WEIGHT1 			0x4A
#define AMI_IIR_WEIGHT2 			0x4B
#define AMI_IIR_WEIGHT3 			0x4C
#define AMI_TWO_STEP_MIN 			0x4D
#define AMI_TWO_STEP_MAX 			0x4E
#define AMI_STATUS_CNT 				0x4F
#define AMI_STATUS_TIME 			0x53
#define AMI_STATUS_STAT 			0x57
#define AMI_RST_STATUS 				0x58
#define AMI_ACC_CNT_TH 				0x5F
#define AMI_WALK_CNT_TH 			0x63
#define AMI_ZERO_CROSS_CHK 			0x65
#define AMI_TWO_STEP_DEF 			0x66
#define AMI_TWO_STEP_CHK 			0x67

/* AMI-Sensor OTP map (Please refer to AMI-Sensor Specifications) */
#define AMI_OTP_GAIN_PARA_AX 		0xAC
#define AMI_OTP_SENSMX 				0xB4
#define AMI_OTP_SENSMY 				0xB6
#define AMI_OTP_SENSMZ 				0xB8
#define AMI_OTP_GAIN_PARA_MX 		0xA6
#define AMI_OTP_GAIN_PARA_MY 		0xA8
#define AMI_OTP_GAIN_PARA_MZ 		0xAA
#define AMI_OTP_ORGAX 				0xC4
#define AMI_OTP_ORGAY 				0xC6
#define AMI_OTP_ORGAZ 				0xC8
#define AMI_OTP_SENSAX 				0xCA
#define AMI_OTP_SENSAY 				0xCC
#define AMI_OTP_SENSAZ 				0xCE
#define AMI_OTP_FINE_OUTPUTX		0xAE
#define AMI_OTP_FINE_OUTPUTY		0xB0
#define AMI_OTP_FINE_OUTPUTZ		0xB2
#define AMI_OTP_0GAUSS_FINEX		0xFC	/* bit[7:0] */
#define AMI_OTP_0GAUSS_FINEY		0xFA	/* bit[7:0] */
#define AMI_OTP_0GAUSS_FINEZ		0xF8	/* bit[7:0] */
#endif
