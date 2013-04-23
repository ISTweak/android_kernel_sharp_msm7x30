/*
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
/* CONFIG_SH_AUDIO_DRIVER newly created */
#ifndef __SHSPAMP_H__
#define __SHSPAMP_H__

#define SHSPAMP_IOC_MAGIC				0xE1

#define SPAMP_SET_MUTEN					_IO  ( SHSPAMP_IOC_MAGIC,  1)

extern void shspamp_poweron(void);
extern void shspamp_poweroff(void);


/* SH_AUDIO_DRIVER AN12969 customize -> */
#if	defined(CONFIG_SHSPAMP_AN12969)

#define	SPAMP_MONO

#define	SPAMP_POWER_ON_MARGIN_TIME	30000	/* SPSave sleep Time(us) */
/* Register Size(Byte) */
#define SPAMP_READ_BUF_SIZE		3	/* data x 3		*/


/***** 0x00[Hex]  *****/
/*
<mono>
-------------------------------------------------------------------------
|   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
-------------------------------------------------------------------------
|  GAIN  |    0   |    0   |    0   |   AGC  | SP Save| Standby|    0   |
|0:+23dB |        |        |        |  0:OFF |  0:ON  |  0:ON  |        |
|1:+26dB |        |        |        |  1:ON  |  1:OFF |  1:0FF |        |
-------------------------------------------------------------------------
<stereo>
-------------------------------------------------------------------------
|   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
-------------------------------------------------------------------------
|    0   |    0   |    0   |    0   |   AGC  | SP Save| Standby|    0   |
|        |        |        |        |  0:OFF |  0:ON  |  0:ON  |        |
|        |        |        |        |  1:ON  |  1:OFF |  1:0FF |        |
-------------------------------------------------------------------------
*/

/* D7: GAIN */					/*D7...   0         */
#define SPAMP_GAIN_NONE			0x00	/* 0------- +23[dB] */
#define SPAMP_GAIN_20DB			0x00	/* 0------- +23[dB] */
#define SPAMP_GAIN_26DB			0x80	/* 1------- +26[dB] */
#define SPAMP_GAIN_MASK			0x80

#ifdef SPAMP_MONO

/* D6-4: 0 */
#define SPAMP_FIX_0HEX			0x00	/* -000---- */
#define SPAMP_FIX_0HEX_MASK		0x70

#else	/* SPAMP_MONO */

/* D7-4:  */
#define SPAMP_FIX_0HEX			0x00	/* 0000---- */
#define SPAMP_FIX_0HEX_MASK		0xF0

#endif	/* SPAMP_MONO */

/* D3: AGC(Auto Gain Control) */
#define SPAMP_AGC_OFF			0x00	/* ----0--- AGC OFF */
#define SPAMP_AGC_ON			0x08	/* ----1--- AGC ON */
#define SPAMP_AGC_MASK			0x08

/* D2: SP Save */
#define SPAMP_SPSAVE_ON			0x00	/* -----0-- SP Save ON */
#define SPAMP_SPSAVE_OFF		0x04	/* -----1-- SP Save OFF */
#define SPAMP_SPSAVE_MASK		0x04

/* D1: Standby */
#define SPAMP_STANDBY_ON		0x00	/* ------0- Standby ON */
#define SPAMP_STANDBY_OFF		0x02	/* ------1- Standby OFF */
#define SPAMP_STANDBY_MASK		0x02

/* D0: Init Condition(0) */
#define SPAMP_INIT_CONDITION		0x00	/* -------0 */
#define SPAMP_INIT_CONDITION_MASK	0x01

#define SPAMP_REG_00			0x00

#define SPAMP_REG_CHECK			0x0E


/***** 0x01[Hex]  *****/
/*
-------------------------------------------------------------------------
|   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
-------------------------------------------------------------------------
| AGC-ON | AGC-ON | AGC-ON | AGC-REC| AGC-REC| AGC-REC| AGC-ATT| AGC-ATT|
|   bit3 |   bit2 |   bit1 |   bit3 |   bit2 |   bit1 |   bit2 |   bit1 |
-------------------------------------------------------------------------
*/
/* D7-5: AGC On Level */
#define SPAMP_AGC_ON_12_6DBV	0x00	/* 000----- 12.6[dBv] */
#define SPAMP_AGC_ON_13_2DBV	0x20	/* 001----- 13.2[dBv] */
#define SPAMP_AGC_ON_13_9DBV	0x40	/* 010----- 13.9[dBv] */
#define SPAMP_AGC_ON_14_5DBV	0x60	/* 011----- 14.5[dBv] */
#define SPAMP_AGC_ON_15_1DBV	0x80	/* 100----- 15.1[dBv] */
#define SPAMP_AGC_ON_15_6DBV	0xA0	/* 101----- 15.6[dBv] */
#define SPAMP_AGC_ON_16_1DBV	0xC0	/* 110----- 16.1[dBv] */
#define SPAMP_AGC_ON_16_6DBV	0xE0	/* 111----- 16.6[dBv] */

#define SPAMP_AGC_ON_MASK		0xE0

/* D4-2: AGC Recovery Time */
#define SPAMP_AGC_REC_1_0S		0x00	/* ---000-- 1.0[s] */
#define SPAMP_AGC_REC_1_5S		0x04	/* ---001-- 1.5[s] */
#define SPAMP_AGC_REC_2_0S		0x08	/* ---010-- 2.0[s] */
#define SPAMP_AGC_REC_3_0S		0x0C	/* ---011-- 3.0[s] */
#define SPAMP_AGC_REC_4_0S		0x10	/* ---100-- 4.0[s] */
#define SPAMP_AGC_REC_6_0S		0x14	/* ---101-- 6.0[s] */
#define SPAMP_AGC_REC_8_0S		0x18	/* ---110-- 8.0[s] */
#define SPAMP_AGC_REC_12_0S		0x1C	/* ---111-- 12.0[s] */
#define SPAMP_AGC_REC_MASK		0x1C

/* D1-0: AGC Attack Time */
#define SPAMP_ATT_0_5MS			0x00	/* ------00 0.5[ms] */
#define SPAMP_ATT_1_0MS			0x01	/* ------01 1.0[ms] */
#define SPAMP_ATT_2_0MS			0x02	/* ------10 2.0[ms] */
#define SPAMP_ATT_4_0MS			0x03	/* ------11 4.0[ms] */
#define SPAMP_ATT_MASK			0x03

#define SPAMP_REG_01			0x01


/***** 0x02[Hex]  *****/
/*
-------------------------------------------------------------------------
|   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
-------------------------------------------------------------------------
|    0   |    0   |    0   |    *   |    *   |    0   |    0   |    0   |
-------------------------------------------------------------------------
*/
/* D7-5, D2-0: 0 */
#define SPAMP_FIX_2HEX			0x00
#define SPAMP_FIX_2HEX_MASK		0xE7

#define SPAMP_REG_02			0x02

#endif	/* CONFIG_SHSPAMP_AN12969 */
/* SH_AUDIO_DRIVER AN12969 customize <- */

#endif /* __SHSPAMP_H__ */
