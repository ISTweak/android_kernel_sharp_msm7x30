/* include/sharp/sh_oncrpc_id.h
 *
 * Copyright (C) 2010 Sharp Corporation
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

#ifndef __SH_ONCRPC_ID_H__
#define __SH_ONCRPC_ID_H__

/* SHSYS (mARM <- aARM) */
#define SHSYS_A2M_PROG		0x50000000
#define SHSYS_A2M_VERS		0x00010001

/* SHSYS (mARM -> aARM) */
#define SHSYS_M2A_PROG		0x50000001
#define SHSYS_M2A_VERS		0x00010001

/* Battery(SHBATT) (mARM <- aARM) */
#define SHBATT_REMOTE_A2MPROG		0x50000050
#define SHBATT_REMOTE_A2MVERS		0x00010001

/* Battery(SH_BATTERY) (mARM -> aARM) */
#define SH_BATTERY_REMOTE_M2APROG	0x50000051
#define SH_BATTERY_REMOTE_M2AVERS	0x00010001

/* Battery(SHCHG) (mARM <- aARM) */
#define SHCHG_REMOTE_A2MPROG		0x50000052
#define SHCHG_REMOTE_A2MVERS		0x00010001

/* Battery(SHTHERM) (mARM <- aARM) */
#define SHTHERM_REMOTE_A2MPROG		0x50000053
#define SHTHERM_REMOTE_A2MVERS		0x00010001

/* Battery(SHTHERM) (mARM -> aARM) */
#define SHTHERM_REMOTE_A2MCBPROG	0x50000054
#define SHTHERM_REMOTE_A2MCBVERS	0x00010001

/* SHTERM M2A */
#define SHTERM_M2A_PROG			0x50000055
#define SHTERM_M2A_VERS			0x00010001

/* SHTERM A2M */
#define SHTERM_A2M_PROG			0x50000056
#define SHTERM_A2M_VERS			0x00010001

/* SHPASS M2A */
#define SHPASS_M2A_PROG			0x50000080
#define SHPASS_M2A_VERS			0x00010001

/* SHPASS A2M */
#define SHPASS_A2M_PROG			0x50000081
#define SHPASS_A2M_VERS			0x00010001

/* SHSWIC (mARM <- aARM) */
#define SHSWIC_REMOTE_A2MPROG	0x50000090
#define SHSWIC_REMOTE_A2MVERS	0x00010001

/* SHSWIC (mARM -> aARM) */
#define SHSWIC_REMOTE_M2APROG	0x50000091
#define SHSWIC_REMOTE_M2AVERS	0x00010001

/* SHRMTS (mARM -> aARM) */
#define SHRMTS_REMOTE_M2APROG	0x500000B0
#define SHRMTS_REMOTE_M2AVERS	0x00010001
#endif /* __SH_ONCRPC_ID_H__ */

