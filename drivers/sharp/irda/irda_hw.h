/* drivers/sharp/irda/irda_hw.h (IrDA SIR/FIR driver module)
 *
 * Copyright (C) 2009-2011 SHARP CORPORATION All rights reserved.
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

#ifndef _IR_HW_H
#define	_IR_HW_H

#ifdef	CONFIG_SHARP_INFRARED_LR388J5
#include "sharp/sh_cpu.h"
#include "sharp/shlcdc_kerl.h"
#endif

#ifdef CONFIG_SHARP_INFRARED_LR388G7
#define	IRDA_DEVICE_NAME	"LR388G7"
#else
#define	IRDA_DEVICE_NAME	"LR388J5"
#endif

#define	IRDA_CAPABILITY_SIR_BAUDRATE	\
		"SIR       : 9600bps, 19.2Kbps, 38.4Kbps, 57.6Kbps, 115.2Kbps"
#define	IRDA_CAPABILITY_FIR_BAUDRATE	"FIR       : 4Mbps"
#define	IRDA_CAPABILITY_ABOF		\
			"aBOF      : 48, 32, 24, 20, 16, 14, 12, 10, 8, 6 - 0"
#define	IRDA_CAPABILITY_MTT		"MinTAT    : 0msec, 0.5msec - 16msec"
#define	IRDA_CAPABILITY_MPI		"MPI       : 100usec - 16msec"
#define	IRDA_CAPABILITY_BUFF		"BuffSize  : 2080bytes"
#define	IRDA_CAPABILITY_HWBUFF		"HW buffer : 2buffers"
#define	IRDA_CAPABILITY_SWTXBUFF	"SW Tx buff: 9buffers"
#define	IRDA_CAPABILITY_SWRXBUFF	"SW Rx buff: 9buffers"

#define	IRDA_CAPABILITY_BAUDRATE_VAL	"1,2,3,4,5,6:"
#define	IRDA_CAPABILITY_ABOF_VAL	\
				"48,32,24,20,16,14,12,10,8,6,5,4,3,2,1,0:"
#define	IRDA_CAPABILITY_MTT_VAL		"0,500-16000:"
#define	IRDA_CAPABILITY_MPI_VAL		"100-16000:"
#define	IRDA_CAPABILITY_BUFF_VAL	"2080:2,9,9;"



#define IRDA_ADDR_HCS_SIZE	0x1000

#define	IRDA_ADDR_HCS1		0x8b000000
#define	IRDA_ADDR_HCS1_SIZE	IRDA_ADDR_HCS_SIZE
#define	IRDA_ADDR_HCS2		0x8c000000
#define	IRDA_ADDR_HCS2_SIZE	IRDA_ADDR_HCS_SIZE






#define IRDA_INT_GPIO_NO	(116)


#define	IR_QSD_IRRX_PU		149

#define	IR_QSD_GPIO_ON		(uint16)0x0001
#define	IR_QSD_GPIO_OFF		(uint16)0x0000











#endif
