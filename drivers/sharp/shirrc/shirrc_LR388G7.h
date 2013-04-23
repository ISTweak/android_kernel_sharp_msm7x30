/* drivers/sharp/shirrc/shirrc_LR388G7.h (Infrared driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#ifndef _IRRC_REG_H
#define	_IRRC_REG_H

#include <stdbool.h>

#define IRRC_REG_SUCCESS	(0)
#define IRRC_REG_ERROR		(-1)

#define IRRC_ADDR_HCS1		0x8B000000
#define IRRC_ADDR_HCS1_SIZE	0x1000

extern void __iomem *irrc_lr388g7_hcs1;

#define	IRRC_GLT_REG_BASE	irrc_lr388g7_hcs1
#define	IRRC_GLT_REG_INTR	(IRRC_GLT_REG_BASE + 0x0002)
#define	IRRC_GLT_REG_INTM	(IRRC_GLT_REG_BASE + 0x0006)
#define	IRRC_GLT_REG_IRRCDIV	(IRRC_GLT_REG_BASE + 0x0030)

#define	IRRC_GLT_WRITE_REG(Reg, Val)				\
{								\
	*((volatile uint16*)(Reg)) = (Val);			\
}

#define	IRRC_REG_OFFSET		0x04C0
#define	IRRC_REG_SYS		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x00)
#define	IRRC_REG_BASE		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x02)
#define	IRRC_REG_CLO		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x04)
#define	IRRC_REG_CHI		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x06)
#define	IRRC_REG_HLOL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x08)
#define	IRRC_REG_HLOH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x0a)
#define	IRRC_REG_HHIL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x0c)
#define	IRRC_REG_HHIH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x0e)
#define	IRRC_REG_D0LOL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x10)
#define	IRRC_REG_D0LOH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x12)
#define	IRRC_REG_D0HIL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x14)
#define	IRRC_REG_D0HIH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x16)
#define	IRRC_REG_D1LOL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x18)
#define	IRRC_REG_D1LOH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x1a)
#define	IRRC_REG_D1HIL		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x1c)
#define	IRRC_REG_D1HIH		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x1e)
#define	IRRC_REG_ENDLENL	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x20)
#define	IRRC_REG_ENDLENH	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x22)
#define	IRRC_REG_BITLEN		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x24)
#define	IRRC_REG_FRMLENL	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x26)
#define	IRRC_REG_FRMLENH	(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x28)
#define	IRRC_REG_OUT0		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x2a)
#define	IRRC_REG_OUT1		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x2c)
#define	IRRC_REG_OUT2		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x2e)
#define	IRRC_REG_OUT3		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x30)
#define	IRRC_REG_OUT4		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x32)
#define	IRRC_REG_OUT5		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x34)
#define	IRRC_REG_OUT6		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x36)
#define	IRRC_REG_OUT7		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x38)
#define	IRRC_REG_SEND		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x3a)
#define	IRRC_REG_REGS		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x3c)
#define	IRRC_REG_DBG		(IRRC_GLT_REG_BASE + IRRC_REG_OFFSET + 0x3e)

#define	IRRC_READ_SYS		(*(volatile uint16*)IRRC_REG_SYS)
#define	IRRC_READ_BASE		(*(volatile uint16*)IRRC_REG_BASE)
#define	IRRC_READ_IRRCDIV	(*(volatile uint16*)IRRC_REG_BASE)
#define	IRRC_READ_CLO		(*(volatile uint16*)IRRC_REG_CLO)
#define	IRRC_READ_CHI		(*(volatile uint16*)IRRC_REG_CHI)
#define	IRRC_READ_HLOL		(*(volatile uint16*)IRRC_REG_HLOL)
#define	IRRC_READ_HLOH		(*(volatile uint16*)IRRC_REG_HLOH)
#define	IRRC_READ_HHIL		(*(volatile uint16*)IRRC_REG_HHIL)
#define	IRRC_READ_HHIH		(*(volatile uint16*)IRRC_REG_HHIH)
#define	IRRC_READ_D0LOL		(*(volatile uint16*)IRRC_REG_D0LOL)
#define	IRRC_READ_D0LOH		(*(volatile uint16*)IRRC_REG_D0LOH)
#define	IRRC_READ_D0HIL		(*(volatile uint16*)IRRC_REG_D0HIL)
#define	IRRC_READ_D0HIH		(*(volatile uint16*)IRRC_REG_D0HIH)
#define	IRRC_READ_D1LOL		(*(volatile uint16*)IRRC_REG_D1LOL)
#define	IRRC_READ_D1LOH		(*(volatile uint16*)IRRC_REG_D1LOH)
#define	IRRC_READ_D1HIL		(*(volatile uint16*)IRRC_REG_D1HIL)
#define	IRRC_READ_D1HIH		(*(volatile uint16*)IRRC_REG_D1HIH)
#define	IRRC_READ_ENDLENL	(*(volatile uint16*)IRRC_REG_ENDLENL)
#define	IRRC_READ_ENDLENH	(*(volatile uint16*)IRRC_REG_ENDLENH)
#define	IRRC_READ_BITLEN	(*(volatile uint16*)IRRC_REG_BITLEN)
#define	IRRC_READ_FRMLENL	(*(volatile uint16*)IRRC_REG_FRMLENL)
#define	IRRC_READ_FRMLENH	(*(volatile uint16*)IRRC_REG_FRMLENH)
#define	IRRC_READ_OUT(v)	(*(volatile uint16*)(IRRC_REG_OUT0+2*(v)))
#define	IRRC_READ_SEND		(*(volatile uint16*)IRRC_REG_SEND)
#define	IRRC_READ_REGS		(*(volatile uint16*)IRRC_REG_REGS)
#define	IRRC_READ_DBG		(*(volatile uint16*)IRRC_REG_DBG)

#define	IRRC_WRITE_SYS(v)	(*(volatile uint16*)IRRC_REG_SYS	= (v))
#define	IRRC_WRITE_BASE(v)	(*(volatile uint16*)IRRC_REG_BASE	= (v))
#define	IRRC_WRITE_CLO(v)	(*(volatile uint16*)IRRC_REG_CLO	= (v))
#define	IRRC_WRITE_CHI(v)	(*(volatile uint16*)IRRC_REG_CHI	= (v))
#define	IRRC_WRITE_HLOL(v)	(*(volatile uint16*)IRRC_REG_HLOL	= (v))
#define	IRRC_WRITE_HLOH(v)	(*(volatile uint16*)IRRC_REG_HLOH	= (v))
#define	IRRC_WRITE_HHIL(v)	(*(volatile uint16*)IRRC_REG_HHIL	= (v))
#define	IRRC_WRITE_HHIH(v)	(*(volatile uint16*)IRRC_REG_HHIH	= (v))
#define	IRRC_WRITE_D0LOL(v)	(*(volatile uint16*)IRRC_REG_D0LOL	= (v))
#define	IRRC_WRITE_D0LOH(v)	(*(volatile uint16*)IRRC_REG_D0LOH	= (v))
#define	IRRC_WRITE_D0HIL(v)	(*(volatile uint16*)IRRC_REG_D0HIL	= (v))
#define	IRRC_WRITE_D0HIH(v)	(*(volatile uint16*)IRRC_REG_D0HIH	= (v))
#define	IRRC_WRITE_D1LOL(v)	(*(volatile uint16*)IRRC_REG_D1LOL	= (v))
#define	IRRC_WRITE_D1LOH(v)	(*(volatile uint16*)IRRC_REG_D1LOH	= (v))
#define	IRRC_WRITE_D1HIL(v)	(*(volatile uint16*)IRRC_REG_D1HIL	= (v))
#define	IRRC_WRITE_D1HIH(v)	(*(volatile uint16*)IRRC_REG_D1HIH	= (v))
#define	IRRC_WRITE_ENDLENL(v)	(*(volatile uint16*)IRRC_REG_ENDLENL	= (v))
#define	IRRC_WRITE_ENDLENH(v)	(*(volatile uint16*)IRRC_REG_ENDLENH	= (v))
#define	IRRC_WRITE_BITLEN(v)	(*(volatile uint16*)IRRC_REG_BITLEN	= (v))
#define	IRRC_WRITE_FRMLENL(v)	(*(volatile uint16*)IRRC_REG_FRMLENL	= (v))
#define	IRRC_WRITE_FRMLENH(v)	(*(volatile uint16*)IRRC_REG_FRMLENH	= (v))
#define	IRRC_WRITE_OUT(u,v)	(*(volatile uint16*)(IRRC_REG_OUT0+2*(u)) \
									= (v))
#define	IRRC_WRITE_SEND(v)	(*(volatile uint16*)IRRC_REG_SEND	= (v))
#define	IRRC_WRITE_REGS(v)	(*(volatile uint16*)IRRC_REG_REGS	= (v))
#define	IRRC_WRITE_DBG(v)	(*(volatile uint16*)IRRC_REG_DBG	= (v))

#define	INTR_IRRCINT		(uint16)0x0004
#define	INTR_IRRC2INT		(uint16)0x0008

#define	IRRC_SYS_CLEAR		(uint16)0x3130
#define	IRRC_SYS_INIT		(uint16)0x0110
#define	IRRC_BASE_INIT		(uint16)0x0000
#define	IRRC_CLO_INIT		(uint16)0x0119
#define	IRRC_CHI_INIT		(uint16)0x008F
#define	IRRC_HLOL_INIT		(uint16)0x00AB
#define	IRRC_HLOH_INIT		(uint16)0x0000
#define	IRRC_HLO_INIT		(uint32)0x000000AB
#define	IRRC_HHIL_INIT		(uint16)0x0158
#define	IRRC_HHIH_INIT		(uint16)0x0000
#define	IRRC_HHI_INIT		(uint32)0x00000158
#define	IRRC_D0LOL_INIT 	(uint16)0x0014
#define	IRRC_D0LOH_INIT		(uint16)0x0000
#define	IRRC_D0LO_INIT		(uint32)0x00000014
#define	IRRC_D0HIL_INIT		(uint16)0x0014
#define	IRRC_D0HIH_INIT		(uint16)0x0000
#define	IRRC_D0HI_INIT		(uint32)0x00000014
#define	IRRC_D1LOL_INIT		(uint16)0x0014
#define	IRRC_D1LOH_INIT		(uint16)0x0000
#define	IRRC_D1LO_INIT		(uint32)0x00000014
#define	IRRC_D1HIL_INIT		(uint16)0x003C
#define	IRRC_D1HIH_INIT		(uint16)0x0000
#define	IRRC_D1HI_INIT		(uint32)0x0000003C
#define	IRRC_ENDLENL_INIT	(uint16)0x0014
#define	IRRC_ENDLENH_INIT	(uint16)0x0000
#define	IRRC_ENDLEN_INIT	(uint32)0x00000014
#define	IRRC_BITLEN_INIT	(uint16)0x0020
#define	IRRC_FRMLENH_INIT	(uint16)0x0000
#define	IRRC_FRMLENL_INIT	(uint16)0x0000
#define	IRRC_FRMLEN_INIT	(uint32)0x00000000
#define	IRRC_OUT_INIT		(uint16)0x0000
#define	IRRC_SEND_INIT		(uint16)0x0000
#define	IRRC_REGS_INIT		(uint16)0x0000
#define	IRRC_DBG_INIT		(uint16)0x0000

#define IRRC_REG_OUTLEN_MAX	0x0080
#define IRRC_OUT_REG_MAX	8

#define	IRRC_PWR_OFF		(uint16)0xFFFE
#define	IRRC_PWR_ON		(uint16)0x0001
#define	IRRC_INV0		(uint16)0x0002
#define	IRRC_INV1		(uint16)0x0004
#define	IRRC_DIVS		(uint16)0x0010
#define	IRRC_OPM		(uint16)0x0020
#define	IRRC_RPT0		(uint16)0x0100
#define	IRRC_RPT_AREA		(uint16)0x0F00
#define	IRRC_FRME		(uint16)0x1000
#define	IRRC_FRMB		(uint16)0x2000
#define	IRRC_RESET		(uint16)0x8000
#define	IRRC_RESET_CLR		(uint16)0x7FFF
#define	IRRC_SEND		(uint16)0x0001
#define	IRRC_REGS_NOT_REPEAT	(uint16)0x0000
#define	IRRC_REGS_REPEAT	(uint16)0x0001
#define	IRRC_CR_CLEAR		(uint16)0x00
#define	IRRC_CR32_IRSEL		(uint16)0x01
#define IRRC_LED_POWER_NON	(uint16)0x0000
#define IRRC_LED_POWER_LOW	(uint16)0x0001
#define IRRC_LED_POWER_HIGH	(uint16)0x0002
#define	IRRC_DBG_VER2		(uint16)0x0001

int irrc_reg_ioremap_nocache(void);

void irrc_reg_iounmap(void);

void irrc_reg_set_bitlen(int32 len);

void irrc_reg_send(void);

void irrc_reg_clear_int_irrc_factor(void);

void irrc_reg_clear_int_irrc2_factor(void);

void irrc_reg_set_data0(
   int32 pulse0_low, int32 pulse0_high);

void irrc_reg_set_data1(
   int32 pulse1_low, int32 pulse1_high);

void irrc_reg_set_leader(
	int32 leader_low, int32 leader_high);

void irrc_reg_set_trailer(
		int32 trailer_high);

void irrc_reg_set_frame_length(
		int32 frame_length);

void irrc_reg_set_sys(void);

void irrc_reg_set_modulation(
	int16 modulation0, int16 modulation1);

void irrc_reg_set_data(uint16 *reg_data);

void irrc_reg_set_regs(uint16 val);

void irrc_reg_set_dbg(void);

void irrc_reg_set_sys_pwron(void);

void irrc_reg_set_sys_pwroff(void);

void irrc_reg_set_base(void);

void irrc_reg_set_carrier(
  int32 carrier_high, int32 carrier_low);

void irrc_reg_set_init(void);

void irrc_reg_init(void);

void irrc_reg_sys_reset(void);

void irrc_reg_set_irrcdiv(void);

#endif
