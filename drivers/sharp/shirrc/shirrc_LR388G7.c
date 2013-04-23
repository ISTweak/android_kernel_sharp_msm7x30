/* drivers/sharp/shirrc/shirrc_LR388G7.c (Infrared driver)
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

#include <linux/module.h>
#include <asm/io.h>

#include "sharp/shirrc_kdrv.h"
#include "shirrc_kdrv_log.h"
#include "shirrc_LR388G7.h"

void __iomem *irrc_lr388g7_hcs1;

int irrc_reg_ioremap_nocache(void)
{
	int	w_ret	= IRRC_REG_SUCCESS;

	irrc_lr388g7_hcs1 = ioremap_nocache(IRRC_ADDR_HCS1,
						IRRC_ADDR_HCS1_SIZE);
	if (irrc_lr388g7_hcs1 == NULL) {
		w_ret = IRRC_REG_ERROR;
		IRRCLOG_ERROR("ioremap_nocache err irrc_lr388g7_hcs1\n");
	}
	return w_ret;
}

void irrc_reg_iounmap(void)
{
	if (irrc_lr388g7_hcs1 != NULL) {
		iounmap(irrc_lr388g7_hcs1);
	}

	irrc_lr388g7_hcs1 = NULL;
	return;
}

void irrc_reg_set_bitlen(int32 len)
{
	IRRC_WRITE_BITLEN((uint16)len);
	return;
}

void irrc_reg_send(void)
{
	IRRC_WRITE_SEND(IRRC_SEND);
	return;
}

void irrc_reg_clear_int_irrc_factor(void)
{
	IRRC_GLT_WRITE_REG(IRRC_GLT_REG_INTR, ~(INTR_IRRCINT));
	return;
}

void irrc_reg_clear_int_irrc2_factor(void)
{
	IRRC_GLT_WRITE_REG(IRRC_GLT_REG_INTR, ~(INTR_IRRC2INT));
	return;
}

void irrc_reg_set_data0(int32 pulse0_low, int32 pulse0_high)
{
	IRRC_WRITE_D0LOH((uint16)(pulse0_low >> 16));
	IRRC_WRITE_D0LOL((uint16)(pulse0_low & 0x0000ffff));
	IRRC_WRITE_D0HIH((uint16)(pulse0_high >> 16));
	IRRC_WRITE_D0HIL((uint16)(pulse0_high & 0x0000ffff));
	return;
}

void irrc_reg_set_data1(int32 pulse1_low, int32 pulse1_high)
{
	IRRC_WRITE_D1LOH((uint16)(pulse1_low >> 16));
	IRRC_WRITE_D1LOL((uint16)(pulse1_low & 0x0000ffff));
	IRRC_WRITE_D1HIH((uint16)(pulse1_high >> 16));
	IRRC_WRITE_D1HIL((uint16)(pulse1_high & 0x0000ffff));
	return;
}

void irrc_reg_set_leader(int32 leader_low, int32 leader_high)
{
	IRRC_WRITE_HHIH((uint16)(leader_high >> 16));
	IRRC_WRITE_HHIL((uint16)(leader_high & 0x0000ffff));
	IRRC_WRITE_HLOH((uint16)(leader_low >> 16));
	IRRC_WRITE_HLOL((uint16)(leader_low & 0x0000ffff));
	return;
}

void irrc_reg_set_trailer(int32 trailer_high)
{
	IRRC_WRITE_ENDLENH((uint16)(trailer_high >> 16));
	IRRC_WRITE_ENDLENL((uint16)(trailer_high & 0x0000ffff));
	return;
}

void irrc_reg_set_frame_length(int32 frame_length)
{
	IRRC_WRITE_FRMLENH((uint16)(frame_length >> 16));
	IRRC_WRITE_FRMLENL((uint16)(frame_length & 0x0000ffff));
	return;
}

void irrc_reg_set_sys(void)
{
	uint16	w_sys ;

	w_sys =  IRRC_READ_SYS;
	w_sys |= IRRC_OPM;
	w_sys |= IRRC_DIVS;
	w_sys &= (~IRRC_RPT_AREA);
	w_sys |= IRRC_RPT0;
	w_sys |= IRRC_FRME;
	w_sys |= IRRC_FRMB;
	IRRC_WRITE_SYS(w_sys);
	return;
}

void irrc_reg_set_modulation(int16 modulation0, int16 modulation1)
{
	uint16	w_sys ;

	w_sys =  IRRC_READ_SYS;
	if (modulation0 == IRRC_PPM_LOW_HIGH) {
		w_sys |= IRRC_INV0;
	} else {
		w_sys &= (~IRRC_INV0);
	}
	if (modulation1 == IRRC_PPM_LOW_HIGH) {
		w_sys |= IRRC_INV1;
	} else {
		w_sys &= (~IRRC_INV1);
	}
	IRRC_WRITE_SYS(w_sys);
	return;
}

void irrc_reg_set_data(uint16 *reg_data)
{
	uint32	reg_idx;

	for (reg_idx = 0; reg_idx < IRRC_OUT_REG_MAX; reg_idx++) {
		IRRC_WRITE_OUT(reg_idx, *(reg_data + reg_idx));
	}
	return;
}

void irrc_reg_set_regs(uint16 val)
{
	IRRC_WRITE_REGS(val);
	return;
}

void irrc_reg_set_dbg(void)
{
	IRRC_WRITE_DBG(IRRC_DBG_VER2);
	return;
}

void irrc_reg_set_sys_pwron(void)
{
	IRRC_WRITE_SYS((IRRC_READ_SYS & IRRC_RESET_CLR) | IRRC_PWR_ON);
	return;
}

void irrc_reg_set_sys_pwroff(void)
{
	IRRC_WRITE_SYS(IRRC_READ_SYS & IRRC_PWR_OFF);
	return;
}

void irrc_reg_set_base(void)
{
	IRRC_WRITE_BASE(IRRC_BASE_NUM);
	return;
}

void irrc_reg_set_carrier(int32 carrier_high, int32 carrier_low)
{
	IRRC_WRITE_CLO((uint16)carrier_low);
	IRRC_WRITE_CHI((uint16)carrier_high);
	return;
}

void irrc_reg_set_init(void)
{
	IRRC_WRITE_CLO(IRRC_CLO_INIT);
	IRRC_WRITE_CHI(IRRC_CHI_INIT);
	IRRC_WRITE_HLOL(IRRC_HLOL_INIT);
	IRRC_WRITE_HLOH(IRRC_HLOH_INIT);
	IRRC_WRITE_HHIL(IRRC_HHIL_INIT);
	IRRC_WRITE_HHIH(IRRC_HHIH_INIT);
	IRRC_WRITE_D0LOL(IRRC_D0LOL_INIT );
	IRRC_WRITE_D0LOH(IRRC_D0LOH_INIT);
	IRRC_WRITE_D0HIL(IRRC_D0HIL_INIT);
	IRRC_WRITE_D0HIH(IRRC_D0HIH_INIT);
	IRRC_WRITE_D1LOL(IRRC_D1LOL_INIT);
	IRRC_WRITE_D1LOH(IRRC_D1LOH_INIT);
	IRRC_WRITE_D1HIL(IRRC_D1HIL_INIT);
	IRRC_WRITE_D1HIH(IRRC_D1HIH_INIT);
	IRRC_WRITE_ENDLENL(IRRC_ENDLENL_INIT);
	IRRC_WRITE_ENDLENH(IRRC_ENDLENH_INIT);
	IRRC_WRITE_BITLEN(IRRC_BITLEN_INIT);
	IRRC_WRITE_FRMLENL(IRRC_FRMLENL_INIT);
	IRRC_WRITE_FRMLENH(IRRC_FRMLENH_INIT);
	IRRC_WRITE_OUT(0,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(1,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(2,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(3,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(4,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(5,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(6,IRRC_OUT_INIT);
	IRRC_WRITE_OUT(7,IRRC_OUT_INIT);
	IRRC_WRITE_SEND(IRRC_SEND_INIT);
	IRRC_WRITE_REGS(IRRC_REGS_INIT);
	IRRC_WRITE_DBG(IRRC_DBG_INIT);
	return;
}

void irrc_reg_init(void)
{
	IRRC_WRITE_SYS(IRRC_SYS_INIT);
	IRRC_WRITE_BASE(IRRC_BASE_INIT);
	irrc_reg_set_init();
	irrc_reg_clear_int_irrc_factor();
	irrc_reg_clear_int_irrc2_factor();
	return;
}

void irrc_reg_sys_reset(void)
{
	IRRC_WRITE_SYS(IRRC_READ_SYS | IRRC_RESET);
	return;
}

void irrc_reg_set_irrcdiv(void)
{
	IRRC_GLT_WRITE_REG(IRRC_GLT_REG_IRRCDIV, IRRC_IRRCDIV);
	return;
}
MODULE_DESCRIPTION("IrRC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION(SH_IRRC_KERNEL_DRIVER_VERSION);
