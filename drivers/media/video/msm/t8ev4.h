/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef T8EV4_H
#define T8EV4_H
#include <linux/types.h>
#include <mach/board.h>
extern struct t8ev4_reg t8ev4_regs;
struct t8ev4_i2c_reg_conf {
	unsigned short waddr;
	unsigned short sdata;
};

enum t8ev4_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum t8ev4_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};
enum t8ev4_setting {
	RES_PREVIEW,
	RES_CAPTURE
};
enum t8ev4_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum t8ev4_reg_pll {
	E_EXTCLK_FREQ_HI = 8,
	E_EXTCLK_FREQ_LO,
	E_PLL_MULTIPLIER_HI,
	E_PLL_MULTIPLIER_LO,
	E_PRE_PLL_CLK_DIV,
	E_VT_PIX_CLK_DIV,
	E_VT_SYS_CLK_DIV,
	E_OP_PIX_CLK_DIV,
	E_OP_SYS_CLK_DIV,
};

enum t8ev4_reg_mode {
	E_FRAME_LENGTH_LINES_HI = 17,
	E_FRAME_LENGTH_LINES_LO,
	E_LINE_LENGTH_PCK_HI,
	E_LINE_LENGTH_PCK_LO,
	E_XADDR_START_MSB,
	E_XADDR_START_LSB,
	E_YADDR_START_MSB,
	E_YADDR_START_LSB,
	E_XADDR_END_MSB,
	E_XADDR_END_LSB,
	E_YADDR_END_MSB,
	E_YADDR_END_LSB,
	E_X_OUTPUT_SIZE_MSB,
	E_X_OUTPUT_SIZE_LSB,
	E_Y_OUTPUT_SIZE_MSB,
	E_Y_OUTPUT_SIZE_LSB,
};

struct t8ev4_reg {
//	const struct t8ev4_i2c_reg_conf *reg_init;
//	const unsigned short reg_init_size;
	const struct t8ev4_i2c_reg_conf *reg_prev;
	const unsigned short reg_prev_size;
	const struct t8ev4_i2c_reg_conf *reg_snap;
	const unsigned short reg_snap_size;
};

#endif /* T8EV4_H */
