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

/*
 * Definitions for driver.
 */
#ifndef AMI_DRIVER_DEF_H
#define AMI_DRIVER_DEF_H

struct ami_pedo_threshold {
	uint8_t rest_out_th;
	uint8_t rest_in_time;
	uint8_t rest_in_cnt;
	uint8_t rest_in_th;
	uint8_t step_up_th;
	uint8_t step_dw_th;
	uint8_t step_min_time;
	uint8_t step_stop_time;
	uint8_t iir_weight1;
	uint8_t iir_weight2;
	uint8_t iir_weight3;
	uint8_t axis_chk_flg;
	uint8_t two_step_min;
	uint8_t two_step_max;
	uint8_t two_step_dif;
	uint8_t two_step_chk;
};

struct ami_pedo_status {
	uint32_t count;			/**< pedmeter */
	uint32_t time;			/**< walk time */
	uint32_t stat;			/**< stat */
};

#endif

