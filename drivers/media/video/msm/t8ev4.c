/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/debugfs.h>
#if 1
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/clk.h>
#endif
#include "t8ev4.h"

#define WAIT_SWS
#define WAIT_SNAPSHOT

/*SENSOR REGISTER DEFINES*/
//#define Q8 4096
#define REG_GROUPED_PARAMETER_HOLD			0x0104
#define GROUPED_PARAMETER_HOLD_OFF			0x00
#define GROUPED_PARAMETER_HOLD				0x01
#define REG_MODE_SELECT					0x100
#define MODE_SELECT_STANDBY_MODE			0x00
#define MODE_SELECT_STREAM				0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME_HI			0x0202
#define REG_COARSE_INTEGRATION_TIME_LO			0x0203
/* Gain */
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_HI		0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LO		0x0205

#define REG_FRAME_LENGTH_LINES_HI	    0x0340
#define REG_FRAME_LENGTH_LINES_LO	    0x0341
/*Test Pattern register*/

#define REG_TEST_PATTERN_MODE_HI               0x0600
#define REG_TEST_PATTERN_MODE_LO               0x0601

/*..... TYPE DECLARATIONS.....*/
#define	T8EV4_OFFSET					8
#define	T8EV4_DEFAULT_MASTER_CLK_RATE        9600000
#define Q12				0x00001000
/* Full	Size */
#define	T8EV4_FULL_SIZE_WIDTH				3280
#define	T8EV4_FULL_SIZE_HEIGHT				2464
#define	T8EV4_FULL_SIZE_DUMMY_PIXELS			0
#define	T8EV4_FULL_SIZE_DUMMY_LINES			0
/* Quarter Size	*/
#define	T8EV4_QTR_SIZE_WIDTH				1632
#define	T8EV4_QTR_SIZE_HEIGHT				1176
#define	T8EV4_QTR_SIZE_DUMMY_PIXELS			0
#define	T8EV4_QTR_SIZE_DUMMY_LINES			0
/* Blanking as measured	on the scope */
/* Full	Size */
#define	T8EV4_HRZ_FULL_BLK_PIXELS			344
#define	T8EV4_VER_FULL_BLK_LINES			36
/* Quarter Size	*/
#define	T8EV4_HRZ_QTR_BLK_PIXELS			180
#define	T8EV4_VER_QTR_BLK_LINES			    57//29.5fps        38//30.0fps
#define	Q8						0x100
#define	Q10						0x400

#define	T8EV4_STEPS_NEAR_TO_CLOSEST_INF		40
#if 0
#define	T8EV4_TOTAL_STEPS_NEAR_TO_FAR		40
#else
#define	T8EV4_TOTAL_STEPS_NEAR_TO_FAR		60
#endif
#define T8EV4_AF_SLAVE_ADDR	(0x0C)

/*STEP_RESOLUTION_VAL = 1; SINGLESTEP_TIME_SETTING = 8*/
#define AF_STEP_OPER_NORMAL 0x0108
/*STEP_RESOLUTION_VAL = 1; SINGLESTEP_TIME_SETTING = 2*/
#define AF_STEP_OPER_JUMP 0x0102
/*STEP_RESOLUTION_VAL = 1; SINGLESTEP_TIME_SETTING = 4*/
#define AF_STEP_OPER_MEDJUMP 0x0104

/*Direct Mode DAC code setting*/
#define AF_DIRECT_MODE_ADDR (0xC0)
/*Step Mode DAC code setting*/
#define AF_STEP_MODE_ADDR (0xC8)
/*Step Time Setting and Resolution Setting in Step Mode*/
#define AF_STS_RS_VALUE_ADDR (0xCC)
/*Stand by mode place in high impedance state*/
#define AF_DISABLE (0x00)
#if 1
#define MIPI_ERROR_AVOIDANCE
#endif

#if 0
uint16_t t8ev4_step_position_table[T8EV4_TOTAL_STEPS_NEAR_TO_FAR+1];
#else
uint16_t* t8ev4_step_position_table;
uint16_t* t8ev4_af_default_table;
uint16_t t8ev4_af_default_total_steps_near_to_far = T8EV4_TOTAL_STEPS_NEAR_TO_FAR;
uint16_t t8ev4_af_total_steps_near_to_far = T8EV4_TOTAL_STEPS_NEAR_TO_FAR;
uint16_t t8ev4_af_normal_default_code = 225;
uint16_t t8ev4_af_macro_default_code = 500;
uint16_t t8ev4_af_full_default_code = 225;
uint16_t t8ev4_af_product_default_code = 0;
#endif

uint16_t t8ev4_l_region_code_per_step = 9;
uint16_t t8ev4_af_initial_code = 190;
uint16_t t8ev4_hw_damping_time_wait = 1;

#ifdef WAIT_SWS
uint16_t  t8ev4_frame_length_lines = (T8EV4_QTR_SIZE_HEIGHT + T8EV4_VER_QTR_BLK_LINES) << 3; //specify min fps = 3.75
#endif
#ifdef WAIT_SNAPSHOT
static uint16_t t8ev4_s_gain = 0;
static uint16_t t8ev4_p_gain = 0;
static uint16_t t8ev4_p_line = 0;
static uint16_t t8ev4_p_frame_lines = 0;
#endif

#if 1
static uint16_t min_fps;
#endif

#if 0
static int cam_debug_init(void);
static struct dentry *debugfs_base;
#endif

struct t8ev4_work_t {
	struct work_struct work;
};

static struct t8ev4_work_t *t8ev4_sensorw;
static struct i2c_client *t8ev4_client;

struct t8ev4_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;
	int16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;
	enum t8ev4_resolution_t prev_res;
	enum t8ev4_resolution_t pict_res;
	enum t8ev4_resolution_t curr_res;
	enum t8ev4_test_mode_t set_test;
	unsigned short imgaddr;
};
static uint16_t t8ev4_delay_msecs_stream = 60;
static int32_t CSI_CONFIG;

static struct t8ev4_ctrl_t *t8ev4_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(t8ev4_wait_queue);
DEFINE_MUTEX(t8ev4_mut);

#if 1
static int32_t t8ev4_init_af_table(struct sensor_cfg_data *cdata);
static int32_t t8ev4_set_af_default_code(struct focus_default_cfg *data);
static int32_t t8ev4_set_default_focus_mode(uint8_t focus_mode);
static int t8ev4_sensor_probe2(const struct msm_camera_sensor_info *info,
                               struct msm_sensor_ctrl *s);

static int t8ev4_otpContStaWrite(unsigned char bdata);
static int t8ev4_otpPselWrite(unsigned char bpage);
#endif

/*=============================================================*/

static int t8ev4_i2c_rxdata(unsigned short saddr,
							 unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(t8ev4_client->adapter, msgs, 2) < 0) {
		CDBG("t8ev4_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}

static int32_t t8ev4_i2c_txdata(unsigned short saddr,
								 unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(t8ev4_client->adapter, msg, 1) < 0) {
		CDBG("t8ev4_i2c_txdata faild 0x%x\n", 0x6F >> 1 );
		return -EIO;
	}

	return 0;
}

static int32_t t8ev4_i2c_read(unsigned short raddr,
							   unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = t8ev4_i2c_rxdata(t8ev4_client->addr  , buf, rlen);
	if (rc < 0) {
		printk(KERN_ERR"t8ev4_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG(KERN_ERR "t8ev4 the i2c read is successful 0x%x\n",raddr);
	return rc;
}
static int16_t t8ev4_i2c_write_w_af(uint8_t baddr,
					uint16_t wdata)
{
	int32_t rc;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = (wdata & 0xFF00) >> 8;
	buf[2] = (wdata & 0x00FF);
	rc = t8ev4_i2c_txdata(T8EV4_AF_SLAVE_ADDR, buf, 3);
	if (rc < 0)
		CDBG("afi2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!",
			T8EV4_AF_SLAVE_ADDR, baddr, wdata);
	return rc;
}

static int16_t t8ev4_i2c_write_b_af(uint8_t baddr,
					uint8_t bdata)
{
	int32_t rc;
	unsigned char buf[2];
	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;

	rc = t8ev4_i2c_txdata(T8EV4_AF_SLAVE_ADDR, buf, 2);
	if (rc < 0)
		CDBG("afi2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!",
			T8EV4_AF_SLAVE_ADDR, baddr, bdata);
	return rc;
}

static int32_t t8ev4_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = t8ev4_i2c_txdata(0x6F >> 1 , buf, 3);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			 waddr, bdata);
	}
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	return rc;
}

static int32_t t8ev4_i2c_write_w_table(struct t8ev4_i2c_reg_conf const
										*reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = t8ev4_i2c_write_b_sensor(reg_conf_tbl->waddr,
									reg_conf_tbl->sdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void t8ev4_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint16_t preview_frame_length_lines, snapshot_frame_length_lines;
	uint16_t preview_line_length_pck, snapshot_line_length_pck;
	uint32_t divider, d1, d2;

	/* Total frame_length_lines and line_length_pck for preview */
	preview_frame_length_lines = T8EV4_QTR_SIZE_HEIGHT +
								 T8EV4_VER_QTR_BLK_LINES;
	preview_line_length_pck = T8EV4_QTR_SIZE_WIDTH +
							  T8EV4_HRZ_QTR_BLK_PIXELS;
	/* Total frame_length_lines and line_length_pck for snapshot */
	snapshot_frame_length_lines = T8EV4_FULL_SIZE_HEIGHT +
								  T8EV4_VER_FULL_BLK_LINES;
	snapshot_line_length_pck = T8EV4_FULL_SIZE_WIDTH +
							   T8EV4_HRZ_FULL_BLK_PIXELS;
	d1 = preview_frame_length_lines * 0x400 /
		 snapshot_frame_length_lines;
	d2 = preview_line_length_pck * 0x400 /
		 snapshot_line_length_pck;

	CDBG("Get pict fps \n");
	divider = d1 * d2  / 0x400;
	*pfps = (uint16_t) (fps * divider  / 0x400);
	CDBG("snapshot fps is %d\n", *pfps);

}

static uint16_t t8ev4_get_prev_lines_pf(void)
{
	if (t8ev4_ctrl->prev_res == QTR_SIZE)
		return T8EV4_QTR_SIZE_HEIGHT + T8EV4_VER_QTR_BLK_LINES;
	else
		return T8EV4_FULL_SIZE_HEIGHT + T8EV4_VER_FULL_BLK_LINES;

}

static uint16_t t8ev4_get_prev_pixels_pl(void)
{
	if (t8ev4_ctrl->prev_res == QTR_SIZE)
		return T8EV4_QTR_SIZE_WIDTH + T8EV4_HRZ_QTR_BLK_PIXELS;
	else
		return T8EV4_FULL_SIZE_WIDTH +	T8EV4_HRZ_FULL_BLK_PIXELS;
}

static uint16_t t8ev4_get_pict_lines_pf(void)
{
	if (t8ev4_ctrl->pict_res == QTR_SIZE)
		return T8EV4_QTR_SIZE_HEIGHT +
		T8EV4_VER_QTR_BLK_LINES;
	else
		return T8EV4_FULL_SIZE_HEIGHT +
		T8EV4_VER_FULL_BLK_LINES;
}

static uint16_t t8ev4_get_pict_pixels_pl(void)
{
	if (t8ev4_ctrl->pict_res == QTR_SIZE)
		return T8EV4_QTR_SIZE_WIDTH +
		T8EV4_HRZ_QTR_BLK_PIXELS;
	else
		return T8EV4_FULL_SIZE_WIDTH +
		T8EV4_HRZ_FULL_BLK_PIXELS;
}

static uint32_t t8ev4_get_pict_max_exp_lc(void)
{
	if (t8ev4_ctrl->pict_res == QTR_SIZE)
		return(T8EV4_QTR_SIZE_HEIGHT +
			   T8EV4_VER_QTR_BLK_LINES)*24;
	else
		return(T8EV4_FULL_SIZE_HEIGHT +
			   T8EV4_VER_FULL_BLK_LINES)*24;
}

static int32_t t8ev4_set_fps(struct fps_cfg    *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	t8ev4_ctrl->fps_divider = fps->fps_div;
	t8ev4_ctrl->pict_fps_divider = fps->pict_fps_div;
	t8ev4_ctrl->fps = fps->f_mult;

#ifdef WAIT_SWS
	if (t8ev4_ctrl->sensormode == SENSOR_PREVIEW_MODE){
		total_lines_per_frame = (uint16_t)(( T8EV4_QTR_SIZE_HEIGHT +
		 	 T8EV4_VER_QTR_BLK_LINES) *
			t8ev4_ctrl->fps_divider/0x400);
		t8ev4_frame_length_lines = total_lines_per_frame;
	}else{
		total_lines_per_frame = (uint16_t)(( T8EV4_FULL_SIZE_HEIGHT +
			 T8EV4_VER_FULL_BLK_LINES) *
			t8ev4_ctrl->pict_fps_divider/0x400);
	}
#else
	if (t8ev4_ctrl->sensormode == SENSOR_PREVIEW_MODE)
		total_lines_per_frame = (uint16_t)(( T8EV4_QTR_SIZE_HEIGHT +
		 	 T8EV4_VER_QTR_BLK_LINES) *
			t8ev4_ctrl->fps_divider/0x400);
	else
		total_lines_per_frame = (uint16_t)(( T8EV4_FULL_SIZE_HEIGHT +
			 T8EV4_VER_FULL_BLK_LINES) *
			t8ev4_ctrl->pict_fps_divider/0x400);
#endif

	t8ev4_i2c_write_b_sensor(0x0104,1);
	rc = t8ev4_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_HI,
		(uint8_t)((total_lines_per_frame & 0xFF00) >> 8));
	if (rc < 0)
		return rc;

	rc = t8ev4_i2c_write_b_sensor( REG_FRAME_LENGTH_LINES_LO,
		(uint8_t)(total_lines_per_frame & 0x00FF));

	t8ev4_i2c_write_b_sensor(0x0104,0);
	return rc;
}

static int32_t t8ev4_write_exp_gain(uint16_t gain, uint32_t line)
{
	static uint16_t max_legal_gain = 0x0240;
	uint8_t gain_msb = 0, gain_lsb = 0;
	uint8_t intg_time_msb = 0, intg_time_lsb = 0;
	uint16_t  frame_length_lines;
	uint8_t frame_length_line_msb, frame_length_line_lsb;
	int32_t rc = -1;

	CDBG("t8ev4_write_exp_gain : gain = %d line = %d\n", gain, line);
	CDBG("t8ev4_write_exp_gain : %d\n",__LINE__);
	if (t8ev4_ctrl->curr_res == QTR_SIZE) {
		frame_length_lines = T8EV4_QTR_SIZE_HEIGHT +
			T8EV4_VER_QTR_BLK_LINES;
		frame_length_lines = frame_length_lines *
			t8ev4_ctrl->fps_divider / 0x400;
		CDBG("t8ev4_write_exp_gain : %d\n",__LINE__);
#ifdef WAIT_SNAPSHOT
		t8ev4_p_gain = gain;
		t8ev4_p_line = line;
		t8ev4_p_frame_lines = frame_length_lines;
#endif
	} else {
		frame_length_lines = T8EV4_FULL_SIZE_HEIGHT +
			 T8EV4_VER_FULL_BLK_LINES;
		frame_length_lines = frame_length_lines *
			t8ev4_ctrl->pict_fps_divider / 0x400;
#ifdef WAIT_SNAPSHOT
		t8ev4_s_gain = gain;
#endif
	}
	if (line > (frame_length_lines - T8EV4_OFFSET))
		frame_length_lines = line + T8EV4_OFFSET;

	/* range: 0 to 576 */
	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* update gain registers */
	gain_msb = (uint8_t) ((gain & 0xFF00) >> 8);
	gain_lsb = (uint8_t) (gain & 0x00FF);

	frame_length_line_msb = (uint8_t) ((frame_length_lines & 0xFF00) >> 8);
	frame_length_line_lsb = (uint8_t) (frame_length_lines & 0x00FF);

#ifdef WAIT_SWS
	t8ev4_frame_length_lines = frame_length_lines;
#endif
	/* update line count registers */
	intg_time_msb = (uint8_t) ((line & 0xFF00) >> 8);
	intg_time_lsb = (uint8_t) (line & 0x00FF);

	CDBG("gain regs: Mode is %d\n",t8ev4_ctrl->sensormode);
	/*Grouped parameter hold*/
	t8ev4_i2c_write_b_sensor(0x0104,1);
	CDBG("gain GAIN_CODE_GLOBAL_HI: %x, GAIN_CODE_GLOBAL_LO: %x\n",
		gain_msb, gain_lsb);
	rc = t8ev4_i2c_write_b_sensor(REG_ANALOGUE_GAIN_CODE_GLOBAL_LO,
		gain_lsb);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(REG_ANALOGUE_GAIN_CODE_GLOBAL_HI,
		gain_msb);
	if (rc < 0)
		return rc;

	CDBG("t8ev4_write_exp_gain : min_fps = %d\n", min_fps);
    if (min_fps > 0) {
	CDBG("framelengthline FRAME_LEN_HI: %x, FRAME_LEN_LO: %x\n",
		frame_length_line_msb, frame_length_line_lsb);
	rc = t8ev4_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_LO,
		frame_length_line_lsb);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_HI,
		frame_length_line_msb);
	if (rc < 0)
		return rc;
    }

	CDBG("CIT: INTEGRATION_TIME_HI: %x,INTEGRATION_TIME_LO: %x\n",
		intg_time_msb, intg_time_lsb);
	rc = t8ev4_i2c_write_b_sensor(REG_COARSE_INTEGRATION_TIME_LO,
		intg_time_lsb);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(REG_COARSE_INTEGRATION_TIME_HI,
		intg_time_msb);
	if (rc < 0)
		return rc;
	t8ev4_i2c_write_b_sensor(0x0104,0);
	return rc;
}

static int32_t t8ev4_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = t8ev4_write_exp_gain(gain, line);
	return rc;
}

static int16_t t8ev4_af_init(void)
{
	int32_t rc = 0;
#if 0
	uint8_t i;
#endif
	unsigned short vcmval, vcmcode10cm, vcmcode50cm, vcmcodesc;

	CDBG("%s E\n", __func__);

	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x01);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(0x3402, 0x09);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x81);
	if (rc < 0)
		return rc;
	usleep(30);

	rc = t8ev4_i2c_read(0x3406, &vcmval, 1);
	if (rc < 0)
		return rc;
	CDBG("%s vcmval =0x%X\n", __func__, vcmval);


	rc = t8ev4_i2c_read(0x3407, &vcmcode10cm, 1);
	if (rc < 0)
		return rc;
	vcmcode10cm |= ((vcmval& 0x0030) << 4);
	CDBG("%s vcmcode10cm =%d\n", __func__, vcmcode10cm);

	rc = t8ev4_i2c_read(0x3408, &vcmcode50cm, 1);
	if (rc < 0)
		return rc;
	vcmcode50cm |= ((vcmval& 0x000C) << 6);
	CDBG("%s vcmcode50cm =%d\n", __func__, vcmcode50cm);

	rc = t8ev4_i2c_read(0x3409, &vcmcodesc, 1);
	if (rc < 0)
		return rc;
	vcmcodesc |= ((vcmval& 0x0003)<< 8);
	CDBG("%s vcmcodesc =%d\n", __func__, vcmcodesc);

	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x01);
	if (rc < 0)
		return rc;
	/*80% of 1024 is 0x332, we are considering the 80% of upward direction for horizontal direction.*/
	t8ev4_af_initial_code = vcmcodesc * 0x332 / 0x400;
	CDBG("%s t8ev4_af_initial_code =%d\n", __func__, t8ev4_af_initial_code);
#if 0
	t8ev4_step_position_table[0] = t8ev4_af_initial_code;
	for (i = 1; i <= T8EV4_TOTAL_STEPS_NEAR_TO_FAR; i++) {
			t8ev4_step_position_table[i] =
			t8ev4_step_position_table[i-1] +
			t8ev4_l_region_code_per_step;
		if (t8ev4_step_position_table[i] > 1023)
			t8ev4_step_position_table[i] = 1023;
	}
#endif

	t8ev4_ctrl->curr_lens_pos = 0;
	CDBG("%s X\n", __func__);
	return rc;
}

static int32_t t8ev4_move_focus(int direction,
	int32_t num_steps)
{
	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	uint8_t codeval_msb, codeval_lsb;

	CDBG("%s E\n",__func__);

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else if (direction == MOVE_FAR)
		step_direction = -1;
	else{
		pr_err("Illegal focus direction\n");
		return -EINVAL;
	}

	dest_step_position = t8ev4_ctrl->curr_step_pos +
			(step_direction * num_steps);
	if (dest_step_position < 0)
		dest_step_position = 0;
#if 0
	else if (dest_step_position > T8EV4_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = T8EV4_TOTAL_STEPS_NEAR_TO_FAR;
#else
	else if (dest_step_position > t8ev4_af_total_steps_near_to_far)
		dest_step_position = t8ev4_af_total_steps_near_to_far;
#endif

	if (dest_step_position == t8ev4_ctrl->curr_step_pos) {
		CDBG("t8ev4_move_focus ==  t8ev4_ctrl->curr_step_pos:exit\n");
		return rc;
	}

	dest_lens_position = t8ev4_step_position_table[dest_step_position];
	if (step_direction < 0) {
		if (num_steps >= 20) {
			rc = t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
				AF_STEP_OPER_JUMP);
			CDBG("%s negative direction AF_STEP_OPER_JUMP\n", __func__);
			if (rc < 0)
				return rc;
		t8ev4_hw_damping_time_wait = 2;
		} else if (num_steps <= 4) {
			rc = t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
					AF_STEP_OPER_NORMAL);
		CDBG("%s negative direction AF_STEP_OPER_NORMAL\n", __func__);
			if (rc < 0)
				return rc;
		t8ev4_hw_damping_time_wait = 8;
		} else {
			rc = t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
				AF_STEP_OPER_MEDJUMP);
		CDBG("%s negative direction AF_STEP_OPER_MEDJUMP\n", __func__);
			if (rc < 0)
				return rc;
		t8ev4_hw_damping_time_wait = 4;
		}
	} else {
		rc = t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
				AF_STEP_OPER_NORMAL);
		CDBG("%s +ve direction AF_STEP_OPER_NORMAL\n", __func__);
		if (rc < 0)
			return rc;
		t8ev4_hw_damping_time_wait = 8;
	}
	CDBG("%s line %d index =[%d] = value = %d\n",
			__func__, __LINE__, dest_step_position, dest_lens_position);
	if (t8ev4_ctrl->curr_lens_pos != dest_lens_position) {
		codeval_msb = (AF_STEP_MODE_ADDR |
					((dest_lens_position & 0x0300)>>8));
		codeval_lsb = dest_lens_position & 0x00FF;
		rc = t8ev4_i2c_write_b_af(codeval_msb, codeval_lsb);
		CDBG("%s normal step\n", __func__);
		if (rc < 0) {
			CDBG("t8ev4 I2C Failed line %d\n", __LINE__);
			return rc;
		}
		usleep(t8ev4_hw_damping_time_wait*50);
	}
	t8ev4_ctrl->curr_lens_pos = dest_lens_position;
	t8ev4_ctrl->curr_step_pos = dest_step_position;

	CDBG("%s X\n", __func__);
	return rc;
}

static int32_t t8ev4_set_default_focus(void)
{
	int32_t rc = 0;
	CDBG("%s ==  enter\n", __func__);
	if (t8ev4_ctrl->curr_step_pos != 0)
		rc = t8ev4_move_focus(MOVE_FAR,
		t8ev4_ctrl->curr_step_pos);
	else {
		CDBG("%s defaultfocus stepmode\n", __func__);
		rc = t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
				AF_STEP_OPER_NORMAL);
		if (rc < 0)
			return rc;

		rc = t8ev4_i2c_write_b_af(AF_STEP_MODE_ADDR,
				(uint8_t)t8ev4_af_initial_code);
		if (rc < 0)
			return rc;
		CDBG("%s defaultfocus step mode\n", __func__);
	}
	usleep(300);
	CDBG("%s ==  exit\n", __func__);
	t8ev4_ctrl->curr_lens_pos = t8ev4_af_initial_code;
	t8ev4_ctrl->curr_step_pos = 0;

	return rc;
}

static void t8ev4_af_power_down(void)
{
	CDBG("%s ==  enter\n", __func__);
	if(t8ev4_ctrl->curr_lens_pos != 0) {
		t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
			AF_STEP_OPER_NORMAL);
		t8ev4_i2c_write_b_af(AF_STEP_MODE_ADDR,
			AF_DISABLE);
		CDBG("%s ==  Zerodown\n", __func__);
#if 0
		msleep(200);
#else
		msleep(10);
#endif
		CDBG("%s ==  Powerdown\n", __func__);
		t8ev4_i2c_write_b_af(AF_DISABLE,
				AF_DISABLE);
	}
	CDBG("%s ==  exit\n", __func__);

}
#if 0
static int32_t t8ev4_test(enum t8ev4_test_mode_t mo)
{

	int32_t rc = 0;
	return rc;
	if (mo == TEST_OFF)
		return rc;
	else {
		t8ev4_i2c_write_b_sensor(REG_TEST_PATTERN_MODE_LO, 0x2);

	}
	return rc;
}
#endif

static int32_t t8ev4_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;

#ifdef WAIT_SWS
	int16_t wait_ms;
#endif
#ifdef WAIT_SNAPSHOT
	int16_t snap_wait_ms, preview_rate, snapshot_rate;
#endif

	struct msm_camera_csi_params t8ev4_csi_params;
	if (update_type == REG_INIT) {
		CSI_CONFIG = 0;
	} else if (update_type == UPDATE_PERIODIC) {

		rc = t8ev4_i2c_write_b_sensor(REG_MODE_SELECT,
									   MODE_SELECT_STANDBY_MODE);
#ifdef WAIT_SWS
	if(rt == RES_CAPTURE){
		wait_ms = (int16_t)(t8ev4_frame_length_lines * 33 / 1214 + 10);
	}else{
        if(!CSI_CONFIG) {
    		wait_ms = 10;
        } else {
    		wait_ms = (int16_t)(t8ev4_frame_length_lines * 138 / 2500 + 10);
        }
	}
	msleep((unsigned int)wait_ms);
	CDBG("%s: WAIT_SWS == %d\n", __func__,(unsigned int)wait_ms);
#else
	msleep(134);
#endif

		if (!CSI_CONFIG) {
			CDBG("%s:%d\n",__func__,__LINE__);
			msm_camio_vfe_clk_rate_set(192000000);
			t8ev4_csi_params.lane_cnt = 2;
			t8ev4_csi_params.data_format = CSI_10BIT;
			t8ev4_csi_params.lane_assign = 0xe4;
			t8ev4_csi_params.dpcm_scheme = 0;
			t8ev4_csi_params.settle_cnt = 20;
#if !defined(MIPI_ERROR_AVOIDANCE)
			rc = msm_camio_csi_config(&t8ev4_csi_params);
			msleep(20);
			CSI_CONFIG = 1;
#endif /* MIPI_ERROR_AVOIDANCE */
		}
#if 0
		cam_debug_init();
#endif
		if (rt == RES_PREVIEW) {
#if 1
			t8ev4_i2c_write_b_sensor(0x33C5, 0x81);
#endif
			t8ev4_i2c_write_b_sensor(0x0104,1);
			if(CSI_CONFIG) {
#ifdef WAIT_SNAPSHOT
				#define T8EV4_PREVIEW_FPS   30  /* 30FPS */
				if (t8ev4_p_frame_lines != 0 && t8ev4_s_gain != 0) {
					preview_rate = (t8ev4_p_line * 1000)
								 / (T8EV4_PREVIEW_FPS * t8ev4_p_frame_lines);
					snapshot_rate = (t8ev4_p_gain * preview_rate)
								  / t8ev4_s_gain;

					snap_wait_ms = snapshot_rate + 10;
					if (snap_wait_ms < 100) {
						snap_wait_ms = 100;
					}
					t8ev4_p_frame_lines = 0;
				}
				else {
					snap_wait_ms = 400;  /* 2.5FPS */
				}
				CDBG("%s: from snapshot wait %d\n",__func__, snap_wait_ms);
				msleep(snap_wait_ms);
#else
				msleep(300);
#endif
			}
			else {
#if 0
				msleep(50);
#else
				msleep(10);
#endif
			}
			t8ev4_i2c_write_w_table(t8ev4_regs.reg_prev,
									 t8ev4_regs.reg_prev_size);
			t8ev4_i2c_write_b_sensor(0x0104,0);
		} else {
			CDBG("%s:%d\n",__func__,__LINE__);
#if 1
			t8ev4_i2c_write_b_sensor(0x33C5, 0x80);
#endif
			t8ev4_i2c_write_b_sensor(0x0104,1);
			t8ev4_i2c_write_w_table(t8ev4_regs.reg_snap,
									 t8ev4_regs.reg_snap_size);
			t8ev4_i2c_write_b_sensor(0x0104,0);

		}
#if 0
		rc = t8ev4_test(t8ev4_ctrl->set_test);
		CDBG("%s:%d\n",__func__,__LINE__);
#endif

#if 0
		rc = t8ev4_i2c_write_b_sensor(REG_MODE_SELECT,
									   MODE_SELECT_STREAM);
#else
		rc = t8ev4_i2c_write_b_sensor(0x332E, 0xBF);
		rc = t8ev4_i2c_write_b_sensor(REG_MODE_SELECT,
									   MODE_SELECT_STREAM);
		rc = t8ev4_i2c_write_b_sensor(0x332E, 0x3F);
#endif

		if (rc < 0)
			return rc;

#if defined(MIPI_ERROR_AVOIDANCE)
		if(!CSI_CONFIG) {
			msleep(1);
			rc = msm_camio_csi_config(&t8ev4_csi_params);
            msleep(35);	/* LP11 wait */
			CSI_CONFIG = 1;
		}
		else
		{
			msleep(t8ev4_delay_msecs_stream);
		}
#else
		msleep(t8ev4_delay_msecs_stream);
#endif /* MIPI_ERROR_AVOIDANCE */
	}
	return rc;
}

static int32_t t8ev4_video_config(int mode)
{

	int32_t rc = 0;
	int rt;
	/* change sensor resolution	if needed */
	if (t8ev4_ctrl->prev_res == QTR_SIZE) {
		rt = RES_PREVIEW;
	} else {
		rt = RES_CAPTURE;
	}
	rc = t8ev4_sensor_setting(UPDATE_PERIODIC, rt);
	if (rc < 0)
		return rc;

	t8ev4_ctrl->curr_res = t8ev4_ctrl->prev_res;
	t8ev4_ctrl->sensormode = mode;
	return rc;
}

static int32_t t8ev4_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	if (t8ev4_ctrl->curr_res != t8ev4_ctrl->pict_res) {
		if (t8ev4_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
		} else {
			rt = RES_CAPTURE;
		}
	}
	rc = t8ev4_sensor_setting(UPDATE_PERIODIC, rt);
	if (rc < 0)
		return rc;

	t8ev4_ctrl->curr_res = t8ev4_ctrl->pict_res;
	t8ev4_ctrl->sensormode = mode;
	return rc;
}
static int32_t t8ev4_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	if (t8ev4_ctrl->curr_res != t8ev4_ctrl->pict_res) {
		if (t8ev4_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
		} else {
			rt = RES_CAPTURE;
		}
	}
	rc = t8ev4_sensor_setting(UPDATE_PERIODIC, rt);
	if (rc < 0)
		return rc;

	t8ev4_ctrl->curr_res = t8ev4_ctrl->pict_res;
	t8ev4_ctrl->sensormode = mode;
	return rc;
}
static int32_t t8ev4_set_sensor_mode(int mode,
									  int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		t8ev4_ctrl->prev_res = res;
		rc = t8ev4_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		t8ev4_ctrl->pict_res = res;
		rc = t8ev4_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		t8ev4_ctrl->pict_res = res;
		rc = t8ev4_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}
static int32_t t8ev4_power_down(void)
{
	t8ev4_i2c_write_b_sensor(REG_MODE_SELECT,
							  MODE_SELECT_STANDBY_MODE);
	return 0;
}
#if 0
static int t8ev4_probe_init_done(const struct msm_camera_sensor_info *data)
{
	gpio_direction_input(data->sensor_reset);
	gpio_free(data->sensor_reset);
	return 0;
}
#endif

static int t8ev4_read_calib(struct sensor_cfg_data *cfg)
{
	int32_t rc = 0;
	uint16_t addr = 0;
#if 1
	uint8_t offset = 0;
	unsigned short act_flg;
#endif
	unsigned short gain_msb, gain_r, gain_gr, gain_b, gain_gb;
	unsigned short blevel_lsb, r_blevel, gr_blevel, b_blevel, gb_blevel;

	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x01);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(0x3402, 0x09);
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x81);
	if (rc < 0)
		return rc;
	usleep(30);

#if 1
	offset = 0x00;
	addr = 0x3414;
	rc = t8ev4_i2c_read(addr, &act_flg,1);
	if (rc < 0) {
		pr_err("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	if (act_flg != 0x55) {
		offset = 0x08;
		addr += offset;
		rc = t8ev4_i2c_read(addr, &act_flg,1);
		if (rc < 0) {
			pr_err("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
			return rc;
		}
		if (act_flg != 0x55) {
			pr_err("%s: Error Reading OTP\n", __func__);
			t8ev4_i2c_write_b_sensor(0x3400, 0x01);
			return -EFAULT;
		}
	}
#endif

#if 0
	addr = 0x3416;
#else
	addr = 0x3416 + offset;
#endif
	rc = t8ev4_i2c_read(addr, &gain_msb,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}

#if 0
	addr = 0x3417;
#else
	addr = 0x3417 + offset;
#endif
	rc = t8ev4_i2c_read(addr, &gain_r,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	gain_r |= ((gain_msb & 0xC0) << 2);
	cfg->cfg.wb_calib_info.r= gain_r;

#if 0
	addr = 0x3418;
#else
	addr = 0x3418 + offset;
#endif
	rc = t8ev4_i2c_read(addr, &gain_gr,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	gain_gr |= ((gain_msb & 0x30) << 4);
	cfg->cfg.wb_calib_info.gr= gain_gr;

#if 0
	addr = 0x3419;
#else
	addr = 0x3419 + offset;
#endif
	rc = t8ev4_i2c_read(addr, &gain_b,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	gain_b |= ((gain_msb & 0xC) << 6);
	cfg->cfg.wb_calib_info.b = gain_b;

#if 0
	addr = 0x341A;
#else
	addr = 0x341A + offset;
#endif
	rc = t8ev4_i2c_read(addr, &gain_gb,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	gain_gb |= ((gain_msb & 0x3) << 8);
	cfg->cfg.wb_calib_info.gb = gain_gb;

	CDBG("WB calibration data: gain_msb = %d\n", gain_msb);
	CDBG("WB calibration data: gain_r = %d\n", gain_r);
	CDBG("WB calibration data: gain_gr = %d\n", gain_gr);
	CDBG("WB calibration data: gain_b = %d\n", gain_b);
	CDBG("WB calibration data: gain_gb = %d\n", gain_gb);

	/* reading the black level*/
	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x01);
	if (rc < 0)
		return rc;
#if 0
	rc = t8ev4_i2c_write_b_sensor(0x3402, 0x0E);
#else
	rc = t8ev4_i2c_write_b_sensor(0x3402, 0x0A);
#endif
	if (rc < 0)
		return rc;
	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x81);
	if (rc < 0)
		return rc;
	usleep(30);

#if 1
	addr = 0x3404;
	rc = t8ev4_i2c_read(addr, &act_flg,1);
	if (rc < 0) {
		pr_err("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	if (act_flg != 0x55) {
		pr_err("%s: Error Reading OTP\n", __func__);
		t8ev4_i2c_write_b_sensor(0x3400, 0x01);
		return -EFAULT;
	}
#endif

	addr = 0x3406;
	rc = t8ev4_i2c_read(addr, &blevel_lsb,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}

	addr = 0x3407;
	rc = t8ev4_i2c_read(addr, &r_blevel,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	r_blevel = (r_blevel << 2) | ((blevel_lsb & 0xC0) >> 6);
	cfg->cfg.wb_calib_info.r_blevel = r_blevel;

	addr = 0x3408;
	rc = t8ev4_i2c_read(addr, &gr_blevel,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	gr_blevel = (gr_blevel << 2) | ((blevel_lsb & 0x30) >> 4);
	cfg->cfg.wb_calib_info.gr_blevel = gr_blevel;

	addr = 0x3409;
	rc = t8ev4_i2c_read(addr, &b_blevel,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	b_blevel = (b_blevel << 2) | ((blevel_lsb & 0x0C) >> 2);
	cfg->cfg.wb_calib_info.b_blevel = b_blevel;

	addr = 0x340A;
	rc = t8ev4_i2c_read(addr, &gb_blevel,1);
	if (rc < 0) {
		CDBG("%s: Error Reading OTP @ 0x%x\n", __func__, addr);
		return rc;
	}
	gb_blevel = (gb_blevel << 2) | (blevel_lsb & 0x03) ;
	cfg->cfg.wb_calib_info.gb_blevel = gb_blevel;

	CDBG("BL calibration data: blevel_lsb = %d\n", blevel_lsb);
	CDBG("BL calibration data: r_blevel = %d\n", r_blevel);
	CDBG("BL calibration data: gr_blevel = %d\n", gr_blevel);
	CDBG("BL calibration data: b_blevel = %d\n", b_blevel);
	CDBG("BL calibration data: gb_blevel = %d\n", gb_blevel);

	rc = t8ev4_i2c_write_b_sensor(0x3400, 0x01);
	if (rc < 0)
		return rc;
	return rc;
}
#if 0
static int t8ev4_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	unsigned short chipidl, chipidh;
	rc = gpio_request(data->sensor_reset, "t8ev4");
	CDBG(" t8ev4_probe_init_sensor \n");
	if (!rc) {
		printk("sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(10);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(10);
	} else {
		printk(KERN_ERR "gpio reset fail");
		goto init_probe_done;
	}
	CDBG("t8ev4_probe_init_sensor is called\n");
	/* 3. Read sensor Model ID: */
	rc = t8ev4_i2c_read(0x0, &chipidh, 1);
	if (rc < 0) {
		//	printk("Model read failed\n");
		//	goto init_probe_fail;
	}
	rc = t8ev4_i2c_read(0x1, &chipidl, 1);
	if (rc < 0) {
		//	CDBG("Model read failed\n");
		//	goto init_probe_fail;
	}
	CDBG(KERN_ERR "t8ev4 model_id = 0x%x  0x%x\n", chipidh, chipidl);


#if 0
	if (chipidh != 0x00 || chipidl != 0x00) {
		rc = -ENODEV;
		CDBG("t8ev4_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}
#endif
	goto init_probe_done;
//init_pobe_fail:
	//printk("t8ev4_probe_init_sensor fails\n");
	//gpio_set_value_cansleep(data->sensor_reset, 0);
	//t8ev4_probe_init_done(data);
	init_probe_done:
	CDBG(" t8ev4_probe_init_sensor finishes\n");
	rc = 0;
	return rc;
}
#endif
/* camsensor_iu060f_t8ev4_reset */
#if 0
int t8ev4_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling t8ev4_sensor_open_init\n");
	t8ev4_ctrl = kzalloc(sizeof(struct t8ev4_ctrl_t), GFP_KERNEL);
	if (!t8ev4_ctrl) {
		CDBG("t8ev4_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	t8ev4_ctrl->fps_divider = 1 * 0x00000400;
	t8ev4_ctrl->pict_fps_divider = 1 * 0x00000400;
	t8ev4_ctrl->fps = 30 * Q8;
	t8ev4_ctrl->set_test = TEST_OFF;
	t8ev4_ctrl->prev_res = QTR_SIZE;
	t8ev4_ctrl->pict_res = FULL_SIZE;
	t8ev4_ctrl->curr_res = INVALID_SIZE;


	if (data)
		t8ev4_ctrl->sensordata = data;

	/* enable mclk first */
	msm_camio_clk_rate_set(T8EV4_DEFAULT_MASTER_CLK_RATE);
	msleep(20);

	rc = t8ev4_probe_init_sensor(data);
	if (rc < 0) {
		CDBG("Calling t8ev4_sensor_open_init fail\n");
		goto probe_fail;
	}

	rc = t8ev4_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0) {
		CDBG("t8ev4_sensor_setting failed\n");
		goto init_fail;
	}
	rc = t8ev4_af_init();
	if (rc < 0) {
		CDBG("AF initialisation failed\n");
		goto init_fail;
	} else
		goto init_done;

	probe_fail:
	CDBG("%s probe failed\n", __func__);
	kfree(t8ev4_ctrl);
	return rc;
	init_fail:
	CDBG("t8ev4_sensor_open_init fail\n");
	gpio_set_value_cansleep(data->sensor_reset, 0);
	t8ev4_probe_init_done(data);
	kfree(t8ev4_ctrl);
	init_done:
	CDBG("t8ev4_sensor_open_init done\n");
	return rc;
}
#endif
static int t8ev4_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&t8ev4_wait_queue);
	return 0;
}

static const struct i2c_device_id t8ev4_i2c_id[] = {
	{"t8ev4", 0},
	{}
};

static int t8ev4_i2c_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("t8ev4_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	t8ev4_sensorw = kzalloc(sizeof(struct t8ev4_work_t), GFP_KERNEL);
	if (!t8ev4_sensorw) {
		printk("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, t8ev4_sensorw);
	t8ev4_init_client(client);
	t8ev4_client = client;

	CDBG("t8ev4_probe successed! rc = %d\n", rc);
	return 0;

	probe_failure:
	CDBG("t8ev4_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit t8ev4_remove(struct i2c_client *client)
{
	struct t8ev4_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	t8ev4_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver t8ev4_i2c_driver = {
	.id_table = t8ev4_i2c_id,
	.probe  = t8ev4_i2c_probe,
	.remove = __exit_p(t8ev4_i2c_remove),
			  .driver = {
		.name = "t8ev4",
	},
};

int t8ev4_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
					   (void *)argp,
					   sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&t8ev4_mut);
	CDBG("t8ev4_sensor_config: cfgtype = %d\n",
		 cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		t8ev4_get_pict_fps(
						   cdata.cfg.gfps.prevfps,
						   &(cdata.cfg.gfps.pictfps));
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf =
		t8ev4_get_prev_lines_pf();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl =
		t8ev4_get_prev_pixels_pl();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf =
		t8ev4_get_pict_lines_pf();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl =
		t8ev4_get_pict_pixels_pl();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
		t8ev4_get_pict_max_exp_lc();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = t8ev4_set_fps(&(cdata.cfg.fps));
		min_fps = (uint16_t)(cdata.cfg.fps.min_fps);
		break;
	case CFG_SET_EXP_GAIN:
		rc =
		t8ev4_write_exp_gain(
							 cdata.cfg.exp_gain.gain,
							 cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_PICT_EXP_GAIN:
		rc =
		t8ev4_set_pict_exp_gain(
								cdata.cfg.exp_gain.gain,
								cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_MODE:
		rc = t8ev4_set_sensor_mode(cdata.mode,
									cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = t8ev4_power_down();
		break;
	case CFG_GET_CALIB_DATA:
		rc = t8ev4_read_calib(&cdata);
		if (rc < 0)
			break;
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(cdata)))
			rc = -EFAULT;
		break;
	case CFG_GET_AF_MAX_STEPS:
#if 0
		cdata.max_steps = T8EV4_TOTAL_STEPS_NEAR_TO_FAR;
#else
		cdata.max_steps = t8ev4_af_total_steps_near_to_far;
#endif
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_MOVE_FOCUS:
		rc =
		t8ev4_move_focus(
						 cdata.cfg.focus.dir,
						 cdata.cfg.focus.steps);
		break;
	case CFG_SET_DEFAULT_FOCUS:
		rc =
		t8ev4_set_default_focus();
		break;
#if 1
	case CFG_GET_AF_LENS_POSITION:
		cdata.cfg.lens_position = t8ev4_ctrl->curr_lens_pos;
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_AF_DEFAULT_CODE:
		rc =
		t8ev4_set_af_default_code(&(cdata.cfg.focus_default_info));
		break;
	case CFG_SET_DEFAULT_FOCUS_MODE:
		rc =
		t8ev4_set_default_focus_mode(cdata.cfg.focus_mode);
		break;
	case CFG_SET_AF_TABLE:
		rc = t8ev4_init_af_table(&cdata);
		break;
#endif
	case CFG_SET_EFFECT:
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&t8ev4_mut);

	return rc;
}
#if 0
static int t8ev4_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&t8ev4_mut);
	t8ev4_power_down();
	t8ev4_af_power_down ();
	gpio_set_value_cansleep(t8ev4_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
	gpio_direction_input(t8ev4_ctrl->sensordata->sensor_reset);
	gpio_free(t8ev4_ctrl->sensordata->sensor_reset);
	kfree(t8ev4_ctrl);
	t8ev4_ctrl = NULL;
	CDBG("t8ev4_release completed\n");
	mutex_unlock(&t8ev4_mut);

	return rc;
}
#endif

#if 0
static int t8ev4_sensor_probe(const struct msm_camera_sensor_info *info,
							   struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&t8ev4_i2c_driver);
	if (rc < 0 || t8ev4_client == NULL) {
		printk(KERN_ERR "*****  failed with not supported error\n");
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	msm_camio_clk_rate_set(T8EV4_DEFAULT_MASTER_CLK_RATE);
	rc = t8ev4_probe_init_sensor(info);
	if (rc < 0) {
		printk(KERN_ERR "probe init sensor faileed \n");
		goto probe_fail;
	}
	s->s_init = t8ev4_sensor_open_init;
	s->s_release = t8ev4_sensor_release;
	s->s_config  = t8ev4_sensor_config;
	s->s_mount_angle = 0;
	t8ev4_probe_init_done(info);

	return rc;

	probe_fail:
	CDBG("t8ev4_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&t8ev4_i2c_driver);
	return rc;
}
#endif

static int __t8ev4_probe(struct platform_device *pdev)
{
#if 0
	return msm_camera_drv_start(pdev, t8ev4_sensor_probe);
#else
	return msm_camera_drv_start(pdev, t8ev4_sensor_probe2);
#endif
}

static struct platform_driver msm_camera_driver = {
	.probe = __t8ev4_probe,
	.driver = {
		.name = "msm_camera_t8ev4",
		.owner = THIS_MODULE,
	},
};

static int __init t8ev4_init(void)
{
	CDBG("module init\n");
	return platform_driver_register(&msm_camera_driver);
}

module_init(t8ev4_init);

MODULE_DESCRIPTION("Toshiba 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static int t8ev4_set_af_codestep(void *data, u64 val)
{
	t8ev4_l_region_code_per_step = val;
	t8ev4_af_init();
	return 0;
}

static int t8ev4_get_af_codestep(void *data, u64 *val)
{
	*val = t8ev4_l_region_code_per_step;
	return 0;
}

static uint16_t t8ev4_linear_total_step = T8EV4_TOTAL_STEPS_NEAR_TO_FAR;
static int t8ev4_set_linear_total_step(void *data, u64 val)
{
	t8ev4_linear_total_step = val;
	return 0;
}

static int t8ev4_af_linearity_test(void *data, u64 *val)
{
	int i = 0;

	t8ev4_set_default_focus();
	msleep(3000);
	for (i = 0; i < t8ev4_linear_total_step; i++) {
		t8ev4_move_focus(MOVE_NEAR, 1);
		CDBG("moved to index =[%d]\n", i);
		msleep(1000);
	}

	for (i = 0; i < t8ev4_linear_total_step; i++) {
		t8ev4_move_focus(MOVE_FAR, 1);
		CDBG("moved to index =[%d]\n", i);
		msleep(1000);
	}
	return 0;
}

static uint16_t t8ev4_step_val = T8EV4_TOTAL_STEPS_NEAR_TO_FAR;
static uint8_t t8ev4_step_dir = MOVE_NEAR;
static int t8ev4_af_step_config(void *data, u64 val)
{
	t8ev4_step_val = val & 0xFFFF;
	t8ev4_step_dir = (val >> 16) & 0x1;
	return 0;
}

static int t8ev4_af_step(void *data, u64 *val)
{
	int i = 0;
	int dir = MOVE_NEAR;
	t8ev4_set_default_focus();
	if (t8ev4_step_dir == 1)
		dir = MOVE_FAR;

	for (i = 0; i < t8ev4_step_val; i+=4) {
		t8ev4_move_focus(dir, 4);
		msleep(1000);
	}
	t8ev4_set_default_focus();
	return 0;
}

static uint16_t t8ev4_singlestep_time_step_res = AF_STEP_OPER_NORMAL;
static int t8ev4_af_set_resolution(void *data, u64 val)
{
	t8ev4_singlestep_time_step_res = val & 0xFFFF;
	 t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
		t8ev4_singlestep_time_step_res);

	return 0;
}

static int t8ev4_af_get_resolution(void *data, u64 *val)
{
	*val = t8ev4_singlestep_time_step_res;
	return 0;
}



DEFINE_SIMPLE_ATTRIBUTE(af_codeperstep, t8ev4_get_af_codestep,
			t8ev4_set_af_codestep, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_linear, t8ev4_af_linearity_test,
			t8ev4_set_linear_total_step, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_step, t8ev4_af_step,
			t8ev4_af_step_config, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_step_res, t8ev4_af_get_resolution,
			t8ev4_af_set_resolution, "%llu\n");
#if 0
static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("t8ev4", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("af_codeperstep", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &af_codeperstep))
		return -ENOMEM;
	if (!debugfs_create_file("af_linear", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &af_linear))
		return -ENOMEM;
	if (!debugfs_create_file("af_step", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &af_step))
		return -ENOMEM;

	if (!debugfs_create_file("af_step_res", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &af_step_res))
		return -ENOMEM;


	return 0;
}
#endif
static int t8ev4_check_camcon(void)
{
    int ret = 0;
    int32_t rc;
    unsigned short chipid = 0;

    rc = t8ev4_i2c_read(0x0000, &chipid, sizeof(short));
    if(0 == rc) {
        chipid &= 0xFFF0;
        if(chipid == 0x1400) {
            ret = 1;
        }
    }
    else {
        printk(KERN_ERR "t8ev4 chipid read error!!\n");
    }

    return ret;
}

static int t8ev4_reg_read(unsigned short waddr, unsigned short* wData)
{
    int ret = 0;
    int32_t rc;
    rc = t8ev4_i2c_read(waddr, wData, sizeof(char));
    if(0 == rc) {
        ret = 1;
    }
    else {
        printk(KERN_ERR "t8ev4 reg read error!!\n");
    }

    return ret;
}

static int t8ev4_reg_write(unsigned short waddr, unsigned short wdata)
{
    int ret = 0;
    int32_t rc;

    rc = t8ev4_i2c_write_b_sensor(waddr, (uint8_t)wdata);
    if(0 == rc) {
        ret = 1;
    }
    else {
        printk(KERN_ERR "[%d] t8ev4_i2c_write_b_sensor err!!\n", __LINE__);
    }

    return ret;
}

static int t8ev4_i2c_rxdata_2(unsigned short saddr,
                             unsigned char* wdata,
                             unsigned char* rdata)
{
    struct i2c_msg msgs[] = {
        {
            .addr  = saddr,
            .flags = 0,
            .len   = 2,
            .buf   = wdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len   = 1,
            .buf   = rdata,
        },
    };
    if (i2c_transfer(t8ev4_client->adapter, msgs, 2) < 0) {
        printk(KERN_ERR "t8ev4_i2c_rxdata_2 failed!\n");
        return -EIO;
    }

    return 0;
}

static int t8ev4_i2c_read_buf(unsigned short raddr,
                               unsigned char *rdata)
{
    int rc  = 0;
    int ret = 0;
    unsigned char buf[2] = {0, 0};

    if(NULL != rdata) {
        buf[0] = (raddr & 0xFF00) >> 8;
        buf[1] = (raddr & 0x00FF);
        rc = t8ev4_i2c_rxdata_2(t8ev4_client->addr, buf, rdata);
        if(0 == rc) {
            ret = 1;
        }
        else {
            printk(KERN_ERR "t8ev4_i2c_rxdata_2 0x%x failed!\n", raddr);
        }
    }

    return ret;
}

static int t8ev4_otpContStaWrite(unsigned char bdata)
{
    int rc;
    int ret = 0;

    rc = t8ev4_i2c_write_b_sensor(0x3400, bdata);
    if(0 == rc) {
        ret = 1;
    }
    else {
        printk(KERN_ERR "[%d] t8ev4_i2c_write_b_sensor err!!\n", __LINE__);
    }

    return ret;
}

static int t8ev4_otpPselWrite(unsigned char bpage)
{
    int rc;
    int ret = 0;

    rc = t8ev4_i2c_write_b_sensor(0x3402, bpage);
    if(0 == rc) {
        ret = 1;
    }
    else {
        printk(KERN_ERR "[%d] t8ev4_i2c_write_b_sensor err!!\n", __LINE__);
    }

    return ret;
}

static int t8ev4_get_otpdata(unsigned char page,
                        unsigned char offset,
                        short len,
                        unsigned char* buf)
{
    int ret = 1;
    short count = 0;
    short i = 0;
    short temp = (len / 64);
    unsigned short address = 0x3404 + offset;

    if(64 < len) {
        len = 64;
    }

    ret = t8ev4_otpContStaWrite(0x01);
    if(ret) {
        do {
            address = 0x3404 + offset;

            ret = t8ev4_otpPselWrite(page++);
            if(!ret) {
                printk(KERN_ERR "[%d] t8ev4_otpPselWrite err!!\n", __LINE__);
                temp = 0;
                break;
            }

            ret = t8ev4_otpContStaWrite(0x81);
            if(!ret) {
                printk(KERN_ERR "[%d] t8ev4_otpContStaWrite err!!\n",
                    __LINE__);
                temp = 0;
                break;
            }

            udelay(30);

            for(count = 0; count < len; count++) {
                ret = t8ev4_i2c_read_buf(address++, &buf[((i * 64) + count)]);
                if(!ret) {
                    printk(KERN_ERR "[%d] t8ev4_i2c_read_buf err!!\n",
                        __LINE__);
                    temp = 0;
                    break;
                }
            }
            i++;
            if(0 < temp) {
                temp--;
            }
        } while(0 != temp);

    }

    t8ev4_otpContStaWrite(0x00);

    return ret;
}



#define T8EV4_LOCAL_DELAY               100
#define T8EV4_VREG_DELAY                400

#define MOTOR_DRIVER_MODE_INIT          0xC4
#define MOTOR_DRIVER_STEP_VALUE         0x0428

static struct vreg *vreg_gp2;
static struct vreg *vreg_gp9;
static struct vreg *vreg_gp15;
static struct vreg *vreg_lvsw1;

static int t8ev4_powerOn(struct platform_device *pdev)
{
    int rc = 0;
    int ret = 0;
    struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
    struct msm_camera_device_platform_data *camdev = sdata->pdata;
    int resrtGpioNo = sdata->sensor_reset;

    /*----- Start Sequence -----*/
    if(!strcmp(pdev->name, "msm_camera_t8ev4")) {
        msm_camio_set_io(camdev);

        /*----- 3.0v -----*/
        vreg_gp2 = vreg_get(NULL, "gp2");
        if (IS_ERR(vreg_gp2)) {
            pr_err("%s: VREG GP2 get failed %ld\n", __func__,
                PTR_ERR(vreg_gp2));
            vreg_gp2 = NULL;
            return ret;
        }
        if (vreg_set_level(vreg_gp2, 3000)) {
            pr_err("%s: VREG GP2 set failed\n", __func__);
            goto gp2_put;
        }
        if (vreg_enable(vreg_gp2)) {
            pr_err("%s: VREG GP2 enable failed\n", __func__);
            goto gp2_put;
        }
        udelay(T8EV4_VREG_DELAY);

        /*----- 1.8v -----*/
        vreg_lvsw1 = vreg_get(NULL, "lvsw1");
        if (IS_ERR(vreg_lvsw1)) {
            pr_err("%s: VREG LVSW1 get failed %ld\n", __func__,
            PTR_ERR(vreg_lvsw1));
            vreg_lvsw1 = NULL;
            goto gp2_disable;
        }
        if (vreg_set_level(vreg_lvsw1, 1800)) {
            pr_err("%s: VREG LVSW1 set failed\n", __func__);
            goto lvsw1_put;
        }
        if (vreg_enable(vreg_lvsw1)) {
            pr_err("%s: VREG LVSW1 enable failed\n", __func__);
            goto lvsw1_put;
        }
        udelay(T8EV4_VREG_DELAY);

        /*----- 2.8v -----*/
        vreg_gp9 = vreg_get(NULL, "gp9");
        if (IS_ERR(vreg_gp9)) {
            pr_err("%s: VREG GP9 get failed %ld\n", __func__,
                PTR_ERR(vreg_gp9));
            vreg_gp9 = NULL;
            goto lvsw1_disable;
        }
        if (vreg_set_level(vreg_gp9, 2800)) {
            pr_err("%s: VREG GP9 set failed\n", __func__);
            goto gp9_put;
        }
        if (vreg_enable(vreg_gp9)) {
            pr_err("%s: VREG GP9 enable failed\n", __func__);
            goto gp9_put;
        }
        udelay(T8EV4_VREG_DELAY);

        /*----- Wait for stable of sensor Power(1.8/2.8/3.0)(T.B.D) -----*/
        udelay(10);

        /*----- Wait for stable H/W 2usec -----*/
        udelay(2);

        /*----- 1.225v -----*/
        vreg_gp15 = vreg_get(NULL, "gp15");
        if (IS_ERR(vreg_gp15)) {
            pr_err("%s: VREG GP15 get failed %ld\n", __func__,
                PTR_ERR(vreg_gp15));
            vreg_gp15 = NULL;
            goto gp9_disable;
//            goto sensor_reset_low;
        }
        if (vreg_set_level(vreg_gp15, 1230)) {
            pr_err("%s: VREG GP15 set failed\n", __func__);
            goto gp15_put;
        }
        if (vreg_enable(vreg_gp15)) {
            pr_err("%s: VREG GP15 enable failed\n", __func__);
            goto gp15_put;
        }
        udelay(T8EV4_VREG_DELAY);

        /*----- Wait for stable of sensor Power(1.255)(T.B.D) -----*/
        udelay(10);

        /*----- Set sensor XSHUTDOWN to High -----*/
        gpio_set_value(resrtGpioNo, 1);
        udelay(T8EV4_LOCAL_DELAY);

        /*----- start EXTCLK(9.6MHz) input -----*/
        camdev->camera_gpio_on();
        udelay(T8EV4_LOCAL_DELAY);
        rc = msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
        udelay(T8EV4_LOCAL_DELAY);
        msm_camio_clk_rate_set(T8EV4_DEFAULT_MASTER_CLK_RATE);

        ret = 1;
    }

    return ret;

gp15_put:
    vreg_put(vreg_gp15);
    vreg_gp15 = NULL;
//sensor_reset_low:
//    gpio_set_value(resrtGpioNo, 0);
//    udelay(T8EV4_LOCAL_DELAY);
gp9_disable:
    vreg_disable(vreg_gp9);
    udelay(T8EV4_VREG_DELAY);
gp9_put:
    vreg_put(vreg_gp9);
    vreg_gp9 = NULL;
lvsw1_disable:
    vreg_disable(vreg_lvsw1);
    udelay(T8EV4_VREG_DELAY);
lvsw1_put:
    vreg_put(vreg_lvsw1);
    vreg_lvsw1 = NULL;
gp2_disable:
    vreg_disable(vreg_gp2);
    udelay(T8EV4_VREG_DELAY);
gp2_put:
    vreg_put(vreg_gp2);
    vreg_gp2 = NULL;

    return ret;
}

static int t8ev4_powerOff(struct platform_device *pdev)
{
    int rc = 0;
    int ret = 0;
    struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
    struct msm_camera_device_platform_data *camdev = sdata->pdata;
    int resrtGpioNo = sdata->sensor_reset;

    /*----- Stop Sequence More -----*/
    if(!strcmp(pdev->name, "msm_camera_t8ev4")) {
        /*----- stop EXTCLK(9.6MHz) input -----*/
        rc = msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
        udelay(T8EV4_LOCAL_DELAY);
        camdev->camera_gpio_off();

        /*----- Wait for stable of EXTCLK(T.B.D) -----*/
        udelay(10);

        /*----- v1.225 -----*/
        if (vreg_gp15) {
            vreg_disable(vreg_gp15);
            vreg_put(vreg_gp15);
            vreg_gp15 = NULL;
            udelay(T8EV4_VREG_DELAY);
        }

        /*----- Wait for stable of sensor Power(1.255) 1msec -----*/
        mdelay(1);

        /*----- Set sensor XSHUTDOWN to Low -----*/
        gpio_set_value(resrtGpioNo, 0);
        udelay(T8EV4_LOCAL_DELAY);

        /*----- Wait for sensor H/W stand-by 100us -----*/
        udelay(100);

        /*----- v2.8 -----*/
        if (vreg_gp9) {
            vreg_disable(vreg_gp9);
            vreg_put(vreg_gp9);
            vreg_gp9 = NULL;
            udelay(T8EV4_VREG_DELAY);
        }

        /*----- v1.8 -----*/
        if (vreg_lvsw1) {
            vreg_disable(vreg_lvsw1);
            vreg_put(vreg_lvsw1);
            vreg_lvsw1 = NULL;
            udelay(T8EV4_VREG_DELAY);
        }

        /*----- v3.0 -----*/
        if (vreg_gp2) {
            vreg_disable(vreg_gp2);
            vreg_put(vreg_gp2);
            vreg_gp2 = NULL;
            udelay(T8EV4_VREG_DELAY);
        }

        /*----- Wait for stable of sensor Power(1.8/2.8/3.0)(T.B.D) -----*/
        udelay(10);

        ret = 1;
    }

    return ret;
}

int t8ev4_power_ctrl(struct platform_device *pdev, int8_t power)
{
    int ret = 0;

    if(1 == power) {
        ret = t8ev4_powerOn(pdev);
    }
    else {
        ret = t8ev4_powerOff(pdev);
    }

    return ret;
}

static int t8ev4_Motor_Driver_Setup(void)
{
    int32_t rc;

    rc = t8ev4_i2c_write_w_af(MOTOR_DRIVER_MODE_INIT, MOTOR_DRIVER_STEP_VALUE);

    return rc;
}

static int t8ev4_sensor_open_init2(const struct msm_camera_sensor_info *data)
{
    int32_t rc = 0;
    int vcmGpioNo = data->vcm_enable;

    CDBG("%s: %d\n", __func__, __LINE__);
    CDBG("Calling t8ev4_sensor_open_init2\n");

    t8ev4_ctrl = kzalloc(sizeof(struct t8ev4_ctrl_t), GFP_KERNEL);
    if (!t8ev4_ctrl) {
        printk(KERN_ERR "t8ev4_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }
    t8ev4_ctrl->fps_divider = 1 * 0x00000400;
    t8ev4_ctrl->pict_fps_divider = 1 * 0x00000400;
    t8ev4_ctrl->fps = 30 * Q8;
    t8ev4_ctrl->set_test = TEST_OFF;
    t8ev4_ctrl->prev_res = QTR_SIZE;
    t8ev4_ctrl->pict_res = FULL_SIZE;
    t8ev4_ctrl->curr_res = INVALID_SIZE;

    if(data) {
        t8ev4_ctrl->sensordata = data;
    }

    /*----- Start Sequence More -----*/
    /*----- Set MotorDriver PS to High -----*/
    gpio_set_value(vcmGpioNo, 1);
    udelay(T8EV4_LOCAL_DELAY);

    /*----- Wait for stable of EXTCLK(T.B.D) -----*/
    udelay(10);

    /*----- Wait for sensor H/W 500us -----*/
    udelay(500);

    /*----- Set Motor Driver parameter (I2C) -----*/
    rc = t8ev4_Motor_Driver_Setup();
    if (rc < 0) {
        printk(KERN_ERR "Motor Driver Setup failed\n");
        goto init_fail;
    }

    /*----- Voice Coil Motor(VCM) Setup (I2C) -----*/
    rc = t8ev4_af_init();
    if (rc < 0) {
        printk(KERN_ERR "AF initialisation failed\n");
        goto init_fail;
    }

    /*----- Set sensor parameter (I2C) -----*/
    rc = t8ev4_sensor_setting(REG_INIT, RES_PREVIEW);
    if (rc < 0) {
        printk(KERN_ERR "t8ev4_sensor_setting failed\n");
        goto init_fail;
    }
    else {
        goto init_done;
    }

//probe_fail:
//    printk(KERN_ERR "%s probe failed\n", __func__);
//    kfree(t8ev4_ctrl);
//    return rc;

init_fail:
    printk(KERN_ERR "t8ev4_sensor_open_init2 fail\n");
    gpio_set_value(vcmGpioNo, 0);
    udelay(T8EV4_LOCAL_DELAY);
    kfree(t8ev4_ctrl);

init_done:
    CDBG("t8ev4_sensor_open_init2 done\n");
    return rc;
}

static int t8ev4_sensor_release2(void)
{
    int rc = -EBADF;

    mutex_lock(&t8ev4_mut);

    /*----- Stop Sequence -----*/
    /*----- Set MODE_SELECT to 0 (I2C) -----*/
    t8ev4_power_down();

    /*----- Wait for completion of Sensor output -----*/
    udelay(500);

    t8ev4_af_power_down ();
    udelay(2); /* >1.3us */

    /*----- Set MotorDriver PS to Low -----*/
    gpio_set_value(t8ev4_ctrl->sensordata->vcm_enable, 0);
    udelay(T8EV4_LOCAL_DELAY);

    kfree(t8ev4_ctrl);
    t8ev4_ctrl = NULL;

    kfree(t8ev4_step_position_table);
    t8ev4_step_position_table = NULL;

    CDBG("t8ev4_release2 completed\n");

    mutex_unlock(&t8ev4_mut);

    return rc;
}

static int t8ev4_set_af_table(unsigned short start,
                unsigned short end, unsigned short step)
{
    int ret = 1;
    uint16_t w_af_total_steps;
    uint32_t table_size;
    uint8_t i;

    t8ev4_l_region_code_per_step = (uint16_t)((end - start) / (step - 1));

    w_af_total_steps = t8ev4_af_total_steps_near_to_far;
    t8ev4_af_total_steps_near_to_far = step;
    table_size = (sizeof(uint16_t) * (t8ev4_af_total_steps_near_to_far + 1));
    if (t8ev4_step_position_table != NULL) {
        kfree(t8ev4_step_position_table);
        t8ev4_step_position_table = NULL;
    }
    t8ev4_step_position_table = kmalloc(table_size, GFP_ATOMIC);
    if (!t8ev4_step_position_table) {
        printk(KERN_ERR "[%s]: \
                t8ev4_step_position_table cannot allocate buffer\n",
                            __func__);
        t8ev4_af_total_steps_near_to_far = w_af_total_steps;
        ret = -ENOMEM;
    }
    else {
        memset(&t8ev4_step_position_table[0], 0xFF, table_size);
        t8ev4_step_position_table[0] = t8ev4_af_initial_code;
        t8ev4_step_position_table[1] = start;
        for (i = 2; i <= t8ev4_af_total_steps_near_to_far; i++) {
                t8ev4_step_position_table[i] =
                t8ev4_step_position_table[i-1] +
                t8ev4_l_region_code_per_step;
            if (t8ev4_step_position_table[i] > end)
                t8ev4_step_position_table[i] = end;
        }
    }
    return ret;
}

static int t8ev4_reset_af_table(void)
{
    int ret = 1;
    uint32_t table_size = sizeof(t8ev4_af_default_table);

    if (t8ev4_step_position_table != NULL) {
        kfree(t8ev4_step_position_table);
        t8ev4_step_position_table = NULL;
    }
    t8ev4_step_position_table = kmalloc(table_size, GFP_ATOMIC);
    if (!t8ev4_step_position_table) {
        printk(KERN_ERR "[%s]: \
                t8ev4_step_position_table cannot allocate buffer\n",
                            __func__);
        ret = -ENOMEM;
    }
    else {
        t8ev4_af_total_steps_near_to_far =
                    t8ev4_af_default_total_steps_near_to_far;
        memset(&t8ev4_step_position_table[0], 0xFF, table_size);
        memcpy(&t8ev4_step_position_table[0],
                    t8ev4_af_default_table, table_size);
    }
    return ret;
}

static int32_t t8ev4_init_af_table(struct sensor_cfg_data *cdata)
{
#if 0
    uint16_t i;
#endif
    uint16_t full_index;
    uint32_t rc = 0;
    uint32_t table_size;
    uint16_t step  = 0;
    uint16_t start = 0;
    uint16_t stop  = 0;

    CDBG("%s E\n", __func__);

    t8ev4_af_default_total_steps_near_to_far =
        cdata->cfg.focus_table_info.full_step;
    t8ev4_af_initial_code = cdata->cfg.focus_table_info.full_start;
    table_size =
        (sizeof(uint16_t) * (t8ev4_af_default_total_steps_near_to_far + 1));
    CDBG("%s table_size =%d t8ev4_af_default_table=%d\n",
            __func__, table_size, sizeof(t8ev4_af_default_table));

    if (t8ev4_af_default_table != NULL) {
        kfree(t8ev4_af_default_table);
        t8ev4_af_default_table = NULL;
    }
    t8ev4_af_default_table = kmalloc(table_size, GFP_ATOMIC);
    if (!t8ev4_af_default_table) {
        printk(KERN_ERR "[%s]: \
                t8ev4_step_position_table cannot allocate buffer\n",
                __func__);
        rc = -ENOMEM;
    }
    else {
        memset(&t8ev4_af_default_table[0], 0x00,
                sizeof(t8ev4_af_default_table));
        if (t8ev4_step_position_table != NULL) {
            kfree(t8ev4_step_position_table);
            t8ev4_step_position_table = NULL;
        }
        t8ev4_step_position_table = kmalloc(table_size, GFP_ATOMIC);
        if (!t8ev4_step_position_table) {
            printk(KERN_ERR "[%s]: \
                    t8ev4_step_position_table cannot allocate buffer\n",
                    __func__);
            rc = -ENOMEM;
        }
        else {
            memset(&t8ev4_step_position_table[0], 0x00, table_size);

            t8ev4_af_total_steps_near_to_far =
                        t8ev4_af_default_total_steps_near_to_far;
            step  = t8ev4_af_total_steps_near_to_far;
            start = cdata->cfg.focus_table_info.full_start;
            stop  = cdata->cfg.focus_table_info.full_stop;
            CDBG("%s step =%d start=%d stop=%d\n",
                    __func__, step, start, stop);

            for ( full_index = 0; full_index <= step; full_index++) {
                t8ev4_step_position_table[full_index] =
                            start + full_index * ( stop - start ) / step;
            }

            memcpy(t8ev4_af_default_table, &t8ev4_step_position_table[0],
                    table_size);
#if 0
            for (i = 0; i <= step; i++) {
            CDBG("%s t8ev4_step_position_table[%d] =%d \n",
                    __func__, i, t8ev4_step_position_table[i]);
            }
#endif
        }
    }
    t8ev4_ctrl->curr_lens_pos = 0;
    CDBG("%s X\n", __func__);

    return rc;
}

static int32_t t8ev4_set_af_default_code(struct focus_default_cfg *data)
{
    int32_t rc = 0;

    CDBG("%s E\n", __func__);

    t8ev4_af_normal_default_code = data->normal_default_position;
    t8ev4_af_macro_default_code = data->macro_default_position;
    t8ev4_af_full_default_code = data->full_default_position;
    t8ev4_af_product_default_code = data->product_default_position;

    CDBG("%s default_code:normal=%d,macro=%d,full=%d,product=%d\n",
        __func__, t8ev4_af_normal_default_code,t8ev4_af_macro_default_code,
        t8ev4_af_full_default_code, t8ev4_af_product_default_code);

    return rc;
}

static int32_t t8ev4_set_default_focus_mode(uint8_t focus_mode)
{
    int32_t rc = 0;
    int16_t af_default_code = 0;
    uint8_t codeval_msb, codeval_lsb;

    CDBG("%s ==  enter\n", __func__);
    if (t8ev4_ctrl->curr_step_pos != 0){
        rc = t8ev4_move_focus(MOVE_FAR,
        t8ev4_ctrl->curr_step_pos);
    }
    CDBG("%s defaultfocus step mode(focus_mode = %d)\n", __func__, focus_mode);

    switch (focus_mode) {
    case CAMERA_FOCUS_NORMAL:
        af_default_code = t8ev4_af_normal_default_code;
        break;
    case CAMERA_FOCUS_MACRO:
        af_default_code = t8ev4_af_macro_default_code;
        break;
    case CAMERA_FOCUS_AUTO:
        af_default_code = t8ev4_af_full_default_code;
        break;
    case CAMERA_FOCUS_PRODUCT:
        af_default_code = t8ev4_af_product_default_code;
        break;
    default:
        printk(KERN_ERR "[%s] Invalid focus mode !!\n", __func__);
        return -EINVAL;
    }

    CDBG("%s af_default_code=%d\n", __func__, af_default_code);
    rc = t8ev4_i2c_write_w_af(AF_STS_RS_VALUE_ADDR,
            AF_STEP_OPER_NORMAL);
    if (rc < 0)
        return rc;

    codeval_msb = (AF_STEP_MODE_ADDR |
                ((af_default_code & 0x0300)>>8));
    codeval_lsb = af_default_code & 0x00FF;
    rc = t8ev4_i2c_write_b_af(codeval_msb, codeval_lsb);
    CDBG("%s defaultfocus step mode\n", __func__);
    if (rc < 0) {
        printk(KERN_ERR "t8ev4 I2C Failed line %d\n", __LINE__);
        return rc;
    }
    usleep(300);
    CDBG("%s ==  exit\n", __func__);
    t8ev4_ctrl->curr_lens_pos = af_default_code;
    t8ev4_ctrl->curr_step_pos = 0;

    return rc;
}

static int t8ev4_sensor_probe2(const struct msm_camera_sensor_info *info,
                               struct msm_sensor_ctrl *s)
{
    int rc = 0;

    rc = i2c_add_driver(&t8ev4_i2c_driver);
    if (rc < 0 || t8ev4_client == NULL) {
        printk(KERN_ERR "*****  failed with not supported error\n");
        rc = -ENOTSUPP;
        goto probe_fail;
    }

    s->s_init           = t8ev4_sensor_open_init2;
    s->s_release        = t8ev4_sensor_release2;
    s->s_config         = t8ev4_sensor_config;
    s->s_mount_angle    = 0;
    s->s_camera_type    = BACK_CAMERA_2D;

    s->s_camcon         = t8ev4_check_camcon;
    s->s_reg_read       = t8ev4_reg_read;
    s->s_reg_write      = t8ev4_reg_write;
    s->s_otp_read       = t8ev4_get_otpdata;
    s->s_power_ctrl     = t8ev4_power_ctrl;
    s->s_set_af_table   = t8ev4_set_af_table;
    s->s_reset_af_table = t8ev4_reset_af_table;

    return rc;

probe_fail:
    printk(KERN_ERR "t8ev4_sensor_probe2: SENSOR PROBE FAILS!\n");
    i2c_del_driver(&t8ev4_i2c_driver);

    return rc;
}
