/* drivers/sharp/shirrc/shirrc_kdrv.c (Infrared driver)
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

#include <linux/stat.h>
#include <linux/fcntl.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/wakelock.h>

#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <sharp/shlcdc_kerl.h>

#include "sharp/shirrc_kdrv.h"
#include "sharp/irda_kdrv_ext_api.h"
#include "shirrc_kdrv_log.h"
#include "shirrc_LR388G7.h"

#define	SH_IRRC_KERNEL_DRIVER_NAME	"LR388G7"
#define	SH_IRRC_KERNEL_DRIVER_VERSION	"1.00.00"
#define	IRRC_SUCCESS 		(0)

typedef enum {
	IRRC_KDRV_STATUS_IDLE,
	IRRC_KDRV_STATUS_STANDBY,
	IRRC_KDRV_STATUS_READY,
	IRRC_KDRV_STATUS_SENDING,
	IRRC_KDRV_STATUS_SENDINGFAINAL,
	IRRC_KDRV_STATUS_MAX
} irrc_kdrv_status_enum;

typedef enum {
	IRRC_KDRV_EVENT_OPEN,
	IRRC_KDRV_EVENT_IOCTL_INIT,
	IRRC_KDRV_EVENT_IOCTL_SETPRAM,
	IRRC_KDRV_EVENT_WRITE,
	IRRC_KDRV_EVENT_IOCTL_STOP,
	IRRC_KDRV_EVENT_CLOSE,
	IRRC_KDRV_EVENT_TIMER,
	IRRC_KDRV_EVENT_DATA_SET,
	IRRC_KDRV_EVENT_WRITE_END,
	IRRC_KDRV_EVENT_SHUTDOWN,
	IRRC_KDRV_EVENT_MAX
} irrc_kdrv_event_enum;

typedef enum {
	IRRC_ACT_OK,
	IRRC_ACT_WARNING,
	IRRC_ACT_NG,
	IRRC_ACT_MAX
} irrc_kdrv_action_enum;

static irrc_kdrv_action_enum
	irrc_kdrv_action[IRRC_KDRV_STATUS_MAX][IRRC_KDRV_EVENT_MAX] = {
	{
		IRRC_ACT_OK,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG
	},
	{
		IRRC_ACT_NG,
		IRRC_ACT_OK,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_OK,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_OK
	},
	{
		IRRC_ACT_NG,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_WARNING,
		IRRC_ACT_OK,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_OK
	},
	{
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_NG,
		IRRC_ACT_OK
	},
	{
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_NG,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK,
		IRRC_ACT_OK
	}
};

#define	IRRC_BLOCK_REPEAT_MAX	(int32)255
#define	IRRC_FRAME_REPEAT_MAX	(int32)255
#define	IRRC_HW_HEADER_MAX	(int32)0x3FFFF
#define	IRRC_HW_END_MAX		(int32)0x3FFFF
#define	IRRC_HW_PULSE_MAX	(int32)0x3FFFF
#define	IRRC_HW_CARRIER_MAX	(int32)0x27F
#define	IRRC_HW_FRAME_MAX	(int32)0x989680
#define	IRRC_HW_ETOS_MAX	(int32)0x1FFFFF
#define	IRRC_HW_BITLEN_MAX	(int32)0x1000
#define	IRRC_HW_PULSE_MIN	(int32)0x8
#define	IRRC_HW_CARRIER_MIN	(int32)0x1
#define	IRRC_HW_ETOS_MIN	(int32)0x4
#define	IRRC_HW_HEADER_MIN	(int32)0
#define	IRRC_HW_END_MIN		(int32)0
#define	IRRC_HW_BITLEN_MIN	(int32)0

#define	IRRC_DATASET_BSIZE	16
#define	IRRC_OUT_DATA_BUFSIZE	(IRRC_HW_BITLEN_MAX / IRRC_DATASET_BSIZE)
#define	IRRC_SEND_OVERTIME	50
#define IRRC_OUT_REG_ADDRESS(v)	&(senddata.out_data[(v) * IRRC_OUT_REG_MAX])
#define	IRRC_SD_LOW		0x0000

#define	IRRC_WAKELOCK_TIMEOUT	((HZ * 105) /10)

typedef enum {
	IRRC_KDRV_WU_EV_SEND_TIMEOUT,
	IRRC_KDRV_WU_EV_ISR_SENDCOMP,
	IRRC_KDRV_WU_EV_ISR_DATA_SET,
	IRRC_KDRV_WU_EV_SEND_STOP,
	IRRC_KDRV_WU_EV_CLOSE,
	IRRC_KDRV_WU_EV_ENUM_MAX
} irrc_kdrv_wakeup_event_enum;

typedef	union {
	uint8	in_data[IRRC_USER_DATA_BUFSIZE];
	uint16	out_data[IRRC_OUT_DATA_BUFSIZE];
} irrc_senddata_u;

static	const irrc_drv_capa_info	irrc_capainfo = {
					IRRC_HW_CARRIER_MIN,
					IRRC_HW_CARRIER_MAX,
					IRRC_HW_PULSE_MIN,
					IRRC_HW_PULSE_MAX,
					IRRC_HW_HEADER_MAX,
					IRRC_HW_END_MAX,
					IRRC_HW_FRAME_MAX,
					IRRC_HW_ETOS_MIN,
					IRRC_HW_ETOS_MAX,
					IRRC_HW_BITLEN_MAX,
					IRRC_FRAME_REPEAT_MAX,
					IRRC_FRAME_COUNT_MAX,
					IRRC_BLOCK_REPEAT_MAX
				};
static	irrc_kdrv_status_enum 		irrc_kdrv_state;
static	irrc_send_info			irrc_sendinfo;
static	irrc_kdrv_wakeup_event_enum	irrc_wakeup_cause;
static	wait_queue_head_t 		irrc_kdrv_wu_que;

static	struct wake_lock 		irrc_wake_lock;

static	int irrc_kdrv_ioctl_init(unsigned long arg);
static	int irrc_kdrv_write(const char __user *buf, size_t count);
static	int irrc_kdrv_ioctl_sendparam(unsigned long arg);
static	int irrc_kdrv_ioctl_stopsend(void);
static	void irrc_kdrv_sendinfo_init(void);
static	int irrc_kdrv_check_sendparam(irrc_send_info *sendinfo);
static	void irrc_kdrv_isr_dataset_cb(void);
static	void irrc_kdrv_isr_sendcomp_cb(void);
static	int irrc_kdrv_bitcount(uint16 bitdata);
static	int irrc_kdrv_irrcout_bitcnt(uint16 *data_ptr);
static	uint32 irrc_kdrv_sleeptime(int bitcnt, int outlen_bitcnt,
							uint32 loopcnt);
static	void irrc_kdrv_data_change(
			irrc_senddata_u *senddata, uint32 data_length);

int	irrc_kdrv_fops_open(struct inode *inode, struct file *filp)
{
	int			ret	= IRRC_SUCCESS;
	irrc_kdrv_action_enum	act_val;

	IRRCLOG_INFO("irrc kernel driver open\n");

	act_val = irrc_kdrv_action[irrc_kdrv_state][IRRC_KDRV_EVENT_OPEN];
	if (act_val == IRRC_ACT_OK) {
		IRRCLOG_DEBUG("change state %d->%d\n", irrc_kdrv_state,
						IRRC_KDRV_STATUS_STANDBY);
		irrc_kdrv_state = IRRC_KDRV_STATUS_STANDBY;

		ret = irrc_reg_ioremap_nocache();
		if (ret == IRRC_REG_SUCCESS) {
			wake_lock_init(&irrc_wake_lock,
				WAKE_LOCK_IDLE, IRRC_FOPS_DEVFILE_NAME);
		} else {
			IRRCLOG_ERROR("irrc_reg_ioremap_nocache error\n");
			IRRCLOG_DEBUG("change state %d->%d\n",
				irrc_kdrv_state, IRRC_KDRV_STATUS_IDLE);
			irrc_kdrv_state = IRRC_KDRV_STATUS_IDLE;
			ret = -ENXIO;
		}
	} else {
		IRRCLOG_ERROR("status error (state=%d)\n", irrc_kdrv_state);
		ret = -EPERM;
	}

	IRRCLOG_DEBUG("OUT ret=%d\n", ret);

	return ret;
}

int	irrc_kdrv_fops_close(struct inode *inode, struct file *filp)
{
 	int 			ret	= IRRC_SUCCESS;
	irrc_kdrv_action_enum	act_val;

	IRRCLOG_INFO("irrc kernel driver close\n");
	act_val = irrc_kdrv_action[irrc_kdrv_state][IRRC_KDRV_EVENT_CLOSE];
	if (act_val == IRRC_ACT_OK) {
		if ((irrc_kdrv_state == IRRC_KDRV_STATUS_SENDING) ||
		    (irrc_kdrv_state == IRRC_KDRV_STATUS_SENDINGFAINAL)) {
			IRRCLOG_INFO("stop send\n");

			irrc_wakeup_cause = IRRC_KDRV_WU_EV_CLOSE;

			irrc_reg_sys_reset();

			wake_up(&irrc_kdrv_wu_que);
			IRRCLOG_DEBUG("irrc close wakeup (cause =%d)\n",
							irrc_wakeup_cause);
		}

		irrc_reg_set_sys_pwroff();

		irrc_reg_init();

		irda_kdrv_irsd_shutdown();

		irrc_reg_iounmap();

		irrc_kdrv_sendinfo_init();

		IRRCLOG_DEBUG("change state %d->%d\n",
				irrc_kdrv_state, IRRC_KDRV_STATUS_IDLE);
		irrc_kdrv_state = IRRC_KDRV_STATUS_IDLE;

		wake_lock_destroy(&irrc_wake_lock);

	} else {
		IRRCLOG_ERROR("status error (state=%d)\n", irrc_kdrv_state);
		ret = -EPERM;
	}

	IRRCLOG_DEBUG("OUT\n");

	return ret;
}

ssize_t	irrc_kdrv_fops_write(
	struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int 			ret 	= IRRC_SUCCESS;
	irrc_kdrv_action_enum	act_val;

	IRRCLOG_INFO("irrc kernel driver write\n");

	if ((count < IRRC_HW_BITLEN_MIN) || (count > IRRC_HW_BITLEN_MAX)) {
		IRRCLOG_ERROR("invalid param : count (%d)\n", count);
		return -EFAULT;
	}

	act_val = irrc_kdrv_action[irrc_kdrv_state][IRRC_KDRV_EVENT_WRITE];
	if (act_val == IRRC_ACT_OK) {

		irrc_reg_set_sys_pwroff();
		irrc_reg_set_carrier(irrc_sendinfo.carrier_high,
						irrc_sendinfo.carrier_low);
		irrc_reg_set_sys_pwron();

		irrc_reg_set_modulation(irrc_sendinfo.modulation0,
						irrc_sendinfo.modulation1);

		irrc_reg_set_data0(irrc_sendinfo.pulse0_low,
						irrc_sendinfo.pulse0_high);

		irrc_reg_set_data1(irrc_sendinfo.pulse1_low,
						irrc_sendinfo.pulse1_high);

		irrc_reg_set_leader(irrc_sendinfo.leader_low,
						irrc_sendinfo.leader_high);

		irrc_reg_set_trailer(irrc_sendinfo.trailer_high);

		irrc_reg_set_frame_length(irrc_sendinfo.end_to_start);

		wake_lock_timeout(&irrc_wake_lock, IRRC_WAKELOCK_TIMEOUT);

		ret = irrc_kdrv_write(buf, count);

		wake_unlock(&irrc_wake_lock);

	} else {
		IRRCLOG_ERROR("status error (state=%d)\n", irrc_kdrv_state);
		ret = -EPERM;
	}

	IRRCLOG_DEBUG("OUT\n");

	return ret;
}

int	irrc_kdrv_fops_ioctl(struct inode *inode, struct file *filp,
					unsigned int cmd, unsigned long arg)
{
	int	ret	= IRRC_SUCCESS;

	switch (cmd) {
	case IRRC_DRV_IOCTL_INIT:
		ret = irrc_kdrv_ioctl_init(arg);
		break;

	case IRRC_DRV_IOCTL_SET_SENDPARAM:
		ret = irrc_kdrv_ioctl_sendparam(arg);
		break;

	case IRRC_DRV_IOCTL_STOP_SEND:
		ret = irrc_kdrv_ioctl_stopsend();
		break;

	default:
		ret = -EFAULT;
		IRRCLOG_ERROR("irrc ioctl invalid command (%d)\n", cmd);
	}

	return ret;
}

static	int irrc_kdrv_ioctl_init(unsigned long arg)
{
	irrc_kdrv_action_enum	act_val;
	unsigned long		rc;
	int			ret	= IRRC_SUCCESS;
	void __user		*argp	= (void __user*)arg;

	IRRCLOG_INFO("irrc kernel driver ioctl (init)\n");

	act_val =
		irrc_kdrv_action[irrc_kdrv_state][IRRC_KDRV_EVENT_IOCTL_INIT];
	if (act_val == IRRC_ACT_OK) {
		rc = copy_to_user(argp,
				&irrc_capainfo, sizeof(irrc_drv_capa_info));
		if (rc != 0) {
			IRRCLOG_ERROR("copy_to_user() error\n");
			ret = -EFAULT;
		} else {
			ret = irda_kdrv_irsd_active();
			if (ret != IRDA_KDRV_EXT_API_SUCCESS) {
				IRRCLOG_ERROR("irda_kdrv_irsd_active error\n");
				ret = -EFAULT;
			} else {
				irrc_reg_set_irrcdiv();

				irrc_reg_sys_reset();

				irrc_reg_set_sys();
				irrc_reg_set_base();

				irrc_reg_set_dbg();

				irrc_reg_set_sys_pwron();

				IRRCLOG_DEBUG("change state %d->%d\n",
						irrc_kdrv_state,
						IRRC_KDRV_STATUS_READY);
				irrc_kdrv_state = IRRC_KDRV_STATUS_READY;
			}
		}
	} else {
		IRRCLOG_ERROR("status error (state=%d)\n", irrc_kdrv_state);
		ret = -EPERM;
	}

	return ret;
}

static	int irrc_kdrv_ioctl_sendparam(unsigned long arg)
{
	irrc_kdrv_action_enum		act_val;
	unsigned long			rc;
	int				ret 	= IRRC_SUCCESS;
	void __user			*argp	= (void __user*)arg;

	IRRCLOG_INFO("irrc kernel driver ioctl (sendparam)\n");

	act_val =
	irrc_kdrv_action[irrc_kdrv_state][IRRC_KDRV_EVENT_IOCTL_SETPRAM];

	if (act_val == IRRC_ACT_OK) {

		rc = copy_from_user(&irrc_sendinfo, argp,
						sizeof(irrc_send_info));
		if (rc == 0) {
			ret = irrc_kdrv_check_sendparam(&irrc_sendinfo);
			if (ret != IRRC_SUCCESS) {
				IRRCLOG_ERROR("Check send-parameter error\n");
			}
		} else {
			IRRCLOG_ERROR("copy_from_user() error\n");
			ret = -EFAULT;
		}

	} else {
		IRRCLOG_ERROR("status error (state=%d)\n", irrc_kdrv_state);
		ret = -EPERM;
	}

	return ret;
}

static	int irrc_kdrv_ioctl_stopsend(void)
{
	int			ret	= IRRC_SUCCESS;
	irrc_kdrv_action_enum	act_val;

	IRRCLOG_INFO("irrc kernel driver ioctl (stopsend)\n");

	act_val =
		irrc_kdrv_action[irrc_kdrv_state][IRRC_KDRV_EVENT_IOCTL_STOP];
	if (act_val == IRRC_ACT_OK) {

		irrc_wakeup_cause = IRRC_KDRV_WU_EV_SEND_STOP;

		irrc_reg_sys_reset();

		wake_up(&irrc_kdrv_wu_que);
		IRRCLOG_DEBUG("irrc stopsend wakeup (cause=%d)\n",
							irrc_wakeup_cause);
	} else if (act_val == IRRC_ACT_WARNING) {
		IRRCLOG_ERROR("status warning\n");
		ret = -ENOTSUPP;
	} else {
		IRRCLOG_ERROR("status error (state=%d)\n", irrc_kdrv_state);
		ret = -EPERM;
	}

	return ret;
}

static	int irrc_kdrv_write(const char __user *buf, size_t count)
{
	int				ret	= IRRC_SUCCESS;
	int				s_data_len;
	int				bitcnt;
	int				outlen_bitcnt;
	uint16				*out_data_addr;
	uint32				data_idx;
	uint32				w_cnt;
	uint32				in_data_len;
	uint32				mask_len;
	uint32				sleeptime;
	unsigned long			rc;
	struct	shlcdc_subscribe 	sc;
	irrc_senddata_u			senddata;
	irrc_kdrv_action_enum		act_val;

	w_cnt = 0;
	mask_len = 0;
	outlen_bitcnt = 0;
	memset(senddata.in_data, 0x00, sizeof(senddata));
	s_data_len = (int)count;

	if (s_data_len > 0) {
		in_data_len = (s_data_len / IRRC_USER_DATA_BSIZE);
		if ((s_data_len % IRRC_USER_DATA_BSIZE) > 0) {
			in_data_len += 1;
		}

		rc = copy_from_user(senddata.in_data, buf, in_data_len);
		if (rc != 0) {
			IRRCLOG_ERROR("copy_from_user() error\n");
			ret = -EPERM;
			return ret;
		}

		if ((s_data_len % IRRC_USER_DATA_BSIZE) > 0) {
			IRRCLOG_DEBUG("mask before %x\n",
				senddata.in_data[in_data_len - 1]);

			mask_len = ((in_data_len * IRRC_USER_DATA_BSIZE) -
								s_data_len);
			senddata.in_data[in_data_len - 1] <<= mask_len;
			senddata.in_data[in_data_len - 1] >>= mask_len;

			IRRCLOG_DEBUG("mask after %x\n",
				senddata.in_data[in_data_len - 1]);
		}
	}

	IRRCLOG_DEBUG("change state %d->%d\n",
				irrc_kdrv_state, IRRC_KDRV_STATUS_SENDING);
	irrc_kdrv_state = IRRC_KDRV_STATUS_SENDING;

	irrc_kdrv_data_change(&senddata, s_data_len);

	if (s_data_len == 0) {
		w_cnt = 1;
		outlen_bitcnt = 0;
	} else {
		w_cnt = (s_data_len / IRRC_REG_OUTLEN_MAX);
		if ((s_data_len % IRRC_REG_OUTLEN_MAX) > 0) {
			w_cnt += 1;
		}
	}

	for (data_idx=0; ((data_idx < w_cnt) && (ret == IRRC_SUCCESS));
							 data_idx++) {
		init_waitqueue_head(&irrc_kdrv_wu_que);

		irrc_wakeup_cause = IRRC_KDRV_WU_EV_SEND_TIMEOUT;

		if (data_idx == w_cnt - 1) {
			IRRCLOG_DEBUG("change state %d->%d\n",
					irrc_kdrv_state,
					IRRC_KDRV_STATUS_SENDINGFAINAL);
			irrc_kdrv_state =
				IRRC_KDRV_STATUS_SENDINGFAINAL;
			sc.event_type = SHLCDC_EVENT_TYPE_IRRC2;
			sc.callback = irrc_kdrv_isr_sendcomp_cb;
			irrc_reg_clear_int_irrc2_factor();
			shlcdc_api_event_subscribe(&sc);

			irrc_reg_set_regs(IRRC_REGS_NOT_REPEAT);

			if (s_data_len == 0) {
				irrc_reg_set_bitlen(outlen_bitcnt);
			} else {
				outlen_bitcnt =
					s_data_len % IRRC_REG_OUTLEN_MAX;

				if (outlen_bitcnt > 0) {
					irrc_reg_set_bitlen(outlen_bitcnt);
				} else {
					irrc_reg_set_bitlen(
						IRRC_REG_OUTLEN_MAX);
					outlen_bitcnt = IRRC_REG_OUTLEN_MAX;
				}
			}
		} else {
			sc.event_type = SHLCDC_EVENT_TYPE_IRRC;
			sc.callback = irrc_kdrv_isr_dataset_cb;
			irrc_reg_clear_int_irrc_factor();
			shlcdc_api_event_subscribe(&sc);

			irrc_reg_set_regs(IRRC_REGS_REPEAT);

			outlen_bitcnt = IRRC_REG_OUTLEN_MAX;

			irrc_reg_set_bitlen(IRRC_REG_OUTLEN_MAX);
		}
		out_data_addr = IRRC_OUT_REG_ADDRESS(data_idx);

		bitcnt = irrc_kdrv_irrcout_bitcnt(out_data_addr);
		sleeptime = irrc_kdrv_sleeptime(bitcnt, outlen_bitcnt,
								data_idx);

		irrc_reg_set_data(out_data_addr);

		if (data_idx == 0) {
			irrc_reg_send();
		}
		wait_event_timeout(irrc_kdrv_wu_que,
		(irrc_wakeup_cause != IRRC_KDRV_WU_EV_SEND_TIMEOUT),
							 sleeptime);
		IRRCLOG_DEBUG("irrc write wakeup (%d)\n", irrc_wakeup_cause);

		shlcdc_api_event_unsubscribe(SHLCDC_EVENT_TYPE_IRRC);
		shlcdc_api_event_unsubscribe(SHLCDC_EVENT_TYPE_IRRC2);

		switch (irrc_wakeup_cause) {
		case IRRC_KDRV_WU_EV_SEND_TIMEOUT:
			act_val =
				irrc_kdrv_action
				[irrc_kdrv_state][IRRC_KDRV_EVENT_TIMER];
			if (act_val == IRRC_ACT_OK) {
				IRRCLOG_ERROR("send timeout\n");
				IRRCLOG_DEBUG("change state %d->%d\n",
						irrc_kdrv_state,
						IRRC_KDRV_STATUS_READY);
			        irrc_kdrv_state = IRRC_KDRV_STATUS_READY;
				ret = -EIO;
			}
			break;

		case IRRC_KDRV_WU_EV_ISR_SENDCOMP:
			act_val =
				irrc_kdrv_action
				[irrc_kdrv_state][IRRC_KDRV_EVENT_WRITE_END];
			if (act_val != IRRC_ACT_OK) {
				IRRCLOG_ERROR("Unexpected error\n");
				IRRCLOG_DEBUG("change state %d->%d\n",
						irrc_kdrv_state,
						IRRC_KDRV_STATUS_READY);
			        irrc_kdrv_state = IRRC_KDRV_STATUS_READY;
				ret = -EIO;
			}
			break;

		case IRRC_KDRV_WU_EV_ISR_DATA_SET:
			act_val =
				irrc_kdrv_action
				[irrc_kdrv_state][IRRC_KDRV_EVENT_DATA_SET];
			if (act_val != IRRC_ACT_OK) {
				IRRCLOG_ERROR("Unexpected error\n");
				IRRCLOG_DEBUG("change state %d->%d\n",
						irrc_kdrv_state,
						IRRC_KDRV_STATUS_READY);
			        irrc_kdrv_state = IRRC_KDRV_STATUS_READY;
				ret = -EIO;
			}
			break ;

		case IRRC_KDRV_WU_EV_SEND_STOP:
			IRRCLOG_DEBUG("change state %d->%d\n",
						irrc_kdrv_state,
						IRRC_KDRV_STATUS_READY);
		        irrc_kdrv_state = IRRC_KDRV_STATUS_READY;
			ret = -ECANCELED;
			break;

		case IRRC_KDRV_WU_EV_CLOSE:
			ret = -ECANCELED;
			break;

		default:
			IRRCLOG_ERROR("Unexpected error\n");
			ret = -EIO;
			break;
		}
        }
        if (ret == IRRC_SUCCESS) {
		IRRCLOG_DEBUG("change state %d->%d\n",
						irrc_kdrv_state,
						IRRC_KDRV_STATUS_READY);
	        irrc_kdrv_state = IRRC_KDRV_STATUS_READY;
		ret = s_data_len;
	}

	return ret;
}

static	void irrc_kdrv_sendinfo_init(void)
{
	irrc_sendinfo.carrier_high = IRRC_CHI_INIT;
	irrc_sendinfo.carrier_low = IRRC_CLO_INIT;
	irrc_sendinfo.modulation0 = IRRC_PPM_HIGH_LOW;
	irrc_sendinfo.modulation1 = IRRC_PPM_HIGH_LOW;
	irrc_sendinfo.pulse0_high = IRRC_D0HI_INIT;
	irrc_sendinfo.pulse0_low = IRRC_D0LO_INIT;
	irrc_sendinfo.pulse1_high = IRRC_D1HI_INIT;
	irrc_sendinfo.pulse1_low = IRRC_D1LO_INIT;
	irrc_sendinfo.leader_high = IRRC_HHI_INIT;
	irrc_sendinfo.leader_low = IRRC_HLO_INIT;
	irrc_sendinfo.trailer_high = IRRC_ENDLEN_INIT;
	irrc_sendinfo.end_to_start = IRRC_FRMLEN_INIT;

	return;
}

static	int irrc_kdrv_check_sendparam(irrc_send_info *sendinfo)
{
	int	ret;
	int	work;

	ret = -EINVAL;

	if ((sendinfo->carrier_high < IRRC_HW_CARRIER_MIN) ||
	    (sendinfo->carrier_high > IRRC_HW_CARRIER_MAX)) {
		IRRCLOG_ERROR("invalid param : carrier high (%ld)\n",
						sendinfo->carrier_high);
		goto checkparam_err;
	}
	work = sendinfo->carrier_high * IRRC_CARRIER_UNIT_CONV;

	work /= IRRC_SOURCE_CLK;

	work += IRRC_CARRIER_ROUND_NUM;
	work /= IRRC_CARRIER_10_TIMES;
	sendinfo->carrier_high = work;

	if ((sendinfo->carrier_low < IRRC_HW_CARRIER_MIN) ||
	    (sendinfo->carrier_low > IRRC_HW_CARRIER_MAX)) {
		IRRCLOG_ERROR("invalid param : carrier low (%ld)\n",
						sendinfo->carrier_low);
		goto checkparam_err;
	}
	work = sendinfo->carrier_low * IRRC_CARRIER_UNIT_CONV;

	work /= IRRC_SOURCE_CLK;

	work += IRRC_CARRIER_ROUND_NUM;
	work /= IRRC_CARRIER_10_TIMES;
	sendinfo->carrier_low = work;

	if ((sendinfo->modulation0 != IRRC_PPM_HIGH_LOW) &&
	    (sendinfo->modulation0 != IRRC_PPM_LOW_HIGH)) {
		IRRCLOG_ERROR("invalid param : modulation0 (%d)\n",
						sendinfo->modulation0);
		goto checkparam_err;
	}

	if ((sendinfo->modulation1 != IRRC_PPM_HIGH_LOW) &&
	    (sendinfo->modulation1 != IRRC_PPM_LOW_HIGH)) {
		IRRCLOG_ERROR("invalid param : modulation1 (%d)\n",
						sendinfo->modulation1);
		goto checkparam_err;
	}

	if ((sendinfo->pulse0_high < IRRC_HW_PULSE_MIN) ||
	    (sendinfo->pulse0_high > IRRC_HW_PULSE_MAX)) {
		IRRCLOG_ERROR("invalid param : pulse0 high (%ld)\n",
						sendinfo->pulse0_high);
		goto checkparam_err;
	}

	if ((sendinfo->pulse0_low < IRRC_HW_PULSE_MIN) ||
	    (sendinfo->pulse0_low > IRRC_HW_PULSE_MAX)) {
		IRRCLOG_ERROR("invalid param : pulse0 low (%ld)\n",
						sendinfo->pulse0_low);
		goto checkparam_err;
	}

	if ((sendinfo->pulse1_high < IRRC_HW_PULSE_MIN) ||
	    (sendinfo->pulse1_high > IRRC_HW_PULSE_MAX)) {
		IRRCLOG_ERROR("invalid param : pulse1 high (%ld)\n",
						sendinfo->pulse1_high);
		goto checkparam_err;
	}

	if ((sendinfo->pulse1_low < IRRC_HW_PULSE_MIN) ||
	    (sendinfo->pulse1_low > IRRC_HW_PULSE_MAX)) {
		IRRCLOG_ERROR("invalid param : pulse1 low (%ld)\n",
						sendinfo->pulse1_low);
		goto checkparam_err;
	}

	if ((sendinfo->leader_high < IRRC_HW_HEADER_MIN) ||
	    (sendinfo->leader_high > IRRC_HW_HEADER_MAX)) {
		IRRCLOG_ERROR("invalid param : leader high (%ld)\n",
						sendinfo->leader_high);
		goto checkparam_err;
	}

	if ((sendinfo->leader_low < IRRC_HW_HEADER_MIN) ||
	    (sendinfo->leader_low > IRRC_HW_HEADER_MAX)) {
		IRRCLOG_ERROR("invalid param : leader low (%ld)\n",
						sendinfo->leader_low);
		goto checkparam_err;
	}

	if ((sendinfo->trailer_high < IRRC_HW_END_MIN) ||
	    (sendinfo->trailer_high > IRRC_HW_END_MAX)) {
		IRRCLOG_ERROR("invalid param : trailer high (%ld)\n",
						sendinfo->trailer_high);
		goto checkparam_err;
	}

	if ((sendinfo->end_to_start < IRRC_HW_ETOS_MIN) ||
	    (sendinfo->end_to_start > IRRC_HW_ETOS_MAX)) {
		IRRCLOG_ERROR("invalid param : end_to_start (%ld)\n",
						sendinfo->end_to_start);
		goto checkparam_err;
	}

	ret = IRRC_SUCCESS;

checkparam_err:
	return ret;
}

static	void irrc_kdrv_isr_dataset_cb(void)
{
	if (irrc_wakeup_cause == IRRC_KDRV_WU_EV_SEND_TIMEOUT) {
		irrc_wakeup_cause	= IRRC_KDRV_WU_EV_ISR_DATA_SET;
		wake_up(&irrc_kdrv_wu_que);
		IRRCLOG_DEBUG("irrc dataset wakeup (cause=%d)\n",
						irrc_wakeup_cause);
	}

	return;
}

static	void irrc_kdrv_isr_sendcomp_cb(void)
{
	if (irrc_wakeup_cause == IRRC_KDRV_WU_EV_SEND_TIMEOUT) {
		irrc_wakeup_cause	= IRRC_KDRV_WU_EV_ISR_SENDCOMP;
		wake_up(&irrc_kdrv_wu_que);
		IRRCLOG_DEBUG("irrc sendcomp wakeup (cause=%d)\n",
						irrc_wakeup_cause);
	}

	return;
}

static	int irrc_kdrv_bitcount(uint16 bitdata)
{
        bitdata = ((bitdata & 0xAAAA) >> 1) + (bitdata & 0x5555);
        bitdata = ((bitdata & 0xCCCC) >> 2) + (bitdata & 0x3333);
        bitdata = ((bitdata & 0xF0F0) >> 4) + (bitdata & 0x0F0F);
        bitdata = ((bitdata & 0xFF00) >> 8) + (bitdata & 0x00FF);

        return bitdata;
}

static	int irrc_kdrv_irrcout_bitcnt(uint16 *data_ptr)
{
	int	bitcnt	= 0;
	int	loopcount;

	for (loopcount = 0; (loopcount < IRRC_REG_OUTLEN_MAX / 16);
							loopcount++) {
		bitcnt  += irrc_kdrv_bitcount(*(data_ptr + loopcount));
	}

	IRRCLOG_DEBUG("on bit=%d\n", bitcnt);

	return bitcnt;
}

static	uint32 irrc_kdrv_sleeptime(int bitcnt, int outlen_bitcnt,
							uint32 loopcnt)
{
	uint32	sleeptime_us	= 0;
	uint32	sleeptime_ms	= 0;
	uint32	jiffies_unit	= 0;
	uint32	onbitcnt	= (uint32)bitcnt;
	uint32	offbitcnt	= (uint32)outlen_bitcnt - onbitcnt;
	uint32	nowtime_ms	= 0;
static	uint32	beforetime_ms;

	sleeptime_us = (irrc_sendinfo.pulse0_low + irrc_sendinfo.pulse0_high)
								 * offbitcnt;
	sleeptime_us += (irrc_sendinfo.pulse1_low + irrc_sendinfo.pulse1_high)
								 * onbitcnt;
	if (loopcnt == 0) {
		beforetime_ms = 0;
		sleeptime_us += (irrc_sendinfo.leader_high +
						irrc_sendinfo.leader_low);
	}

	if (irrc_kdrv_state == IRRC_KDRV_STATUS_SENDINGFAINAL) {
		sleeptime_us += (irrc_sendinfo.trailer_high +
						irrc_sendinfo.end_to_start);
	}

	sleeptime_ms = (sleeptime_us / 1000) ;
	if (sleeptime_us % 1000 > 0) {
		sleeptime_ms += 1;
	}

	sleeptime_ms += IRRC_SEND_OVERTIME;
	if (((sleeptime_ms * HZ) % 1000) > 0) {
		jiffies_unit = 1 ;
	}

	sleeptime_ms = (sleeptime_ms * HZ / 1000);
	sleeptime_ms += jiffies_unit;
	nowtime_ms = sleeptime_ms;

	if (loopcnt > 0 && irrc_kdrv_state == IRRC_KDRV_STATUS_SENDING) {
		sleeptime_ms = beforetime_ms;
	} else if (loopcnt > 0 &&
			irrc_kdrv_state == IRRC_KDRV_STATUS_SENDINGFAINAL) {
		sleeptime_ms += beforetime_ms;
	}
	beforetime_ms = nowtime_ms;

	IRRCLOG_DEBUG("sleeptime=%ld(ms)\n", sleeptime_ms*10);

	return sleeptime_ms;
}

static	void irrc_kdrv_data_change(
		irrc_senddata_u *senddata, uint32 data_length)
{
	int	wordcnt;
	int	wordnum;
	uint8	cnvdata[2];

	wordnum = (data_length / IRRC_DATASET_BSIZE);

	if ((data_length % IRRC_DATASET_BSIZE) > 0) {
		wordnum += 1;
	}

	if (data_length == 0) {
		wordcnt = 0;
		senddata->out_data[wordcnt] = 0x0000;
	} else {
		for (wordcnt = 0; (wordcnt < wordnum); wordcnt++) {
			if (wordcnt == (wordnum - 1)) {
				if ((data_length % IRRC_DATASET_BSIZE) > 0 &&
				     (data_length % IRRC_DATASET_BSIZE) <=
							IRRC_USER_DATA_BSIZE) {
					cnvdata[0] = 0x00;
				} else {
					cnvdata[0] =
					   senddata->in_data[wordcnt * 2 + 1];
				}
			} else {
				cnvdata[0] =
					senddata->in_data[wordcnt * 2 + 1];
			}
			cnvdata[1] = senddata->in_data[wordcnt * 2];
			senddata->out_data[wordcnt] = cnvdata[0];
			senddata->out_data[wordcnt] <<= 8;
			senddata->out_data[wordcnt] += cnvdata[1];
		}
	}

	return;
}

static	ssize_t show_irrc_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t	result;

	IRRCLOG_DEBUG("IN\n");

	result = scnprintf(buf, PAGE_SIZE, "%s\n",
			SH_IRRC_KERNEL_DRIVER_NAME);

	IRRCLOG_DEBUG("show name buf=%s", buf);

	return result;
}
static	ssize_t show_irrc_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t	result;

	result = scnprintf(buf, PAGE_SIZE, "%s\n",
			SH_IRRC_KERNEL_DRIVER_VERSION);

	IRRCLOG_DEBUG("show version buf=%s", buf);

	return result;
}
static	ssize_t show_irrc_capability(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t	result;
	int	cnt 	  = 0;
static	uint8	*format[] = {
				"Capability\n",
				"  Carrier H/L MIN(0.1us) : %d\n",
				"  Carrier H/L MAX(0.1us) : %d\n",
				"  Data 0/1 MAX(us)       : %d\n",
				"  Data 0/1 MIN(us)       : %d\n",
				"  Leader H/L MAX(us)     : %d\n",
				"  Trailer H MAX(us)      : %d\n",
				"  Frame MAX(us)          : %d\n",
				"  End to start MIN(us)   : %d\n",
				"  End to start MAX(us)   : %d\n",
				"  Data length MAX(bit)   : %d\n",
				"  Frame repeat time MAX  : %d\n",
				"  Frame count MAX        : %d\n",
				"  Block repeat time MAX  : %d\n",
				NULL
			};
	uint8	outstr[512];

	IRRCLOG_DEBUG("IN\n");

	outstr[0] = '\0';
	while (format[cnt] != NULL) {
		strncat(outstr, format[cnt], strlen(format[cnt])+1);
		cnt++;
	}
	result = scnprintf(buf, PAGE_SIZE, outstr,
				irrc_capainfo.carrier_min,
				irrc_capainfo.carrier_max,
				irrc_capainfo.pulse_min,
				irrc_capainfo.pulse_max,
				irrc_capainfo.leader_max,
				irrc_capainfo.trailer_max,
				irrc_capainfo.frame_max,
				irrc_capainfo.end_to_start_min,
				irrc_capainfo.end_to_start_max,
				irrc_capainfo.data_max,
				irrc_capainfo.frame_repeat_max,
				irrc_capainfo.frame_count_max,
				irrc_capainfo.block_repeat_max);

	IRRCLOG_DEBUG("OUT\n");

	return result;
}
static	ssize_t show_irrc_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;
	int	cnt 	  = 0;
	int32	carrier_high, carrier_low;
static	uint8	*format[] = {
				"Send parameter\n",
				"  Carrier High(0.1us) : %d\n",
				"  Carrier Low(0.1us)  : %d\n",
				"  Data0 Modulation    : %d\n",
				"  Data1 Modulation    : %d\n",
				"  Data0 High(us)      : %d\n",
				"  Data0 Low(us)       : %d\n",
				"  Data1 High(us)      : %d\n",
				"  Data1 Low(us)       : %d\n",
				"  Leader High(us)     : %d\n",
				"  Leader Low(us)      : %d\n",
				"  Trailer High(us)    : %d\n",
				"  End to start(us)    : %d\n",
				"\nSTATUS              : %s\n",
				NULL
			};
static	const	uint8	*strstat[] = {
				"IDLE",
				"STANDBY",
				"READY",
				"SENDING",
				"SENDINGFAINAL"
			};
	uint8	outstr[512];

	IRRCLOG_DEBUG("IN\n");

	outstr[0] = '\0';
	while (format[cnt] != NULL) {
		strncat(outstr, format[cnt], strlen(format[cnt])+1);
		cnt++;
	}

	carrier_high = irrc_sendinfo.carrier_high * 10;
	carrier_high *= IRRC_SOURCE_CLK;
	carrier_high /= IRRC_CARRIER_UNIT_CONV;
	carrier_low = irrc_sendinfo.carrier_low * 10;
	carrier_low *= IRRC_SOURCE_CLK;
	carrier_low /= IRRC_CARRIER_UNIT_CONV;

	result = scnprintf(buf, PAGE_SIZE, outstr,
				carrier_high,
				carrier_low,
				irrc_sendinfo.modulation0,
				irrc_sendinfo.modulation1,
				irrc_sendinfo.pulse0_high,
				irrc_sendinfo.pulse0_low,
				irrc_sendinfo.pulse1_high,
				irrc_sendinfo.pulse1_low,
				irrc_sendinfo.leader_high,
				irrc_sendinfo.leader_low,
				irrc_sendinfo.trailer_high,
				irrc_sendinfo.end_to_start,
				strstat[irrc_kdrv_state]);

	IRRCLOG_DEBUG("OUT\n");

	return result;
}

static	DEVICE_ATTR(
	name,
	S_IRUGO,
	show_irrc_name,
	NULL
);
static	DEVICE_ATTR(
	version,
	S_IRUGO,
	show_irrc_version,
	NULL
);
static	DEVICE_ATTR(
	capability,
	S_IRUGO,
	show_irrc_capability,
	NULL
);
static	DEVICE_ATTR(
	status,
	S_IRUGO,
	show_irrc_status,
	NULL
);

static	struct attribute *irrc_device_attributes[] = {
	&dev_attr_name.attr,
	&dev_attr_version.attr,
	&dev_attr_capability.attr,
	&dev_attr_status.attr,
	NULL,
};

static	struct attribute_group irrc_device_attributes_gourp = {
	.attrs = irrc_device_attributes,
};
static	struct file_operations	IrRC_kdrv_fops = {
	.owner	 = THIS_MODULE,
	.open	 = irrc_kdrv_fops_open,
	.release = irrc_kdrv_fops_close,
	.ioctl	 = irrc_kdrv_fops_ioctl,
	.write	 = irrc_kdrv_fops_write
};
static	int 			IrRC_kdrv_major;
static	struct cdev 		IrRC_kdrv_cdev;

#define	IRRC_KDRV_DEVS		(1)
#define	IRRC_KDRV_MINOR		(0)
#define	IRRC_KDRV_BASE_MINOR	(0)
#define	IRRC_KDRV_MINOR_COUNT	(1)

#define	IRRC_KDRV_CLASS_NAME	"cls_shirrc"
static	struct class 		*IrRC_kdrv_class = NULL;
static	dev_t 			IrRC_kdrv_dev;

static	int irrc_driver_remove(struct platform_device *pdev)
{
	int	result	= IRRC_SUCCESS;
	dev_t	dev;

	if (irrc_kdrv_state != IRRC_KDRV_STATUS_IDLE) {
 		if ((irrc_kdrv_state == IRRC_KDRV_STATUS_SENDING) ||
			(irrc_kdrv_state == IRRC_KDRV_STATUS_SENDINGFAINAL)) {
			IRRCLOG_INFO("stop send\n");

			irrc_wakeup_cause = IRRC_KDRV_WU_EV_CLOSE;

			irrc_reg_sys_reset();

			wake_up(&irrc_kdrv_wu_que);
			IRRCLOG_DEBUG("irrc driver remove wakeup (cause=%d)\n",
							irrc_wakeup_cause);
		}

		irrc_reg_set_sys_pwroff();

		irrc_reg_init();

		irda_kdrv_irsd_shutdown();

		irrc_reg_iounmap();
	}

	sysfs_remove_group(&(pdev->dev.kobj), &irrc_device_attributes_gourp);

	dev = MKDEV(IrRC_kdrv_major, IRRC_KDRV_BASE_MINOR);
	device_destroy(IrRC_kdrv_class, IrRC_kdrv_dev);
	class_destroy(IrRC_kdrv_class);
	cdev_del(&IrRC_kdrv_cdev);
	unregister_chrdev_region(dev, IRRC_KDRV_DEVS);

	return result;
}

static	struct platform_driver irrc_driver = {
		.remove = __devexit_p(irrc_driver_remove),
		.driver = {
		.name = IRRC_PLFM_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static	int irrc_driver_init(void)
{
	int	result;

	result = platform_driver_register(&irrc_driver);
	if (result != 0) {
		IRRCLOG_ERROR(
			"platform_driver_register() failed (%d)\n", result);
	}

	return result;
}

static	struct platform_device irrc_device = {
		.name	= IRRC_PLFM_DRIVER_NAME,
		.id	= -1,
	};

static	int __init irrc_device_init(void)
{
	int	result	= 0;

	result = platform_device_register(&irrc_device);
	if (result != 0) {
		IRRCLOG_ERROR(
			"platform_device_register() failed (%d)\n", result);
	}

	result = sysfs_create_group(&(irrc_device.dev.kobj),
					&irrc_device_attributes_gourp);
	if (result != 0) {
		IRRCLOG_ERROR("sysfs_create_group() failed (%d)\n", result);
	}

	return result;
}

static	int __init irrc_kdrv_module_init(void)
{
	struct device	*dev_create;
	dev_t 		dev;
	int 		result		= IRRC_SUCCESS;
	int 		alloc_ret	= 0;
	int		cdev_ret	= 0;

	result = irrc_device_init();
	if (result == 0) {
		result = irrc_driver_init();
	}
	if (result != 0) {
		return result;
	}

	IrRC_kdrv_major = 0;
	dev = MKDEV(IrRC_kdrv_major, 0);
	alloc_ret = alloc_chrdev_region(&dev, IRRC_KDRV_BASE_MINOR,
				IRRC_KDRV_DEVS, IRRC_FOPS_DEVFILE_NAME);
	if (alloc_ret != 0) {
		IRRCLOG_ERROR(
			"alloc_chrdev_region() failed (%d)\n", alloc_ret);
		result = -EIO;
		goto moduleiniterr;
	}

	IrRC_kdrv_major = MAJOR(dev);
	cdev_init(&IrRC_kdrv_cdev, &IrRC_kdrv_fops);
	IrRC_kdrv_cdev.owner = THIS_MODULE;
	IrRC_kdrv_cdev.ops = &IrRC_kdrv_fops;
	cdev_ret = cdev_add(&IrRC_kdrv_cdev,
				MKDEV(IrRC_kdrv_major, IRRC_KDRV_MINOR),
				IRRC_KDRV_MINOR_COUNT);
	if (cdev_ret != 0) {
		IRRCLOG_ERROR("cdev_add() failed (%d)\n", cdev_ret);
		unregister_chrdev_region(dev, IRRC_KDRV_DEVS);
		result = -EIO;
		goto moduleiniterr;
	}

	IrRC_kdrv_class = class_create(THIS_MODULE, IRRC_KDRV_CLASS_NAME);
	if (IS_ERR(IrRC_kdrv_class)) {
		IRRCLOG_ERROR("class_create() failed\n");
		cdev_del(&IrRC_kdrv_cdev);
		unregister_chrdev_region(dev, IRRC_KDRV_DEVS);
		result = -ENOMEM;
		goto moduleiniterr;
	}

	IrRC_kdrv_dev = MKDEV(IrRC_kdrv_major, IRRC_KDRV_MINOR);
	dev_create = device_create(IrRC_kdrv_class,
			NULL, IrRC_kdrv_dev, NULL, IRRC_FOPS_DEVFILE_NAME);
	if (IS_ERR(dev_create)) {
		IRRCLOG_ERROR("device_create() failed\n");
		class_destroy(IrRC_kdrv_class);
		cdev_del(&IrRC_kdrv_cdev);
		unregister_chrdev_region(dev, IRRC_KDRV_DEVS);
		result = -ENODEV;
		goto moduleiniterr;
	}

	irrc_kdrv_state = IRRC_KDRV_STATUS_IDLE;

	irrc_kdrv_sendinfo_init();

moduleiniterr:
	if (result != IRRC_SUCCESS) {
	 	platform_driver_unregister(&irrc_driver);
		platform_device_unregister(&irrc_device);
	}

	return result;
}

static	void __exit irrc_kdrv_module_exit(void)
{
	platform_driver_unregister(&irrc_driver);
	platform_device_unregister(&irrc_device);
}

module_init(irrc_kdrv_module_init);
module_exit(irrc_kdrv_module_exit);

MODULE_DESCRIPTION("IrRC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION(SH_IRRC_KERNEL_DRIVER_VERSION);
