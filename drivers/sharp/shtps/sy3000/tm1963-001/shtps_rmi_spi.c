/* drivers/sharp/shtps/sy3000/tm1836-001/shtps_rmi_spi.c
 *
 * Copyright (c) 2010, Sharp. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>

#include <sharp/shtps_dev.h>

//#define SHTPS_LOG_SEQ_ENABLE
//#define SHTPS_LOG_SPIACCESS_SEQ_ENABLE
//#define SHTPS_LOG_DEBUG_ENABLE
#define SHTPS_LOG_ERROR_ENABLE
#define SHTPS_LOG_EVENT_ENABLE

#ifdef SHTPS_LOG_EVENT_ENABLE
	#define SHTPS_LOG_OUTPUT_SWITCH_ENABLE
#endif /* #if defined( SHTPS_LOG_EVENT_ENABLE ) */

#define SHTPS_DEBUG_VARIABLE_DEFINES
#define SHTPS_BOOT_FWUPDATE_ENABLE
//#define SHTPS_BOOT_FWUPDATE_FORCE_UPDATE
#define SHTPS_OWN_THREAD_ENABLE
#define SHTPS_NOTIFY_INTERVAL_GUARANTEED


#if defined ( CONFIG_SHTPS_SY3000_AUTOREZERO_CONTROL )
	#define SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE
	#define SHTPS_AUTOREZERO_CONTROL_ENABLE
#endif /* #if defined ( CONFIG_SHTPS_SY3000_AUTOREZERO_CONTROL ) */

#if defined( CONFIG_SPI_CUST_SH )
	#define SHTPS_SPIRETRY_ENABLE
#endif /* #if defined( CONFIG_SPI_CUST_SH ) */

#if defined( CONFIG_SHTPS_SY3000_TM1836_001 )
	#define SHTPS_VKEY_INVALID_AREA_ENABLE
#endif /* #if defined( CONFIG_SHTPS_SY3000_TM1836_001 ) */

#if defined( CONFIG_SHTPS_SY3000_TM1918_001 ) || defined( CONFIG_SHTPS_SY3000_TM1963_001 )
	#define SHTPS_VKEY_CANCEL_ENABLE
#endif /* #if defined( CONFIG_SHTPS_SY3000_TM1918_001 ) || defined( CONFIG_SHTPS_SY3000_TM1963_001 ) */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
	#if defined( CONFIG_SHTPS_SY3000_TM1836_001 )
		#include "shtps_fw_tm1836-001.h"
	#elif defined( CONFIG_SHTPS_SY3000_TM1901_001 )
		#include "shtps_fw_tm1901-001.h"
	#elif defined( CONFIG_SHTPS_SY3000_TM1918_001 )
		#include "shtps_fw_tm1918-001.h"
	#elif defined( CONFIG_SHTPS_SY3000_TM1963_001 )
		#include "shtps_fw_tm1963-001.h"
	#elif defined( CONFIG_SHTPS_SY3000_TM1980_001 )
		#include "shtps_fw_tm1980-001.h"
	#else
		#undef SHTPS_BOOT_FWUPDATE_ENABLE
	#endif
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_LOG_SEQ_ENABLE )
	#include "shtps_seqlog.h"
	#define _log_msg_sync(id, fmt, ...) \
		_seq_log(SHTPS_SEQLOG_TAG, _SEQ_LOG_MSGKIND_SYNC, id, fmt "\n", ##__VA_ARGS__)

	#define _log_msg_send(id, fmt, ...) \
		_seq_log(SHTPS_SEQLOG_TAG, _SEQ_LOG_MSGKIND_SEND, id, fmt "\n", ##__VA_ARGS__)
		
	#define _log_msg_recv(id, fmt, ...) \
		_seq_log(SHTPS_SEQLOG_TAG, _SEQ_LOG_MSGKIND_RECEIVE, id, fmt "\n", ##__VA_ARGS__)
#else
	#define _log_msg_sync(id, fmt, ...)
	#define _log_msg_send(id, fmt, ...)
	#define _log_msg_recv(id, fmt, ...)
#endif /* defined( SHTPS_LOG_SEQ_ENABLE ) */

#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_ERROR(p)	p
#else
	#define SHTPS_LOG_ERROR(p)
#endif /* defined( SHTPS_LOG_ERROR_ENABLE ) */

#if defined( SHTPS_LOG_DEBUG_ENABLE )
	#define SHTPS_LOG_DEBUG(p)	p
	#define SHTPS_LOG_FUNC_CALL()	printk(KERN_DEBUG "[shtps]%s()\n", __func__)
	#define SHTPS_LOG_FUNC_CALL_INPARAM(param)	printk(KERN_DEBUG "[shtps]%s(%d)\n", __func__, param)
#else
	#define SHTPS_LOG_DEBUG(p)
	#define SHTPS_LOG_FUNC_CALL()
	#define SHTPS_LOG_FUNC_CALL_INPARAM(param)
#endif /* defined( SHTPS_LOG_DEBUG_ENABLE ) */

#if defined( SHTPS_LOG_EVENT_ENABLE ) && defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	#define SHTPS_LOG_EVENT(p)		\
		if(gLogOutputEnable != 0){	\
			p						\
		}
#elif defined( SHTPS_LOG_EVENT_ENABLE )
	#define SHTPS_LOG_EVENT(p)	p
#else
	#define SHTPS_LOG_EVENT(p)
#endif /* defined( SHTPS_LOG_EVENT_ENABLE ) */

#define SPI_ERR_CHECK(check, label) \
	if((check)) goto label


/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	static int SHTPS_EVENT_POLL_INTERVAL = 			10;
	static int SHTPS_POLL_MIN_INTERVAL =			15000;
	static int SHTPS_POLL_STANDARD_VAL =			15000;
	#else
	static int SHTPS_EVENT_POLL_INTERVAL = 			12;
	#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */
	static int SHTPS_DRAG_THRESH_VAL_X_1ST = 		CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_2ND = 		CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI =	CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI =	CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_1ST = 		CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_2ND = 		CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI =	CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI =	CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_RETURN_TIME =		250;
#else /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	#define SHTPS_EVENT_POLL_INTERVAL				10
	#define SHTPS_POLL_MIN_INTERVAL					15000
	#define SHTPS_POLL_STANDARD_VAL					15000
	#else
	#define SHTPS_EVENT_POLL_INTERVAL				12
	#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */
	#define SHTPS_DRAG_THRESH_VAL_X_1ST 			CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_X_2ND 			CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI		CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI		CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_1ST 			CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_2ND 			CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI		CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI		CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_RETURN_TIME			250
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

#define SHTPS_BOOTLOADER_ACK_TMO				1000
#define SHTPS_FWTESTMODE_ACK_TMO				1000
#define SHTPS_DIAGPOLL_TIME						100

#define SHTPS_FINGER_WIDTH_PALMDET				15
#define SHTPS_FINGER_WIDTH_MIN					1
#define SHTPS_FINGER_WIDTH_MAX					8

#define SHTPS_LOS_SINGLE						0x05
#define SHTPS_LOS_MULTI							0x0D

#define SHTPS_STARTUP_MIN_TIME					300
#define SHTPS_POWERON_WAIT_MS					400
#define SHTPS_POWEROFF_WAIT_MS					10
#define SHTPS_HWRESET_TIME_US					1
#define SHTPS_HWRESET_AFTER_TIME_MS				1
#define SHTPS_HWRESET_WAIT_MS					290
#define SHTPS_SWRESET_WAIT_MS					300
#define SHTPS_RESET_BOOTLOADER_WAIT_MS			400
#define SHTPS_SLEEP_OUT_WAIT_US					67500
#define SHTPS_SLEEP_IN_WAIT_MS					230
#define SHTPS_LOADERCMD_WAIT_US					750

#define SHTPS_REZERO_DISABLE_COUNT				( 300 / SHTPS_EVENT_POLL_INTERVAL)
#define SHTPS_REZERO_SINGLE_FINGERS_COUNT_BOOT	(1000 / SHTPS_EVENT_POLL_INTERVAL)
#define SHTPS_REZERO_SINGLE_FINGERS_COUNT_ECALL	(1000 / SHTPS_EVENT_POLL_INTERVAL)
#define SHTPS_REZERO_SINGLE_FINGERS_COUNT		( 100 / SHTPS_EVENT_POLL_INTERVAL)

#define SHTPS_POSTYPE_X							0
#define SHTPS_POSTYPE_Y							1

#define SHTPS_POS_SCALE_X(ts)		(((CONFIG_SHTPS_SY3000_PANEL_SIZE_X) * 10000) / ts->map.fn11.ctrl.maxXPosition)
#define SHTPS_POS_SCALE_Y(ts)		(((CONFIG_SHTPS_SY3000_PANEL_SIZE_Y) * 10000) / ts->map.fn11.ctrl.maxYPosition)

/* -----------------------------------------------------------------------------------
 */
static DEFINE_MUTEX(shtps_ctrl_lock);
static DEFINE_MUTEX(shtps_loader_lock);

/* -----------------------------------------------------------------------------------
 */

struct shtps_irq_info{
	int							irq;
	u8							state;
	u8							wake;
};

struct shtps_state_info{
	int							state;
	int							mode;
	int							starterr;
	unsigned long				starttime;
};

struct shtps_loader_info{
	int							ack;
	wait_queue_head_t			wait_ack;
};

struct shtps_diag_info{
	u8							pos_mode;
	u8							tm_mode;
	u8							tm_data[SHTPS_TM_TXNUM_MAX * SHTPS_TM_RXNUM_MAX * 2];
	int							event;
	int							tm_ack;
	int							tm_stop;
	wait_queue_head_t			wait;
	wait_queue_head_t			tm_wait_ack;
};

struct shtps_facetouch_info{		
	int							mode;
	int							state;
	int							off_detect;
	int							palm_thresh;
	int							wake_sig;
	wait_queue_head_t			wait_off;
	struct wake_lock			wake_lock;
};

struct shtps_offset_info{
	int							enabled;
	u16							base[5];
	signed short				diff[12];
};

struct shtps_polling_info{
	int							boot_rezero_flag;
	int							stop_margin;
	int							stop_count;
	int							single_fingers_count;
	int							single_fingers_max;
	u8							single_fingers_enable;
};

#if defined( SHTPS_OWN_THREAD_ENABLE )
struct shtps_req_msg {
	struct list_head	request;
	struct list_head	queue;
	void	(*complete)(void *context);
	void	*context;
	int		status0;
	int		status1;
};

struct shtps_req_buf {
	struct list_head	request_list;
	int		event;
	int		param;
	int		result;
	
	int		param0;
	int		param1;
	int		param2;
};

struct shtps_spi_complete_ctrl_tbl{
	struct completion	work;
	int					wait_utime;
};
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */

struct shtps_rmi_spi {
	struct spi_device*			spi;
	struct input_dev*			input;
	int							rst_pin;
	struct work_struct			irq_workq;
	struct shtps_irq_info		irq_mgr;
	struct shtps_touch_info		report_info;
	struct shtps_touch_info		center_info;
	struct shtps_state_info		state_mgr;
	struct shtps_loader_info	loader;
	struct shtps_diag_info		diag;
	struct shtps_facetouch_info	facetouch;
	struct shtps_offset_info	offset;
	struct shtps_polling_info	poll_info;
	struct rmi_map				map;
	struct shtps_touch_state	touch_state;
	wait_queue_head_t			wait_start;
	struct hrtimer				polling_timer;
	struct work_struct			polling_work;
#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	struct hrtimer				usleep_timer;
	struct timeval				notify_tv;
	struct timeval				pollstart_tv;
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */
	struct delayed_work 		tmo_check;
	unsigned char				finger_state[SHTPS_FINGER_MAX / 4 + 1];
	struct hrtimer				rezero_delayed_timer;
	struct work_struct			rezero_delayed_work;
#if defined( SHTPS_OWN_THREAD_ENABLE )
	spinlock_t					queue_lock;
	struct list_head			queue;
	struct workqueue_struct		*workqueue_p;
	struct work_struct			work_data;
	struct shtps_req_msg		*cur_msg_p;
	struct shtps_req_buf		*cur_request_p;
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */

#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
	u8							invalid_area_touch;
#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */

#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	u8							vkey_invalid;
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

static dev_t 					shtpsif_devid;
static struct class*			shtpsif_class;
static struct device*			shtpsif_device;
struct cdev 					shtpsif_cdev;
static struct shtps_rmi_spi*	gShtps_rmi_spi = NULL;

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static u8						gLogOutputEnable = 0;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
static DECLARE_COMPLETION(shtps_usleep_timer_comp);
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */
/* -----------------------------------------------------------------------------------
 */
enum{
	SHTPS_FWTESTMODE_V01 = 0x00,
	SHTPS_FWTESTMODE_V02,
};

enum{
	SHTPS_REZERO_REQUEST_REZERO				= 0x01,
	SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE	= 0x02,
	SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE	= 0x04,
	SHTPS_REZERO_REQUEST_WAKEUP_REZERO		= 0x08,
};

enum{
	SHTPS_REZERO_HANDLE_EVENT_MTD = 0,
	SHTPS_REZERO_HANDLE_EVENT_TOUCH,
	SHTPS_REZERO_HANDLE_EVENT_TOUCHUP,
};

enum{
	SHTPS_REZERO_TRIGGER_BOOT = 0,
	SHTPS_REZERO_TRIGGER_WAKEUP,
	SHTPS_REZERO_TRIGGER_ENDCALL,
};

enum{
	SHTPS_EVENT_TU,
	SHTPS_EVENT_TD,
	SHTPS_EVENT_DRAG,
	SHTPS_EVENT_MTDU,
};

enum{
	SHTPS_STARTUP_SUCCESS,
	SHTPS_STARTUP_FAILED
};

enum{
	SHTPS_IRQ_WAKE_DISABLE,
	SHTPS_IRQ_WAKE_ENABLE,
};

enum{
	SHTPS_IRQ_STATE_DISABLE,
	SHTPS_IRQ_STATE_ENABLE,
};

enum{
	SHTPS_MODE_NORMAL,
	SHTPS_MODE_LOADER,
};

enum{
	SHTPS_EVENT_START,
	SHTPS_EVENT_STOP,
	SHTPS_EVENT_SLEEP,
	SHTPS_EVENT_WAKEUP,
	SHTPS_EVENT_STARTLOADER,
	SHTPS_EVENT_STARTTM,
	SHTPS_EVENT_STOPTM,
	SHTPS_EVENT_FACETOUCHMODE_ON,
	SHTPS_EVENT_FACETOUCHMODE_OFF,
	SHTPS_EVENT_GETSENSOR,
	SHTPS_EVENT_INTERRUPT,
	SHTPS_EVENT_TIMEOUT,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	SHTPS_EVENT_FORMCHANGE,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

enum{
	SHTPS_STATE_IDLE,
	SHTPS_STATE_WAIT_WAKEUP,
	SHTPS_STATE_WAIT_READY,
	SHTPS_STATE_ACTIVE,
	SHTPS_STATE_BOOTLOADER,
	SHTPS_STATE_FACETOUCH,
	SHTPS_STATE_FWTESTMODE,
	SHTPS_STATE_SLEEP,
	SHTPS_STATE_SLEEP_FACETOUCH,
};

typedef int (shtps_state_func)(struct shtps_rmi_spi *ts, int param);
struct shtps_state_func {
	shtps_state_func	*enter;
	shtps_state_func	*start;
	shtps_state_func	*stop;
	shtps_state_func	*sleep;
	shtps_state_func	*wakeup;
	shtps_state_func	*start_ldr;
	shtps_state_func	*start_tm;
	shtps_state_func	*stop_tm;
	shtps_state_func	*facetouch_on;
	shtps_state_func	*facetouch_off;
	shtps_state_func	*get_sensor;
	shtps_state_func	*interrupt;
	shtps_state_func	*timeout;
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	shtps_state_func	*form_change;
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
static const struct{
	int	sx;
	int sy;
	int ex;
	int ey;
} shtps_invalid_area[] = {
	{    0, 960,  39, 1025 },
	{  500, 960, 539, 1025 },
	{ -1, -1, -1, -1 }
};
#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_rmi_spi *ts, int event, int param);
static int state_change(struct shtps_rmi_spi *ts, int state);
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_getsensor(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param);
static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts);
static int shtps_start(struct shtps_rmi_spi *ts);
static int shtps_wait_startup(struct shtps_rmi_spi *ts);
static u16 shtps_fwver(struct shtps_rmi_spi *ts);
static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param);
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
static int shtps_statef_active_formchange(struct shtps_rmi_spi *ts, int param);
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
static void shtps_rezero(struct shtps_rmi_spi *ts);

#if defined( SHTPS_OWN_THREAD_ENABLE )
static void shtps_func_api_request_from_irq_complete(void *arg_p);
static void shtps_func_api_request_from_irq( struct shtps_rmi_spi *ts, int event, int param);
static void shtps_func_workq( struct work_struct *work_p );
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data);
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
const static struct shtps_state_func state_idle = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_idle_start,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_idle_start_ldr,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_idle_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change   = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_waiwakeup = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_waiwakeup_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_waiwakeup_int,
    .timeout        = shtps_statef_waiwakeup_tmo,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_waiready = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_state_waiready_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_state_waiready_int,
    .timeout        = shtps_state_waiready_tmo,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_active = {
    .enter          = shtps_statef_active_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_active_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_active_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_active_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_active_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_active_formchange,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_loader = {
    .enter          = shtps_statef_loader_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_nop,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_loader_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_facetouch = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_facetouch_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_facetouch_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_facetouch_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_fwtm = {
    .enter          = shtps_statef_fwtm_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_nop,
    .stop_tm        = shtps_statef_fwtm_stoptm,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_fwtm_getsensor,
    .interrupt      = shtps_statef_fwtm_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_sleep = {
    .enter          = shtps_statef_sleep_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_sleep_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_sleep_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func state_sleep_facetouch = {
    .enter          = shtps_statef_sleep_facetouch_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_sleep_facetouch_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_facetouch_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_sleep_facetouch_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_sleep_facetouch_int,
    .timeout        = shtps_statef_nop,
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	.form_change    = shtps_statef_nop,
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
};

const static struct shtps_state_func *state_func_tbl[] = {
	&state_idle,
	&state_waiwakeup,
	&state_waiready,
	&state_active,
	&state_loader,
	&state_facetouch,
	&state_fwtm,
	&state_sleep,
	&state_sleep_facetouch,
};

/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_spisync(struct shtps_rmi_spi *ts, struct spi_message *msg)
{
#if defined( SHTPS_SPIRETRY_ENABLE )
	int retry = 3;
	int err;
	
	do{
		err = spi_sync(ts->spi, msg);
		if(err != 0){
			spi_setup(ts->spi);
		}
	}while(err != 0 && retry-- > 0);
	
	return err;
#else
	return spi_sync(ts->spi, msg);
#endif /* #if defined( SHTPS_SPI_INIT_ENABLE ) */
}

static int shtps_rmi_write(struct shtps_rmi_spi *ts, u16 addr, u8 data)
{
	struct spi_message	message;
	struct spi_transfer	t;
	u8					tx_buf[3];
	int					status;

	memset(&t, 0, sizeof(t));
	
	spi_message_init(&message);
	spi_message_add_tail(&t, &message);
	t.tx_buf 		= tx_buf;
	t.rx_buf 		= NULL;
	t.len    		= 3;

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;
	tx_buf[2] = data;
	
#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = shtps_rmi_spisync(ts, &message);
#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	udelay(65);

	return status;
}

static u8 shtps_rmi_readb(struct shtps_rmi_spi *ts, u16 addr, u8 *buf)
{
	struct spi_message	message;
	struct spi_transfer	t;
	u8					tx_buf[2];
	u8					rx_buf[2];
	int					status;

	memset(&t, 0, sizeof(t));
	
	spi_message_init(&message);
	spi_message_add_tail(&t, &message);
	t.tx_buf		= tx_buf;
	t.rx_buf  		= rx_buf;
	t.len			= 2;
	tx_buf[0] = ((addr >> 8) & 0xff) | 0x80;
	tx_buf[1] = addr & 0xff;
		
#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR, "0x%04X", addr);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_sync(ts->spi, &message);
#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR_COMP, "%d", status);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status){
		return -1;
	}
	spi_transfer_del(&t);
	udelay(40);
	
	t.len			= 1;
	t.tx_buf		= NULL;
	t.rx_buf  		= rx_buf;
	spi_message_add_tail(&t, &message);

#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_READ, "0x%04X", addr);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_sync(ts->spi, &message);
#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_READ_COMP, "0x%04X|0x%02X", addr, rx_buf[0]);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status){
		return -1;
	}
	udelay(40);
	
	*buf = rx_buf[0];

	return 0;
}

static int shtps_rmi_read(struct shtps_rmi_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int i;
	int status;
	
	for(i = 0;i < size;i++){
#if defined( SHTPS_SPIRETRY_ENABLE )
		int retry;
		retry = 3;
		do{
			status = shtps_rmi_readb(ts, addr + i, buf + i);
			if(status != 0){
				spi_setup(ts->spi);
			}
		}while(status != 0 && retry-- > 0);
#else
		status = shtps_rmi_readb(ts, addr + i, buf + i);
#endif /* #if defined( SHTPS_SPIRETRY_ENABLE ) */
		if(status){	
			goto err_exit;
		}
	}
	return 0;

err_exit:
	return status;
}

/* -----------------------------------------------------------------------------------
 */
static irqreturn_t shtps_irq(int irq, void *dev_id)
{
	struct shtps_rmi_spi	*ts = dev_id;

	_log_msg_send( LOGMSG_ID__IRQ_NOTIFY, "");
#if defined( SHTPS_OWN_THREAD_ENABLE )
	shtps_func_api_request_from_irq(ts, SHTPS_EVENT_INTERRUPT, 1);
#else
	schedule_work(&ts->irq_workq);
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */
	return IRQ_HANDLED;
}
static void shtps_work_irqf(struct work_struct *work)
{
#if defined( SHTPS_OWN_THREAD_ENABLE )
#else
	struct shtps_rmi_spi *ts = container_of(work, struct shtps_rmi_spi, irq_workq);

	_log_msg_recv( LOGMSG_ID__IRQ_NOTIFY, "");
	request_event(ts, SHTPS_EVENT_INTERRUPT, 1);
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */
}

static enum hrtimer_restart shtps_polling_timer_function(struct hrtimer *timer)
{
	struct shtps_rmi_spi *ts = container_of(timer, struct shtps_rmi_spi, polling_timer);
	
	_log_msg_send( LOGMSG_ID__POLL_TIMEOUT, "");
#if defined( SHTPS_OWN_THREAD_ENABLE )
	shtps_func_api_request_from_irq(ts, SHTPS_EVENT_INTERRUPT, 0);
#else
	schedule_work(&ts->polling_work);
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */
	return HRTIMER_NORESTART;
}
static void shtps_work_pollf(struct work_struct *work)
{
#if defined( SHTPS_OWN_THREAD_ENABLE )
#else
	struct shtps_rmi_spi *ts = container_of(work, struct shtps_rmi_spi, polling_work);

	_log_msg_recv( LOGMSG_ID__POLL_TIMEOUT, "");
	request_event(ts, SHTPS_EVENT_INTERRUPT, 0);
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */
}

static void shtps_work_tmof(struct work_struct *data)
{
	struct delayed_work  *dw = container_of(data, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, tmo_check);
	
	_log_msg_sync( LOGMSG_ID__TIMER_TIMEOUT, "");
	request_event(ts, SHTPS_EVENT_TIMEOUT, 0);
}

#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
static enum hrtimer_restart shtps_usleep_timer_function(struct hrtimer *timer)
{
	complete(&shtps_usleep_timer_comp);
	return HRTIMER_NORESTART;
}

static void shtps_usleep_timer(struct shtps_rmi_spi *ts, u32 usec)
{
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_usleep_timer usec:%d\n",usec); );

	hrtimer_cancel(&ts->usleep_timer);
	hrtimer_start(&ts->usleep_timer, ktime_set(0, usec * 1000), HRTIMER_MODE_REL);
	wait_for_completion(&shtps_usleep_timer_comp);

	return;
}

static void shtps_set_notify_time(struct shtps_rmi_spi *ts)
{
	do_gettimeofday(&ts->notify_tv);
}

static void shtps_wait_notify_interval(struct shtps_rmi_spi *ts)
{
	u32 calc_time = 0;
	struct timeval tv;
	
	do_gettimeofday(&tv);
	if(tv.tv_usec <= ts->notify_tv.tv_usec){
		tv.tv_usec += 1000000;
	}
	calc_time = tv.tv_usec - ts->notify_tv.tv_usec;
	
	if(calc_time < SHTPS_POLL_STANDARD_VAL){
		shtps_usleep_timer(ts, (SHTPS_POLL_STANDARD_VAL - calc_time));
	}
}

static void shtps_set_pollstart_time(struct shtps_rmi_spi *ts)
{
	do_gettimeofday(&ts->pollstart_tv);
}

static void shtps_wait_poll_interval(struct shtps_rmi_spi *ts)
{
	u32 calc_time = 0;
	struct timeval tv;
	
	do_gettimeofday(&tv);
	if(tv.tv_usec <= ts->pollstart_tv.tv_usec){
		tv.tv_usec += 1000000;
	}
	calc_time = tv.tv_usec - ts->pollstart_tv.tv_usec;
	
	if(calc_time < SHTPS_POLL_MIN_INTERVAL){
		shtps_usleep_timer(ts, (SHTPS_POLL_MIN_INTERVAL - calc_time));
	}
}
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */

static enum hrtimer_restart shtps_delayed_rezero_timer_function(struct hrtimer *timer)
{
	struct shtps_rmi_spi *ts = container_of(timer, struct shtps_rmi_spi, rezero_delayed_timer);

	_log_msg_send( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	schedule_work(&ts->rezero_delayed_work);
	return HRTIMER_NORESTART;
}
static void shtps_rezero_delayed_work_function(struct work_struct *work)
{
	struct shtps_rmi_spi *ts = container_of(work, struct shtps_rmi_spi, rezero_delayed_work);

	_log_msg_recv( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	mutex_lock(&shtps_ctrl_lock);
	shtps_rezero(ts);
	mutex_unlock(&shtps_ctrl_lock);
}
static void shtps_set_palmthresh(struct shtps_rmi_spi *ts, int thresh)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	if(ts->facetouch.palm_thresh != thresh){
		int offset = 0;
		
		offset += (F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) != 0)? 6: 0;
		ts->facetouch.palm_thresh = thresh;
		shtps_rmi_write(ts,
						ts->map.fn11.ctrlBase + F11_QUERY_MAXELEC(ts->map.fn11.query.data) * 2 + 0x0C + offset, 
						ts->facetouch.palm_thresh);

		_log_msg_sync( LOGMSG_ID__SET_PALM_THRESH, "0x%04X|0x%02X", 
							ts->map.fn11.ctrlBase + F11_QUERY_MAXELEC(ts->map.fn11.query.data) * 2 + 0x0C + offset,
							ts->facetouch.palm_thresh);
		SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps] palm thresh = %d\n", ts->facetouch.palm_thresh); )
	}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static void shtps_delayed_rezero(struct shtps_rmi_spi *ts, unsigned long delay_us)
{
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_SET, "%lu", delay_us);
	hrtimer_cancel(&ts->rezero_delayed_timer);
	hrtimer_start(&ts->rezero_delayed_timer, ktime_set(0, delay_us * 1000), HRTIMER_MODE_REL);
}

static void shtps_delayed_rezero_cancel(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_CANCEL, "");
	hrtimer_try_to_cancel(&ts->rezero_delayed_timer);
}

static void shtps_rezero(struct shtps_rmi_spi *ts)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	u8  val;

	_log_msg_sync( LOGMSG_ID__REZERO_EXEC, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps] fw rezero execute\n"); )
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val & ~0x01);
	shtps_rmi_write(ts, ts->map.fn11.commandBase, 0x01);
	shtps_rmi_write(ts, 0xF0, val | 0x01);
#else
	_log_msg_sync( LOGMSG_ID__REZERO_EXEC, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps] fw rezero execute\n"); )
	shtps_rmi_write(ts, ts->map.fn11.commandBase, 0x01);
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
static u8 shtps_is_singlefinger(struct shtps_rmi_spi *ts, u8 gs_info)
{
#if defined( SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE )
	u8  ret = 0x00;
	u8  buf;
	
	shtps_rmi_read(ts, 0xF0, &buf, 1);
	if(buf & 0x02){
		ret = 0x01;
	}
	_log_msg_sync( LOGMSG_ID__GET_SINGLE_FINGER_FLAG, "%d", ret);
	return ret;
#else
	return 0x01;
#endif /* #if defined( SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE ) */
}

static void shtps_autorezero_disable(struct shtps_rmi_spi *ts)
{
	u8  val;

	_log_msg_sync( LOGMSG_ID__AUTO_REZERO_DISABLE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps] fw auto rezero disable\n"); )
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val | 0x01);
}

static void shtps_autorezero_enable(struct shtps_rmi_spi *ts)
{
	u8  val;

	_log_msg_sync( LOGMSG_ID__AUTO_REZERO_ENABLE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps] fw auto rezero enable\n"); )
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val & ~0x01);
}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

static void shtps_rezero_request(struct shtps_rmi_spi *ts, u8 request, u8 trigger)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	int single_finger_count;
	
	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}
	
	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE){
		shtps_autorezero_disable(ts);
	}
	
	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE){
		shtps_autorezero_enable(ts);

		ts->poll_info.stop_margin = SHTPS_REZERO_DISABLE_COUNT;
		ts->poll_info.single_fingers_count = 0;
		ts->poll_info.single_fingers_enable= 1;
		
		if(trigger == SHTPS_REZERO_TRIGGER_WAKEUP){
			single_finger_count = SHTPS_REZERO_SINGLE_FINGERS_COUNT;
		}else if(trigger == SHTPS_REZERO_TRIGGER_BOOT){
			single_finger_count = SHTPS_REZERO_SINGLE_FINGERS_COUNT_BOOT;
		}else if(trigger == SHTPS_REZERO_TRIGGER_ENDCALL){
			single_finger_count = SHTPS_REZERO_SINGLE_FINGERS_COUNT_ECALL;
		}else{
			single_finger_count = SHTPS_REZERO_SINGLE_FINGERS_COUNT;
		}
		if(ts->poll_info.single_fingers_max < single_finger_count){
			ts->poll_info.single_fingers_max = single_finger_count;
		}
	}
#else
	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_rezero_handle(struct shtps_rmi_spi *ts, u8 event, u8 gs)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	_log_msg_sync( LOGMSG_ID__REZERO_HANDLE, "%d|%d|%d", ts->poll_info.stop_margin, event, gs);
	if(!ts->poll_info.stop_margin){
		return;
	}
	
	if(event == SHTPS_REZERO_HANDLE_EVENT_MTD){
		ts->poll_info.single_fingers_enable = 0;
		
	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCH){
		if(ts->poll_info.single_fingers_enable && shtps_is_singlefinger(ts, gs)){
			ts->poll_info.single_fingers_count++;
			SHTPS_LOG_DEBUG( printk(KERN_DEBUG "single finger count = %d\n", ts->poll_info.single_fingers_count); )
		}
		
	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCHUP){
		ts->poll_info.single_fingers_enable = 1;
		if((++ts->poll_info.stop_count >= ts->poll_info.stop_margin) &&
			(ts->poll_info.single_fingers_count >= ts->poll_info.single_fingers_max))
		{
			shtps_autorezero_disable(ts);

			ts->poll_info.stop_margin          = 0;
			ts->poll_info.single_fingers_count = 0;
			ts->poll_info.single_fingers_enable= 0;
			ts->poll_info.single_fingers_max   = 0;
		}
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_reset_startuptime(struct shtps_rmi_spi *ts)
{
	ts->state_mgr.starttime = jiffies + msecs_to_jiffies(SHTPS_STARTUP_MIN_TIME);
}

static unsigned long shtps_check_startuptime(struct shtps_rmi_spi *ts)
{
	if(time_after(jiffies, ts->state_mgr.starttime)){
		return 0;
	}
	return jiffies_to_msecs(ts->state_mgr.starttime - jiffies);
}

static int shtps_start(struct shtps_rmi_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_START, 0);
}

static void shtps_shutdown(struct shtps_rmi_spi *ts)
{
	request_event(ts, SHTPS_EVENT_STOP, 0);
}

/* -----------------------------------------------------------------------------------
 */
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
static int shtps_offset_area(struct shtps_rmi_spi *ts, int x, int y)
{
	if(y < ts->offset.base[2]){
		if(x < ts->offset.base[0]){
			return 0x00;
		}else if(x < ts->offset.base[1]){
			return 0x01;
		}else{
			return 0x02;
		}
	}else if(y < ts->offset.base[3]){
		if(x < ts->offset.base[0]){
			return 0x03;
		}else if(x < ts->offset.base[1]){
			return 0x04;
		}else{
			return 0x05;
		}
	}else if(y < ts->offset.base[4]){
		if(x < ts->offset.base[0]){
			return 0x06;
		}else if(x < ts->offset.base[1]){
			return 0x07;
		}else{
			return 0x08;
		}
	}else{
		if(x < ts->offset.base[0]){
			return 0x09;
		}else if(x < ts->offset.base[1]){
			return 0x0A;
		}else{
			return 0x0B;
		}
	}
	return 0x00;
}
#endif /* #if deifned( CONFIG_SHTPS_SY3000_POSITION_OFFSET ) */

static int shtps_offset_pos(struct shtps_rmi_spi *ts, int *x, int *y)
{
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
	int area;
	int pq, rs;
	int xp, xq, xr, xs;
	int yp, yq, yr, ys;
	int base_xp, base_xq;
	int base_yp, base_yq;
	
	if(!ts->offset.enabled){
		return 0;
	}

	area = shtps_offset_area(ts, *x, *y);
	
	xp = xq = xr = xs = yp = yq = yr = ys = 0;
	if(area == 0x00){
		xq = xs = ts->offset.diff[0];
		yr = ys = ts->offset.diff[1];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x01){
		xp = xr = ts->offset.diff[0];
		xq = xs = ts->offset.diff[2];
		yr = ts->offset.diff[1];
		ys = ts->offset.diff[3];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x02){
		xq = xr = ts->offset.diff[2];
		yr = ys = ts->offset.diff[3];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x03){
		xq = ts->offset.diff[0];
		xs = ts->offset.diff[4];
		yp = yq = ts->offset.diff[1];
		yr = ys = ts->offset.diff[5];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x04){
		xp = ts->offset.diff[0];
		xq = ts->offset.diff[2];
		xr = ts->offset.diff[4];
		xs = ts->offset.diff[6];
		yp = ts->offset.diff[1];
		yq = ts->offset.diff[3];
		yr = ts->offset.diff[5];
		ys = ts->offset.diff[7];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x05){
		xp = ts->offset.diff[2];
		xr = ts->offset.diff[6];
		yp = yq = ts->offset.diff[3];
		yr = ys = ts->offset.diff[7];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x06){
		xq = ts->offset.diff[4];
		xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[5];
		yr = ys = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x07){
		xp = ts->offset.diff[4];
		xq = ts->offset.diff[6];
		xr = ts->offset.diff[8];
		xs = ts->offset.diff[10];
		yp = ts->offset.diff[5];
		yq = ts->offset.diff[7];
		yr = ts->offset.diff[9];
		ys = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x08){
		xp = ts->offset.diff[6];
		xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[7];
		yr = ys = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x09){
		xq = xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = ts->offset.diff[8];
		xq = xs = ts->offset.diff[10];
		yp = ts->offset.diff[9];
		yq = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else{
		xq = xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);
#endif /* #if deifned( CONFIG_SHTPS_SY3000_POSITION_OFFSET ) */
	return 0;
}

static int shtps_map_construct(struct shtps_rmi_spi *ts)
{
	struct rmi_pdt	pdt;
	int				i;
	int				rc;
	int				page;
	u8				maxPosition[4];
	char			productID[11];
	
	_log_msg_sync( LOGMSG_ID__MAP_CONSTRUCT, "");
	SHTPS_LOG_FUNC_CALL();

	msleep(3);
	memset(&ts->map, 0, sizeof(ts->map));
	
	/* Read the PDT */
	for(page = 0;page < 2;page++){
		shtps_rmi_write(ts, 0xFF, page);
		for(i = 0xE9;i > 0x0a;i-=sizeof(pdt)){
			rc = shtps_rmi_read(ts, ((page & 0x0f) << 8) | i, (u8*)&pdt, sizeof(pdt));
			if(rc){
				goto err_exit;
			}
			
			if(!pdt.functionNumber){
				/** End of PDT */
				break;
			}
			
			switch(pdt.functionNumber){
			case 0x01:
				SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]Found: RMI Device Control\n"); );
				ts->map.fn01.enable		= 1;
				ts->map.fn01.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn01.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn01.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn01.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn01.queryBase,
									ts->map.fn01.query.data, sizeof(ts->map.fn01.query.data));

				memcpy(productID, &F01_QUERY_PRODUCTID(ts->map.fn01.query.data), sizeof(productID));			
				productID[10] = '\0';
				SHTPS_LOG_DEBUG( 
					printk(KERN_DEBUG "[shtps]F01 Query Data\n");
					printk(KERN_DEBUG "[shtps]-------------------------------------------------\n");
					printk(KERN_DEBUG "[shtps]Manufacturer ID : 0x%02x\n", F01_QUERY_MANUFACTURERID(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]NonCompliant    : 0x%02x\n", F01_QUERY_NONCOMPLIANT(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]cutomMap        : 0x%02x\n", F01_QUERY_CUSTOMMAP(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]productInfo0    : 0x%02x\n", F01_QUERY_PRODUCTINFO0(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]productInfo1    : 0x%02x\n", F01_QUERY_PRODUCTINFO1(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]dataCodeYear    : 0x%02x\n", F01_QUERY_DATACODEYEAR(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]dataCodeMonth   : 0x%02x\n", F01_QUERY_DATACODEMONTH(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]dataCodeDay     : 0x%02x\n", F01_QUERY_DATACODEDAY(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]testID_h        : 0x%02x\n", F01_QUERY_TESTID_HI(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]testID_l        : 0x%02x\n", F01_QUERY_TESTID_LO(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]serialNum_h     : 0x%02x\n", F01_QUERY_SERIALNUM_HI(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]serialNum_l     : 0x%02x\n", F01_QUERY_SERIALNUM_LO(ts->map.fn01.query.data));
					printk(KERN_DEBUG "[shtps]Product ID      : %s\n"    , productID);
				);
				break;

			case 0x05:
				SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]Found: Image Reporting\n"); );
				ts->map.fn05.enable		= 1;
				ts->map.fn05.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn05.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn05.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn05.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn05.queryBase,
									ts->map.fn05.query.data, sizeof(ts->map.fn05.query.data));

				SHTPS_LOG_DEBUG( 
					printk(KERN_DEBUG "[shtps]F05 Query Data\n");
					printk(KERN_DEBUG "[shtps]-------------------------------------------------\n");
					printk(KERN_DEBUG "[shtps]NumberOfReceiverElectrodes    : 0x%02x\n", F05_QUERY_NUMOFRCVEL(ts->map.fn05.query.data));
					printk(KERN_DEBUG "[shtps]NumberOfTransmitterElectrodes : 0x%02x\n", F05_QUERY_NUMOFTRANSEL(ts->map.fn05.query.data));
					printk(KERN_DEBUG "[shtps]Has15bitDelta                 : 0x%02x\n", F05_QUERY_HAS16BIT(ts->map.fn05.query.data));
					printk(KERN_DEBUG "[shtps]SizeOfF05ImageWindow          : 0x%02x\n", F05_QUERY_IMAGEWINDOWSIZE(ts->map.fn05.query.data));
				);
				break;
				
			case 0x11:
				SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]Found: 2-D Sensors\n"); );
				ts->map.fn11.enable		= 1;
				ts->map.fn11.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn11.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn11.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn11.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn11.queryBase, 
									ts->map.fn11.query.data, sizeof(ts->map.fn11.query.data));
				if(rc){
					goto err_exit;
				}
				rc = shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x06, maxPosition, 4);
				if(rc){
					goto err_exit;
				}
				ts->map.fn11.ctrl.maxXPosition = maxPosition[0] | ((maxPosition[1] & 0x0F) << 0x08);
				ts->map.fn11.ctrl.maxYPosition = maxPosition[2] | ((maxPosition[3] & 0x0F) << 0x08);
				
				SHTPS_LOG_DEBUG( 
					printk(KERN_DEBUG "[shtps]F11 Query Data\n");
					printk(KERN_DEBUG "[shtps]-------------------------------------------------\n");
					printk(KERN_DEBUG "[shtps]numOfSensors    : 0x%02x\n", F11_QUERY_NUMOFSENSORS(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]configurable    : 0x%02x\n", F11_QUERY_CONFIGURABLE(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasSensAdjust   : 0x%02x\n", F11_QUERY_HASSENSADJUST(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasGestures     : 0x%02x\n", F11_QUERY_HASGESTURES(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasAbs          : 0x%02x\n", F11_QUERY_HASABS(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasRel          : 0x%02x\n", F11_QUERY_HASREL(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]numOfFingers    : 0x%02x\n", F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]numOfXElec      : 0x%02x\n", F11_QUERY_NUMOFXELEC(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]numOfYElec      : 0x%02x\n", F11_QUERY_NUMOFYELEC(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]maxElec         : 0x%02x\n", F11_QUERY_MAXELEC(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasAnchored     : 0x%02x\n", F11_QUERY_HASANCHORED(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]absDataSize     : 0x%02x\n", F11_QUERY_ABSDATASIZE(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasPinch        : 0x%02x\n", F11_QUERY_HASPINCH(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasPress        : 0x%02x\n", F11_QUERY_HASPRESS(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasFlick        : 0x%02x\n", F11_QUERY_HASFLICK(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasEarlyTap     : 0x%02x\n", F11_QUERY_HASEARLYTAP(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasDoubleTap    : 0x%02x\n", F11_QUERY_HASDOUBLETAP(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasTapHost      : 0x%02x\n", F11_QUERY_HASTAPHOST(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasSingleTap    : 0x%02x\n", F11_QUERY_HASSINGLETAP(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasRotate       : 0x%02x\n", F11_QUERY_HASROTATE(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]hasPalmDet      : 0x%02x\n", F11_QUERY_HASPALMDET(ts->map.fn11.query.data));
					printk(KERN_DEBUG "[shtps]2D MAX X POS    : 0x%04x\n", ts->map.fn11.ctrl.maxXPosition);
					printk(KERN_DEBUG "[shtps]2D MAX Y POS    : 0x%04x\n", ts->map.fn11.ctrl.maxYPosition);
				);
				break;
				
			case 0x34:
				SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]Found: Flash memory management\n"); );
				ts->map.fn34.enable		= 1;
				ts->map.fn34.queryBase = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn34.dataBase  = ((page & 0x0f) << 8) | pdt.dataBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn34.queryBase, 
									ts->map.fn34.query.data, sizeof(ts->map.fn34.query.data));
				if(rc){
					goto err_exit;
				}
				SHTPS_LOG_DEBUG( 
					printk(KERN_DEBUG "[shtps]F34 Query Data\n");
					printk(KERN_DEBUG "[shtps]-------------------------------------------------\n");
					printk(KERN_DEBUG "[shtps]bootLoaderID0       : 0x%02x\n", F34_QUERY_BOOTLOADERID0(ts->map.fn34.query.data));
					printk(KERN_DEBUG "[shtps]bootLoaderID1       : 0x%02x\n", F34_QUERY_BOOTLOADERID1(ts->map.fn34.query.data));
					printk(KERN_DEBUG "[shtps]unlocked            : 0x%02x\n", F34_QUERY_UNLOCKED(ts->map.fn34.query.data));
					printk(KERN_DEBUG "[shtps]blockSize           : 0x%04x\n", F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data));
					printk(KERN_DEBUG "[shtps]firmBlockCount      : 0x%04x\n", F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data));
					printk(KERN_DEBUG "[shtps]configBlockCount    : 0x%04x\n", F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data));
				);
				break;
			
			default:
				break;
			}
		}
	}
	shtps_rmi_write(ts, 0xFF, 0x00);
	
	if(0 == ts->map.fn01.enable){
		rc = -1;
		goto err_exit;
	}
	
	return 0;
	
err_exit:
	return rc;
}

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
static void shtps_wake_lock(struct shtps_rmi_spi *ts)
{
	if(ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH){
		SHTPS_LOG_FUNC_CALL();
		_log_msg_sync( LOGMSG_ID__FACETOUCH_WAKE_LOCK, "");
		wake_lock(&ts->facetouch.wake_lock);
	}
}

static void shtps_wake_unlock(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FACETOUCH_WAKE_UNLOCK, "");
	wake_unlock(&ts->facetouch.wake_lock);
}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

static void shtps_reset(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__HW_RESET, "");

	gpio_set_value(ts->rst_pin, 0);
	udelay(SHTPS_HWRESET_TIME_US);
	gpio_set_value(ts->rst_pin, 1);
	
	mdelay(SHTPS_HWRESET_AFTER_TIME_MS);
}

static void shtps_sleep(struct shtps_rmi_spi *ts, int on)
{
	u8 val;

	SHTPS_LOG_FUNC_CALL_INPARAM(on);
	
	_log_msg_sync( LOGMSG_ID__SET_SLEEP, "%d", on);
	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &val, 1);
	if(on){
		shtps_delayed_rezero_cancel(ts);
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val & ~0x04);
		val &= ~0x04;
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val | 0x01);
	}else{
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val | 0x04);
		val |= 0x04;
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val & 0xFC);
	}
}

static int shtps_fwdate(struct shtps_rmi_spi *ts, u8 *year, u8 *month)
{
	u8 buf[2] = { 0x00, 0x00 };
	u8 retry = 3;

	do{
		shtps_rmi_read(ts, ts->map.fn01.queryBase + 4, buf, 2);
		if(buf[0] != 0x00 && buf[1] != 0x00){
			break;
		}
	}while(retry-- > 0);
	
	*year = buf[0] & 0x1F;
	*month= buf[1] & 0x0F;
	
	_log_msg_sync( LOGMSG_ID__GET_FWDATE, "%d|%d", *year, *month);
	return 0;
}

static u16 shtps_fwver(struct shtps_rmi_spi *ts)
{
	u8 ver[2] = { 0x00, 0x00 };
	u8 retry = 3;
	
	do{
		shtps_rmi_read(ts, ts->map.fn01.queryBase + 2, ver, 2);
		if(ver[0] != 0x00 || ver[1] != 0x00){
			break;
		}
	}while(retry-- > 0);
	
	_log_msg_sync( LOGMSG_ID__GET_FWVER, "0x%04X", ((ver[0] << 0x08) & 0xff00) | ver[1]);
	return ((ver[0] << 0x08) & 0xff00) | ver[1];
}

static int shtps_init_param(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__FW_INIT, "");
	SHTPS_LOG_FUNC_CALL();
	
	rc = shtps_rmi_write(ts, ts->map.fn01.dataBase, 0x00);
	SPI_ERR_CHECK(rc, err_exit);

	if(ts->map.fn11.enable){
		u8 data;
		rc = shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x03);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
		SPI_ERR_CHECK(rc, err_exit);
		if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
			rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
			SPI_ERR_CHECK(rc, err_exit);
		}
	}
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1, 0x0F);
	SPI_ERR_CHECK(rc, err_exit);
	
	shtps_set_palmthresh(ts, SHTPS_LOS_SINGLE);

	#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x84);
	#else
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x80);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
	SPI_ERR_CHECK(rc, err_exit);

#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	{
		u8  val;
		_log_msg_sync( LOGMSG_ID__AUTO_REZERO_ENABLE, "");
		shtps_rmi_read(ts, 0xF0, &val, 1);
		shtps_rmi_write(ts, 0xF0, val | 0x01);
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
	
	return 0;

err_exit:
	return -1;
}

static void shtps_standby_param(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__FW_STANDBY, "");
	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x01);
	shtps_set_palmthresh(ts, 0);

#if 0
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	while(retry-- > 0){
		shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
		if((buf[1] & 0x0F) == 0x00){
			break;
		}
		udelay(1000);
	}
#endif
}

static void shtps_clr_startup_err(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->state_mgr.starterr = SHTPS_STARTUP_SUCCESS;
}

static int shtps_wait_startup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_COMP_WAIT, "");
	wait_event_interruptible(ts->wait_start, 
		ts->state_mgr.state == SHTPS_STATE_ACTIVE          ||
		ts->state_mgr.state == SHTPS_STATE_BOOTLOADER      ||
		ts->state_mgr.state == SHTPS_STATE_FWTESTMODE      ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP           ||
		ts->state_mgr.state == SHTPS_STATE_FACETOUCH       ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH ||
		ts->state_mgr.starterr == SHTPS_STARTUP_FAILED);

	_log_msg_recv( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);

	return ts->state_mgr.starterr;
}

static void shtps_notify_startup(struct shtps_rmi_spi *ts, u8 err)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(err);
	ts->state_mgr.starterr = err;
	_log_msg_send( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);
	wake_up_interruptible(&ts->wait_start);
}

static void shtps_set_startmode(struct shtps_rmi_spi *ts, u8 mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_MODE, "%d", mode);
	ts->state_mgr.mode = mode;
}

static int shtps_get_startmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->state_mgr.mode;
}

static int shtps_get_facetouchmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->facetouch.mode;
}

static void shtps_set_facetouchmode(struct shtps_rmi_spi *ts, int mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__SET_FACETOUCH_MODE, "%d", mode);
	ts->facetouch.mode = mode;
	if(mode == 0){
		ts->facetouch.off_detect = 0;
	}
}

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
static void shtps_notify_facetouchoff(struct shtps_rmi_spi *ts, int force)
{
	ts->facetouch.state = 0;
	ts->facetouch.off_detect = 1;
	_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
	wake_up_interruptible(&ts->facetouch.wait_off);
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "face touch off detect. wake_up()\n"); )
}

static void shtps_check_facetouch(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state == 0x01){
			if(info->fingers[i].wx >= 15){
				ts->facetouch.state = 1;
				break;
			}
		}
	}
}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

/* -----------------------------------------------------------------------------------
 */
static int shtps_get_diff(unsigned short pos1, unsigned short pos2, unsigned long factor)
{
	int diff = (pos1 * factor / 10000) - (pos2 * factor / 10000);
	return (diff >= 0)? diff : -diff;
}

static void shtps_rec_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
	ts->touch_state.drag_timeout[index][xy] = jiffies + msecs_to_jiffies(SHTPS_DRAG_THRESH_RETURN_TIME);
}

static int shtps_chk_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
	if(time_after(jiffies, ts->touch_state.drag_timeout[index][xy])){
		return -1;
	}
	return 0;
}

static int shtps_get_fingerwidth(struct shtps_rmi_spi *ts, int num, struct shtps_touch_info *info)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	int w;
	
	if(info->gs2 & 0x01 || (shtps_get_facetouchmode(ts) && info->finger_num >= 2)){
		w = SHTPS_FINGER_WIDTH_PALMDET;
	}else{
		w = (info->fingers[num].wx >= info->fingers[num].wy)? info->fingers[num].wx : info->fingers[num].wy;
		if(w < SHTPS_FINGER_WIDTH_MIN){
			w = SHTPS_FINGER_WIDTH_MIN;
		}else if(w > SHTPS_FINGER_WIDTH_MAX){
			w = SHTPS_FINGER_WIDTH_MAX;
		}
	}
	return w;
#else  /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	int w = (info->fingers[num].wx >= info->fingers[num].wy)? info->fingers[num].wx : info->fingers[num].wy;
	return (w < SHTPS_FINGER_WIDTH_MIN)? SHTPS_FINGER_WIDTH_MIN : w;
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_get_dragstep(struct shtps_rmi_spi *ts, int xy, int type, int fingers)
{
	int dragStep;
	
	if(type == SHTPS_DRAG_THRESHOLD_1ST){
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_1ST : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_1ST : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
		}
	}else{
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_2ND : SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_2ND : SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI;
		}
	}
	
	return dragStep;
}

static void shtps_set_dragstep(struct shtps_rmi_spi *ts, 
		struct shtps_touch_info *info, int type, int xy, int finger)
{

	_log_msg_sync( LOGMSG_ID__SET_DRAG_STEP, "%d|%d|%d", type, xy, finger);
	if(type == SHTPS_DRAG_THRESHOLD_1ST){
		ts->touch_state.dragStep[finger][xy] = type;
	}else{
		if(xy == SHTPS_POSTYPE_X){
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy, 
								info->fingers[finger].x * SHTPS_POS_SCALE_X(ts) / 10000, info->fingers[finger].x);
			ts->center_info.fingers[finger].x = info->fingers[finger].x;
		}else{
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy, 
								info->fingers[finger].y * SHTPS_POS_SCALE_Y(ts) / 10000, info->fingers[finger].y);
			ts->center_info.fingers[finger].y = info->fingers[finger].y;
		}
		ts->touch_state.dragStep[finger][xy] = type;
		shtps_rec_notify_time(ts, xy, finger);
	}
}

static void shtps_calc_notify(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info, u8 *event)
{
	int		i;
	u8 		fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;
	u8		numOfFingers = 0;
	u8*		fingerInfo;
	u8		base;
	int 	diff_x;
	int 	diff_y;
	int 	diff_cx;
	int 	diff_cy;
	int		dragStep1stX;
	int		dragStep1stY;
	int		dragStepCurX;
	int		dragStepCurY;

#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	u8		isVKeyOn = 0;
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */

	SHTPS_LOG_FUNC_CALL();

	*event = 0xff;

	base = (fingerMax > 4)? 2 : 1;
	for(i = 0;i < fingerMax;i++){
		info->fingers[i].state = (buf[i / 4] >> (2 * (i % 4))) & 0x03;
		if(info->fingers[i].state == 0x01){
			numOfFingers++;
		}
	}
	info->finger_num = numOfFingers;
	
	dragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers);
	dragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers);
	for(i = 0;i < fingerMax;i++){
		fingerInfo = &buf[base + i * 5];
		info->fingers[i].x 		= F11_DATA_XPOS(fingerInfo);
		info->fingers[i].y 		= F11_DATA_YPOS(fingerInfo);
		info->fingers[i].wx 	= F11_DATA_WX(fingerInfo);
		info->fingers[i].wy 	= F11_DATA_WY(fingerInfo);
		info->fingers[i].z 		= F11_DATA_Z(fingerInfo);

		_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state, 
							info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000, info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[i].x, info->fingers[i].y,info->fingers[i].wx, info->fingers[i].wy, info->fingers[i].z);
		
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
		if((info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000) >= CONFIG_SHTPS_SY3000_LCD_SIZE_Y){
			isVKeyOn = 1;
			if(ts->vkey_invalid){
				info->fingers[i].state = 0;
				SHTPS_LOG_DEBUG( printk("[shtps][test] vkey is invalid. finger[%d].state <= 0\n", i); )
			}
		}
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
		
		dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers);
		dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers);

		if(info->fingers[i].state == 0x01){
			diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
			diff_cx= shtps_get_diff(info->fingers[i].x, ts->center_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_cy= shtps_get_diff(info->fingers[i].y, ts->center_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

			if(ts->report_info.fingers[i].state == 0x01){
				if(diff_cy >= dragStep1stY){
					if(ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);
						dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, 
											ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers);
					}
				}
				
				if(diff_x >= dragStepCurX){
					if(diff_cx >= dragStep1stX){
						*event = SHTPS_EVENT_DRAG;
						if(ts->touch_state.dragStep[i][0] != SHTPS_DRAG_THRESHOLD_2ND){
							shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);
							dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, 
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers);
						}
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);

					}else if(shtps_chk_notify_time(ts, 0, i) == 0){
						*event = SHTPS_EVENT_DRAG;

					}else{
						info->fingers[i].x = ts->report_info.fingers[i].x;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
					}
				}else{
					info->fingers[i].x = ts->report_info.fingers[i].x;
				}
				
				if(diff_y >= dragStepCurY){
					if(diff_cy >= dragStep1stY){
						*event = SHTPS_EVENT_DRAG;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);

					}else if(shtps_chk_notify_time(ts, 1, i) == 0 ||
								ts->touch_state.dragStep[i][1] == SHTPS_DRAG_THRESHOLD_1ST)
					{
						*event = SHTPS_EVENT_DRAG;
						ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_2ND;

					}else{
						info->fingers[i].y = ts->report_info.fingers[i].y;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
					}
				}else{
					info->fingers[i].y = ts->report_info.fingers[i].y;
				}
			}else{
				ts->center_info.fingers[i].x = info->fingers[i].x;
				ts->center_info.fingers[i].y = info->fingers[i].y;
			}
		}else{
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
		}
		
		if(info->fingers[i].state != ts->report_info.fingers[i].state){
			*event = SHTPS_EVENT_MTDU;
		}
	}
	
	if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) == 0){
		info->gs1 		= 0;
		info->gs2 		= buf[base + fingerMax * 5];
	}else{
		info->gs1 		= buf[base + fingerMax * 5];
		info->gs2 		= buf[base + fingerMax * 5 + 1];
	}

#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	if(isVKeyOn == 0 && ts->vkey_invalid == 1){
		SHTPS_LOG_DEBUG( printk("[shtps][test] vkey invalid flag OFF\n"); )
		ts->vkey_invalid = 0;
	}
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */

	if(numOfFingers > 0){
		ts->poll_info.stop_count = 0;
		if(ts->touch_state.numOfFingers == 0){
			*event = SHTPS_EVENT_TD;
		}
		if(numOfFingers >= 2 && ts->touch_state.numOfFingers < 2){
			if(shtps_get_facetouchmode(ts)){
				shtps_set_palmthresh(ts, SHTPS_LOS_MULTI);
			}
			shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_MTD, info->gs2);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCH, info->gs2);
	}else{
		if(ts->touch_state.numOfFingers != 0){
			*event = SHTPS_EVENT_TU;
			shtps_set_palmthresh(ts, SHTPS_LOS_SINGLE);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCHUP, info->gs2);
	}
}

static void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event)
{
	int	i;
	int x;
	int y;
	int width;
	u8	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;

	SHTPS_LOG_FUNC_CALL();

#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	shtps_wait_notify_interval(ts);
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */

	for(i = 0;i < fingerMax;i++){
		x = info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		y = info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		shtps_offset_pos(ts, &x, &y);
		
		if(info->fingers[i].state == 0x01){
			width = shtps_get_fingerwidth(ts, i, info);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, width);
			
			SHTPS_LOG_DEBUG(printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100, x=%d(%d), y=%d(%d)\n",
								i,
								x, (int)(info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000),
								y, (int)(info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000)); );
			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
								i,
								info->fingers[i].z,
								x, info->fingers[i].x,
								y, info->fingers[i].y,
								width, info->fingers[i].wx, info->fingers[i].wy); );
			_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|100|%d|%d|%d|%d|%d|%d|%d|%d",
								i,
								x, info->fingers[i].x,
								y, info->fingers[i].y,
								width, info->fingers[i].wx, info->fingers[i].wy, info->fingers[i].z);
			input_mt_sync(ts->input);
			
		}else if(ts->report_info.fingers[i].state != 0x00){
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
							 
			SHTPS_LOG_DEBUG(printk(KERN_DEBUG "[shtps]Notify event[%d] touch=  0, x=%d(%d), y=%d(%d)\n",
								i,
								x, (int)(info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000),
								y, (int)(info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000)); );
			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=  0(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
								i,
								info->fingers[i].z,
								x, info->fingers[i].x,
								y, info->fingers[i].y,
								0, info->fingers[i].wx, info->fingers[i].wy); );
			_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|0|%d|%d|%d|%d|%d|%d|%d|%d",
								i,
								x, info->fingers[i].x,
								y, info->fingers[i].y,
								width, info->fingers[i].wx, info->fingers[i].wy, info->fingers[i].z);
			input_mt_sync(ts->input);
		}
	}
	input_sync(ts->input);
#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	shtps_set_notify_time(ts);
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */

	ts->touch_state.numOfFingers = info->finger_num;
	
	ts->diag.event = 1;
	memcpy(&ts->report_info, info, sizeof(ts->report_info));
	wake_up_interruptible(&ts->diag.wait);
}

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
static void shtps_event_update(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int	i;
	u8	numOfFingers = 0;
	u8	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state == 0x01){
			numOfFingers++;
		}
	}
	ts->touch_state.numOfFingers = numOfFingers;
}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

static void shtps_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int	i;
	int x;
	int y;
	int isEvent = 0;
	u8	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		x = ts->report_info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		y = ts->report_info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		shtps_offset_pos(ts, &x, &y);
		
		if(ts->report_info.fingers[i].state != 0x00){
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
							 
			SHTPS_LOG_DEBUG(printk(KERN_DEBUG "[shtps]Notify event[%d] touch=  0, x=%d(%d), y=%d(%d)\n",
								i,
								x, (int)(ts->report_info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000),
								y, (int)(ts->report_info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000)); );
			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=  0, x=%d, y=%d w=%d\n",
								i,
								x,
								y,
								0); );
			_log_msg_sync( LOGMSG_ID__EVENT_FORCE_TU, "%d|0|%d|%d",i , x, y);
			input_mt_sync(ts->input);
			isEvent = 1;
		}
	}
	if(isEvent){
		input_sync(ts->input);
		ts->touch_state.numOfFingers = 0;
		memset(&ts->report_info, 0, sizeof(ts->report_info));
	}
	
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	SHTPS_LOG_DEBUG( printk("[shtps][test] vkey invalid flag OFF\n"); )
	ts->vkey_invalid = 0;
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_tm_irqcheck(struct shtps_rmi_spi *ts)
{
	u8 buf[2];
	
	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	
	if((buf[1] & 0x08) != 0x00){
		return 1;
	}
	return 0;
}

static int shtps_tm_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;
	
	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_WAIT, "");
	rc = wait_event_interruptible_timeout(ts->diag.tm_wait_ack, 
			ts->diag.tm_ack == 1 || ts->diag.tm_stop == 1, 
			msecs_to_jiffies(SHTPS_FWTESTMODE_ACK_TMO));
	_log_msg_recv( LOGMSG_ID__FW_TESTMODE_ATTN, "");

#if defined( SHTPS_LOG_SEQ_ENABLE )
	if(rc == 0){
		_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_TIMEOUT, "");
	}
#endif /* #if defined( SHTPS_LOG_SEQ_ENABLE ) */

	if(ts->diag.tm_ack == 0 || ts->diag.tm_stop == 1){
		return -1;
	}
	
	ts->diag.tm_ack = 0;
	return 0;
}

static void shtps_tm_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_ack = 1;
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static void shtps_tm_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_stop = 1;
	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_CANCEL, "");
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static int shtps_get_tm_rxsize(struct shtps_rmi_spi *ts)
{
	int receive_num = F05_QUERY_NUMOFRCVEL(ts->map.fn05.query.data);
	return (receive_num > SHTPS_TM_TXNUM_MAX)? SHTPS_TM_TXNUM_MAX : receive_num;
}

static int shtps_get_tm_txsize(struct shtps_rmi_spi *ts)
{
	int trans_num   = F05_QUERY_NUMOFTRANSEL(ts->map.fn05.query.data);
	return (trans_num > SHTPS_TM_RXNUM_MAX)? SHTPS_TM_RXNUM_MAX : trans_num;
}

static int shtps_start_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_START, "%d", ts->diag.tm_mode);
	ts->diag.tm_stop = 0;
	if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V01){
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase,      0x04);
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1,  0x00);
		shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
		if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
		}
		shtps_rmi_write(ts, ts->map.fn01.dataBase,      0x42);
		shtps_rmi_write(ts, ts->map.fn01.dataBase,      0xe1);
		
		shtps_rmi_write(ts, 0xff, 0x80);
		shtps_rmi_write(ts, 0x00, 0x01);

	}else{
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase,      0x04);
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1,  0x08);
		shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
		if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
		}
		shtps_rmi_write(ts, 0xff, 0x01);
	}
	return 0;
}

static void shtps_stop_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_STOP, "%d", ts->diag.tm_mode);
	shtps_tm_cancel(ts);
	shtps_rmi_write(ts, 0xff, 0x00);
	shtps_init_param(ts);
}

static void shtps_read_tmdata(struct shtps_rmi_spi *ts, u8 mode)
{
	int i;
	int j;
	int receive_num = shtps_get_tm_rxsize(ts);
	int trans_num   = shtps_get_tm_txsize(ts);
	u8  tmp[SHTPS_TM_TXNUM_MAX * 2];

	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_GETDATA, "%d", ts->diag.tm_mode);
	
	if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V01){
		mutex_lock(&shtps_ctrl_lock);
		if(mode == SHTPS_TMMODE_FRAMELINE){
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, 0x0201, (0x80 | (i & 0x3f)));
				shtps_rmi_read(ts, 0x0202, tmp, receive_num);

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num) + i] = tmp[j];
				}
			}
		}else if(mode == SHTPS_TMMODE_BASELINE){
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, 0x0201, (0x40 | (i & 0x3f)));
				shtps_rmi_read(ts, 0x0202, tmp, receive_num * 2);

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[j * 2 + 1];
				}
			}
		}
		mutex_unlock(&shtps_ctrl_lock);
	}else{
		if(mode == SHTPS_TMMODE_FRAMELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, 0x80);
			shtps_rmi_write(ts, ts->map.fn05.commandBase, 0x04);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;
			
			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, (0x80 | (i & 0x3f)));
				shtps_rmi_read(ts, ts->map.fn05.dataBase + 2, tmp, receive_num);
				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num) + i] = tmp[j];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}else if(mode == SHTPS_TMMODE_BASELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, 0x40);
			shtps_rmi_write(ts, ts->map.fn05.commandBase, 0x04);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, (0x40 | (i & 0x3f)));
				shtps_rmi_read(ts, ts->map.fn05.dataBase + 2, tmp, receive_num * 2);
				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[j * 2 + 1];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}
	}
	return;
	
tm_read_cancel:
	memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	return;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_irq_wake_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_DISABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
		disable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;
	}
}

static void shtps_irq_wake_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_ENABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
		enable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;
	}
}

static void shtps_irq_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_DISABLE, "%d", ts->irq_mgr.state);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_DISABLE){
		disable_irq(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_DISABLE;
	}
}

static void shtps_irq_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_ENABLE, "%d", ts->irq_mgr.state);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_ENABLE){
		enable_irq(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	}
}

static int shtps_irq_resuest(struct shtps_rmi_spi *ts)
{
	int rc;
	
	SHTPS_LOG_FUNC_CALL();
	
	_log_msg_sync( LOGMSG_ID__IRQ_REQUEST, "%d", ts->irq_mgr.irq);
	set_irq_type(ts->irq_mgr.irq, IRQ_TYPE_EDGE_FALLING);
	rc = request_irq(ts->irq_mgr.irq,
					 shtps_irq,
					 IRQF_TRIGGER_FALLING | IRQF_DISABLED,
					 SH_TOUCH_DEVNAME,
					 ts);
	if(rc){
		_log_msg_sync( LOGMSG_ID__IRQ_REQUEST_NACK, "");
		printk(KERN_WARNING "shtps:request_irq error\n");
		return -1;
	}
	
	ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	ts->irq_mgr.wake  = SHTPS_IRQ_WAKE_DISABLE;
	shtps_irq_disable(ts);
	return 0;
}

static void shtps_irqtimer_start(struct shtps_rmi_spi *ts, long time_ms)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_START, "%lu", time_ms);
	schedule_delayed_work(&ts->tmo_check, msecs_to_jiffies(time_ms));
}

static void shtps_irqtimer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_CANCEL, "");
	cancel_delayed_work(&ts->tmo_check);
}

static void shtps_polling_start(struct shtps_rmi_spi *ts)
{
	extern	void sh_spi_tps_active(struct spi_device *spi);
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__POLLING_TIMER_START, "%d", SHTPS_EVENT_POLL_INTERVAL);
	hrtimer_cancel(&ts->polling_timer);

#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	shtps_wait_poll_interval(ts);
	shtps_set_pollstart_time(ts);
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */

	hrtimer_start(&ts->polling_timer, ktime_set(0, SHTPS_EVENT_POLL_INTERVAL * 1000 * 1000), HRTIMER_MODE_REL);
	sh_spi_tps_active(ts->spi);
}

static void shtps_polling_stop(struct shtps_rmi_spi *ts)
{
	extern	void sh_spi_tps_standby(struct spi_device *spi);
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__POLLING_TIMER_CANCEL, "");
	hrtimer_try_to_cancel(&ts->polling_timer);

	sh_spi_tps_standby(ts->spi);
}

static void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state)
{
	u8 buf[2 + SHTPS_FINGER_MAX * 5 + (SHTPS_FINGER_MAX / 4) + 2];
	u8 event;
	u8 finger;
	u8 fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;
	u8 base;
	struct shtps_touch_info info;
	
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__READ_EVENT, "%d", state);

#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
	ts->invalid_area_touch = 0;
#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */

	memset(buf, 0, sizeof(buf));
	if(fingerMax > 4){
		shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 4);
		base = 4;
		ts->finger_state[0] = buf[2];
		ts->finger_state[1] = buf[3];
	}else{
		shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 3);
		base = 3;
		ts->finger_state[0] = buf[2];
		ts->finger_state[1] = 0x00;
	}
	
	for(finger = 0;finger < fingerMax;finger++){
		if(((buf[2 + finger / 4] >> (2 * (finger % 4))) & 0x03) == 0x01 ||
		   ((ts->finger_state[finger / 4] >> (2 * (finger % 4))) & 0x03) == 0x01)
		{
			shtps_rmi_read(ts, 
						   ts->map.fn01.dataBase + base + (finger * 5), 
						   &buf[base + (finger * 5)], 5);

			#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
			{
				int x, y, idx;
				x = (((buf[base + (finger * 5) + 0] << 0x04) & 0x0FF0) | (buf[base + (finger * 5) + 2] & 0x0F));
				y = (((buf[base + (finger * 5) + 1] << 0x04) & 0x0FF0) | (buf[base + (finger * 5) + 2] >> 0x04 & 0x0F));
				x = x * SHTPS_POS_SCALE_X(ts) / 10000;
				y = y * SHTPS_POS_SCALE_Y(ts) / 10000;
				for(idx = 0;shtps_invalid_area[idx].sx != -1;idx++){
					if(shtps_invalid_area[idx].sx <= x && x <= shtps_invalid_area[idx].ex &&
					   shtps_invalid_area[idx].sy <= y && y <= shtps_invalid_area[idx].ey)
					{
						_log_msg_sync( LOGMSG_ID__INVALID_AREA, "%d|%d|%d", finger, x, y);
						buf[2 + finger / 4] &= ~(0x03 << (2 * (finger % 4)));
						ts->invalid_area_touch = 1;
					}
				}
			}
			#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */
		}
	}
	
	if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) == 0){
		shtps_rmi_read(ts, ts->map.fn01.dataBase + base + fingerMax * 5, 
					   &buf[base + fingerMax * 5], 1);
	}else{
		shtps_rmi_read(ts, ts->map.fn01.dataBase + base + fingerMax * 5, 
					   &buf[base + fingerMax * 5], 2);
	}

	switch(state){
	case SHTPS_STATE_SLEEP:
	case SHTPS_STATE_IDLE:
		break;
	
	case SHTPS_STATE_SLEEP_FACETOUCH:
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_calc_notify(ts, &buf[2], &info, &event);
		shtps_event_update(ts, &info);
		if(event == SHTPS_EVENT_TU){
			shtps_notify_facetouchoff(ts, 1);
		}else{
			shtps_wake_unlock(ts);
		}
		if(ts->touch_state.numOfFingers != 0){
			shtps_polling_start(ts);
		}else{
			shtps_polling_stop(ts);
		}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
		break;

	case SHTPS_STATE_FACETOUCH:
	case SHTPS_STATE_ACTIVE:
	default:
		shtps_calc_notify(ts, &buf[2], &info, &event);
		if(event != 0xff){
			shtps_event_report(ts, &info, event);
			
			#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			if(SHTPS_STATE_FACETOUCH == state){
				if(event == SHTPS_EVENT_TU){
					shtps_notify_facetouchoff(ts, 0);
				}else{
					shtps_check_facetouch(ts, &info);
				}
			}
			#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
		}

		if(ts->touch_state.numOfFingers != 0 || 
			(ts->poll_info.stop_count < ts->poll_info.stop_margin))
		{
			shtps_polling_start(ts);
		}else{
			#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
			if(ts->invalid_area_touch){
				shtps_polling_start(ts);
				break;
			}
			#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */
			shtps_polling_stop(ts);
		}
		break;
	}
}

static void shtps_loader_irqclr(struct shtps_rmi_spi *ts)
{
	u8 buf[2];
	
	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	_log_msg_sync( LOGMSG_ID__BL_IRQCLR, "0x%02X", buf[1]);
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_touchpanel_enable(struct shtps_rmi_spi *ts)
{
	u8 buf;
	shtps_rmi_write(ts, 0xFF, 0x00);
	shtps_rmi_read(ts, 0xEE, &buf, 1);
	
	_log_msg_sync( LOGMSG_ID__CHECK_TOUCHPANEL, "%d", (buf != 0x00)? 1:0);
	return (buf != 0x00);
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts)
{
	return 1;
}

static int shtps_loader_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;
	
	_log_msg_sync( LOGMSG_ID__BL_ATTN_WAIT, "");
	rc = wait_event_interruptible_timeout(ts->loader.wait_ack, 
			ts->loader.ack == 1, 
			msecs_to_jiffies(SHTPS_BOOTLOADER_ACK_TMO));

	_log_msg_recv( LOGMSG_ID__BL_ATTN, "");

	if(0 == rc && 0 == ts->loader.ack){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_ERROR, "");
		return -1;
	}
	
	if(0 == rc){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_TIMEOUT, "");
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[shtps]shtps_loader_wait_attn() warning rc = %d\n", rc); );
	}
	
	ts->loader.ack = 0;
	return 0;
}

static void shtps_loader_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->loader.ack = 1;
	_log_msg_send( LOGMSG_ID__BL_ATTN, "");
	wake_up_interruptible(&ts->loader.wait_ack);
}

static int shtps_loader_cmd(struct shtps_rmi_spi *ts, u8 cmd, u8 isLockdown)
{
	int rc;
	u8  buf;
	u16 blockSize;
	
	_log_msg_sync( LOGMSG_ID__BL_COMMAND, "0x%02X|%d", cmd, isLockdown);
	if(isLockdown){
		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2,
						 	 F34_QUERY_BOOTLOADERID0(ts->map.fn34.query.data));
		SPI_ERR_CHECK(rc, err_exit);
		
		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 3,
				 			 F34_QUERY_BOOTLOADERID1(ts->map.fn34.query.data));
		SPI_ERR_CHECK(rc, err_exit);
	}
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
	
	rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize, &buf, 1);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + blockSize, (buf & 0xF0) | (cmd & 0x0F));
	SPI_ERR_CHECK(rc, err_exit);

#if 0	
#if defined( CONFIG_SHTPS_SY3000_TM1963_001 )
	udelay(SHTPS_LOADERCMD_WAIT_US);
#endif /* #if defined( CONFIG_SHTPS_SY3000_TM1963_001 ) */
#endif

	return 0;

err_exit:
	return rc;
	
}

static int shtps_enter_bootloader(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__BL_ENTER, "");
	
	mutex_lock(&shtps_loader_lock);
	
	request_event(ts, SHTPS_EVENT_STOP, 0);
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	if(request_event(ts, SHTPS_EVENT_STARTLOADER, 0) != 0){
		return -1;
	}
	shtps_wait_startup(ts);
	
	shtps_sleep(ts, 1);
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	
	shtps_loader_cmd(ts, 0x0F, 1);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERROR( printk(KERN_ERR "[shtps]shtps_enter_bootloader() mode change error\n"); );
		goto err_exit;
	}
	shtps_map_construct(ts);

	_log_msg_sync( LOGMSG_ID__BL_ENTER_DONE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_enter_bootloader() done\n"); );
	mutex_unlock(&shtps_loader_lock);
	return 0;
	
err_exit:
	_log_msg_sync( LOGMSG_ID__BL_ENTER_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_exit_bootloader(struct shtps_rmi_spi *ts)
{
	u8  status;
	
	_log_msg_sync( LOGMSG_ID__BL_EXIT, "");
	shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01);
	msleep(SHTPS_RESET_BOOTLOADER_WAIT_MS);
	
	shtps_rmi_read(ts, ts->map.fn01.dataBase, &status, 1);
	request_event(ts, SHTPS_EVENT_STOP, 0);

	if((status & 0x40) != 0 || (status & 0x0F) == 4 || (status & 0x0F) == 5 || (status & 0x0F) == 6){
		SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_exit_bootloader() error status = 0x%02x\n", status); );
		_log_msg_sync( LOGMSG_ID__BL_EXIT_FAIL, "0x%02X", status);
		return -1;
	}
	
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_exit_bootloader() done\n"); );
	_log_msg_sync( LOGMSG_ID__BL_EXIT_DONE, "");
	return 0;
}

static int shtps_lockdown_bootloader(struct shtps_rmi_spi *ts, u8* fwdata)
{
	return 0;
}

static int shtps_flash_erase(struct shtps_rmi_spi *ts)
{
	int rc;
	u8  status;
	u16 blockSize;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}
	

	_log_msg_sync( LOGMSG_ID__BL_ERASE, "");

	mutex_lock(&shtps_loader_lock);

	shtps_loader_cmd(ts, 0x03, 1);
	msleep(1000);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[shtps]shtps_loader_wait_attn() err %d\n", rc); );
		goto err_exit;
	}

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
	rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize,
						&status, 1);
	SPI_ERR_CHECK((rc || status != 0x80), err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase, 0);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 1, 0);
	SPI_ERR_CHECK(rc, err_exit);

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_flash_erase() done\n"); );
	_log_msg_sync( LOGMSG_ID__BL_ERASE_DONE, "");
	mutex_unlock(&shtps_loader_lock);
	return 0;
	
err_exit:
	_log_msg_sync( LOGMSG_ID__BL_ERASE_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_flash_writeImage(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	int i;
	u16 blockNum;
	u16 blockSize;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE, "");

	mutex_lock(&shtps_loader_lock);
	
	blockNum  = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	for(i = 0;i < blockSize;i++){
		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + i, fwdata[i]);
		SPI_ERR_CHECK(rc, err_exit);
	}
	rc = shtps_loader_cmd(ts, 0x02, 0);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		goto err_exit;
	}

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_flash_writeImage() done\n"); );
	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE_DONE, "");
	mutex_unlock(&shtps_loader_lock);
	return 0;
	
err_exit:
	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_flash_writeConfig(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	int i;
	int block;
	u16 blockNum;
	u16 blockSize;
	u8  status;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG, "");

	mutex_lock(&shtps_loader_lock);
	
	blockNum  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase, 0);
	SPI_ERR_CHECK(rc, err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 1, 0);
	SPI_ERR_CHECK(rc, err_exit);
	
	for(block = 0;block < blockNum;block++){
		for(i = 0;i < blockSize;i++){
			rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + i, 
								 fwdata[(block * blockSize) + i]);
			SPI_ERR_CHECK(rc, err_exit);
		}
		rc = shtps_loader_cmd(ts, 0x06, 0);
		SPI_ERR_CHECK(rc, err_exit);
		
		rc = shtps_loader_wait_attn(ts);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize,
							&status, 1);
		SPI_ERR_CHECK((rc || status != 0x80), err_exit);
	}
	
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_flash_writeConfig() done\n"); );
	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG_DONE, "");
	rc = shtps_exit_bootloader(ts);
	SPI_ERR_CHECK(rc, err_exit);
	
	mutex_unlock(&shtps_loader_lock);
	return 0;
	
err_exit:
	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

#if defined( SHTPS_OWN_THREAD_ENABLE )
static void shtps_func_api_request_from_irq_complete(void *arg_p)
{
	kfree( arg_p );
}

static void shtps_func_api_request_from_irq( struct shtps_rmi_spi *ts, int event, int param)
{
	struct shtps_req_msg		*msg_p;
	struct shtps_req_buf		*buf_p;
	unsigned long	flags;

	msg_p = (struct shtps_req_msg *)kzalloc( sizeof( struct shtps_req_msg ) + sizeof( struct shtps_req_buf ) , GFP_KERNEL );
	if ( msg_p == NULL ){
		SHTPS_LOG_ERROR( printk(KERN_ERR "[shtps]Out of memory(TPS)\n"); );
		return;
	}
	buf_p = (struct shtps_req_buf *)( &(msg_p[1]) );

	INIT_LIST_HEAD( &(msg_p->request) );

	buf_p->event = event;
	buf_p->param = param;
	list_add_tail( &(buf_p->request_list), &(msg_p->request));

	msg_p->complete = shtps_func_api_request_from_irq_complete;
	msg_p->context = msg_p;
	msg_p->status0 = -EINPROGRESS;
	msg_p->status1 = 0;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg_p->queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );
}

static void shtps_func_workq( struct work_struct *work_p )
{
	struct shtps_rmi_spi	*ts;
	unsigned long			status_error;
	unsigned long			flags;

	ts = container_of(work_p, struct shtps_rmi_spi, work_data);
	status_error = 0;

	if(0){
		status_error = 1;
	}

	while( list_empty( &(ts->queue) ) == 0 ){
		spin_lock_irqsave( &(ts->queue_lock), flags );	/* Lock for queue protect */
		ts->cur_msg_p = list_entry( ts->queue.next, struct shtps_req_msg, queue);
		list_del_init( &(ts->cur_msg_p->queue) );
		spin_unlock_irqrestore( &(ts->queue_lock), flags );	/* Unlock for queue protect */

		if(status_error != 0){
			ts->cur_msg_p->status0 = (-EIO);

		}else{
			ts->cur_msg_p->status0 = 0;
			ts->cur_msg_p->status1 = 0;
			list_for_each_entry(ts->cur_request_p, &(ts->cur_msg_p->request), request_list){
				ts->cur_request_p->result = (int)request_event(ts, ts->cur_request_p->event, ts->cur_request_p->param);
				if(ts->cur_request_p->result != 0){
					ts->cur_msg_p->status0 = (-EIO);
				}
			}
		}
		if( ts->cur_msg_p->complete ){
			if((status_error == 0) && (ts->cur_msg_p->status0 != 0)){
				/* Error proc */
			}
			ts->cur_msg_p->complete( ts->cur_msg_p->context );
		}
	}
}
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_rmi_spi *ts, int event, int param)
{
	int ret;

	_log_msg_sync( LOGMSG_ID__REQUEST_EVENT, "%d|%d", event, ts->state_mgr.state);
	
	SHTPS_LOG_DEBUG( printk(KERN_ERR "[shtps]event %d in state %d\n", event, ts->state_mgr.state); );
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_lock()\n"); );
	
	mutex_lock(&shtps_ctrl_lock);
	
	switch(event){
	case SHTPS_EVENT_START:
		ret = state_func_tbl[ts->state_mgr.state]->start(ts, param);
		break;
	case SHTPS_EVENT_STOP:
		ret = state_func_tbl[ts->state_mgr.state]->stop(ts, param);
		break;
	case SHTPS_EVENT_SLEEP:
		ret = state_func_tbl[ts->state_mgr.state]->sleep(ts, param);
		break;
	case SHTPS_EVENT_WAKEUP:
		ret = state_func_tbl[ts->state_mgr.state]->wakeup(ts, param);
		break;
	case SHTPS_EVENT_STARTLOADER:
		ret = state_func_tbl[ts->state_mgr.state]->start_ldr(ts, param);
		break;
	case SHTPS_EVENT_STARTTM:
		ret = state_func_tbl[ts->state_mgr.state]->start_tm(ts, param);
		break;
	case SHTPS_EVENT_STOPTM:
		ret = state_func_tbl[ts->state_mgr.state]->stop_tm(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_ON:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_on(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_OFF:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_off(ts, param);
		break;
	case SHTPS_EVENT_GETSENSOR:
		ret = state_func_tbl[ts->state_mgr.state]->get_sensor(ts, param);
		break;
	case SHTPS_EVENT_INTERRUPT:
		ret = state_func_tbl[ts->state_mgr.state]->interrupt(ts, param);
		break;
	case SHTPS_EVENT_TIMEOUT:
		ret = state_func_tbl[ts->state_mgr.state]->timeout(ts, param);
		break;
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	case SHTPS_EVENT_FORMCHANGE:
		ret = state_func_tbl[ts->state_mgr.state]->form_change(ts, param);
		break;
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
	default:
		ret = -1;
		break;
	}

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_unlock()\n"); );
	mutex_unlock(&shtps_ctrl_lock);
	
	return ret;
}

static int state_change(struct shtps_rmi_spi *ts, int state)
{
	int ret = 0;
	int old_state = ts->state_mgr.state;
	
	_log_msg_sync( LOGMSG_ID__STATE_CHANGE, "%d|%d", ts->state_mgr.state, state);
	SHTPS_LOG_DEBUG( printk(KERN_ERR "[shtps]state %d -> %d\n", ts->state_mgr.state, state); );
	
	if(ts->state_mgr.state != state){
		ts->state_mgr.state = state;
		ret = state_func_tbl[ts->state_mgr.state]->enter(ts, old_state);
	}
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_NOP, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return 0;
}

static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_ERROR, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return -1;
}

static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_STOP, "%d", ts->state_mgr.state);
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	shtps_set_facetouchmode(ts, 1);
	return 0;
}

static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	shtps_set_facetouchmode(ts, 0);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param)
{
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_NORMAL);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param)
{
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_LOADER);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	shtps_irqtimer_stop(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_reset_startuptime(ts);
	shtps_irqtimer_stop(ts);
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param)
{
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	shtps_irqtimer_stop(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param)
{
	unsigned long time;
	if((time = shtps_check_startuptime(ts)) != 0){
		SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]startup wait time : %lu\n", time); );
		msleep(time);
	}
	
	shtps_irqtimer_stop(ts);

	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[shtps]shtps_map_construct() error!!\n"); );
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param)
{
	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[shtps]shtps_map_construct() error!!\n"); );
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param)
{
	if(param == SHTPS_STATE_WAIT_READY){
		shtps_init_param(ts);
		if(ts->poll_info.boot_rezero_flag == 0){
			ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts, 
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
		}
		shtps_read_touchevent(ts, ts->state_mgr.state);
		shtps_irq_wake_enable(ts);
	}

	if(1 == shtps_get_facetouchmode(ts)){
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
		state_change(ts, SHTPS_STATE_FACETOUCH);
	}

	return 0;
}

static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_disable(ts);
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_disable(ts);
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
	shtps_set_facetouchmode(ts, 1);
	state_change(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

#if defined( SHTPS_VKEY_CANCEL_ENABLE )
static int shtps_statef_active_formchange(struct shtps_rmi_spi *ts, int param)
{
	int	i;
	int x;
	int y;
	u8	touchkey_cancel = 0;
	u8	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		y = ts->report_info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		shtps_offset_pos(ts, &x, &y);
		if(ts->report_info.fingers[i].state != 0x00 && y >= CONFIG_SHTPS_SY3000_LCD_SIZE_Y){
			touchkey_cancel = 1;
			break;
		}
	}
	
	if(touchkey_cancel == 0){
		return 0;
	}

	for(i = 0;i < fingerMax;i++){
		x = ts->report_info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		y = ts->report_info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		shtps_offset_pos(ts, &x, &y);
		
		if(ts->report_info.fingers[i].state != 0x00 && y >= CONFIG_SHTPS_SY3000_LCD_SIZE_Y){
			SHTPS_LOG_DEBUG( printk("[shtps][test] vkey cancel (finger[%d])\n", i); )
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  0);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  9999);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 1);

			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100, x=%d, y=%d w=%d\n",
								i,
								0,
								9999,
								1); );
			_log_msg_sync( LOGMSG_ID__EVENT_FORCE_TU, "%d|0|%d|%d",i , x, y);
			input_mt_sync(ts->input);
			
		}else if(ts->report_info.fingers[i].state != 0x00){
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, shtps_get_fingerwidth(ts, i, &(ts->report_info)));

			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100, x=%d, y=%d w=%d\n",
								i,
								x,
								y,
								1); );
			_log_msg_sync( LOGMSG_ID__EVENT_FORCE_TU, "%d|0|%d|%d",i , x, y);
			input_mt_sync(ts->input);
		}
	}
	input_sync(ts->input);
		
	for(i = 0;i < fingerMax;i++){
		x = ts->report_info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		y = ts->report_info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		shtps_offset_pos(ts, &x, &y);
		
		if(ts->report_info.fingers[i].state != 0x00 && y >= CONFIG_SHTPS_SY3000_LCD_SIZE_Y){
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  0);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  0);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);

			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=  0, x=%d, y=%d w=%d\n",
								i,
								0,
								0,
								0); );
			_log_msg_sync( LOGMSG_ID__EVENT_FORCE_TU, "%d|0|%d|%d",i , x, y);
			input_mt_sync(ts->input);

			ts->report_info.fingers[i].state = 0;
			ts->report_info.fingers[i].x = 0;
			ts->report_info.fingers[i].y = 0;
			
			SHTPS_LOG_DEBUG(
				if(ts->vkey_invalid == 0)
					printk(KERN_DEBUG "[shtps][test] vkey invalid flag ON.\n");
			)
			ts->vkey_invalid = 1;
			
		}else if(ts->report_info.fingers[i].state != 0x00){
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, shtps_get_fingerwidth(ts, i, &(ts->report_info)));

			SHTPS_LOG_EVENT( printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100, x=%d, y=%d w=%d\n",
								i,
								x,
								y,
								1); );
			_log_msg_sync( LOGMSG_ID__EVENT_FORCE_TU, "%d|0|%d|%d",i , x, y);
			input_mt_sync(ts->input);
		}
	}
	input_sync(ts->input);

	return 0;
}
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */


/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param)
{
	shtps_loader_irqclr(ts);
	return 0;
}

static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_loader_irqclr(ts);
	shtps_loader_wakeup(ts);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_disable(ts);
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	shtps_rezero_request(ts, 
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_ENDCALL);
	shtps_set_facetouchmode(ts, 0);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param)
{
	shtps_tm_irqcheck(ts);
	shtps_start_tm(ts);
	return 0;
}

static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param)
{
	shtps_stop_tm(ts);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_fwtm_getsensor(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_tmdata(ts, param);;
	return 0;
}

static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param)
{
	if(shtps_tm_irqcheck(ts)){
		shtps_tm_wakeup(ts);
	}
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param)
{
	shtps_event_force_touchup(ts);
	shtps_irq_wake_disable(ts);
	shtps_sleep(ts, 1);
	return 0;
}

static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_enable(ts);
	shtps_sleep(ts, 0);
	shtps_rezero_request(ts, 
						 SHTPS_REZERO_REQUEST_WAKEUP_REZERO |
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_WAKEUP);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param)
{
	shtps_sleep(ts, 0);
	msleep(25);
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_irq_wake_enable(ts);
	shtps_sleep(ts, 0);
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_set_facetouchmode(ts, 1);
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	return 0;
#else
	shtps_irq_wake_disable(ts);
	shtps_sleep(ts, 1);
	return 0;
#endif /* #if !defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
	shtps_irq_wake_disable(ts);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
#else
	shtps_irq_wake_enable(ts);
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_WAKEUP_REZERO, 0);
	state_change(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
	shtps_irq_wake_disable(ts);
#else
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
	shtps_rezero_request(ts, 
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_ENDCALL);
	shtps_irq_wake_disable(ts);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_set_facetouchmode(ts, 0);
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	if(param == 1){
		shtps_wake_lock(ts);
		ts->facetouch.wake_sig = 1;
		_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
		wake_up_interruptible(&ts->facetouch.wait_off);
		shtps_read_touchevent(ts, SHTPS_STATE_SLEEP_FACETOUCH);
		ts->facetouch.wake_sig = 0;
	}
#else
	shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_ioctl_enable(struct shtps_rmi_spi *ts)
{
	int ret = shtps_start(ts);
	shtps_wait_startup(ts);
	return ret;
}

static int shtps_ioctl_disable(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
	return 0;
}

static int shtps_ioctl_reset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;
	
	if(power == 0){
		shtps_irq_enable(ts);
	}
	
	shtps_reset(ts);
	msleep(SHTPS_HWRESET_WAIT_MS);
	
	if(power == 0){
		shtps_irq_disable(ts);
	}
	
	return 0;
}

static int shtps_ioctl_softreset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;
	
	if(power == 0){
		shtps_irq_enable(ts);
	}

	if(shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01) != 0){
		return -EFAULT;
	}
	msleep(SHTPS_SWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}
	
	return 0;
}

static int shtps_ioctl_getver(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u16 ver;
	
	if(0 == arg){
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[%s] error - arg == 0\n", __func__); )
		return -EINVAL;
	}
	
	if(0 != shtps_start(ts)){
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[%s] error - shtps_start()\n", __func__); )
		return -EFAULT;
	}
	shtps_wait_startup(ts);

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_lock()\n"); );
	mutex_lock(&shtps_ctrl_lock);
	
	ver = shtps_fwver(ts);

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_unlock()\n"); );
	mutex_unlock(&shtps_ctrl_lock);
	
	if(copy_to_user((u16*)arg, &ver, sizeof(ver))){
		SHTPS_LOG_ERROR( printk(KERN_DEBUG "[%s] error - copy_to_user()\n", __func__); )
		return -EFAULT;
	}

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[%s] version = 0x%04x\n", __func__, ver); )
	return 0;
}

static int shtps_ioctl_enter_bootloader(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	struct shtps_bootloader_info info;
	
	if(0 == arg){
		return -EINVAL;
	}
	request_event(ts, SHTPS_EVENT_STOP, 0);
	rc = shtps_enter_bootloader(ts);
	
	if(0 == rc){
		info.block_size        = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
		info.program_block_num = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
		info.config_block_num  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);
	
		if(copy_to_user((u8*)arg, (u8*)&info, sizeof(struct shtps_bootloader_info))){
			return -EFAULT;
		}
	}
	
	if(rc){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_lockdown_bootloader(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;
	
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}
	
	data = (u8*)kzalloc(param.size, GFP_KERNEL);
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	rc = shtps_lockdown_bootloader(ts, data);
	kfree(data);
	
	if(rc){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_erase_flash(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return shtps_flash_erase(ts);
}

static int shtps_ioctl_write_image(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;
	
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}
	
	data = (u8*)kzalloc(param.size, GFP_KERNEL);
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	rc = shtps_flash_writeImage(ts, data);
	kfree(data);
	
	if(rc){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_write_config(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;
	
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}
	
	data = (u8*)kzalloc(param.size, GFP_KERNEL);
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	rc = shtps_flash_writeConfig(ts, data);
	kfree(data);
	
	if(rc){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_get_touchinfo(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int i;
	u8	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;
	struct shtps_touch_info info;
	
	memcpy(&info, &ts->report_info, sizeof(info));
	for(i = 0;i < fingerMax;i++){
		info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}
	
	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_get_touchinfo_untrans(struct shtps_rmi_spi *ts, unsigned long arg)
{
	if(copy_to_user((u8*)arg, (u8*)&ts->report_info, sizeof(ts->report_info))){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_set_touchinfo_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.pos_mode = arg;
	return 0;
}

static int shtps_ioctl_reg_read(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 buf;
	u8 data[2];
	struct shtps_ioctl_param param;
	
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size == 1){
		if(0 != copy_from_user(data, param.data, 1)){
			return -EINVAL;
		}
		if(shtps_rmi_read(ts, data[0], &buf, 1)){
			return -EFAULT;
		}
	}else{
		if(0 != copy_from_user(data, param.data, 2)){
			return -EINVAL;
		}
		if(shtps_rmi_read(ts, data[0] << 0x08 | data[1], &buf, 1)){
			return -EFAULT;
		}
	}
	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)&buf, 1)){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_reg_allread(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 dbbuf[0x100];
	u8 data;
	struct shtps_ioctl_param param;
	
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}
	
	if(0 != copy_from_user(&data, param.data, 1)){
		return -EINVAL;
	}
	if(shtps_rmi_read(ts, data << 0x08, dbbuf, 0x100)){
		return -EFAULT;
	}
	
	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)dbbuf, 0x100)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_reg_write(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 data[3];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}
	
	if(param.size == 2){
		if(0 != copy_from_user(data, param.data, 2)){
			return -EINVAL;
		}
		if(shtps_rmi_write(ts, data[0], data[1])){
			return -EFAULT;
		}
	}else{
		if(0 != copy_from_user(data, param.data, 3)){
			return -EINVAL;
		}
		if(shtps_rmi_write(ts, data[0] << 0x08 | data[1], data[2])){
			return -EFAULT;
		}
	}
	return 0;
}

static int shtps_ioctl_tm_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.tm_mode = SHTPS_FWTESTMODE_V01;
	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_tm_stop(struct shtps_rmi_spi *ts)
{
	if(0 != request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_baseline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data, 
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_frameline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_FRAMELINE);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data, 
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_start_facetouchmode(struct shtps_rmi_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_FACETOUCHMODE_ON, 0);
}

static int shtps_ioctl_stop_facetouchmode(struct shtps_rmi_spi *ts)
{
	int ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_OFF, 0);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
	wake_up_interruptible(&ts->facetouch.wait_off);
	shtps_wake_unlock(ts);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return ret;
}

static int shtps_ioctl_poll_facetouchoff(struct shtps_rmi_spi *ts)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	int rc;

	_log_msg_sync( LOGMSG_ID__DETECT_FACETOUCH_WAIT, "");
	rc = wait_event_interruptible(ts->facetouch.wait_off, 
		(ts->facetouch.off_detect == 1) || (ts->facetouch.mode == 0) ||
		(ts->facetouch.wake_sig == 1));

	_log_msg_recv( LOGMSG_ID__DETECT_FACETOUCH, "%d|%d|%d", 
						ts->facetouch.off_detect, ts->facetouch.mode, ts->facetouch.wake_sig);

	ts->facetouch.wake_sig = 0;
	
	if(ts->facetouch.off_detect){
		SHTPS_LOG_DEBUG( printk(KERN_DEBUG "face touch off detect\n"); )
		rc = TPSDEV_FACETOUCHOFF_DETECT;
		ts->facetouch.off_detect = 0;
	}else{
		rc = TPSDEV_FACETOUCHOFF_NOCHG;
	}
	
	return rc;
#else
	return TPSDEV_FACETOUCHOFF_NOCHG;
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_ioctl_get_fwstatus(struct shtps_rmi_spi *ts, unsigned long arg)
{
	unsigned char status;

	shtps_rmi_read(ts, ts->map.fn01.dataBase, &status, 1);
	status = status & 0x0F;
	
	if(copy_to_user((u8*)arg, (u8*)&status, sizeof(status))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_fwdate(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 year;
	u8 month;
	unsigned short date;
	
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_lock()\n"); );
	mutex_lock(&shtps_ctrl_lock);
	
	shtps_fwdate(ts, &year, &month);

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_unlock()\n"); );
	mutex_unlock(&shtps_ctrl_lock);
	
	date = (year << 0x08) | month;
	
	if(copy_to_user((u16*)arg, (u16*)&date, sizeof(date))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_calibration_param(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 *data;
	struct shtps_ioctl_param param;
	
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}
	
	data = (u8*)kzalloc(param.size, GFP_KERNEL);
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	memcpy(ts->offset.base, data, sizeof(u16) * 5);
	//memcpy(ts->offset.diff, &data[10], sizeof(u16) * 12);
	ts->offset.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->offset.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->offset.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->offset.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->offset.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->offset.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->offset.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->offset.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->offset.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->offset.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->offset.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->offset.diff[11]= (signed short)(data[33] << 0x08 | data[32]);
	kfree(data);
	
	if(ts->offset.base[0] == 0){
		ts->offset.enabled = 0;
	}else{
		ts->offset.enabled = 1;
	}
	
	return 0;
}

static int shtps_ioctl_debug_reqevent(struct shtps_rmi_spi *ts, unsigned long arg)
{
	request_event(ts, (int)arg, 0);
	return 0;
}

static int shtps_ioctl_set_dragstep_x(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg != 0){
		SHTPS_DRAG_THRESH_VAL_X_1ST = (int)(arg & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_2ND = (int)((arg >> 0x08) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_VAL_X_1ST | 
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI << 0x18) & 0xFF000000);
}
static int shtps_ioctl_set_dragstep_y(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg != 0){
		SHTPS_DRAG_THRESH_VAL_Y_1ST = (int)(arg & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_2ND = (int)((arg >> 0x08) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_VAL_Y_1ST | 
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI << 0x18) & 0xFF000000);
}
static int shtps_ioctl_set_dragstep(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_set_pollinginterval(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg < 0xFFFF){
		#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
		SHTPS_EVENT_POLL_INTERVAL = (int)(arg & 0xFF);
		SHTPS_POLL_MIN_INTERVAL   = (int)(arg & 0xFF) * 1000;
		SHTPS_POLL_STANDARD_VAL   = (int)((arg >> 0x08) & 0xFF) * 1000;
		#else
		SHTPS_EVENT_POLL_INTERVAL = (int)arg;
		#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	return (((SHTPS_POLL_STANDARD_VAL / 1000) & 0xFF) << 0x08) | (SHTPS_EVENT_POLL_INTERVAL & 0xFF);
	#else
	return SHTPS_EVENT_POLL_INTERVAL;
	#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */
}

static int shtps_ioctl_set_fingerfixtime(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg < 0xFFFF){
		SHTPS_DRAG_THRESH_RETURN_TIME =	(int)arg;
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_RETURN_TIME;
}

static int shtps_ioctl_rezero(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_lock()\n"); );
	mutex_lock(&shtps_ctrl_lock);
	
	if(ts->poll_info.boot_rezero_flag == 0){
		ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts, 
								 SHTPS_REZERO_REQUEST_REZERO |
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
	}
	
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_unlock()\n"); );
	mutex_unlock(&shtps_ctrl_lock);
	
	return 0;
}

static int shtps_ioctl_ack_facetouchoff(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	mutex_lock(&shtps_ctrl_lock);
	shtps_wake_unlock(ts);
	mutex_unlock(&shtps_ctrl_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return 0;
}

static int shtps_ioctl_tmf05_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.tm_mode = SHTPS_FWTESTMODE_V02;
	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static int shtps_ioctl_log_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	gLogOutputEnable = (int)arg;
	return 0;
}
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int shtpsif_open(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_OPEN, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtpsif]Open(PID:%ld)\n", sys_getpid()); );
	return 0;
}

static int shtpsif_release(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_RELEASE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtpsif]Close(PID:%ld)\n", sys_getpid()); );
	return 0;
}

static ssize_t shtpsif_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	int i;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8	fingerMax = F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data) + 1;
	struct shtps_touch_info info;

	_log_msg_sync( LOGMSG_ID__DEVICE_READ, "");
	
	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}
	
	wait_event_interruptible(ts->diag.wait, ts->diag.event == 1);
	
	memcpy(&info, &ts->report_info, sizeof(info));
	if(ts->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
		for(i = 0;i < fingerMax;i++){
			info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
	if(copy_to_user((u8*)buf, (u8*)&info, sizeof(info))){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}
	
	ts->diag.event = 0;
	_log_msg_sync( LOGMSG_ID__DEVICE_READ_DONE, "");
	return sizeof(ts->report_info);
}

static unsigned int shtpsif_poll(struct file *file, poll_table *wait)
{
	int ret;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_FAIL, "");
		return POLLERR;
	}
	
	ret = wait_event_interruptible_timeout(ts->diag.wait, 
			ts->diag.event == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(0 != ret){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "%d", POLLIN | POLLRDNORM);
		return POLLIN | POLLRDNORM;
	}
	
	_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "0");
	return 0;
}

static int shtpsif_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int	rc = 0;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL, "%ld|0x%08X|0x%lX", sys_getpid(), cmd, arg);
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtpsif]ioctl(PID:%ld,CMD:%d,ARG:0x%lx)\n", 
											sys_getpid(), cmd, arg); );
	if(ts == NULL){
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		return -EFAULT;
	}
	
	switch(cmd){
	case TPSDEV_ENABLE: 				rc = shtps_ioctl_enable(ts);					break;
	case TPSDEV_DISABLE:				rc = shtps_ioctl_disable(ts);					break;
	case TPSDEV_RESET:					rc = shtps_ioctl_reset(ts);						break;
	case TPSDEV_SOFT_RESET:				rc = shtps_ioctl_softreset(ts);					break;
	case TPSDEV_GET_FW_VERSION:			rc = shtps_ioctl_getver(ts, arg);				break;
	case TPSDEV_ENTER_BOOTLOADER:		rc = shtps_ioctl_enter_bootloader(ts, arg);		break;
	case TPSDEV_LOCKDOWN_BOOTLOADER:	rc = shtps_ioctl_lockdown_bootloader(ts, arg);	break;
	case TPSDEV_ERASE_FLASE:			rc = shtps_ioctl_erase_flash(ts, arg);			break;
	case TPSDEV_WRITE_IMAGE:			rc = shtps_ioctl_write_image(ts, arg);			break;
	case TPSDEV_WRITE_CONFIG:			rc = shtps_ioctl_write_config(ts, arg);			break;
	case TPSDEV_GET_TOUCHINFO:			rc = shtps_ioctl_get_touchinfo(ts, arg);		break;
	case TPSDEV_GET_TOUCHINFO_UNTRANS:	rc = shtps_ioctl_get_touchinfo_untrans(ts, arg);break;
	case TPSDEV_SET_TOUCHMONITOR_MODE:	rc = shtps_ioctl_set_touchinfo_mode(ts, arg);	break;
	case TPSDEV_READ_REG:				rc = shtps_ioctl_reg_read(ts, arg);				break;
	case TPSDEV_READ_ALL_REG:			rc = shtps_ioctl_reg_allread(ts, arg);			break;
	case TPSDEV_WRITE_REG:				rc = shtps_ioctl_reg_write(ts, arg);			break;
	case TPSDEV_START_TM:				rc = shtps_ioctl_tm_start(ts, arg);				break;
	case TPSDEV_STOP_TM:				rc = shtps_ioctl_tm_stop(ts);					break;
	case TPSDEV_GET_BASELINE:			rc = shtps_ioctl_get_baseline(ts, arg);			break;
	case TPSDEV_GET_FRAMELINE:			rc = shtps_ioctl_get_frameline(ts, arg);		break;
	case TPSDEV_START_FACETOUCHMODE:	rc = shtps_ioctl_start_facetouchmode(ts);		break;
	case TPSDEV_STOP_FACETOUCHMODE:		rc = shtps_ioctl_stop_facetouchmode(ts);		break;
	case TPSDEV_POLL_FACETOUCHOFF:		rc = shtps_ioctl_poll_facetouchoff(ts);			break;
	case TPSDEV_GET_FWSTATUS:			rc = shtps_ioctl_get_fwstatus(ts, arg);			break;
	case TPSDEV_GET_FWDATE:				rc = shtps_ioctl_get_fwdate(ts, arg);			break;
	case TPSDEV_CALIBRATION_PARAM:		rc = shtps_ioctl_calibration_param(ts, arg);	break;
	case TPSDEV_DEBUG_REQEVENT:			rc = shtps_ioctl_debug_reqevent(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP:			rc = shtps_ioctl_set_dragstep(ts, arg);			break;
	case TPSDEV_SET_POLLINGINTERVAL:	rc = shtps_ioctl_set_pollinginterval(ts, arg);	break;
	case TPSDEV_SET_FINGERFIXTIME:		rc = shtps_ioctl_set_fingerfixtime(ts, arg);	break;
	case TPSDEV_REZERO:					rc = shtps_ioctl_rezero(ts, arg);				break;
	case TPSDEV_ACK_FACETOUCHOFF:		rc = shtps_ioctl_ack_facetouchoff(ts, arg);		break;
	case TPSDEV_START_TM_F05:			rc = shtps_ioctl_tmf05_start(ts, arg);			break;
	case TPSDEV_SET_DRAGSTEP_X:			rc = shtps_ioctl_set_dragstep_x(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP_Y:			rc = shtps_ioctl_set_dragstep_y(ts, arg);		break;
#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	case TPSDEV_LOGOUTPUT_ENABLE:		rc = shtps_ioctl_log_enable(ts, arg);			break;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */
	default: 
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		rc = -ENOIOCTLCMD;
		break;
	}
	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_DONE, "%d", rc);
	return rc;
}

static const struct file_operations shtpsif_fileops = {
	.owner   = THIS_MODULE,
	.open    = shtpsif_open,
	.release = shtpsif_release,
	.read    = shtpsif_read,
	.poll    = shtpsif_poll,
	.ioctl   = shtpsif_ioctl,
};

int __init shtpsif_init(void)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__DEVICE_INIT, "");
	rc = alloc_chrdev_region(&shtpsif_devid, 0, 1, SH_TOUCH_IF_DEVNAME);
	if(rc < 0){
		SHTPS_LOG_ERROR( printk(KERN_WARNING "shtpsif:alloc_chrdev_region error\n"); );
		return rc;
	}

	shtpsif_class = class_create(THIS_MODULE, SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_class)) {
		rc = PTR_ERR(shtpsif_class);
		SHTPS_LOG_ERROR( printk(KERN_WARNING "shtpsif:class_create error\n"); );
		goto error_vid_class_create;
	}

	shtpsif_device = device_create(shtpsif_class, NULL, 
								shtpsif_devid, &shtpsif_cdev, 
								SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_device)) {
		rc = PTR_ERR(shtpsif_device);
		SHTPS_LOG_ERROR(printk(KERN_WARNING "shtpsif:device_create error\n"); );
		goto error_vid_class_device_create;
	}
	
	cdev_init(&shtpsif_cdev, &shtpsif_fileops);
	shtpsif_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shtpsif_cdev, shtpsif_devid, 1);
	if(rc < 0){
		SHTPS_LOG_ERROR( printk(KERN_WARNING "shtpsif:cdev_add error\n"); );
		goto err_via_cdev_add;
	}

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtpsif]shtpsif_init() done\n"); );
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_DONE, "");
	
	return 0;
	
err_via_cdev_add:
	cdev_del(&shtpsif_cdev);
error_vid_class_device_create:
	class_destroy(shtpsif_class);
error_vid_class_create:
	unregister_chrdev_region(shtpsif_devid, 1);
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_FAIL, "");

	return rc;
}
module_init(shtpsif_init);

static void __exit shtpsif_exit(void)
{
	cdev_del(&shtpsif_cdev);
	class_destroy(shtpsif_class);
	unregister_chrdev_region(shtpsif_devid, 1);
	
	_log_msg_sync( LOGMSG_ID__DEVICE_EXIT_DONE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtpsif]shtpsif_exit() done\n"); );
}
module_exit(shtpsif_exit);

/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_open(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);
	
	_log_msg_sync( LOGMSG_ID__OPEN, "%ld", sys_getpid());
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]Open(PID:%ld)\n", sys_getpid()); );

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
	if(shtps_touchpanel_enable(ts) && shtps_fwupdate_enable(ts)){
		u8 buf;
		const unsigned char* fw_data = NULL;
		#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
		int ver;
		#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */

		if(shtps_start(ts) == 0){
			shtps_wait_startup(ts);
		}

		#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
		ver = shtps_fwver(ts);
		printk(KERN_DEBUG "fw version = 0x%04x\n", ver);
		#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */
		shtps_rmi_read(ts, 0x0013, &buf, 1);
		
		#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
		if(((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6) || ver != SHTPS_FWVER_NEWER){
		#else
		if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
			printk(KERN_DEBUG "[shtps] Touch panel CRC error detect\n");
		#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */
			fw_data = tps_fw_data;
			if(fw_data){
				int ret;
				int retry = 5;
				do{
					ret = shtps_fw_update(ts, fw_data);
					request_event(ts, SHTPS_EVENT_STOP, 0);
				}while(ret != 0 && (retry-- > 0));
			}
		}
	}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */
	shtps_start(ts);
	
	return 0;
}

static void shtps_rmi_close(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	_log_msg_sync( LOGMSG_ID__CLOSE, "%ld", sys_getpid());
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]Close(PID:%ld)\n", sys_getpid()); );
	shtps_shutdown(ts);
}

static int __devinit shtps_rmi_probe(struct spi_device *spi)
{
	int i;
	int result;
	struct input_dev *input_dev;
	struct shtps_rmi_spi *ts;
	struct shtps_platform_data *pdata = spi->dev.platform_data;

	_log_msg_sync( LOGMSG_ID__PROBE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_lock()\n"); );
	mutex_lock(&shtps_ctrl_lock);

	ts = kzalloc(sizeof(struct shtps_rmi_spi), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !ts) {
		result = -ENOMEM;
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}
	spi_set_drvdata(spi, ts);

	if (pdata && pdata->setup) {
		result = pdata->setup(&spi->dev);
		if (result){
			mutex_unlock(&shtps_ctrl_lock);
			goto fail_alloc_mem;
		}
	}
	INIT_WORK(&ts->irq_workq, shtps_work_irqf);
	INIT_DELAYED_WORK(&ts->tmo_check, shtps_work_tmof);
	hrtimer_init(&ts->polling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->polling_timer.function = shtps_polling_timer_function;
	INIT_WORK(&ts->polling_work, shtps_work_pollf);

#if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED )
	hrtimer_init(&ts->usleep_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->usleep_timer.function  = shtps_usleep_timer_function;
#endif /* #if defined( SHTPS_NOTIFY_INTERVAL_GUARANTEED ) */

	init_waitqueue_head(&ts->wait_start);
	init_waitqueue_head(&ts->loader.wait_ack);
	init_waitqueue_head(&ts->diag.wait);
	init_waitqueue_head(&ts->diag.tm_wait_ack);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	init_waitqueue_head(&ts->facetouch.wait_off);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	hrtimer_init(&ts->rezero_delayed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->rezero_delayed_timer.function = shtps_delayed_rezero_timer_function;
	INIT_WORK(&ts->rezero_delayed_work, shtps_rezero_delayed_work_function);

#if defined( SHTPS_OWN_THREAD_ENABLE )
	spin_lock_init( &(ts->queue_lock) );
	INIT_LIST_HEAD( &(ts->queue) );
	INIT_WORK( &(ts->work_data), shtps_func_workq );
	ts->workqueue_p = create_singlethread_workqueue( "TPS_WORKQ" );
#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */

	ts->map.fn01.ctrlBase = SHTPS_F01_RMI_CTRL_ADR;
	ts->state_mgr.state = SHTPS_STATE_IDLE;
	ts->loader.ack = 0;
	ts->diag.event = 0;
	ts->facetouch.mode = 0;
	ts->facetouch.state = 0;
	ts->facetouch.off_detect = 0;
	ts->facetouch.wake_sig = 0;
	ts->offset.enabled = 0;
	memset(&ts->poll_info,   0, sizeof(ts->poll_info));
	memset(&ts->report_info, 0, sizeof(ts->report_info));
	memset(&ts->center_info, 0, sizeof(ts->center_info));
	memset(&ts->touch_state, 0, sizeof(ts->touch_state));
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		ts->touch_state.dragStep[i][0] = SHTPS_DRAG_THRESHOLD_1ST;
		ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_1ST;
	}
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	wake_lock_init(&ts->facetouch.wake_lock, WAKE_LOCK_SUSPEND, "shtps_wake_lock");
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	/** get platform info */
	gShtps_rmi_spi	= ts;
	ts->spi			= spi;
	ts->input		= input_dev;
	ts->rst_pin		= pdata->gpio_rst;
	ts->irq_mgr.irq	= spi->irq;

	/** setup touchscreen */
	input_dev->name 		= SH_TOUCH_DEVNAME;
	input_dev->phys 		= "msm_touch/input0";
	//input_dev->id.bustype = BUS_SPI;
	input_dev->id.vendor 	= 0x0001;
	input_dev->id.product 	= 0x0002;
	input_dev->id.version 	= 0x0100;
	input_dev->dev.parent 	= &spi->dev;
	input_dev->open			= shtps_rmi_open;
	input_dev->close		= shtps_rmi_close;

	if(shtps_irq_resuest(ts)){
		SHTPS_LOG_ERROR( printk(KERN_WARNING "shtps:request_irq error\n"); );
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}

	#if defined( SHTPS_GPIO_SPI_ENABLE )
	shtps_rmi_spi_init();
	#endif /* #if defined( SHTPS_GPIO_SPI_ENABLE ) */

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]mutex_unlock()\n"); );
	mutex_unlock(&shtps_ctrl_lock);

	__set_bit(EV_KEY, input_dev->evbit);
#if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY )
	__set_bit(KEY_PROG1, input_dev->keybit);
#endif /* #if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY ) */
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X,  input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y,  input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	
	input_set_drvdata(input_dev, ts);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, SHTPS_FINGER_WIDTH_PALMDET, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1, 0, 0);

	result = input_register_device(input_dev);
	if (result)
		goto fail_input_register_device;

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_rmi_probe() done\n"); );
	_log_msg_sync( LOGMSG_ID__PROBE_DONE, "");
	return 0;

fail_input_register_device:
fail_alloc_mem:
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	wake_lock_destroy(&ts->facetouch.wake_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	input_free_device(input_dev);
	kfree(ts);
	_log_msg_sync( LOGMSG_ID__PROBE_FAIL, "");
	return result;
}

static int __devexit shtps_rmi_remove(struct spi_device *spi)
{
	struct shtps_rmi_spi *ts = spi_get_drvdata(spi);
	struct shtps_platform_data *pdata = spi->dev.platform_data;

	hrtimer_cancel(&ts->polling_timer);
	hrtimer_cancel(&ts->rezero_delayed_timer);

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	wake_lock_destroy(&ts->facetouch.wake_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	free_irq(ts->irq_mgr.irq, ts);
	if (pdata && pdata->teardown){
		pdata->teardown(&spi->dev);
	}
	
	gShtps_rmi_spi = NULL;
	kfree(ts);

	_log_msg_sync( LOGMSG_ID__REMOVE_DONE, "");
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_rmi_remove() done\n"); );
	return 0;
}

static int shtps_rmi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	_log_msg_sync( LOGMSG_ID__SUSPEND, "");
	return 0;
}

static int shtps_rmi_resume(struct spi_device *spi)
{
	_log_msg_sync( LOGMSG_ID__RESUME, "");
#if defined( SHTPS_GPIO_SPI_ENABLE )
	mutex_lock(&shtps_ctrl_lock);
	shtps_rmi_spi_init();
	mutex_unlock(&shtps_ctrl_lock);
#endif /* #if defined( SHTPS_GPIO_SPI_ENABLE ) */
	return 0;
}

static struct spi_driver shtps_rmi_driver = {
	.probe		= shtps_rmi_probe,
	.remove		= __devexit_p(shtps_rmi_remove),
	.suspend	= shtps_rmi_suspend,
	.resume		= shtps_rmi_resume,
	.driver		= {
		.name  = SH_TOUCH_DEVNAME,
		.owner = THIS_MODULE,
		
	},
};

static int __init shtps_rmi_init(void)
{
	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_rmi_init() start\n"); );
	_log_msg_sync( LOGMSG_ID__INIT, "");
	
	return spi_register_driver(&shtps_rmi_driver);
}
module_init(shtps_rmi_init);

static void __exit shtps_rmi_exit(void)
{
	spi_unregister_driver(&shtps_rmi_driver);

	SHTPS_LOG_DEBUG( printk(KERN_DEBUG "[shtps]shtps_rmi_exit() done\n"); );
	_log_msg_sync( LOGMSG_ID__EXIT, "");
}
module_exit(shtps_rmi_exit);

/* -----------------------------------------------------------------------------------
 */
void msm_tps_setsleep(int on)
{
	extern	void sh_spi_tps_standby(struct spi_device *spi);
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	
	_log_msg_sync( LOGMSG_ID__API_SLEEP, "%d", on);
	if(ts){
		if(on){
			sh_spi_tps_standby(ts->spi);
			request_event(ts, SHTPS_EVENT_SLEEP, 0);
		}else{
			request_event(ts, SHTPS_EVENT_WAKEUP, 0);
		}
	}
	_log_msg_sync( LOGMSG_ID__API_SLEEP_DONE, "");
}
EXPORT_SYMBOL(msm_tps_setsleep);

void shtps_setFlipInformation(int state)
{
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	
	#if defined( SHTPS_OWN_THREAD_ENABLE )
	shtps_func_api_request_from_irq(ts, SHTPS_EVENT_FORMCHANGE, state);
	#else
	request_event(ts, SHTPS_EVENT_FORMCHANGE, state);
	#endif /* #if defined( SHTPS_OWN_THREAD_ENABLE ) */	
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
}
EXPORT_SYMBOL(shtps_setFlipInformation);

MODULE_LICENSE("GPL");


#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data)
{
	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE, "");

	if(0 != shtps_enter_bootloader(ts)){
		printk(KERN_DEBUG "[shtps] error - shtps_enter_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "0");
		return -1;
	}
	
	if(0 != shtps_lockdown_bootloader(ts, (u8*)&fw_data[0x00d0])){
		printk(KERN_DEBUG "[shtps] error - shtps_lockdown_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "1");
		return -1;
	}
	
	if(0 != shtps_flash_erase(ts)){
		printk(KERN_DEBUG "[shtps] error - shtps_flash_erase()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "2");
		return -1;
	}
	
	{
		int i;
		unsigned long blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
		unsigned long blockNum  = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
		
		for(i = 0;i < blockNum;i++){
			if(0 != shtps_flash_writeImage(ts, (u8*)&fw_data[0x0100 + i * blockSize])){
				printk(KERN_DEBUG "[shtps] error - shtps_flash_writeImage(%d)\n", i);
				_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "3");
				return -1;
			}
		}
	}

	if(0 != shtps_flash_writeConfig(ts, (u8*)&fw_data[0x6f00])){
		printk(KERN_DEBUG "[shtps] error - shtps_flash_writeConfig()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "4");
		return -1;
	}
	if(0 != shtps_exit_bootloader(ts)){
		printk(KERN_DEBUG "[shtps] error - shtps_exit_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "5");
		return -1;
	}
	printk(KERN_DEBUG "[shtps] fw update done\n");
	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_DONE, "");
	
	return 0;
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */
