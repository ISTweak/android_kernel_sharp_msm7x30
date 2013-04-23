/* arch/arm/mach-msm/rpc_server_handset.c
 *
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/miscdevice.h>

#include <../../../include/sharp/sh_smem.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <mach/rpc_server_handset.h>

#if 1
#include <mach/msm_i2ckbd.h>
#endif

#include <sharp/shterm_k.h>

#ifdef CONFIG_SHEXTDEV_FLIP
#define FLIP_USE
#endif
#ifdef CONFIG_SHEXTDEV_SWIVEL
#define FLIP_USE
#define SWIVEL_USE
#endif

#ifdef FLIP_USE
#define CONFIG_PERF_LOCK_ENABLE
#endif

#ifdef CONFIG_PERF_LOCK_ENABLE
#include <mach/perflock.h>
#endif

#if defined(FLIP_USE) || defined(SWIVEL_USE)
#if defined(CONFIG_SENSORS_AMI602) || defined(CONFIG_SENSORS_AMI603)
#include <../../../include/sharp/shmds_driver.h>
#endif	/* CONFIG_SENSORS_AMI602 */

#ifdef CONFIG_SH_YAS530
#include <sharp/shyas_driver.h>
#endif	/* CONFIG_SH_YAS530 */

#if defined( CONFIG_SHTPS_SY3000_TM1918_001 ) | defined( CONFIG_SHTPS_SY3000_TM1963_001 ) | defined( CONFIG_SHTPS_TMA3XX_TMA340_005 )
#include <sharp/shtps_dev.h>
#endif	/* #if defined( CONFIG_SHTPS_SY3000_TM1918_001 ) | defined( CONFIG_SHTPS_SY3000_TM1963_001 ) | defined( CONFIG_SHTPS_TMA3XX_TMA340_005 ) */

#endif	/* FLIP_USE */

#include <sharp/sh_boot_manager.h>

#define DRIVER_NAME	"msm-handset"
#if 1
#define DRIVER_SH_PM_NAME	"SH_pm_key"
#define DRIVER_SH_HS_NAME	"SH_headset_key"
#endif

#define HS_SERVER_PROG 0x30000062
#define HS_SERVER_VERS 0x00010001

#define HS_RPC_PROG 0x30000091

#if 0
#define HS_RPC_VERS_1 0x00010001
#define HS_RPC_VERS_2 0x00020001
#else
#define HS_RPC_VERS_1 0x00030001
#define HS_RPC_VERS_2 0x00030002
#endif

#define HS_PROCESS_CMD_PROC 0x02
#define HS_SUBSCRIBE_SRVC_PROC 0x03
#define HS_REPORT_EVNT_PROC    0x05
#if 1
#define HS_SHEXTDET_API_INITIALIZE_REMOTE_PROC 0x06
#define HS_SHEXTDET_API_VREG_ON 0x07
#define HS_SHEXTDET_API_VREG_OFF 0x08
#endif
#define HS_EVENT_CB_PROC	1
#define HS_EVENT_DATA_VER	1

#define RPC_KEYPAD_NULL_PROC 0
#define RPC_KEYPAD_PASS_KEY_CODE_PROC 2
#define RPC_KEYPAD_SET_PWR_KEY_STATE_PROC 3

#define HS_PWR_K		0x6F	/* Power key */
#define HS_END_K		0x51	/* End key or Power key */
#define HS_STEREO_HEADSET_K	0x82
#define HS_HEADSET_SWITCH_K	0x84
#if defined(FLIP_USE) || defined(SWIVEL_USE)
#define HS_FLIP_K		0x88
#endif
#define HS_HEADSET_SWITCH_2_K	0xF0
#define HS_HEADSET_SWITCH_3_K	0xF1
#define HS_HEADSET_HEADPHONE_K	0xF6
#define HS_HEADSET_MICROPHONE_K 0xF7
#define HS_REL_K		0xFF	/* key release */

#define SW_HEADPHONE_INSERT_W_MIC 1 /* HS with mic */

#define KEY(hs_key, input_key) ((hs_key << 24) | input_key)

#define DEBUG 0

enum hs_event {
	HS_EVNT_EXT_PWR = 0,	/* External Power status        */
	HS_EVNT_HSD,		/* Headset Detection            */
	HS_EVNT_HSTD,		/* Headset Type Detection       */
	HS_EVNT_HSSD,		/* Headset Switch Detection     */
	HS_EVNT_KPD,
	HS_EVNT_FLIP,		/* Flip / Clamshell status (open/close) */
	HS_EVNT_CHARGER,	/* Battery is being charged or not */
	HS_EVNT_ENV,		/* Events from runtime environment like DEM */
	HS_EVNT_REM,		/* Events received from HS counterpart on a
				remote processor*/
	HS_EVNT_DIAG,		/* Diag Events  */
	HS_EVNT_LAST,		 /* Should always be the last event type */
	HS_EVNT_MAX		/* Force enum to be an 32-bit number */
};

enum hs_src_state {
	HS_SRC_STATE_UNKWN = 0,
	HS_SRC_STATE_LO,
	HS_SRC_STATE_HI,
};

struct hs_event_data {
	uint32_t	ver;		/* Version number */
	enum hs_event	event_type;     /* Event Type	*/
	enum hs_event	enum_disc;     /* discriminator */
	uint32_t	data_length;	/* length of the next field */
	enum hs_src_state	data;    /* Pointer to data */
	uint32_t	data_size;	/* Elements to be processed in data */
};

enum hs_return_value {
	HS_EKPDLOCKED     = -2,	/* Operation failed because keypad is locked */
	HS_ENOTSUPPORTED  = -1,	/* Functionality not supported */
	HS_FALSE          =  0, /* Inquired condition is not true */
	HS_FAILURE        =  0, /* Requested operation was not successful */
	HS_TRUE           =  1, /* Inquired condition is true */
	HS_SUCCESS        =  1, /* Requested operation was successful */
	HS_MAX_RETURN     =  0x7FFFFFFF/* Force enum to be a 32 bit number */
};

struct hs_key_data {
	uint32_t ver;        /* Version number to track sturcture changes */
	uint32_t code;       /* which key? */
	uint32_t parm;       /* key status. Up/down or pressed/released */
};

enum hs_subs_srvc {
	HS_SUBS_SEND_CMD = 0, /* Subscribe to send commands to HS */
	HS_SUBS_RCV_EVNT,     /* Subscribe to receive Events from HS */
	HS_SUBS_SRVC_MAX
};

enum hs_subs_req {
	HS_SUBS_REGISTER,    /* Subscribe   */
	HS_SUBS_CANCEL,      /* Unsubscribe */
	HS_SUB_STATUS_MAX
};

enum hs_event_class {
	HS_EVNT_CLASS_ALL = 0, /* All HS events */
	HS_EVNT_CLASS_LAST,    /* Should always be the last class type   */
	HS_EVNT_CLASS_MAX
};

enum hs_cmd_class {
	HS_CMD_CLASS_LCD = 0, /* Send LCD related commands              */
	HS_CMD_CLASS_KPD,     /* Send KPD related commands              */
	HS_CMD_CLASS_LAST,    /* Should always be the last class type   */
	HS_CMD_CLASS_MAX
};

/*
 * Receive events or send command
 */
union hs_subs_class {
	enum hs_event_class	evnt;
	enum hs_cmd_class	cmd;
};

struct hs_subs {
	uint32_t                ver;
	enum hs_subs_srvc	srvc;  /* commands or events */
	enum hs_subs_req	req;   /* subscribe or unsubscribe  */
	uint32_t		host_os;
	enum hs_subs_req	disc;  /* discriminator    */
	union hs_subs_class      id;
};

struct hs_event_cb_recv {
	uint32_t cb_id;
	uint32_t hs_key_data_ptr;
	struct hs_key_data key;
};
enum hs_ext_cmd_type {
	HS_EXT_CMD_KPD_SEND_KEY = 0, /* Send Key */
	HS_EXT_CMD_KPD_BKLT_CTRL, /* Keypad backlight intensity	*/
	HS_EXT_CMD_LCD_BKLT_CTRL, /* LCD Backlight intensity */
	HS_EXT_CMD_DIAG_KEYMAP, /* Emulating a Diag key sequence */
	HS_EXT_CMD_DIAG_LOCK, /* Device Lock/Unlock */
	HS_EXT_CMD_GET_EVNT_STATUS, /* Get the status for one of the drivers */
	HS_EXT_CMD_KPD_GET_KEYS_STATUS,/* Get a list of keys status */
	HS_EXT_CMD_KPD_SET_PWR_KEY_RST_THOLD, /* PWR Key HW Reset duration */
	HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD, /* Set pwr key threshold duration */
	HS_EXT_CMD_LAST, /* Should always be the last command type */
	HS_EXT_CMD_MAX = 0x7FFFFFFF /* Force enum to be an 32-bit number */
};

struct hs_cmd_data_type {
	uint32_t hs_cmd_data_type_ptr; /* hs_cmd_data_type ptr length */
	uint32_t ver; /* version */
	enum hs_ext_cmd_type id; /* command id */
	uint32_t handle; /* handle returned from subscribe proc */
	enum hs_ext_cmd_type disc_id1; /* discriminator id */
	uint32_t input_ptr; /* input ptr length */
	uint32_t input_val; /* command specific data */
	uint32_t input_len; /* length of command input */
	enum hs_ext_cmd_type disc_id2; /* discriminator id */
	uint32_t output_len; /* length of output data */
	uint32_t delayed; /* execution context for modem
				true - caller context
				false - hs task context*/
};

static const uint32_t hs_key_map[] = {
	KEY(HS_PWR_K, KEY_POWER),
	KEY(HS_END_K, KEY_END),
	KEY(HS_STEREO_HEADSET_K, SW_HEADPHONE_INSERT),
	KEY(HS_STEREO_HEADSET_K, SW_HEADPHONE_INSERT_W_MIC),
	KEY(HS_HEADSET_HEADPHONE_K, SW_HEADPHONE_INSERT),
	KEY(HS_HEADSET_MICROPHONE_K, SW_MICROPHONE_INSERT),
	KEY(HS_HEADSET_SWITCH_K, KEY_MEDIA),
	KEY(HS_HEADSET_SWITCH_2_K, KEY_VOLUMEUP),
	KEY(HS_HEADSET_SWITCH_3_K, KEY_VOLUMEDOWN),
#ifdef FLIP_USE	/* FLIP */
	KEY(HS_FLIP_K, SW_LID),
#endif
#if defined(CONFIG_TENKEY_KEYPAD) && (CONFIG_TENKEY_KEYPAD_SUBID == 2)
	KEY(HS_PWR_K, KEY_SAVE),
#endif
	0
};

#if 0
enum {
	NO_DEVICE	= 0,
	MSM_HEADSET	= 1,
};
#else
enum {
	NO_DEVICE			= 0x00,
	MSM_HEADSET_STE		= 0x01,
	MSM_HEADSET_MONO	= 0x02,
	MSM_HEADSET_OTHER	= 0x03,
};
#endif

#if 1
enum {
	MSM_OPEN	= 0x00,
	MSM_CLOSE	= 0x01,
	MSM_SWIVEL_CLOSE	= 0x02,
	MSM_OPEN_CHATT = 0x10,
	MSM_CLOSE_CHATT = 0x11,
};
enum {
	MSM_HSSW_RELEASE	= 0x00,
	MSM_HSSW_PRESS		= 0x01,
};
#endif

/* Add newer versions at the top of array */
static const unsigned int rpc_vers[] = {
	0x00030001,
	0x00020001,
	0x00010001,
};
/* hs subscription request parameters */
struct hs_subs_rpc_req {
	uint32_t hs_subs_ptr;
	struct hs_subs hs_subs;
	uint32_t hs_cb_id;
	uint32_t hs_handle_ptr;
	uint32_t hs_handle_data;
};

static struct hs_subs_rpc_req *hs_subs_req;

struct msm_handset {
	struct input_dev *ipdev;
	struct switch_dev sdev;
	struct msm_handset_platform_data *hs_pdata;
	bool mic_on, hs_on;
#ifdef FLIP_USE	/* FLIP */
	struct switch_dev sdev_flip;
	struct switch_dev sdev_flip_chatt;
#endif
#if 1
	struct switch_dev sdev_hssw;
#endif
};

static struct msm_rpc_client *rpc_client;
static struct msm_handset *hs;

#if 1
static struct msm_handset *hssw;
static struct msm_handset *pm_key;

static uint8_t Headset_Status = NO_DEVICE;

#endif

#if 1
static struct msm_rpc_endpoint* rpc_svc_p;
#endif

shextdet_form_position_result_t shextdev_form_state = SHEXTDET_FORM_POSITION_CLOSE;

#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
#include <linux/wakelock.h>
static struct wake_lock shext_hs_suspend_wake_lock;
static struct wake_lock shext_hs_idle_wake_lock;
static bool shext_hs_wake_lock_init = false;
static int shext_hs_wake_lock_count = 0;
static bool shext_hs_wake_lock_start_flg = false;
#endif	/* Wake_Lock <- */

#ifdef CONFIG_PERF_LOCK_ENABLE
static struct perf_lock flip_perf_lock;
#endif
#if defined(CONFIG_TENKEY_KEYPAD) && (CONFIG_TENKEY_KEYPAD_SUBID == 2)
static bool shext_hw_revision_cleck = false;
#endif

void headset_vreg(unsigned long onoff)
{
	int rc;

	struct headset_rpc_start_t {
		struct rpc_request_hdr hdr;
	} send_p;

	struct headset_rpc_ret_t {
		struct rpc_reply_hdr hdr;
	} recv_p;

	if(IS_ERR(rpc_svc_p)) {
		pr_err("%s: couldn't connect compatible rpc client err %ld\n", __func__,
			 PTR_ERR(rpc_svc_p));
	}

	if(onoff==1){
		rc = msm_rpc_call_reply(rpc_svc_p,
							HS_SHEXTDET_API_VREG_ON,
							&send_p,sizeof(send_p),
							&recv_p,sizeof(recv_p),
							5 * HZ);
	}else{
		rc = msm_rpc_call_reply(rpc_svc_p,
							HS_SHEXTDET_API_VREG_OFF,
							&send_p,sizeof(send_p),
							&recv_p,sizeof(recv_p),
							5 * HZ);
	}
	if (rc < 0) {
#if DEBUG
		printk(KERN_ERR "%s: rpc call failed! error: (%d)\n", __func__, rc);
#endif
	}
}

shextdet_form_position_result_t shextdet_get_form_state(void)
{
	return shextdev_form_state;
}

void shextdev_set_form_state(shextdet_form_position_result_t state)
{
	shextdev_form_state = state;
}

void shextdev_SetFlipInformation(shextdet_form_position_result_t state)
{
#ifdef FLIP_USE
#if defined(CONFIG_SENSORS_AMI602) || defined(CONFIG_SENSORS_AMI603)
	switch(state)
	{
	case SHEXTDET_FORM_POSITION_OPEN:
		AMI602_SetFlipInformation(MS_POSITION_OPEN);
		break;
	case SHEXTDET_FORM_POSITION_CLOSE:
		AMI602_SetFlipInformation(MS_POSITION_CLOSE);
		break;
	case SHEXTDET_FORM_POSITION_SWIVEL:
		AMI602_SetFlipInformation(MS_POSITION_SWIVEL);
		break;
	}
#endif	/* CONFIG_SENSORS_AMI602 */

#ifdef CONFIG_SH_YAS530
	switch(state)
	{
	case SHEXTDET_FORM_POSITION_OPEN:
		YAS530_SetShape(YAS_POSITION_OPEN);
		break;
	case SHEXTDET_FORM_POSITION_CLOSE:
		YAS530_SetShape(YAS_POSITION_CLOSE);
		break;
	}
#endif	/* CONFIG_SH_YAS530 */

#if defined( CONFIG_SHTPS_SY3000_TM1918_001 ) | defined( CONFIG_SHTPS_SY3000_TM1963_001 ) | defined( CONFIG_SHTPS_TMA3XX_TMA340_005 )
	switch(state)
	{
	case SHEXTDET_FORM_POSITION_OPEN:
		shtps_setFlipInformation(TPS_POSITION_OPEN);
		break;
	case SHEXTDET_FORM_POSITION_CLOSE:
		shtps_setFlipInformation(TPS_POSITION_CLOSE);
		break;
	}
#endif /* #if defined( CONFIG_SHTPS_SY3000_TM1918_001 ) | defined( CONFIG_SHTPS_SY3000_TM1963_001 ) | defined( CONFIG_SHTPS_TMA3XX_TMA340_005 ) */

#ifdef CONFIG_PERF_LOCK_ENABLE
		perf_lock(&flip_perf_lock);
		perf_unlock(&flip_perf_lock);
#endif	/* CONFIG_PERF_LOCK_ENABLE */

#endif	/* FLIP_USE */
}


static int hs_find_key(uint32_t hscode)
{
	int i, key;

	key = KEY(hscode, 0);

	for (i = 0; hs_key_map[i] != 0; i++) {
		if ((hs_key_map[i] & 0xff000000) == key)
			return hs_key_map[i] & 0x00ffffff;
	}
	return -1;
}


static void
report_headset_switch(struct input_dev *dev, int key, int value)
{
	struct msm_handset *hs = input_get_drvdata(dev);
#if 0
	input_report_switch(dev, key, value);
	switch_set_state(&hs->sdev, value);
#else

#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
	if( shext_hs_wake_lock_start_flg ){
		if( !shext_hs_wake_lock_init ){
			wake_lock_init(&shext_hs_suspend_wake_lock, WAKE_LOCK_SUSPEND, "shext_hs_suspend_lock");
			wake_lock_init(&shext_hs_idle_wake_lock, WAKE_LOCK_IDLE, "shext_hs_idle_lock");

			shext_hs_wake_lock_init = true;
		}
	}else{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] shext_hs_wake_lock_start_flg = false\n", __func__);
#endif
	}
#endif	/* Wake_Lock <- */

	switch(value)
	{
		case MSM_HEADSET_STE:
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] MSM_HEADSET_STE key = %d, !!value = %d, value = %d\n", __func__, key, !!value, value);
#endif

#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
			if( shext_hs_wake_lock_init ){
				if( shext_hs_wake_lock_count == 0 )
			    {
					wake_lock(&shext_hs_suspend_wake_lock);
					wake_lock(&shext_hs_idle_wake_lock);
					#if DEBUG
					printk(KERN_INFO "[msm-handset][%s]wake_lock MSM_HEADSET_STE shext_headset_wake_lock Call\n",__func__);
					#endif
				}
				shext_hs_wake_lock_count++;
				#if DEBUG
				printk(KERN_INFO "[msm-handset][%s]wake_lock MSM_HEADSET_STE shext_headset_wake_lock count:%d\n",__func__,shext_hs_wake_lock_count);
				#endif
			}
#endif	/* Wake_Lock <- */
			input_report_switch(dev, key, !!value);
			break;
		case NO_DEVICE:
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s]NO_DEVICE key = %d, !!value = %d, value = %d\n", __func__, key, !!value, value);
#endif

#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
			if( shext_hs_wake_lock_init ){
				if( shext_hs_wake_lock_count == 0 )
				{
					wake_lock(&shext_hs_suspend_wake_lock);
					wake_lock(&shext_hs_idle_wake_lock);
					#if DEBUG
					printk(KERN_INFO "[msm-handset][%s]wake_lock NO_DEVICE shext_headset_wake_lock Call\n",__func__);
					#endif
				}
				shext_hs_wake_lock_count++;
				#if DEBUG
				printk(KERN_INFO "[msm-handset][%s]wake_lock NO_DEVICE shext_headset_wake_lock count:%d\n",__func__,shext_hs_wake_lock_count);
				#endif
			}
#endif	/* Wake_Lock <- */
			input_report_switch(dev, key, !!value);
			break;
		default:
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] non Value [%d]\n", __func__, value);
#endif
			break;
	}
	Headset_Status = value;
	switch_set_state(&hs->sdev, !!value);
#endif
}

#ifdef FLIP_USE	/* FLIP */
static int
report_flip_switch(struct input_dev *dev, int key, int value)
{
	struct msm_handset *hs = input_get_drvdata(dev);
	static int old_state = 0x0F;
#if 0
	static int old_state2 = 0x0F;
#endif
#if DEBUG
	printk(KERN_INFO "[msm-handset] [%s] sdev_flip_chatt value = 0x%02x\n", __func__, value);
#endif
	switch_set_state(&hs->sdev_flip_chatt, value);
	
#if 0
	if(old_state2 != value)
	{
		msm_i2ctps_flipchange(value);
	}input_report_switch
	old_state2 = value;
#endif
	if( ((value & 0xF0) == 0x00) && (old_state != value) )
	{
#if 0
		msm_i2ckbd_flipchange(value);
#endif
#if DEBUG
//		printk(KERN_INFO "[msm-handset] [%s] key = %d, !!value = %d, value = %d\n", __func__, key, !!value, value);
		printk(KERN_INFO "[msm-handset] [%s] flip = %d, !!value = %d, value = %d\n", __func__, key, !!value, value);
#endif
		switch_set_state(&hs->sdev_flip, value);
		shextdev_set_form_state(value);
		shextdev_SetFlipInformation(value);
		input_report_switch(dev, key, !!value);
#if 1
        /* Flip Open? */
        if(value == 0x00)
        {
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] shterm_flip_status_set(SHTERM_FLIP_STATE_OPEN)\n", __func__);
#endif
            shterm_flip_status_set(SHTERM_FLIP_STATE_OPEN);
        }
        else if(value == 0x01)
        {
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] shterm_flip_status_set(SHTERM_FLIP_STATE_CLOSE)\n", __func__);
#endif
            shterm_flip_status_set(SHTERM_FLIP_STATE_CLOSE);
        }
        else if(value == 0x02)
        {
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] shterm_flip_status_set(SHTERM_FLIP_STATE_OPEN)\n", __func__);
#endif
            shterm_flip_status_set(SHTERM_FLIP_STATE_OPEN);
        }
#endif
		old_state = value;
		return 0;
	}
	return -1;
}
#endif

static void
report_headphone_switch(struct input_dev *dev, int key, int value)
{
#if 0
	struct msm_handset *hs = input_get_drvdata(dev);
#if DEBUG
	printk(KERN_INFO "[msm-handset] [%s] sdev_hssw value = 0x%02x\n", __func__, value);
#endif
	switch_set_state(&hs->sdev_hssw, value);
	input_report_key(dev, key, value);
#else
	struct msm_handset *hssw = input_get_drvdata(dev);
#if DEBUG
	printk(KERN_INFO "[msm-handset] [%s] sdev_hssw value = 0x%02x\n", __func__, value);
#endif
	switch_set_state(&hssw->sdev_hssw, value);
	input_report_key(dev, key, value);
#endif
}

static void update_state(void)
{
	int state;

	if (hs->mic_on && hs->hs_on)
		state = 1 << 0;
	else if (hs->hs_on)
		state = 1 << 1;
	else if (hs->mic_on)
		state = 1 << 2;
	else
		state = 0;
#if 0
	switch_set_state(&hs->sdev, state);
#endif
}

/*
 * tuple format: (key_code, key_param)
 *
 * old-architecture:
 * key-press = (key_code, 0)
 * key-release = (0xff, key_code)
 *
 * new-architecutre:
 * key-press = (key_code, 0)
 * key-release = (key_code, 0xff)
 */
static void report_hs_key(uint32_t key_code, uint32_t key_parm)
{
	int key, temp_key_code;

	if (key_code == HS_REL_K)
		key = hs_find_key(key_parm);
	else
		key = hs_find_key(key_code);

	temp_key_code = key_code;

	if (key_parm == HS_REL_K)
		key_code = key_parm;

	switch (key) {
	case KEY_POWER:
	case KEY_END:
#if 0
	case KEY_MEDIA:
	case KEY_VOLUMEUP:
	case KEY_VOLUMEDOWN:
#endif
#if 0
		input_report_key(hs->ipdev, key, (key_code != HS_REL_K));
#else
		if(pm_key)
		{
#if DEBUG
		switch (key) {
		case KEY_POWER:
			printk(KERN_INFO "[msm-handset] [%s] KEY_POWER[%d] key_code[%d]\n", __func__,key,key_code);
		break;
		case KEY_END:
			printk(KERN_INFO "[msm-handset] [%s] KEY_END[%d] key_code[%d]\n", __func__,key,key_code);
		break;
		}
#endif
#if defined(CONFIG_TENKEY_KEYPAD) && (CONFIG_TENKEY_KEYPAD_SUBID == 2)
		if(shext_hw_revision_cleck == true)
		{
			input_report_key(pm_key->ipdev, KEY_SAVE, (key_code != HS_REL_K));
		}else{
			input_report_key(pm_key->ipdev, key, (key_code != HS_REL_K));
			input_sync(pm_key->ipdev);
		}
#else
			input_report_key(pm_key->ipdev, key, (key_code != HS_REL_K));
			input_sync(pm_key->ipdev);
#endif
		}
		else
		{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] pm_key = NULL\n", __func__);
#endif
		}
#endif
		break;
#if 1
	case KEY_MEDIA:
#if 0
		report_headphone_switch(hs->ipdev, key, (key_code != HS_REL_K));
#else
		if(hssw)
		{
			report_headphone_switch(hssw->ipdev, key, (key_code != HS_REL_K));
			input_sync(hssw->ipdev);
		}
		else
		{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] hssw = NULL\n", __func__);
#endif
		}
#endif
		break;
#endif
	case SW_HEADPHONE_INSERT_W_MIC:
		hs->mic_on = hs->hs_on = (key_code != HS_REL_K) ? 1 : 0;
		input_report_switch(hs->ipdev, SW_HEADPHONE_INSERT,
							hs->hs_on);
		input_report_switch(hs->ipdev, SW_MICROPHONE_INSERT,
							hs->mic_on);
		update_state();
		break;

	case SW_HEADPHONE_INSERT:
#if 0
		report_headset_switch(hs->ipdev, key, (key_code != HS_REL_K));
#else
		if(hs)
		{
			report_headset_switch(hs->ipdev, key, key_parm);
#endif
#if 1
			input_sync(hs->ipdev);
#endif
		}
		else
		{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] hs = NULL\n", __func__);
#endif
		}
		hs->hs_on = (key_code != HS_REL_K) ? 1 : 0;
//		input_report_switch(hs->ipdev, key, hs->hs_on);
		update_state();
		break;
	case SW_MICROPHONE_INSERT:
		hs->mic_on = (key_code != HS_REL_K) ? 1 : 0;
		input_report_switch(hs->ipdev, key, hs->mic_on);
		update_state();
		break;
#ifdef FLIP_USE	/* FLIP */
	case SW_LID:
		if(hs)
		{
			if(report_flip_switch(hs->ipdev, key, key_parm) != 0)
			{
				return;
			}
#if 1
			input_sync(hs->ipdev);
#endif
		}
		else
		{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] hs = NULL\n", __func__);
#endif
		}
		break;
#endif
	case -1:
		printk(KERN_ERR "%s: No mapping for remote handset event %d\n",
				 __func__, temp_key_code);
		return;
	}
#if 0
	input_sync(hs->ipdev);
#endif
}

static int handle_hs_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	struct rpc_keypad_pass_key_code_args {
		uint32_t key_code;
		uint32_t key_parm;
	};

	switch (req->procedure) {
	case RPC_KEYPAD_NULL_PROC:
		return 0;

	case RPC_KEYPAD_PASS_KEY_CODE_PROC: {
		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code, args->key_parm);

		return 0;
	}

	case RPC_KEYPAD_SET_PWR_KEY_STATE_PROC:
		/* This RPC function must be available for the ARM9
		 * to function properly.  This function is redundant
		 * when RPC_KEYPAD_PASS_KEY_CODE_PROC is handled. So
		 * input_report_key is not needed.
		 */
		return 0;
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_SERVER_PROG,
	.vers		= HS_SERVER_VERS,
	.rpc_call	= handle_hs_rpc_call,
};

static int process_subs_srvc_callback(struct hs_event_cb_recv *recv)
{
	if (!recv)
		return -ENODATA;

	report_hs_key(be32_to_cpu(recv->key.code), be32_to_cpu(recv->key.parm));

	return 0;
}

static void process_hs_rpc_request(uint32_t proc, void *data)
{
	if (proc == HS_EVENT_CB_PROC)
		process_subs_srvc_callback(data);
	else
		pr_err("%s: unknown rpc proc %d\n", __func__, proc);
}

#if 0
static int hs_rpc_report_event_arg(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	struct hs_event_rpc_req {
		uint32_t hs_event_data_ptr;
		struct hs_event_data data;
	};

	struct hs_event_rpc_req *req = buffer;

	req->hs_event_data_ptr	= cpu_to_be32(0x1);
	req->data.ver		= cpu_to_be32(HS_EVENT_DATA_VER);
	req->data.event_type	= cpu_to_be32(HS_EVNT_HSD);
	req->data.enum_disc	= cpu_to_be32(HS_EVNT_HSD);
	req->data.data_length	= cpu_to_be32(0x1);
	req->data.data		= cpu_to_be32(*(enum hs_src_state *)data);
	req->data.data_size	= cpu_to_be32(sizeof(enum hs_src_state));

	return sizeof(*req);
}

static int hs_rpc_report_event_res(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	enum hs_return_value result;

	result = be32_to_cpu(*(enum hs_return_value *)buffer);
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	if (result == HS_SUCCESS)
		return 0;

	return 1;
}
#endif

void report_headset_status(bool connected)
{
#if 0
	int rc = -1;
	enum hs_src_state status;

	if (connected == true)
		status = HS_SRC_STATE_HI;
	else
		status = HS_SRC_STATE_LO;

	rc = msm_rpc_client_req(rpc_client, HS_REPORT_EVNT_PROC,
				hs_rpc_report_event_arg, &status,
				hs_rpc_report_event_res, NULL, -1);

	if (rc)
		pr_err("%s: couldn't send rpc client request\n", __func__);
#endif
}
EXPORT_SYMBOL(report_headset_status);

#if 0
static int hs_rpc_pwr_cmd_arg(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	struct hs_cmd_data_type *hs_pwr_cmd = buffer;

	hs_pwr_cmd->hs_cmd_data_type_ptr = cpu_to_be32(0x01);

	hs_pwr_cmd->ver = cpu_to_be32(0x03);
	hs_pwr_cmd->id = cpu_to_be32(HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD);
	hs_pwr_cmd->handle = cpu_to_be32(hs_subs_req->hs_handle_data);
	hs_pwr_cmd->disc_id1 = cpu_to_be32(HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD);
	hs_pwr_cmd->input_ptr = cpu_to_be32(0x01);
	hs_pwr_cmd->input_val = cpu_to_be32(hs->hs_pdata->pwr_key_delay_ms);
	hs_pwr_cmd->input_len = cpu_to_be32(0x01);
	hs_pwr_cmd->disc_id2 = cpu_to_be32(HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD);
	hs_pwr_cmd->output_len = cpu_to_be32(0x00);
	hs_pwr_cmd->delayed = cpu_to_be32(0x00);

	return sizeof(*hs_pwr_cmd);
}

static int hs_rpc_pwr_cmd_res(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	uint32_t result;

	result = be32_to_cpu(*((uint32_t *)buffer));
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}
#endif

static int hs_rpc_register_subs_arg(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	hs_subs_req = buffer;

	hs_subs_req->hs_subs_ptr	= cpu_to_be32(0x1);
	hs_subs_req->hs_subs.ver	= cpu_to_be32(0x1);
	hs_subs_req->hs_subs.srvc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	hs_subs_req->hs_subs.req	= cpu_to_be32(HS_SUBS_REGISTER);
	hs_subs_req->hs_subs.host_os	= cpu_to_be32(0x4); /* linux */
	hs_subs_req->hs_subs.disc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	hs_subs_req->hs_subs.id.evnt	= cpu_to_be32(HS_EVNT_CLASS_ALL);

	hs_subs_req->hs_cb_id		= cpu_to_be32(0x1);

	hs_subs_req->hs_handle_ptr	= cpu_to_be32(0x1);
	hs_subs_req->hs_handle_data	= cpu_to_be32(0x0);

	return sizeof(*hs_subs_req);
}

static int hs_rpc_register_subs_res(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	uint32_t result;

	result = be32_to_cpu(*((uint32_t *)buffer));
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}

static int hs_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	int rc = -1;

	struct rpc_request_hdr *hdr = buffer;

	hdr->type = be32_to_cpu(hdr->type);
	hdr->xid = be32_to_cpu(hdr->xid);
	hdr->rpc_vers = be32_to_cpu(hdr->rpc_vers);
	hdr->prog = be32_to_cpu(hdr->prog);
	hdr->vers = be32_to_cpu(hdr->vers);
	hdr->procedure = be32_to_cpu(hdr->procedure);

	process_hs_rpc_request(hdr->procedure,
			    (void *) (hdr + 1));

	msm_rpc_start_accepted_reply(client, hdr->xid,
				     RPC_ACCEPTSTAT_SUCCESS);
	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int __init hs_rpc_cb_init(void)
{
	int rc = 0, i, num_vers;

	num_vers = ARRAY_SIZE(rpc_vers);

	for (i = 0; i < num_vers; i++) {
		rpc_client = msm_rpc_register_client("hs",
			HS_RPC_PROG, rpc_vers[i], 0, hs_cb_func);

		if (IS_ERR(rpc_client))
			pr_debug("%s: RPC Client version %d failed, fallback\n",
				 __func__, rpc_vers[i]);
		else
			break;
	}

	if (IS_ERR(rpc_client)) {
		pr_err("%s: Incompatible RPC version error %ld\n",
			 __func__, PTR_ERR(rpc_client));
		return PTR_ERR(rpc_client);
	}

	rc = msm_rpc_client_req(rpc_client, HS_SUBSCRIBE_SRVC_PROC,
				hs_rpc_register_subs_arg, NULL,
				hs_rpc_register_subs_res, NULL, -1);
	if (rc) {
		pr_err("%s: RPC client request failed for subscribe services\n",
						__func__);
		goto err_client_req;
	}
	#if 0
	rc = msm_rpc_client_req(rpc_client, HS_PROCESS_CMD_PROC,
			hs_rpc_pwr_cmd_arg, NULL,
			hs_rpc_pwr_cmd_res, NULL, -1);
	if (rc)
		pr_err("%s: RPC client request failed for pwr key"
			" delay cmd, using normal mode\n", __func__);
	#endif
	return 0;
err_client_req:
	msm_rpc_unregister_client(rpc_client);
	return rc;
}

static int __devinit hs_rpc_init(void)
{
	int rc;

	rc = hs_rpc_cb_init();
	if (rc) {
		pr_err("%s: failed to initialize rpc client, try server...\n",
						__func__);
		}
		rc = msm_rpc_create_server(&hs_rpc_server);
		if (rc) {
			pr_err("%s: failed to create rpc server\n", __func__);
			return rc;
		}
	
	rpc_svc_p = msm_rpc_connect_compatible(HS_RPC_PROG,HS_RPC_VERS_1,0);

	if(IS_ERR(rpc_svc_p)) {
		pr_err("%s: couldn't connect compatible rpc client err %ld\n", __func__,
			 PTR_ERR(rpc_svc_p));
		rpc_svc_p = msm_rpc_connect_compatible(HS_RPC_PROG,HS_RPC_VERS_2,0);
	}

	if(IS_ERR(rpc_svc_p)) {
		pr_err("%s: couldn't connect compatible rpc client err %ld\n", __func__,
			 PTR_ERR(rpc_svc_p));
		return PTR_ERR(rpc_svc_p);
	}

	return rc;
}

static void __devexit hs_rpc_deinit(void)
{
	if (rpc_client)
		msm_rpc_unregister_client(rpc_client);
}

static ssize_t msm_headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs->sdev)) {
#if 0
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case MSM_HEADSET:
		return sprintf(buf, "Headset\n");
#else
	case NO_DEVICE:
		return sprintf(buf, "NoDevice\n");
	case MSM_HEADSET_MONO:
		return sprintf(buf, "MonoHeadset\n");
	case MSM_HEADSET_OTHER:
	case MSM_HEADSET_STE:
		return sprintf(buf, "StereoHeadset\n");
#endif
	}
	return -EINVAL;
}

#ifdef FLIP_USE	/* FLIP */
static ssize_t msm_flip_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs->sdev_flip)) {
	case MSM_OPEN:
		return sprintf(buf, "OPEN\n");
	case MSM_CLOSE:
		return sprintf(buf, "CLOSE\n");
#ifdef SWIVEL_USE
	case MSM_SWIVEL_CLOSE:
		return sprintf(buf, "SWIVEL_CLOSE\n");
#endif
	}
	return -EINVAL;
}

static ssize_t msm_flip_chatt_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs->sdev_flip_chatt)) {
	case MSM_OPEN:
		return sprintf(buf, "OPEN\n");
	case MSM_CLOSE:
		return sprintf(buf, "CLOSE\n");
#ifdef SWIVEL_USE
	case MSM_SWIVEL_CLOSE:
		return sprintf(buf, "SWIVEL_CLOSE\n");
#endif
	case MSM_OPEN_CHATT:
		return sprintf(buf, "OPEN_CHATT\n");
	case MSM_CLOSE_CHATT:
		return sprintf(buf, "CLOSE_CHATT\n");
	}
	return -EINVAL;
}
#endif

#if 1
static ssize_t msm_hssw_print_name(struct switch_dev *sdev, char *buf)
{
#if 0
	switch (switch_get_state(&hs->sdev_hssw)) {
	case MSM_HSSW_RELEASE:
		return sprintf(buf, "RELEASE\n");
	case MSM_HSSW_PRESS:
		return sprintf(buf, "PRESS\n");
	}
	return -EINVAL;
#else
	switch (switch_get_state(&hssw->sdev_hssw)) {
	case MSM_HSSW_RELEASE:
		return sprintf(buf, "RELEASE\n");
	case MSM_HSSW_PRESS:
		return sprintf(buf, "PRESS\n");
	}
	return -EINVAL;
#endif
}
#endif

static int __devinit hs_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct input_dev *ipdev;

#if 1
	struct api_remote_req_t1 {
		struct rpc_request_hdr hdr;
	} send_p;
#endif

	hs = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	hs->sdev.name	= "h2w";
	hs->sdev.print_name = msm_headset_print_name;

	rc = switch_dev_register(&hs->sdev);
	if (rc)
		goto err_switch_dev_register;

#ifdef FLIP_USE	/* FLIP */
	hs->sdev_flip.name	= "flip";
	hs->sdev_flip.print_name = msm_flip_print_name;
	
	rc = switch_dev_register(&hs->sdev_flip);
	if (rc)
		goto err_switch_dev_register;
		
	hs->sdev_flip_chatt.name	= "flip_chatt";
	hs->sdev_flip_chatt.print_name = msm_flip_chatt_print_name;
	
	rc = switch_dev_register(&hs->sdev_flip_chatt);
	if (rc)
		goto err_switch_dev_register;
#endif
#if 0	
	hs->sdev_hssw.name	= "headphone_switch";
	hs->sdev_hssw.print_name = msm_hssw_print_name;
	
	rc = switch_dev_register(&hs->sdev_hssw);
	if (rc)
		goto err_switch_dev_register;
#endif
	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hs);

	hs->ipdev = ipdev;

	if (pdev->dev.platform_data)
		hs->hs_pdata = pdev->dev.platform_data;

	if (hs->hs_pdata->hs_name)
		ipdev->name = hs->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_NAME;


	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

#if 0
	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(ipdev, EV_SW, SW_MICROPHONE_INSERT);
	input_set_capability(ipdev, EV_KEY, KEY_POWER);
	input_set_capability(ipdev, EV_KEY, KEY_END);
#else
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_INSERT);
#endif
#ifdef FLIP_USE	/* FLIP */
	input_set_capability(ipdev, EV_SW, SW_LID);
	
#ifdef CONFIG_PERF_LOCK_ENABLE
	perf_lock_init(&flip_perf_lock, PERF_LOCK_HIGHEST, "Flip");
#endif
	
#endif

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hs);

	rc = hs_rpc_init();
	if (rc) {
		dev_err(&ipdev->dev, "rpc init failure\n");
		goto err_hs_rpc_init;
	}

#if 1
	rc = msm_rpc_call_reply(rpc_svc_p,
							HS_SHEXTDET_API_INITIALIZE_REMOTE_PROC,
							&send_p,sizeof(send_p),
							NULL,0,
							5 * HZ);
	if(rc)
		goto err_hs_rpc_init;
#endif

	return 0;

err_hs_rpc_init:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	switch_dev_unregister(&hs->sdev);
#ifdef FLIP_USE	/* FLIP */
	switch_dev_unregister(&hs->sdev_flip);
	switch_dev_unregister(&hs->sdev_flip_chatt);
#endif
#if 0
	switch_dev_unregister(&hs->sdev_hssw);
#endif
err_switch_dev_register:
	kfree(hs);
	return rc;
}

static int __devexit hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hs = platform_get_drvdata(pdev);

	input_unregister_device(hs->ipdev);
	switch_dev_unregister(&hs->sdev);
#ifdef FLIP_USE	/* FLIP */
	switch_dev_unregister(&hs->sdev_flip);
	switch_dev_unregister(&hs->sdev_flip_chatt);
#endif
#if 0
	switch_dev_unregister(&hs->sdev_hssw);
#endif
	kfree(hs);
	hs_rpc_deinit();
	return 0;
}

#if 1
static int __devinit sh_pm_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;

	pm_key = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!pm_key)
		return -ENOMEM;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, pm_key);

	pm_key->ipdev = ipdev;

	if (pdev->dev.platform_data)
		pm_key->hs_pdata = pdev->dev.platform_data;

	if (pm_key->hs_pdata->hs_name)
		ipdev->name = pm_key->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_SH_PM_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_POWER);
	input_set_capability(ipdev, EV_KEY, KEY_END);
#if defined(CONFIG_TENKEY_KEYPAD) && (CONFIG_TENKEY_KEYPAD_SUBID == 2)
	if((sh_boot_get_hw_revision()==0x0002) || (sh_boot_get_hw_revision()==0x0003))
	{
		input_set_capability(ipdev, EV_KEY, KEY_SAVE);
		shext_hw_revision_cleck = true;
	}
#endif
	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, pm_key);

	return 0;

err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	kfree(pm_key);
	pm_key = NULL;
	return rc;
}

static int __devexit sh_pm_remove(struct platform_device *pdev)
{
	struct msm_handset *pm_key_data = platform_get_drvdata(pdev);

	input_unregister_device(pm_key_data->ipdev);

	kfree(pm_key_data);
	return 0;
}

static int __devinit sh_hs_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;

	hssw = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hssw)
		return -ENOMEM;

	hssw->sdev_hssw.name	= "headphone_switch";
	hssw->sdev_hssw.print_name = msm_hssw_print_name;
	
	rc = switch_dev_register(&hssw->sdev_hssw);
	if (rc)
		goto err_switch_dev_register;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hssw);

	hssw->ipdev = ipdev;

	if (pdev->dev.platform_data)
		hssw->hs_pdata = pdev->dev.platform_data;

	if (hssw->hs_pdata->hs_name)
		ipdev->name = hssw->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_SH_HS_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hssw);

	return 0;

err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	switch_dev_unregister(&hssw->sdev_hssw);
err_switch_dev_register:
	kfree(hssw);
	hssw = NULL;
	return rc;
}

static int __devexit sh_hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hssw_data = platform_get_drvdata(pdev);

	input_unregister_device(hssw_data->ipdev);
	switch_dev_unregister(&hssw_data->sdev_hssw);

	kfree(hssw_data);
	return 0;
}
#endif

static struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};
#if 1
static struct platform_driver sh_pm_driver = {
	.probe		= sh_pm_probe,
	.remove		= __devexit_p(sh_pm_remove),
	.driver		= {
		.name	= DRIVER_SH_PM_NAME,
		.owner	= THIS_MODULE,
	},
};
static struct platform_driver sh_hs_driver = {
	.probe		= sh_hs_probe,
	.remove		= __devexit_p(sh_hs_remove),
	.driver		= {
		.name	= DRIVER_SH_HS_NAME,
		.owner	= THIS_MODULE,
	},
};
#endif

#if 1
static int headset_diag_open(struct inode *inode, struct file *file)
{
#if 0
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int headset_diag_release(struct inode *inode, struct file *file)
{
#if 0
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return 0;
}

static ssize_t headset_diag_read(struct file *file, char __user *buf,
			   size_t count, loff_t *ppos)
{
	char data[4];
	memset(data, 0x00, sizeof(data));
#if 0
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	sprintf(data, "%d\n",Headset_Status);

	if(copy_to_user(buf, &data, sizeof(&data)))
	{
		printk(KERN_ERR "%s: copy_to_user Error\n", __func__);
		return -EFAULT;
	}
	return strlen(data);
}

#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
static int hs_wake_unlock(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	if( cmd == SHEXTDET_WAKE_UNLOCK ){
		if( shext_hs_wake_lock_init ){
			if( shext_hs_wake_lock_count > 0 ){
				shext_hs_wake_lock_count--;
				if( shext_hs_wake_lock_count == 0 ){
					wake_unlock(&shext_hs_suspend_wake_lock);
					wake_unlock(&shext_hs_idle_wake_lock);
					#if DEBUG
					printk(KERN_ERR
					   "[msm-handset][%s]wake_unlock hs_wake_unlock Call\n", __func__);
					#endif
				}else{
					#if DEBUG
					printk(KERN_ERR
					   "[msm-handset][%s]wake_unlock hs_wake_unlock Use:%d\n", __func__,shext_hs_wake_lock_count);
					#endif                 
				}
			}else{
				#if DEBUG
				printk(KERN_ERR
					"[msm-handset][%s]wake_unlock not use:%d\n", __func__,shext_hs_wake_lock_count);	     
				#endif
			 }
		}else{
			#if DEBUG
			if( !shext_hs_wake_lock_init )
				printk(KERN_ERR
					"[msm-handset][%s]wake_unlock error! noinit\n", __func__);
			#endif		
		}
	}else if( cmd == SHEXTDET_WAKE_LOCK_START ){
		shext_hs_wake_lock_start_flg = true;
		#if DEBUG
		printk(KERN_INFO "[msm-handset] [%s] shext_hs_wake_lock_start_flg set true\n", __func__);
		#endif		
	}
	return 0;
}

static struct file_operations headset_diag_fops = {
	.owner = THIS_MODULE,
	.open    = headset_diag_open,
	.release = headset_diag_release,
	.read    = headset_diag_read,
	.ioctl = hs_wake_unlock,
};
#else
static struct file_operations headset_diag_fops = {
	.owner = THIS_MODULE,
	.open    = headset_diag_open,
	.release = headset_diag_release,
	.read    = headset_diag_read,
};
#endif	/* Wake_Lock <- */

static struct miscdevice headset_diag_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "headset_diag",
	.fops = &headset_diag_fops,
};
#endif

static int __init hs_init(void)
{
	int err = -ENODEV;

#if 0
	return platform_driver_register(&hs_driver);
#else
	int rc;
	if(0 != (rc = platform_driver_register(&sh_pm_driver))) {
		return rc;
	}
	if(0 != (rc = platform_driver_register(&sh_hs_driver))) {
		platform_driver_unregister(&sh_pm_driver);
		return rc;
	}
	if(0 != (rc = platform_driver_register(&hs_driver))) {
		platform_driver_unregister(&sh_pm_driver);
		platform_driver_unregister(&sh_hs_driver);
		return rc;
	}
	err = misc_register(&headset_diag_device);
	if (err) {
		printk(KERN_ERR
		       "headset_diag_device: register failed\n");
	}
	return rc;
#endif
}
late_initcall(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
#if 1
	platform_driver_unregister(&sh_pm_driver);
	platform_driver_unregister(&sh_hs_driver);
	misc_deregister(&headset_diag_device);
#endif
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm-handset");
