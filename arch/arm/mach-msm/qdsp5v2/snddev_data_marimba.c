/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/uaccess.h>
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/marimba_profile.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/qdsp5v2/snddev_virtual.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/qdsp5v2/snddev_mi2s.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_acdb_def.h>
#ifdef CONFIG_SH_AUDIO_DRIVER
#include <sharp/shspamp.h>
#endif /* CONFIG_SH_AUDIO_DRIVER */

/* define the value for BT_SCO */
#define BT_SCO_PCM_CTL_VAL (PCM_CTL__RPCM_WIDTH__LINEAR_V |\
				PCM_CTL__TPCM_WIDTH__LINEAR_V)
#define BT_SCO_DATA_FORMAT_PADDING (DATA_FORMAT_PADDING_INFO__RPCM_FORMAT_V |\
				DATA_FORMAT_PADDING_INFO__TPCM_FORMAT_V)
#define BT_SCO_AUX_CODEC_INTF   AUX_CODEC_INTF_CTL__PCMINTF_DATA_EN_V

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_hsed_config;
static void snddev_hsed_config_modify_setting(int type);
static void snddev_hsed_config_restore_setting(void);
#endif

#ifdef CONFIG_SH_AUDIO_DRIVER
#if defined( CONFIG_SHEXTDEV )
#define SH_AUDIO_HPAMP_CAPLESS_ENABLE

#if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
extern void headset_vreg(unsigned long onoff);
#endif /* #if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */

static void snddev_hsed_speaker_voltage_on(void)
{
#if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	headset_vreg(1);
#endif /* #if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
	msm_snddev_hsed_voltage_on();
}

static void snddev_hsed_speaker_voltage_off(void)
{
#if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	headset_vreg(0);
#endif /* #if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
	msm_snddev_hsed_voltage_off();
}

static void snddev_hsed_voltage_on(void)
{
#if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	headset_vreg(1);
#endif /* #if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
}

static void snddev_hsed_voltage_off(void)
{
#if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	headset_vreg(0);
#endif /* #if defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
}
#endif /* #if defined( CONFIG_SHEXTDEV ) */
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
	HANDSET_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER ==  203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1700,-1200,-700,-200,200},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2200,-1700,-1200,-700,-200,200},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 302)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 303)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 304)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 305)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 306)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 307)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-1700,-1300,-900,-500,-100,300},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-1700,-1300,-900,-500,-100,300},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-1700,-1400,-1100,-800,-500,-200},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-1700,-1400,-1100,-800,-500,-200},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit iearpiecemute_48KHz_osr256_actions[] =
	HANDSET_RXMUTE_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiecemute_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiecemute_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiecemute_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiecemute_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiecemute_settings,
	.setting_sz = ARRAY_SIZE(iearpiecemute_settings),
};

static struct snddev_icodec_data snddev_iearpiecemute_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx_mute",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiecemute_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = 0,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000
};

static struct platform_device msm_iearpiecemute_device = {
	.name = "snddev_icodec",
	.id = 33,
	.dev = { .platform_data = &snddev_iearpiecemute_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_action_unit imic_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256;

static struct adie_codec_action_unit imic_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256;

static struct adie_codec_action_unit imic_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static enum hsed_controller imic_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_device = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit imicmute_48KHz_osr256_actions[] =
	HANDSET_TXMUTE_48000_OSR_256;

static struct adie_codec_hwsetting_entry imicmute_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imicmute_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imicmute_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imicmute_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imicmute_settings,
	.setting_sz = ARRAY_SIZE(imicmute_settings),
};


static struct snddev_icodec_data snddev_imicmute_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx_mute",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imicmute_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imicmute_device = {
	.name = "snddev_icodec",
	.id = 31,
	.dev = { .platform_data = &snddev_imicmute_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions[] =
#ifdef CONFIG_SH_AUDIO_DRIVER
#if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;
#else
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256;
#endif /* #if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
#else
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifndef CONFIG_SH_AUDIO_DRIVER
	.property = SIDE_TONE_MASK,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if defined( CONFIG_SHEXTDEV )
	.voltage_on = snddev_hsed_voltage_on,
	.voltage_off = snddev_hsed_voltage_off,
#endif /* #if defined( CONFIG_SHEXTDEV ) */
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 302)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 303)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 305)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 307)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2000,-1600,-1200,-800,-400,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2000,-1600,-1200,-800,-400,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 304)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 306)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2300,-1900,-1500,-1100,-700,-300},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2300,-1900,-1500,-1100,-700,-300},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1900,-1600,-1300,-1000,-700},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2400,-2100,-1800,-1500,-1200,-900},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_ihs_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data },
};

static struct adie_codec_action_unit ihs_mono_rx_48KHz_osr256_actions[] =
#ifdef CONFIG_SH_AUDIO_DRIVER
#if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	HEADSET_RX_LEGACY_48000_OSR_256;
#else
	HEADSET_RX_CAPLESS_48000_OSR_256;
#endif /* #if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
#else
	HEADSET_RX_LEGACY_48000_OSR_256;
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_hwsetting_entry ihs_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifndef CONFIG_SH_AUDIO_DRIVER
	.property = SIDE_TONE_MASK,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if defined( CONFIG_SHEXTDEV )
	.voltage_on = snddev_hsed_voltage_on,
	.voltage_off = snddev_hsed_voltage_off,
#endif /* #if defined( CONFIG_SHEXTDEV ) */
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1900,-1600,-1300,-1000,-700},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2400,-2100,-1800,-1500,-1200,-900},
#else
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
#endif /* CONFIG_SH_AUDIO_DRIVER */
};

static struct platform_device msm_ihs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 3,
	.dev = { .platform_data = &snddev_ihs_mono_rx_data },
};

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_action_unit ihs_ffa_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ffa_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ffa_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ffa_stereo_rx_48KHz_osr256_actions),
	}
};
#endif /* #ifdef CONFIG_DEBUG_FS */

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_action_unit
	ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CLASS_D_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_ffa_stereo_rx_class_d_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions),
	}
};

static struct adie_codec_action_unit
	ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_ffa_stereo_rx_class_ab_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions),
	}
};
#endif

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_dev_profile ihs_ffa_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ffa_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ffa_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ffa_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_ffa_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1900,-1600,-1300,-1000,-700},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2400,-2100,-1800,-1500,-1200,-900},
#else
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
#endif /* CONFIG_SH_AUDIO_DRIVER */

};

static struct platform_device msm_ihs_ffa_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 4,
	.dev = { .platform_data = &snddev_ihs_ffa_stereo_rx_data },
};
#endif /* #ifdef CONFIG_DEBUG_FS */

#ifndef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ihs_ffa_mono_rx_48KHz_osr256_actions[] =
	HEADSET_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ffa_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ffa_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ffa_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_ffa_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ffa_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ffa_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ffa_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_ffa_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
};

static struct platform_device msm_ihs_ffa_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 5,
	.dev = { .platform_data = &snddev_ihs_ffa_mono_rx_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions[] =
	HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions[] =
	HEADSET_MONO_TX_16000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions[] =
	HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data },
};

static struct adie_codec_action_unit ifmradio_handset_osr64_actions[] =
	FM_HANDSET_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_handset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_handset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_handset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_handset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_handset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_handset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_handset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_handset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_handset_device = {
	.name = "snddev_icodec",
	.id = 7,
	.dev = { .platform_data = &snddev_ifmradio_handset_data },
};


static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions[] =
   SPEAKER_STEREO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_rx_profile,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.channel_mode = 1,
#else
	.channel_mode = 2,
#endif /* CONFIG_SH_AUDIO_DRIVER */
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.pamp_on = &shspamp_poweron,
	.pamp_off = &shspamp_poweroff,
#else
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 302)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 303)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 304)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 305)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 306)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 307)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2000,-1600,-1200,-800,-400,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2000,-1600,-1200,-800,-400,0},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-500,-200,100,400,700,1000},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-500,-200,100,400,700,1000},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data },

};

static struct adie_codec_action_unit ifmradio_speaker_osr64_actions[] =
	FM_SPEAKER_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_speaker_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_speaker_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_speaker_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_speaker_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_speaker_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_speaker_settings),
};

static struct snddev_icodec_data snddev_ifmradio_speaker_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_speaker_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_speaker_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.pamp_on = &shspamp_poweron,
	.pamp_off = &shspamp_poweroff,
#else
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
#endif /* CONFIG_SH_AUDIO_DRIVER */
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_speaker_device = {
	.name = "snddev_icodec",
	.id = 9,
	.dev = { .platform_data = &snddev_ifmradio_speaker_data },
};

static struct adie_codec_action_unit ifmradio_headset_osr64_actions[] =
	FM_HEADSET_STEREO_CLASS_D_LEGACY_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_headset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_headset_device = {
	.name = "snddev_icodec",
	.id = 10,
	.dev = { .platform_data = &snddev_ifmradio_headset_data },
};

#ifndef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ifmradio_ffa_headset_osr64_actions[] =
	FM_HEADSET_CLASS_AB_STEREO_CAPLESS_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_ffa_headset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_ffa_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_ffa_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_ffa_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_ffa_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_ffa_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_ffa_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_ffa_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_ffa_headset_device = {
	.name = "snddev_icodec",
	.id = 11,
	.dev = { .platform_data = &snddev_ifmradio_ffa_headset_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct snddev_ecodec_data snddev_bt_sco_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.voice_rx_vol_step[VOC_NB_INDEX] = {-1100,-800,-500,-200,100,400},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-1100,-800,-500,-200,100,400},
#else
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
#endif /* CONFIG_SH_AUDIO_DRIVER */
};

static struct snddev_ecodec_data snddev_bt_sco_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_MIC,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};

struct platform_device msm_bt_sco_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data },
};

struct platform_device msm_bt_sco_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 1,
	.dev = { .platform_data = &snddev_bt_sco_mic_data },
};

#ifndef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit idual_mic_endfire_8KHz_osr256_actions[] =
	MIC1_LEFT_LINE_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_endfire_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_endfire_settings),
};

static enum hsed_controller idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_ENDFIRE,
	.profile = &idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 12,
	.dev = { .platform_data = &snddev_idual_mic_endfire_data },
};


static struct snddev_icodec_data\
		snddev_idual_mic_endfire_real_stereo_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx_real_stereo",
	.copp_id = 0,
	.acdb_id = PSEUDO_ACDB_ID,
	.profile = &idual_mic_endfire_profile,
	.channel_mode = REAL_STEREO_CHANNEL_MODE,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_real_stereo_tx_device = {
	.name = "snddev_icodec",
	.id = 26,
	.dev = { .platform_data =
			&snddev_idual_mic_endfire_real_stereo_data },
};

static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings),
};

static enum hsed_controller idual_mic_broadside_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data },
};

static struct adie_codec_action_unit ispk_dual_mic_ef_8KHz_osr256_actions[] =
	SPEAKER_MIC1_LEFT_LINE_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispk_dual_mic_ef_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16Khz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ispk_dual_mic_ef_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispk_dual_mic_ef_settings,
	.setting_sz = ARRAY_SIZE(ispk_dual_mic_ef_settings),
};

static struct snddev_icodec_data snddev_spk_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &ispk_dual_mic_ef_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_spk_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 14,
	.dev = { .platform_data = &snddev_spk_idual_mic_endfire_data },
};

static struct adie_codec_action_unit ispk_dual_mic_bs_8KHz_osr256_actions[] =
	SPEAKER_MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispk_dual_mic_bs_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16Khz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ispk_dual_mic_bs_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispk_dual_mic_bs_settings,
	.setting_sz = ARRAY_SIZE(ispk_dual_mic_bs_settings),
};
static struct snddev_icodec_data snddev_spk_idual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_BROADSIDE,
	.profile = &ispk_dual_mic_bs_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_spk_idual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 15,
	.dev = { .platform_data = &snddev_spk_idual_mic_broadside_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_action_unit itty_hs_mono_tx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_tx_settings[] = {
	/* 8KHz, 16KHz, 48KHz TTY Tx devices can shared same set of actions */
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = itty_hs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_MIC,
	.profile = &itty_hs_mono_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_itty_hs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 16,
	.dev = { .platform_data = &snddev_itty_hs_mono_tx_data },
};

static struct adie_codec_action_unit itty_hs_mono_rx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_8000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_16KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_16000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_48KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_48000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_rx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = itty_hs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_SPKR,
	.profile = &itty_hs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.voice_rx_vol_step[VOC_NB_INDEX] = {0,0,0,0,0,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {0,0,0,0,0,0},
#else
	.max_voice_rx_vol[VOC_NB_INDEX] = 0,
	.min_voice_rx_vol[VOC_NB_INDEX] = 0,
	.max_voice_rx_vol[VOC_WB_INDEX] = 0,
	.min_voice_rx_vol[VOC_WB_INDEX] = 0,
#endif /* CONFIG_SH_AUDIO_DRIVER */
};

static struct platform_device msm_itty_hs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 17,
	.dev = { .platform_data = &snddev_itty_hs_mono_rx_data },
};

static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions[] =
	SPEAKER_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions[] =
	SPEAKER_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings),
};

static enum hsed_controller ispk_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data },
};

#ifndef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit iearpiece_ffa_48KHz_osr256_actions[] =
	HANDSET_RX_48000_OSR_256_FFA;

static struct adie_codec_hwsetting_entry iearpiece_ffa_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_ffa_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_ffa_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_ffa_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_ffa_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_ffa_settings),
};

static struct snddev_icodec_data snddev_iearpiece_ffa_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_ffa_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -1400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2900,
};

static struct platform_device msm_iearpiece_ffa_device = {
	.name = "snddev_icodec",
	.id = 19,
	.dev = { .platform_data = &snddev_iearpiece_ffa_data },
};

static struct adie_codec_action_unit imic_ffa_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256_FFA;

static struct adie_codec_action_unit imic_ffa_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256_FFA;

static struct adie_codec_action_unit imic_ffa_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256_FFA;

static struct adie_codec_hwsetting_entry imic_ffa_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_ffa_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_ffa_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_ffa_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_ffa_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_ffa_settings,
	.setting_sz = ARRAY_SIZE(imic_ffa_settings),
};

static struct snddev_icodec_data snddev_imic_ffa_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_ffa_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_ffa_device = {
	.name = "snddev_icodec",
	.id = 20,
	.dev = { .platform_data = &snddev_imic_ffa_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_SPEAKER_STEREO_RX_CAPLESS_48000_OSR_256;


static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.channel_mode = 1,
#else
	.channel_mode = 2,
#endif /* CONFIG_SH_AUDIO_DRIVER */
	.default_sample_rate = 48000,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.pamp_on = &shspamp_poweron,
	.pamp_off = &shspamp_poweroff,
#else
	.pamp_on = msm_snddev_poweramp_on,
	.pamp_off = msm_snddev_poweramp_off,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if defined( CONFIG_SHEXTDEV )
	.voltage_on = snddev_hsed_speaker_voltage_on,
	.voltage_off = snddev_hsed_speaker_voltage_off,
#else
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
#endif /* #if defined( CONFIG_SHEXTDEV ) */
#else
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2000,-1700,-1400,-1100,-800,-500},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2000,-1700,-1400,-1100,-800,-500},
#else
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
#endif /* CONFIG_SH_AUDIO_DRIVER */
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data },
};

static struct snddev_mi2s_data snddev_mi2s_stereo_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "hdmi_stereo_rx",
	.copp_id = 3,
	.acdb_id = ACDB_ID_HDMI,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_0,
	.route = msm_snddev_tx_route_config,
	.deroute = msm_snddev_tx_route_deconfig,
	.default_sample_rate = 48000,
};

static struct platform_device msm_snddev_mi2s_stereo_rx_device = {
	.name = "snddev_mi2s",
	.id = 0,
	.dev = { .platform_data = &snddev_mi2s_stereo_rx_data },
};


static struct snddev_mi2s_data snddev_mi2s_fm_tx_data = {
	.capability = SNDDEV_CAP_TX ,
	.name = "fmradio_stereo_tx",
	.copp_id = 2,
	.acdb_id = ACDB_ID_FM_TX,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_3,
	.route = NULL,
	.deroute = NULL,
	.default_sample_rate = 48000,
};

static struct platform_device  msm_snddev_mi2s_fm_tx_device = {
	.name = "snddev_mi2s",
	.id = 1,
	.dev = { .platform_data = &snddev_mi2s_fm_tx_data},
};

#ifndef CONFIG_SH_AUDIO_DRIVER
static struct snddev_icodec_data snddev_fluid_imic_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_imic_tx_device = {
	.name = "snddev_icodec",
	.id = 22,
	.dev = { .platform_data = &snddev_fluid_imic_tx_data },
};

static struct snddev_icodec_data snddev_fluid_iearpiece_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1000,
};

static struct platform_device msm_fluid_iearpeice_rx_device = {
	.name = "snddev_icodec",
	.id = 23,
	.dev = { .platform_data = &snddev_fluid_iearpiece_rx_data },
};

static struct adie_codec_action_unit fluid_idual_mic_ef_8KHz_osr256_actions[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry fluid_idual_mic_endfire_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can also be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile fluid_idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = fluid_idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(fluid_idual_mic_endfire_settings),
};

static enum hsed_controller fluid_idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_fluid_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &fluid_idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = fluid_idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(fluid_idual_mic_endfire_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 24,
	.dev = { .platform_data = &snddev_fluid_idual_mic_endfire_data },
};

static struct snddev_icodec_data snddev_fluid_spk_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &fluid_idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = fluid_idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(fluid_idual_mic_endfire_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_spk_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 25,
	.dev = { .platform_data = &snddev_fluid_spk_idual_mic_endfire_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct snddev_virtual_data snddev_a2dp_tx_data = {
	.capability = SNDDEV_CAP_TX,
	.name = "a2dp_tx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct snddev_virtual_data snddev_a2dp_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "a2dp_rx",
	.copp_id = 2,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_a2dp_rx_device = {
	.name = "snddev_virtual",
	.id = 0,
	.dev = { .platform_data = &snddev_a2dp_rx_data },
};

static struct platform_device msm_a2dp_tx_device = {
	.name = "snddev_virtual",
	.id = 1,
	.dev = { .platform_data = &snddev_a2dp_tx_data },
};

static struct snddev_virtual_data snddev_uplink_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "uplink_rx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_uplink_rx_device = {
	.name = "snddev_virtual",
	.id = 2,
	.dev = { .platform_data = &snddev_uplink_rx_data },
};

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit iearpiece_lb_8KHz_osr256_actions[] =
	HANDSET_LB_RX_8000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_lb_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = iearpiece_lb_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_lb_8KHz_osr256_actions),
	}
};
static struct adie_codec_dev_profile iearpiece_lb_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_lb_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_lb_settings),
};

static struct snddev_icodec_data snddev_iearpiece_lb_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_lb_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_lb_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.voice_rx_vol_step[VOC_NB_INDEX] = {-1700,-1400,-1100,-800,-500,-200},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-1700,-1400,-1100,-800,-500,-200},
#else
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
#endif /* CONFIG_SH_AUDIO_DRIVER */
};

static struct platform_device msm_iearpiece_lb_device = {
	.name = "snddev_icodec",
	.id = 27,
	.dev = { .platform_data = &snddev_iearpiece_lb_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit imic_lb_8KHz_osr256_actions[] =
	HANDSET_LB_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry imic_lb_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_lb_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_lb_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile imic_lb_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_lb_settings,
	.setting_sz = ARRAY_SIZE(imic_lb_settings),
};

static struct snddev_icodec_data snddev_imic_lb_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_lb_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_lb_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_lb_device = {
	.name = "snddev_icodec",
	.id = 28,
	.dev = { .platform_data = &snddev_imic_lb_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ihs_stereo_lb_tx_8KHz_osr256_actions[] =
	HEADSET_LB_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_stereo_lb_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_stereo_lb_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_lb_tx_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ihs_stereo_lb_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_stereo_lb_tx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_lb_tx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_lb_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_lb_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_stereo_lb_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_ihs_stereo_lb_tx_device = {
	.name = "snddev_icodec",
	.id = 29,
	.dev = { .platform_data = &snddev_ihs_stereo_lb_tx_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit imic_lb_spkr_8KHz_osr256_actions[] =
	HANDSET_LB_SPKR_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry imic_lb_spkr_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_lb_spkr_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_lb_spkr_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile imic_lb_spkr_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_lb_spkr_settings,
	.setting_sz = ARRAY_SIZE(imic_lb_spkr_settings),
};

static struct snddev_icodec_data snddev_imic_lb_spkr_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_lb_spkr_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_lb_spkr_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_lb_spkr_device = {
	.name = "snddev_icodec",
	.id = 30,
	.dev = { .platform_data = &snddev_imic_lb_spkr_data },
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ispeaker_builtin_mic_tx_8KHz_osr256_actions[] =
	SPEAKER_BUILTIN_MIC_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_builtin_mic_tx_48KHz_osr256_actions[] =
	SPEAKER_BUILTIN_MIC_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_builtin_mic_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_builtin_mic_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_builtin_mic_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_builtin_mic_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_builtin_mic_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_builtin_mic_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_builtin_mic_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_builtin_mic_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_builtin_mic_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_builtin_mic_tx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_builtin_mic_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_builtin_mic_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_BUILTIN_MIC,
	.profile = &ispeaker_builtin_mic_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_builtin_mic_tx_device = {
	.name = "snddev_icodec",
	.id = 32,
	.dev = { .platform_data = &snddev_ispeaker_builtin_mic_tx_data },
};

/* speaker in communication path path [start] */
static struct adie_codec_action_unit ispeaker_in_communication_tx_8KHz_osr256_actions[] =
	SPEAKER_IN_COMMUNICATION_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_in_communication_tx_48KHz_osr256_actions[] =
	SPEAKER_IN_COMMUNICATION_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_in_communication_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_in_communication_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_in_communication_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_in_communication_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_in_communication_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_in_communication_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_in_communication_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_in_communication_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_in_communication_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_in_communication_tx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_in_communication_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_in_communication_mic_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_IN_COMMUNICATION,
	.profile = &ispeaker_in_communication_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_in_communication_tx_device = {
	.name = "snddev_icodec",
	.id = 34,
	.dev = { .platform_data = &snddev_ispeaker_in_communication_tx_data },
};
/* speaker in communication path path [end] */
#endif /* CONFIG_SH_AUDIO_DRIVER */

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ihs_stereo_lb_rx_48KHz_osr256_actions[] =
#ifdef CONFIG_SH_AUDIO_DRIVER
#if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	HEADSET_LB_STEREO_RX_LEGACY_48000_OSR_256;
#else
	HEADSET_LB_STEREO_RX_CAPLESS_48000_OSR_256;
#endif /* #if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
#else
	HEADSET_LB_STEREO_RX_LEGACY_48000_OSR_256;
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_hwsetting_entry ihs_stereo_lb_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_lb_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_lb_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_lb_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_lb_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_lb_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_lb_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_lb_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_lb_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifdef CONFIG_SH_AUDIO_DRIVER
#if defined( CONFIG_SHEXTDEV )
	.voltage_on = snddev_hsed_voltage_on,
	.voltage_off = snddev_hsed_voltage_off,
#endif /* #if defined( CONFIG_SHEXTDEV ) */
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1900,-1600,-1300,-1000,-700},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2400,-2100,-1800,-1500,-1200,-900},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_ihs_stereo_lb_rx_device = {
	.name = "snddev_icodec",
	.id = 35,
	.dev = { .platform_data = &snddev_ihs_stereo_lb_rx_data },
};
#endif /*CONFIG_SH_AUDIO_DRIVER*/

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ispeaker_lb_rx_48KHz_osr256_actions[] =
   SPEAKER_LB_STEREO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_lb_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_lb_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_lb_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_lb_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_lb_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_lb_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_lb_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_lb_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_lb_rx_profile,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.channel_mode = 1,
#else
	.channel_mode = 2,
#endif /* CONFIG_SH_AUDIO_DRIVER */
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.pamp_on = &shspamp_poweron,
	.pamp_off = &shspamp_poweroff,
#else
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-500,-200,100,400,700,1000},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-500,-200,100,400,700,1000},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_ispeaker_lb_rx_device = {
	.name = "snddev_icodec",
	.id = 36,
	.dev = { .platform_data = &snddev_ispeaker_lb_rx_data },

};
#endif /*CONFIG_SH_AUDIO_DRIVER*/

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ispeaker_melo_rx_48KHz_osr256_actions[] =
   SPEAKER_MELO_STEREO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_melo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_melo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_melo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_melo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_melo_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_melo_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_melo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_melo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO_DIAG,
	.profile = &ispeaker_melo_rx_profile,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.channel_mode = 1,
#else
	.channel_mode = 2,
#endif /* CONFIG_SH_AUDIO_DRIVER */
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.pamp_on = &shspamp_poweron,
	.pamp_off = &shspamp_poweroff,
#else
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-500,-200,100,400,700,1000},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-500,-200,100,400,700,1000},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_ispeaker_melo_rx_device = {
	.name = "snddev_icodec",
	.id = 37,
	.dev = { .platform_data = &snddev_ispeaker_melo_rx_data },

};
#endif /*CONFIG_SH_AUDIO_DRIVER*/

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit ihs_stereo_melo_rx_48KHz_osr256_actions[] =
#ifdef CONFIG_SH_AUDIO_DRIVER
#if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE )
	HEADSET_MELO_STEREO_RX_LEGACY_48000_OSR_256;
#else
	HEADSET_MELO_STEREO_RX_CAPLESS_48000_OSR_256;
#endif /* #if !defined( SH_AUDIO_HPAMP_CAPLESS_ENABLE ) */
#else
	HEADSET_MELO_STEREO_RX_LEGACY_48000_OSR_256;
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct adie_codec_hwsetting_entry ihs_stereo_melo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_melo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_melo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_melo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_melo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_melo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_melo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_melo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO_DIAG,
	.profile = &ihs_stereo_melo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifdef CONFIG_SH_AUDIO_DRIVER
#if defined( CONFIG_SHEXTDEV )
	.voltage_on = snddev_hsed_voltage_on,
	.voltage_off = snddev_hsed_voltage_off,
#endif /* #if defined( CONFIG_SHEXTDEV ) */
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2500,-2000,-1500,-1000,-500,0},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1900,-1600,-1300,-1000,-700},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2400,-2100,-1800,-1500,-1200,-900},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_ihs_stereo_melo_rx_device = {
	.name = "snddev_icodec",
	.id = 38,
	.dev = { .platform_data = &snddev_ihs_stereo_melo_rx_data },
};
#endif /*CONFIG_SH_AUDIO_DRIVER*/

#ifdef CONFIG_SH_AUDIO_DRIVER
static struct adie_codec_action_unit iearpiece_melo_48KHz_osr256_actions[] =
	HANDSET_MELO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_melo_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_melo_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_melo_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_melo_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_melo_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_melo_settings),
};

static struct snddev_icodec_data snddev_iearpiece_melo_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_melo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR_DIAG,
	.profile = &iearpiece_melo_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
#ifdef CONFIG_SH_AUDIO_DRIVER
	.property = SIDE_TONE_MASK,
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SH_AUDIO_DRIVER
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 102)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 202)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 204)||(CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 208)
	.voice_rx_vol_step[VOC_NB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-2200,-1700,-1200,-700,-200,300},
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
	.voice_rx_vol_step[VOC_NB_INDEX] = {-1700,-1400,-1100,-800,-500,-200},
	.voice_rx_vol_step[VOC_WB_INDEX] = {-1700,-1400,-1100,-800,-500,-200},
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#else /*CONFIG_SH_AUDIO_DRIVER*/
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
#endif /*CONFIG_SH_AUDIO_DRIVER*/
};

static struct platform_device msm_iearpiece_melo_device = {
	.name = "snddev_icodec",
	.id = 39,
	.dev = { .platform_data = &snddev_iearpiece_melo_data },
};
#endif /*CONFIG_SH_AUDIO_DRIVER*/

#ifdef CONFIG_SH_AUDIO_DRIVER
/* [in ambient path] [start] */
/* speaker_in_ambient_mic_tx */
static struct adie_codec_action_unit ispeaker_in_ambient_tx_8KHz_osr256_actions[] =
	SPEAKER_IN_AMBIENT_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_in_ambient_tx_48KHz_osr256_actions[] =
	SPEAKER_IN_AMBIENT_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_in_ambient_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_in_ambient_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_in_ambient_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_in_ambient_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_in_ambient_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_in_ambient_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_in_ambient_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_in_ambient_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_in_ambient_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_in_ambient_tx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_in_ambient_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_in_ambient_mic_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_IN_AMBIENT,
	.profile = &ispeaker_in_ambient_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_in_ambient_tx_device = {
	.name = "snddev_icodec",
	.id = 40,
	.dev = { .platform_data = &snddev_ispeaker_in_ambient_tx_data },
};


/* handset_in_ambient_mic_tx */
static struct adie_codec_action_unit ihandset_in_ambient_tx_8KHz_osr256_actions[] =
	HANDSET_IN_AMBIENT_TX_8000_OSR_256;

static struct adie_codec_action_unit ihandset_in_ambient_tx_48KHz_osr256_actions[] =
	HANDSET_IN_AMBIENT_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihandset_in_ambient_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihandset_in_ambient_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihandset_in_ambient_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihandset_in_ambient_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihandset_in_ambient_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihandset_in_ambient_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihandset_in_ambient_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihandset_in_ambient_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihandset_in_ambient_tx_settings,
	.setting_sz = ARRAY_SIZE(ihandset_in_ambient_tx_settings),
};

static struct snddev_icodec_data snddev_ihandset_in_ambient_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_in_ambient_mic_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_IN_AMBIENT,
	.profile = &ihandset_in_ambient_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihandset_in_ambient_tx_device = {
	.name = "snddev_icodec",
	.id = 41,
	.dev = { .platform_data = &snddev_ihandset_in_ambient_tx_data },
};
/* [in ambient path] [end] */
#endif /* CONFIG_SH_AUDIO_DRIVER */


#ifndef CONFIG_SH_AUDIO_DRIVER
static struct platform_device *snd_devices_ffa[] __initdata = {
	&msm_iearpiece_ffa_device,
	&msm_imic_ffa_device,
	&msm_ifmradio_handset_device,
	&msm_ihs_ffa_stereo_rx_device,
	&msm_ihs_ffa_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ispeaker_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_ffa_headset_device,
	&msm_idual_mic_endfire_device,
	&msm_idual_mic_broadside_device,
	&msm_spk_idual_mic_endfire_device,
	&msm_spk_idual_mic_broadside_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_uplink_rx_device,
	&msm_real_stereo_tx_device,
};

static struct platform_device *snd_devices_surf[] __initdata = {
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_uplink_rx_device,
};

static struct platform_device *snd_devices_fluid[] __initdata = {
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_ispeaker_rx_device,
	&msm_ispeaker_tx_device,
	&msm_fluid_imic_tx_device,
	&msm_fluid_iearpeice_rx_device,
	&msm_fluid_idual_mic_endfire_device,
	&msm_fluid_spk_idual_mic_endfire_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_uplink_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
};
#endif /* CONFIG_SH_AUDIO_DRIVER */

static struct platform_device *snd_devices[] __initdata = {
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_uplink_rx_device,
#ifdef CONFIG_SH_AUDIO_DRIVER
	&msm_iearpiece_lb_device,
	&msm_imic_lb_device,
	&msm_ihs_stereo_lb_tx_device,
	&msm_imicmute_device,
	&msm_imic_lb_spkr_device,
	&msm_iearpiecemute_device,
	&msm_ispeaker_builtin_mic_tx_device,
	&msm_ispeaker_in_communication_tx_device,
	&msm_ihs_stereo_lb_rx_device,
	&msm_ispeaker_lb_rx_device,
	&msm_ispeaker_melo_rx_device,
	&msm_ihs_stereo_melo_rx_device,
	&msm_iearpiece_melo_device,
	&msm_ispeaker_in_ambient_tx_device,
	&msm_ihandset_in_ambient_tx_device,
#endif /* CONFIG_SH_AUDIO_DRIVER */
};

#ifdef CONFIG_DEBUG_FS
static void snddev_hsed_config_modify_setting(int type)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_ihs_ffa_stereo_rx_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		if (type == 1) {
			icodec_data->voltage_on = NULL;
			icodec_data->voltage_off = NULL;
			icodec_data->profile->settings =
				ihs_ffa_stereo_rx_class_d_legacy_settings;
			icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_class_d_legacy_settings);
		} else if (type == 2) {
			icodec_data->voltage_on = NULL;
			icodec_data->voltage_off = NULL;
			icodec_data->profile->settings =
				ihs_ffa_stereo_rx_class_ab_legacy_settings;
			icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_class_ab_legacy_settings);
		}
	}
}

static void snddev_hsed_config_restore_setting(void)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_ihs_ffa_stereo_rx_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		icodec_data->voltage_on = msm_snddev_hsed_voltage_on;
		icodec_data->voltage_off = msm_snddev_hsed_voltage_off;
		icodec_data->profile->settings = ihs_ffa_stereo_rx_settings;
		icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_settings);
	}
}

static ssize_t snddev_hsed_config_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *lb_str = filp->private_data;
	char cmd;

	if (get_user(cmd, ubuf))
		return -EFAULT;

	if (!strcmp(lb_str, "msm_hsed_config")) {
		switch (cmd) {
		case '0':
			snddev_hsed_config_restore_setting();
			break;

		case '1':
			snddev_hsed_config_modify_setting(1);
			break;

		case '2':
			snddev_hsed_config_modify_setting(2);
			break;

		default:
			break;
		}
	}
	return cnt;
}

static int snddev_hsed_config_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations snddev_hsed_config_debug_fops = {
	.open = snddev_hsed_config_debug_open,
	.write = snddev_hsed_config_debug_write
};
#endif

void __ref msm_snddev_init(void)
{
#ifdef CONFIG_SH_AUDIO_DRIVER
	platform_add_devices(snd_devices,
	ARRAY_SIZE(snd_devices));

#ifdef CONFIG_DEBUG_FS
	debugfs_hsed_config = debugfs_create_file("msm_hsed_config",
				S_IFREG | S_IRUGO, NULL,
	(void *) "msm_hsed_config", &snddev_hsed_config_debug_fops);
#endif
#else
	if (machine_is_msm7x30_ffa() || machine_is_msm8x55_ffa() ||
		machine_is_msm8x55_svlte_ffa()) {
		platform_add_devices(snd_devices_ffa,
		ARRAY_SIZE(snd_devices_ffa));
#ifdef CONFIG_DEBUG_FS
		debugfs_hsed_config = debugfs_create_file("msm_hsed_config",
					S_IFREG | S_IRUGO, NULL,
		(void *) "msm_hsed_config", &snddev_hsed_config_debug_fops);
#endif
	} else if (machine_is_msm7x30_surf() || machine_is_msm8x55_surf() ||
		machine_is_msm8x55_svlte_surf())
		platform_add_devices(snd_devices_surf,
		ARRAY_SIZE(snd_devices_surf));
	else if (machine_is_msm7x30_fluid())
		platform_add_devices(snd_devices_fluid,
		ARRAY_SIZE(snd_devices_fluid));
	else
		pr_err("%s: Unknown machine type\n", __func__);
#endif /* CONFIG_SH_AUDIO_DRIVER */
}
