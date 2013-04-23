/* include/sharp/sh_smem.h
 *
 * Copyright (C) 2010 Sharp Corporation
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
 

#include <sharp/shdisp_kerl.h>

#include <sharp/shrlog_type.h>

typedef struct 
{
    unsigned long       sh_filesystem_init;			/* file system innitialize flag */
    unsigned long       sh_hw_revision;				/* hardware revision number */
    unsigned long       sh_model_type;				/* model type information */
    unsigned long       sh_boot_mode;				/* power up mode information */
    unsigned long       sh_softupdate_mode;			/* software update mode  */
    unsigned long       sh_pwr_on_status;			/* power on status information from pmic  */
    struct shdisp_boot_context shdisp_boot_ctx;
    unsigned long       shdiag_FlagData;			/* shdiag flag information */
    unsigned short      shdiag_BootMode;			/* shdiag Powerup mode */
    unsigned char       shdiag_FirstBoot;			/* shdiag FirstBoot information */
    unsigned char       rec[3];                     /* reserved */
    unsigned char       shdiag_AdjChashe[16];		/* shdiag Adj chashe information */
    unsigned long       support_FlagData;			/* support flag information */
    int                 sh_sleep_test_mode;
    unsigned char       shsecure_PassPhrase[32];
    unsigned char       bootimg_header[2048];		/* APPSBOOT header information */
    shrlog_reset_info   smem_reset_info;            /* Reset Log information structure */
    unsigned long       fota_boot_mode;				/* FOTA mode information */
    unsigned char       pImeiData[12];				/* W-CDMA Imei data  */
    unsigned char       sh_fwupdateFlag;			/* Update Information */
    unsigned long       sh_boot_key;				/* key(s) ditected OSBL */
    unsigned char       sh_camver[4];				/* Version information */
    unsigned char       sh_touchver[4];				/* Version information	*/
    unsigned char       sh_miconver[4];				/* Version information	*/
    unsigned char       shdarea_QRData[128];		/* QRdata */
    unsigned char       sh_swicdev;					/* USB SWIC exist information */
    unsigned char       shdarea_WlanMacAddress[6];	/* WLAN Mac Address */
    unsigned char       sh_camImgAdjustData[102];	/* Camera Image Adjust Data */
    unsigned char       conf_clrvari[4];			/* Color Variations information */
    unsigned char       shrmts_data_buf[16384+32];	/* Buffer for shrmts */
    unsigned char       sh_camOtpData[8864];		/* 8MCMOS Camera OTPData */
    unsigned char       shsys_wait_for_modem_flag;	/* before flash_read wait flag*/
    unsigned long       kp_magic;					/* panic flag */
    unsigned char       kp_buf[32768];				/* panic log buffer */
    unsigned char       shdarea_WlanCalPAData[20];	/* WLAN Calibration PaParam */
    unsigned long       sh_tps_control;				/* TPS Calibreation Data */
    unsigned long       reserved[2];				/* reserved */
} sharp_smem_common_type;

/*=============================================================================

FUNCTION sh_smem_get_common_address

=============================================================================*/
sharp_smem_common_type *sh_smem_get_common_address( void );

