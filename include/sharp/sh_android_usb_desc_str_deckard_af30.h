/* kernel/include/sharp/sh_android_usb_desc_str_deckard_af30.h
 *
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

#ifndef __SH_ANDROID_USB_DESC_STR_DECKARD_AF30_H
#define __SH_ANDROID_USB_DESC_STR_DECKARD_AF30_H

#define USB_OBEX_STRING_DESC_WORD	"au INFOBAR C01 OBEX Port"
#define USB_MDLM_STRING_DESC_WORD	"-au INFOBAR C01 High Speed Serial Port"
#define USB_GS_STRING_DESC_WORD		"au INFOBAR C01"
#define USB_ADB_STRING_DESC_WORD	"Android ADB Interface"
#define USB_QXDM_STRING_DESC_WORD	"Qualcomm Port"
#define USB_MSC_STRING_DESC_WORD	"au INFOBAR C01 SD Storage"
#define USB_MTP_STRING_DESC_WORD	"au INFOBAR C01 MTP Port"

#define USB_PID_MODE_CDC			0x953F
#define USB_PID_MODE_MTP			0x953E
#define USB_PID_MODE_MSC			0x953D
#define USB_PID_MODE_CDC_2			0x933A
#define USB_PID_MODE_RNDIS			0x9540
#define USB_PID_MODE_CDROM			0x9541

#define USB_PID_MODE_MTP_C			0x9543
#define USB_PID_MODE_MSC_C			0x9542
#define USB_PID_MODE_RNDIS_C		0x9544


#define USB_CDC_GUID				0xC2,0x29,0x9F,0xCC,0xD4,0x89,0x40,0x66,\
									0x89,0x2B,0x10,0xC3,0x41,0xDD,0x98,0xA9

#define USB_MSC_VENDOR_ID_LEN	(8)
#define USB_MSC_VENDOR_ID		"SHARP   "
#define USB_MSC_PRODUCT_ID_LEN	(16)
#define USB_MSC_PRODUCT_ID		"INFOBAR C01 SD  "
#define USB_MSC_CD_PRODUCT_ID	"INFOBAR C01 CD  "

#endif /* __SH_ANDROID_USB_DESC_STR_DECKARD_AF30_H */
