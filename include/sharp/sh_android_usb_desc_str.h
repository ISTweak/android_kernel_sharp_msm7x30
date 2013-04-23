/* kernel/include/sharp/sh_android_usb_desc_str.h
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

#ifndef __SH_ANDROID_USB_DESC_STR_H
#define __SH_ANDROID_USB_DESC_STR_H

#if defined (CONFIG_MACH_DECKARD_AF21)
	#include "sh_android_usb_desc_str_deckard_af21.h"
#elif defined (CONFIG_MACH_DECKARD_AF30)
	#include "sh_android_usb_desc_str_deckard_af30.h"
#elif defined( CONFIG_MACH_DECKARD_AS32 )
	#include "sh_android_usb_desc_str_deckard_as32.h"
#endif

#endif /* __SH_ANDROID_USB_DESC_STR_H */
