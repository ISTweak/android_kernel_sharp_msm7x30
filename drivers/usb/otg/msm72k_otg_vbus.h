/* drivers/usb/otg/msm72k_otg_vbus.h  (SHVBUS driver)
 * 
 * USB Gadget VBUS driver module
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

#ifndef __LINUX_USB_GADGET_MSM72K_VBUS_H__
#define __LINUX_USB_GADGET_MSM72K_VBUS_H__

#ifdef CONFIG_USB_SWIC
#define SHVBUS_TMRVAL_DISCON_USB_WAIT	720
#else /* CONFIG_USB_SWIC */
#define SHVBUS_TMRVAL_DISCON_USB_WAIT	300
#endif /* CONFIG_USB_SWIC */

#define SHVBUS_TMR_DISCON_USB_WAIT	7

#define SHVBUS_EVT_CONNECT		1
#define SHVBUS_EVT_DISCONNECT		2
#define SHVBUS_EVT_CONNECT_AC		3
#define SHVBUS_EVT_CONNECT_USB		4
#define SHVBUS_EVT_DISCON_USB_TO	5

enum usb_shvbus_state {
	SHVBUS_STS_INIT = 0,
	SHVBUS_STS_DISCONNECT,
	SHVBUS_STS_CONNECT_AC,
	SHVBUS_STS_CONNECT_USB,
	SHVBUS_STS_DISCON_USB_WAIT,
};

#endif /* __LINUX_USB_GADGET_MSM72K_VBUS_H__ */
