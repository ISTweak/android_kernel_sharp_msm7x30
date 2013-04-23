/* include/sharp/shswic_kerl.h
 *
 * Copyright (C) 2010 SHARP CORPORATION
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
 
#ifndef _SHSWIC_KERL_
#define _SHSWIC_KERL_

#define SHSWIC_IO				0xA2
#define SHSWIC_IOCTL_ID_READ	_IO(SHSWIC_IO, 0x01)

typedef enum
{
	SHSWIC_SUCCESS =		0,
	SHSWIC_FAILURE =		-1,
	SHSWIC_PARAM_ERROR =	-2,
}shswic_result_t;

enum
{
	SHSWIC_VBUS_DEVICE =	0,
	SHSWIC_HEADSET_DEVICE,
};

enum
{
	SHSWIC_ID_USB_CABLE		= 0x01,
	SHSWIC_ID_AC_ADAPTER	= 0x02,
	SHSWIC_ID_HEADSET		= 0x04,
	SHSWIC_ID_HEADSET_SW	= 0x08,
	SHSWIC_ID_NONE			= 0x10,
	SHSWIC_ID_UNKNOWN		= 0x20,
};

shswic_result_t shswic_detect_cb_regist(uint8_t cb_dev, uint32_t cb_irq, void* cb_func, void* user_data );
shswic_result_t shswic_get_usb_port_status( uint8_t* device );

#endif /* _SHSWIC_KERL_ */
