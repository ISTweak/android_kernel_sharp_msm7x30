/* include/sharp/shhdmi_kerl.h (SHARP HDMI Driver)
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

#ifndef __SHHDMI_KERL_HEADER__
#define __SHHDMI_KERL_HEADER__

// ____________________________________________________________________________________________________________________________
/**
 *	Include
 */
#include <linux/ioctl.h>

// ____________________________________________________________________________________________________________________________
/**
 *	Define
 */
#define SH_HDMI_IOCTL_MAGIC 'h'

#define SH_HDMI_IOCTL_START_IRQ					_IO(SH_HDMI_IOCTL_MAGIC, 0)
#define SH_HDMI_IOCTL_EXIT_IRQ					_IO(SH_HDMI_IOCTL_MAGIC, 1)
#define SH_HDMI_IOCTL_READ_IRQ					_IO(SH_HDMI_IOCTL_MAGIC, 2)

#define SH_HDMI_IOCTL_PORT_HDMIRST_N_LO			_IO(SH_HDMI_IOCTL_MAGIC, 3)
#define SH_HDMI_IOCTL_PORT_HDMIRST_N_HI			_IO(SH_HDMI_IOCTL_MAGIC, 4)
#define SH_HDMI_IOCTL_PORT_HDMIPOW_LO			_IO(SH_HDMI_IOCTL_MAGIC, 5)
#define SH_HDMI_IOCTL_PORT_HDMIPOW_HI			_IO(SH_HDMI_IOCTL_MAGIC, 6)
#define SH_HDMI_IOCTL_PORT_HDMIPD_N_LO			_IO(SH_HDMI_IOCTL_MAGIC, 7)
#define SH_HDMI_IOCTL_PORT_HDMIPD_N_HI			_IO(SH_HDMI_IOCTL_MAGIC, 8)

#define SH_HDMI_IOCTL_WAKE_LOCK					_IO(SH_HDMI_IOCTL_MAGIC, 9)
#define SH_HDMI_IOCTL_WAKE_UNLOCK				_IO(SH_HDMI_IOCTL_MAGIC, 10)

#define SH_HDMI_IOCTL_READ_HDMIINT_N			_IO(SH_HDMI_IOCTL_MAGIC, 11)

#define SH_HDMI_IOCTL_DTVIF_GPIOCONFIG_SET		_IO(SH_HDMI_IOCTL_MAGIC, 12)
#define SH_HDMI_IOCTL_DTVIF_GPIOCONFIG_RESET	_IO(SH_HDMI_IOCTL_MAGIC, 13)

#define SH_HDMI_IOCTL_REPORT_INPUT_KEY_PRESS	_IO(SH_HDMI_IOCTL_MAGIC, 14)
#define SH_HDMI_IOCTL_REPORT_INPUT_KEY_RELEASE	_IO(SH_HDMI_IOCTL_MAGIC, 15)

#endif	//__SHHDMI_KERL_HEADER__

