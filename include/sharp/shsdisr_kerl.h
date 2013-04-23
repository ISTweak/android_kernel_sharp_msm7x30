/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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

#if !defined( SHSDISR_KERL_H )
#define SHSDISR_KERL_H

#define SHSDISR_MAGIC 'd'
#define SHSDISR_IOCTL_EVENT_SUBSCRIBE _IOW( SHSDISR_MAGIC, 1, int )
#define SHSDISR_IOCTL_EVENT_UNSUBSCRIBE _IOW( SHSDISR_MAGIC, 2, int )

#endif
