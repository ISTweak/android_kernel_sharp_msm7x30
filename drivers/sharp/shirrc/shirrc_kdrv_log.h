/* drivers/sharp/shirrc/shirrc_kdrv_log.h (Infrared driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#ifndef _IRRC_KDRV_LOG_H
#define _IRRC_KDRV_LOG_H

#include <linux/kernel.h>

#define IRRC_NAME	"irrc_kern"

#define IRRCLOG_ALERT(format, args...)		\
	printk(KERN_ALERT "[%s] " format, IRRC_NAME, ##args)
#define IRRCLOG_ERROR(format, args...)		\
	printk(KERN_ERR "[%s] " format, IRRC_NAME, ##args)
#define IRRCLOG_WARNING(format, args...)	\
	printk(KERN_WARNING "[%s] " format, IRRC_NAME, ##args)
#define IRRCLOG_INFO(format, args...)
#define IRRCLOG_DEBUG(format, args...)

#endif
