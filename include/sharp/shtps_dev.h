/* include/sharp/shtps_dev.h
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
#ifndef __SHTPS_DEV_H__
#define __SHTPS_DEV_H__

#if defined( CONFIG_SHTPS_SY3000_TM1836_001 )
	#include <sharp/shtps_sy3000_tm1836-001.h>
#elif defined( CONFIG_SHTPS_SY3000_TM1901_001 )
	#include <sharp/shtps_sy3000_tm1901-001.h>
#elif defined( CONFIG_SHTPS_SY3000_TM1918_001 )
	#include <sharp/shtps_sy3000_tm1918-001.h>
#elif defined( CONFIG_SHTPS_SY3000_TM1963_001 )
	#include <sharp/shtps_sy3000_tm1963-001.h>
#elif defined( CONFIG_SHTPS_SY3000_TM1980_001 )
	#include <sharp/shtps_sy3000_tm1980-001.h>
#elif defined( CONFIG_SHTPS_TMA3XX_TMA340_003 )
	#include <sharp/shtps_tma3xx_tma340-003.h>
#elif defined( CONFIG_SHTPS_TMA3XX_TMA340_004 )
	#include <sharp/shtps_tma3xx_tma340-004.h>
#elif defined( CONFIG_SHTPS_TMA3XX_TMA340_005 )
	#include <sharp/shtps_tma3xx_tma340-005.h>
#endif

#endif /* __SHTPS_DEV_H__ */
