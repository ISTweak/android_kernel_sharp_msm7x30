/* include/sharp/sh_boot_manager.h
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

#ifndef SH_BOOT_MANAGER_H
#define SH_BOOT_MANAGER_H
/*===========================================================================
INCLUDE
===========================================================================*/

/*===========================================================================
DEFINE
===========================================================================*/
#define SH_BOOT_NORMAL 0xFFFF

/*===========================================================================
FUNCTION
===========================================================================*/
unsigned short sh_boot_get_hw_revision(void);
unsigned short sh_boot_get_bootmode(void);

#endif /* SH_BOOT_MANAGER_H */
