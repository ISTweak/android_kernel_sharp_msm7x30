/* drivers/sharp/shboot/sh_boot_manager.c
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

/*===========================================================================
INCLUDE
===========================================================================*/
#include <sharp/sh_smem.h>

/*===========================================================================
DEFINE
===========================================================================*/

/*=============================================================================
FUNCTION
=============================================================================*/
unsigned short sh_boot_get_hw_revision(void)
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != 0 )
	{
		return p_sharp_smem_common_type->sh_hw_revision;
	}else{
		return 0xFF;
	}
}

unsigned short sh_boot_get_bootmode(void)
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type  = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != 0 )
	{
		return p_sharp_smem_common_type->sh_boot_mode;
	}else{
		return 0;
	}
}
