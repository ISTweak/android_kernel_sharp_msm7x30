/* include/sharp/shdiag_smd.h
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

#ifndef _SHDIAG_SMD_H_
#define _SHDIAG_SMD_H_

/*
 * Defines
 */

#define SHDIAG_SMD_DEVFILE "/dev/smd_read"

/*
 * TYPES
 */

struct smem_comm_mode {
	unsigned long BootMode;
	unsigned long UpDateFlg;
};

/*End of File*/
#endif /* _SHDIAG_SMD_H_ */
