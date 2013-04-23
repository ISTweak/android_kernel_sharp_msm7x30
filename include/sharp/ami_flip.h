/*
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

#ifndef __AMI_FLIP_H__
#define __AMI_FLIP_H__

#define AMI_FLIP_DEVFILE	"ami_flip"

#define AMI_FLIP_IOC							'F'
#define AMI_FLIP_IOCTL_READ_POSITION		_IOR(AMI_FLIP_IOC, 0x01 , int )
#define AMI_FLIP_IOCTL_WRITE_POSITION		_IOW(AMI_FLIP_IOC, 0x02 , int )


#endif				/* __AMI_FLIP_H__ */
