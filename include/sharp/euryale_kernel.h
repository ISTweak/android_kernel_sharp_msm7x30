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

#if !defined( EURYALE_KERNEL_H )
#define EURYALE_KERNEL_H

enum{
    EURYALE_COMMAND_WRITE = 0,
    EURYALE_COMMAND_READ,
    EURYALE_COMMAND_INIT
};

struct euryale_command{
    int cmd;
    int inited;
    int ret;
    unsigned long sector;
    unsigned long nr;
};

struct euryale_block{
    char *buffer;
    unsigned int length;
};

int euryale_write_process( unsigned long sector, unsigned long nr, const char *buffer );
int euryale_read_process( unsigned long sector, unsigned long nr, char *buffer );
int euryale_api_blockwrite( unsigned long sector, unsigned long nr, struct euryale_block *block );
int euryale_api_blockread( unsigned long sector, unsigned long nr, struct euryale_block *block );
int euryale_api_init( void );
int euryale_need_backwardcompatibility( void );

#endif
