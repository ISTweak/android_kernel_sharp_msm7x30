/*
 * sh_i2c.h
 * sharp i2c driver module
 *
 * Copyright (C) 2010 SHARP CORPORATION All rights reserved.
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

#ifndef  SH_I2C_H
#define  SH_I2C_H

/* =================================================================
 * Prototype global functions
 * ================================================================= */
#if defined( CONFIG_I2C_CUST_SH )
void sh_msmi2c_1_mutex_lock( struct i2c_client *client );
void sh_msmi2c_1_mutex_free( struct i2c_client *client );
void sh_msmi2c_2_mutex_lock( struct i2c_client *client );
void sh_msmi2c_2_mutex_free( struct i2c_client *client );
void sh_qupi2c_mutex_lock( struct i2c_client *client );
void sh_qupi2c_mutex_free( struct i2c_client *client );
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

#endif /* SH_I2C_H */
