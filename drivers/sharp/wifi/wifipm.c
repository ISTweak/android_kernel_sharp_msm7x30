/* drivers/sharp/wifi/wifipm.c  (WiFi Power Management)
 *
 * Copyright (C) 2009 SHARP CORPORATION
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/pmic.h>
#include <linux/slab.h>

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Definition

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
#define _WIFIPM_NAME_	"wifipm"

#define disp_err( format, args... ) \
	printk( KERN_ERR "[%s] " format, _WIFIPM_NAME_ , ##args )
#define disp_war( format, args... ) \
	printk( KERN_WARNING "[%s] " format, _WIFIPM_NAME_ , ##args )

#ifdef WIFIPM_DEBUG
  #define disp_inf( format, args... ) \
	printk( KERN_ERR "[%s] " format, _WIFIPM_NAME_ , ##args )
  #define disp_dbg( format, args... ) \
	printk( KERN_ERR "[%s] " format, _WIFIPM_NAME_ , ##args )
  #define disp_trc( format, args... ) \
	printk( KERN_ERR "[%s] trace:%s " format, _WIFIPM_NAME_ , __func__ , ##args )
#else
  #define disp_inf( format, args... ) 
  #define disp_dbg( format, args... ) 
  #define disp_trc( format, args... )
#endif /* WIFIPM_DEBUG */

/* private data */
typedef struct {
	int active; /* power status of WiFi */
	int shdiag; /* WiFi Shdiag Flag  SHDiag Start:1 Normal Start:0 */ 
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review Start */
	int bcmsdh; /* init/cleanup bcmsdh */
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review  End  */
} wifipm_data_t;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Configuration

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GPIO mapping for Wireless LAN device */
#define WIFIPM_PORT_WL_RESET_N		143	      /* WLAN Reset */
#define WIFIPM_REG_VIA_BT

#ifdef CONFIG_SHWLAN_BCM4330_1
#define WIFIPM_PORT_WL_REG		143	      /* WLAN Reset */
#undef WIFIPM_PORT_WL_RESET_N
#undef WIFIPM_REG_VIA_BT
#endif

/* [WLAN][SHARP] 2011.09.21 Mod for Model dependence GPIO current value Start */
#if defined(CONFIG_MACH_DECKARD_AS35) || defined(CONFIG_MACH_DECKARD_AS50) || \
    defined(CONFIG_MACH_DECKARD_AF20) || defined(CONFIG_MACH_DECKARD_AS36) || \
    defined(CONFIG_MACH_DECKARD_AF30) || defined(CONFIG_MACH_DECKARD_AF21) || \
    defined(CONFIG_MACH_DECKARD_AF33)
  #define WIFI_GPIO_VALUE GPIO_CFG_8MA
#else
  #define WIFI_GPIO_VALUE GPIO_CFG_6MA
#endif
/* [WLAN][SHARP] 2011.09.21 Mod for Model dependence GPIO current value End */

/* Port Configuration Table */
static unsigned wifipm_gpio_config[] = {
	/*        gpio,                  func, dir,          pull,        drvstr */
#ifdef WIFIPM_PORT_WL_RESET_N
  GPIO_CFG( WIFIPM_PORT_WL_RESET_N,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, WIFI_GPIO_VALUE),
#endif
#ifdef WIFIPM_PORT_WL_REG
  GPIO_CFG( WIFIPM_PORT_WL_REG,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, WIFI_GPIO_VALUE),
#endif
};





#ifndef WIFIPM_INI
  #define WIFIPM_INI 0
#endif

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Proto type

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#ifdef WIFIPM_REG_VIA_BT
void btpm_wifi_reg( int on );
#endif

/* [WLAN][SHARP] 2011.05.12 mod Static Link Review Start */
/* control bcmsdh from user space */
void sdio_function_cleanup(void);
int sdio_function_init(void);
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review  End  */

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Resource

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
struct platform_device *p_wifipm_dev = NULL;

/* [WLAN][SHARP] 2011/06/15 export power management API Start */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Global

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void wifipm_dev_reset( int reset ){

    if(reset){
#ifdef WIFIPM_PORT_WL_RESET_N
		gpio_set_value( WIFIPM_PORT_WL_RESET_N, 0 );
#elif defined(WIFIPM_PORT_WL_REG)
		gpio_set_value( WIFIPM_PORT_WL_REG, 0 );
#endif
        msleep(20);
        disp_dbg( "%s: Device Reset ON\n" , __func__);
    } else{
#ifdef WIFIPM_PORT_WL_RESET_N
		gpio_set_value( WIFIPM_PORT_WL_RESET_N, 1 );
#elif defined(WIFIPM_PORT_WL_REG)
		gpio_set_value( WIFIPM_PORT_WL_REG, 1 );
#endif
        msleep(110);
        disp_dbg( "%s: Device Reset OFF\n" , __func__);
    }

}
/* [WLAN][SHARP] 2011/06/15 export power management API End */

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Local

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int wifipm_reset( struct device *pdev )
{
	wifipm_data_t *p_priv;

	if ( pdev == NULL ){
		disp_err( "device for wifipm not found\n" );
		return -1;
	}

	p_priv = (wifipm_data_t *)dev_get_drvdata(pdev);
	if ( p_priv == NULL ){
		disp_err( "driver infomation for wifipm not found\n" );
		return -1;
	}


	/* WLAN Reset */
#ifdef WIFIPM_PORT_WL_RESET_N
	gpio_set_value( WIFIPM_PORT_WL_RESET_N, 0 );	/* WLAN_RST */
#endif

	/* WL_REG Disable */
#ifdef WIFIPM_PORT_WL_REG
	gpio_set_value( WIFIPM_PORT_WL_REG, 0 );	/* WLAN_REG */
#elif defined(WIFIPM_REG_VIA_BT)
    btpm_wifi_reg( 0 );
#endif

	/* init status */
	p_priv->active = 0;
	p_priv->shdiag = 0;
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review Start */
	p_priv->bcmsdh = 0; 
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review  End  */

	return 0;
}



static int wifipm_power( struct device *pdev , int on )
{
	wifipm_data_t *p_priv;

	if ( pdev == NULL ){
		disp_err( "device for wifipm not found\n" );
		return -1;
	}

	p_priv = (wifipm_data_t *)dev_get_drvdata(pdev);
	if ( p_priv == NULL ){
		disp_err( "driver infomation for wifipm not found\n" );
		return -1;
	}

	if ( p_priv->active == on ){
		disp_dbg( "%s: no need to change status (%d->%d)\n" , __func__, p_priv->active , on );
		return 0;
	}

	if ( on ){

		/* WL_REG Enable */
#ifdef WIFIPM_PORT_WL_REG
	    gpio_set_value( WIFIPM_PORT_WL_REG, 1 );
#elif defined(WIFIPM_REG_VIA_BT)
        btpm_wifi_reg( 1 );
#endif
  		disp_dbg( "%s: WL_REG ON\n" , __func__);


	/* WLAN_RST */
#ifdef WIFIPM_PORT_WL_RESET_N
        msleep(20);
	    gpio_set_value( WIFIPM_PORT_WL_RESET_N, 1 );
#endif


	} else {

	/* WLAN_RST */
#ifdef WIFIPM_PORT_WL_RESET_N
	    gpio_set_value( WIFIPM_PORT_WL_RESET_N, 0 );
        msleep(1);
#endif


		/* WL_REG Disable */
#ifdef WIFIPM_PORT_WL_REG
	    gpio_set_value( WIFIPM_PORT_WL_REG, 0 );
#elif defined(WIFIPM_REG_VIA_BT)
    btpm_wifi_reg( 0 );
#endif
    	disp_dbg( "%s: WL_REG OFF\n" , __func__);

	}

	if ( p_priv->active < 0 ){
		disp_inf( "WiFi power on reset\n" );
	} else {
		disp_dbg( "%s: change status (%d->%d)\n" , __func__, p_priv->active , on );
	}

	p_priv->active = on;

	return 0;
}


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Device attribute

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static
ssize_t show_wifi_power(struct device *pdev, struct device_attribute *pattr, char *buf)
{
	wifipm_data_t *p_priv = (wifipm_data_t *)dev_get_drvdata(pdev);
	int status;

	status = p_priv->active;
	
	return snprintf( buf, PAGE_SIZE, "%d\n" , status );
}

static
ssize_t set_wifi_power(struct device *pdev, struct device_attribute *pattr, const char *buf, size_t count)
{
	int new_status;

	sscanf( buf, "%1d", &new_status );

	if ( (new_status==0) || (new_status==1) ){
		wifipm_power( pdev, new_status );
	}

	return count;
}

static
ssize_t show_wifi_shdiag(struct device *pdev, struct device_attribute *pattr, char *buf)
{
	wifipm_data_t *p_priv = (wifipm_data_t *)dev_get_drvdata(pdev);
	int status;

	status = p_priv->shdiag;
	
	return snprintf( buf, PAGE_SIZE, "%d\n" , status );
	
}

static
ssize_t set_wifi_shdiag(struct device *pdev, struct device_attribute *pattr, const char *buf, size_t count)
{
	wifipm_data_t *p_priv = (wifipm_data_t *)dev_get_drvdata(pdev);
	int new_status;

	sscanf( buf, "%1d", &new_status );

	if ( (new_status==0) || (new_status==1) ){
		p_priv->shdiag = new_status;
	}

	return count;

}

/* [WLAN][SHARP] 2011.05.12 mod Static Link Review Start */
/* control bcmsdh from user space */
static
ssize_t ctl_bcmsdh(struct device *pdev, struct device_attribute *pattr, const char *buf, size_t count)
{
	int enable;
/* [WLAN][SHARP] 2011.11.30 mod SDIO resource release Start */
	wifipm_data_t *p_priv = (wifipm_data_t *)dev_get_drvdata(pdev);

	sscanf( buf, "%1d", &enable );

	if ( p_priv->bcmsdh == enable ){
		disp_err("%s: Forbid double process %d\n ", __FUNCTION__, enable);
		return count;
	}

	if ( enable == 1 ){
		sdio_function_init();
		p_priv->bcmsdh = enable;
	} else if (enable == 0 ){
		p_priv->bcmsdh = enable;
		sdio_function_cleanup();
	}else{
		/* Do nothing except 1 and 0   */
		disp_err("%s: Illegal State %d\n",__FUNCTION__, enable);
	}
/* [WLAN][SHARP] 2011.11.30 mod SDIO resource release  End  */

	return count;

}
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review  End  */


/* device attribute structure */
static DEVICE_ATTR(
	active,
	S_IRUGO | S_IWUGO,
	show_wifi_power,
	set_wifi_power
);

static DEVICE_ATTR(
	shdiag,
	S_IRUGO | S_IWUGO,
	show_wifi_shdiag,
	set_wifi_shdiag
);

/* [WLAN][SHARP] 2011.05.12 mod Static Link Review Start */
/* control bcmsdh from user space */
static DEVICE_ATTR(
	bcmsdh,
	S_IWUGO,
	NULL,
	ctl_bcmsdh
);
/* [WLAN][SHARP] 2011.05.12 mod Static Link Review  End  */

static struct attribute *wifipm_device_attributes[] = {
	&dev_attr_active.attr,
	&dev_attr_shdiag.attr,
	&dev_attr_bcmsdh.attr, /* [WLAN][SHARP] 2011/04/01 control bcmsdh from user space */
	NULL,
};

static struct attribute_group wifipm_device_attributes_gourp = {
	.attrs = wifipm_device_attributes,
};

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Driver Description

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int __init wifipm_driver_probe( struct platform_device *pdev )
{
	int pin;
	int ret;
	wifipm_data_t *p_priv;

	int ini;

	/* Port Configuration */
	for (pin = 0; pin < ARRAY_SIZE(wifipm_gpio_config); pin++) {
		ret = gpio_tlmm_config( wifipm_gpio_config[pin], GPIO_CFG_ENABLE );
		if (ret) {
			disp_err( "gpio_tlmm_config(%d)<-%#x : %d\n", pin,wifipm_gpio_config[pin], ret);
			return -EIO;
		}
	}

	/* Initialize private data */
	p_priv = kmalloc( sizeof(*p_priv) , GFP_KERNEL );
	if ( p_priv == NULL ){
		disp_err( "memory allocation for private data failed\n" );
		return -ENOMEM;
	}
	platform_set_drvdata( pdev , p_priv );

	/* power on reset */
	wifipm_reset( &(pdev->dev) );

	/* power on */
	ini = WIFIPM_INI;
	if ( ini ){
		wifipm_power( &(pdev->dev) , 1 );
	}

	/* create sysfs interface */
	ret = sysfs_create_group( &(pdev->dev.kobj), &wifipm_device_attributes_gourp);
	if ( ret ){
		disp_err( "Sysfs attribute export failed with error %d.\n" , ret );
	}

	return ret;
}

static int wifipm_driver_remove( struct platform_device *pdev )
{
	wifipm_data_t *p_priv;

	sysfs_remove_group( &(pdev->dev.kobj), &wifipm_device_attributes_gourp);
	
	p_priv = platform_get_drvdata( pdev );
	platform_set_drvdata( pdev , NULL );
	if ( p_priv != NULL ){
		kfree( p_priv );
	}
	
	return 0;
}

static void wifipm_driver_shutdown( struct platform_device *pdev )
{
/* [WLAN][SHARP] 2011.11.30 mod SDIO resource release Start */

	wifipm_data_t *p_priv;
	p_priv = platform_get_drvdata( pdev );

	printk("%s:enter\n",__FUNCTION__);

	sysfs_remove_group( &(pdev->dev.kobj), &wifipm_device_attributes_gourp);

	if( p_priv->bcmsdh ){
		printk("%s:sdio_function_cleanup\n",__FUNCTION__);
		p_priv->bcmsdh = 0;
		sdio_function_cleanup();
	}
	printk("%s:wifipm_power\n",__FUNCTION__);
/* [WLAN][SHARP] 2011.11.30 mod SDIO resource release  End  */
	wifipm_power( &(pdev->dev), 0 );

	return;
}

/* driver structure */
static struct platform_driver wifipm_driver = {
	.remove = __devexit_p(wifipm_driver_remove),
	.shutdown = __devexit_p(wifipm_driver_shutdown),
	.driver = {
		.name = _WIFIPM_NAME_,
		.owner = THIS_MODULE,
	},
};

static int wifipm_driver_init( void )
{
	int ret;
	
	/* regist driver */
	ret = platform_driver_probe( &wifipm_driver, wifipm_driver_probe );
	if ( ret != 0 ){
		disp_err( "driver register failed (%d)\n" , ret );
	}

	return ret;
}

static void wifipm_driver_exit( void )
{
	platform_driver_unregister( &wifipm_driver );
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Device Description

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int wifipm_device_init( void )
{
	int ret;
	
	/* allocate device structure */
	p_wifipm_dev = platform_device_alloc( _WIFIPM_NAME_ , -1 );
	if ( p_wifipm_dev == NULL ){
		disp_err( "device allocation for wifipm failed\n" );
		return -ENOMEM;
	}

	/* regist device */
	ret = platform_device_add( p_wifipm_dev );
	if ( ret != 0 ){
		disp_err( "device register for wifipm failed (%d)\n" , ret );
	}

	return ret;
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Module Description

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int __init wifipm_module_init( void )
{
	int ret;
	
	disp_inf( "Wifi Power Management\n" );

	ret = wifipm_device_init();
	if ( ret == 0 ){
		ret = wifipm_driver_init();
	}

	return ret;
}

static void __exit wifipm_module_exit( void )
{
	wifipm_driver_exit();
}


/*
EXPORT_SYMBOL(wifipm_power);
*/
/* [WLAN][SHARP] 2011/06/15 export power management API Start */
EXPORT_SYMBOL(wifipm_dev_reset);
/* [WLAN][SHARP] 2011/06/15 export power management API End */

MODULE_DESCRIPTION("WiFi Power Management");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("0.10");

module_init(wifipm_module_init);
module_exit(wifipm_module_exit);

