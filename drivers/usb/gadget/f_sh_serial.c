/* drivers/usb/gadget/f_sh_serial.c
 *
 * f_sh_serial.c - Gadget Driver for Android generic USB serial function driver
 *
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This code borrows from f_serial.c, which is
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/android_composite.h>

#include "u_serial.h"
#include "gadget_chips.h"

#ifdef CONFIG_USB_ANDROID_SH_DTFER
#define D_SH_SERIAL_SETUP_PORT_NUM		(4)
#define D_SH_SERIAL_SETUP_PORT_DTFER		(3)
#else /* CONFIG_USB_ANDROID_SH_DTFER */
#define D_SH_SERIAL_SETUP_PORT_NUM		(3)
#endif /* CONFIG_USB_ANDROID_SH_DTFER */
#define D_SH_SERIAL_SETUP_PORT_OBEX		(0)
#define D_SH_SERIAL_SETUP_PORT_MDLM		(1)
#define D_SH_SERIAL_SETUP_PORT_MODEM		(2)

/* Module */
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL");


int sh_serial_obex_bind_config(struct usb_configuration *c)
{
	int ret;

	/* See if composite driver can allocate
	 * serial ports. But for now allocate
	 * two ports for modem and nmea.
	 */
	ret = gserial_setup(c->cdev->gadget, D_SH_SERIAL_SETUP_PORT_NUM);
	if (ret)
		return ret;
	return obex_bind_config(c, D_SH_SERIAL_SETUP_PORT_OBEX);
}


static struct android_usb_function shmobex_function = {
	.name = "obex",
	.bind_config = sh_serial_obex_bind_config,
};

int sh_serial_mdlm_bind_config(struct usb_configuration *c)
{
	return gser_bind_config(c, D_SH_SERIAL_SETUP_PORT_MDLM);
}

static struct android_usb_function shmdlm_function = {
	.name = "mdlm",
	.bind_config = sh_serial_mdlm_bind_config,
};

int sh_serial_modem_bind_config(struct usb_configuration *c)
{
	return acm_bind_config(c, D_SH_SERIAL_SETUP_PORT_MODEM);
}


static struct android_usb_function shmodem_function = {
	.name = "modem",
	.bind_config = sh_serial_modem_bind_config,
};

#ifdef CONFIG_USB_ANDROID_SH_DTFER
int sh_serial_dtfer_bind_config(struct usb_configuration *c)
{
	return dtfer_bind_config(c, D_SH_SERIAL_SETUP_PORT_DTFER);
}

static struct android_usb_function shdtfer_function = {
	.name = "dtfer",
	.bind_config = sh_serial_dtfer_bind_config,
};
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

static int __init init(void)
{

	android_register_function(&shmobex_function);
	android_register_function(&shmdlm_function);
	android_register_function(&shmodem_function);
#ifdef CONFIG_USB_ANDROID_SH_DTFER
	android_register_function(&shdtfer_function);
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

	return 0;
}
module_init(init);


