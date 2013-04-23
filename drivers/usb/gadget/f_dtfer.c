/* drivers/usb/gadget/f_dtfer.c
 * 
 * f_dtfer.c - Data Transfer function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This code also borrows from f_serial.c, which is
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/android_composite.h>

#include "u_serial.h"
#include "gadget_chips.h"

#include <linux/switch.h>
#include <sharp/sh_android_usb_desc_str.h>

/*-------------------------------------------------------------------------*/

#define SWITCH_NAME_ALT_DTFER	"dtfer_open_sts"

/*-------------------------------------------------------------------------*/

//#define D_SH_DTFER_DEBUG_LOG

/*-------------------------------------------------------------------------*/

/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

struct dtfer_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_dtfer {
	struct gserial		port;
	u8					data_id;
	u8					port_num;

	struct dtfer_descs	fs;
	struct dtfer_descs	hs;
};

static inline struct f_dtfer *func_to_dtfer(struct usb_function *f)
{
	return container_of(f, struct f_dtfer, port.func);
}

struct switch_dtfer_open_sts {
	struct switch_dev		sdev;
	struct work_struct		swork;
	int						open_sts;
};

static struct switch_dtfer_open_sts *switch_dtfer;

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor dtfer_interface_desc __initdata = {
	.bLength			= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= 0,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 2,
	.bInterfaceClass	= USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass	= USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol	= 0xFF,
};

/* full speed support: */

static struct usb_endpoint_descriptor dtfer_fs_in_desc __initdata = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor dtfer_fs_out_desc __initdata = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *dtfer_fs_function[] __initdata = {
	(struct usb_descriptor_header *) &dtfer_interface_desc,
	(struct usb_descriptor_header *) &dtfer_fs_in_desc,
	(struct usb_descriptor_header *) &dtfer_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor dtfer_hs_in_desc __initdata = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor dtfer_hs_out_desc __initdata = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_descriptor_header *dtfer_hs_function[] __initdata = {
	(struct usb_descriptor_header *) &dtfer_interface_desc,
	(struct usb_descriptor_header *) &dtfer_hs_in_desc,
	(struct usb_descriptor_header *) &dtfer_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string dtfer_string_defs[] = {
	[0].s = USB_DTFER_STRING_DESC_WORD,
	{  } /* end of list */
};

static struct usb_gadget_strings dtfer_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		dtfer_string_defs,
};

static struct usb_gadget_strings *dtfer_strings[] = {
	&dtfer_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

static ssize_t print_switch_name_alt_sts_dtfer(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SWITCH_NAME_ALT_DTFER);
}

static ssize_t print_switch_state_alt_sts_dtfer(struct switch_dev *sdev, char *buf)
{
	if (switch_dtfer)
		return sprintf(buf, "%s\n", (switch_dtfer->open_sts ? "dtfer_open" : "dtfer_close"));
	else
		return -ENODEV;
}

static int dtfer_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_dtfer		*dtfer = func_to_dtfer(f);
	struct usb_composite_dev *cdev = f->config->cdev;

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s f=[%p],intf=[%d],alt=[%d]\n",
		   __func__, f, intf, alt);
#endif /* D_SH_DTFER_DEBUG_LOG */

	/* we know alt == 0, so this is an activation or a reset */

	if (dtfer->port.in->driver_data) {
		DBG(cdev, "reset generic ttyGS%d\n", dtfer->port_num);
		gserial_disconnect(&dtfer->port);
	} else {
		DBG(cdev, "activate generic ttyGS%d\n", dtfer->port_num);
	}

	dtfer->port.in_desc = ep_choose(cdev->gadget,
			dtfer->hs.in, dtfer->fs.in);
	dtfer->port.out_desc = ep_choose(cdev->gadget,
			dtfer->hs.out, dtfer->fs.out);

	gserial_connect(&dtfer->port, dtfer->port_num);

	if (switch_dtfer) {
		switch_dtfer->open_sts = 1;
		schedule_work(&switch_dtfer->swork);
	}

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

	return 0;
}

static void dtfer_disable(struct usb_function *f)
{
	struct f_dtfer	*dtfer = func_to_dtfer(f);
	struct usb_composite_dev *cdev = f->config->cdev;

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s f=[%p]\n", __func__, f);
#endif /* D_SH_DTFER_DEBUG_LOG */

	DBG(cdev, "generic ttyGS%d deactivated\n", dtfer->port_num);
	gserial_disconnect(&dtfer->port);

	if (switch_dtfer && switch_dtfer->open_sts) {
		switch_dtfer->open_sts = 0;
		schedule_work(&switch_dtfer->swork);
	}

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int __init
dtfer_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_dtfer		*dtfer = func_to_dtfer(f);
	int			status;
	struct usb_ep		*ep;

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s c=[%p],f=[%p]\n", __func__, c, f);
#endif /* D_SH_DTFER_DEBUG_LOG */

	if (!switch_dtfer) {
		status = -ENODEV;
		goto fail;
	}
	status = switch_dev_register(&switch_dtfer->sdev);
	if (status < 0)
		goto fail;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	dtfer->data_id = status;
	dtfer_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &dtfer_fs_in_desc);
	if (!ep)
		goto fail;
	dtfer->port.in = ep;
	ep->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, &dtfer_fs_out_desc);
	if (!ep)
		goto fail;
	dtfer->port.out = ep;
	ep->driver_data = cdev;

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(dtfer_fs_function);

	dtfer->fs.in = usb_find_endpoint(dtfer_fs_function,
			f->descriptors, &dtfer_fs_in_desc);
	dtfer->fs.out = usb_find_endpoint(dtfer_fs_function,
			f->descriptors, &dtfer_fs_out_desc);


	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		dtfer_hs_in_desc.bEndpointAddress =
				dtfer_fs_in_desc.bEndpointAddress;
		dtfer_hs_out_desc.bEndpointAddress =
				dtfer_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(dtfer_hs_function);

		dtfer->hs.in = usb_find_endpoint(dtfer_hs_function,
				f->hs_descriptors, &dtfer_hs_in_desc);
		dtfer->hs.out = usb_find_endpoint(dtfer_hs_function,
				f->hs_descriptors, &dtfer_hs_out_desc);
	}

	DBG(cdev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
			dtfer->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			dtfer->port.in->name, dtfer->port.out->name);

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

	return 0;

fail:
	/* we might as well release our claims on endpoints */
	if (dtfer->port.out)
		dtfer->port.out->driver_data = NULL;
	if (dtfer->port.in)
		dtfer->port.in->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

	return status;
}

static void
dtfer_unbind(struct usb_configuration *c, struct usb_function *f)
{

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s c=[%p],f=[%p]\n", __func__, c, f);
#endif /* D_SH_DTFER_DEBUG_LOG */

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(func_to_dtfer(f));

	if (switch_dtfer) {
		switch_dev_unregister(&switch_dtfer->sdev);
		kfree(switch_dtfer);
		switch_dtfer = NULL;
	}

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

}

static void dtfer_reset_descriptor(struct usb_configuration *c, 
                      struct usb_function *f,  
                      struct usb_descriptor_header **descriptors ,
                      u8 bInterfaceNumber)
{
	struct f_dtfer		*dtfer = func_to_dtfer(f);
	struct usb_descriptor_header *descriptor;
	struct usb_interface_descriptor *intf;

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s c=[%p],f=[%p],desc=[%p],IFNumber=[%d]\n",
		   __func__, c, f, descriptors, bInterfaceNumber);
#endif /* D_SH_DTFER_DEBUG_LOG */

	/* suppose that first descriptor is interface descriptor */
	if ((descriptor = *descriptors++) == NULL) {
#ifdef D_SH_DTFER_DEBUG_LOG
		printk(KERN_INFO "[USB/Drv]<<%s NULL\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */
		return;
	}
	if (descriptor->bDescriptorType != USB_DT_INTERFACE) {
#ifdef D_SH_DTFER_DEBUG_LOG
		printk(KERN_INFO "[USB/Drv]<<%s !USB_DT_INTERFACE\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */
		return;
	}

	intf = (struct usb_interface_descriptor *)descriptor;
	if ((intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC) &&
		(intf->bInterfaceSubClass == USB_SUBCLASS_VENDOR_SPEC))
		dtfer->data_id = bInterfaceNumber;
	else if (intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC) {
		if (dtfer->data_id != bInterfaceNumber)
			printk(KERN_ERR "dtfer_reset_descriptor() : !bInterfaceNumber\n");
	}
	else
		printk("dtfer_reset_descriptor() : dont match!\n");

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

	return;
}

static void dtfer_suspend(struct usb_function *f)
{
	struct f_dtfer	*dtfer = func_to_dtfer(f);
	struct usb_composite_dev *cdev = f->config->cdev;

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s f=[%p]\n", __func__, f);
#endif /* D_SH_DTFER_DEBUG_LOG */

	DBG(cdev, "dtfer ttyGS%d suspend\n", dtfer->port_num);
	dtfer_disable(f);

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */

}

static void dtfer_setinterface_switch(struct work_struct *w)
{
	if (switch_dtfer)
		switch_set_state(&switch_dtfer->sdev, switch_dtfer->open_sts);
}

/**
 * dtfer_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int __init dtfer_bind_config(struct usb_configuration *c ,u8 port_num)
{
	struct f_dtfer	*dtfer;
	int		status;

#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]>>%s c=[%p]\n", __func__, c);
#endif /* D_SH_DTFER_DEBUG_LOG */

	BUG_ON(switch_dtfer);

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (dtfer_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0) {
#ifdef D_SH_DTFER_DEBUG_LOG
			printk(KERN_INFO "[USB/Drv]<<%s status=[%d]\n", __func__, status);
#endif /* D_SH_DTFER_DEBUG_LOG */
			return status;
		}
		dtfer_string_defs[0].id = status;
		dtfer_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	dtfer = kzalloc(sizeof *dtfer, GFP_KERNEL);
	if (!dtfer) {
#ifdef D_SH_DTFER_DEBUG_LOG
		printk(KERN_INFO "[USB/Drv]<<%s -ENOMEM\n", __func__);
#endif /* D_SH_DTFER_DEBUG_LOG */
		return -ENOMEM;
	}

	dtfer->port_num = port_num;

	dtfer->port.func.name = "dtfer";
	dtfer->port.func.strings = dtfer_strings;
	dtfer->port.func.bind = dtfer_bind;
	dtfer->port.func.unbind = dtfer_unbind;
	dtfer->port.func.set_alt = dtfer_set_alt;
	dtfer->port.func.disable = dtfer_disable;
	dtfer->port.func.reset_descriptor = dtfer_reset_descriptor;
	dtfer->port.func.suspend = dtfer_suspend;

	switch_dtfer = kzalloc(sizeof *switch_dtfer, GFP_KERNEL);
	if (!switch_dtfer) {
		status = -ENOMEM;
		goto fail;
	}

	switch_dtfer->sdev.name = SWITCH_NAME_ALT_DTFER;
	switch_dtfer->sdev.print_name = print_switch_name_alt_sts_dtfer;
	switch_dtfer->sdev.print_state = print_switch_state_alt_sts_dtfer;

	INIT_WORK(&switch_dtfer->swork, dtfer_setinterface_switch);

	status = usb_add_function(c, &dtfer->port.func);
	
	if (!status)
		goto done;
	if(switch_dtfer)
		kfree(switch_dtfer);
		
fail:
	if(dtfer)
		kfree(dtfer);

done:
#ifdef D_SH_DTFER_DEBUG_LOG
	printk(KERN_INFO "[USB/Drv]<<%s status=[%d]\n", __func__, status);
#endif /* D_SH_DTFER_DEBUG_LOG */

	return status;
}

MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL");
