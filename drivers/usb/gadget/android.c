/* drivers/usb/gadget/android.c
 * 
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#ifdef CONFIG_USB_ANDROID_SH_CUST
#include <linux/mutex.h>
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#ifdef CONFIG_USB_ANDROID_ACCESSORY
#include <linux/usb/f_accessory.h>
#endif /* CONFIG_USB_ANDROID_ACCESSORY */

#include "gadget_chips.h"

#ifdef CONFIG_USB_ANDROID_SH_CUST
#include <linux/switch.h>
#include <mach/rpc_hsusb.h>
#include <sharp/sh_android_usb_desc_str.h>
#include <sharp/sh_android_usb_apis.h>
#endif /* CONFIG_USB_ANDROID_SH_CUST */

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001


#ifdef CONFIG_USB_ANDROID_SH_CUST
/* pullup_enable_flg */
#define D_USB_ANDROID_PULLUP_NOREASON	(0)
#define D_USB_ANDROID_PULLUP_USER		(1<<0)
#define D_USB_ANDROID_PULLUP_QXDM		(1<<1)
#define D_USB_ANDROID_PULLUP_BOOTSET	(1<<2)
#define SWITCH_NAME_RNDIS "rndis_switch"
#endif /* CONFIG_USB_ANDROID_SH_CUST */


struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **functions;

	int product_id;
	int version;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	int adb_enable;
	int diag_enable;
	int current_mode;
	int default_mode;
	int charge_enable;

	int pullup_enable;
	struct switch_dev sdev;
	
	struct mutex mutex_lock;
#endif /* CONFIG_USB_ANDROID_SH_CUST */
};

static struct android_dev *_android_dev;

#define MAX_STR_LEN		16
/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

char serial_number[MAX_STR_LEN];
/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
	.bcdOTG               = __constant_cpu_to_le16(0x0200),
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static int _registered_function_count = 0;

static void android_set_default_product(int product_id);

#ifdef CONFIG_USB_ANDROID_SH_CUST
static struct device *sysfs_device = NULL;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifdef CONFIG_USB_ANDROID_SH_CUST

struct mode_to_pid_tbl {
	int		index;
	char*	string;
	int		pid_normal;
	int		pid_adb_diag;
};

static struct mode_to_pid_tbl mode_to_pid_tbl[] = 
{
	{
		D_SH_ANDROID_USB_MODE_N_CDC,
		D_SH_ANDROID_USB_MODE_CDC,
		USB_PID_MODE_CDC,
		USB_PID_MODE_CDC
	},
#ifdef CONFIG_USB_ANDROID_SH_MTP
	{
		D_SH_ANDROID_USB_MODE_N_MTP,
		D_SH_ANDROID_USB_MODE_MTP,
		USB_PID_MODE_MTP,
		USB_PID_MODE_MTP_C
	},
#endif /* CONFIG_USB_ANDROID_SH_MTP */
	{
		D_SH_ANDROID_USB_MODE_N_MSC,
		D_SH_ANDROID_USB_MODE_MSC,
		USB_PID_MODE_MSC,
		USB_PID_MODE_MSC_C
	},
	{
		D_SH_ANDROID_USB_MODE_N_RNDIS,
		D_SH_ANDROID_USB_MODE_RNDIS,
		USB_PID_MODE_RNDIS,
		USB_PID_MODE_RNDIS_C
	},
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
	{
		D_SH_ANDROID_USB_MODE_N_CDROM,
		D_SH_ANDROID_USB_MODE_CDROM,
		USB_PID_MODE_CDROM,
		USB_PID_MODE_CDROM
	},
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	{
		D_SH_ANDROID_USB_MODE_N_CDC_2,
		D_SH_ANDROID_USB_MODE_CDC_2,
		USB_PID_MODE_CDC_2,
		USB_PID_MODE_CDC_2
	},
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		D_SH_ANDROID_USB_MODE_N_ACC,
		D_SH_ANDROID_USB_MODE_ACC,
		USB_ACCESSORY_PRODUCT_ID,
		USB_ACCESSORY_ADB_PRODUCT_ID
	},
#endif /* CONFIG_USB_ANDROID_ACCESSORY */
#ifdef CONFIG_USB_ANDROID_SH_DTFER
    {
        D_SH_ANDROID_USB_MODE_N_DTFER,
        D_SH_ANDROID_USB_MODE_DTFER,
        USB_PID_MODE_DTFER,
        USB_PID_MODE_DTFER
    },
#endif /* CONFIG_USB_ANDROID_SH_DTFER */
	{
		D_SH_ANDROID_USB_MODE_N_NULL,
		NULL,
		D_SH_ANDROID_USB_PID_NULL,
		D_SH_ANDROID_USB_PID_NULL
	},
};

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SWITCH_NAME_RNDIS);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->state ? "enable" : "disable");
}

static int get_pid_from_tbl(struct mode_to_pid_tbl *tbl) 
{
	int pid;

	if (!_android_dev || !tbl)
		return -ENODEV;

	if (_android_dev->adb_enable || _android_dev->diag_enable)
		pid = tbl->pid_adb_diag;
	else
		pid = tbl->pid_normal;

	return pid;
}

static int get_pid_from_mode_index(int mode)
{
	int pid;
	struct mode_to_pid_tbl *tbl;

	for (tbl = &mode_to_pid_tbl[0]; 
		 tbl->index != D_SH_ANDROID_USB_MODE_N_NULL; tbl++) {
		if (tbl->index == mode) {
			break;
		}
	}

	pid = get_pid_from_tbl(tbl);

	return pid;
}

static int get_mode_index_from_pid(int pid)
{
	struct mode_to_pid_tbl *tbl;

	for (tbl = &mode_to_pid_tbl[0]; 
		 tbl->index != D_SH_ANDROID_USB_MODE_N_NULL; tbl++) {
		if ((tbl->pid_normal == pid) || (tbl->pid_adb_diag == pid)) {
			break;
		}
	}

	return tbl->index;
}

static int get_pid_from_mode_str(const char *str)
{
	struct mode_to_pid_tbl *tbl;
	int length;
	int pid;

	length = strlen(str);

	if (length <= 0) {
		return D_SH_ANDROID_USB_MODE_N_NULL;
	}

	for (tbl = &mode_to_pid_tbl[0];
		 tbl->index != D_SH_ANDROID_USB_MODE_N_NULL; tbl++) {
		if ( !strncmp(str, tbl->string, length) &&
			 (length == strlen(tbl->string))) {
			break;
		}
	}

	pid = get_pid_from_tbl(tbl);

	return pid;
}

static int get_str_from_mode_index(int mode, char *buf, int size)
{
	int length = 0;
	struct mode_to_pid_tbl *tbl;

	for (tbl = &mode_to_pid_tbl[0]; 
		 tbl->index != D_SH_ANDROID_USB_MODE_N_NULL; tbl++) {
		if (tbl->index == mode) {
			length = scnprintf(buf, size, "%s", tbl->string);
			break;
		}
	}

	return length;
}

static void force_reenumeration_with_pid(int pid)
{
	if (pid > 0) {
		if (_android_dev->cdev && _android_dev->cdev->gadget) {
			mutex_lock(&_android_dev->mutex_lock);
			usb_gadget_disconnect(_android_dev->cdev->gadget);

			android_set_default_product(pid);
			device_desc.idProduct = __constant_cpu_to_le16(pid);
			_android_dev->cdev->desc.idProduct = device_desc.idProduct;

#ifdef CONFIG_USB_ANDROID_ACCESSORY
			if(_android_dev->cdev->desc.idProduct == USB_ACCESSORY_PRODUCT_ID
			|| _android_dev->cdev->desc.idProduct == USB_ACCESSORY_ADB_PRODUCT_ID) {
				device_desc.idVendor = __constant_cpu_to_le16(USB_ACCESSORY_VENDOR_ID);
				_android_dev->cdev->desc.idVendor = USB_ACCESSORY_VENDOR_ID;
			} else {
				device_desc.idVendor = __constant_cpu_to_le16(0x04DD);
				_android_dev->cdev->desc.idVendor = 0x04DD;
			}
#endif /* CONFIG_USB_ANDROID_ACCESSORY */

			msleep(10);
			usb_gadget_connect(_android_dev->cdev->gadget);
			mutex_unlock(&_android_dev->mutex_lock);
		}
	}
	else 
		printk(KERN_ERR "android_usb:force_reenumeration_with_pid:pid = 0x%x\n",pid);
}

#endif /* CONFIG_USB_ANDROID_SH_CUST */

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
#ifdef CONFIG_USB_ANDROID_SH_CUST
		if (connected) {
			/* if this function is used from user-space,
			   add setting pullup-enable */
			usb_gadget_allow_pullup(_android_dev->cdev->gadget);
			usb_gadget_connect(_android_dev->cdev->gadget);
		}
#else /* CONFIG_USB_ANDROID_SH_CUST */
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int i;

	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
			f->bind_config(dev->config);
		else
			pr_err("%s: function %s not found\n", __func__, name);
	}

	/*
	 * set_alt(), or next config->bind(), sets up
	 * ep->driver_data as needed.
	 */
	usb_ep_autoconfig_reset(dev->cdev->gadget);
}

static int __ref android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	pr_debug("android_bind_config\n");
	dev->config = c;

	/* bind our functions if they have all registered */
	if (_registered_function_count == dev->num_functions)
		bind_functions(dev);

	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;

	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

#ifndef CONFIG_USB_ANDROID_SH_CUST
static int product_has_function(struct android_usb_product *p,
		struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strcmp(name, *functions++))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->disabled)
			return 0;
	}
	return 1;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->product_id;
}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static int __devinit android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, product_id, ret;

	pr_debug("android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	if (gadget_is_otg(cdev->gadget))
		android_config_driver.descriptors = otg_desc;

#ifdef CONFIG_USB_ANDROID_SH_CUST
	if (dev->charge_enable)
		usb_gadget_clear_selfpowered(gadget);
	else
		usb_gadget_set_selfpowered(gadget);
#else /* CONFIG_USB_ANDROID_SH_CUST */
	if (!usb_gadget_set_selfpowered(gadget))
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_SELFPOWER;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifndef CONFIG_USB_ANDROID_SH_CUST
	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		pr_err("%s: usb_add_config failed\n", __func__);
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

#ifndef CONFIG_USB_ANDROID_SH_CUST
	usb_gadget_set_selfpowered(gadget);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	dev->cdev = cdev;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	/* boot_mode get */
	switch ( msm_hsusb_get_boot_mode() ) {
		case 1:
			dev->current_mode = D_SH_ANDROID_USB_MODE_N_CDC_2;
			dev->pullup_enable |= D_USB_ANDROID_PULLUP_BOOTSET;
			usb_gadget_force_fullspeed(gadget);
			break;
		case 2:
			dev->current_mode = D_SH_ANDROID_USB_MODE_N_CDC_2;
			dev->pullup_enable |= D_USB_ANDROID_PULLUP_BOOTSET;
			break;
		default:
			dev->current_mode = D_SH_ANDROID_USB_MODE_N_DEF;
			break;
	}

	product_id = get_pid_from_mode_index(dev->current_mode);
	if(product_id <= 0){
		pr_err("%s: get_pid_from_mode_index failed\n", __func__);
		return ret;
	}
	dev->product_id = product_id;
	device_desc.idProduct = __constant_cpu_to_le16(dev->product_id);
	cdev->desc.idProduct = device_desc.idProduct;

	if( dev->diag_enable != 0 )
		dev->pullup_enable |= D_USB_ANDROID_PULLUP_QXDM;
	
	if (!dev->pullup_enable)
		usb_gadget_prevent_pullup(gadget);
#else /* CONFIG_USB_ANDROID_SH_CUST */
	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	return 0;
}

#ifdef CONFIG_USB_ANDROID_SH_CUST
static void android_disconnect(struct usb_composite_dev *cdev)
{
	struct usb_gadget	*gadget = cdev->gadget;
	struct usb_function		*f;
	int					online;

	if (!_android_dev || !gadget)
		return;

	online = usb_gadget_is_vbus_connected(gadget);
	if (!online) {
		_android_dev->pullup_enable &= ~D_USB_ANDROID_PULLUP_USER;
		list_for_each_entry(f, &android_config_driver.functions, list) {
			if (!strcmp(f->name, "mtp") && f->disabled == 0)
				if(f->disable)
					f->disable(f);
		}
	}
	else if (online < 0)
		printk(KERN_ERR "android_disconnect:\n");

	if (!_android_dev->pullup_enable)
		usb_gadget_prevent_pullup(gadget);

	return;
}
#endif /* CONFIG_USB_ANDROID_SH_CUST */


static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
#ifdef CONFIG_USB_ANDROID_SH_CUST
	.disconnect = android_disconnect,
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	.enable_function = android_enable_function,
};

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	pr_debug("%s: %s\n", __func__, f->name);
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	if (dev->config && _registered_function_count == dev->num_functions) {
		bind_functions(dev);
		android_set_default_product(dev->product_id);
	}
}

/**
 * android_set_function_mask() - enables functions based on selected pid.
 * @up: selected product id pointer
 *
 * This function enables functions related with selected product id.
 */
static void android_set_function_mask(struct android_usb_product *up)
{
	int index;
	struct usb_function *func;

	list_for_each_entry(func, &android_config_driver.functions, list) {
#ifdef CONFIG_USB_ANDROID_SH_CUST
		int enabled = 0;
		for (index = 0; index < up->num_functions; index++) {
			if (!strcmp(up->functions[index], func->name)) {
				enabled = 1;
				break;
			}
		}

		if (!strcmp(func->name, "diag")) {
			if (_android_dev->diag_enable == 0)
				enabled = 0;
		}
		if (!strcmp(func->name, "adb")) {
			if ((_android_dev->diag_enable == 0) && (_android_dev->adb_enable == 0))
				enabled = 0;
		}

		if (func->disabled == enabled)
			usb_function_set_enabled(func,enabled);
#else /* CONFIG_USB_ANDROID_SH_CUST */
		/* adb function enable/disable handled separetely */
		if (!strcmp(func->name, "adb"))
			continue;
		func->disabled = 1;
		for (index = 0; index < up->num_functions; index++) {
			if (!strcmp(up->functions[index], func->name))
				func->disabled = 0;
		}
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	}
}

/**
 * android_set_defaut_product() - selects default product id and enables
 * required functions
 * @product_id: default product id
 *
 * This function selects default product id using pdata information and
 * enables functions for same.
*/
static void android_set_default_product(int pid)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_product *up = dev->products;
	int index;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	int mode;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	for (index = 0; index < dev->num_products; index++, up++) {
		if (pid == up->product_id)
			break;
	}
#ifdef CONFIG_USB_ANDROID_SH_CUST
	if (index == dev->num_products) {
		pr_err("%s: deafult product %d not found\n", __func__, pid);
		return;
	}

	mode = get_mode_index_from_pid(pid);
	if (mode >= 0)
		dev->current_mode = mode;

	if (dev->cdev) {
		if ((pid == USB_PID_MODE_CDC) || (pid == USB_PID_MODE_CDC_2)) {
				dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
				dev->cdev->desc.bDeviceSubClass      = 0;
				dev->cdev->desc.bDeviceProtocol      = 0;
		}
		else {
				dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
				dev->cdev->desc.bDeviceSubClass      = 0;
				dev->cdev->desc.bDeviceProtocol      = 0;
		}
	}

#endif /* CONFIG_USB_ANDROID_SH_CUST */

	android_set_function_mask(up);
}

#ifndef CONFIG_USB_ANDROID_SH_CUST
/* dont use android_config_functions() */
/**
 * android_config_functions() - selects product id based on function need
 * to be enabled / disabled.
 * @f: usb function
 * @enable : function needs to be enable or disable
 *
 * This function selects product id having required function at first index.
 * TODO : Search of function in product id can be extended for all index.
 * RNDIS function enable/disable uses this.
*/
#ifdef CONFIG_USB_ANDROID_RNDIS
static void android_config_functions(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_product *up = dev->products;
	int index;
	char **functions;

	/* Searches for product id having function at first index */
	if (enable) {
		for (index = 0; index < dev->num_products; index++, up++) {
			functions = up->functions;
			if (!strcmp(*functions, f->name))
				break;
		}
		android_set_function_mask(up);
	} else
		android_set_default_product(dev->product_id);
}
#endif
#endif /* CONFIG_USB_ANDROID_SH_CUST */

void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int product_id;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	int current_index = dev->current_mode;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifdef CONFIG_USB_ANDROID_SH_CUST
	if (!!f->disabled == disable) {
		/* adb is only exceptinal */
		if (strcmp(f->name, "adb"))
			return;
	}

	if (!strcmp(f->name, "rndis")) {
		if (enable) {
			dev->default_mode = current_index;
			current_index = D_SH_ANDROID_USB_MODE_N_RNDIS;
			switch_set_state(&dev->sdev, 1);
		}
		else {
			switch_set_state(&dev->sdev, 0);
			return;
		}
	}
	else if (!strcmp(f->name, "diag")) {
		int set;

		set = msm_hsusb_set_qxdmen(enable);
		if (set < 0)
			return;
		dev->diag_enable = set;
		/* The change of the pullup flag along with settlement of a QXDM flag */
		if( set > 0 ){
			dev->pullup_enable |= D_USB_ANDROID_PULLUP_QXDM;
			usb_gadget_allow_pullup(dev->cdev->gadget);
		} else {
			dev->pullup_enable &= ~D_USB_ANDROID_PULLUP_QXDM;
			if (!dev->pullup_enable)
				usb_gadget_prevent_pullup(dev->cdev->gadget);
		}
	}
	else if (!strcmp(f->name, "adb")) {
		if (current_index == D_SH_ANDROID_USB_MODE_N_CDC_2) {
			return;
		}

		dev->adb_enable = enable;
	}
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	else if (!strcmp(f->name, "accessory")) {
		if (enable) {
			current_index = D_SH_ANDROID_USB_MODE_N_ACC;
		}
		else {
			current_index = D_SH_ANDROID_USB_MODE_N_MSC;
		}
	}
#endif /* CONFIG_USB_ANDROID_ACCESSORY */
	else {
		return;
	}

	product_id = get_pid_from_mode_index(current_index);
	if(product_id <= 0){
		printk(KERN_ERR "%s:product_id(%d) invalid pid\n", __func__, product_id);
		return;
	}
	force_reenumeration_with_pid(product_id);

#else /* CONFIG_USB_ANDROID_SH_CUST */
	if (!!f->disabled != disable) {
		f->disabled = disable;

#ifdef CONFIG_USB_ANDROID_RNDIS
		if (!strcmp(f->name, "rndis")) {

			/* We need to specify the COMM class in the device descriptor
			 * if we are using RNDIS.
			 */
			if (enable) {
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
				dev->cdev->desc.bDeviceClass = USB_CLASS_MISC;
				dev->cdev->desc.bDeviceSubClass      = 0x02;
				dev->cdev->desc.bDeviceProtocol      = 0x01;
#else
				dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
#endif
			} else {
				dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
				dev->cdev->desc.bDeviceSubClass      = 0;
				dev->cdev->desc.bDeviceProtocol      = 0;
			}

			android_config_functions(f, enable);
		}
#endif

		product_id = get_product_id(dev);
		device_desc.idProduct = __constant_cpu_to_le16(product_id);
		if (dev->cdev)
			dev->cdev->desc.idProduct = device_desc.idProduct;
		usb_composite_force_reset(dev->cdev);
	}
#endif /* CONFIG_USB_ANDROID_SH_CUST */
}

#ifdef CONFIG_USB_ANDROID_SH_CUST

static ssize_t show_usb_modeswitch(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int length;

	length = get_str_from_mode_index(_android_dev->current_mode, 
										buf, PAGE_SIZE);

	return length;
}
static ssize_t store_usb_modeswitch(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	char str[D_SH_ANDROID_USB_MODE_WRMAX];
	int pid;
	int length;
	int is_reenum = 1;

	if (size < D_SH_ANDROID_USB_MODE_WRMAX)
		length = size;
	else
		length = D_SH_ANDROID_USB_MODE_WRMAX - 1;

	memset(str, '\0', sizeof(str));
	memcpy(str, buf, length);

	if (str[length - 1] == '\n')
		str[length - 1] = '\0';

	pid = get_pid_from_mode_str(str);

	if (pid <= 0) {
		printk(KERN_ERR "store_usb_modeswitch:no match(%s) \n",str);
		return -EOPNOTSUPP;
	}

	if (!_android_dev->cdev || !_android_dev->cdev->gadget) {
		printk(KERN_ERR "store_usb_modeswitch:no device(%s) \n",str);
		return -ENODEV;
	}

	if (_android_dev->pullup_enable) {
		int mode_next = get_mode_index_from_pid(pid);
		if (mode_next == _android_dev->current_mode) {
			is_reenum = 0;
		}
	}

	_android_dev->pullup_enable |= D_USB_ANDROID_PULLUP_USER;
	usb_gadget_allow_pullup(_android_dev->cdev->gadget);

	if (is_reenum)
		force_reenumeration_with_pid(pid);

	printk(KERN_INFO "store_usb_modeswitch:%s,pid=0x%x \n",str,pid);

	return length;
}


static ssize_t android_show_charge_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct android_dev *adev = _android_dev;
	int value;

	if (!adev)
		return -ENODEV;

	value = snprintf(buf, PAGE_SIZE, "%d", adev->charge_enable);

	return value;
}

static ssize_t android_store_charge_enable (struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct android_dev *adev = _android_dev;
	int value;
	int ret;

	if (!adev)
		return -ENODEV;

	if ((size == 0) || (buf == NULL))
		return -EOPNOTSUPP;

	if (size >= 3)
		return -EOPNOTSUPP;

	if ((size == 2) && (*(buf+1) != 0x0a) && (*(buf+1) != 0x00))
		return -EOPNOTSUPP;

	if (*buf == '1') {
		adev->charge_enable = 1;
		value = size;
	}
	else if (*buf == '0') {
		adev->charge_enable = 0;
		value = size;
	}
	else {
		value = -EOPNOTSUPP;
	}

	if (value > 0) {
		ret = msm_hsusb_set_chgen(adev->charge_enable);
		if (ret < 0)
			printk(KERN_ERR 
				"android_store_charge_enable: err rpc(%d) \n",ret);

		if (adev->cdev && adev->cdev->gadget) {
			if (adev->charge_enable) 
				ret = usb_gadget_clear_selfpowered(adev->cdev->gadget);
			else 
				ret = usb_gadget_set_selfpowered(adev->cdev->gadget);

			if (ret < 0)
				printk(KERN_ERR 
					"android_store_charge_enable: cant set(%d) \n",ret);
		}
	}

	return value;
}


static DEVICE_ATTR(charge_enable, 0664,
		android_show_charge_enable, android_store_charge_enable);

static DEVICE_ATTR(mode, 0664,
		show_usb_modeswitch, store_usb_modeswitch);


static int android_sysfs_init(struct device *dev)
{
	int			retval;

	if (dev) {
		sysfs_device = dev;
		retval = device_create_file(sysfs_device, &dev_attr_mode);
		if (retval != 0)
			dev_err(sysfs_device,
				"failed to create sysfs entry(mode):"
				"err:(%d)\n", retval);
		retval = device_create_file(sysfs_device, &dev_attr_charge_enable);
		if (retval != 0)
			dev_err(sysfs_device,
				"failed to create sysfs entry(charge_enable):"
				"err:(%d)\n", retval);
	}

	return 0;
}

static void android_sysfs_cleanup(void)
{
	if (sysfs_device) {
		device_remove_file(sysfs_device, &dev_attr_charge_enable);
		device_remove_file(sysfs_device, &dev_attr_mode);
	}

}

#endif /* CONFIG_USB_ANDROID_SH_CUST */


#ifdef CONFIG_DEBUG_FS
static int android_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t android_debugfs_serialno_write(struct file *file, const char
				__user *buf,	size_t count, loff_t *ppos)
{
	char str_buf[MAX_STR_LEN];

	if (count > MAX_STR_LEN)
		return -EFAULT;

	if (copy_from_user(str_buf, buf, count))
		return -EFAULT;

	memcpy(serial_number, str_buf, count);

	if (serial_number[count - 1] == '\n')
		serial_number[count - 1] = '\0';

	strings_dev[STRING_SERIAL_IDX].s = serial_number;

	return count;
}
const struct file_operations android_fops = {
	.open	= android_debugfs_open,
	.write	= android_debugfs_serialno_write,
};

struct dentry *android_debug_root;
struct dentry *android_debug_serialno;

static int android_debugfs_init(struct android_dev *dev)
{
	android_debug_root = debugfs_create_dir("android", NULL);
	if (!android_debug_root)
		return -ENOENT;

	android_debug_serialno = debugfs_create_file("serial_number", 0222,
						android_debug_root, dev,
						&android_fops);
	if (!android_debug_serialno) {
		debugfs_remove(android_debug_root);
		android_debug_root = NULL;
		return -ENOENT;
	}
	return 0;
}

static void android_debugfs_cleanup(void)
{
       debugfs_remove(android_debug_serialno);
       debugfs_remove(android_debug_root);
}
#endif
static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int result;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	int rpc_ret;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	dev_dbg(&pdev->dev, "%s: pdata: %p\n", __func__, pdata);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	result = pm_runtime_get(&pdev->dev);
	if (result < 0) {
		dev_err(&pdev->dev,
			"Runtime PM: Unable to wake up the device, rc = %d\n",
			result);
		return result;
	}

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
	}
#ifdef CONFIG_USB_ANDROID_SH_CUST
	dev->sdev.name = SWITCH_NAME_RNDIS;
	dev->sdev.print_name = print_switch_name;
	dev->sdev.print_state = print_switch_state;

	result = switch_dev_register(&dev->sdev);
	if (result)
		pr_info("%s: switch_dev_register failed\n", __func__);

	result = android_sysfs_init(&pdev->dev);
	if (result)
		pr_info("%s: android_sysfs_init failed\n", __func__);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
#ifdef CONFIG_DEBUG_FS
	result = android_debugfs_init(dev);
	if (result)
		pr_debug("%s: android_debugfs_init failed\n", __func__);
#endif
#ifdef CONFIG_USB_ANDROID_SH_CUST
	/* maybe otg driver already called it. call it */
	msm_hsusb_rpc_connect();
	rpc_ret = msm_hsusb_get_qxdmen();
	if (rpc_ret <= 0)
		dev->diag_enable = 0;
	else
		dev->diag_enable = !!rpc_ret;

	rpc_ret = msm_hsusb_get_chgen();
	if (rpc_ret <= 0)
		dev->charge_enable = 0;
	else
		dev->charge_enable = !!rpc_ret;

	mutex_init(&dev->mutex_lock);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	return usb_composite_register(&android_usb_driver);
}

static int andr_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int andr_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static struct dev_pm_ops andr_dev_pm_ops = {
	.runtime_suspend = andr_runtime_suspend,
	.runtime_resume = andr_runtime_resume,
};

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", .pm = &andr_dev_pm_ops},
};

static int __init init(void)
{
	struct android_dev *dev;

	pr_debug("android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	_android_dev = dev;

	return platform_driver_probe(&android_platform_driver, android_probe);
}
module_init(init);

static void __exit cleanup(void)
{
#ifdef CONFIG_DEBUG_FS
	android_debugfs_cleanup();
#endif
#ifdef CONFIG_USB_ANDROID_SH_CUST
	android_sysfs_cleanup();
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	usb_composite_unregister(&android_usb_driver);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
