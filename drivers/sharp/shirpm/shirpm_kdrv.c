/* drivers/sharp/shirpm/shirpm_kdrv.c (Infrared driver Power Management)
 *
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "sharp/shirpm_kdrv.h"

#include <sharp/shlcdc_kerl.h>

#include <mach/vreg.h>

#define	SH_IRPOWER_KERNEL_DRIVER_VERSION	"1.00.01"

#define IRPOWER_NAME	"shirpm_kern"

#define IRPOWERLOG_ALERT(format, args...)	\
	printk(KERN_ALERT "[%s] " format, IRPOWER_NAME, ##args)
#define IRPOWERLOG_ERROR(format, args...)	\
	printk(KERN_ERR "[%s] " format, IRPOWER_NAME, ##args)
#define IRPOWERLOG_WARNING(format, args...)	\
	printk(KERN_WARNING "[%s] " format, IRPOWER_NAME, ##args)
#define IRPOWERLOG_INFO(format, args...)
#define IRPOWERLOG_DEBUG(format, args...)

#define IRPOWER_KERNEL_API_SUCCESS	(0)
#define IRPOWER_KERNEL_API_ERROR	(-1)

#define IRPOWER_DEVTYPE_IRDA		(0)
#define	IRPOWER_DEVTYPE_IRRC		(1)
static	char	ResourceStatus[2];

#define	IR_LEDA_CTL_VREG_ID	"gp10"
#define	IR_LEDA_CTL_VREG_V	2850

#define IRPM_VERSION_INIT_STR	"0.00.00"

typedef struct {
	char irpm_user_drv_version[16];
	char irda_user_drv_version[16];
	char irrc_user_drv_version[16];
	char stack_version[16];
} irpm_udrv_info;
static	irpm_udrv_info			user_drv_info;

#define IRPM_KERNEL_OFF			(0)
#define IRPM_KERNEL_ON			(1)







static int irpower_resource_ctl(int devtype, char ctltype);
static int irpower_resource_free(int devtype);
static int irpower_resource_get(int devtype);
static int irpower_set_power_mode(int power);
static int irpower_set_leda_power(int power);

static int irpower_resource_ctl(int devtype, char ctltype)
{
	int	result = IRPOWER_KERNEL_API_SUCCESS;

	switch (ctltype) {
	case IRPOWER_DRV_RESOURCE_FREE:
		result = irpower_resource_free(devtype);
		break;

	case IRPOWER_DRV_RESOURCE_GET:
		result = irpower_resource_get(devtype);
		break;

	default:
		IRPOWERLOG_ERROR("parametar error ctltype=%d\n", ctltype);
		result = -EINVAL;
		break;
	}

	return result;
}

static int irpower_resource_free(int devtype)
{
	int result = IRPOWER_KERNEL_API_SUCCESS;
	int otherdev;

	if (ResourceStatus[devtype] == IRPOWER_DRV_RESOURCE_NOT_USE) {
		IRPOWERLOG_ERROR(
			"resource not using (devtype=%d)\n", devtype);
		result = -EALREADY;
	}

	if (devtype == IRPOWER_DEVTYPE_IRDA) {
		otherdev = IRPOWER_DEVTYPE_IRRC;
	} else {
		otherdev = IRPOWER_DEVTYPE_IRDA;
	}
	if (ResourceStatus[otherdev] == IRPOWER_DRV_RESOURCE_USING) {
		IRPOWERLOG_ERROR(
			"resource busy (devtype=%d)\n", devtype);
		result = -EBUSY;
	}

	if (result == IRPOWER_KERNEL_API_SUCCESS) {
		result = (irpower_set_leda_power(IRPM_KERNEL_OFF)
			== IRPOWER_KERNEL_API_SUCCESS)
			? result : (result - 1);

		result = (irpower_set_power_mode(IRPM_KERNEL_OFF)
			== IRPOWER_KERNEL_API_SUCCESS)
			? result : (result - 1);

		result = (result == IRPOWER_KERNEL_API_SUCCESS)
			? result : (-EIO);


		IRPOWERLOG_DEBUG("change status %d->%d",
					ResourceStatus[devtype],
					IRPOWER_DRV_RESOURCE_NOT_USE);
		ResourceStatus[devtype] = IRPOWER_DRV_RESOURCE_NOT_USE;
	}

	return result;
}

static int irpower_resource_get(int devtype)
{
	int		result = IRPOWER_KERNEL_API_SUCCESS;
	int		otherdev;

	if (ResourceStatus[devtype] == IRPOWER_DRV_RESOURCE_USING) {
		IRPOWERLOG_ERROR(
			"resource already using (devtype=%d)\n", devtype);
		result = -EALREADY;
	}

	if (devtype == IRPOWER_DEVTYPE_IRDA) {
		otherdev = IRPOWER_DEVTYPE_IRRC;
	} else {
		otherdev = IRPOWER_DEVTYPE_IRDA;
	}
	if (ResourceStatus[otherdev] == IRPOWER_DRV_RESOURCE_USING) {
		IRPOWERLOG_ERROR(
			"resource busy (devtype=%d)\n", devtype);
		result = -EBUSY;
	}


	if (result == IRPOWER_KERNEL_API_SUCCESS) {
		result = irpower_set_power_mode(IRPM_KERNEL_ON);
	}

	if (result == IRPOWER_KERNEL_API_SUCCESS) {
		result = irpower_set_leda_power(IRPM_KERNEL_ON);

		if (result != IRPOWER_KERNEL_API_SUCCESS) {
			irpower_set_power_mode(IRPM_KERNEL_OFF);
		}
	}


	if (result == IRPOWER_KERNEL_API_SUCCESS) {
		IRPOWERLOG_DEBUG("change status %d->%d",
					ResourceStatus[devtype],
					IRPOWER_DRV_RESOURCE_USING);
		ResourceStatus[devtype] = IRPOWER_DRV_RESOURCE_USING;
	}

	return result;
}






static int irpower_set_power_mode(int power)
{
	int result = IRPOWER_KERNEL_API_SUCCESS;
	int lcdc_request = SHLCDC_DEV_PWR_ON;

	if (power == IRPM_KERNEL_ON) {
		lcdc_request = SHLCDC_DEV_PWR_ON;
	} else {
		lcdc_request = SHLCDC_DEV_PWR_OFF;
	}

	result = (shlcdc_api_set_power_mode(SHLCDC_DEV_TYPE_IR, lcdc_request)
		== SHLCDC_RESULT_SUCCESS)
		? IRPOWER_KERNEL_API_SUCCESS
		: (-EIO);

	if (result != IRPOWER_KERNEL_API_SUCCESS) {
		IRPOWERLOG_ERROR("shlcdc_api_set_power_mode(%d) failed\n",
			lcdc_request);
	}

	return result;
}

static int irpower_set_leda_power(int power)
{
	int result = IRPOWER_KERNEL_API_SUCCESS;
	struct vreg	*vreg = NULL;

	vreg = vreg_get(NULL, IR_LEDA_CTL_VREG_ID);

	if (IS_ERR(vreg)) {
		IRPOWERLOG_ERROR("LEDA power-off failed\n");
		return (-EIO);
	}

	if (power == IRPM_KERNEL_ON) {
		vreg_set_level(vreg, IR_LEDA_CTL_VREG_V);
		vreg_enable(vreg);
	} else {
		vreg_disable(vreg);
	}


	return result;
}





static int irpower_set_udrv_info(int devtype, irpower_resource_write_info *info)
{
	int	result = IRPOWER_KERNEL_API_SUCCESS;

	memcpy(user_drv_info.irpm_user_drv_version, info->irpm_user_drv_version,
				sizeof(user_drv_info.irpm_user_drv_version));
	user_drv_info.irpm_user_drv_version[
			sizeof(user_drv_info.irpm_user_drv_version) - 1] = '\0';

	if (devtype == IRPOWER_DEVTYPE_IRDA) {
		memcpy(user_drv_info.irda_user_drv_version,
							info->user_drv_version,
				sizeof(user_drv_info.irda_user_drv_version));
		user_drv_info.irda_user_drv_version[
			sizeof(user_drv_info.irda_user_drv_version) - 1] = '\0';

		memcpy(user_drv_info.stack_version, info->stack_version,
					sizeof(user_drv_info.stack_version));
		user_drv_info.stack_version[
				sizeof(user_drv_info.stack_version) - 1] = '\0';
	} else {
		memcpy(user_drv_info.irrc_user_drv_version,
							info->user_drv_version,
				sizeof(user_drv_info.irrc_user_drv_version));
		user_drv_info.irrc_user_drv_version[
			sizeof(user_drv_info.irrc_user_drv_version) - 1] = '\0';
	}

	return result;
}

static ssize_t show_irpower_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;

	IRPOWERLOG_INFO("Show version\n");

	result = scnprintf(buf, PAGE_SIZE, "IrPM(K) %s\nIrPM(U) %s\nIrDA(U) %s\nIrRC(U) %s\nIrDA(S) %s\n",
				SH_IRPOWER_KERNEL_DRIVER_VERSION,
				user_drv_info.irpm_user_drv_version,
				user_drv_info.irda_user_drv_version,
				user_drv_info.irrc_user_drv_version,
				user_drv_info.stack_version);

	IRPOWERLOG_DEBUG("OUT buf=%s", buf);

	return result;
}

static ssize_t show_irpower_irdaresource(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;

	IRPOWERLOG_INFO("Show IrDA resource\n");

	result = scnprintf(buf, PAGE_SIZE, "%1d\n",
			ResourceStatus[IRPOWER_DEVTYPE_IRDA]);

	IRPOWERLOG_DEBUG("OUT buf=%s", buf);

	return result;
}

static ssize_t show_irpower_irrcresource(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;

	IRPOWERLOG_INFO("Show IrRC resource\n");

	result = scnprintf(buf, PAGE_SIZE, "%1d\n",
			ResourceStatus[IRPOWER_DEVTYPE_IRRC]);

	IRPOWERLOG_DEBUG("OUT buf=%s", buf);

	return result;
}

static ssize_t set_irpower_irdaresource(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t result = count;

	irpower_resource_write_info *info = (irpower_resource_write_info *)buf;

	IRPOWERLOG_INFO("Set IrDA resource (buf=%s)\n", info->ir_device_use);

	result = irpower_resource_ctl(IRPOWER_DEVTYPE_IRDA,
							info->ir_device_use[0]);

	IRPOWERLOG_DEBUG("OUT (result=%d)\n", result);

	if (count >= sizeof(irpower_resource_write_info)) {
		irpower_set_udrv_info(IRPOWER_DEVTYPE_IRDA, info);
	}

	return result;
}

static ssize_t set_irpower_irrcresource(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t result = count;

	irpower_resource_write_info *info = (irpower_resource_write_info *)buf;

	IRPOWERLOG_INFO("Set IrRC resource (buf=%s)\n", info->ir_device_use);

	result = irpower_resource_ctl(IRPOWER_DEVTYPE_IRRC,
							info->ir_device_use[0]);

	IRPOWERLOG_DEBUG("OUT (result=%d)\n", result);

	if (count >= sizeof(irpower_resource_write_info)) {
		irpower_set_udrv_info(IRPOWER_DEVTYPE_IRRC, info);
	}

	return result;
}

static DEVICE_ATTR(
	version,
	S_IRUGO,
	show_irpower_version,
	NULL
);

static DEVICE_ATTR(
	irdaresource,
	S_IRUGO | S_IWUSR | S_IWGRP,
	show_irpower_irdaresource,
	set_irpower_irdaresource
);

static DEVICE_ATTR(
	irrcresource,
	S_IRUGO | S_IWUSR | S_IWGRP,
	show_irpower_irrcresource,
	set_irpower_irrcresource
);

static struct attribute *irpower_device_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_irdaresource.attr,
	&dev_attr_irrcresource.attr,
	NULL,
};

static struct attribute_group irpower_device_attributes_gourp = {
	.attrs = irpower_device_attributes,
};

static int irpower_driver_remove(struct platform_device *pdev)
{
	int result = IRPOWER_KERNEL_API_SUCCESS;

	sysfs_remove_group(&(pdev->dev.kobj),
		&irpower_device_attributes_gourp);

	if ((ResourceStatus[IRPOWER_DEVTYPE_IRDA] ==
		IRPOWER_DRV_RESOURCE_USING) ||
		(ResourceStatus[IRPOWER_DEVTYPE_IRRC] ==
		IRPOWER_DRV_RESOURCE_USING)) {
		result = (irpower_set_leda_power(IRPM_KERNEL_OFF)
			== IRPOWER_KERNEL_API_SUCCESS)
			? result : (result - 1);

		result = (irpower_set_power_mode(IRPM_KERNEL_OFF)
			== IRPOWER_KERNEL_API_SUCCESS)
			? result : (result - 1);

		result = (result == IRPOWER_KERNEL_API_SUCCESS)
			? result : (-EIO);

	}
	ResourceStatus[IRPOWER_DEVTYPE_IRDA] = IRPOWER_DRV_RESOURCE_NOT_USE;
	ResourceStatus[IRPOWER_DEVTYPE_IRRC] = IRPOWER_DRV_RESOURCE_NOT_USE;

	return result;
}

static struct platform_driver irpower_driver = {
	.remove = __devexit_p(irpower_driver_remove),
	.driver = {
		.name = IRPOWER_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int irpower_driver_init(void)
{
	int result;

	result = platform_driver_register(&irpower_driver);
	if (result != 0) {
		IRPOWERLOG_ERROR(
			"platform_driver_register() failed (%d)\n", result);
	}

	return result;
}

static struct platform_device irpower_device = {
	.name	= IRPOWER_DRIVER_NAME,
	.id	= -1,
};

static int __init irpower_device_init(void)
{
	int result = 0;

	result = platform_device_register(&irpower_device);
	if (result != 0) {
		IRPOWERLOG_ERROR(
			"platform_device_register() failed (%d)\n", result);
	}

	result = sysfs_create_group(&(irpower_device.dev.kobj),
					&irpower_device_attributes_gourp);
	if (result != 0) {
		IRPOWERLOG_ERROR(
			"sysfs_create_group() failed (%d)\n", result);
	}

	return result;
}

static int __init irpower_kdrv_module_init(void)
{
	int result;

	result = irpower_device_init();
	if (result == 0) {
		result = irpower_driver_init();
	}

	ResourceStatus[IRPOWER_DEVTYPE_IRDA] = IRPOWER_DRV_RESOURCE_NOT_USE;
	ResourceStatus[IRPOWER_DEVTYPE_IRRC] = IRPOWER_DRV_RESOURCE_NOT_USE;

	strcpy(user_drv_info.irpm_user_drv_version, IRPM_VERSION_INIT_STR);
	strcpy(user_drv_info.irda_user_drv_version, IRPM_VERSION_INIT_STR);
	strcpy(user_drv_info.irrc_user_drv_version, IRPM_VERSION_INIT_STR);
	strcpy(user_drv_info.stack_version, IRPM_VERSION_INIT_STR);

	return result;
}

static void __exit irpower_kdrv_module_exit(void)
{
	platform_driver_unregister(&irpower_driver);
	platform_device_unregister(&irpower_device);
}

module_init(irpower_kdrv_module_init);
module_exit(irpower_kdrv_module_exit);

MODULE_DESCRIPTION("IrPower driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION(SH_IRPOWER_KERNEL_DRIVER_VERSION);
