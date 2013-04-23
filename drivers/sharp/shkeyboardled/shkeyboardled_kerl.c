/* drivers/sharp/shkeyboardled/shkeyboardled_kerl.c  (Keyboard LED Driver)
 *
 * Copyright (C) 2010 SHARP CORPORATION
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/pmic.h>

#define SHKEYBOARDLED_POWER_ENABLE
#define SHKEYBOARDLED_SHTERM_ENABLE

#ifdef SHKEYBOARDLED_POWER_ENABLE
#include "../shcamled/sh_camled_gpio_ctrl.h"
#endif /* SHKEYBOARDLED_POWER_ENABLE */

#ifdef SHKEYBOARDLED_SHTERM_ENABLE
#include <sharp/shterm_k.h>
#endif /* SHKEYBOARDLED_SHTERM_ENABLE */

#define SHKEYBOARDLED_LOG_TAG "SHKEYBOARDLEDkerl"
#define  SHKEYBOARDLED_DEBUG_LOG_ENABLE_1
//#define  SHKEYBOARDLED_DEBUG_LOG_ENABLE_2

#ifdef SHKEYBOARDLED_DEBUG_LOG_ENABLE_1
#define SHKEYBOARDLED_DEBUG_LOG_1(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHKEYBOARDLED_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHKEYBOARDLED_DEBUG_LOG_1(fmt, args...)
#endif

#ifdef SHKEYBOARDLED_DEBUG_LOG_ENABLE_2
#define SHKEYBOARDLED_DEBUG_LOG_2(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHKEYBOARDLED_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHKEYBOARDLED_DEBUG_LOG_2(fmt, args...)
#endif

#define KEYBOARDLED_MAX 2

static void shkeyboardled_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	int ret;
	uint ma;
	
	ma = value / 16;
	if( (value % 16) > 0 )
	{
		ma += 1;
	}
	if(ma > KEYBOARDLED_MAX)
	{
		ma = KEYBOARDLED_MAX;
	}
	
	SHKEYBOARDLED_DEBUG_LOG_2("value = %d, mA = %d", value, ma);
	
	ret = pmic_set_led_intensity(LED_KEYPAD, ma);
	if(ret)
	{
		SHKEYBOARDLED_DEBUG_LOG_1("pmic_set_led_intensity Error");
	}
	else
	{
#ifdef SHKEYBOARDLED_POWER_ENABLE
		ret = sh_camled_gpio_ctrl( (ma == 0 ? SH_CAMLED_GPIO102_LO : SH_CAMLED_GPIO102_HI), SH_CAMLED_USE_KEYLED_DRV );
		SHKEYBOARDLED_DEBUG_LOG_2("sh_camled_gpio_ctrl ret = %d", ret);
		if( (ret ==SH_CAMLED_GPIO_HI_ERR) || (ret == SH_CAMLED_GPIO_LO_ERR) )
		{
			SHKEYBOARDLED_DEBUG_LOG_1("sh_camled_gpio_ctrl Error %d", ret);
		}
		else
		{
#endif /* SHKEYBOARDLED_POWER_ENABLE */
			#ifdef SHKEYBOARDLED_SHTERM_ENABLE
			if(ma == 0)
			{
				if(shterm_k_set_info(SHTERM_INFO_KEYBACKLIGHT, 0) != SHTERM_SUCCESS)
				{
					SHKEYBOARDLED_DEBUG_LOG_1("shterm_k_set_info 0 Error");
				}
				else
				{
					SHKEYBOARDLED_DEBUG_LOG_2("shterm_k_set_info 0");
				}
			}
			else
			{
				if(shterm_k_set_info(SHTERM_INFO_KEYBACKLIGHT, 1) != SHTERM_SUCCESS)
				{
					SHKEYBOARDLED_DEBUG_LOG_1("shterm_k_set_info 1 Error");
				}
				else
				{
					SHKEYBOARDLED_DEBUG_LOG_2("shterm_k_set_info 1");
				}
			}
			#endif /* SHKEYBOARDLED_SHTERM_ENABLE */
#ifdef SHKEYBOARDLED_POWER_ENABLE
		}
#endif /* SHKEYBOARDLED_POWER_ENABLE */
	}
}

static struct led_classdev shkeyboardled_dev =
{
	.name			= "keyboard-backlight",
	.brightness_set	= shkeyboardled_set,
	.brightness		= LED_OFF,
};

static int shkeyboardled_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &shkeyboardled_dev);
	if (rc)
	{
		SHKEYBOARDLED_DEBUG_LOG_1("led_classdev_register Error");
		return rc;
	}
	shkeyboardled_set(&shkeyboardled_dev, LED_OFF);
	return rc;
}

static int __devexit shkeyboardled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&shkeyboardled_dev);

	return 0;
}

#ifdef CONFIG_PM
static int shkeyboardled_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&shkeyboardled_dev);

	return 0;
}

static int shkeyboardled_resume(struct platform_device *dev)
{
	led_classdev_resume(&shkeyboardled_dev);

	return 0;
}
#else
#define shkeyboardled_suspend NULL
#define shkeyboardled_resume NULL
#endif

static struct platform_driver shkeyboardled_driver = {
	.probe		= shkeyboardled_probe,
	.remove		= __devexit_p(shkeyboardled_remove),
	.suspend	= shkeyboardled_suspend,
	.resume		= shkeyboardled_resume,
	.driver		= {
		.name	= "shkeyboardled",
		.owner	= THIS_MODULE,
	},
};

static int __init shkeyboardled_init(void)
{
	return platform_driver_register(&shkeyboardled_driver);
}

static void __exit shkeyboardled_exit(void)
{
	platform_driver_unregister(&shkeyboardled_driver);
}

module_exit(shkeyboardled_exit);
module_init(shkeyboardled_init);

MODULE_DESCRIPTION("SHARP KEYBOARDLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
