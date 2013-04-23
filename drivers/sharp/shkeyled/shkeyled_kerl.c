/* drivers/sharp/shkeyled/shkeyled_kerl.c  (Key LED Driver)
 *
 * Copyright (C) 2010-2011 SHARP CORPORATION
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

#include <sharp/sh_boot_manager.h>

#if (CONFIG_LEDS_KEY_LED_PORT == 0)
#define SHKEYLED_PORT_TYPE_0
#elif (CONFIG_LEDS_KEY_LED_PORT == 1)
#define SHKEYLED_PORT_TYPE_1
#elif (CONFIG_LEDS_KEY_LED_PORT == 2)
#define SHKEYLED_PORT_TYPE_2
#else
#define SHKEYLED_PORT_TYPE_0
#endif

#if defined(SHKEYLED_PORT_TYPE_0)
#define SHKEYLED_SHTERM_ENABLE
#elif defined(SHKEYLED_PORT_TYPE_1)
#define SHKEYLED_SHTERM_ENABLE
#elif defined(SHKEYLED_PORT_TYPE_2)
#define SHKEYLED_SHTERM_ENABLE
#else
#endif

#ifdef SHKEYLED_SHTERM_ENABLE
#include <sharp/shterm_k.h>
#endif /* SHKEYLED_SHTERM_ENABLE */

#if defined(SHKEYLED_PORT_TYPE_2)
//#define SHKEYLED_POWER_ENABLE

#ifdef SHKEYLED_POWER_ENABLE
#include "../shcamled/sh_camled_gpio_ctrl.h"
#endif /* SHKEYLED_POWER_ENABLE */
#endif /* SHKEYLED_PORT_TYPE_2 */

#define SHKEYLED_LOG_TAG "SHKEYLEDkerl"
#define  SHKEYLED_DEBUG_LOG_ENABLE_1
//#define  SHKEYLED_DEBUG_LOG_ENABLE_2

#ifdef SHKEYLED_DEBUG_LOG_ENABLE_1
#define SHKEYLED_DEBUG_LOG_1(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHKEYLED_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHKEYLED_DEBUG_LOG_1(fmt, args...)
#endif

#ifdef SHKEYLED_DEBUG_LOG_ENABLE_2
#define SHKEYLED_DEBUG_LOG_2(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHKEYLED_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHKEYLED_DEBUG_LOG_2(fmt, args...)
#endif

#if defined(SHKEYLED_PORT_TYPE_0)
	#define KEYLED_FULL			20
	#define KEYLED_CURRENT_MAX	40
	#define KEYLED_CURRENT_UP	2

	#define KEYLED_FULL2		20
	#define KEYLED_CURRENT_MAX2	150
	#define KEYLED_CURRENT_UP2	10
#elif defined(SHKEYLED_PORT_TYPE_1)
	#define KEYLED_FULL			2
	#define KEYLED_CURRENT_MAX	40
	#define KEYLED_CURRENT_UP	2
#elif defined(SHKEYLED_PORT_TYPE_2)
#if 0
	#define KEYLED_FULL			2
	#define KEYLED_STEP			1
	
	#define KEYLED_FULL2		40
	#define KEYLED_STEP2		2
#else
	#define KEYLED_FULL			12
	#define KEYLED_CURRENT_MAX	40
	#define KEYLED_CURRENT_UP	2
#endif
#else
	#define KEYLED_FULL			2
	#define KEYLED_CURRENT_MAX	40
	#define KEYLED_CURRENT_UP	2
#endif

#if defined(CONFIG_LEDS_KEY_KEYBOARDLED)
#if defined(SHKEYLED_PORT_TYPE_0)
	#define KEYBOARDLED_FULL			20
	#define KEYBOARDLED_CURRENT_MAX		40
	#define KEYBOARDLED_CURRENT_UP		2
#elif defined(SHKEYLED_PORT_TYPE_1)
	#define KEYBOARDLED_FULL			2
	#define KEYBOARDLED_CURRENT_MAX		40
	#define KEYBOARDLED_CURRENT_UP		2
#elif defined(SHKEYLED_PORT_TYPE_2)
	#define KEYBOARDLED_FULL			12
	#define KEYBOARDLED_CURRENT_MAX		40
	#define KEYBOARDLED_CURRENT_UP		2
#else
	#define KEYBOARDLED_FULL			2
	#define KEYBOARDLED_CURRENT_MAX		40
	#define KEYBOARDLED_CURRENT_UP		2
#endif
#endif

enum {
	SHDISP_HW_TYPE_0ES  = 0x03,
	SHDISP_HW_TYPE_1ES  = 0x02,
	SHDISP_HW_TYPE_1PP  = 0x01,
	SHDISP_HW_TYPE_15PP = 0x04,
	SHDISP_HW_TYPE_2PP  = 0x00,
	SHDISP_HW_TYPE_25PP = 0x05,
	NUM_SHDISP_HW_TYPE  = 0xFF
};

uint	shkeyled_ma = 0;
uint	shkeyled_full        = KEYLED_FULL;
#if defined(SHKEYLED_PORT_TYPE_0) || defined(SHKEYLED_PORT_TYPE_1)
uint	shkeyled_current_max = KEYLED_CURRENT_MAX;
uint	shkeyled_current_up  = KEYLED_CURRENT_UP;
#endif /* SHKEYLED_PORT_TYPE_0 || SHKEYLED_PORT_TYPE_1 */
#if defined(SHKEYLED_PORT_TYPE_0) || defined(SHKEYLED_PORT_TYPE_2)
uint	shkeyled_hw_revision = SHDISP_HW_TYPE_15PP;
#endif /* SHKEYLED_PORT_TYPE_0 */
#if defined(SHKEYLED_PORT_TYPE_2)
#if 0
uint	shkeyled_step  = KEYLED_STEP;
#else
uint	shkeyled_current_max = KEYLED_CURRENT_MAX;
uint	shkeyled_current_up  = KEYLED_CURRENT_UP;
#endif
#endif

#if defined(CONFIG_LEDS_KEY_KEYBOARDLED)
uint	shkeyled_keyboardled_ma = 0;
uint	shkeyled_keyboardled_full        = KEYBOARDLED_FULL;
uint	shkeyled_keyboardled_current_max = KEYBOARDLED_CURRENT_MAX;
uint	shkeyled_keyboardled_current_up	 = KEYBOARDLED_CURRENT_UP;
#endif

static void shkeyled_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	int ret = 1;
	
#if defined(SHKEYLED_PORT_TYPE_0) || defined(SHKEYLED_PORT_TYPE_1)
	if(value == LED_OFF)
	{
		shkeyled_ma = 0;
	}
	else if(value == LED_FULL)
	{
		shkeyled_ma = shkeyled_full;
	}
	else if(value == LED_UP)
	{
		if(shkeyled_ma < shkeyled_current_max)
		{
			shkeyled_ma += shkeyled_current_up;
		}
	}
	else if(value == LED_DOWN)
	{
		if(shkeyled_ma > 0)
		{
			shkeyled_ma -= shkeyled_current_up;
		}
	}
	else
	{
		shkeyled_ma = 0;
	}

	#ifdef SHKEYLED_SHTERM_ENABLE
	if(shkeyled_ma == 0)
	{
		if(shterm_k_set_info(SHTERM_INFO_CTSBACKLIGHT, 0) != SHTERM_SUCCESS)
		{
			SHKEYLED_DEBUG_LOG_1("shterm_k_set_info 0 Error");
		}
		else
		{
			SHKEYLED_DEBUG_LOG_2("shterm_k_set_info 0");
		}
	}
	else
	{
		if(shterm_k_set_info(SHTERM_INFO_CTSBACKLIGHT, 1) != SHTERM_SUCCESS)
		{
			SHKEYLED_DEBUG_LOG_1("shterm_k_set_info 1 Error");
		}
		else
		{
			SHKEYLED_DEBUG_LOG_2("shterm_k_set_info 1");
		}
	}
	#endif /* SHKEYLED_SHTERM_ENABLE */


#elif defined(SHKEYLED_PORT_TYPE_2)
#if 0
	shkeyled_ma = value / 16;
	if( (value % 16) > 0 )
	{
		shkeyled_ma += 1;
	}
	shkeyled_ma *= shkeyled_step;
	if(shkeyled_ma > shkeyled_full)
	{
		shkeyled_ma = shkeyled_full;
	}
#else
	if(value == LED_OFF)
	{
		shkeyled_ma = 0;
	}
	else if(value == LED_FULL)
	{
		shkeyled_ma = shkeyled_full;
	}
	else if(value == LED_UP)
	{
		if(shkeyled_ma < shkeyled_current_max)
		{
			shkeyled_ma += shkeyled_current_up;
		}
	}
	else if(value == LED_DOWN)
	{
		if(shkeyled_ma > 0)
		{
			shkeyled_ma -= shkeyled_current_up;
		}
	}
	else
	{
		shkeyled_ma = 0;
	}
#endif

#endif /* SHKEYLED_PORT_TYPE_0 || SHKEYLED_PORT_TYPE_1 */

	
	SHKEYLED_DEBUG_LOG_2("value = %d, mA = %d", value, shkeyled_ma);
	
#if defined(SHKEYLED_PORT_TYPE_0)
#if 0
	if((shkeyled_hw_revision == SHDISP_HW_TYPE_15PP) || 
	  (shkeyled_hw_revision == SHDISP_HW_TYPE_2PP)   || 
	  (shkeyled_hw_revision == SHDISP_HW_TYPE_25PP))
	{
		ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("pmic_low_current_led_set_current Error");
		}
	}
	else
	{
		ret = pmic_high_current_led_set_current(2,shkeyled_ma);//KBD_DRV_LED
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("pmic_high_current_led_set_current Error");
		}
	}
#else
		ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("pmic_low_current_led_set_current Error");
		}
#endif
#elif defined(SHKEYLED_PORT_TYPE_1)
	ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV0, shkeyled_ma);
	if(ret)
	{
		SHKEYLED_DEBUG_LOG_1("pmic_low_current_led_set_current Error");
	}
#elif defined(SHKEYLED_PORT_TYPE_2)
#if 0
	if( (shkeyled_hw_revision & 0x03) == 0x00 )
	{
		ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("pmic_low_current_led_set_current Error");
		}
	}
	else
	{
		ret = pmic_set_led_intensity(LED_KEYPAD, shkeyled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("pmic_set_led_intensity Error");
		}
	}
#else
		ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("pmic_low_current_led_set_current Error");
		}
#endif
	if(ret == 0)
	{
#ifdef SHKEYLED_POWER_ENABLE
		ret = sh_camled_gpio_ctrl( (shkeyled_ma == 0 ? SH_CAMLED_GPIO102_LO : SH_CAMLED_GPIO102_HI), SH_CAMLED_USE_KEYLED_DRV );
		SHKEYLED_DEBUG_LOG_2("sh_camled_gpio_ctrl ret = %d", ret);
		if( (ret ==SH_CAMLED_GPIO_HI_ERR) || (ret == SH_CAMLED_GPIO_LO_ERR) )
		{
			SHKEYLED_DEBUG_LOG_1("sh_camled_gpio_ctrl Error %d", ret);
		}
		else
		{
#endif /* SHKEYLED_POWER_ENABLE */
			#ifdef SHKEYLED_SHTERM_ENABLE
			if(shkeyled_ma == 0)
			{
				if(shterm_k_set_info(SHTERM_INFO_KEYBACKLIGHT, 0) != SHTERM_SUCCESS)
				{
					SHKEYLED_DEBUG_LOG_1("shterm_k_set_info 0 Error");
				}
				else
				{
					SHKEYLED_DEBUG_LOG_2("shterm_k_set_info 0");
				}
			}
			else
			{
				if(shterm_k_set_info(SHTERM_INFO_KEYBACKLIGHT, 1) != SHTERM_SUCCESS)
				{
					SHKEYLED_DEBUG_LOG_1("shterm_k_set_info 1 Error");
				}
				else
				{
					SHKEYLED_DEBUG_LOG_2("shterm_k_set_info 1");
				}
			}
			#endif /* SHKEYLED_SHTERM_ENABLE */
#ifdef SHKEYLED_POWER_ENABLE
		}
#endif /* SHKEYLED_POWER_ENABLE */
	}
#else

#endif
}

#ifdef CONFIG_LEDS_KEY_KEYBOARDLED
static void shkeyled_keyboardled_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	int ret = 1;

	if(value == LED_OFF)
	{
		shkeyled_keyboardled_ma = 0;
	}
	else if(value == LED_FULL)
	{
		shkeyled_keyboardled_ma = shkeyled_keyboardled_full;
	}
	else if(value == LED_UP)
	{
		if(shkeyled_ma < shkeyled_current_max)
		{
			shkeyled_keyboardled_ma += shkeyled_keyboardled_current_up;
		}
	}
	else if(value == LED_DOWN)
	{
		if(shkeyled_keyboardled_ma > 0)
		{
			shkeyled_keyboardled_ma -= shkeyled_keyboardled_current_up;
		}
	}
	else
	{
		shkeyled_keyboardled_ma = 0;
	}
#if defined(SHKEYLED_PORT_TYPE_0)
		ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_keyboardled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("shkeyboardled_set Error");
		}
#elif defined(SHKEYLED_PORT_TYPE_1)
	ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV1, shkeyled_keyboardled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("shkeyboardled_set LED1 Error");
		}
	ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_keyboardled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("shkeyboardled_set LED2 Error");
		}
#elif defined(SHKEYLED_PORT_TYPE_2)
		ret = pmic_low_current_led_set_current(LOW_CURRENT_LED_DRV2, shkeyled_keyboardled_ma);
		if(ret)
		{
			SHKEYLED_DEBUG_LOG_1("shkeyboardled_set Error");
		}
#endif
}
#endif

static struct led_classdev shkeyled_dev =
{
	.name			= "button-backlight",
	.brightness_set	= shkeyled_set,
	.brightness		= LED_OFF,
};
#ifdef CONFIG_LEDS_KEY_KEYBOARDLED
static struct led_classdev shkeyled_keyboardled_dev =
{
	.name			= "keyboard-backlight",
	.brightness_set	= shkeyled_keyboardled_set,
	.brightness		= LED_OFF,
};
#endif

static int shkeyled_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &shkeyled_dev);
	if (rc)
	{
		SHKEYLED_DEBUG_LOG_1("led_classdev_register Error");
		return rc;
	}

#if defined(SHKEYLED_PORT_TYPE_0)
#if 0
	shkeyled_hw_revision = (uint)sh_boot_get_hw_revision();
	if((shkeyled_hw_revision == SHDISP_HW_TYPE_15PP) ||
	  (shkeyled_hw_revision == SHDISP_HW_TYPE_2PP)   ||
	  (shkeyled_hw_revision == SHDISP_HW_TYPE_25PP))
	{
		shkeyled_full        = KEYLED_FULL;
		shkeyled_current_max = KEYLED_CURRENT_MAX;
		shkeyled_current_up	 = KEYLED_CURRENT_UP;
	}
	else
	{
		shkeyled_full        = KEYLED_FULL2;
		shkeyled_current_max = KEYLED_CURRENT_MAX2;
		shkeyled_current_up  = KEYLED_CURRENT_UP2;
	}
	#else
		shkeyled_full        = KEYLED_FULL;
		shkeyled_current_max = KEYLED_CURRENT_MAX;
		shkeyled_current_up	 = KEYLED_CURRENT_UP;
#endif
#elif defined(SHKEYLED_PORT_TYPE_2)
#if 0
	shkeyled_hw_revision = (uint)sh_boot_get_hw_revision();
	if( (shkeyled_hw_revision & 0x03) == 0x00 )
	{
		shkeyled_full = KEYLED_FULL2;
		shkeyled_step = KEYLED_STEP2;
	}
	else
	{
		shkeyled_full = KEYLED_FULL;
		shkeyled_step = KEYLED_STEP;
	}
#else
		shkeyled_full        = KEYLED_FULL;
		shkeyled_current_max = KEYLED_CURRENT_MAX;
		shkeyled_current_up	 = KEYLED_CURRENT_UP;
#endif
#endif

	shkeyled_set(&shkeyled_dev, LED_OFF);

#ifdef CONFIG_LEDS_KEY_KEYBOARDLED
	rc = led_classdev_register(&pdev->dev, &shkeyled_keyboardled_dev);
	if (rc)
	{
		SHKEYLED_DEBUG_LOG_1("led_classdev_register Error");
		return rc;
	}

#if defined(SHKEYLED_PORT_TYPE_0)
		shkeyled_keyboardled_full        = KEYBOARDLED_FULL;
		shkeyled_keyboardled_current_max = KEYBOARDLED_CURRENT_MAX;
		shkeyled_keyboardled_current_up	 = KEYBOARDLED_CURRENT_UP;
#elif defined(SHKEYLED_PORT_TYPE_1)
		shkeyled_keyboardled_full        = KEYBOARDLED_FULL;
		shkeyled_keyboardled_current_max = KEYBOARDLED_CURRENT_MAX;
		shkeyled_keyboardled_current_up	 = KEYBOARDLED_CURRENT_UP;
#elif defined(SHKEYLED_PORT_TYPE_2)
		shkeyled_keyboardled_full        = KEYBOARDLED_FULL;
		shkeyled_keyboardled_current_max = KEYBOARDLED_CURRENT_MAX;
		shkeyled_keyboardled_current_up	 = KEYBOARDLED_CURRENT_UP;
#endif
	shkeyled_keyboardled_set(&shkeyled_keyboardled_dev, LED_OFF);

#endif

	return rc;
}

static int __devexit shkeyled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&shkeyled_dev);
#ifdef CONFIG_LEDS_KEY_KEYBOARDLED
	led_classdev_unregister(&shkeyled_keyboardled_dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int shkeyled_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&shkeyled_dev);
#ifdef CONFIG_LEDS_KEY_KEYBOARDLED
	led_classdev_suspend(&shkeyled_keyboardled_dev);
#endif
	return 0;
}

static int shkeyled_resume(struct platform_device *dev)
{
	led_classdev_resume(&shkeyled_dev);
#ifdef CONFIG_LEDS_KEY_KEYBOARDLED
	led_classdev_resume(&shkeyled_keyboardled_dev);
#endif
	return 0;
}
#else
#define shkeyled_suspend NULL
#define shkeyled_resume NULL
#endif

static struct platform_driver shkeyled_driver = {
	.probe		= shkeyled_probe,
	.remove		= __devexit_p(shkeyled_remove),
	.suspend	= shkeyled_suspend,
	.resume		= shkeyled_resume,
	.driver		= {
		.name	= "shkeyled",
		.owner	= THIS_MODULE,
	},
};

static int __init shkeyled_init(void)
{
	return platform_driver_register(&shkeyled_driver);
}

static void __exit shkeyled_exit(void)
{
	platform_driver_unregister(&shkeyled_driver);
}

module_exit(shkeyled_exit);
module_init(shkeyled_init);

MODULE_DESCRIPTION("SHARP KEYLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
