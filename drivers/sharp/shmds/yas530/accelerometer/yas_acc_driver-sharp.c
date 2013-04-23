/* drivers/sharp/compass/accelerometer/yas_acc_driver-sharp.c
 *
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
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sharp/shpem_kerl.h>

#if SENSOR_TYPE == 1
#define SENSOR_NAME "accelerometer"
#elif SENSOR_TYPE == 2
#define SENSOR_NAME "geomagnetic"
#elif SENSOR_TYPE == 3
#define SENSOR_NAME "orientation"
#elif SENSOR_TYPE == 4
#define SENSOR_NAME "gyroscope"
#elif SENSOR_TYPE == 5
#define SENSOR_NAME "light"
#elif SENSOR_TYPE == 6
#define SENSOR_NAME "pressure"
#elif SENSOR_TYPE == 7
#define SENSOR_NAME "temperature"
#elif SENSOR_TYPE == 8
#define SENSOR_NAME "proximity"
#endif


/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH							   9806550
#define ABSMAX_2G					   (GRAVITY_EARTH * 2)
#define ABSMIN_2G					  (-GRAVITY_EARTH * 2)
#define RESOLUTION_BOSCH				256
#define RESOLUTION_STM					512

#define MAKE_UINT16(a, b)		((unsigned short)( ((((unsigned short)a) <<   8)&     0xFF00) | \
												   (( (unsigned short)b)        &     0x00FF) ) )

#define SENSOR_DEFAULT_DELAY			(200)	/* 200 ms */
#define SENSOR_MIN_DELAY				 (20)	/*	20 ms */
#define SENSOR_MAX_DELAY				(200)	/* 200 ms */

#define ABS_WAKE						(ABS_MISC)

#define HW_ES0		0x03
#define HW_ES1		0x02
#define HW_PP1		0x01
#define HW_PP15		0x04
#define HW_PP2		0x00


struct sensor_data {
	struct mutex mutex;
	int enabled;
	int delay;
	int suspend;
	int suspend_enable;
	struct delayed_work work;
};

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

/* LIS331DLH */
static const int position_map[][3][3] = {
    {{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}}, /* top/upper-left */
    {{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}}, /* top/upper-right */
    {{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}}, /* top/lower-right */
    {{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}}, /* top/lower-left */
    {{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}}, /* bottom/upper-left */
    {{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}}, /* bottom/upper-right */
    {{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}}, /* bottom/lower-right */
    {{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}}, /* bottom/lower-right */
};

/* Sysfs interface */
static ssize_t
sensor_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int delay;

	mutex_lock(&data->mutex);

	delay = data->delay;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

static ssize_t
sensor_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int value = simple_strtol(buf, NULL, 10);

	if (value < 0) {
		return count;
	}

	if(SENSOR_MAX_DELAY < value)
	{
		value = SENSOR_MAX_DELAY;
	}

	if(SENSOR_MIN_DELAY > value)
	{
		value = SENSOR_MIN_DELAY;
	}

	mutex_lock(&data->mutex);

	data->delay = value;

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
sensor_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int enabled;

	mutex_lock(&data->mutex);

	enabled = data->enabled;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
sensor_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int value = simple_strtol(buf, NULL, 10);
	ssize_t ret;

	value = !!value;

	mutex_lock(&data->mutex);

	if((data->enabled == 0) && (value == 1))
	{
		if(SHPEM_SUCCESS == shpem_set_sleep_state_kerl(SHPEM_MICON_SLEEP_ACCEL, SHPEM_MICON_WAKEUP))
		{
			if(SHPEM_SUCCESS == shpem_set_sensor_state_kerl(SHPEM_MICON_SENSOR_ACCEL, SHPEM_MICON_SENSOR_ON))
			{
				data->enabled = value;
				ret = count;
			}
			else
			{
				shpem_set_sensor_state_kerl(SHPEM_MICON_SENSOR_ACCEL, SHPEM_MICON_SENSOR_OFF);
				shpem_set_sleep_state_kerl(SHPEM_MICON_SLEEP_ACCEL, SHPEM_MICON_SLEEP);

				ret = 0;
				goto ERR;
			}
		}
		else
		{
			shpem_set_sleep_state_kerl(SHPEM_MICON_SLEEP_ACCEL, SHPEM_MICON_SLEEP);
			goto ERR;
		}

		schedule_delayed_work(&data->work, msecs_to_jiffies(data->delay));
	}
	else if((data->enabled == 1) && (value == 0))
	{
		cancel_delayed_work_sync(&data->work);

		shpem_set_sensor_state_kerl(SHPEM_MICON_SENSOR_ACCEL, SHPEM_MICON_SENSOR_OFF);
		shpem_set_sleep_state_kerl(SHPEM_MICON_SLEEP_ACCEL, SHPEM_MICON_SLEEP);

		data->enabled = value;
		data->delay = SENSOR_DEFAULT_DELAY;

		ret = count;
	}

ERR:
	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
sensor_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

#if DEBUG

static int sensor_suspend(struct platform_device *pdev, pm_message_t state);
static int sensor_resume(struct platform_device *pdev);

static ssize_t
sensor_debug_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t
sensor_debug_suspend_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long suspend = simple_strtol(buf, NULL, 10);

	if (suspend) {
		pm_message_t msg;
		memset(&msg, 0, sizeof(msg));
		sensor_suspend(sensor_pdev, msg);
	} else {
		sensor_resume(sensor_pdev);
	}

	return count;
}

#endif /* DEBUG */

static ssize_t
sensor_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x, y, z;

	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];
	y = input_data->abs[ABS_Y];
	z = input_data->abs[ABS_Z];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	return sprintf(buf, "%d %d %d\n", x, y, z);
}


static DEVICE_ATTR(delay, 0660,
		sensor_delay_show, sensor_delay_store);
static DEVICE_ATTR(enable, 0660,
		sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, 0220,
		NULL, sensor_wake_store);
static DEVICE_ATTR(data, 0444, sensor_data_show, NULL);
#if DEBUG
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
				   sensor_debug_suspend_show, sensor_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *sensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group sensor_attribute_group = {
	.attrs = sensor_attributes
};

static void sensor_work_func(struct work_struct *work)
{
	struct sensor_data	*data = input_get_drvdata(this_data);
	shpem_com_param_t	shpem_param;
	shpem_result_t		result;
	uint8_t				write_buf[4] = {'R','D','M','S'};
	uint8_t				read_buf[11];
	short				raw_data[3];
	uint8_t				i,j;
	int32_t				out_data[3] = {0,0,GRAVITY_EARTH};
	uint16_t			resolution;
	uint8_t				shift_value;

	/* STu */
	resolution = RESOLUTION_STM;
	shift_value = 4;

	if(data->enabled == 1)
	{
		shpem_param.cmd_id = SHPEM_CMD_RDMS;
		shpem_param.send_buf = &write_buf[0];
		shpem_param.recv_buf = &read_buf[0];
		result = shpem_com_kerl(&shpem_param);

		if(SHPEM_SUCCESS == result)
		{
			raw_data[0] = MAKE_UINT16(read_buf[4],read_buf[5]);
			raw_data[1] = MAKE_UINT16(read_buf[6],read_buf[7]);
			raw_data[2] = MAKE_UINT16(read_buf[8],read_buf[9]);

			for(i=0; i < 3; i++)
			{
				raw_data[i] <<= shift_value;
				raw_data[i] = raw_data[i] >> shift_value;

			}

			/* for X, Y, Z axis */
			for(i = 0; i < 3; i++)
			{
				/* coordinate transformation */
				out_data[i] = 0;
				for(j = 0; j < 3; j++)
				{
					out_data[i] += raw_data[j] * position_map[CONFIG_SH_YAS530_ACCELEROMETER_POSITION][i][j];
				}
				/* normalization */
				out_data[i] *= (GRAVITY_EARTH / resolution);
			}
		}
		else if(SHPEM_SLEEP_ERROR == result)
		{
			shpem_set_sleep_state_kerl(SHPEM_MICON_SLEEP_ACCEL, SHPEM_MICON_WAKEUP);
			shpem_set_sensor_state_kerl(SHPEM_MICON_SENSOR_ACCEL, SHPEM_MICON_SENSOR_ON);
			out_data[0] = 0;
			out_data[0] = 0;
			out_data[0] = GRAVITY_EARTH;
		}
		else
		{
			out_data[0] = 0;
			out_data[0] = 0;
			out_data[0] = GRAVITY_EARTH;
		}

		input_report_abs(this_data, ABS_X, out_data[0]);
		input_report_abs(this_data, ABS_Y, out_data[1]);
		input_report_abs(this_data, ABS_Z, out_data[2]);
		input_sync(this_data);

		schedule_delayed_work(&data->work, msecs_to_jiffies(data->delay));
	}
}

static int
sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sensor_data *data = input_get_drvdata(this_data);
	int rt = 0;

	mutex_lock(&data->mutex);

	if (data->suspend == 0) {
		data->suspend_enable = data->enabled;
		if (data->suspend_enable) {
			cancel_delayed_work_sync(&data->work);
			data->enabled = 0;
		}
	}
	data->suspend = 1;

	mutex_unlock(&data->mutex);

	return rt;
}

static int
sensor_resume(struct platform_device *pdev)
{
	struct sensor_data *data = input_get_drvdata(this_data);
	int rt = 0;
	int delay;

	mutex_lock(&data->mutex);

	if (data->suspend == 1) {
		if (data->suspend_enable) {
			delay = data->delay;
			schedule_delayed_work(&data->work, msecs_to_jiffies(delay));
			data->enabled = 1;
		}
	}
	data->suspend = 0;

	mutex_unlock(&data->mutex);

	return rt;
}

static int
sensor_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	struct input_dev *input_data = NULL;
	int input_registered = 0, sysfs_created = 0;
	int rt;

	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;
	data->suspend = 0;
	data->suspend_enable = 0;

	/* Setup driver interface */
	INIT_DELAYED_WORK(&data->work, sensor_work_func);

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		YLOGE(("sensor_probe: Failed to allocate input_data device\n"));
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);
	input_set_capability(input_data, EV_ABS, ABS_Y);
	input_set_capability(input_data, EV_ABS, ABS_Z);
	input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
	input_data->name = SENSOR_NAME;

	rt = input_register_device(input_data);
	if (rt) {
		YLOGE(("sensor_probe: Unable to register input_data device: %s\n",
			   input_data->name));
		goto err;
	}
	input_set_drvdata(input_data, data);
	input_registered = 1;

	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		YLOGE(("sensor_probe: sysfs_create_group failed[%s]\n",
			   input_data->name));
		goto err;
	}
	sysfs_created = 1;
	mutex_init(&data->mutex);
	this_data = input_data;

	return 0;

err:
	if (data != NULL) {
		if (input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&sensor_attribute_group);
			}
			if (input_registered) {
				input_unregister_device(input_data);
			}
			else {
				input_free_device(input_data);
			}
			input_data = NULL;
		}
		kfree(data);
	}

	return rt;
}

static int
sensor_remove(struct platform_device *pdev)
{
	struct sensor_data *data;

	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		sysfs_remove_group(&this_data->dev.kobj,
				&sensor_attribute_group);
		input_unregister_device(this_data);
		if (data != NULL) {
			kfree(data);
		}
	}

	return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver sensor_driver = {
	.probe		= sensor_probe,
	.remove 	= sensor_remove,
	.suspend	= sensor_suspend,
	.resume 	= sensor_resume,
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init sensor_init(void)
{
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	}
	return platform_driver_register(&sensor_driver);
}
module_init(sensor_init);

static void __exit sensor_exit(void)
{
	platform_driver_unregister(&sensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
