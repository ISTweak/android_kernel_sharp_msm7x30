/* drivers/sharp/shspamp/shspamp_ak7833.c
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
/* CONFIG_SH_AUDIO_DRIVER newly created */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sharp/shspamp.h>

/* [BatteryTemperatureLog] [start] */
#include <sharp/shterm_k.h>
/* [BatteryTemperatureLog] [end] */

#define SHSPAMP_DRIVER_NAME 		"sh_spamp"
#define SHSPAMP_I2C_DRIVER_NAME 	"sh_spamp_i2c"
#define SHSPAMP_PDN				99
#define SHSPAMP_PDN_MSG			"gpio_spamp_pdn"

#define	SHSPAMP_MUTEN_MUTE		0x00
#define	SHSPAMP_MUTEN_UNMUTE	0x01

static int shspamp_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int shspamp_i2c_remove(struct i2c_client *client);
static int shspamp_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size);
static int shspamp_i2c_write(struct i2c_client *client,
									int reg, u8 data);

static dev_t shspamp_devid;
static struct class *shspamp_class;
static struct device *shspamp_device;
struct cdev shspamp_cdev;

struct shspamp_i2c_data {
	struct i2c_client *client_p;
};
struct shspamp_i2c_data *spamp_data = NULL;

/* [BatteryTemperatureLog] [start] */
static bool shspamp_shterm_flg = false;
/* [BatteryTemperatureLog] [end] */

static	u8	svSetMuten = SHSPAMP_MUTEN_UNMUTE;

void shspamp_poweron()
{
/* [BatteryTemperatureLog] [start] */
    if (shspamp_shterm_flg == false) {
        shterm_k_set_info( SHTERM_INFO_SPEAKER, 1 );
        shspamp_shterm_flg = true;
    }
/* [BatteryTemperatureLog] [end] */

	gpio_set_value(SHSPAMP_PDN, 1);
	msleep(25);
	
	if(spamp_data){
		shspamp_i2c_write(spamp_data->client_p, 0x02, SHSPAMP_MUTEN_MUTE);
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 103)
		shspamp_i2c_write(spamp_data->client_p, 0x03, 0x0A);
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 304)
		shspamp_i2c_write(spamp_data->client_p, 0x04, 0x1E);
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 306)
		shspamp_i2c_write(spamp_data->client_p, 0x04, 0x26);
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
		shspamp_i2c_write(spamp_data->client_p, 0x04, 0x20);
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER  */
		shspamp_i2c_write(spamp_data->client_p, 0x05, 0x00);
		shspamp_i2c_write(spamp_data->client_p, 0x06, 0x80);
		shspamp_i2c_write(spamp_data->client_p, 0x02, SHSPAMP_MUTEN_UNMUTE);

		msleep(30);

		if(svSetMuten == SHSPAMP_MUTEN_MUTE)
		{
			shspamp_i2c_write(spamp_data->client_p, 0x02, svSetMuten);
			msleep(30);
		}
		svSetMuten = SHSPAMP_MUTEN_UNMUTE;
	}
}
EXPORT_SYMBOL(shspamp_poweron);

void shspamp_poweroff()
{
	svSetMuten = SHSPAMP_MUTEN_UNMUTE;

	if(spamp_data){
		shspamp_i2c_write(spamp_data->client_p, 0x02, SHSPAMP_MUTEN_MUTE);
	}
	gpio_set_value(SHSPAMP_PDN, 0);

/* [BatteryTemperatureLog] [start] */
    if (shspamp_shterm_flg == true) {
        shterm_k_set_info( SHTERM_INFO_SPEAKER, 0 );
        shspamp_shterm_flg = false;
    }
/* [BatteryTemperatureLog] [end] */
}
EXPORT_SYMBOL(shspamp_poweroff);

static int shspamp_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags= 0,
			.len  = 1,
			.buf  = buf,
		},
		{
			.addr = client->addr,
			.flags= I2C_M_RD,
			.len  = size,
			.buf  = data,
		}
	};

	buf[0] = reg;
	rc = i2c_transfer(client->adapter, msg, 2);
	if(rc != 2){
		dev_err(&client->dev,
		       "shspamp_i2c_read FAILED: read of register %d\n", reg);
		rc = -1;
		goto i2c_rd_exit;
	}
	
i2c_rd_exit:
	return rc;
}

static int shspamp_i2c_write(struct i2c_client *client,
									int reg, u8 data)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags= 0,
		.len  = 2,
		.buf  = buf,
	};
	
	buf[0] = reg;
	buf[1] = data;
	rc = i2c_transfer(client->adapter, &msg, 1);
	if(rc != 1){
		dev_err(&client->dev,
		       "shspamp_i2c_write FAILED: writing to reg %d\n", reg);
		rc = -1;
	}
	
	return rc;
}

static const struct i2c_device_id shspamp_i2c_id[] = {
	{ SHSPAMP_I2C_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver shspamp_i2c_driver = {
	.driver		= {
		.name = SHSPAMP_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= shspamp_i2c_probe,
	.remove		= __devexit_p(shspamp_i2c_remove),
	.id_table	= shspamp_i2c_id
};

static int shspamp_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rc;
	struct shspamp_i2c_data *sd;
	
	if(spamp_data){
		rc = -EPERM;
		goto probe_exit;
	}
	
	sd = (struct shspamp_i2c_data*)kzalloc(sizeof *sd, GFP_KERNEL);
	if(!sd){
		rc = -ENOMEM;
		goto probe_exit;
	}
	spamp_data = sd;
	i2c_set_clientdata(client, sd);
	sd->client_p = client;
	
	return 0;
	
probe_exit:
	return rc;
}

static int shspamp_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int __init shspamp_i2c_init(void)
{
	return i2c_add_driver(&shspamp_i2c_driver);
}

static void shspamp_i2c_exit(void)
{
	i2c_del_driver(&shspamp_i2c_driver);
}

static int shspamp_setMute(u8 val)
{
	int	retVal;
	int	gpioVal;
	u8	setVal;
	
	u8	getVal;

	if(val == 1)
	{
		setVal = SHSPAMP_MUTEN_MUTE;
	}
	else
	{
		setVal = SHSPAMP_MUTEN_UNMUTE;
	}

	if(spamp_data == NULL){
		return -EFAULT;
	}

	gpioVal = gpio_get_value(SHSPAMP_PDN);
	if(gpioVal == 0)
	{
		svSetMuten = setVal;
		return -EFAULT;
	}
	
	shspamp_i2c_read(spamp_data->client_p, 0x02, &getVal, 1);
	if(getVal != setVal)
	{
		retVal = shspamp_i2c_write(spamp_data->client_p, 0x02, setVal);

		msleep(30);
	}
	svSetMuten = setVal;

	return	retVal;
}

static int shspamp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int shspamp_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int shspamp_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int rc;
	
	switch(cmd){
	case	SPAMP_SET_MUTEN:
		rc = shspamp_setMute(arg);
		break;

	default:
		rc = -ENOIOCTLCMD;
		break;
	}
	
	return rc;
}

static const struct file_operations shspamp_fileops = {
	.owner   = THIS_MODULE,
	.open    = shspamp_open,
	.release = shspamp_release,
	.ioctl   = shspamp_ioctl,
};

int __init shspamp_init(void)
{
	int rc;

	rc = alloc_chrdev_region(&shspamp_devid, 0, 1, SHSPAMP_DRIVER_NAME);
	if(rc < 0){
		printk(KERN_WARNING "ak7833:alloc_chrdev_region error\n");
		return rc;
	}

	shspamp_class = class_create(THIS_MODULE, SHSPAMP_DRIVER_NAME);
	if (IS_ERR(shspamp_class)) {
		rc = PTR_ERR(shspamp_class);
		printk(KERN_WARNING "ak7833:class_create error\n");
		goto error_vid_class_create;
	}

	shspamp_device = device_create(shspamp_class, NULL, 
								shspamp_devid, &shspamp_cdev, 
								SHSPAMP_DRIVER_NAME);
	if (IS_ERR(shspamp_device)) {
		rc = PTR_ERR(shspamp_device);
		printk(KERN_WARNING "ak7833:device_create error\n");
		goto error_vid_class_device_create;
	}
	
	cdev_init(&shspamp_cdev, &shspamp_fileops);
	shspamp_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shspamp_cdev, shspamp_devid, 1);
	if(rc < 0){
		printk(KERN_WARNING "ak7833:cdev_add error\n");
		goto err_via_cdev_add;
	}
	
	rc = shspamp_i2c_init();
	if(rc < 0){
		printk(KERN_WARNING "ak7833:shspamp_i2c_init error\n");
		goto err_via_cdev_add;
	}
	
	rc = gpio_request(SHSPAMP_PDN, SHSPAMP_PDN_MSG);
	if(rc < 0){
		printk(KERN_WARNING "ak7833:gpio_request error\n");
		goto err_via_gpio_config;
	}
	
#if 0
	rc = gpio_tlmm_config(GPIO_CFG(SHSPAMP_PDN, 0, 
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), GPIO_ENABLE);
	if(rc < 0){
		printk(KERN_WARNING "ak7833:gpio_tlmm_config error\n");
		goto err_via_gpio_config;
	}
#endif

	return 0;
	
err_via_gpio_config:
	gpio_free(SHSPAMP_PDN);
err_via_cdev_add:
	cdev_del(&shspamp_cdev);
error_vid_class_device_create:
	class_destroy(shspamp_class);
error_vid_class_create:
	unregister_chrdev_region(shspamp_devid, 1);

	return rc;
}

static void __exit shspamp_exit(void)
{
	shspamp_i2c_exit();
	cdev_del(&shspamp_cdev);
	class_destroy(shspamp_class);
	unregister_chrdev_region(shspamp_devid, 1);
}

MODULE_LICENSE("GPL");

module_init(shspamp_init);
module_exit(shspamp_exit);
