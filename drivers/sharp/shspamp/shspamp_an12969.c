/* drivers/sharp/shspamp/shspamp_an12969.c
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
#define SHSPAMP_PDN					99
#define SHSPAMP_PDN_MSG				"gpio_spamp_pdn"

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

/* register0 access type */
enum{
	SPAMP_REG0_INIT,
	SPAMP_REG0_ENABLE,
	SPAMP_REG0_DISABLE,
	SPAMP_REG0_SPSAVE,
};

/* local function */
static bool SpAmp_Init( void );
static bool SpAmp_set_reg0(int type);
static bool SpAmp_set_reg1(void);
static bool SpAmp_set_reg2(void);



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

/*
	GAIN	"0"(+23)	AN12969
	AGC	"1"(ON)
	AGC_ON	"000"	(12.6dBv)
	AGC_REC	"011"	(3.0sec)
	AGC_ATT	"01"	(1msec)
*/
	if(spamp_data){
		/* write reg-0 */
		if(!SpAmp_set_reg0(SPAMP_REG0_ENABLE))
			return;

		/* Standby sleep */
		usleep(SPAMP_POWER_ON_MARGIN_TIME);

		/* write reg-1 */
		if(!SpAmp_set_reg1())
			return;

		/* write reg-2 */
		if(!SpAmp_set_reg2())
			return;

		/* sp save off */
		if(!SpAmp_set_reg0(SPAMP_REG0_SPSAVE))
			return;
	}
}
EXPORT_SYMBOL(shspamp_poweron);

void shspamp_poweroff()
{
	if(spamp_data){
        SpAmp_set_reg0(SPAMP_REG0_DISABLE);
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
	u8 read_buf[SPAMP_READ_BUF_SIZE]={0,0,0};
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
			.len  = SPAMP_READ_BUF_SIZE,
			.buf  = read_buf,
		}
	};

	buf[0] = reg;
	buf[1] = 0;
	rc = i2c_transfer(client->adapter, msg, 2);
	if(rc != 2){
		dev_err(&client->dev,
		       "shspamp_an12969_i2c_read FAILED: read of register %d\n", reg);
		rc = -1;
		goto i2c_rd_exit;
	}
	*data = read_buf[reg];

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
		       "shspamp_an12969_i2c_write FAILED: writing to reg %d\n", reg);
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
	
	sd = (struct shspamp_i2c_data*)kzalloc(sizeof(*sd), GFP_KERNEL);
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
		printk(KERN_WARNING "an12969:alloc_chrdev_region error\n");
		return rc;
	}

	shspamp_class = class_create(THIS_MODULE, SHSPAMP_DRIVER_NAME);
	if (IS_ERR(shspamp_class)) {
		rc = PTR_ERR(shspamp_class);
		printk(KERN_WARNING "an12969:class_create error\n");
		goto error_vid_class_create;
	}

	shspamp_device = device_create(shspamp_class, NULL, 
								shspamp_devid, &shspamp_cdev, 
								SHSPAMP_DRIVER_NAME);
	if (IS_ERR(shspamp_device)) {
		rc = PTR_ERR(shspamp_device);
		printk(KERN_WARNING "an12969:device_create error\n");
		goto error_vid_class_device_create;
	}

	cdev_init(&shspamp_cdev, &shspamp_fileops);
	shspamp_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shspamp_cdev, shspamp_devid, 1);
	if(rc < 0){
		printk(KERN_WARNING "an12969:cdev_add error\n");
		goto err_via_cdev_add;
	}

	rc = shspamp_i2c_init();
	if(rc < 0){
		printk(KERN_WARNING "an12969:shspamp_an12969_i2c_init error\n");
		goto err_via_cdev_add;
	}

	rc = gpio_request(SHSPAMP_PDN, SHSPAMP_PDN_MSG);
	if(rc < 0){
		printk(KERN_WARNING "an12969:gpio_request error\n");
		goto err_via_gpio_config;
	}
	
	if(spamp_data){
		SpAmp_Init();
	}
	else
	{
		rc = -1;
		goto err_via_gpio_config;
	}
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

static bool SpAmp_Init( void )
{
	bool	ret	= false;

	gpio_set_value(SHSPAMP_PDN, 1);
	/* init reg-0 */
	if(!SpAmp_set_reg0(SPAMP_REG0_INIT))
		goto err_spamp_reg_init;

	/* init reg-1 */
	if(!SpAmp_set_reg1())
		goto err_spamp_reg_init;

	/* init reg-2 */
	if(!SpAmp_set_reg2())
		goto err_spamp_reg_init;

	ret	= true;

err_spamp_reg_init:
	gpio_set_value(SHSPAMP_PDN, 0);

	return ret;
}
static bool SpAmp_set_reg0( int type )
{
	char	buf;
    int		i2c_ret = 0;

    i2c_ret = shspamp_i2c_read(spamp_data->client_p, SPAMP_REG_00, &buf, 1);
	if(i2c_ret == -1){
		pr_err("SpAmp_set_reg0: I2C READ ERROR! \n" );
	    return false;
	}

	/***** Register 0x00 *****/
	switch(type){
	case SPAMP_REG0_INIT:
	case SPAMP_REG0_ENABLE:
		/* D7: GAIN +23[dB] */
		buf = (buf & ~SPAMP_GAIN_MASK) | SPAMP_GAIN_26DB;

		/* D6-4: 0 */
		buf = (buf & ~SPAMP_FIX_0HEX_MASK) | SPAMP_FIX_0HEX;
		/* D3: AGC ON */
		buf = (buf & ~SPAMP_AGC_MASK) | SPAMP_AGC_ON;

		/* D2: SP Save */
		buf = (buf & ~SPAMP_SPSAVE_MASK) | SPAMP_SPSAVE_ON;

		/* D1: Standby */
		if(type == SPAMP_REG0_INIT)
			buf = (buf & ~SPAMP_STANDBY_MASK) | SPAMP_STANDBY_ON;
		else
			buf = (buf & ~SPAMP_STANDBY_MASK) | SPAMP_STANDBY_OFF;

		/* D0: Init Condition(0) */
		buf = (buf & ~SPAMP_INIT_CONDITION_MASK) | SPAMP_INIT_CONDITION;
		break;
	case SPAMP_REG0_DISABLE:
		/* D2: SP Save */
		buf = (buf & ~SPAMP_SPSAVE_MASK) | SPAMP_SPSAVE_ON;
		/* D1: Standby ON */
		buf = (buf & ~SPAMP_STANDBY_MASK) | SPAMP_STANDBY_ON;
		break;
	case SPAMP_REG0_SPSAVE:
		/* D2: SP Save */
		buf = (buf & ~SPAMP_SPSAVE_MASK) | SPAMP_SPSAVE_OFF;
		break;
	default:
		pr_err("SpAmp_set_reg0: PARAM ERROR! type=0x%2x \n", type );
		return false;	/* exit */
	}

	/* I2C write */
	i2c_ret = shspamp_i2c_write(spamp_data->client_p, SPAMP_REG_00, buf );

	if(i2c_ret == -1){
		pr_err("SpAmp_set_reg0: I2C WRITE ERROR! val=0x%2x \n", buf );
		return false;
	}

	return true;
}

static bool SpAmp_set_reg1( void )
{
	char	buf;
	int		i2c_ret = 0;

    i2c_ret = shspamp_i2c_read(spamp_data->client_p, SPAMP_REG_01, &buf, 1);
	if(i2c_ret == -1){
		pr_err("SpAmp_set_reg1: I2C READ ERROR! \n" );
		return false;
	}
	/***** Register 0x01(AGC) *****/
	/* D7-5: AGC On Level(13.9dBv) */
	buf = (buf & ~SPAMP_AGC_ON_MASK) | SPAMP_AGC_ON_13_9DBV;

	/* D4-2: AGC Recovery Time(3.0s) */
	buf = (buf & ~SPAMP_AGC_REC_MASK) | SPAMP_AGC_REC_3_0S;

	/* D1-0: AGC Attack Time(1.0ms) */
	buf = (buf & ~SPAMP_ATT_MASK) | SPAMP_ATT_1_0MS;

	/* I2C write */
	i2c_ret = shspamp_i2c_write(spamp_data->client_p, SPAMP_REG_01, buf );

	if(i2c_ret == -1){
		pr_err("SpAmp_set_reg1: I2C WRITE ERROR! val=0x%2x \n", buf );
		return false;
	}

	return true;
}

static bool SpAmp_set_reg2( void )
{
	char	buf;
	int		i2c_ret = 0;

    i2c_ret = shspamp_i2c_read(spamp_data->client_p, SPAMP_REG_02, &buf, 1);
	if(i2c_ret == -1){
		pr_err("SpAmp_set_reg2: I2C READ ERROR! \n" );
		return false;
	}
	/***** Register 0x02 *****/

	/* D7-5, D2-0: 0 */
	buf = (buf & ~SPAMP_FIX_2HEX_MASK) | SPAMP_FIX_2HEX;

	/* I2C write */
	i2c_ret = shspamp_i2c_write(spamp_data->client_p, SPAMP_REG_02, buf );
	if(i2c_ret == -1){
		pr_err("SpAmp_set_reg2: I2C WRITE ERROR! val=0x%2x \n", buf );
		return false;
	}

	return true;
}

MODULE_LICENSE("GPL");

module_init(shspamp_init);
module_exit(shspamp_exit);
