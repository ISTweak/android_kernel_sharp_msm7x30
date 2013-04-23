/*
  *Copyright (C) 2010 SHARP CORPORATION All rights reserved.
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

/*
 * SHARP PROXIMITY DRIVER(Y2659)
*/

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <sharp/proximity.h>
#include <sharp/sh_boot_manager.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#define DEBUG 0 
#define PROX_DEBUG_MSG			0
#define PROX_DEBUG_FUNC			0

#define HW_ES0		0x03
#define HW_ES1		0x02
#define HW_PP1		0x01
#define HW_PP15		0x04
#define HW_PP2		0x00
#define HW_PMP		0x05

#if PROX_DEBUG_FUNC
#define FUNC_LOG() printk(KERN_DEBUG "[PROXIMITY] %s is called\n", __func__)
#else
#define FUNC_LOG()
#endif

#if PROX_DEBUG_MSG
#define DEBUG_LOG(format, ...) printk(KERN_DEBUG "[PROXIMITY] " format "\n", ## __VA_ARGS__)
#else
#define DEBUG_LOG(format, ...)
#endif


/*+-------------------------------------------------------------------------+*/
/*|																			|*/
/*+-------------------------------------------------------------------------+*/
typedef struct drv_data_tag     drv_data;
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;
typedef struct work_struct      WorkStruct;
typedef struct input_dev        InputDev;
typedef struct device           Device;

#define	I2C_RETRY			3

struct drv_data_tag
{
	int			irq_gpio;
	InputDev	*input_dev;
	WorkStruct	IrqWork;
};

typedef struct
{
	uint8_t	mbRegAdr;					/* Register */
	uint8_t mbData;						/* Data */
} I2cWriteData;


static I2cClt		*this_client;
static char			MVO = 0;
static atomic_t		open_flag = ATOMIC_INIT(0);
static atomic_t		sensor_data = ATOMIC_INIT(7);	/* Init = Far */
static atomic_t		enable_mode = ATOMIC_INIT(0);	/* 0=Disable,1=Enable */
static const short CYCLE_DATA[8][2] =	{	{0x04,    8},	/*    8ms */
											{0x0C,   16},	/*   16ms */
											{0x14,   32},	/*   32ms */
											{0x1C,   64},	/*   64ms */
											{0x24,  128},	/*  128ms */
											{0x2C,  256},	/*  256ms */
											{0x34,  512},	/*  512ms */
											{0x3C, 1024},	/* 1024ms */
										};

static struct vreg *vreg_gp7;
#ifdef CONFIG_PROXIMITY_WAKESW
static struct wake_lock prox_wake_lock;
#endif
static int PROX_Probe(I2cClt *client, const I2cDevID *poDevId);


//static uint16_t sh_get_hw_revision(void)
//{
//	sharp_smem_common_type *p_sharp_smem_common_type;
//
//	p_sharp_smem_common_type = sh_smem_get_common_address();
//	if( p_sharp_smem_common_type != 0 )
//	{
//		return p_sharp_smem_common_type->sh_hw_revision;
//	}else{
//		return 0xFF;
//	}
//}


/*+-------------------------------------------------------------------------+*/
/*|	I2C Read																|*/
/*+-------------------------------------------------------------------------+*/
static int PROX_I2cRead(I2cClt *client, char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY; loop_i++) {
		if (i2c_transfer(client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= I2C_RETRY) {
		printk(KERN_ERR "I2cRead: error\n");

		return -EIO;
	}

	return 0;
}

/*+-------------------------------------------------------------------------+*/
/*|	I2C Write																|*/
/*+-------------------------------------------------------------------------+*/
static int PROX_I2cWrite(I2cClt *client, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= client->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
		};

	FUNC_LOG();

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(client->adapter, oMsgs, 1);
		if(nResult == 1)
		{
            DEBUG_LOG("I2cWrite success(%02X,reg:%02X,Data:%02X)=%d", client->addr, bRegAdr, bData, nResult);
			return 0;
		}
	}

	printk(KERN_ERR "I2cWrite: error\n");
	return -1;
}

static int IOECS_Enable(void)
{
	drv_data *data = NULL;
	uint8_t bData;
		int nResult = 0;

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
		data = i2c_get_clientdata(this_client);

		/* LED Output Enable = ON */
//	    gpio_direction_output(data->mnEnablePin, 1);
		if( CONFIG_PROXIMITY_POWER_SUPPLY_SETTING == 1 ){
			vreg_enable(vreg_gp7);
		}

		/*  Proximity Report Data Initialize */
		data->input_dev->abs[ABS_DISTANCE] = 99;

		/* Master Detection value Initialize */
		MVO = 0;

		/* Init = Far */
		atomic_set(&sensor_data, 7);

		/* VOUT = High(CON = H'18) */
		bData = 0x18;
		nResult = PROX_I2cWrite(this_client, Y2659_REG_CON, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}

		/* HYS = H'20 */
		bData = 0x20;
		nResult = PROX_I2cWrite(this_client, Y2659_REG_HYS, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}

		/* OPMOD = H'03 */
		bData = 0x03;
		nResult = PROX_I2cWrite(this_client, Y2659_REG_OPMOD, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}

		enable_irq_wake(this_client->irq);
		enable_irq(this_client->irq);

		/* VOUT = Low(CON = H'00) */
		bData = 0x00;
		nResult = PROX_I2cWrite(this_client, Y2659_REG_CON, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}
	}

	atomic_set(&enable_mode, 1);
	return 0;
}

static int IOECS_Disable(void)
{
	drv_data *data = NULL;
	uint8_t bData;
	int nResult = 0;

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 1)
	{
		data = i2c_get_clientdata(this_client);

		disable_irq_wake(this_client->irq);
		disable_irq(this_client->irq);

		/* LED Output Enable = Off */
//	    gpio_direction_output(data->mnEnablePin, 0);
		if( CONFIG_PROXIMITY_POWER_SUPPLY_SETTING ==1 ){
			vreg_disable(vreg_gp7);
		}

		/* OPMOD = H'02 */
		bData = 0x02;
		nResult = PROX_I2cWrite(this_client, Y2659_REG_OPMOD, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}
	}

	atomic_set(&enable_mode, 0);
	return 0;
}

static int IOECS_SetCycle(short cycle_data)
{
	uint8_t bData;
	int nResult = 0;
	uint8_t loop;
	short cmp_data;

	FUNC_LOG();

	for(loop=0; loop<7; loop++)
	{
		cmp_data = CYCLE_DATA[loop][1] + ((CYCLE_DATA[loop+1][1] - CYCLE_DATA[loop][1]) / 2);

		DEBUG_LOG("cycle_data : %d,cmp_data : %d",cycle_data,cmp_data);

		if(cycle_data <= cmp_data)
		{
			break;
		}
	}

	DEBUG_LOG("CYCLE_DATA : %d",CYCLE_DATA[loop][1]);

	/* CYCLE Setting */
	bData = CYCLE_DATA[loop][0];
	nResult = PROX_I2cWrite(this_client, Y2659_REG_CYCLE, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	return 0;
}

static int IOECS_GetVO_DATA(uint8_t *cycle_data)
{
	uint8_t buffer[2] = {0};

	FUNC_LOG();

	PROX_I2cRead(this_client, buffer,2);

	*cycle_data = (buffer[1] & 0x01);

	return 0;
}

static int IOECS_Initialize( void )
{
//	int nResult = 0;
//
	FUNC_LOG();

	/* PROX Initialize */
//	nResult = PROX_Initialize();
//	if (nResult < 0) {
//		DEBUG_LOG("PROXIMITY initialize failed.");
//	}

	return 0;
}

static int IOECS_POWERMODE( void )
{
	FUNC_LOG();
	
	if(CONFIG_PROXIMITY_POWER_SUPPLY_SETTING == 0){
		DEBUG_LOG("PROXIMITY_POWER_SUPPLY_TYPE_ALL");
		return 0;
	}

	DEBUG_LOG("PROXIMITY_POWER_SUPPLY_TYPE1");
	return -EIO;

}



static int PROX_open(struct inode *inode, struct file *filp)
{
	int ret = -1;

	FUNC_LOG();

	if (atomic_cmpxchg(&open_flag, 0, 1) == 0)
	{
		/* Init = Far */
		atomic_set(&sensor_data, 7);
		ret = 0;
	}

	//#ifdef CONFIG_PROXIMITY_WAKESW
	//wake_lock_init(&prox_wake_lock, WAKE_LOCK_SUSPEND, "proximity_wake_lock");
	//#endif


	return ret;
}

static int PROX_release(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

	IOECS_Disable();

	atomic_set(&open_flag, 0);

	#ifdef CONFIG_PROXIMITY_WAKESW
	wake_unlock(&prox_wake_lock);
	//wake_lock_destroy(&prox_wake_lock);
	#endif

	return 0;
}

static int PROX_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *ppos)

{
	char tmp = (char)atomic_read(&sensor_data);

	if (copy_to_user(buf, &tmp, sizeof(tmp))) {
		return -EFAULT;
	}

	return 0;
}

static int PROX_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	uint8_t bData;
	short cycle;

	FUNC_LOG();

	switch (cmd) {
		case ECS_IOCTL_SET_CYCLE:
			DEBUG_LOG("ECS_IOCTL_SET_CYCLE");

			if (copy_from_user(&cycle, argp, sizeof(cycle))) {
				DEBUG_LOG("ECS_IOCTL_SET_CYCLE ERR");
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	switch (cmd) {
		case ECS_IOCTL_ENABLE:
			DEBUG_LOG("ECS_IOCTL_ENABLE");
			if(IOECS_Enable() < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_DISABLE:
			DEBUG_LOG("ECS_IOCTL_DISABLE");
			if(IOECS_Disable() < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_SET_CYCLE:
			DEBUG_LOG("ECS_IOCTL_SET_CYCLE");
			if(IOECS_SetCycle(cycle) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_GET_VO_DATA:
			DEBUG_LOG("ECS_IOCTL_GET_VO_DATA");
			if(IOECS_GetVO_DATA(&bData) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_INITIALIZE:
			DEBUG_LOG("ECS_IOCTL_INITIALIZE");
			if(IOECS_Initialize() < 0)
			{
				return -EIO;
			}
			break;	
		case ECS_IOCTL_POWERMODE:
			DEBUG_LOG("ECS_IOCTL_POWERMODE");
			if(IOECS_POWERMODE() < 0)
			{
				return -EIO;
			}
			break;
		default:
			break;
	}

	switch (cmd) {
		case ECS_IOCTL_GET_VO_DATA:
			if (copy_to_user(argp, &bData, sizeof(bData))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	return 0;
}


static struct file_operations PROX_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= PROX_open,
	.release	= PROX_release,
	.ioctl		= PROX_ioctl,
	.read		= PROX_read,
};

static struct miscdevice PROX_device = {
 .minor = MISC_DYNAMIC_MINOR,
	.name = "proximity_dev",
	.fops = &PROX_ctl_fops,
};

static irqreturn_t PROX_interrupt(int irq, void *dev_id)
{
	drv_data *poProximityRec = dev_id;

	FUNC_LOG();

	disable_irq_wake(this_client->irq);
	disable_irq_nosync(this_client->irq);
	schedule_work(&poProximityRec->IrqWork);

	if(MVO == 1)
	{
		MVO = 0;
	}
	else
	{
		MVO = 1;
	}

	return IRQ_HANDLED;
}

static void PROX_Irq_workfunc(WorkStruct *work)
{
	uint8_t buffer[5] = {0};
	uint8_t bData;
	drv_data *data = i2c_get_clientdata(this_client);
	unsigned short rev;

	FUNC_LOG();

	rev = sh_boot_get_hw_revision();
	rev = rev & 0x07;

	/* default */
	if(MVO == 1)
	{
		/* HYS = H'00 */
		bData = 0x00;
	}
	else
	{
		/* HYS = H'20 */
		bData = 0x20;
	}
	
	switch (CONFIG_PROXIMITY_SENSITIVITY_SETTING){
		case 0:
			if(MVO == 1)
			{
				/* HYS = H'00 */
				bData = 0x00;
			}
			else
			{
				/* HYS = H'20 */
				bData = 0x20;
			}
			break;
		case 1:
			if(MVO == 1)
			{
				/* HYS = H'20 */
				bData = 0x20;
			}
			else
			{
				/* HYS = H'40 */
				bData = 0x40;
			}
			break;
		case 2:
			switch(rev){
				case HW_ES1:
					if(MVO == 1)
					{
						/* HYS = H'20 */
						bData = 0x20;
					}
					else
					{
						/* HYS = H'40 */
						bData = 0x40;
					}
					break;
				case HW_ES0:
				case HW_PP1:
				case HW_PP15:
				case HW_PP2:
				case HW_PMP:
					if(MVO == 1)
					{
						/* HYS = H'00 */
						bData = 0x00;
					}
					else
					{
						/* HYS = H'20 */
						bData = 0x20;
					}
					break;
			}
	}
	
	PROX_I2cWrite(this_client, Y2659_REG_HYS, bData);

	/* VOUT = High(CON = H'18) */
	bData = 0x18;
	PROX_I2cWrite(this_client, Y2659_REG_CON, bData);

	PROX_I2cRead(this_client, buffer,2);
	DEBUG_LOG("%02X:%02X",buffer[0],buffer[1]);

	/* Err Check */
	if(MVO != (buffer[1] & 0x01))
	{
		DEBUG_LOG("Dephasing");

		/* OPMOD = H'02 */
		bData = 0x02;
		PROX_I2cWrite(this_client, Y2659_REG_OPMOD, bData);

		/* HYS = H'20 */
		bData = 0x20;
		PROX_I2cWrite(this_client, Y2659_REG_HYS, bData);

		/* OPMOD = H'03 */
		bData = 0x03;
		PROX_I2cWrite(this_client, Y2659_REG_OPMOD, bData);

		/* MVO mode reset */
		MVO = 0;
	}
	else
	{
		/* Change(near = 0,far = 7) */
		if((buffer[1] & 0x01) == 0)
		{
			atomic_set(&sensor_data, 7);
		}
		else
		{
			atomic_set(&sensor_data, 0);
		}
		#ifdef CONFIG_PROXIMITY_WAKESW
		wake_lock_timeout(&prox_wake_lock, 1 * HZ);
		#endif
		input_report_abs(data->input_dev, ABS_DISTANCE, atomic_read(&sensor_data));
		input_sync(data->input_dev);
	}
	enable_irq_wake(this_client->irq);
	enable_irq(this_client->irq);

	/* VOUT = Low(CON = H'00) */
	bData = 0x00;
	PROX_I2cWrite(this_client, Y2659_REG_CON, bData);
}

static int PROX_ReleaseGPIO(drv_data *poProximityRec)
{
	FUNC_LOG();

	gpio_free(poProximityRec->irq_gpio);

	return 0;
}

static int PROX_ConfigGPIO(drv_data *poProximityRec)
{
	unsigned short rev;

	FUNC_LOG();

	rev = sh_boot_get_hw_revision();
	rev = rev & 0x07;

	DEBUG_LOG("HW_revision:%02Xh",rev);

	/* default */
	poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_NEW;

	switch (CONFIG_PROXIMITY_IRQ_SETTING){
		case 0:
			poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_NEW;
			break;
		case 1:
			switch(rev)
			{
			case HW_ES0:
			case HW_ES1:
				poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_OLD;
				break;
			case HW_PP1:
			case HW_PP15:
			case HW_PP2:
				poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_NEW;
				break;
			}
			break;
		case 2:
			switch(rev)
			{
			case HW_ES0:
			case HW_ES1:
			case HW_PP1:
			case HW_PP15:
				poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_OLD;
				break;
			case HW_PP2:
			case HW_PMP:
				poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_NEW;
				break;
			}
			break;
		case 3:
			if((rev & 0x01) == 0){
				poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_OLD;
			}
			if((rev & 0x01) == 1){
				poProximityRec->irq_gpio = SH_PROXIMITY_IRQ_NEW;
			}
			break;
	}


	gpio_request(poProximityRec->irq_gpio, "gpio_proximity_irq");
	//gpio_tlmm_config(GPIO_CFG(poProximityRec->irq_gpio,   0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	this_client->irq = MSM_GPIO_TO_INT(poProximityRec->irq_gpio);

	return 0;
}

static int PROX_Initialize(drv_data *poProximityRec, I2cClt *client)
{
	int nResult = 0;
	uint8_t bData;
	uint8_t buffer[2] = {0};

	FUNC_LOG();

	/* IRQ clear(DummyRead) */
	nResult = PROX_I2cRead(client, buffer,2);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cRead-->Error");
		return -EIO;
	}

	/* LED Output Enable = Off */
//    gpio_direction_output(poProximityRec->mnEnablePin, 0);

	/* GAIN = H'08 */
	bData = 0x08;
	nResult = PROX_I2cWrite(client, Y2659_REG_GAIN, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	/* HYS = H'20 */
	bData = 0x20;
	nResult = PROX_I2cWrite(client, Y2659_REG_HYS, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	/* CYCLE = H'3C */
	bData = 0x3C;
	nResult = PROX_I2cWrite(client, Y2659_REG_CYCLE, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	/* OPMOD = H'02 */
	bData = 0x02;
	nResult = PROX_I2cWrite(client, Y2659_REG_OPMOD, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	return 0;
}

static int PROX_Remove(I2cClt *client)
{
	drv_data *poProximityRec = i2c_get_clientdata(client);

	FUNC_LOG();

	DEBUG_LOG("Remove()");

	dev_info(&client->dev, "removing driver\n");
	device_init_wakeup(&client->dev, 0);
	PROX_ReleaseGPIO(poProximityRec);
	kfree(poProximityRec);
	return 0;
}

static const I2cDevID gI2cDevIdTableAcc[] =
{
   { SH_PROXIMITY_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);

static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_PROXIMITY_I2C_DEVNAME,
	},
	.probe	  = PROX_Probe,
	.remove	  = __devexit_p(PROX_Remove),
	.id_table = gI2cDevIdTableAcc,
};

static int PROX_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	drv_data *poProximityRec = NULL;
	int nResult;

	FUNC_LOG();

	/* CLIENT CHECKING */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_LOG("check_functionality failed.");
		nResult = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	poProximityRec = kzalloc(sizeof(drv_data), GFP_KERNEL);
	if (!poProximityRec) {
		DEBUG_LOG("memory allocation failed.");
		nResult = -ENOMEM;
		goto exit1;
	}

	INIT_WORK(&poProximityRec->IrqWork, PROX_Irq_workfunc);
	i2c_set_clientdata(client, poProximityRec);

	/* Copy to global variable */
	this_client = client;

	/* GPIO setting */
	nResult = PROX_ConfigGPIO(poProximityRec);
	if(nResult < 0)
	{
		DEBUG_LOG("ConfigGPIO is Err.");
		goto exit2;
	}

	/* PROX Initialize */
	nResult = PROX_Initialize(poProximityRec,this_client);
	if (nResult < 0) {
		DEBUG_LOG("initialize failed.");
		goto exit3;
	}

	nResult = misc_register(&PROX_device);
	if (nResult)
	{
		DEBUG_LOG("misc_register failed.");
		goto exit4;
	}

	/* wakeup set */
	set_irq_type(client->irq, IRQF_TRIGGER_LOW);

	/* IRQ */
	nResult = request_irq(client->irq, PROX_interrupt, IRQF_TRIGGER_LOW | IRQF_DISABLED,
					  "PROX_VOUT", poProximityRec);
	if (nResult < 0) {
		DEBUG_LOG("request irq failed.");
		goto exit5;
	}

	disable_irq(this_client->irq);


	/* Declare input device */
	poProximityRec->input_dev = input_allocate_device();
	if (!poProximityRec->input_dev) {
		nResult = -ENOMEM;
		DEBUG_LOG("Failed to allocate input device.");
		goto exit5;
	}
	/* Setup input device */
	set_bit(EV_ABS, poProximityRec->input_dev->evbit);

	/* proximity value near=7, far=0 */
	input_set_abs_params(poProximityRec->input_dev, ABS_DISTANCE, 0, 7, 0, 0);

	/* Set name */
	poProximityRec->input_dev->name = "proximity";

	/* Register */
	nResult = input_register_device(poProximityRec->input_dev);
	if (nResult) {
		DEBUG_LOG("Unable to register input device.");
		goto exit6;
	}
	
	if( CONFIG_PROXIMITY_POWER_SUPPLY_SETTING == 1){
		vreg_gp7 = vreg_get(NULL, "gp7");
		vreg_set_level(vreg_gp7, 2850);
	}

	return 0;

exit6:
	input_free_device(poProximityRec->input_dev);
exit5:
	misc_deregister(&PROX_device);
exit4:
exit3:
	PROX_ReleaseGPIO(poProximityRec);
exit2:
	kfree(poProximityRec);
exit1:
exit0:
	return nResult;
}

static int __init PROX_Init(void)
{
	FUNC_LOG();

	#ifdef CONFIG_PROXIMITY_WAKESW
	wake_lock_init(&prox_wake_lock, WAKE_LOCK_SUSPEND, "proximity_wake_lock");
	#endif

	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);
}

static void __exit PROX_Exit(void)
{
	FUNC_LOG();

	#ifdef CONFIG_PROXIMITY_WAKESW
	wake_unlock(&prox_wake_lock);
	wake_lock_destroy(&prox_wake_lock);
	#endif

	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
}


module_init(PROX_Init);
module_exit(PROX_Exit);


MODULE_DESCRIPTION("proximity sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
