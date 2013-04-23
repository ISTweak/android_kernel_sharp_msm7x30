/**************************************************************************************************/
/** 
	@file		TunerDev.c
	@brief		Tuner Device Control
*/
/**************************************************************************************************/

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/smp_lock.h>
#include <mach/vreg.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <asm/uaccess.h> /* for access_ok() */

#include "gpio_def.h"

#if 0
extern int gpio_direction_output(unsigned gpio, int value);
extern int gpio_direction_input(unsigned gpio);
extern int gpio_get_value(unsigned gpio);
extern void gpio_set_value(unsigned gpio, int value);
#endif

static int gpio_init(void);
static int gpio_get(unsigned int no, int *val);
static int gpio_set(unsigned int no, int val);
static int tuner_vreg_enable(void);
static int tuner_vreg_disable(void);
static int tuner_clk_enable(void);
static int tuner_clk_disable(void);

static struct clk *gp_clk;

#if 0
stGPIO_DEF use_gpiono[] = {
	/* GPIO No				, Direction		, Initialized Value	, Initialized Flag	*/
	{GPIO_PWRDWN_PORTNO		, DirctionOut	, 1					, 0					},
	{GPIO_GTDION_PORTNO		, DirctionOut	, 0					, 0					},
	{GPIO_LDO_PORTNO		, DirctionOut	, 0					, 0					}
};
#else
stGPIO_DEF use_gpiono[USE_GPIO_MAX] = {
	/* GPIO No				, Direction		, Initialized Value	, Initialized Flag	*/
	{GPIO_DTVEN_PORTNO		, DirctionOut	, 0					, 0					},
	{GPIO_DTVRST_PORTNO		, DirctionOut	, 0					, 0					},
	{GPIO_DTVLNAEN_PORTNO	, DirctionOut	, 0					, 0					}
};
#endif

/**************************************************************************************************/
/**
	@brief	tuner_open
	@param	struct inode	*inode		[I]
	@param	struct file		*file		[I]
	@retval	0	Success
	@retval	-1	Failed
*/
/**************************************************************************************************/
static int tuner_open(struct inode *inode, struct file *file)
{
	int ret;

	ret  = gpio_init();
	if (ret == 1) {
		/* Failed */
		printk("%s:%d !!!tuner_open gpio_init() error \n", __FILE__, __LINE__);
		return (-1);
	}
	return (0);
}

/**************************************************************************************************/
/**
	@brief	tuner_release
	@param	struct inode	*inode		[I]
	@param	struct file		*file		[I]
	@retval	0	Success
	@retval	-1	Failed
*/
/**************************************************************************************************/
static int tuner_release(struct inode *inode, struct file *file)
{
	return (0);
}

/**************************************************************************************************/
/**
	@brief	tuner_release
	@param	struct file		*file	[I]
	@param	unsigned int	cmd		[I]
	@param	unsigned long	arg		[I]
	@retval	0		Suncess
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static long tuner_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	ioctl_cmd io_cmd_work;
	ioctl_cmd *io_cmd = &io_cmd_work;

	if (copy_from_user(io_cmd, (ioctl_cmd*)arg, sizeof(ioctl_cmd))) {
		printk("%s:%d  !!!tuner_ioctl copy_from_user error %08x\n", __FILE__, __LINE__, (unsigned int)arg);
		return -EFAULT;
	}

	switch ( cmd ) {
	case IOC_GPIO_VAL_SET:
		ret = gpio_set(io_cmd->no, io_cmd->val);
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[cmd %x val %d][ret:%d]\n", __FILE__, __LINE__, io_cmd->no, io_cmd->val, ret);
			return -EINVAL;
		}
		break;
	case IOC_GPIO_VAL_GET:
		ret = gpio_get(io_cmd->no, &(io_cmd->val));
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl get error[cmd %x val %d][ret:%d]\n", __FILE__, __LINE__, io_cmd->no, io_cmd->val, ret);
			return -EINVAL;
		}
		if (copy_to_user((ioctl_cmd*)arg, io_cmd, sizeof(ioctl_cmd))) {
			printk("%s:%d  !!!tuner_ioctl copy_to_user error %08x\n", __FILE__, __LINE__, (unsigned int)arg);
			return -EFAULT;
		}
		break;
	case IOC_VREG_ENABLE:
		ret = tuner_vreg_enable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_VREG_DISABLE:
		ret = tuner_vreg_disable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_CLK_ENABLE:
		ret = tuner_clk_enable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_CLK_DISABLE:
		ret = tuner_clk_disable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}

static struct file_operations tuner_fops = {
	.owner				= THIS_MODULE,
	.unlocked_ioctl		= tuner_ioctl,
	.open				= tuner_open,
	.release			= tuner_release,
};

static struct miscdevice tuner_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tunctrl",
	.fops = &tuner_fops,
};

/**************************************************************************************************/
/**
	@brief	tuner_init
	@param	none
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int __init tuner_init(void)
{
	int ret;

	ret = misc_register(&tuner_dev);
	if (ret) {
		printk("%s.%s.%d !!! fail to misc_register (MISC_DYNAMIC_MINOR)\n", __FILE__, __func__, __LINE__);
		return ret;
	}
	
	ret = gpio_request(GPIO_DTVEN_PORTNO , "gpio_dtv_en");
	if(ret < 0){
		printk(KERN_DEBUG "gpio_dtv_en gpio_request() error : %d\n", ret);
	}
	
	ret = gpio_request(GPIO_DTVRST_PORTNO , "gpio_dtv_reset");
	if(ret < 0){
		printk(KERN_DEBUG "gpio_dtv_reset gpio_request() error : %d\n", ret);
	}

	ret = gpio_request(GPIO_DTVLNAEN_PORTNO , "gpio_dtv_lnaen");
	if(ret < 0){
		printk(KERN_DEBUG "gpio_dtv_lna_en gpio_request() error : %d\n", ret);
	}
	
	return 0;
}

/**************************************************************************************************/
/**
	@brief	tuner_cleanup
	@param	none
	@retval	none
*/
/**************************************************************************************************/
static void __exit tuner_cleanup(void)
{
	gpio_free(GPIO_DTVEN_PORTNO);
	gpio_free(GPIO_DTVRST_PORTNO);
	gpio_free(GPIO_DTVLNAEN_PORTNO);

	misc_deregister(&tuner_dev);
}

/**************************************************************************************************/
/**
	@brief	gpio_init
	@param	none
	@retval	0	Success
	@retval	1	Failed
*/
/**************************************************************************************************/
static int gpio_init(void)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	int errcnt = 0;
	stGPIO_DEF *p = &use_gpiono[0];
	int i;
	
	for (i=0; i<loop; i++, p++) {
		if (p->direction == DirctionIn) {
			/* GPIO Input */
			if (gpio_direction_input(p->no) < 0) {
				/* Failed */
				errcnt ++;
				printk( "%s:%d gpio_direction_input error NO.%d \n", __FILE__,__LINE__, p->no);
				continue;
			}
			p->init_done = 1;
			continue;
		}
		if (p->direction == DirctionOut) {
			/* GPIO Output */
			if (gpio_direction_output(p->no, p->out_val) < 0) {
				/* Failed */
				errcnt ++;
				printk("%s: gpio_direction_output error NO.%d \n", __FILE__, p->no);
				continue;
			}
			p->init_done = 1;
			continue;
		}
	}

	if (errcnt != 0) {
		printk("%s: gpio_init error count %d\n", __FILE__, errcnt);
		return 1;
	}
	return 0;
}

static int tuner_clk_enable(void)
{
	unsigned 	gpio98_cfg;
	int			ret;

	gpio98_cfg = GPIO_CFG( 98, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA );

	ret = gpio_tlmm_config( gpio98_cfg, GPIO_CFG_ENABLE );
	if ( ret < 0 ) {
		printk("%s: gpio98_cfg error %d\n", __FILE__, ret);
		return 1;
	}

	gp_clk = clk_get( NULL, "gp_clk" );
	clk_set_rate( gp_clk, 24576000 );
	clk_enable( gp_clk );

	return 0;
}

static int tuner_clk_disable(void)
{
	clk_disable( gp_clk );

	return 0;
}
	

/**************************************************************************************************/
/**
	@brief	gpio_set
	@param	unsigned int	no			[I]	GPIO No
	@param	int				value		[I]	GPIO Value
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int gpio_set(unsigned int no, int value)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	stGPIO_DEF *p = use_gpiono;
	int flag = 0;
	int i;
	
	for(i=0; i<loop; i++, p++){
		if (p->no == no){
			flag = 1;
			break;
		}
	}
	if (flag == 0) {
		printk("%s: !!! gpio_set() error No.%d value %d \n", __FILE__, no, value);
		return EINVAL;
	}
	gpio_set_value(no, value);
	return 0;
}

/**************************************************************************************************/
/**
	@brief	gpio_get
	@param	unsigned int	no			[I]	GPIO No
	@param	int				*val		[I]	GPIO Value
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int gpio_get(unsigned int no, int *val)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	stGPIO_DEF *p = &use_gpiono[0];
	int flag = 0;
	int i;
	
	*val = 0;

	for(i=0; i<loop; i++, p++){
		if (p->no == no){
			flag = 1;
			break;
		}
	}
	if (flag == 0) {
		printk("%s: !!! gpio_get() No.%d error \n", __FILE__, no);
		return EINVAL;
	}
	*val = gpio_get_value(no);

	return 0;
}

/**************************************************************************************************/
/**
	@brief	tuner_vreg_enable
	@param	none
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int tuner_vreg_enable(void)
{
	struct vreg *vreg_gp5;
	int rc;

	vreg_gp5 = vreg_get(NULL, "gp5");
	if (IS_ERR(vreg_gp5)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
			__func__, "gp5", PTR_ERR(vreg_gp5));
		return -1;
	}

	rc = vreg_set_level(vreg_gp5, 1200);
	if (rc) {
		printk(KERN_ERR "%s: vreg gp5 set level failed (%d)\n",
			__func__, rc);
		return -2;
	}

	rc = vreg_enable(vreg_gp5);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
			__func__, rc);
		return -3;
	}

	return 0;
}

/**************************************************************************************************/
/**
	@brief	tuner_vreg_disable
	@param	none
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int tuner_vreg_disable(void)
{
	struct vreg *vreg_gp5;
	int rc;

	vreg_gp5 = vreg_get(NULL, "gp5");
	if (IS_ERR(vreg_gp5)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
			__func__, "gp5", PTR_ERR(vreg_gp5));
		return -1;
	}

	rc = vreg_set_level(vreg_gp5, 0);
	if (rc) {
		printk(KERN_ERR "%s: vreg gp5 set level failed (%d)\n",
			__func__, rc);
		return -2;
	}

	rc = vreg_disable(vreg_gp5);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
			__func__, rc);
		return -3;
	}

	return 0;
}

MODULE_LICENSE("GPL");
module_init(tuner_init);
module_exit(tuner_cleanup);
