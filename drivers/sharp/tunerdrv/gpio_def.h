/**************************************************************************************************/
/** 
	@file		gpio_def.h
	@brief		GPIO Definition Header
*/
/**************************************************************************************************/

#ifndef GPIO_DEF
	#define GPIO_DEF

typedef struct GPIO_DEF {
	unsigned int no;		/* GPIO Number */
	int direction;			/* I/O Direction */
	int out_val;			/* Initialized Value */
	int init_done;			/* GPIO Initialized ? 1:Complete (Don't Care) */
} stGPIO_DEF;

#define DirctionIn (0)
#define DirctionOut (1)

#if 0
#define GPIO_PWRDWN_PORTNO		(43)
#define GPIO_GTDION_PORTNO		(34)
#define GPIO_LDO_PORTNO			(152)
#define GPIO_SRDT_TUNER_CTL		(107)
#define GPIO_PBVAL_TUNER		(108)
#define GPIO_SRCK_TUNER			(109)
#else
#define GPIO_DTVEN_PORTNO		(91)
#define GPIO_DTVRST_PORTNO		(90)
#define GPIO_DTVLNAEN_PORTNO	(37)
#endif

typedef struct __ioctl_cmd{
	unsigned int no;
	unsigned int val;
} ioctl_cmd;

#define TUNERDRV_IOCTL_MAGIC 't'
#define IOC_GPIO_VAL_SET	_IOW(TUNERDRV_IOCTL_MAGIC, 0x01, int)
#define IOC_GPIO_VAL_GET	_IOR(TUNERDRV_IOCTL_MAGIC, 0x02, int)
#define IOC_VREG_ENABLE		_IOW(TUNERDRV_IOCTL_MAGIC, 0x03, int)
#define IOC_VREG_DISABLE	_IOW(TUNERDRV_IOCTL_MAGIC, 0x04, int)
#define IOC_CLK_ENABLE		_IOW(TUNERDRV_IOCTL_MAGIC, 0x05, int)
#define IOC_CLK_DISABLE		_IOW(TUNERDRV_IOCTL_MAGIC, 0x06, int)

#define USE_GPIO_MAX	(3)

#endif	//GPIO_DEF
