/* drivers/i2c/busses/i2c-msm.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/board.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/remote_spinlock.h>
#include <linux/pm_qos_params.h>
#include <mach/gpio.h>

#if defined( CONFIG_I2C_CUST_SH_SMEM )
#include <../../../arch/arm/mach-msm/smd_private.h>
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */

#if defined( CONFIG_I2C_CUST_SH )
#define	XFER_RETRY	2
/* #define SH_I2C_WAIT_NEED */
#define	SH_I2C_SUSPEND_RETRY	5

#ifdef SH_I2C_WAIT_NEED
#define	SH_I2C_WAIT	30
#endif /* SH_I2C_WAIT_NEED */

#define	SH_I2C_CLK_STRETCH_WAIT	1000000
#define I2C_CLK_STATE_MASK		0xe000
#define I2C_CLK_STATE_BUSIDLE	0x0000

/* #define I2C_CUST_SH_MSM_RETRY_LOG */
#define SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_ES1	0x0E
#define SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_PP1	0x0F
#define SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_MIC	0x1E
#define SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_SES	0x2E

#define I2C_CUST_SH_MSM_ERROR_LOG_DISABLE
#if defined(I2C_CUST_SH_MSM_ERROR_LOG_DISABLE)
# undef dev_err
# define dev_err(dev, format, arg...)
#endif	/* I2C_CUST_SH_MSM_ERROR_LOG_DISABLE */

static int msm_i2c_sub_xfer(struct i2c_adapter *adap,
							struct i2c_msg msgs[], int num, int cnt);

#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

enum {
	I2C_WRITE_DATA          = 0x00,
	I2C_CLK_CTL             = 0x04,
	I2C_STATUS              = 0x08,
	I2C_READ_DATA           = 0x0c,
	I2C_INTERFACE_SELECT    = 0x10,

	I2C_WRITE_DATA_DATA_BYTE            = 0xff,
	I2C_WRITE_DATA_ADDR_BYTE            = 1U << 8,
	I2C_WRITE_DATA_LAST_BYTE            = 1U << 9,

	I2C_CLK_CTL_FS_DIVIDER_VALUE        = 0xff,
	I2C_CLK_CTL_HS_DIVIDER_VALUE        = 7U << 8,

	I2C_STATUS_WR_BUFFER_FULL           = 1U << 0,
	I2C_STATUS_RD_BUFFER_FULL           = 1U << 1,
	I2C_STATUS_BUS_ERROR                = 1U << 2,
	I2C_STATUS_PACKET_NACKED            = 1U << 3,
	I2C_STATUS_ARB_LOST                 = 1U << 4,
	I2C_STATUS_INVALID_WRITE            = 1U << 5,
	I2C_STATUS_FAILED                   = 3U << 6,
	I2C_STATUS_BUS_ACTIVE               = 1U << 8,
	I2C_STATUS_BUS_MASTER               = 1U << 9,
	I2C_STATUS_ERROR_MASK               = 0xfc,

	I2C_INTERFACE_SELECT_INTF_SELECT    = 1U << 0,
	I2C_INTERFACE_SELECT_SCL            = 1U << 8,
	I2C_INTERFACE_SELECT_SDA            = 1U << 9,
	I2C_STATUS_RX_DATA_STATE            = 3U << 11,
	I2C_STATUS_LOW_CLK_STATE            = 3U << 13,
};

struct msm_i2c_dev {
	struct device                *dev;
	void __iomem                 *base;	/* virtual */
	int                          irq;
	struct clk                   *clk;
	struct i2c_adapter           adap_pri;
	struct i2c_adapter           adap_aux;

	spinlock_t                   lock;

	struct i2c_msg               *msg;
	int                          rem;
	int                          pos;
	int                          cnt;
	int                          err;
	int                          flush_cnt;
	int                          rd_acked;
	int                          one_bit_t;
	remote_mutex_t               r_lock;
#if defined( CONFIG_I2C_CUST_SH_SMEM )
	remote_spinlock_t            s_lock;
	int                          ch_id;
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */
	int                          suspended;
	struct mutex                 mlock;
	struct msm_i2c_platform_data *pdata;
	struct timer_list            pwr_timer;
	int                          clk_state;
	void                         *complete;

	struct pm_qos_request_list *pm_qos_req;
};

static void
msm_i2c_pwr_mgmt(struct msm_i2c_dev *dev, unsigned int state)
{
	dev->clk_state = state;
	if (state != 0)
		clk_enable(dev->clk);
	else
		clk_disable(dev->clk);
}

static void
msm_i2c_pwr_timer(unsigned long data)
{
	struct msm_i2c_dev *dev = (struct msm_i2c_dev *) data;
	dev_dbg(dev->dev, "I2C_Power: Inactivity based power management\n");
	if (dev->clk_state == 1)
		msm_i2c_pwr_mgmt(dev, 0);
}

#if defined( CONFIG_I2C_CUST_SH )
static struct msm_i2c_dev *msm_i2c_dev_ch1 = 0;
static inline void msm_i2c_store_devices( struct msm_i2c_dev *dev )
{
	if( dev->ch_id == 0 )
		msm_i2c_dev_ch1 = dev;
}

void sh_msm_i2c_pm_idle(void)
{
	if( msm_i2c_dev_ch1 == NULL )
		return;

	if( ! mutex_trylock(&msm_i2c_dev_ch1->mlock) ) {
		dev_dbg(msm_i2c_dev_ch1->dev,"sh_msm_i2c_pm_idle() : mutex_trylock() failed\n");
		return;
	}

	del_timer_sync(&msm_i2c_dev_ch1->pwr_timer);
	if( msm_i2c_dev_ch1->clk_state != 0 )
		msm_i2c_pwr_mgmt(msm_i2c_dev_ch1, 0);
	mutex_unlock(&msm_i2c_dev_ch1->mlock);
}
EXPORT_SYMBOL(sh_msm_i2c_pm_idle);
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

#ifdef DEBUG
static void
dump_status(uint32_t status)
{
	printk("STATUS (0x%.8x): ", status);
	if (status & I2C_STATUS_BUS_MASTER)
		printk("MST ");
	if (status & I2C_STATUS_BUS_ACTIVE)
		printk("ACT ");
	if (status & I2C_STATUS_INVALID_WRITE)
		printk("INV_WR ");
	if (status & I2C_STATUS_ARB_LOST)
		printk("ARB_LST ");
	if (status & I2C_STATUS_PACKET_NACKED)
		printk("NAK ");
	if (status & I2C_STATUS_BUS_ERROR)
		printk("BUS_ERR ");
	if (status & I2C_STATUS_RD_BUFFER_FULL)
		printk("RD_FULL ");
	if (status & I2C_STATUS_WR_BUFFER_FULL)
		printk("WR_FULL ");
	if (status & I2C_STATUS_FAILED)
		printk("FAIL 0x%x", (status & I2C_STATUS_FAILED));
	printk("\n");
}
#endif

#if defined( CONFIG_I2C_CUST_SH )
static int sh_i2c_clk_status_check( struct msm_i2c_dev *dev )
{
	unsigned long loop_count = SH_I2C_CLK_STRETCH_WAIT;

	/* Wait I2C_CLK_STATE_BUSIDLE */
	while ( I2C_CLK_STATE_BUSIDLE != (readl(dev->base + I2C_STATUS)
												 & I2C_CLK_STATE_MASK) ){
		if (loop_count == 0){
			printk(KERN_ERR "%s: i2c_not idle %x \n"
							, __func__
							, readl(dev->base + I2C_STATUS) );
			return -EIO;
		}
		udelay(1);
		loop_count--;
    }

	return 0;
}	/* sh_i2c_clk_status_check */
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

static irqreturn_t
msm_i2c_interrupt(int irq, void *devid)
{
	struct msm_i2c_dev *dev = devid;
	uint32_t status = readl(dev->base + I2C_STATUS);
	int err = 0;

#ifdef DEBUG
	dump_status(status);
#endif

	spin_lock(&dev->lock);
	if (!dev->msg) {
		printk(KERN_ERR "%s: IRQ but nothing to do!\n", __func__);
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (status & I2C_STATUS_ERROR_MASK) {
		err = -EIO;
		goto out_err;
	}

	if (dev->msg->flags & I2C_M_RD) {
		if (status & I2C_STATUS_RD_BUFFER_FULL) {

			/*
			 * Theres something in the FIFO.
			 * Are we expecting data or flush crap?
			 */
			if (dev->cnt) { /* DATA */
				uint8_t *data = &dev->msg->buf[dev->pos];

				/* This is in spin-lock. So there will be no
				 * scheduling between reading the second-last
				 * byte and writing LAST_BYTE to the controller.
				 * So extra read-cycle-clock won't be generated
				 * Per I2C MSM HW Specs: Write LAST_BYTE befure
				 * reading 2nd last byte
				 */
				if (dev->cnt == 2)
					writel(I2C_WRITE_DATA_LAST_BYTE,
						dev->base + I2C_WRITE_DATA);
				*data = readl(dev->base + I2C_READ_DATA);
				dev->cnt--;
				dev->pos++;
				if (dev->msg->len == 1)
					dev->rd_acked = 0;
				if (dev->cnt == 0)
					goto out_complete;

			} else {
				/* Now that extra read-cycle-clocks aren't
				 * generated, this becomes error condition
				 */
				dev_err(dev->dev,
					"read did not stop, status - %x\n",
					status);
				err = -EIO;
				goto out_err;
			}
		} else if (dev->msg->len == 1 && dev->rd_acked == 0 &&
				((status & I2C_STATUS_RX_DATA_STATE) ==
				 I2C_STATUS_RX_DATA_STATE))
			writel(I2C_WRITE_DATA_LAST_BYTE,
				dev->base + I2C_WRITE_DATA);
	} else {
		uint16_t data;

		if (status & I2C_STATUS_WR_BUFFER_FULL) {
			dev_err(dev->dev,
				"Write buffer full in ISR on write?\n");
			err = -EIO;
			goto out_err;
		}

		if (dev->cnt) {
			/* Ready to take a byte */
			data = dev->msg->buf[dev->pos];
			if (dev->cnt == 1 && dev->rem == 1)
				data |= I2C_WRITE_DATA_LAST_BYTE;

			status = readl(dev->base + I2C_STATUS);
			/*
			 * Due to a hardware timing issue, data line setup time
			 * may be reduced to less than recommended 250 ns.
			 * This happens when next byte is written in a
			 * particular window of clock line being low and master
			 * not stretching the clock line. Due to setup time
			 * violation, some slaves may miss first-bit of data, or
			 * misinterprete data as start condition.
			 * We introduce delay of just over 1/2 clock cycle to
			 * ensure master stretches the clock line thereby
			 * avoiding setup time violation. Delay is introduced
			 * only if I2C clock FSM is LOW. The delay is not needed
			 * if I2C clock FSM is HIGH or FORCED_LOW.
			 */
			if ((status & I2C_STATUS_LOW_CLK_STATE) ==
					I2C_STATUS_LOW_CLK_STATE)
				udelay((dev->one_bit_t >> 1) + 1);
			writel(data, dev->base + I2C_WRITE_DATA);
			dev->pos++;
			dev->cnt--;
		} else
			goto out_complete;
	}

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;

 out_err:
	dev->err = err;
 out_complete:
	complete(dev->complete);
	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

static int
msm_i2c_poll_writeready(struct msm_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + I2C_STATUS);

		if (!(status & I2C_STATUS_WR_BUFFER_FULL))
			return 0;
		if (retries++ > 1000)
			usleep_range(100, 200);
	}
	return -ETIMEDOUT;
}

static int
msm_i2c_poll_notbusy(struct msm_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + I2C_STATUS);

		if (!(status & I2C_STATUS_BUS_ACTIVE))
			return 0;
		if (retries++ > 1000)
			usleep_range(100, 200);
	}
	return -ETIMEDOUT;
}

static int
msm_i2c_recover_bus_busy(struct msm_i2c_dev *dev, struct i2c_adapter *adap)
{
	int i;
	int gpio_clk;
	int gpio_dat;
	uint32_t status = readl(dev->base + I2C_STATUS);
#if defined( CONFIG_I2C_CUST_SH )
	uint32_t intfc = readl(dev->base + I2C_INTERFACE_SELECT);
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */
	bool gpio_clk_status = false;

	if (!(status & (I2C_STATUS_BUS_ACTIVE | I2C_STATUS_WR_BUFFER_FULL)))
		return 0;

	dev->pdata->msm_i2c_config_gpio(adap->nr, 0);
	/* Even adapter is primary and Odd adapter is AUX */
	if (adap->nr % 2) {
		gpio_clk = dev->pdata->aux_clk;
		gpio_dat = dev->pdata->aux_dat;
	} else {
		gpio_clk = dev->pdata->pri_clk;
		gpio_dat = dev->pdata->pri_dat;
	}

	disable_irq(dev->irq);
	if (status & I2C_STATUS_RD_BUFFER_FULL) {
		dev_warn(dev->dev, "Read buffer full, status %x, intf %x\n",
			 status, readl(dev->base + I2C_INTERFACE_SELECT));
		writel(I2C_WRITE_DATA_LAST_BYTE, dev->base + I2C_WRITE_DATA);
		readl(dev->base + I2C_READ_DATA);
	} else if (status & I2C_STATUS_BUS_MASTER) {
		dev_warn(dev->dev, "Still the bus master, status %x, intf %x\n",
			 status, readl(dev->base + I2C_INTERFACE_SELECT));
		writel(I2C_WRITE_DATA_LAST_BYTE | 0xff,
		       dev->base + I2C_WRITE_DATA);
	}

#if defined( CONFIG_I2C_CUST_SH )
	if (!(intfc & I2C_INTERFACE_SELECT_SCL)) {
		if (gpio_get_value(gpio_clk)) {
			dev_err(dev->dev, ">>> SCL Line is setted LOW by Master !! status = %x, interface_select = %x <<<\n", status, intfc);
		} else {
			dev_err(dev->dev, ">>> SCL Line is setted LOW by Slave  !! status = %x, interface_select = %x <<<\n", status, intfc);
		}
	}
	if (!(intfc & I2C_INTERFACE_SELECT_SDA)) {
		if (gpio_get_value(gpio_dat)) {
			dev_err(dev->dev, ">>> SDA Line is setted LOW by Master !! status = %x, interface_select = %x <<<\n", status, intfc);
		} else {
			dev_err(dev->dev, ">>> SDA Line is setted LOW by Slave  !! status = %x, interface_select = %x <<<\n", status, intfc);
		}
	}
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

	for (i = 0; i < 9; i++) {
		if (gpio_get_value(gpio_dat) && gpio_clk_status)
			break;
		gpio_direction_output(gpio_clk, 0);
		udelay(5);
		gpio_direction_output(gpio_dat, 0);
		udelay(5);
		gpio_direction_input(gpio_clk);
		udelay(5);
		if (!gpio_get_value(gpio_clk))
			usleep_range(20, 30);
		if (!gpio_get_value(gpio_clk))
			msleep(10);
		gpio_clk_status = gpio_get_value(gpio_clk);
		gpio_direction_input(gpio_dat);
		udelay(5);
	}
	dev->pdata->msm_i2c_config_gpio(adap->nr, 1);
	udelay(10);

	status = readl(dev->base + I2C_STATUS);
	if (!(status & I2C_STATUS_BUS_ACTIVE)) {
		dev_info(dev->dev, "Bus busy cleared after %d clock cycles, "
			 "status %x, intf %x\n",
			 i, status, readl(dev->base + I2C_INTERFACE_SELECT));
		enable_irq(dev->irq);
		return 0;
	}

	dev_err(dev->dev, "Bus still busy, status %x, intf %x\n",
		 status, readl(dev->base + I2C_INTERFACE_SELECT));
	enable_irq(dev->irq);
	return -EBUSY;
}

#if defined( CONFIG_I2C_CUST_SH_SMEM )
static void
msm_i2c_rspin_lock(struct msm_i2c_dev *dev)
{
	int gotlock = 0;
	unsigned long flags;
	uint32_t *smem_ptr = (uint32_t *)dev->pdata->rmutex;

	do {
		remote_spin_lock_irqsave(&dev->s_lock, flags);

		if (*smem_ptr == 0) {
			*smem_ptr = 1;
			gotlock = 1;
		}

		remote_spin_unlock_irqrestore(&dev->s_lock, flags);
	} while (!gotlock);
}

static void
msm_i2c_rspin_unlock(struct msm_i2c_dev *dev)
{
	unsigned long flags;
	uint32_t *smem_ptr = (uint32_t *)dev->pdata->rmutex;

	remote_spin_lock_irqsave(&dev->s_lock, flags);

	*smem_ptr = 0;

	remote_spin_unlock_irqrestore(&dev->s_lock, flags);
}

static void sh_msmi2c_mutex_lock( struct i2c_client *client )
{
	struct msm_i2c_dev *dev = i2c_get_adapdata(client->adapter);

	mutex_lock(&dev->mlock);
	msm_i2c_rspin_lock(dev);
}

void sh_msmi2c_1_mutex_lock( struct i2c_client *client )
{
	sh_msmi2c_mutex_lock(client);
}
EXPORT_SYMBOL(sh_msmi2c_1_mutex_lock);

static void sh_msmi2c_2_mutex_lock( struct i2c_client *client )
{}
EXPORT_SYMBOL(sh_msmi2c_2_mutex_lock);


static void sh_msmi2c_mutex_free( struct i2c_client *client )
{
	struct msm_i2c_dev *dev = i2c_get_adapdata(client->adapter);

	msm_i2c_rspin_unlock(dev);
	mutex_unlock(&dev->mlock);
}

void sh_msmi2c_1_mutex_free( struct i2c_client *client )
{
	sh_msmi2c_mutex_free(client);
}
EXPORT_SYMBOL(sh_msmi2c_1_mutex_free);

static void sh_msmi2c_2_mutex_free( struct i2c_client *client )
{}
EXPORT_SYMBOL(sh_msmi2c_2_mutex_free);
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */

static int
msm_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
#if defined( CONFIG_I2C_CUST_SH )
{
	int	ret;
	int	cnt;
	int	retry_cnt = XFER_RETRY;

	if(( msgs[0].addr == SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_ES1 ) || ( msgs[0].addr == SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_PP1 ) || 
		(msgs[0].addr ==  SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_MIC) || (msgs[0].addr == SH_I2C_INVALID_RETRY_SLAVE_ADDRESS_SES))
	{
		retry_cnt =  0;
	}

	for (cnt=0; cnt<=retry_cnt; cnt++) {
		ret = msm_i2c_sub_xfer(adap, msgs, num, cnt);
		if (ret >= 0) {
			return ret;
		}
	}
	return ret;
}
static int
msm_i2c_sub_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num, int cnt)
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */
{
	DECLARE_COMPLETION_ONSTACK(complete);
	struct msm_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	int rem = num;
	uint16_t addr;
	long timeout;
	unsigned long flags;
	int check_busy = 1;

	del_timer_sync(&dev->pwr_timer);
	mutex_lock(&dev->mlock);
	if (dev->suspended) {
#if defined( CONFIG_I2C_CUST_SH )
		uint32_t loop_cnt;

		for(loop_cnt=0;loop_cnt<SH_I2C_SUSPEND_RETRY;loop_cnt++){
			if(dev->suspended == 0)
				break;
			msleep(20);
		}
		if(dev->suspended == 1){
			mutex_unlock(&dev->mlock);
			dev_err(dev->dev,
				"Error: waiting for resume...[Addr=0x%x]\n",
											(char)msgs->addr << 1);
			return -EIO;
		}
#else
		mutex_unlock(&dev->mlock);
		return -EIO;
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */
	}

#if defined( CONFIG_I2C_CUST_SH_SMEM )
	if((dev->ch_id == 0) && (dev->pdata->rmutex == 0)) {
		dev->pdata->rmutex=(uint32_t)smem_alloc(SMEM_I2C_MUTEX, 8);
		if(dev->pdata->rmutex) {
			remote_spinlock_id_t rmid;
			rmid = dev->pdata->rsl_id;
			if (remote_spin_lock_init(&dev->s_lock, rmid) != 0) {
				dev_err(dev->dev, ">>> msm_i2c_sub_xfer() : remote_spin_lock_init() Error ! <<<\n");
				dev->pdata->rmutex = 0;
			}
		} else {
			dev_err(dev->dev, ">>> msm_i2c_sub_xfer() : SMEM NG ! <<<\n");
		}
	}
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */

	if (dev->clk_state == 0) {
		dev_dbg(dev->dev, "I2C_Power: Enable I2C clock(s)\n");
		msm_i2c_pwr_mgmt(dev, 1);
	}

	/* Don't allow power collapse until we release remote spinlock */
	pm_qos_update_request(dev->pm_qos_req,  dev->pdata->pm_lat);
	if (dev->pdata->rmutex) {
#if defined( CONFIG_I2C_CUST_SH_SMEM )
		if(dev->pdata->rsl_id[0] == 'S')
		{
			msm_i2c_rspin_lock(dev);
		}else{
			remote_mutex_lock(&dev->r_lock);
		}
#else
		remote_mutex_lock(&dev->r_lock);
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */
		/* If other processor did some transactions, we may have
		 * interrupt pending. Clear it
		 */
		get_irq_chip(dev->irq)->ack(dev->irq);
	}

	if (adap == &dev->adap_pri)
		writel(0, dev->base + I2C_INTERFACE_SELECT);
	else
		writel(I2C_INTERFACE_SELECT_INTF_SELECT,
				dev->base + I2C_INTERFACE_SELECT);
	enable_irq(dev->irq);
	while (rem) {
		addr = msgs->addr << 1;

		#if defined( CONFIG_I2C_CUST_SH )
		if (msgs->flags & I2C_M_RD){
			addr |= 1;
			readl(dev->base + I2C_READ_DATA);
		}
		#else
		if (msgs->flags & I2C_M_RD)
			addr |= 1;
		#endif	/* #if defined( CONFIG_I2C_CUST_SH ) */

		spin_lock_irqsave(&dev->lock, flags);
		dev->msg = msgs;
		dev->rem = rem;
		dev->pos = 0;
		dev->err = 0;
		dev->flush_cnt = 0;
		dev->cnt = msgs->len;
		dev->complete = &complete;
		spin_unlock_irqrestore(&dev->lock, flags);

		if (check_busy) {
			ret = msm_i2c_poll_notbusy(dev);
			if (ret)
				ret = msm_i2c_recover_bus_busy(dev, adap);
				if (ret) {
					dev_err(dev->dev,
						"Error waiting for notbusy\n");
					goto out_err;
				}
			check_busy = 0;
		}

		if (rem == 1 && msgs->len == 0)
			addr |= I2C_WRITE_DATA_LAST_BYTE;

		/* Wait for WR buffer not full */
		ret = msm_i2c_poll_writeready(dev);
		if (ret) {
			ret = msm_i2c_recover_bus_busy(dev, adap);
			if (ret) {
				dev_err(dev->dev,
				"Error waiting for write ready before addr\n");
				goto out_err;
			}
		}

		/* special case for doing 1 byte read.
		 * There should be no scheduling between I2C controller becoming
		 * ready to read and writing LAST-BYTE to I2C controller
		 * This will avoid potential of I2C controller starting to latch
		 * another extra byte.
		 */
		if ((msgs->len == 1) && (msgs->flags & I2C_M_RD)) {
			uint32_t retries = 0;
			spin_lock_irqsave(&dev->lock, flags);

			writel(I2C_WRITE_DATA_ADDR_BYTE | addr,
				dev->base + I2C_WRITE_DATA);

			/* Poll for I2C controller going into RX_DATA mode to
			 * ensure controller goes into receive mode.
			 * Just checking write_buffer_full may not work since
			 * there is delay between the write-buffer becoming
			 * empty and the slave sending ACK to ensure I2C
			 * controller goes in receive mode to receive data.
			 */
			while (retries != 2000) {
				uint32_t status = readl(dev->base + I2C_STATUS);

					if ((status & I2C_STATUS_RX_DATA_STATE)
						== I2C_STATUS_RX_DATA_STATE)
						break;
				retries++;
			}
			if (retries >= 2000) {
				dev->rd_acked = 0;
				spin_unlock_irqrestore(&dev->lock, flags);
				/* 1-byte-reads from slow devices in interrupt
				 * context
				 */
				goto wait_for_int;
			}

			dev->rd_acked = 1;
			writel(I2C_WRITE_DATA_LAST_BYTE,
					dev->base + I2C_WRITE_DATA);
			spin_unlock_irqrestore(&dev->lock, flags);
		} else {
			writel(I2C_WRITE_DATA_ADDR_BYTE | addr,
					 dev->base + I2C_WRITE_DATA);
		}
		/* Polling and waiting for write_buffer_empty is not necessary.
		 * Even worse, if we do, it can result in invalid status and
		 * error if interrupt(s) occur while polling.
		 */

		/*
		 * Now that we've setup the xfer, the ISR will transfer the data
		 * and wake us up with dev->err set if there was an error
		 */
wait_for_int:

		timeout = wait_for_completion_timeout(&complete, HZ);
		if (!timeout) {
			dev_err(dev->dev, "Transaction timed out\n");
			writel(I2C_WRITE_DATA_LAST_BYTE,
				dev->base + I2C_WRITE_DATA);
			msleep(100);
			/* FLUSH */
			readl(dev->base + I2C_READ_DATA);
			readl(dev->base + I2C_STATUS);
			ret = -ETIMEDOUT;
			goto out_err;
		}
		if (dev->err) {
			dev_err(dev->dev,
				"(%04x) Error during data xfer (%d)\n",
				addr, dev->err);
			ret = dev->err;
			goto out_err;
		}

		if (msgs->flags & I2C_M_RD)
			check_busy = 1;

		msgs++;
		rem--;
	}

	ret = num;

#if defined( CONFIG_I2C_CUST_SH )
	if( sh_i2c_clk_status_check( dev ) < 0 ){
		ret = -EIO;
	}
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

 out_err:
	spin_lock_irqsave(&dev->lock, flags);
	dev->complete = NULL;
	dev->msg = NULL;
	dev->rem = 0;
	dev->pos = 0;
	dev->err = 0;
	dev->flush_cnt = 0;
	dev->cnt = 0;
	spin_unlock_irqrestore(&dev->lock, flags);
	disable_irq(dev->irq);

#if defined( CONFIG_I2C_CUST_SH )
	if( ret < 0){
		#if defined( I2C_CUST_SH_MSM_RETRY_LOG )
			printk(KERN_ERR "%s: Retry [%d] [Addr=0x%x]\n", __func__, (cnt+1), (char)addr);
		#endif /* #if defined( I2C_CUST_SH_MSM_RETRY_LOG ) */
	}else{
		if (cnt != 0) {
			#if defined( I2C_CUST_SH_MSM_RETRY_LOG )
				printk(KERN_ERR "%s: Retry Complete [Addr=0x%x]\n", __func__,(char)addr);
			#endif /* #if defined( I2C_CUST_SH_MSM_RETRY_LOG ) */
		}
		#ifdef SH_I2C_WAIT_NEED
		udelay( SH_I2C_WAIT );
		#endif /* SH_I2C_WAIT_NEED */
	}
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

#if defined( CONFIG_I2C_CUST_SH_SMEM )
	if (dev->pdata->rmutex)
	{
		if (dev->pdata->rsl_id[0] == 'S')
			msm_i2c_rspin_unlock(dev);
		else
			remote_mutex_unlock(&dev->r_lock);
	}
#else
	if (dev->pdata->rmutex)
		remote_mutex_unlock(&dev->r_lock);
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */
	pm_qos_update_request(dev->pm_qos_req,
			      PM_QOS_DEFAULT_VALUE);
	mod_timer(&dev->pwr_timer, (jiffies + 3*HZ));
	mutex_unlock(&dev->mlock);
	return ret;
}

static u32
msm_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm msm_i2c_algo = {
	.master_xfer	= msm_i2c_xfer,
	.functionality	= msm_i2c_func,
};

static int
msm_i2c_probe(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev;
	struct resource		*mem, *irq, *ioarea;
	int ret;
	int fs_div;
	int hs_div;
	int i2c_clk;
	int clk_ctl;
	struct clk *clk;
	struct msm_i2c_platform_data *pdata;

	printk(KERN_INFO "msm_i2c_probe\n");

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}
	clk = clk_get(&pdev->dev, "i2c_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Could not get clock\n");
		ret = PTR_ERR(clk);
		goto err_clk_get_failed;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform data not initialized\n");
		ret = -ENOSYS;
		goto err_clk_get_failed;
	}
	if (!pdata->msm_i2c_config_gpio) {
		dev_err(&pdev->dev, "config_gpio function not initialized\n");
		ret = -ENOSYS;
		goto err_clk_get_failed;
	}
	/* We support frequencies upto FAST Mode(400KHz) */
	if (pdata->clk_freq <= 0 || pdata->clk_freq > 400000) {
		dev_err(&pdev->dev, "clock frequency not supported\n");
		ret = -EIO;
		goto err_clk_get_failed;
	}

	dev = kzalloc(sizeof(struct msm_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_alloc_dev_failed;
	}

	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	dev->clk = clk;
	dev->pdata = pdata;
	dev->base = ioremap(mem->start, (mem->end - mem->start) + 1);
	if (!dev->base) {
		ret = -ENOMEM;
		goto err_ioremap_failed;
	}

	dev->one_bit_t = USEC_PER_SEC/pdata->clk_freq;
	spin_lock_init(&dev->lock);
	platform_set_drvdata(pdev, dev);

	clk_enable(clk);

#if defined( CONFIG_I2C_CUST_SH_SMEM )
	dev->ch_id = (int)pdev->id;
	if (pdata->rmutex && pdata->rsl_id[0] == 'S') {
		remote_spinlock_id_t rmid;
		rmid = pdata->rsl_id;
		if (remote_spin_lock_init(&dev->s_lock, rmid) != 0) {
			pdata->rmutex = 0;
			dev_err(&pdev->dev, ">>> msm_i2c_probe() : SMEM NG ! <<<\n");
		}
	} else if (pdata->rmutex) {
		struct remote_mutex_id rmid;
		rmid.r_spinlock_id = pdata->rsl_id;
		rmid.delay_us = 10000000/pdata->clk_freq;
		if (remote_mutex_init(&dev->r_lock, &rmid) != 0) {
			pdata->rmutex = 0;
			dev_err(&pdev->dev, ">>> msm_i2c_probe() : SMEM NG ! <<<\n");
		}
	} else {
		if(pdev->id == 0) {
			dev_err(&pdev->dev, ">>> msm_i2c_probe() : SMEM NG ! <<<\n");
		}
	}
#else
	if (pdata->rmutex) {
		struct remote_mutex_id rmid;
		rmid.r_spinlock_id = pdata->rsl_id;
		rmid.delay_us = 10000000/pdata->clk_freq;
		if (remote_mutex_init(&dev->r_lock, &rmid) != 0)
			pdata->rmutex = 0;
	}
#endif /* #if defined( CONFIG_I2C_CUST_SH_SMEM ) */

	/* I2C_HS_CLK = I2C_CLK/(3*(HS_DIVIDER_VALUE+1) */
	/* I2C_FS_CLK = I2C_CLK/(2*(FS_DIVIDER_VALUE+3) */
	/* FS_DIVIDER_VALUE = ((I2C_CLK / I2C_FS_CLK) / 2) - 3 */
	i2c_clk = 19200000; /* input clock */
	fs_div = ((i2c_clk / pdata->clk_freq) / 2) - 3;
	hs_div = 3;
	clk_ctl = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
	writel(clk_ctl, dev->base + I2C_CLK_CTL);
	printk(KERN_INFO "msm_i2c_probe: clk_ctl %x, %d Hz\n",
	       clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 3)));

	i2c_set_adapdata(&dev->adap_pri, dev);
	dev->adap_pri.algo = &msm_i2c_algo;
	strlcpy(dev->adap_pri.name,
		"MSM I2C adapter-PRI",
		sizeof(dev->adap_pri.name));

	dev->adap_pri.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adap_pri);
	if (ret) {
		dev_err(&pdev->dev, "Primary i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}

	i2c_set_adapdata(&dev->adap_aux, dev);
	dev->adap_aux.algo = &msm_i2c_algo;
	strlcpy(dev->adap_aux.name,
		"MSM I2C adapter-AUX",
		sizeof(dev->adap_aux.name));

	dev->adap_aux.nr = pdev->id + 1;
	ret = i2c_add_numbered_adapter(&dev->adap_aux);
	if (ret) {
		dev_err(&pdev->dev, "auxiliary i2c_add_adapter failed\n");
		i2c_del_adapter(&dev->adap_pri);
		goto err_i2c_add_adapter_failed;
	}
	ret = request_irq(dev->irq, msm_i2c_interrupt,
			IRQF_TRIGGER_RISING, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	dev->pm_qos_req = pm_qos_add_request(PM_QOS_CPU_DMA_LATENCY,
					     PM_QOS_DEFAULT_VALUE);
	if (!dev->pm_qos_req) {
		dev_err(&pdev->dev, "pm_qos_add_request failed\n");
		goto err_pm_qos_add_request_failed;
	}

#if defined( CONFIG_I2C_CUST_SH_SMEM )
	msm_i2c_store_devices( dev );
#endif /* #if defined( CONFIG_I2C_CUST_SH ) */

	disable_irq(dev->irq);
	dev->suspended = 0;
	mutex_init(&dev->mlock);
	dev->clk_state = 0;
	/* Config GPIOs for primary and secondary lines */
	pdata->msm_i2c_config_gpio(dev->adap_pri.nr, 1);
	pdata->msm_i2c_config_gpio(dev->adap_aux.nr, 1);
	clk_disable(dev->clk);
	setup_timer(&dev->pwr_timer, msm_i2c_pwr_timer, (unsigned long) dev);

	return 0;

err_pm_qos_add_request_failed:
	free_irq(dev->irq, dev); 
err_request_irq_failed:
	i2c_del_adapter(&dev->adap_pri);
	i2c_del_adapter(&dev->adap_aux);
err_i2c_add_adapter_failed:
	clk_disable(clk);
	iounmap(dev->base);
err_ioremap_failed:
	kfree(dev);
err_alloc_dev_failed:
	clk_put(clk);
err_clk_get_failed:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int
msm_i2c_remove(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	/* Grab mutex to ensure ongoing transaction is over */
	mutex_lock(&dev->mlock);
	dev->suspended = 1;
	mutex_unlock(&dev->mlock);
	mutex_destroy(&dev->mlock);
	del_timer_sync(&dev->pwr_timer);
	if (dev->clk_state != 0)
		msm_i2c_pwr_mgmt(dev, 0);
	platform_set_drvdata(pdev, NULL);
	pm_qos_remove_request(dev->pm_qos_req);
	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adap_pri);
	i2c_del_adapter(&dev->adap_aux);
	clk_put(dev->clk);
	iounmap(dev->base);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem)
		release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

static int msm_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct msm_i2c_dev *dev = platform_get_drvdata(pdev);
	/* Wait until current transaction finishes
	 * Make sure remote lock is released before we suspend
	 */
	if (dev) {
		/* Grab mutex to ensure ongoing transaction is over */
		mutex_lock(&dev->mlock);
		dev->suspended = 1;
		mutex_unlock(&dev->mlock);
		del_timer_sync(&dev->pwr_timer);
		if (dev->clk_state != 0)
			msm_i2c_pwr_mgmt(dev, 0);
	}

	return 0;
}

static int msm_i2c_resume(struct platform_device *pdev)
{
	struct msm_i2c_dev *dev = platform_get_drvdata(pdev);
	dev->suspended = 0;
	return 0;
}

static struct platform_driver msm_i2c_driver = {
	.probe		= msm_i2c_probe,
	.remove		= msm_i2c_remove,
	.suspend	= msm_i2c_suspend,
	.resume		= msm_i2c_resume,
	.driver		= {
		.name	= "msm_i2c",
		.owner	= THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
msm_i2c_init_driver(void)
{
	return platform_driver_register(&msm_i2c_driver);
}
subsys_initcall(msm_i2c_init_driver);

static void __exit msm_i2c_exit_driver(void)
{
	platform_driver_unregister(&msm_i2c_driver);
}
module_exit(msm_i2c_exit_driver);

