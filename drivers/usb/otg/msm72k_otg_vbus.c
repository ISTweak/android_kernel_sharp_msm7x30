/* drivers/usb/otg/msm72k_otg_vbus.c  (SHVBUS driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This code borrows from msm72k_otg.c, which is
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/pm_runtime.h>

#include <linux/device.h>
#include <linux/pm_qos_params.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm72k_otg.h>
#include <mach/msm_hsusb.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <mach/clk.h>
#include <mach/msm_xo.h>
#ifdef CONFIG_USB_SWIC
	#include <sharp/shswic_kerl.h>
#endif /* CONFIG_USB_SWIC */
#include "msm72k_otg_vbus.h"


#ifdef CONFIG_USB_SWIC
	#define D_WAIT_TIME_FOR_STABILITY	(0)
#else /* CONFIG_USB_SWIC */
	#define D_WAIT_TIME_FOR_STABILITY	(msecs_to_jiffies(500))
#endif /* CONFIG_USB_SWIC */

#ifdef CONFIG_USB_MSM_OTG_FS_RISEFALL_TIME_PLUS_30
	#define ULPI_CONFIG_REG4					(0x33)
	#define FS_RISE_FALL_TIME_CONTROL_MASK		(3<<2)
	#define FS_RISE_FALL_TIME_CONTROL_DEFAULT	(1<<2)
	#define FS_RISE_FALL_TIME_CONTROL_MINUS30	(0<<2)
	#define FS_RISE_FALL_TIME_CONTROL_PLUS30	(3<<2)
#endif /* CONFIG_USB_MSM_OTG_FS_RISEFALL_TIME_PLUS_30 */

#define MSM_USB_BASE	(dev->regs)
#define USB_LINK_RESET_TIMEOUT	(msecs_to_jiffies(10))
#define DRIVER_NAME	"msm_otg"
static void otg_reset(struct otg_transceiver *xceiv, int phy_reset);
static void msm_otg_set_vbus_state(int online);
static void msm_otg_set_id_state(int online);
static int msm_otg_set_suspend(struct otg_transceiver *xceiv, int suspend);

struct msm_otg *the_msm_otg;
static unsigned shvbus_charge_wait_val = 0;
static unsigned shvbus_charge_val = 0;
static u32	shvbus_force_charge_flg = 0;
static u32	shvbus_usb_force_disconnect = 0;


static unsigned ulpi_read(struct msm_otg *dev, unsigned reg)
{
	unsigned ret, timeout = 100000;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		pr_err("%s: timeout %08x\n", __func__,
				 readl(USB_ULPI_VIEWPORT));
		spin_unlock_irqrestore(&dev->lock, flags);
		return 0xffffffff;
	}
	ret = ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));

	spin_unlock_irqrestore(&dev->lock, flags);

	return ret;
}

static int ulpi_write(struct msm_otg *dev, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		;

	if (timeout == 0) {
		pr_err("%s: timeout\n", __func__);
		spin_unlock_irqrestore(&dev->lock, flags);
		return -1;
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static int usb_ulpi_write(struct otg_transceiver *xceiv, u32 val, u32 reg)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	return ulpi_write(dev, val, reg);
}

static int usb_ulpi_read(struct otg_transceiver *xceiv, u32 reg)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	return ulpi_read(dev, reg);
}

#ifndef CONFIG_USB_SWIC
static void enable_sess_valid(struct msm_otg *dev)
{
	/* Do nothing if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return;

	ulpi_write(dev, (1<<2), 0x0E);
	ulpi_write(dev, (1<<2), 0x11);
	writel(readl(USB_OTGSC) | OTGSC_BSVIE, USB_OTGSC);
}
#endif /* CONFIG_USB_SWIC */

static void disable_sess_valid(struct msm_otg *dev)
{
	/* Do nothing if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return;

	ulpi_write(dev, (1<<2), 0x0F);
	ulpi_write(dev, (1<<2), 0x12);
	writel(readl(USB_OTGSC) & ~OTGSC_BSVIE, USB_OTGSC);
}

#define get_aca_bmaxpower(dev)		0
#define set_aca_bmaxpower(dev, power)

static inline void set_pre_emphasis_level(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->pemp_level == PRE_EMPHASIS_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_CONFIG_REG3);
	res &= ~(ULPI_PRE_EMPHASIS_MASK);
	if (dev->pdata->pemp_level != PRE_EMPHASIS_DISABLE)
		res |= dev->pdata->pemp_level;
	ulpi_write(dev, res, ULPI_CONFIG_REG3);
}

static inline void set_cdr_auto_reset(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->cdr_autoreset == CDR_AUTO_RESET_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_DIGOUT_CTRL);
	if (dev->pdata->cdr_autoreset == CDR_AUTO_RESET_ENABLE)
		res &=  ~ULPI_CDR_AUTORESET;
	else
		res |=  ULPI_CDR_AUTORESET;
	ulpi_write(dev, res, ULPI_DIGOUT_CTRL);
}

static inline void set_se1_gating(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->se1_gating == SE1_GATING_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_DIGOUT_CTRL);
	if (dev->pdata->se1_gating == SE1_GATING_ENABLE)
		res &=  ~ULPI_SE1_GATE;
	else
		res |=  ULPI_SE1_GATE;
	ulpi_write(dev, res, ULPI_DIGOUT_CTRL);
}
static inline void set_driver_amplitude(struct msm_otg *dev)
{
	unsigned res = 0;

	if (!dev->pdata || dev->pdata->drv_ampl == HS_DRV_AMPLITUDE_DEFAULT)
		return;

	res = ulpi_read(dev, ULPI_CONFIG_REG2);
	res &= ~ULPI_DRV_AMPL_MASK;
	if (dev->pdata->drv_ampl != HS_DRV_AMPLITUDE_ZERO_PERCENT)
		res |= dev->pdata->drv_ampl;
	ulpi_write(dev, res, ULPI_CONFIG_REG2);
}

#ifdef CONFIG_USB_MSM_OTG_FS_RISEFALL_TIME_PLUS_30
static inline void set_fs_risefall_time(struct msm_otg *dev)
{
	unsigned res = 0;
	unsigned set_value = FS_RISE_FALL_TIME_CONTROL_PLUS30;

	res = ulpi_read(dev, ULPI_CONFIG_REG4);
	res &= ~FS_RISE_FALL_TIME_CONTROL_MASK;
	res |= set_value;
	ulpi_write(dev, res, ULPI_CONFIG_REG4);
}
#endif /* CONFIG_USB_MSM_OTG_FS_RISEFALL_TIME_PLUS_30 */

static const char *state_string(enum usb_shvbus_state state)
{
	switch (state) {
	case SHVBUS_STS_INIT:			return "init";
	case SHVBUS_STS_DISCONNECT:		return "disconnect";
	case SHVBUS_STS_CONNECT_AC:		return "connect_ac";
	case SHVBUS_STS_CONNECT_USB:		return "connect_usb";
	case SHVBUS_STS_DISCON_USB_WAIT:	return "discon_usb_wait";
	default:				return "UNDEFINED";
	}
}

static const char *timer_string(int bit)
{
	switch (bit) {
	case A_WAIT_VRISE:		return "a_wait_vrise";
	case A_WAIT_VFALL:		return "a_wait_vfall";
	case B_SRP_FAIL:		return "b_srp_fail";
	case A_WAIT_BCON:		return "a_wait_bcon";
	case A_AIDL_BDIS:		return "a_aidl_bdis";
	case A_BIDL_ADIS:		return "a_bidl_adis";
	case B_ASE0_BRST:		return "b_ase0_brst";
	case SHVBUS_TMR_DISCON_USB_WAIT:return "usb_discon_tmr";
	default:			return "UNDEFINED";
	}
}

/* Prevent idle power collapse(pc) while operating in peripheral mode */
static void otg_pm_qos_update_latency(struct msm_otg *dev, int vote)
{
	struct msm_otg_platform_data *pdata = dev->pdata;
	u32 swfi_latency = 0;

	if (pdata)
		swfi_latency = pdata->swfi_latency + 1;

	if (vote)
		pm_qos_update_request(pdata->pm_qos_req_dma,
				swfi_latency);
	else
		pm_qos_update_request(pdata->pm_qos_req_dma,
				PM_QOS_DEFAULT_VALUE);
}

/* If USB Core is running its protocol engine based on PCLK,
 * PCLK must be running at >60Mhz for correct HSUSB operation and
 * USB core cannot tolerate frequency changes on PCLK. For such
 * USB cores, vote for maximum clk frequency on pclk source
 */
static void msm_otg_vote_for_pclk_source(struct msm_otg *dev, int vote)
{
	if (!pclk_requires_voting(&dev->otg))
		return;

	if (vote)
		clk_enable(dev->pclk_src);
	else
		clk_disable(dev->pclk_src);
}

/* Controller gives interrupt for every 1 mesc if 1MSIE is set in OTGSC.
 * This interrupt can be used as a timer source and OTG timers can be
 * implemented. But hrtimers on MSM hardware can give atleast 1/32 KHZ
 * precision. This precision is more than enough for OTG timers.
 */
static enum hrtimer_restart msm_otg_timer_func(struct hrtimer *_timer)
{
	unsigned long	flags;
	struct msm_otg *dev = container_of(_timer, struct msm_otg, timer);

	if (dev->active_tmout == SHVBUS_TMR_DISCON_USB_WAIT) {
		spin_lock_irqsave(&dev->lock, flags);
		dev->inputs = SHVBUS_EVT_DISCON_USB_TO;
		spin_unlock_irqrestore(&dev->lock, flags);
		queue_work(dev->wq, &dev->sm_work);
	}

	pr_debug("expired %s timer\n", timer_string(dev->active_tmout));
	return HRTIMER_NORESTART;
}

static void msm_otg_del_timer(struct msm_otg *dev)
{
	int bit = dev->active_tmout;

	pr_debug("deleting %s timer. remaining %lld msec \n", timer_string(bit),
			div_s64(ktime_to_us(hrtimer_get_remaining(&dev->timer)),
					1000));
	hrtimer_cancel(&dev->timer);
	clear_bit(bit, &dev->tmouts);
	dev->active_tmout = 0;
}

static void msm_otg_start_timer(struct msm_otg *dev, int time, int bit)
{
	clear_bit(bit, &dev->tmouts);
	dev->active_tmout = bit;
	pr_debug("starting %s timer\n", timer_string(bit));
	hrtimer_start(&dev->timer,
			ktime_set(time / 1000, (time % 1000) * 1000000),
			HRTIMER_MODE_REL);
}

/* No two otg timers run in parallel. So one hrtimer is sufficient */
static void msm_otg_init_timer(struct msm_otg *dev)
{
	hrtimer_init(&dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->timer.function = msm_otg_timer_func;
}

#ifndef CONFIG_USB_SWIC
/* You must queue other work together to call wake_unlock() when queueing 
 * msm_otg_del_timer_w().
 */
static void msm_otg_del_timer_w(struct work_struct *w)
{
	struct msm_otg *dev = container_of(w, struct msm_otg, del_timer_work);
	
	msm_otg_del_timer(dev);
	
	return;
}
#endif /* CONFIG_USB_SWIC */

static const char *event_string(unsigned long event)
{
	switch (event) {
	case SHVBUS_EVT_CONNECT:		return "CONNECT";
	case SHVBUS_EVT_DISCONNECT:		return "DISCONNECT";
	case SHVBUS_EVT_CONNECT_AC:		return "CONNECT_AC";
	case SHVBUS_EVT_CONNECT_USB:		return "CONNECT_USB";
	case SHVBUS_EVT_DISCON_USB_TO:		return "DISCON_USB_TO";
	default:				return "UNDEFINED";
	}
}

static int msm_otg_shvbus_set_power(struct otg_transceiver *xceiv, unsigned mA)
{
	struct msm_otg		*dev = container_of(xceiv, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = dev->pdata;

	if ((shvbus_charge_val == mA) && !shvbus_force_charge_flg)
		return 0;

	pr_debug("%s: Charger set power=%d\n", __func__, mA);

	if (pdata->chg_vbus_draw){
		shvbus_charge_val = mA;
		pdata->chg_vbus_draw(mA);
	}
	shvbus_force_charge_flg = 0;
	
	return 0;
}


static int msm_otg_set_power(struct otg_transceiver *xceiv, unsigned mA)
{
	struct msm_otg		*dev = container_of(xceiv, struct msm_otg, otg);
	enum usb_shvbus_state	state;
	unsigned long		flags;
	int			ret = 0;

	spin_lock_irqsave(&dev->lock, flags);
	state = dev->otg.state;
	spin_unlock_irqrestore(&dev->lock, flags);

	pr_debug("%s: Charger set power=%d, state=%d\n",
		__func__, mA, state);

	switch (state) {
	case SHVBUS_STS_INIT:
		break;
	case SHVBUS_STS_DISCONNECT:
		if (mA != 0)
			ret = -1;
		break;
	case SHVBUS_STS_CONNECT_AC:
		ret = -1;
		break;
	case SHVBUS_STS_CONNECT_USB:
		msm_otg_shvbus_set_power(xceiv, mA);
		break;
	case SHVBUS_STS_DISCON_USB_WAIT:
		shvbus_charge_wait_val = mA;
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static int msm_otg_set_connect(struct otg_transceiver *xceiv)
{
	struct msm_otg		*dev = container_of(xceiv, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = dev->pdata;
	enum chg_type 		new_chg = atomic_read(&dev->chg_type);
	
	pr_debug("%s: Charger connect=%d\n", __func__, (int)new_chg);
	
	/* Call chg_connected only if the charger has changed */
	if (pdata->chg_connected)
		pdata->chg_connected(new_chg);
	
	return 0;
}

static int msm_otg_set_clk(struct otg_transceiver *xceiv, int on)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	if (on)
		/* enable clocks */
		clk_enable(dev->hs_clk);
	else
		clk_disable(dev->hs_clk);

	return 0;
}

#ifndef CONFIG_USB_SWIC
/* Use spin_lock if use this function. */
static int msm_otg_chg_legacy_detect(struct msm_otg *dev)
{
	unsigned int chg_val;
	unsigned int chg_det;
	unsigned int fctl_val;
	int status;
	
	fctl_val = ulpi_read(dev, ULPI_FUNCTION_CONTROL_REG);
	
	if( (fctl_val & 0x18) != 0x08 ) {
		/* if not non-driving mode,then change non-driving mode */
		ulpi_write(dev, (fctl_val & ~0x18 ) | 0x08 , ULPI_FUNCTION_CONTROL_REG);
	}
	
	chg_val = ulpi_read(dev, ULPI_CHG_DETECT_REG);
	chg_val &= ~0x0f;
	ulpi_write(dev, chg_val | 0x07 , ULPI_CHG_DETECT_REG);
	ulpi_write(dev, chg_val | 0x05 , ULPI_CHG_DETECT_REG);
	udelay(20);	/* wait */
	ulpi_write(dev, chg_val | 0x04 , ULPI_CHG_DETECT_REG);
	udelay(20);	/* wait */
	chg_det = ulpi_read(dev, ULPI_CHG_DETECT_REG);
	ulpi_write(dev, chg_val | 0x0f , ULPI_CHG_DETECT_REG);
	
	if( (fctl_val & 0x18) != 0x08 ) {
		ulpi_write(dev, fctl_val , ULPI_FUNCTION_CONTROL_REG);
	}
	
	if( (chg_det != 0xffffffff) && ((chg_det & 0x10) != 0) ) {
		status = SHVBUS_EVT_CONNECT_AC;
	}
	else {
		status = SHVBUS_EVT_CONNECT_USB;
	}
	
	pr_debug("%s: evt = %s\n",__func__, event_string(status));
	
	return status;
}
#endif /* CONFIG_USB_SWIC */

static int get_cable_status(struct msm_otg *dev)
{
	int status = SHVBUS_EVT_DISCONNECT;
#ifdef CONFIG_USB_SWIC
	u8 device  = 0;
	shswic_result_t swic_ret;
#else /* CONFIG_USB_SWIC */
	unsigned long flags;
	unsigned long vbus;
#endif /* CONFIG_USB_SWIC */
	
	if (!dev)
		return status;
	
#ifdef CONFIG_USB_SWIC
	swic_ret = shswic_get_usb_port_status(&device);
	if (swic_ret) {
		pr_err("%s: shswic_get_usb_port_status err\n"
			, __func__);
		return status;
	}
		
	switch(device) {
	case SHSWIC_ID_USB_CABLE:
		status = SHVBUS_EVT_CONNECT_USB;
		break;
	case SHSWIC_ID_AC_ADAPTER:
		status = SHVBUS_EVT_CONNECT_AC;
		break;
	case SHSWIC_ID_NONE:
		status = SHVBUS_EVT_DISCONNECT;
		break;
	default:
		/* other cable connect */
		break;
	}
#else /* CONFIG_USB_SWIC */
	spin_lock_irqsave(&dev->lock, flags);
	vbus = OTGSC_BSV & readl(USB_OTGSC);
	if (vbus) {
		status = msm_otg_chg_legacy_detect(dev);
	}
	else {
		status = SHVBUS_EVT_DISCONNECT;
	}
	spin_unlock_irqrestore(&dev->lock, flags);
#endif /* CONFIG_USB_SWIC */
	
	pr_debug("%s: evt = %s\n",__func__, event_string(status));
	
	return status;
}

static void msm_otg_chk_cable_status_w(struct work_struct *w)
{
	struct msm_otg *dev = container_of(w, struct msm_otg, chk_cable_status_work.work);
	
	if (atomic_read(&dev->in_lpm)){
		msm_otg_set_suspend(&dev->otg, 0);
		cancel_delayed_work(&dev->chk_cable_status_work);
		queue_delayed_work(dev->wq, &dev->chk_cable_status_work,
					 msecs_to_jiffies(500));
		return;
	}
	
	dev->inputs = get_cable_status(dev);
	
	queue_work(dev->wq, &dev->sm_work);
	
	return;
}

static void msm_otg_start_peripheral(struct otg_transceiver *xceiv, int on)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = dev->pdata;

	if (!xceiv->gadget)
		return;

	if (on) {
		if (pdata->setup_gpio)
			pdata->setup_gpio(USB_SWITCH_PERIPHERAL);
		/* vote for minimum dma_latency to prevent idle
		 * power collapse(pc) while running in peripheral mode.
		 */
		otg_pm_qos_update_latency(dev, 1);

		/* increment the clk reference count so that
		 * it would be still on when disabled from
		 * low power mode routine
		 */
		if (dev->pdata->pclk_required_during_lpm)
			clk_enable(dev->hs_pclk);

		usb_gadget_vbus_connect(xceiv->gadget);
	} else {
		atomic_set(&dev->chg_type, USB_CHG_TYPE__INVALID);
		usb_gadget_vbus_disconnect(xceiv->gadget);

		/* decrement the clk reference count so that
		 * it would be off when disabled from
		 * low power mode routine
		 */
		if (dev->pdata->pclk_required_during_lpm)
			clk_disable(dev->hs_pclk);

		otg_pm_qos_update_latency(dev, 0);
		if (pdata->setup_gpio)
			pdata->setup_gpio(USB_SWITCH_DISABLE);
	}
}


static int msm_otg_suspend(struct msm_otg *dev)
{
	unsigned long timeout;
	unsigned ret;
	enum chg_type chg_type = atomic_read(&dev->chg_type);
	unsigned long flags;

	disable_irq(dev->irq);
	if (atomic_read(&dev->in_lpm))
		goto out;
	ulpi_read(dev, 0x14);/* clear PHY interrupt latch register */

	/*
	 * Turn on PHY comparators if,
	 * 1. USB wall charger is connected (bus suspend is not supported)
	 * 2. Host bus suspend
	 * 3. host is supported, but, id is not routed to pmic
	 * 4. peripheral is supported, but, vbus is not routed to pmic
	 */
	if ((dev->otg.gadget && chg_type == USB_CHG_TYPE__WALLCHARGER) ||
		(dev->otg.gadget && !dev->pmic_vbus_notif_supp)) {
		ulpi_write(dev, 0x01, 0x30);
	}

	ulpi_write(dev, 0x08, 0x09);/* turn off PLL on integrated phy */

	timeout = jiffies + msecs_to_jiffies(500);
	disable_phy_clk();
	while (!is_phy_clk_disabled()) {
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Unable to suspend phy\n", __func__);
			/*
			 * Start otg state machine in default state upon
			 * phy suspend failure*/
			spin_lock_irqsave(&dev->lock, flags);
			dev->otg.state = SHVBUS_STS_INIT;
			spin_unlock_irqrestore(&dev->lock, flags);
			queue_work(dev->wq, &dev->sm_work);
			goto out;
		}
		msleep(1);
		/* check if there are any pending interrupts*/
		if (((readl(USB_OTGSC) & OTGSC_INTR_MASK) >> 8) &
				readl(USB_OTGSC)) {
			goto out;
		}
	}

	writel(readl(USB_USBCMD) | ASYNC_INTR_CTRL | ULPI_STP_CTRL, USB_USBCMD);
	/* Ensure that above operation is completed before turning off clocks */
	dsb();

	if (dev->hs_pclk)
		clk_disable(dev->hs_pclk);
	if (dev->hs_cclk)
		clk_disable(dev->hs_cclk);
	/* usb phy no more require TCXO clock, hence vote for TCXO disable*/
	ret = msm_xo_mode_vote(dev->xo_handle, MSM_XO_MODE_OFF);
	if (ret)
		pr_err("%s failed to devote for"
			"TCXO D1 buffer%d\n", __func__, ret);

	if (device_may_wakeup(dev->otg.dev)) {
		enable_irq_wake(dev->irq);
		if (dev->vbus_on_irq)
			enable_irq_wake(dev->vbus_on_irq);
	}

	msm_otg_vote_for_pclk_source(dev, 0);

	atomic_set(&dev->in_lpm, 1);

	if (dev->pmic_vbus_notif_supp) {
		pr_debug("phy can power collapse: (%d)\n",
			can_phy_power_collapse(dev));
		if (can_phy_power_collapse(dev) && dev->pdata->ldo_enable) {
			pr_debug("disabling the regulators\n");
			dev->pdata->ldo_enable(0);
		}
	}

	/* phy can interrupts when vddcx is at 0.75, so irrespective
	 * of pmic notification support, configure vddcx @0.75
	 */
	if (dev->pdata->config_vddcx)
		dev->pdata->config_vddcx(0);
	pr_info("%s: usb in low power mode\n", __func__);

out:
	enable_irq(dev->irq);

	return 0;
}

static int msm_otg_resume(struct msm_otg *dev)
{
	unsigned temp;
	unsigned ret;

	if (!atomic_read(&dev->in_lpm))
		return 0;
	/* vote for vddcx, as PHY cannot tolerate vddcx below 1.0V */
	if (dev->pdata->config_vddcx) {
		ret = dev->pdata->config_vddcx(1);
		if (ret) {
			pr_err("%s: unable to enable vddcx digital core:%d\n",
				__func__, ret);
		}
	}
	if (dev->pdata->ldo_set_voltage)
		dev->pdata->ldo_set_voltage(3400);

	/* Vote for TCXO when waking up the phy */
	ret = msm_xo_mode_vote(dev->xo_handle, MSM_XO_MODE_ON);
	if (ret)
		pr_err("%s failed to vote for"
			"TCXO D1 buffer%d\n", __func__, ret);

	msm_otg_vote_for_pclk_source(dev, 1);

	if (dev->hs_pclk)
		clk_enable(dev->hs_pclk);
	if (dev->hs_cclk)
		clk_enable(dev->hs_cclk);

	temp = readl(USB_USBCMD);
	temp &= ~ASYNC_INTR_CTRL;
	temp &= ~ULPI_STP_CTRL;
	writel(temp, USB_USBCMD);

	if (device_may_wakeup(dev->otg.dev)) {
		disable_irq_wake(dev->irq);
		if (dev->vbus_on_irq)
			disable_irq_wake(dev->vbus_on_irq);
	}

	atomic_set(&dev->in_lpm, 0);

	pr_info("%s: usb exited from low power mode\n", __func__);

	return 0;
}

static void msm_otg_get_resume(struct msm_otg *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_get_noresume(dev->otg.dev);
	pm_runtime_resume(dev->otg.dev);
#else
	msm_otg_resume(dev);
#endif
}

static void msm_otg_put_suspend(struct msm_otg *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put_sync(dev->otg.dev);
#else
	msm_otg_suspend(dev);
#endif
}

static void msm_otg_resume_w(struct work_struct *w)
{
	struct msm_otg	*dev = container_of(w, struct msm_otg, otg_resume_work);
	unsigned long timeout;

	msm_otg_get_resume(dev);

	if (!is_phy_clk_disabled())
		goto phy_resumed;

	timeout = jiffies + usecs_to_jiffies(100);
	enable_phy_clk();
	while (is_phy_clk_disabled()) {
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Unable to wakeup phy\n", __func__);
			/* Reset both phy and link */
			otg_reset(&dev->otg, 1);
			break;
		}
		udelay(10);
	}

phy_resumed:

	/* Enable irq which was disabled before scheduling this work.
	 * But don't release wake_lock, as we got async interrupt and
	 * there will be some work pending for OTG state machine.
	 */
	enable_irq(dev->irq);
}

static int msm_otg_set_suspend(struct otg_transceiver *xceiv, int suspend)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	enum usb_otg_state state;

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;


	pr_debug("suspend request in state: %s\n",
			state_string(state));

	if (!suspend) {
		unsigned long timeout;

		if (suspend == atomic_read(&dev->in_lpm))
			return 0;

		disable_irq(dev->irq);
		if (dev->pmic_vbus_notif_supp)
			if (can_phy_power_collapse(dev) &&
					dev->pdata->ldo_enable)
				dev->pdata->ldo_enable(1);

		msm_otg_get_resume(dev);

		if (!is_phy_clk_disabled())
			goto out;

		timeout = jiffies + usecs_to_jiffies(100);
		enable_phy_clk();
		while (is_phy_clk_disabled()) {
			if (time_after(jiffies, timeout)) {
				pr_err("%s: Unable to wakeup phy\n", __func__);
				/* Reset both phy and link */
				otg_reset(&dev->otg, 1);
				break;
			}
			udelay(10);
		}
out:
		enable_irq(dev->irq);

	}

	return 0;
}

static int msm_otg_set_peripheral(struct otg_transceiver *xceiv,
			struct usb_gadget *gadget)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);

	if (!dev || (dev != the_msm_otg))
		return -ENODEV;

	if (!gadget) {
		msm_otg_start_peripheral(xceiv, 0);
		dev->otg.gadget = 0;
		disable_sess_valid(dev);
		return 0;
	}
	dev->otg.gadget = gadget;
	pr_info("peripheral driver registered w/ tranceiver\n");

	wake_lock(&dev->wlock);
	otg_reset(&dev->otg, 1);
	queue_delayed_work(dev->wq, &dev->chk_cable_status_work, 0);
	return 0;
}


void msm_otg_set_id_state(int id)
{
	/* ignore id-notification because we use in only peripheral mode */
}

void msm_otg_set_vbus_state(int online)
{
#ifndef CONFIG_USB_SWIC
	struct msm_otg *dev = the_msm_otg;

	if (!atomic_read(&dev->in_lpm) || !online)
		return;
	
	wake_lock(&dev->wlock);
	cancel_delayed_work(&dev->chk_cable_status_work);
	queue_delayed_work(dev->wq, &dev->chk_cable_status_work, 0);
#endif /* CONFIG_USB_SWIC */

}

#ifndef CONFIG_USB_SWIC
static irqreturn_t msm_otg_irq(int irq, void *data)
{
	struct msm_otg *dev = data;
	u32 otgsc, sts, sts_mask;
	irqreturn_t ret = IRQ_HANDLED;
	unsigned long flags;
	int queue_work_ret = 0;
	enum usb_otg_state state;
	
	if (atomic_read(&dev->in_lpm)) {
		disable_irq_nosync(dev->irq);
		wake_lock(&dev->wlock);
		queue_work_ret = queue_work(dev->wq, &dev->otg_resume_work);
		if(!queue_work_ret)
			enable_irq(dev->irq);
		goto out;
	}

	/* Return immediately if instead of ID pin, USER controls mode switch */
	if (dev->pdata->otg_mode == OTG_USER_CONTROL)
		return IRQ_NONE;


	otgsc = readl(USB_OTGSC);
	sts = readl(USB_USBSTS);

	sts_mask = (otgsc & OTGSC_INTR_MASK) >> 8;

	if (!((otgsc & sts_mask) || (sts & STS_PCI))) {
		ret = IRQ_NONE;
		goto out;
	}

	pr_debug("otgsc = %x\n", otgsc);

	if (otgsc & OTGSC_BSVIS) {
		cancel_delayed_work(&dev->chk_cable_status_work);
		
		spin_lock_irqsave(&dev->lock, flags);
		state = dev->otg.state;
		spin_unlock_irqrestore(&dev->lock, flags);
		
		if (state == SHVBUS_STS_DISCON_USB_WAIT) {
			queue_work(dev->wq, &dev->del_timer_work);
		}
		
		writel(otgsc, USB_OTGSC);
		/* BSV interrupt comes when operating as an A-device
		 * (VBUS on/off).
		 * But, handle BSV when charger is removed from ACA in ID_A
		 */
		if (otgsc & OTGSC_BSV) {
			pr_debug("BSV set\n");
			wake_lock(&dev->wlock);
			queue_delayed_work(dev->wq, &dev->chk_cable_status_work,
						 msecs_to_jiffies(500));
		} else {
			pr_debug("BSV clear\n");
			spin_lock_irqsave(&dev->lock, flags);
			dev->inputs = SHVBUS_EVT_DISCONNECT;
			spin_unlock_irqrestore(&dev->lock, flags);
			wake_lock(&dev->wlock);
			queue_work(dev->wq, &dev->sm_work);
		}
	}
out:
	return ret;
}
#endif /* CONFIG_USB_SWIC */

#ifdef CONFIG_USB_SWIC
static void msm_otg_swic_irq(u8 device, void *data)
{
	struct msm_otg *dev;
	int work = 0;
	unsigned long flags;

	if(!data){
		pr_err("%s: data is NULL\n", __func__);
		return;
	}

	dev = data;

	spin_lock_irqsave(&dev->lock, flags);
	dev->inputs = 0;
	switch(device) {
	case SHSWIC_ID_USB_CABLE:
		pr_debug("%s: USB Cable Connect set\n", __func__);
		dev->inputs = SHVBUS_EVT_CONNECT_USB;
		work = 1;
		break;
	case SHSWIC_ID_AC_ADAPTER:
		pr_debug("%s: AC Cable Connect set\n", __func__);
		dev->inputs = SHVBUS_EVT_CONNECT_AC;
		work = 1;
		break;
	case SHSWIC_ID_NONE:
		pr_debug("%s: Cable Disconnect\n", __func__);
		dev->inputs = SHVBUS_EVT_DISCONNECT;
		work = 1;
		break;
	default:
		pr_err("%s: other swic_irq=%08x\n",
			__func__, (int)device);
		break;
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	
	if (work) {
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}

	return;
}
#endif /* CONFIG_USB_SWIC */

#define ULPI_VERIFY_MAX_LOOP_COUNT  5
#define PHY_CALIB_RETRY_COUNT 10
static void phy_clk_reset(struct msm_otg *dev)
{
	unsigned rc;
	enum clk_reset_action assert = CLK_RESET_ASSERT;

	if (dev->pdata->phy_reset_sig_inverted)
		assert = CLK_RESET_DEASSERT;

	rc = clk_reset(dev->phy_reset_clk, assert);
	if (rc) {
		pr_err("%s: phy clk assert failed\n", __func__);
		return;
	}

	msleep(1);

	rc = clk_reset(dev->phy_reset_clk, !assert);
	if (rc) {
		pr_err("%s: phy clk deassert failed\n", __func__);
		return;
	}

	msleep(1);
}

static unsigned ulpi_read_with_reset(struct msm_otg *dev, unsigned reg)
{
	int temp;
	unsigned res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_read(dev, reg);
		if (res != 0xffffffff)
			return res;

		phy_clk_reset(dev);
	}

	pr_err("%s: ulpi read failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

static int ulpi_write_with_reset(struct msm_otg *dev,
unsigned val, unsigned reg)
{
	int temp, res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = ulpi_write(dev, val, reg);
		if (!res)
			return 0;
		phy_clk_reset(dev);
	}
	pr_err("%s: ulpi write failed for %d times\n",
		__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

/* some of the older targets does not turn off the PLL
 * if onclock bit is set and clocksuspendM bit is on,
 * hence clear them too and initiate the suspend mode
 * by clearing SupendM bit.
 */
static inline int turn_off_phy_pll(struct msm_otg *dev)
{
	unsigned res;

	res = ulpi_read_with_reset(dev, ULPI_CONFIG_REG1);
	if (res == 0xffffffff)
		return -ETIMEDOUT;

	res = ulpi_write_with_reset(dev,
		res & ~(ULPI_ONCLOCK), ULPI_CONFIG_REG1);
	if (res)
		return -ETIMEDOUT;

	res = ulpi_write_with_reset(dev,
		ULPI_CLOCK_SUSPENDM, ULPI_IFC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	/*Clear SuspendM bit to initiate suspend mode */
	res = ulpi_write_with_reset(dev,
		ULPI_SUSPENDM, ULPI_FUNC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	return res;
}

static inline int check_phy_caliberation(struct msm_otg *dev)
{
	unsigned res;

	res = ulpi_read_with_reset(dev, ULPI_DEBUG);

	if (res == 0xffffffff)
		return -ETIMEDOUT;

	if (!(res & ULPI_CALIB_STS) && ULPI_CALIB_VAL(res))
		return 0;

	return -1;
}

static int msm_otg_phy_caliberate(struct msm_otg *dev)
{
	int i = 0;
	unsigned long res;

	do {
		res = turn_off_phy_pll(dev);
		if (res)
			return -ETIMEDOUT;

		/* bring phy out of suspend */
		phy_clk_reset(dev);

		res = check_phy_caliberation(dev);
		if (!res)
			return res;
		i++;

	} while (i < PHY_CALIB_RETRY_COUNT);

	return res;
}

static int msm_otg_phy_reset(struct msm_otg *dev)
{
	unsigned rc;
	unsigned temp;
	unsigned long timeout;

	rc = clk_reset(dev->hs_clk, CLK_RESET_ASSERT);
	if (rc) {
		pr_err("%s: usb hs clk assert failed\n", __func__);
		return -1;
	}

	phy_clk_reset(dev);

	rc = clk_reset(dev->hs_clk, CLK_RESET_DEASSERT);
	if (rc) {
		pr_err("%s: usb hs clk deassert failed\n", __func__);
		return -1;
	}

	/* select ULPI phy */
	temp = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

	rc = msm_otg_phy_caliberate(dev);
	if (rc)
		return rc;

	/* TBD: There are two link resets. One is below and other one
	 * is done immediately after this function. See if we can
	 * eliminate one of these.
	 */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	do {
		if (time_after(jiffies, timeout)) {
			pr_err("msm_otg: usb link reset timeout\n");
			break;
		}
		msleep(1);
	} while (readl(USB_USBCMD) & USBCMD_RESET);

	if (readl(USB_USBCMD) & USBCMD_RESET) {
		pr_err("%s: usb core reset failed\n", __func__);
		return -1;
	}

	return 0;
}

static void otg_reset(struct otg_transceiver *xceiv, int phy_reset)
{
	struct msm_otg *dev = container_of(xceiv, struct msm_otg, otg);
	unsigned long timeout;
#ifndef CONFIG_USB_SWIC
	unsigned long vbus;
	unsigned long flags;
	enum usb_shvbus_state state;
#endif /* CONFIG_USB_SWIC */

	clk_enable(dev->hs_clk);

	if (!phy_reset)
		goto reset_link;

	if (dev->pdata->phy_reset)
		dev->pdata->phy_reset(dev->regs);
	else
		msm_otg_phy_reset(dev);

	/*disable all phy interrupts*/
	ulpi_write(dev, 0xFF, 0x0F);
	ulpi_write(dev, 0xFF, 0x12);
	msleep(100);

reset_link:
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	do {
		if (time_after(jiffies, timeout)) {
			pr_err("msm_otg: usb link reset timeout\n");
			break;
		}
		msleep(1);
	} while (readl(USB_USBCMD) & USBCMD_RESET);

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	set_pre_emphasis_level(dev);
	set_cdr_auto_reset(dev);
	set_driver_amplitude(dev);
	set_se1_gating(dev);

#ifdef CONFIG_USB_MSM_OTG_FS_RISEFALL_TIME_PLUS_30
	set_fs_risefall_time(dev);
#endif /* CONFIG_USB_MSM_OTG_FS_RISEFALL_TIME_PLUS_30 */

	writel(0x0, USB_AHB_BURST);
	writel(0x00, USB_AHB_MODE);
	/* Ensure that RESET operation is completed before turning off clock */
	dsb();

	clk_disable(dev->hs_clk);

	writel(USBMODE_SDIS | USBMODE_DEVICE, USB_USBMODE);

#ifndef CONFIG_USB_SWIC
	if (dev->otg.gadget){
		enable_sess_valid(dev);
		
		if(shvbus_usb_force_disconnect == 1)
			return;
		
		spin_lock_irqsave(&dev->lock, flags);
		state = dev->otg.state;
		spin_unlock_irqrestore(&dev->lock, flags);
		vbus = OTGSC_BSV & readl(USB_OTGSC);
		if (vbus && ((state == SHVBUS_STS_DISCONNECT) ||
				(state == SHVBUS_STS_DISCON_USB_WAIT))) {
			msm_otg_del_timer(dev);
			cancel_delayed_work(&dev->chk_cable_status_work);
			wake_lock(&dev->wlock);
			queue_delayed_work(dev->wq, &dev->chk_cable_status_work,
						 msecs_to_jiffies(500));
		}
		else if (!vbus && ((state == SHVBUS_STS_CONNECT_USB) || 
				(state == SHVBUS_STS_CONNECT_AC))) {
			cancel_delayed_work(&dev->chk_cable_status_work);
			wake_lock(&dev->wlock);
			spin_lock_irqsave(&dev->lock, flags);
			dev->inputs = SHVBUS_EVT_DISCONNECT;
			spin_unlock_irqrestore(&dev->lock, flags);
			queue_work(dev->wq, &dev->sm_work);
		}
	}
#endif /* CONFIG_USB_SWIC */
}

static void msm_otg_sm_work_set_status(struct msm_otg *dev,
					enum usb_shvbus_state state)
{
	spin_lock_irq(&dev->lock);
	dev->otg.state = state;
	spin_unlock_irq(&dev->lock);
	
	return;
}

static void msm_otg_sm_work_stop_connect(struct msm_otg *dev)
{
	/* Reset */
	otg_reset(&dev->otg, 1);
	/* suspend */
	msm_otg_put_suspend(dev);
	if (dev->pdata->ldo_set_voltage)
		dev->pdata->ldo_set_voltage(3075);
	return;
}

static void msm_otg_sm_work_start_ac(struct msm_otg *dev)
{
	/* reset */
	otg_reset(&dev->otg, 1);
	/* suspend */
	msm_otg_put_suspend(dev);
	/* Allow idle power collapse */
	otg_pm_qos_update_latency(dev, 0);
	return;
}

static void msm_otg_sm_work_set_chg_type(struct msm_otg *dev,
					enum chg_type new, int force)
{
	enum chg_type old;
	
	old = (enum chg_type)atomic_read(&dev->chg_type);
	if ((old == new) && !force) {
		pr_err("%s: charge type: %#08x, foece=%d\n"
			, __func__, (int)new, force);
		return;
	}
	
	switch(new) {
	case USB_CHG_TYPE__INVALID:
		atomic_set(&dev->chg_type, USB_CHG_TYPE__INVALID);
		msm_otg_shvbus_set_power(&dev->otg, 0);
		msm_otg_set_connect(&dev->otg);
		if (old == USB_CHG_TYPE__SDP)
			msm_otg_start_peripheral(&dev->otg, 0);
		break;
	case USB_CHG_TYPE__WALLCHARGER:
		atomic_set(&dev->chg_type, USB_CHG_TYPE__WALLCHARGER);
		msm_otg_set_connect(&dev->otg);
		msm_otg_shvbus_set_power(&dev->otg, USB_IDCHG_MAX);
		break;
	case USB_CHG_TYPE__SDP:
		atomic_set(&dev->chg_type, USB_CHG_TYPE__SDP);
		msm_otg_set_connect(&dev->otg);
		msm_otg_start_peripheral(&dev->otg, 1);
		break;
	default:
		pr_err("%s: Undefined charge type: %#08x\n", __func__, (int)new);
		break;
	}
	
	return;
}


static int msm_otg_sm_work_sts_init(struct msm_otg *dev, unsigned long inputs)
{
	int need_reset = 0;
	
	switch(inputs){
	case SHVBUS_EVT_DISCONNECT:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_DISCONNECT);
		shvbus_force_charge_flg = 1;
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 1);
		msm_otg_sm_work_stop_connect(dev);
		break;
	case SHVBUS_EVT_CONNECT_AC:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_AC);
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		shvbus_force_charge_flg = 1;
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__WALLCHARGER, 1);
		msm_otg_sm_work_start_ac(dev);
		break;
	case SHVBUS_EVT_CONNECT_USB:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_USB);
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		shvbus_force_charge_flg = 1;
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__SDP, 1);
		break;
	case SHVBUS_EVT_DISCON_USB_TO:
		break;
	default:
		pr_err("%s: Event Undefined\n", __func__);
		break;
	}
	
	return need_reset;
}

static int msm_otg_sm_work_sts_disconnect(struct msm_otg *dev,
					unsigned long inputs)
{
	int need_reset = 0;
	
	if(shvbus_usb_force_disconnect){
		/* already disconnect status */
		return need_reset;
	}
	
	switch(inputs){
	case SHVBUS_EVT_DISCONNECT:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_INIT);
		need_reset = 1;
		break;
	case SHVBUS_EVT_CONNECT_AC:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_AC);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__WALLCHARGER, 0);
		msm_otg_sm_work_start_ac(dev);
		break;
	case SHVBUS_EVT_CONNECT_USB:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_USB);
		shvbus_force_charge_flg = 1;
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__SDP, 0);
		break;
	/* case SHVBUS_EVT_DISCON_USB_TO: */
	default:
		/* suspend */
		msm_otg_put_suspend(dev);
		pr_err("%s: event error: %#08x\n", __func__, (int)inputs);
		break;
	}
	
	return need_reset;
}

static int msm_otg_sm_work_sts_connect_ac(struct msm_otg *dev,
					unsigned long inputs)
{
	int need_reset = 0;
	
	switch(inputs){
	case SHVBUS_EVT_DISCONNECT:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_DISCONNECT);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		break;
	case SHVBUS_EVT_CONNECT_AC:
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__WALLCHARGER, 0);
		msm_otg_sm_work_start_ac(dev);
		break;
	case SHVBUS_EVT_CONNECT_USB:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_USB);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		shvbus_force_charge_flg = 1;
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__SDP, 0);
		break;
	/* case SHVBUS_EVT_DISCON_USB_TO: */
	default:
		/* suspend */
		msm_otg_put_suspend(dev);
		pr_err("%s: event error: %#08x\n", __func__, (int)inputs);
			break;
	}
	
	return need_reset;
}

static int msm_otg_sm_work_sts_connect_usb(struct msm_otg *dev,
					unsigned long inputs)
{
	int need_reset = 0;
	
	switch(inputs){
	case SHVBUS_EVT_DISCONNECT:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_DISCON_USB_WAIT);
		/* To Charger */
		msm_otg_shvbus_set_power(&dev->otg, 0);
		/* Start Timer */
		msm_otg_start_timer(dev, SHVBUS_TMRVAL_DISCON_USB_WAIT,
					SHVBUS_TMR_DISCON_USB_WAIT);
		/* Pullup OFF */
		usb_gadget_disconnect_internal(dev->otg.gadget);
		break;
	case SHVBUS_EVT_CONNECT_AC:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_AC);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__WALLCHARGER, 0);
		msm_otg_sm_work_start_ac(dev);
		break;
	case SHVBUS_EVT_CONNECT_USB:
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		shvbus_force_charge_flg = 1;
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__SDP, 0);
		break;
	case SHVBUS_EVT_DISCON_USB_TO:
		break;
	default:
		pr_err("%s: Event Undefined\n", __func__);
		break;
	}
	
	return need_reset;
}


static int msm_otg_sm_work_sts_discon_usb_wait(struct msm_otg *dev,
						unsigned long inputs)
{
	int need_reset = 0;
	
	switch(inputs){
	case SHVBUS_EVT_DISCONNECT:
		if(shvbus_usb_force_disconnect){
			/* Status changes from DISCON_USB_WAIT to DISCONNECT
			 *  to wait "SHVBUS_EVT_DISCON_USB_TO"
			 */
			break;
		}
		
#ifdef CONFIG_USB_SWIC
		/* Delete Timer */
		msm_otg_del_timer(dev);
#endif /* CONFIG_USB_SWIC */
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_INIT);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		/* Clear chage_wait */
		shvbus_charge_wait_val = 0;
		
		need_reset = 1;
		break;
	case SHVBUS_EVT_CONNECT_AC:
#ifdef CONFIG_USB_SWIC
		/* Delete Timer */
		msm_otg_del_timer(dev);
#endif /* CONFIG_USB_SWIC */
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_AC);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		/* Clear chage_wait */
		shvbus_charge_wait_val = 0;
		
		/* resume */
		msm_otg_set_suspend(&dev->otg, 0);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__WALLCHARGER, 0);
		msm_otg_sm_work_start_ac(dev);
		break;
	case SHVBUS_EVT_CONNECT_USB:
#ifdef CONFIG_USB_SWIC
		/* Delete Timer */
		msm_otg_del_timer(dev);
#endif /* CONFIG_USB_SWIC */
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_CONNECT_USB);
		/* To Charger */
		if (shvbus_charge_wait_val)
			msm_otg_shvbus_set_power(&dev->otg, shvbus_charge_wait_val);
		shvbus_charge_wait_val = 0;
		/* Reset */
		usb_gadget_status_reset(dev->otg.gadget);
		break;
	case SHVBUS_EVT_DISCON_USB_TO:
		msm_otg_sm_work_set_status(dev, SHVBUS_STS_DISCONNECT);
		msm_otg_sm_work_set_chg_type(dev, USB_CHG_TYPE__INVALID, 0);
		msm_otg_sm_work_stop_connect(dev);
		/* Clear chage_wait */
		shvbus_charge_wait_val = 0;
		break;
	default:
		pr_err("%s: Event Undefined\n", __func__);
		break;
	}
	
	return need_reset;
}

static void msm_otg_sm_work(struct work_struct *w)
{
	struct msm_otg	*dev = container_of(w, struct msm_otg, sm_work);
	enum usb_shvbus_state state;
	unsigned long inputs;
	int reset_sm = 0;
	
	/* resume */
	if (atomic_read(&dev->in_lpm)){
		msm_otg_set_suspend(&dev->otg, 0);
	}
	
	/* state */
	spin_lock_irq(&dev->lock);
	state = dev->otg.state;
	inputs = dev->inputs;
	dev->inputs = 0;
	spin_unlock_irq(&dev->lock);
	
	pr_info("%s: state: %#08x, event: %#08x\n", __func__
		, (int)state, (int)(inputs));
	
	switch (state) {
	case SHVBUS_STS_INIT:
		reset_sm = msm_otg_sm_work_sts_init(dev, inputs);
		break;
	case SHVBUS_STS_DISCONNECT:
		reset_sm = msm_otg_sm_work_sts_disconnect(dev, inputs);
		break;
	case SHVBUS_STS_CONNECT_AC:
		reset_sm = msm_otg_sm_work_sts_connect_ac(dev, inputs);
		break;
	case SHVBUS_STS_CONNECT_USB:
		reset_sm = msm_otg_sm_work_sts_connect_usb(dev, inputs);
		break;
	case SHVBUS_STS_DISCON_USB_WAIT:
		reset_sm = msm_otg_sm_work_sts_discon_usb_wait(dev, inputs);
		break;
	default:
		pr_err("%s: State Undefined\n", __func__);
		break;
	}

	if (reset_sm){
		cancel_delayed_work(&dev->chk_cable_status_work);
		queue_delayed_work(dev->wq, &dev->chk_cable_status_work, 
					D_WAIT_TIME_FOR_STABILITY);
	}

	/* IRQ/sysfs may queue work. Check work_pending. otherwise
	 * we might endup releasing wakelock after it is acquired
	 * in IRQ/sysfs.
	 */
	if (!work_pending(&dev->sm_work) && !hrtimer_active(&dev->timer) &&
			!work_pending(&dev->otg_resume_work) &&
			!work_pending(&dev->chk_cable_status_work.work))
		wake_unlock(&dev->wlock);
}

#ifdef CONFIG_DEBUG_FS
static int otg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t otg_mode_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct msm_otg *dev = file->private_data;
	int ret = count;
	int work = 0;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	if ((!memcmp(buf, "DISCONNECT", count - 1)) &&
		(count - 1 == sizeof("DISCONNECT"))) {
		dev->inputs = SHVBUS_EVT_DISCONNECT;
		work = 1;
	} else if ((!memcmp(buf, "CONNECT", count - 1)) &&
		(count - 1 == sizeof("CONNECT"))) {
		dev->inputs = SHVBUS_EVT_CONNECT;
		work = 1;
	} else if ((!memcmp(buf, "CONNECT_AC", count - 1)) &&
		(count - 1 == sizeof("CONNECT_AC"))) {
		dev->inputs = SHVBUS_EVT_CONNECT_AC;
		work = 1;
	} else if ((!memcmp(buf, "CONNECT_USB", count - 1)) &&
		(count - 1 == sizeof("CONNECT_USB"))) {
		dev->inputs = SHVBUS_EVT_CONNECT_USB;
		work = 1;
	} else if ((!memcmp(buf, "DISCON_USB_TO", count - 1)) &&
		(count - 1 == sizeof("DISCON_USB_TO"))) {
		dev->inputs = SHVBUS_EVT_DISCON_USB_TO;
		work = 1;
	} else {
		pr_info("%s: unknown mode specified\n", __func__);
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	if (work) {
		wake_lock(&dev->wlock);
		queue_work(dev->wq, &dev->sm_work);
	}

	return ret;
}
const struct file_operations otgfs_fops = {
	.open	= otg_open,
	.write	= otg_mode_write,
};

struct dentry *otg_debug_root;
struct dentry *otg_debug_mode;
#endif

static int otg_debugfs_init(struct msm_otg *dev)
{
#ifdef CONFIG_DEBUG_FS
	otg_debug_root = debugfs_create_dir("otg", NULL);
	if (!otg_debug_root)
		return -ENOENT;

	otg_debug_mode = debugfs_create_file("mode", 0222,
						otg_debug_root, dev,
						&otgfs_fops);
	if (!otg_debug_mode) {
		debugfs_remove(otg_debug_root);
		otg_debug_root = NULL;
		return -ENOENT;
	}
#endif
	return 0;
}

static void otg_debugfs_cleanup(void)
{
#ifdef CONFIG_DEBUG_FS
       debugfs_remove(otg_debug_mode);
       debugfs_remove(otg_debug_root);
#endif
}

static ssize_t
show_usb_force_disconnect(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	size_t val;

	val = sprintf(buf, "%d", (int)shvbus_usb_force_disconnect);

	return val;
}

static ssize_t
store_usb_force_disconnect(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct msm_otg *dev = the_msm_otg;
	size_t value = -EINVAL;
	unsigned long flags;
#ifdef CONFIG_USB_SWIC
	shswic_result_t swic_ret;
#else /* CONFIG_USB_SWIC */
	int ret = 0;
#endif /* CONFIG_USB_SWIC */

	if ((size == 0) || (size > 2) || (buf == NULL)){
		pr_err("%s: invalid parmeter\n", __func__);
		return -EINVAL;
	}

	if ((size == 2) && (*(buf+1) != 0x0a) && (*(buf+1) != 0x00))
		return -EINVAL;

	switch(*buf){
	case '0':
		if (shvbus_usb_force_disconnect == 1) {
#ifdef CONFIG_USB_SWIC
			if (dev->pdata->pmic_vbus_notif_init)
				dev->pmic_vbus_notif_supp = 1;
			if (dev->pdata->pmic_id_notif_init)
				dev->pmic_id_notif_supp = 1;
#else /* CONFIG_USB_SWIC */
			if (dev->pdata->pmic_vbus_notif_init) {
				ret = dev->pdata->pmic_vbus_notif_init
					(&msm_otg_set_vbus_state, 1);
				if (!ret)
					dev->pmic_vbus_notif_supp = 1;
			}
			
			if (dev->pdata->pmic_id_notif_init) {
				ret = dev->pdata->pmic_id_notif_init
						(&msm_otg_set_id_state, 1);
				if (!ret) {
					dev->pmic_id_notif_supp = 1;
				}
			}
#endif /* CONFIG_USB_SWIC */
			
			if (dev->pdata->pmic_vbus_irq)
				dev->vbus_on_irq = dev->pdata->pmic_vbus_irq;
			
#ifdef CONFIG_USB_SWIC
			swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, 
				(SHSWIC_ID_USB_CABLE | SHSWIC_ID_AC_ADAPTER), 
				(void*)msm_otg_swic_irq, dev);
			if (swic_ret) {
				pr_err("%s: shswic_detect_cb_regist err\n",
					__func__);
				value = -EINVAL;
				break;
			}
#else /* CONFIG_USB_SWIC */
			ret = request_irq(dev->irq, msm_otg_irq
					, IRQF_SHARED,"msm_otg", dev);
			if (ret) {
				pr_err("%s: request irq failed\n"
					, __func__);
				value = ret;
				break;
			}
#endif /* CONFIG_USB_SWIC */

			msm_otg_set_suspend(&dev->otg, 0);
			
			spin_lock_irqsave(&dev->lock, flags);
			dev->otg.state = SHVBUS_STS_INIT;
			spin_unlock_irqrestore(&dev->lock, flags);
			
			wake_lock(&dev->wlock);
			queue_delayed_work(dev->wq, &dev->chk_cable_status_work, 0);
		}
		shvbus_usb_force_disconnect = 0;
		value = size;
		break;
	case '1':
		if (shvbus_usb_force_disconnect == 0) {
#ifdef CONFIG_USB_SWIC
			swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE
						, 0, (void*)NULL, NULL);
			if (swic_ret) {
				pr_err("%s: shswic_detect_cb_regist err\n",
					__func__);
				value = -EINVAL;
				break;
			}
#else /* CONFIG_USB_SWIC */
			free_irq(dev->irq, dev);
#endif /* CONFIG_USB_SWIC */

			if (dev->pdata->pmic_vbus_irq)
				dev->vbus_on_irq = 0;
			
#ifndef CONFIG_USB_SWIC
			if (dev->pdata->pmic_id_notif_init) {
				ret = dev->pdata->pmic_id_notif_init
						(&msm_otg_set_id_state, 0);
			}
			
			if (dev->pdata->pmic_vbus_notif_init) {
				ret = dev->pdata->pmic_vbus_notif_init
					(&msm_otg_set_vbus_state, 0);
			}
#endif /* CONFIG_USB_SWIC */
			cancel_work_sync(&dev->otg_resume_work);
			cancel_delayed_work_sync(&dev->chk_cable_status_work);
			cancel_work_sync(&dev->sm_work);
			
			spin_lock_irqsave(&dev->lock, flags);
			dev->inputs = SHVBUS_EVT_DISCONNECT;
			spin_unlock_irqrestore(&dev->lock, flags);
			
			wake_lock(&dev->wlock);
			queue_work(dev->wq, &dev->sm_work);
		}
		shvbus_usb_force_disconnect = 1;
		value = size;
		break;
	default:
		value = -EINVAL;
		pr_err("%s: invalid parmeter = %d\n", __func__, (int)*buf);
		break;
	}
	
	return value;
}

static DEVICE_ATTR(usb_force_disconnect, 0664
		, show_usb_force_disconnect, store_usb_force_disconnect);

struct otg_io_access_ops msm_otg_io_ops = {
	.read = usb_ulpi_read,
	.write = usb_ulpi_write,
};

static int __init msm_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct msm_otg *dev;
#ifdef CONFIG_USB_SWIC
	shswic_result_t swic_ret;
#endif /* CONFIG_USB_SWIC */

	dev = kzalloc(sizeof(struct msm_otg), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	the_msm_otg = dev;
	dev->otg.dev = &pdev->dev;
	dev->pdata = pdev->dev.platform_data;

	if (!dev->pdata) {
		ret = -ENODEV;
		goto free_dev;
	}

#ifdef CONFIG_USB_MSM_OTG_FORCE_AMPLITUDE_75
	/* override */
	dev->pdata->drv_ampl = HS_DRV_AMPLITUDE_75_PERCENT;
#endif /* CONFIG_USB_MSM_OTG_FORCE_AMPLITUDE_75 */

	if (dev->pdata->rpc_connect) {
		ret = dev->pdata->rpc_connect(1);
		pr_debug("%s: rpc_connect(%d)\n", __func__, ret);
		if (ret) {
			pr_err("%s: rpc connect failed\n", __func__);
			ret = -ENODEV;
			goto free_dev;
		}
	}

#ifdef CONFIG_USB_SWIC
	pr_debug("%s: shvbus_swicdev=VALID \n",__func__);
#else /* CONFIG_USB_SWIC */
	pr_debug("%s: shvbus_swicdev=UNVALID\n",__func__);
#endif /* CONFIG_USB_SWIC */

	dev->hs_clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(dev->hs_clk)) {
		pr_err("%s: failed to get usb_hs_clk\n", __func__);
		ret = PTR_ERR(dev->hs_clk);
		goto rpc_fail;
	}
	clk_set_rate(dev->hs_clk, 60000000);

	/* pm qos request to prevent apps idle power collapse */
	dev->pdata->pm_qos_req_dma = pm_qos_add_request(PM_QOS_CPU_DMA_LATENCY,
					PM_QOS_DEFAULT_VALUE);

	/* If USB Core is running its protocol engine based on PCLK,
	 * PCLK must be running at >60Mhz for correct HSUSB operation and
	 * USB core cannot tolerate frequency changes on PCLK. For such
	 * USB cores, vote for maximum clk frequency on pclk source
	 */
	if (dev->pdata->pclk_src_name) {
		dev->pclk_src = clk_get(0, dev->pdata->pclk_src_name);
		if (IS_ERR(dev->pclk_src))
			goto put_hs_clk;
		clk_set_rate(dev->pclk_src, INT_MAX);
		msm_otg_vote_for_pclk_source(dev, 1);
	}

	if (!dev->pdata->pclk_is_hw_gated) {
		dev->hs_pclk = clk_get(&pdev->dev, "usb_hs_pclk");
		if (IS_ERR(dev->hs_pclk)) {
			pr_err("%s: failed to get usb_hs_pclk\n", __func__);
			ret = PTR_ERR(dev->hs_pclk);
			goto put_pclk_src;
		}
		clk_enable(dev->hs_pclk);
	}

	if (dev->pdata->core_clk) {
		dev->hs_cclk = clk_get(&pdev->dev, "usb_hs_core_clk");
		if (IS_ERR(dev->hs_cclk)) {
			pr_err("%s: failed to get usb_hs_core_clk\n", __func__);
			ret = PTR_ERR(dev->hs_cclk);
			goto put_hs_pclk;
		}
		clk_enable(dev->hs_cclk);
	}

	if (!dev->pdata->phy_reset) {
		dev->phy_reset_clk = clk_get(&pdev->dev, "usb_phy_clk");
		if (IS_ERR(dev->phy_reset_clk)) {
			pr_err("%s: failed to get usb_phy_clk\n", __func__);
			ret = PTR_ERR(dev->phy_reset_clk);
			goto put_hs_cclk;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("%s: failed to get platform resource mem\n", __func__);
		ret = -ENODEV;
		goto put_phy_clk;
	}

	dev->regs = ioremap(res->start, resource_size(res));
	if (!dev->regs) {
		pr_err("%s: ioremap failed\n", __func__);
		ret = -ENOMEM;
		goto put_phy_clk;
	}
	dev->irq = platform_get_irq(pdev, 0);
	if (!dev->irq) {
		pr_err("%s: platform_get_irq failed\n", __func__);
		ret = -ENODEV;
		goto free_regs;
	}
	dev->xo_handle = msm_xo_get(MSM_XO_TCXO_D1, "usb");
	if (IS_ERR(dev->xo_handle)) {
		pr_err(" %s not able to get the handle"
			"to vote for TCXO D1 buffer\n", __func__);
		ret = PTR_ERR(dev->xo_handle);
		goto free_regs;
	}

	ret = msm_xo_mode_vote(dev->xo_handle, MSM_XO_MODE_ON);
	if (ret) {
		pr_err("%s failed to vote for TCXO"
			"D1 buffer%d\n", __func__, ret);
		goto free_xo_handle;
	}


	msm_otg_init_timer(dev);
	INIT_WORK(&dev->sm_work, msm_otg_sm_work);
	INIT_WORK(&dev->otg_resume_work, msm_otg_resume_w);
#ifndef CONFIG_USB_SWIC
	INIT_WORK(&dev->del_timer_work, msm_otg_del_timer_w);
#endif /* CONFIG_USB_SWIC */
	INIT_DELAYED_WORK(&dev->chk_cable_status_work, msm_otg_chk_cable_status_w);
	spin_lock_init(&dev->lock);
	wake_lock_init(&dev->wlock, WAKE_LOCK_SUSPEND, "msm_otg");

	dev->wq = create_singlethread_workqueue("k_otg");
	if (!dev->wq) {
		ret = -ENOMEM;
		goto free_wlock;
	}

	if (dev->pdata->init_gpio) {
		ret = dev->pdata->init_gpio(1);
		if (ret) {
			pr_err("%s: gpio init failed with err:%d\n",
					__func__, ret);
			goto free_wq;
		}
	}
#ifdef CONFIG_USB_SWIC
	if (dev->pdata->pmic_vbus_notif_init)
		dev->pmic_vbus_notif_supp = 1;
	if (dev->pdata->pmic_id_notif_init)
		dev->pmic_id_notif_supp = 1;
#else /* CONFIG_USB_SWIC */
	/* To reduce phy power consumption and to avoid external LDO
	 * on the board, PMIC comparators can be used to detect VBUS
	 * session change.
	 */
	if (dev->pdata->pmic_vbus_notif_init) {
		ret = dev->pdata->pmic_vbus_notif_init
			(&msm_otg_set_vbus_state, 1);
		if (!ret) {
			dev->pmic_vbus_notif_supp = 1;
		} else if (ret != -ENOTSUPP) {
			pr_err("%s: pmic_vbus_notif_init() failed, err:%d\n",
					__func__, ret);
			goto free_gpio;
		}
	}

	if (dev->pdata->pmic_id_notif_init) {
		ret = dev->pdata->pmic_id_notif_init(&msm_otg_set_id_state, 1);
		if (!ret) {
			dev->pmic_id_notif_supp = 1;
		} else if (ret != -ENOTSUPP) {
			pr_err("%s: pmic_id_ notif_init failed err:%d",
					__func__, ret);
			goto free_pmic_vbus_notif;
		}
	}
#endif /* CONFIG_USB_SWIC */

	if (dev->pdata->pmic_vbus_irq)
		dev->vbus_on_irq = dev->pdata->pmic_vbus_irq;

	/* vote for vddcx, as PHY cannot tolerate vddcx below 1.0V */
	if (dev->pdata->init_vddcx) {
		ret = dev->pdata->init_vddcx(1);
		if (ret) {
			pr_err("%s: unable to enable vddcx digital core:%d\n",
				__func__, ret);
			goto free_pmic_id_notif;
		}
	}

	if (dev->pdata->ldo_init) {
		ret = dev->pdata->ldo_init(1);
		if (ret) {
			pr_err("%s: ldo_init failed with err:%d\n",
					__func__, ret);
			goto free_config_vddcx;
		}
	}

	if (dev->pdata->ldo_enable) {
		ret = dev->pdata->ldo_enable(1);
		if (ret) {
			pr_err("%s: ldo_enable failed with err:%d\n",
					__func__, ret);
			goto free_ldo_init;
		}
	}


	/* ACk all pending interrupts and clear interrupt enable registers */
	writel((readl(USB_OTGSC) & ~OTGSC_INTR_MASK), USB_OTGSC);
	writel(readl(USB_USBSTS), USB_USBSTS);
	writel(0, USB_USBINTR);
	/* Ensure that above STOREs are completed before enabling interrupts */
	dsb();

#ifdef CONFIG_USB_SWIC
	swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, 
				(SHSWIC_ID_USB_CABLE | SHSWIC_ID_AC_ADAPTER), 
				(void*)msm_otg_swic_irq, dev);
	if (swic_ret) {
		pr_err("%s: shswic_detect_cb_regist err\n",
			__func__);
		ret = -EINVAL;
		goto free_ldo_enable;
	}
#else /* CONFIG_USB_SWIC */
	ret = request_irq(dev->irq, msm_otg_irq, IRQF_SHARED,
					"msm_otg", dev);
	if (ret) {
		pr_err("%s: request irq failed\n", __func__);
		goto free_ldo_enable;
	}
#endif /* CONFIG_USB_SWIC */

	dev->otg.set_peripheral = msm_otg_set_peripheral;
	dev->otg.set_suspend = msm_otg_set_suspend;
	dev->otg.set_power = msm_otg_set_power;
	dev->set_clk = msm_otg_set_clk;
	dev->reset = otg_reset;
	dev->otg.io_ops = &msm_otg_io_ops;
	if (otg_set_transceiver(&dev->otg)) {
		WARN_ON(1);
		goto free_otg_irq;
	}

	atomic_set(&dev->chg_type, USB_CHG_TYPE__INVALID);
	if (dev->pdata->chg_init && dev->pdata->chg_init(1))
		pr_err("%s: chg_init failed\n", __func__);

	device_init_wakeup(&pdev->dev, 1);

	ret = pm_runtime_set_active(&pdev->dev);
	if (ret < 0)
		pr_err("%s: pm_runtime: Fail to set active\n", __func__);

	ret = 0;
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get(&pdev->dev);


	ret = otg_debugfs_init(dev);
	if (ret) {
		pr_err("%s: otg_debugfs_init failed\n", __func__);
		goto chg_deinit;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_usb_force_disconnect);
	if (ret < 0) {
		pr_err("%s: Failed to create the file entry\n", __func__);
		otg_debugfs_cleanup();
		goto chg_deinit;
	}


	return 0;

chg_deinit:
	if (dev->pdata->chg_init)
		dev->pdata->chg_init(0);
free_otg_irq:
#ifdef CONFIG_USB_SWIC
	swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, 
					0, (void*)NULL, NULL);
#else /* CONFIG_USB_SWIC */
	free_irq(dev->irq, dev);
#endif /* CONFIG_USB_SWIC */
free_ldo_enable:
	if (dev->pdata->ldo_enable)
		dev->pdata->ldo_enable(0);
	if (dev->pdata->setup_gpio)
		dev->pdata->setup_gpio(USB_SWITCH_DISABLE);
free_ldo_init:
	if (dev->pdata->ldo_init)
		dev->pdata->ldo_init(0);
free_config_vddcx:
	if (dev->pdata->init_vddcx)
		dev->pdata->init_vddcx(0);
free_pmic_id_notif:
#ifndef CONFIG_USB_SWIC
	if (dev->pdata->pmic_id_notif_init && dev->pmic_id_notif_supp)
		dev->pdata->pmic_id_notif_init(&msm_otg_set_id_state, 0);
free_pmic_vbus_notif:
	if (dev->pdata->pmic_vbus_notif_init && dev->pmic_vbus_notif_supp)
		dev->pdata->pmic_vbus_notif_init(&msm_otg_set_vbus_state, 0);
free_gpio:
#endif /* CONFIG_USB_SWIC */
	if (dev->pdata->init_gpio)
		dev->pdata->init_gpio(0);
free_wq:
	destroy_workqueue(dev->wq);
free_wlock:
	wake_lock_destroy(&dev->wlock);
free_xo_handle:
	msm_xo_put(dev->xo_handle);
free_regs:
	iounmap(dev->regs);
put_phy_clk:
	if (dev->phy_reset_clk)
		clk_put(dev->phy_reset_clk);
put_hs_cclk:
	if (dev->hs_cclk) {
		clk_disable(dev->hs_cclk);
		clk_put(dev->hs_cclk);
	}
put_hs_pclk:
	if (dev->hs_pclk) {
		clk_disable(dev->hs_pclk);
		clk_put(dev->hs_pclk);
	}
put_pclk_src:
	if (dev->pclk_src) {
		msm_otg_vote_for_pclk_source(dev, 0);
		clk_put(dev->pclk_src);
	}
put_hs_clk:
	if (dev->hs_clk)
		clk_put(dev->hs_clk);
rpc_fail:
	if (dev->pdata->rpc_connect)
		dev->pdata->rpc_connect(0);
free_dev:
	kfree(dev);
	return ret;
}

static int __exit msm_otg_remove(struct platform_device *pdev)
{
	struct msm_otg *dev = the_msm_otg;

	otg_debugfs_cleanup();
	device_remove_file(&pdev->dev, &dev_attr_usb_force_disconnect);
	destroy_workqueue(dev->wq);
	wake_lock_destroy(&dev->wlock);

	if (dev->pdata->setup_gpio)
		dev->pdata->setup_gpio(USB_SWITCH_DISABLE);

	if (dev->pdata->init_vddcx)
		dev->pdata->init_vddcx(0);
	if (dev->pdata->ldo_enable)
		dev->pdata->ldo_enable(0);

	if (dev->pdata->ldo_init)
		dev->pdata->ldo_init(0);

	if (dev->pmic_vbus_notif_supp)
		dev->pdata->pmic_vbus_notif_init(&msm_otg_set_vbus_state, 0);

	if (dev->pmic_id_notif_supp)
		dev->pdata->pmic_id_notif_init(&msm_otg_set_id_state, 0);

	if (dev->pdata->chg_init)
		dev->pdata->chg_init(0);
	free_irq(dev->irq, pdev);
	iounmap(dev->regs);
	if (dev->hs_cclk) {
		clk_disable(dev->hs_cclk);
		clk_put(dev->hs_cclk);
	}
	if (dev->hs_pclk) {
		clk_disable(dev->hs_pclk);
		clk_put(dev->hs_pclk);
	}
	if (dev->hs_clk)
		clk_put(dev->hs_clk);
	if (dev->phy_reset_clk)
		clk_put(dev->phy_reset_clk);
	if (dev->pdata->rpc_connect)
		dev->pdata->rpc_connect(0);
	msm_xo_put(dev->xo_handle);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	kfree(dev);
	pm_qos_remove_request(dev->pdata->pm_qos_req_dma);
	clk_put(dev->pclk_src);
	return 0;
}

static int msm_otg_runtime_suspend(struct device *dev)
{
	struct msm_otg *otg = the_msm_otg;

	dev_dbg(dev, "pm_runtime: suspending...\n");
	msm_otg_suspend(otg);
	return  0;
}

static int msm_otg_runtime_resume(struct device *dev)
{
	struct msm_otg *otg = the_msm_otg;

	dev_dbg(dev, "pm_runtime: resuming...\n");
	msm_otg_resume(otg);
	return  0;
}

static int msm_otg_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: idling...\n");
	return  0;
}

static struct dev_pm_ops msm_otg_dev_pm_ops = {
	.runtime_suspend = msm_otg_runtime_suspend,
	.runtime_resume = msm_otg_runtime_resume,
	.runtime_idle = msm_otg_runtime_idle,
};

static struct platform_driver msm_otg_driver = {
	.remove = __exit_p(msm_otg_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &msm_otg_dev_pm_ops,
	},
};

static int __init msm_otg_init(void)
{
	return platform_driver_probe(&msm_otg_driver, msm_otg_probe);
}

static void __exit msm_otg_exit(void)
{
	platform_driver_unregister(&msm_otg_driver);
}

module_init(msm_otg_init);
module_exit(msm_otg_exit);

MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM usb transceiver driver");
MODULE_VERSION("1.00");
