/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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
 * SHARP DISPLAY DRIVER FOR KERNEL
 */

#ifndef SHLCDC_KERN_H
#define SHLCDC_KERN_H

#ifdef CONFIG_SHLCDC_SUBDISPLAY
/*
 * MACROS
 */ 
#define SHLCDC_SUB_DISP_UPDATE_SIZE_MAX  288
#define INITCMDSIZE                       37
#define SHLCDC_SUB_DISP_SIZE_X            96
#define SHLCDC_SUB_DISP_SIZE_Y            24

#define SHLCDC_SPIRETRY_LIMIT              3

#define SHLCDC_GPIO_NUM_SPI_CS            44
#define SHLCDC_GPIO_NUM_SPI_CLOCK         45
#define SHLCDC_GPIO_NUM_SPI_MOSI          47
#define SHLCDC_GPIO_NUM_DATACOMMAND_LINE  56

#define SHLCDC_SET_CHIPSELECT(d) gpio_set_value(SHLCDC_GPIO_NUM_SPI_CS,d)
#define SHLCDC_SET_DCLINE(d) gpio_set_value(SHLCDC_GPIO_NUM_DATACOMMAND_LINE,d)

#define SHLCDC_ADDR_DATA_WRITE           0x08

#define SHLCDC_ADDR_SUB_DISP_ONOFF       0x02
#define SHLCDC_ADDR_DISP_STB             0x14
#define SHLCDC_DATA_SUB_DISP_OFF         0x00
#define SHLCDC_DATA_SUB_DISP_ON          0x01
#define SHLCDC_DATA_DISP_STB_OFF         0x00
#define SHLCDC_DATA_DISP_STB_ON          0x01


#define SHLCDC_ADDR_SET_POS_XSTART       0x34
#define SHLCDC_ADDR_SET_POS_XEND         0x35
#define SHLCDC_ADDR_SET_POS_YSTART       0x36
#define SHLCDC_ADDR_SET_POS_YEND         0x37

#define SHLCDC_DATA_DEFAULT_POS_XSTART   0x00
#define SHLCDC_DATA_DEFAULT_POS_XEND     0x0b
#define SHLCDC_DATA_DEFAULT_POS_YSTART   0x01
#define SHLCDC_DATA_DEFAULT_POS_YEND     0x18

#endif /* CONFIG_SHLCDC_SUBDISPLAY */

/*
 * TYPES
 */

enum {
    SHLCDC_RESULT_SUCCESS,
    SHLCDC_RESULT_FAILURE,
    SHLCDC_RESULT_FAILURE_I2C_TMO,
    NUM_SHLCDC_RESULT
};

enum {
    SHLCDC_EVENT_TYPE_IRRC,
    SHLCDC_EVENT_TYPE_SDHC,
    SHLCDC_EVENT_TYPE_GPIO,
    SHLCDC_EVENT_TYPE_PVSYNC,
    SHLCDC_EVENT_TYPE_I2C,
    SHLCDC_EVENT_TYPE_CAMVIEW,
    SHLCDC_EVENT_TYPE_SDDET_H,
    SHLCDC_EVENT_TYPE_SDDET_L,
    SHLCDC_EVENT_TYPE_CSTM,
    SHLCDC_EVENT_TYPE_CSI,
    SHLCDC_EVENT_TYPE_IRRC2,
    NUM_SHLCDC_EVENT_TYPE
};

#ifdef CONFIG_SHLCDC_ILLUMI
enum {
    SHLCDC_DEV_TYPE_HANDSET,
    SHLCDC_DEV_TYPE_TAKT,
    SHLCDC_DEV_TYPE_MDDI,
    SHLCDC_DEV_TYPE_BDIC,
    SHLCDC_DEV_TYPE_CAM,
    SHLCDC_DEV_TYPE_CAM_PREVIEW,
    SHLCDC_DEV_TYPE_SUBCAM,
    SHLCDC_DEV_TYPE_SD,
    SHLCDC_DEV_TYPE_IR,
    SHLCDC_DEV_TYPE_TP,
    SHLCDC_DEV_TYPE_DIAG,
    SHLCDC_DEV_TYPE_MID,
    SHLCDC_DEV_TYPE_HDMI_480P,
    SHLCDC_DEV_TYPE_HDMI_720P,
    SHLCDC_DEV_TYPE_HDMI_1080P,
    SHLCDC_DEV_TYPE_SASUKE,
    SHLCDC_DEV_TYPE_3DSWITCH,
    SHLCDC_DEV_TYPE_CAM_CSI,
    SHLCDC_DEV_TYPE_SUBCAM_CSI,
    SHLCDC_DEV_TYPE_ILM,
    NUM_SHLCDC_DEV_TYPE
};
#else /* CONFIG_SHLCDC_ILLUMI */
enum {
    SHLCDC_DEV_TYPE_HANDSET,
    SHLCDC_DEV_TYPE_TAKT,
    SHLCDC_DEV_TYPE_MDDI,
    SHLCDC_DEV_TYPE_BDIC,
    SHLCDC_DEV_TYPE_CAM,
    SHLCDC_DEV_TYPE_CAM_PREVIEW,
    SHLCDC_DEV_TYPE_SUBCAM,
    SHLCDC_DEV_TYPE_SD,
    SHLCDC_DEV_TYPE_IR,
    SHLCDC_DEV_TYPE_TP,
    SHLCDC_DEV_TYPE_DIAG,
    SHLCDC_DEV_TYPE_MID,
    SHLCDC_DEV_TYPE_HDMI_480P,
    SHLCDC_DEV_TYPE_HDMI_720P,
    SHLCDC_DEV_TYPE_HDMI_1080P,
    SHLCDC_DEV_TYPE_SASUKE,
    SHLCDC_DEV_TYPE_3DSWITCH,
    SHLCDC_DEV_TYPE_CAM_CSI,
    SHLCDC_DEV_TYPE_SUBCAM_CSI,
    NUM_SHLCDC_DEV_TYPE
};
#endif /* CONFIG_SHLCDC_ILLUMI */

enum {
    SHLCDC_DEV_PWR_OFF,
    SHLCDC_DEV_PWR_ON,
    NUM_SHLCDC_DEV_PWR
};

enum {
    SHLCDC_IR_IRSD_MODE_LO,
    SHLCDC_IR_IRSD_MODE_HI,
    SHLCDC_IR_IRSD_MODE_IRDACC_CTL,
    NUM_SHLCDC_IR_IRSD_MODE
};

enum {
    SHLCDC_IR_IRSEL_MODE_LO,
    SHLCDC_IR_IRSEL_MODE_HI,
    SHLCDC_IR_IRSEL_MODE_IRDACC_CTL,
    NUM_SHLCDC_IR_IRSEL_MODE
};

enum {
    SHLCDC_TP_PSOC_STBY_LO,
    SHLCDC_TP_PSOC_STBY_HI,
    NUM_SHLCDC_TP_PSOC_STBY
};

enum {
    SHLCDC_TP_PSOC_RESET_LO,
    SHLCDC_TP_PSOC_RESET_HI,
    NUM_SHLCDC_TP_PSOC_RESET
};

struct shlcdc_subscribe {
    int event_type;
    void (*callback)(void);
};

int shlcdc_api_set_power_mode(int dev_type, int dev_pwr_req);
int shlcdc_api_event_subscribe(struct shlcdc_subscribe *subscribe);
int shlcdc_api_event_unsubscribe(int event_type);
int shlcdc_api_ir_set_irsd_mode(int irsd_mode);
int shlcdc_api_ir_set_irsel_mode(int irsel_mode);
int shlcdc_api_tp_set_psoc_stby_mode(int stby_mode);
int shlcdc_api_tp_set_psoc_reset_mode(int reset_mode);

#endif /* SHLCDC_KERN_H */
