/* drivers/sharp/shmds/shmds_driver.c  (MotionSensor Driver)
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/freezer.h>
#include <linux/cdev.h>
#include <linux/timer.h>
#include <asm/gpio.h>

#include <sharp/shmds_driver.h>
#include <mach/vreg.h>

#define DEBUG 0

#ifdef CONFIG_SHEXTDEV
#include <mach/rpc_server_handset.h>
#endif

#define MS_MTIME 1;

static struct i2c_client *this_client;
static unsigned char Mes_ReadData[9];
static unsigned char Pedo_ReadData[9];
static MS_InitData_t MS_InitData;
static unsigned char MS_Interrupt_State = MS_INTERRUPT_OFF;
static unsigned char MS_PedoDataGetWaitCnt = 0;
static struct timer_list MS_TimerParam;
static unsigned short MS_TimerCount = 2 - MS_MTIME;
static unsigned char MS_InterruptCnt = 0;
static unsigned char MS_TimerOnFlg = MS_FLG_OFF;
static unsigned char MS_Position;
static unsigned char MS_FirstIntFlg = MS_FLG_OFF;
static unsigned char MS_ReadDataFlg = MS_FLG_OFF;

static unsigned char MSState = 0;
static unsigned short MS_MesWait = 0;
static MS_CustodyData_t MS_CustodyData;
static MS_CalibrationParam_t MS_CalibrationParam;
static unsigned char MS_InitFlg = 0;
static MS_UserCalibrationData_t MS_UserCalData;
static MS_PedoParam_t MS_PedoParam;
static unsigned char MS_PedoPause = 0;
static MS_BatteryLid_t MS_BatteryLid;
static unsigned char MSPedoThresh[17] = {50,20,25,3,20,40,20,5,50,4,9,100,15,10,50,13,3};
static unsigned long MS_ActiveSensor = 0x00;
static MS_DisplayData_t MS_DisplayData;
static MS_TPthread_t MS_TPthread;
static unsigned char MS_TPMode = 0;
static atomic_t MS_AutoMaticV;

struct ami602_data {
    struct input_dev *input_dev;
    struct work_struct work;
};

static int ami602_I2C_read(char Cmd, char *Data ,char Length)
{
    int nResult;
    char readbuf[20];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

        nResult = i2c_master_send(this_client, &Cmd, 1);
        if(nResult == 1){
            if(Cmd == SET_PED_TH){
                udelay(500);
            }else{
                udelay(350);
            }
            nResult = i2c_master_recv(this_client, readbuf,( Length + 1));
            if(nResult == (Length + 1)){
                if(readbuf[0] != 0){
                    return MS_RESULT_ERR;
                }
                memcpy( Data, &readbuf[1], Length );
                return 0;
            }
        }
    return -EPIPE;
}

static int ami602_I2C_write(char Cmd, char *Data ,char Length)
{
    int nResult;
    char writebuf[20];
    char readbuf[1];
    
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
    
    writebuf[0] = Cmd;
    memcpy(&writebuf[1], Data, Length);
        nResult = i2c_master_send(this_client, writebuf, Length + 1);
        if(nResult == (Length + 1)){
            if(Cmd == SET_PED_TH){
                udelay(500);
            }else{
                udelay(350);
            }
            nResult = i2c_master_recv(this_client, readbuf, 1);
            if(nResult == 1){
                if(readbuf[0] != 0){
                    return MS_RESULT_ERR;
                }
                return 0;
            }
        }
    return -EPIPE;
}

static void ami602_ChangeActive(void)
{
    udelay(30);
    gpio_direction_output(SH_MS_TRG, 0);
    udelay(1);
    gpio_direction_output(SH_MS_TRG, 1);
    udelay(20);
}

static void ami602_PowerDown(void)
{
    unsigned char i;
    int nResult;
    unsigned char writedata[2];

    writedata[0] = 'p';
    writedata[1] = 'd';

    udelay(100);

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(SET_PWR_DOWN, writedata, 2);
        if (nResult == 0) {
            break;
        }
        mdelay(5);
    }
}

static void ami602_Reset(void)
{
    gpio_direction_output(SH_MS_RST, 1);
    udelay(50);
    gpio_direction_output(SH_MS_RST, 0);
    udelay(5);
    gpio_direction_output(SH_MS_RST, 1);
    udelay(50);
    mdelay(3);
}

static void ami602_timerinterrupt(unsigned long data)
{
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    enable_irq(this_client->irq);
}

static void ami602_starttimer(void)
{
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if (MS_TimerOnFlg == MS_FLG_ON) {
        del_timer_sync(&MS_TimerParam);
        MS_TimerOnFlg = MS_FLG_OFF;
    }

    if ( MS_MesWait <= 20) {
        return;
    }
    MS_TimerCount = (MS_MesWait / 10) - MS_MTIME;

    init_timer(&MS_TimerParam);

    MS_TimerParam.function = ami602_timerinterrupt;
    MS_TimerParam.data = (unsigned long)NULL;
    MS_TimerParam.expires = jiffies + (MS_TimerCount * HZ / 100);
    
    add_timer(&MS_TimerParam);
    MS_TimerOnFlg = MS_FLG_ON;
}

static void ami602_stoptimer(void)
{
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if (MS_TimerOnFlg == MS_FLG_OFF) {
        return;
    }

    del_timer_sync(&MS_TimerParam);
    MS_TimerOnFlg = MS_FLG_OFF;
}

static void ami602_work_func(struct work_struct *work)
{
    unsigned char i,j;
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;
    int nResult;
    unsigned char readdata[9],pedodata[9];
    unsigned short meswait = 20;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if (MS_MesWait >= 200) {
        return;
    }

    if (MS_Interrupt_State == MS_INTERRUPT_OFF) {
        enable_irq(this_client->irq);
        return;
    }

    if (MS_FirstIntFlg == MS_FLG_ON) {
        MS_FirstIntFlg = MS_FLG_OFF;
        enable_irq(this_client->irq);
        return;
    }

    if (MS_ReadDataFlg == MS_FLG_ON) {
        MS_ReadDataFlg = MS_FLG_OFF;
    }

    if (MS_InterruptCnt >= 1) {
        MS_InterruptCnt--;
        enable_irq(this_client->irq);
        return;
    }

    if (MS_MesWait <= 20) {
        MS_InterruptCnt = 0;
    }
    else {
        MS_InterruptCnt = 1;
    }

    udelay(10);

    for (i=0; i<5; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
            I2C_PermissionFlg = MS_FLG_ON;
            break;
        }
        mdelay(1);
    }

    if (I2C_PermissionFlg == MS_FLG_OFF) {
        if ( MS_MesWait <= 20) {
            enable_irq(this_client->irq);
        }
        else {
            ami602_starttimer();
        }
        return;
    }

    if (MS_MesWait < 20) {
        meswait = 20;
    }
    else {
        meswait = MS_MesWait;
    }

    if (MS_Interrupt_State == MS_INTERRUPT_MES_PEDO) {
        MS_PedoDataGetWaitCnt++;
    }

    if (MS_PedoDataGetWaitCnt >= (320 / meswait)) {
        I2C_PermissionFlg = MS_FLG_OFF;
        MS_PedoDataGetWaitCnt = 0;
        for (i=0; i<5; i++) {

            ami602_ChangeActive();

            nResult = ami602_I2C_read(GET_MES, readdata, 9);
            if(nResult == 0){
                memcpy(Mes_ReadData,readdata,9);
                break;
            }

            for (j=0; j<10; j++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                    break;
                }
                mdelay(1);
            }
            if( I2C_PermissionFlg == MS_FLG_OFF ){
                break;
            }
        }

        if( nResult == 0) {
            I2C_PermissionFlg = MS_FLG_OFF;
            for (i=0; i<16; i++) {

                ami602_ChangeActive();

                nResult = ami602_I2C_read(GET_MES_PED_AUTO_SUSPEND, pedodata, 9);
                if(nResult == 0){
                    memcpy(Pedo_ReadData,pedodata,9);
                    break;
                }

                for (j=0; j<5; j++) {
                    if (gpio_get_value(SH_MS_IRQ) == 1) {
                        I2C_PermissionFlg = MS_FLG_ON;
                        break;
                    }
                    mdelay(1);
                }
                if( I2C_PermissionFlg == MS_FLG_OFF ){
                    break;
                }
            }
        }
    }
    else {
        I2C_PermissionFlg = MS_FLG_OFF;
        for (i=0; i<3; i++) {

            ami602_ChangeActive();

            nResult = ami602_I2C_read(GET_MES_SUSPEND, readdata, 9);
            if(nResult == 0){
                memcpy(Mes_ReadData,readdata,9);
                break;
            }

            for (j=0; j<10; j++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                    break;
                }
                mdelay(3);
            }
            if( I2C_PermissionFlg == MS_FLG_OFF ){
                break;
            }
        }
    }

    if ( MS_MesWait <= 20) {
        enable_irq(this_client->irq);
    }
    else {
        ami602_starttimer();
    }
}

static irqreturn_t ami602_interrupt(int irq, void *dev_id)
{
    struct ami602_data *pdata = dev_id;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    disable_irq_nosync(this_client->irq);
    schedule_work(&(pdata->work));

    return IRQ_HANDLED;
}

static int ami602_Start_SensorInterrupt(void)
{
    struct ami602_data *pdata;
    int ret;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    pdata = i2c_get_clientdata(this_client);

    MS_FirstIntFlg = MS_FLG_ON;
    ret = request_irq(this_client->irq, ami602_interrupt, IRQF_TRIGGER_RISING, "ami602", pdata);
    if (MS_MesWait >= 200) {
        MS_ReadDataFlg = MS_FLG_ON;
        disable_irq_nosync(this_client->irq);
    }
    else {
        MS_ReadDataFlg = MS_FLG_OFF;
    }
    if (ret < 0) {
#if DEBUG
        printk(KERN_ERR "ami602 request irq failed\n");
#endif
        return ret;
    }

    return ret;
}

static int ami602_Stop_SensorInterrupt(void)
{
    struct ami602_data *pdata;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    pdata = i2c_get_clientdata(this_client);

    free_irq(this_client->irq, pdata);

    return 0;
}

static void ami602_ReadRawValue(void)
{
    unsigned char i,j;
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;
    int nResult;
    unsigned char readdata[9],pedodata[9];
    unsigned short meswait = 20;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if (MS_Interrupt_State == MS_INTERRUPT_OFF) {
        return;
    }

    if (MS_ReadDataFlg == MS_FLG_OFF) {
        if (MS_MesWait >= 200) {
            MS_ReadDataFlg = MS_FLG_ON;
        }
        else {
            return;
        }
    }

    udelay(10);

    for (i=0; i<5; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
            I2C_PermissionFlg = MS_FLG_ON;
            break;
        }
        mdelay(1);
    }

    if (MS_MesWait < 20) {
        meswait = 20;
    }
    else {
        meswait = MS_MesWait;
    }

    if (MS_Interrupt_State == MS_INTERRUPT_MES_PEDO) {
        MS_PedoDataGetWaitCnt++;
    }

    if (MS_PedoDataGetWaitCnt >= (320 / meswait)) {
        I2C_PermissionFlg = MS_FLG_OFF;
        MS_PedoDataGetWaitCnt = 0;
        for (i=0; i<5; i++) {

            ami602_ChangeActive();

            nResult = ami602_I2C_read(GET_MES, readdata, 9);
            if(nResult == 0){
                memcpy(Mes_ReadData,readdata,9);
                break;
            }

            for (j=0; j<10; j++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                    break;
                }
                mdelay(1);
            }
            if( I2C_PermissionFlg == MS_FLG_OFF ){
                break;
            }
        }

        if( nResult == 0) {
            I2C_PermissionFlg = MS_FLG_OFF;
            for (i=0; i<16; i++) {

                ami602_ChangeActive();

                nResult = ami602_I2C_read(GET_MES_PED_AUTO_SUSPEND, pedodata, 9);
                if(nResult == 0){
                    memcpy(Pedo_ReadData,pedodata,9);
                    break;
                }

                for (j=0; j<5; j++) {
                    if (gpio_get_value(SH_MS_IRQ) == 1) {
                        I2C_PermissionFlg = MS_FLG_ON;
                        break;
                    }
                    mdelay(1);
                }
                if( I2C_PermissionFlg == MS_FLG_OFF ){
                    break;
                }
            }
        }
    }
    else {
        I2C_PermissionFlg = MS_FLG_OFF;
        for (i=0; i<3; i++) {

            ami602_ChangeActive();

            nResult = ami602_I2C_read(GET_MES_SUSPEND, readdata, 9);
            if(nResult == 0){
                memcpy(Mes_ReadData,readdata,9);
                break;
            }

            for (j=0; j<10; j++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                    break;
                }
                mdelay(3);
            }
            if( I2C_PermissionFlg == MS_FLG_OFF ){
                break;
            }
        }
    }

    if (MS_MesWait < 200) {
        enable_irq(this_client->irq);
        MS_FirstIntFlg = MS_FLG_OFF;
    }
}

int ami602_ReqMES(void)
{
    unsigned char i;
    int nResult = 0;
    unsigned char writedata = 0;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(REQ_MES, &writedata, 0);
        if (nResult == 0) {
            break;
        }
        mdelay(3);
    }
    return nResult;
}

int ami602_CalibrationDataMeasurement(unsigned short *Data)
{
    unsigned char ReadData[9];
    unsigned char i;
    int Result = 0;
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    memset(ReadData,0,sizeof(ReadData));

    Result = ami602_ReqMES();
    if(Result < 0 ){
        return Result;
    }

    mdelay(5);

    for (i=0; i<5; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
            I2C_PermissionFlg = MS_FLG_ON;
            break;
        }
        mdelay(1);
    }
     if( I2C_PermissionFlg == MS_FLG_OFF ){
        return -1;
     }

    I2C_PermissionFlg = MS_FLG_OFF;

    for (i=0; i<16; i++) {
        ami602_ChangeActive();
        Result = ami602_I2C_read(GET_MES, ReadData, 9);
        if (Result >= 0) {
            mdelay(1);
            break;
        }
        else {
            for (i=0; i<5; i++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                     break;
                   }
                   mdelay(1);
            }
            if (I2C_PermissionFlg == MS_FLG_OFF) {
                return -1;
            }
        }
    }


    Data[0] = ( ReadData[0] << 4) | ((ReadData[1] & 0xF0) >> 4);
    Data[1] = ((ReadData[1] & 0x0F) << 8) | ReadData[2];
    Data[2] = ( ReadData[3] << 4) | ((ReadData[4] & 0xF0) >> 4);
    Data[3] = ((ReadData[4] & 0x0F) << 8) | ReadData[5];
    Data[4] = ( ReadData[6] << 4) | ((ReadData[7] & 0xF0) >> 4);
    Data[5] = ((ReadData[7] & 0x0F) << 8) | ReadData[8];

    for( i=0; i<6; i++ ){
        if( Data[i] & 0x8000 ){
            Data[i] = 0;
        }
        if( Data[i] > 4095 ){
            Data[i] = 4095;
        }
    }

    return 0;
}

static void ami602_SetAEN(unsigned char data)
{
    unsigned char i;
    int nResult;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(SET_AEN, &data, 1);
        if (nResult == 0) {
            break;
        }
        mdelay(5);
    }
}

static int ami602_SetCOSR(unsigned char *Data)
{
        unsigned char i;
        unsigned char writedata[6];
        int nResult = 0;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

        for (i=0; i<6; i++) {
            writedata[i] = Data[i];
            if(writedata[i] > 7){
                return -1;
            }
        }

        for (i=0;i<0x10;i++) {
            ami602_ChangeActive();
            nResult = ami602_I2C_write(SET_COSR, writedata, 6);
            if (nResult == 0) {
                break;
            }
            mdelay(3);
        }

        return nResult;
}

static int ami602_SetFINR(unsigned char *Data)
{
        unsigned char i;
        unsigned char writedata[6];
        int nResult;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

        for (i=0; i<6; i++) {
            writedata[i] = Data[i];
            if (writedata[i] > 51 || writedata[i] < 12) {
                    return -1;
            }
        }

        for (i=0;i<0x10;i++) {
            ami602_ChangeActive();
            nResult = ami602_I2C_write(SET_FINR, writedata, 6);
            if (nResult == 0) {
                break;
            }
            mdelay(3);
        }

        return nResult;
}

static int  ami602_SerchCosr(unsigned char *Param)
{
    int Result;
    unsigned char i,j;
    unsigned char EndFlg = 0x1C;
    unsigned char CosrData[6];
    unsigned char Cosr[6];
    unsigned short Data[6];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    memset(Cosr, 0, sizeof(Cosr));
    memset(CosrData, 0, sizeof(CosrData));

    for (i=0; i<=7; i++) {
            for (j=0; j<6; j++) {
            if (!(EndFlg & (1<<j))) {
                Cosr[j] = i;
            }
        }
        Result = ami602_SetCOSR( Cosr );
        if ( Result < 0 ) {
            return -1;
        }
        Result = ami602_CalibrationDataMeasurement( Data );
        if ( Result < 0 ) {
            return -1;
        }

        for (j=0; j<6; j++) {
            if(j < 2 || j == 5){
                if (!(EndFlg & (1<<j))) {
                    if ( Data[j] > 2048) {
                        CosrData[j] = i;
                        EndFlg |= 1<<j;
                    }
                }
            }
        }

        if (EndFlg == 0x3F) {
            break;
        }
    }

    if (EndFlg != 0x3F) {
        return -1;
    }

    memcpy(Param, CosrData, sizeof(CosrData));
    return 0;
}

static int  ami602_SerchFinr(unsigned char *Param)
{
    int Result;
    unsigned char i,j;
    unsigned char EndFlg = 0x1C;
    unsigned char FinrData[6];
    unsigned char Finr[6];
    unsigned short Data[6];
    unsigned short LastData[6];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    for (i=0; i<6; i++) {
        Finr[i] = 12;
    }
    memset( FinrData, 12, sizeof(FinrData) );
    memset( LastData, 0xFF, sizeof(LastData) );

    for (i=12; i<=51; i++) {
        for (j=0; j<6; j++) {
            Finr[j] = i;
        }
        Result = ami602_SetFINR( Finr );
        if ( Result < 0 ) {
            return -1;
        }
        Result = ami602_CalibrationDataMeasurement( Data );
        if ( Result < 0 ) {
            return -1;
        }

        for (j=0; j<6; j++) {
            if(j < 2 || j == 5){
                if (!(EndFlg & (1<<j))) {
                    if ((LastData[j] < 500)
                    && (LastData[j] < abs(2048 - Data[j]))){
                        EndFlg |= 1<<j;
                    }
                    else {
                        FinrData[j] = i;
                        LastData[j] = abs(2048 - Data[j]);
                    }
                }
            }
        }

        if (EndFlg == 0x3F) {
            break;
        }
    }

    if (EndFlg != 0x3F) {
        return -1;
    }

    memcpy(Param, FinrData, sizeof(FinrData));
    return 0;
}

static int ami602_CalibrationCosrFinrStop(void)
{
    int i;
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
    
    for (i=0; i<50; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
            I2C_PermissionFlg = MS_FLG_ON;
            break;
        }
        mdelay(4);
    }
    if (I2C_PermissionFlg == MS_FLG_OFF) {
        return -1;
    }

    ami602_SetAEN(0);

    for (i=0; i<50; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
            I2C_PermissionFlg = MS_FLG_ON;
            break;
        }
        mdelay(4);
    }
    if (I2C_PermissionFlg == MS_FLG_OFF) {
        return -1;
    }

    return 0;
}

static int ami602_CFCalibrationSequence(void)
{
    int Result;
    int i;
    unsigned char Finr[6],FinrData[6],CosrData[6];
 
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    for (i=0; i<6; i++) {
        Finr[i] = 12;
    }
    ami602_SetFINR(Finr);

    Result = ami602_SerchCosr(CosrData);
    if( Result < 0 ){
        ami602_CalibrationCosrFinrStop();
        return -1;
    }

    Result = ami602_SerchFinr(FinrData);
    if( Result < 0 ){
        ami602_CalibrationCosrFinrStop();
        return -1;
    }

    if (MS_Position) {
        memcpy((void*)MS_UserCalData.OtherCosrValue,(void*)CosrData,sizeof(MS_UserCalData.OtherCosrValue));
        memcpy((void*)MS_UserCalData.OtherFinrValue,(void*)FinrData,sizeof(MS_UserCalData.OtherFinrValue));
        MS_UserCalData.CalibrationFlg |= CAL_COSRFINR2_MASK;
    }
    else {
        memcpy((void*)MS_UserCalData.OpenCosrValue,(void*)CosrData,sizeof(MS_UserCalData.OpenCosrValue));
        memcpy((void*)MS_UserCalData.OpenFinrValue,(void*)FinrData,sizeof(MS_UserCalData.OpenFinrValue));
        MS_UserCalData.CalibrationFlg |= CAL_COSRFINR_MASK;
    }

    return 0;
}

static int ami602_CFCalibration_SensorValueCheck( void )
{
    unsigned short Data[6];
    int Result;
    int i;
    int cal_check = 0;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    ami602_SetAEN(1);
    mdelay(2);
    for (i=0; i<3; i++){
        memset(Data, 0, sizeof(Data));
        Result = ami602_CalibrationDataMeasurement( Data );
        if(Result < 0){
            return -1;
        }
 
        if(((Data[0] < 1000) || (Data[0] > 3000))
            ||((Data[1] < 1000) || (Data[1] > 3000))
            ||((Data[5] < 1000) || (Data[5] > 3000))){
            cal_check = 1;
        }else{
            cal_check = 0;
            break;
        }
         mdelay(1);
    }
    return cal_check;
}

static void ami602_MeasureStart(void)
{
    unsigned char i;
    int nResult;
    unsigned char writedata[3];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    gpio_direction_output(SH_MS_TRG, 0);
    udelay(1);
    gpio_direction_output(SH_MS_TRG, 1);
    mdelay(3);
    for (i=0; i<5; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
                   break;
                }
                 mdelay(1);
        }

    writedata[0] = 0x00;
    writedata[1] = 's';
    writedata[2] = 't';
    udelay(100);

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(SET_MES_6CH_AUTO_START, writedata, 3);
        if (nResult == 0) {
             break;
        }
        mdelay(5);
    }
}

static void ami602_MeasureStop(void)
{
    unsigned char i;
    int nResult;
    unsigned char writedata[2];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
  
    writedata[0] = 'e';
    writedata[1] = 'd';

    udelay(100);

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(SET_MES_6CH_AUTO_STOP, writedata, 2);
        if (nResult == 0) {
            break;
        }
        mdelay(5);
    }
}

static void ami602_PedometerStart(void)
{
    unsigned char i;
    int nResult;
    unsigned char writedata[2];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    writedata[0] = 's';
    writedata[1] = 't';

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(SET_MES_PED_AUTO_START, writedata, 2);
        if (nResult == 0) {
             break;
        }
        mdelay(5);
    }
}

static void ami602_PedometerStop(void)
{
    unsigned char i;
    int nResult;
    unsigned char writedata[2];

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    writedata[0] = 'e';
    writedata[1] = 'd';

    udelay(100);

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_write(SET_MES_PED_AUTO_STOP, writedata, 2);
        if (nResult == 0) {
             break;
        }
        mdelay(5);
    }
}

static void ami602_SetCosrFinr(void)
{
    MS_CosrFinrData_t MS_CosrFinrData;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    MS_CosrFinrData.CalibrationFlg = 0;

    if (MS_Position) {
            if (MS_UserCalData.CalibrationFlg & CAL_COSRFINR2_MASK) {
                    memcpy( MS_CosrFinrData.CosrValue, MS_UserCalData.OtherCosrValue, sizeof(MS_CosrFinrData.CosrValue) );
                    memcpy( MS_CosrFinrData.FinrValue, MS_UserCalData.OtherFinrValue, sizeof(MS_CosrFinrData.FinrValue) );
                    MS_CosrFinrData.CalibrationFlg = 0x01;
            }
            else {
                    if (MS_CustodyData.CalibrationFlg & CAL_COSRFINR2_MASK) {
                            memcpy( MS_CosrFinrData.CosrValue, MS_CustodyData.OtherCosrValue, sizeof(MS_CosrFinrData.CosrValue) );
                            memcpy( MS_CosrFinrData.FinrValue, MS_CustodyData.OtherFinrValue, sizeof(MS_CosrFinrData.FinrValue) );
                            MS_CosrFinrData.CalibrationFlg = 0x01;
                    }
                    else{
                            memcpy( MS_CosrFinrData.CosrValue, MS_InitData.ChipCosr, sizeof(MS_CosrFinrData.CosrValue) );
                            memcpy( MS_CosrFinrData.FinrValue, MS_InitData.ChipFinr, sizeof(MS_CosrFinrData.FinrValue) );
                            MS_CosrFinrData.CalibrationFlg = 0x01;
                     }
            }
    }
    else {
            if (MS_UserCalData.CalibrationFlg & CAL_COSRFINR_MASK) {
                    memcpy( MS_CosrFinrData.CosrValue, MS_UserCalData.OpenCosrValue, sizeof(MS_CosrFinrData.CosrValue) );
                    memcpy( MS_CosrFinrData.FinrValue, MS_UserCalData.OpenFinrValue, sizeof(MS_CosrFinrData.FinrValue) );
                    MS_CosrFinrData.CalibrationFlg = 0x01;
            }
            else {
                    if (MS_CustodyData.CalibrationFlg & CAL_COSRFINR_MASK) {
                            memcpy( MS_CosrFinrData.CosrValue, MS_CustodyData.OpenCosrValue, sizeof(MS_CosrFinrData.CosrValue) );
                            memcpy( MS_CosrFinrData.FinrValue, MS_CustodyData.OpenFinrValue, sizeof(MS_CosrFinrData.FinrValue) );
                            MS_CosrFinrData.CalibrationFlg = 0x01;
                     }
                    else
                    {
                        memcpy( MS_CosrFinrData.CosrValue, MS_InitData.ChipCosr, sizeof(MS_InitData.ChipCosr) );
                        memcpy( MS_CosrFinrData.FinrValue, MS_InitData.ChipFinr, sizeof(MS_InitData.ChipFinr) );
                        MS_CosrFinrData.CalibrationFlg = 0x01;
                    }
            }
    }

    if (MS_CosrFinrData.CalibrationFlg & 0x01 ) {
        ami602_SetCOSR(MS_CosrFinrData.CosrValue);
        ami602_SetFINR(MS_CosrFinrData.FinrValue);
    }

}

static void ami602_AdjustCosrFinr(void)
{
    char check = 0;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if (MS_Position == MS_POSITION_CLOSE) {
        return;
    }

    switch (MSState) {
        case MEASURE_PEDOMETER_STANDBY:
            ami602_SetCosrFinr();
            check = ami602_CFCalibration_SensorValueCheck();
            if(check > 0){
                ami602_CFCalibrationSequence();
                ami602_SetCosrFinr();
            }
            ami602_PowerDown();
            break;

        case MEASURE_ONLY_ACTIVE:
            ami602_Stop_SensorInterrupt();
            ami602_stoptimer();
            ami602_MeasureStop();
            ami602_SetCosrFinr();
            check = ami602_CFCalibration_SensorValueCheck();
            if(check > 0){
                ami602_CFCalibrationSequence();
                ami602_SetCosrFinr();
            }
            ami602_MeasureStart();
            ami602_Start_SensorInterrupt();
            break;

        case PEDOMETER_ONLY_ACTIVE:
            AMI602Pedometer_Pause();
            ami602_PedometerStop();
            ami602_SetCosrFinr();
            check = ami602_CFCalibration_SensorValueCheck();
            if(check > 0){
                ami602_CFCalibrationSequence();
                ami602_SetCosrFinr();
            }
            AMI602Pedometer_ReStart();
            ami602_PedometerStart();
            break;

        case MEASURE_PEDOMETER_ACTIVE:
            AMI602Pedometer_Pause();
            ami602_PedometerStop();
            ami602_Stop_SensorInterrupt();
            ami602_stoptimer();
            ami602_MeasureStop();
            ami602_SetCosrFinr();
            check = ami602_CFCalibration_SensorValueCheck();
            if(check > 0){
                ami602_CFCalibrationSequence();
                ami602_SetCosrFinr();
            }
            ami602_MeasureStart();
            ami602_Start_SensorInterrupt();
            AMI602Pedometer_ReStart();
            ami602_PedometerStart();
            break;

        default:
            break;;
    }

}

static void ami602_GetPosition(void)
{
#ifdef CONFIG_SHEXTDEV
    shextdet_form_position_result_t position;
#else
    int position = 0;
#endif

#ifdef CONFIG_SHEXTDEV
        position = shextdet_get_form_state();
#endif

        if (position != MS_POSITION_CLOSE) {
            MS_Position = (unsigned char)position;
        }
}

static int ami602_open(struct inode *inode, struct file *file)
{
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    return nonseekable_open(inode, file);
}

static int ami602_release(struct inode *inode, struct file *file)
{
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if(atomic_read(&MS_AutoMaticV) == 1)
    {
        ami602_AdjustCosrFinr();
        atomic_set(&MS_AutoMaticV, 0);
    }
    return 0;
}

static int ami602_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    char gpio_param;
    MS_I2CParam_t i2cparam;
    MS_PedoParam_t pedoparam;
    int ret = 0;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    switch (cmd) {
/* ami602 ioctl */
    case MS_I2C_READ:
        if (copy_from_user(&i2cparam, argp, sizeof(i2cparam))){
            return -EFAULT;
        }

        ret = ami602_I2C_read(i2cparam.cmd, i2cparam.data, i2cparam.length);

        if (copy_to_user(argp, &i2cparam, sizeof(i2cparam))){
            return -EFAULT;
        }
        break;

    case MS_I2C_WRITE:
        if (copy_from_user(&i2cparam, argp, sizeof(i2cparam))){
            return -EFAULT;
        }
        ret = ami602_I2C_write(i2cparam.cmd, i2cparam.data, i2cparam.length);

        if(i2cparam.cmd == SET_MES_6CH_AUTO_START)
        {
            memset(Mes_ReadData, 0x00, sizeof(Mes_ReadData));
        }
        if((i2cparam.cmd == SET_MES_PED_AUTO_START) || (i2cparam.cmd == CLR_PED_SUSPEND)){
            memset(Pedo_ReadData, 0x00, sizeof(Pedo_ReadData));
        }

        if((i2cparam.cmd == SET_COSR) || (i2cparam.cmd == SET_FINR))
        {
            memset(Mes_ReadData, 0x00, sizeof(Mes_ReadData));
        }
        break;

    case MS_MISTART_GPIO:
        gpio_direction_output(SH_MS_TRG, arg);
        break;

    case MS_MIBUSY_GPIO:
        gpio_param = gpio_get_value(SH_MS_IRQ);
        if (copy_to_user(argp, &gpio_param, sizeof(gpio_param))) {
            return -EFAULT;
        }
        break;

    case MS_MIRST_GPIO:
        gpio_direction_output(SH_MS_RST, arg);
        break;

    case MS_INTERRUPT:
        MS_InterruptCnt = 0;
        MS_PedoDataGetWaitCnt = 0;
        if (arg == MS_INTERRUPT_MES) {
            if (MS_Interrupt_State == MS_INTERRUPT_OFF) {
                ami602_Start_SensorInterrupt();
            }
            MS_Interrupt_State = MS_INTERRUPT_MES;
        }
        else if (arg == MS_INTERRUPT_MES_PEDO) {
            if (MS_Interrupt_State == MS_INTERRUPT_OFF) {
                ami602_Start_SensorInterrupt();
            }
            MS_Interrupt_State = MS_INTERRUPT_MES_PEDO;
        }
        else {
            MS_Interrupt_State = MS_INTERRUPT_OFF;
            ami602_Stop_SensorInterrupt();
            ami602_stoptimer();
        }
        break;

    case MS_GETMES:
        ami602_ReadRawValue();
        if (copy_to_user(argp, Mes_ReadData, sizeof(Mes_ReadData))) {
            return -EFAULT;
        }
        break;

    case MS_GETPED:
        if (copy_to_user(argp, Pedo_ReadData, sizeof(Pedo_ReadData))) {
            return -EFAULT;
        }
        break;

    case MS_GET_INIT:
        if (copy_to_user(argp, &MS_InitData, sizeof(MS_InitData))) {
            return -EFAULT;
        }
        break;

    case MS_GETPOSITION:
        ami602_GetPosition();
        if (copy_to_user(argp, &MS_Position, sizeof(MS_Position))) {
            return -EFAULT;
        }
        break;

    case MS_COSRFINRCHECK:
        ami602_GetPosition();
        ami602_AdjustCosrFinr();
        break;

/* user data hold */
    case MS_GETSTATE:
        if (copy_to_user(argp, &MSState, sizeof(MSState))) {
            return -EFAULT;
        }
        break;

    case MS_SETSTATE:
        if (copy_from_user(&MSState, argp, sizeof(MSState))) {
            return -EFAULT;
        }
        break;

    case MS_GETCUSTODY:
        if (copy_to_user(argp, &MS_CustodyData, sizeof(MS_CustodyData_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETCUSTODY:
        if (copy_from_user(&MS_CustodyData, argp, sizeof(MS_CustodyData_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETUSERDATA:
        if (copy_to_user(argp, &MS_UserCalData, sizeof(MS_UserCalibrationData_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETUSERDATA:
        if (copy_from_user(&MS_UserCalData, argp, sizeof(MS_UserCalibrationData_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETPEDOPARAM:
        MS_PedoParam.KernelPause = MS_PedoPause;
        if (copy_to_user(argp, &MS_PedoParam, sizeof(MS_PedoParam_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETPEDOPARAM:
        if (copy_from_user(&pedoparam, argp, sizeof(MS_PedoParam_t))) {
            return -EFAULT;
        }
        if (pedoparam.KernelPause == MS_PedoPause) {
            memcpy(&MS_PedoParam, &pedoparam, sizeof(MS_PedoParam_t));
        }
        break;

    case MS_INITPEDOPARAM:
        if (copy_from_user(&MS_PedoParam, argp, sizeof(MS_PedoParam_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETPEDOPAUSE:
        if (copy_to_user(argp, &MS_PedoPause, sizeof(MS_PedoPause))) {
            return -EFAULT;
        }
        break;

    case MS_SETPEDOPAUSE:
        if (copy_from_user(&MS_PedoPause, argp, sizeof(MS_PedoPause))) {
            return -EFAULT;
        }
        break;

    case MS_GETPEDOTHRESH:
        if (copy_to_user(argp, MSPedoThresh, sizeof(MSPedoThresh))) {
            return -EFAULT;
        }
        break;

    case MS_SETPEDOTHRESH:
        if (copy_from_user(MSPedoThresh, argp, sizeof(MSPedoThresh))) {
            return -EFAULT;
        }
        break;

    case MS_GETMESWAIT:
        if (copy_to_user(argp, &MS_MesWait, sizeof(MS_MesWait))) {
            return -EFAULT;
        }
        break;

    case MS_SETMESWAIT:
        if (copy_from_user(&MS_MesWait, argp, sizeof(MS_MesWait))) {
            return -EFAULT;
        }
        break;

    case MS_GETBATTERYLID:
        if (copy_to_user(argp, &MS_BatteryLid, sizeof(MS_BatteryLid_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETBATTERYLID:
        if (copy_from_user(&MS_BatteryLid, argp, sizeof(MS_BatteryLid_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETINITFLG:
        if (copy_to_user(argp, &MS_InitFlg, sizeof(MS_InitFlg))) {
            return -EFAULT;
        }
        break;

    case MS_SETINITFLG:
        if (copy_from_user(&MS_InitFlg, argp, sizeof(MS_InitFlg))) {
            return -EFAULT;
        }
        break;

    case MS_GETCALIBRATION:
        if (copy_to_user(argp, &MS_CalibrationParam, sizeof(MS_CalibrationParam_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETCALIBRATION:
        if (copy_from_user(&MS_CalibrationParam, argp, sizeof(MS_CalibrationParam_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETACTIVE:
        if (copy_to_user(argp, &MS_ActiveSensor, sizeof(MS_ActiveSensor))) {
            return -EFAULT;
        }
        break;

    case MS_SETACTIVE:
        if (copy_from_user(&MS_ActiveSensor, argp, sizeof(MS_ActiveSensor))) {
            return -EFAULT;
        }
        break;

    case MS_GETDISPLAY:
        if (copy_to_user(argp, &MS_DisplayData, sizeof(MS_DisplayData_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETDISPLAY:
        if (copy_from_user(&MS_DisplayData, argp, sizeof(MS_DisplayData_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETTPTHREAD:
        if (copy_to_user(argp, &MS_TPthread, sizeof(MS_TPthread_t))) {
            return -EFAULT;
        }
        break;

    case MS_SETTPTHREAD:
        if (copy_from_user(&MS_TPthread, argp, sizeof(MS_TPthread_t))) {
            return -EFAULT;
        }
        break;

    case MS_GETTPMODE:
        if (copy_to_user(argp, &MS_TPMode, sizeof(MS_TPMode))) {
            return -EFAULT;
        }
        break;

    case MS_SETTPMODE:
        if (copy_from_user(&MS_TPMode, argp, sizeof(MS_TPMode))) {
            return -EFAULT;
        }
        break;

    default:
        break;
    }
    return ret;
}

static struct file_operations ami602_fops = {
    .owner = THIS_MODULE,
    .open = ami602_open,
    .release = ami602_release,
    .ioctl = ami602_ioctl,
};

static struct miscdevice ami602_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ami602_dev",
    .fops = &ami602_fops,
};

static int ami602_Initialize(void)
{
    unsigned char i;
    unsigned char WriteData;
    int nResult;
    unsigned char ReadData[6];
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    memset(&MS_CustodyData, 0x00, sizeof(MS_CustodyData_t));
    memset(&MS_UserCalData, 0x00, sizeof(MS_UserCalibrationData_t));
    memset(&MS_PedoParam, 0x00, sizeof(MS_PedoParam_t));
    memset(&MS_BatteryLid, 0x00, sizeof(MS_BatteryLid_t));
    MS_Position = 0;

    ami602_Reset();

    for (i=0; i<50; i++) {
        if (gpio_get_value(SH_MS_IRQ) == 1) {
            I2C_PermissionFlg = MS_FLG_ON;
            break;
        }
        mdelay(4);
    }
    if (I2C_PermissionFlg == MS_FLG_OFF) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }


    for (i=0; i<5; i++) {
        ami602_ChangeActive();
        WriteData = i;
        nResult = ami602_I2C_write(SET_INDEX_DAT, &WriteData, 1);
        if (nResult != 0) {
            MS_InitData.InitErrFlg = MS_FLG_ON;
            return 0;
        }

        ami602_ChangeActive();
        nResult = ami602_I2C_read( GET_DAT, ReadData, 6);
        if (nResult != 0) {
            MS_InitData.InitErrFlg = MS_FLG_ON;
            return 0;
        }
        MS_InitData.OriginSensitivity[i][0] = (ReadData[0] << 8) | ReadData[1];
        MS_InitData.OriginSensitivity[i][1] = (ReadData[2] << 8) | ReadData[3];
        MS_InitData.OriginSensitivity[i][2] = (ReadData[4] << 8) | ReadData[5];
    }


    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_read(GET_FIRMWARE, MS_InitData.VersionData, 6);
        if(nResult == 0){
            break;
        }
        mdelay( 3 );
    }
    if (nResult != 0) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }


    ami602_ChangeActive();
    WriteData = 30;
    nResult = ami602_I2C_write(SET_INDEX_DAT, &WriteData, 1);
    if (nResult != 0) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }

    ami602_ChangeActive();
    nResult = ami602_I2C_read(GET_DAT, ReadData, 6);
    if (nResult != 0) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }
    MS_InitData.ADOutputChangeValue[0] = (ReadData[0] << 8) | ReadData[1];
    MS_InitData.ADOutputChangeValue[1] = (ReadData[2] << 8) | ReadData[3];
    MS_InitData.ADOutputChangeValue[2] = (ReadData[4] << 8) | ReadData[5];


    ami602_ChangeActive();
    WriteData = 9;
    nResult = ami602_I2C_write(SET_INDEX_DAT, &WriteData, 1);
    if (nResult != 0) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }
    ami602_ChangeActive();
    nResult = ami602_I2C_read(GET_DAT, ReadData, 6);
    if(nResult != 0){
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }
    MS_InitData.SerialIDValue = (ReadData[0] << 8) | ReadData[1];


    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_read(GET_COSR, ReadData, 6);
        if(nResult == 0){
            break;
        }
        mdelay(3);
    }
    if (nResult != 0) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }
    memcpy(MS_InitData.ChipCosr, ReadData, sizeof(MS_InitData.ChipCosr));

    for (i=0; i<0x10; i++) {
        ami602_ChangeActive();
        nResult = ami602_I2C_read(GET_FINR, ReadData, 6);
        if(nResult == 0){
            break;
        }
        mdelay(3);
    }
    if (nResult != 0) {
        MS_InitData.InitErrFlg = MS_FLG_ON;
        return 0;
    }
    memcpy(MS_InitData.ChipFinr, ReadData, sizeof(MS_InitData.ChipFinr));

    ami602_PowerDown();

    return 0;
}

static int ami602_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
    struct ami602_data *pdata;
    struct sh_i2c_ms_platform_data *poSetupData;
    int ret = -ENODEV;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!ret) {
        return -ENODEV;
    }

    pdata = kzalloc(sizeof(struct ami602_data), GFP_KERNEL);
    if (!pdata) {
        return -ENOMEM;
    }
    
    INIT_WORK(&pdata->work, ami602_work_func);
    i2c_set_clientdata(client, pdata);
    this_client = client;
    poSetupData = client->dev.platform_data;

    pdata->input_dev = input_allocate_device();

    if (!pdata->input_dev) {
        input_free_device(pdata->input_dev);
        kfree(pdata);
        return -ENOMEM;
    }

    if(ami602_Initialize()){
        kfree(pdata);
        return -ENODEV;
    }

    ret = misc_register(&ami602_device);
    if (ret) {
        input_free_device(pdata->input_dev);
        kfree(pdata);
        return -ENODEV;
    }

    return 0;
}

static int ami602_remove(struct i2c_client *client)
{
    struct ami602_data *pdata;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    pdata = i2c_get_clientdata(client);

    free_irq(client->irq, pdata);
    del_timer_sync(&MS_TimerParam);

    ami602_Reset();
    ami602_PowerDown();

    return 0;
}

void AMI602_SetFlipInformation(unsigned char position)
{
#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    if ((position == MS_POSITION_CLOSE) || (MS_TPMode == MS_FLG_ON)) {
        return;
    }

    switch (MSState) {
        case MEASURE_PEDOMETER_STANDBY:
        case MEASURE_ONLY_ACTIVE:
        case PEDOMETER_ONLY_ACTIVE:
        case MEASURE_PEDOMETER_ACTIVE:
            MS_Position = position;
            break;

        default:
            break;
    }

    atomic_set(&MS_AutoMaticV, 1);
    return;

}
EXPORT_SYMBOL(AMI602_SetFlipInformation);

void AMI602Pedometer_ReStart(void)
{
    unsigned char i,j;
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;
    int nResult;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
    if (MS_PedoPause == MS_FLG_OFF) {
        return;
    }

    switch (MSState) {
        case PEDOMETER_ONLY_ACTIVE:
        case MEASURE_PEDOMETER_ACTIVE:
            for (i=0; i<10; i++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                    break;
                }
                mdelay(3);
            }

            for (i=0; i<16; i++) {
                ami602_ChangeActive();

                nResult = ami602_I2C_write(CLR_PED_SUSPEND, NULL, 0);
                if(nResult == 0){
                    break;
                }

                for (j=0; j<10; j++) {
                    if (gpio_get_value(SH_MS_IRQ) == 1) {
                        I2C_PermissionFlg = MS_FLG_ON;
                        break;
                    }
                    mdelay(3);
                }
            }
            break;

        default:
            break;
    }

    MS_PedoParam.CntData = 0;
    MS_PedoParam.TimeData = 0;
    MS_PedoPause = MS_FLG_OFF;
}
EXPORT_SYMBOL(AMI602Pedometer_ReStart);

void AMI602Pedometer_Pause(void)
{
    unsigned char i,j;
    unsigned char I2C_PermissionFlg = MS_FLG_OFF;
    int nResult;
    unsigned char pedodata[9];
    unsigned long Pre_PedometerCnt;
    unsigned long Pre_PedometerTime;

#if DEBUG
    printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
    if (MS_PedoPause == MS_FLG_ON) {
        return;
    }

    switch (MSState) {
        case PEDOMETER_ONLY_ACTIVE:
            for (i=0; i<10; i++) {
                if (gpio_get_value(SH_MS_IRQ) == 1) {
                    I2C_PermissionFlg = MS_FLG_ON;
                    break;
                }
                mdelay(3);
            }

            for (i=0; i<16; i++) {
                ami602_ChangeActive();

                nResult = ami602_I2C_read(GET_MES_PED_AUTO_SUSPEND, pedodata, 9);
                if(nResult == 0){
                    memcpy(Pedo_ReadData,pedodata,9);
                    break;
                }

                for (j=0; j<10; j++) {
                    if (gpio_get_value(SH_MS_IRQ) == 1) {
                        I2C_PermissionFlg = MS_FLG_ON;
                        break;
                    }
                    mdelay(3);
                }
                if( I2C_PermissionFlg == MS_FLG_OFF ){
                    return;
                }
            }
            Pre_PedometerCnt = (( Pedo_ReadData[0] << 24) & 0xFF000000) | (( Pedo_ReadData[1] << 16) & 0x00FF0000)
                                        | (( Pedo_ReadData[2] <<  8) & 0x0000FF00) |    Pedo_ReadData[3] ;
            Pre_PedometerTime = (( Pedo_ReadData[4] << 24) & 0xFF000000) | (( Pedo_ReadData[5] << 16) & 0x00FF0000)
                                        | (( Pedo_ReadData[6] <<  8) & 0x0000FF00) |    Pedo_ReadData[7] ;
            MS_PedoParam.StatusData = 0;
            if ((Pre_PedometerCnt > (MS_PedoParam.CntData + 30000) ) || (Pre_PedometerCnt < MS_PedoParam.CntData)) {
                return;
            }
            MS_PedoParam.KeepCnt += Pre_PedometerCnt;
            MS_PedoParam.KeepTime += Pre_PedometerTime;
            break;

        case MEASURE_PEDOMETER_ACTIVE:
            Pre_PedometerCnt = (( Pedo_ReadData[0] << 24) & 0xFF000000) | (( Pedo_ReadData[1] << 16) & 0x00FF0000)
                                        | (( Pedo_ReadData[2] <<  8) & 0x0000FF00) |    Pedo_ReadData[3] ;
            Pre_PedometerTime = (( Pedo_ReadData[4] << 24) & 0xFF000000) | (( Pedo_ReadData[5] << 16) & 0x00FF0000)
                                        | (( Pedo_ReadData[6] <<  8) & 0x0000FF00) |    Pedo_ReadData[7] ;
            MS_PedoParam.StatusData = 0;
            if ((Pre_PedometerCnt > (MS_PedoParam.CntData + 30000) ) || (Pre_PedometerCnt < MS_PedoParam.CntData)) {
                return;
            }
            MS_PedoParam.KeepCnt += Pre_PedometerCnt;
            MS_PedoParam.KeepTime += Pre_PedometerTime;
            break;

        default:
            return;
    }

    MS_PedoPause = MS_FLG_ON;
}
EXPORT_SYMBOL(AMI602Pedometer_Pause);

static const struct i2c_device_id ami602_id[] = {
    { "ami602", 0 },
    { }
};

static struct i2c_driver ami602_driver = {
    .class = I2C_CLASS_HWMON,
    .probe        = ami602_probe,
    .remove        = ami602_remove,
    .id_table = ami602_id,
    .driver = {
        .owner = THIS_MODULE,
        .name    = "ami602",
    },
};

static int __init ami602_init(void)
{
    int ret;

#if DEBUG
    printk(KERN_INFO "ami602 ms driver: initialize\n");
#endif
    ret = i2c_add_driver(&ami602_driver);
    
    atomic_set(&MS_AutoMaticV, 0);
    return ret;
}

static void __exit ami602_exit(void)
{
#if DEBUG
    printk(KERN_INFO "ami602 ms driver: release\n");
#endif
    i2c_del_driver(&ami602_driver);
}

module_init(ami602_init);
module_exit(ami602_exit);

MODULE_DESCRIPTION("ami602 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
