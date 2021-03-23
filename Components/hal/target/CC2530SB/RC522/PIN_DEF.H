/*******************************************************************
*文 件 名：PIN_DEF.H
*作    者：yizedxl
*修改日期：2013.5.23
*******************************************************************/
//***************************************************************************
//                  
//         引脚功能定义            
//                  
//***************************************************************************
#ifndef __PIN_DEF_H__
#define __PIN_DEF_H__


#define LED_RED                BIT0  // P1.0 红灯亮起
#define RF_LPCTL               BIT2  // P1.2 射频卡休眠控制 ------RST   
#define RF_SS                  BIT3  // p1.3 射频卡从机选择(SS)---SDA
#define RF_SCLK                BIT4  // p1.4 射频卡数据时钟输出(SCLK)
#define RF_DATA_OUT            BIT5  // p1.5 射频卡数据输出(MOSI)
#define RF_DATA_IN             BIT6  // p1.6 射频模块输入(MISO) 

//****************************************************************************
//                   *
//             操作宏定义          *
//                   *
//****************************************************************************

//------------------------------- RC522操作宏 ------------------------------//
//#define    RF_POWER_ON             P8OUT &=~RF_PCTL        // 射频模块上电
//#define    RF_POWER_OFF            P8OUT |= RF_PCTL        // 射频模块下电
#define    RF_POWER_ON             asm("NOP")                // 射频模块上电
#define    RF_POWER_OFF            asm("NOP")                // 射频模块下电
#define    RED_LED_ON              P1_0 = 1        // 红灯亮起
#define    RED_LED_OFF             P1_0 = 0        // 红灯熄灭

#define    RF_SLEEP                P2_0 = 0        // 射频模块唤醒
#define    RF_WAKEUP               P2_0 = 1        // 射频模块休眠
#define    SCLK_HIGH               P1_5 = 1        // 数据时钟置高
#define    SCLK_LOW                P1_5 = 0        // 数据时钟置低
#define    DATA_OUT_HIGH           P1_6 = 1        // 数据置高
#define    DATA_OUT_LOW            P1_6 = 0        // 数据置低
#define    SS_HIGH                 P1_4 = 1        // 从机选择无效
#define    SS_LOW                  P1_4 = 0        // 从机选择有效
#define    DATA_IN                 P1_7            // 数据输入

#define    NSS522_1                SS_HIGH        // 从机选择无效       
#define    NSS522_0                SS_LOW        // 从机选择有效
#define    SCK522_1                SCLK_HIGH       // 数据时钟置高
#define    SCK522_0                SCLK_LOW        // 数据时钟置低
#define    SI522_1                 DATA_OUT_HIGH        // 数据置高        
#define    SI522_0                 DATA_OUT_LOW        // 数据置低        
#define    SO522                   DATA_IN            // 数据输入
#define    RST522_1                RF_WAKEUP 
#define    RST522_0                RF_SLEEP

#define    RC522_EA                P1_3 = 1
#define    RC522_I2C               P1_2 = 0

//------------------------------ 操作结果  ---------------------------------------//
#define    SUCCESS      0
#define    FAILURE      1
#define    CRC_ERROR    2


#endif