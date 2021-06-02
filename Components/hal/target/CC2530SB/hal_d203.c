//*******************************************************************************
//
//文 件 名: hal_d203.c
// 作    者: 柳成林

// 创建时间: 2021.3.10

//
//说    明：D203S热释电红外线传感器，本程序运行在SensorBoard底板，使用P1_1管脚
//          相连；管脚 输入低电平时，表示产生报警，即检测到人体红外信号。
//
//******************************************************************************/

#include <iocc2530.h>
#include "hal_defs.h"
#include "hal_types.h"

//*****************************************************************************/
//函数名：HalD203Init
//功  能：D203输出管脚配置
//参  数：无
//返回值：无
//*****************************************************************************/
void HalD203Init()
{
    P1DIR &= ~BV(1);
}

//*****************************************************************************/
//函数名：HalD203Check
//功  能：检测D203输出管脚信号
//参  数：无
//返回值：1：检测到报警信号；
//        0：没有报警信号
//*****************************************************************************/
uint8 HalD203Check()
{
    if (P1_1)
    {
        return 0;
    }
    else 
    {
        return 1;
    }
}
