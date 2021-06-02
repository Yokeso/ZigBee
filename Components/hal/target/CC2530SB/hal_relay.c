//*****************************************************************************/
//
// 文 件 名：hal_relay.c
//
// 作    者: 柳成林
// 创建时间: 2021.4.6
//
// 说    明：继电器驱动程序，继电器模块由两个IO管脚驱动，P1_2(继电器K1)和P1_3(继电器K2)；
//           管脚置高电平，继电器模块开关打开，相反，管脚低电平，继电器模块关闭；
//                        
//
//*****************************************************************************/

#include "iocc2530.h"
#include "hal_types.h"
#include "hal_defs.h"

//*****************************************************************************/
#define  RELAY_MODULE_1_PIN_SET  P1DIR |= BV(2) 
#define  RELAY_MODULE_2_PIN_SET  P1DIR |= BV(3)
#define  RELAY_MODULE_1_TOGGLE   P1_2  ^= 1
#define  RELAY_MODULE_2_TOGGLE   P1_3  ^= 1

#define  RELAY_MODULE_1_PIN      P1_2
#define  RELAY_MODULE_2_PIN      P1_3

#define  RELAY_SWITCH_ON         1
#define  RELAY_SWITCH_OFF        0

//*****************************************************************************/
//本地变量
static uint8 relayStatus; // 继电器当前状态

//*****************************************************************************/
//函数名：HalRelayCtl
//功  能：继电器四种模式配置
//参  数：uint8 mode：继电器模式切换：
//              0x11：K1关闭(1), K2关闭(1)
//              0x12：K1打开(2), K2关闭(1)
//              0x21：K1关闭(1), K2打开(2)
//              0x22：K1打开(2), K2打开(2)
//              0x00：无任何操作
//返回值：无
//*****************************************************************************/
void HalRelayCtl(uint8 mode)
{
    RELAY_MODULE_1_PIN_SET;
    RELAY_MODULE_2_PIN_SET;
  
    // 继电器K1状态切换
    switch(mode & 0x0F)
    {
    case 0x02: // 打开
      {
        RELAY_MODULE_1_PIN =  RELAY_SWITCH_ON;
        relayStatus = (relayStatus&0xF0) | 0x02;
        break;
      }
    case 0x01: // 关闭
      {
        RELAY_MODULE_1_PIN =  RELAY_SWITCH_OFF;
        relayStatus = (relayStatus&0xF0) | 0x01;
        break;
      }
    default:
        break;
    }
    
    // 继电器K2状态切换
    switch(mode & 0xF0)
    {
    case 0x10: // 关闭
      {
        RELAY_MODULE_2_PIN =  RELAY_SWITCH_OFF;
        relayStatus = (relayStatus&0x0F) | 0x10;
        break;
      }
    case 0x20: // 打开
      {
        RELAY_MODULE_2_PIN =  RELAY_SWITCH_ON;
        relayStatus = (relayStatus&0x0F) | 0x20;
        break;
      }
    default:
        break;
    }
}

//*****************************************************************************/
//函数名：HalRelayStatus
//功  能：将存储的继电器的状态通过函数返回
//参  数：无
//返回值：返回继电器设置的状态
//*****************************************************************************/
uint8 HalRelayStatus()
{
    return relayStatus;
}