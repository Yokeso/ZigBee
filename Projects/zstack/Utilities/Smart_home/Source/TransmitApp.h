/**************************************************************************************************
    Filename:       TransmitApp.h
    Revised:        $Date: 2013-10-2 $
    Revision:       $ $

    Description:    This file contains the Transmit Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef TRANSMITAPP_H
#define TRANSMITAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define TRANSMITAPP_ENDPOINT           1

#define TRANSMITAPP_PROFID             0x0F05
#define TRANSMITAPP_DEVICEID           0x0001
#define TRANSMITAPP_DEVICE_VERSION     0
#define TRANSMITAPP_FLAGS              0
  
#define TRANSMITAPP_MAX_INCLUSTERS           8
#define TRANSMITAPP_CLUSTERID_HUMITMSG       1  // 温湿度
#define TRANSMITAPP_CLUSTERID_TEMPLIGHTMSG   2  // 温度光照
#define TRANSMITAPP_CLUSTERID_RFIDMSG        3  // 射频卡
#define TRANSMITAPP_CLUSTERID_GASFLAMEMSG    4  // 烟雾火焰
#define TRANSMITAPP_CLUSTERID_INFRAREDMSG    5  // 人体红外
#define TRANSMITAPP_CLUSTERID_SOUNDVBMSG     6  // 声音振动
#define TRANSMITAPP_CLUSTERID_MOTORSTATUSMSG 7  // 直流电机状态信息
#define TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG 8  // 继电器状态信息

  
#define TRANSMITAPP_MAX_OUTCLUSTERS        3
#define TRANSMITAPP_CLUSTERID_TESTMSG      9    // 测试预留
#define TRANSMITAPP_CLUSTERID_RELAYCTLMSG  10   // 继电器控制
#define TRANSMITAPP_CLUSTERID_MOTORCTLMSG  11   // 直流电机控制
  
// Application Events (OSAL) - These are bit weighted definitions.
#define TRANSMITAPP_SEND_MSG_EVT        0x0001  // 周期发送事件
#define TRANSMITAPP_RCVTIMER_EVT        0x0002  // 没有使用
#define TRANSMITAPP_SEND_ERR_EVT        0x0004  // 发送失败处理事件，本实验没有使用
#define TRANSMITAPP_MATCHRSP_EVT        0x0008  // 描述符匹配事件
#define TRANSMITAPP_DISPLAY_EVT         0x0010  // LCD显示事件
#define TRANSMITAPP_BINDRSP_EVT         0x0020  // 绑定事件，本实验没有使用
#define TRANSMITAPP_HALCHECK_EVT        0x0040  // 报警检测设备的定时查询事件
#define TRANSMITAPP_DEVICE_CHECK_EVT    0x0080  // 设备在线检测事件(协调器)
  
#define TRANSMITAPP_MOTOR_DATA_LEN        6     // 电机控制设备的数据长度
#define TRANSMITAPP_RELAY_DATA_LEN        5     // 继电器控制设备的数据长度
#define TRANSMITAPP_TEMPLIGHT_DATA_LEN    8     // 温度和光照度测量设备的数据长度
#define TRANSMITAPP_DHT11_DATA_LEN        8     // 温度和湿度测量设备的数据长度  
#define TRANSMITAPP_RC522_DATA_LEN        9     // RFID读卡器输出信息的数据长度
#define TRANSMITAPP_GASFLAME_DATA_LEN     5     // 烟雾与火焰检测设备的数据长度
#define TRANSMITAPP_D203S_DATA_LEN        5     // 红外热释电检测设备的数据长度 
#define TRANSMITAPP_SOUNDVB_DATA_LEN      5     // 声音振动与检测设备的数据长度 
  
/*********************************************************************
 * MACROS
 */

// Define to enable fragmentation transmit test
//#define TRANSMITAPP_FRAGMENTED

typedef struct deviceInfo
{
  //uint8 deviceID;
  uint8 deviceNWKStatus;
  uint8 data[5];
} deviceInfo_t;

// 设备在线、离线标志
#define  DEVICE_NWK_ONLINE   1
#define  DEVICE_NWK_OFFLINE  2
  
#define ZB_BINDING_ADDR               INVALID_NODE_ADDR
#define ZB_BROADCAST_ADDR             0xFFFF
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Transmit Application
 */
extern void TransmitApp_Init( byte task_id );

/*
 * Task Event Processor for the Transmit Application
 */
extern UINT16 TransmitApp_ProcessEvent( byte task_id, UINT16 events );

extern void TransmitApp_ChangeState( void );
extern void TransmitApp_SetSendEvt( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TRANSMITAPP_H */
