/**************************************************************************************************
  文件名：Smart_home.h
  作 者： 柳成林
  功 能： 设置了网关服务亲，路由器，终端节点都要遵循的相关宏与常量。
          具体设计需见更新日志
  更新日志(2021)
  3.13
  + 增加 设备在线状态相关宏
         备簇名称相关宏
         设备数据长度相关宏  （仅设定名称，具体长度需要更改）
  3.14
  + 修改 设备相关簇转化为输入与输出两部分
  + 添加 设备描述相关数据结构

  3.17
  + 增加 事件相关宏
         设备开始后的周期检查时间宏
  3.21
  + 修改 设备命名相关部分 （添加网关作为0号设备 ）
**************************************************************************************************/

#ifndef Smart_home_H
#define Smart_home_H

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

#define Smart_home_ENDPOINT           1

#define Smart_home_PROFID             0x0F05
#define Smart_home_DEVICEID           0x0001
#define Smart_home_DEVICE_VERSION     0
#define Smart_home_FLAGS              0

/*数据发送相关宏*/
/*设备簇名称*/
//输入部分   3.14
//3.21 DeviceList设置
#define Smart_home_MAX_INCLUSTERS                9
#define Smart_home_CLUSTERID_COORDINATOR         0  // 网关板
#define Smart_home_CLUSTERID_HUMITMSG            1  // 温湿度
#define Smart_home_CLUSTERID_TEMPLIGHTMSG        2  // 温度光照
#define Smart_home_CLUSTERID_RFIDMSG             3  // 射频卡
#define Smart_home_CLUSTERID_GASFLAMEMSG         4  // 烟雾火焰
#define Smart_home_CLUSTERID_INFRAREDMSG         5  // 人体红外
#define Smart_home_CLUSTERID_SOUNDVBMSG          6  // 声音振动
#define Smart_home_CLUSTERID_MOTORSTATUSMSG      7  // 直流电机状态信息
#define Smart_home_CLUSTERID_RELAYSTATUSMSG      8  // 继电器状态信息

//输出部分
#define Smart_home_MAX_OUTCLUSTERS               3  
#define Smart_home_CLUSTERID_TEXT                12  //测试用
#define Smart_home_CLUSTERID_MOTORCTRL           11  // 直流电机控制信息
#define Smart_home_CLUSTERID_RELAYCTRL           10  // 继电器控制信息  
 
/*数据发送长度*/
/*尚未完全确定*/
#define MSG_MAX_LEN                   102 //最大数据长度
#define HUMITMSG_LEN                  8  // 温湿度
#define TEMPLIGHTMSG_LEN              8  // 温度光照
#define RFIDMSG_LEN                   9  // 射频卡
#define GASFLAMEMSG_LEN               5  // 烟雾火焰
#define INFRAREDMSG_LEN               5  // 人体红外
#define SOUNDVBMSG_LEN                5  // 声音振动
#define MOTORSTATUSMSG_LEN            6  // 直流电机状态信息
#define RELAYSTATUSMSG_LEN            5  // 继电器状态信息
 
  
//3.17 事件编号
#define SMART_HOME_SEND_MSG_EVT        0x0001  // 周期发送事件
#define SMART_HOME_RCVTIMER_EVT        0x0002  // 没有使用
#define SMART_HOME_SEND_ERR_EVT        0x0004  // 发送失败处理事件
#define SMART_HOME_MATCHRSP_EVT        0x0008  // 描述符匹配事件
#define SMART_HOME_DISPLAY_EVT         0x0010  // LCD显示事件
#define SMART_HOME_BINDRSP_EVT         0x0020  // 绑定事件，
#define SMART_HOME_HALCHECK_EVT        0x0040  // 报警检测设备的定时查询事件
#define SMART_HOME_DEVICE_CHECK_EVT    0x0080  // 设备在线检测事件(协调器)

// OTA Flow Control Delays
#define Smart_home_ACK_DELAY          1
#define Smart_home_NAK_DELAY          16

// OTA Flow Control Status
#define OTA_SUCCESS                  ZSuccess
#define OTA_DUP_MSG                 (ZSuccess+1)
#define OTA_SER_BUSY                (ZSuccess+2)
  
/*********************************************************************
 * MACROS
 */
/*3.14 设备描述*/
typedef struct DeviceInfo
{
  uint8 deviceid;
  uint8 deviceStatus;
  uint8 data[5];
} DeviceInfo;  

/*设备在线状态*/
#define DEVICE_ONLINE                 1  //在线
#define DEVICE_OFFLINE                2  //设备不在线

/*每次检查设备的延时*/
#define SMART_HOME_DEVICE_CHECK_DELAY         5000
#define SMART_HOME_DEVICE_DISPLAY_DELAY       10000

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern byte Smart_home_TaskID;
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Smart_home Application
 */
extern void Smart_home_Init( byte task_id );

/*
 * Task Event Processor for the Smart_home Application
 */
extern UINT16 Smart_home_ProcessEvent( byte task_id, UINT16 events );

extern void Smart_home_ChangeState( void );
extern void Smart_home_SetSendEvt( void );
/*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* Smart_home_H */
