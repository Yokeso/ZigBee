/**************************************************************************************************
  文件名：Smart_home.c
  作 者： 柳成林
  功 能： 作为网关节点，实现传感器信息的采集，并可以通过其控制一些节点
          终端设备主要分为8个，分别为
     1、温度与光照度传感器；2、烟雾与火焰传感器；3、继电器；4、温湿度传感器；
     5、声音与震动传感器；  6、人体红外检测传感器；7、电机；8、RFID检测器。
  更新日志(2021)
  3.13
  + 新增 8种传感器的Project并设置好编译情况
  + 修改 Smart_home_ProcessEvent() （增加各种处理器的事件）
         Smart_home_Init()         （初始化内容修改，添加设备默认离线）
  3.14
  + 修改 Smart_home_ProcessMSGCmd()（添加终端节点类型判断，具体处理留空）
  + 新增 Hal层各种传感器驱动(头文件)
         ClusterList (添加8个终端，分开出与入)
  3.17
  + 修改 Smart_home_ProcessMSGCmd() （完成接收后对于各终端处理）
  + 添加 Smart_home_HandleKeys()     (修改原来函数增添概念，未补全)
  + 删除 static void Smart_home_Send(void);
         static void Smart_home_Resp(void);
         两个函数
  3.19
  + 检查 出现的warning并做标识
  + 修改 Smart_home_HandleKeys() （补全设计函数）
  + 新增 Smart_home_Device_check() (设备是否在线检查)

  3.21
  + 新增 DeviceList以方便初始化和设备在线检查（本地变量部分）
         Smart_home_Display()    （屏幕显示，未补全）
         Smart_home_Key_add()     (控制按键，附加项)
  + 修改 Smart_home_Device_check()
         Smart_home_Init()
         Smart_home_ProcessMSGCmd()
         调整有关DeviceList（设备列表）部分的修改
         直接使用数组位置号作为设备编号
         按键控制方式，用按键情况控制
  3.22
  + 修改 Smart_home_Display()    （屏幕显示，补全）
         Smart_home_Key_add()     (屏幕显示中的电机与继电器部分加入这里)
         重写别名方式减少内存损耗
         出现无法初始化的问题，需要项目重构
  3.23
  + Warn 代码初始化后会产生直接进入key的情况,需debug
         OK键问题无法修复，放弃使用OK键，修改为Cencel组网 禁止组网
  3.23 正式完成网关节点部分，（显示函数部分除外）
  3.24 
  + 修改 Smart_home_Key_add()与display函数整合
  + bug:在每十秒一次调用的周期检查函数中会导致显示屏将近一秒时间不显示
    已解决： 在状态0的时候作为在线即可
  + 完成 显示功能设置
   
  经调试程序无bug，可以接收各个传感器消息。//3.22
  当我没说。。好像出了内存问题//3.22
  项目重构啦！，现在又只有一个小问题了/3.23
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "nwk_util.h"

#include "Smart_home.h"
#include "OnBoard.h"

#include "DebugTrace.h"
#include "string.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/*********************************************************************
 * MACROS
 */
/*设备链接相关设置*/
/*判断设备是否在线的最大次数*/
#define DEVICE_HEART_BEAT    3
#define DEVICE_CHECK_DELAY   10000

/*LCD相关设置*/
/*单屏显示时常   MS*/
#define LCD_DISPLAY_LENGTH   10000
#define LCD_DISPLAY_TIMER    1000  //多久更新一次 

/*关于显示部分相关的宏*/
#define LCD_PAGE_MAX         4     //目前目录页最多4页

/*********************************************************************
 * CONSTANTS
 */

#if !defined( SMART_HOME_PORT )
#define SMART_HOME_PORT  0
#endif

#if !defined( SMART_HOME_BAUD )
#define SMART_HOME_BAUD  HAL_UART_BR_38400
//#define SMART_HOME_BAUD  HAL_UART_BR_115200
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SMART_HOME_THRESH )
#define SMART_HOME_THRESH  64
#endif

#if !defined( SMART_HOME_RX_SZ )
#define SMART_HOME_RX_SZ  128
#endif

#if !defined( SMART_HOME_TX_SZ )
#define SMART_HOME_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SMART_HOME_IDLE )
#define SMART_HOME_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SMART_HOME_LOOPBACK )
#define SMART_HOME_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SMART_HOME_TX_MAX )
#define SMART_HOME_TX_MAX  80
#endif

#define SMART_HOME_RSP_CNT  4

//3.14 This list should be filled with Application specific Cluster IDs.
const cId_t Smart_home_ClusterList_IN[Smart_home_MAX_INCLUSTERS ] =
{
  Smart_home_CLUSTERID_HUMITMSG,              // 温湿度
  Smart_home_CLUSTERID_TEMPLIGHTMSG,          // 温度光照
  Smart_home_CLUSTERID_RFIDMSG,               // 射频卡
  Smart_home_CLUSTERID_GASFLAMEMSG,           // 烟雾火焰
  Smart_home_CLUSTERID_INFRAREDMSG,           // 人体红外
  Smart_home_CLUSTERID_SOUNDVBMSG,            // 声音振动
  Smart_home_CLUSTERID_MOTORSTATUSMSG,        // 直流电机状态信息
  Smart_home_CLUSTERID_RELAYSTATUSMSG         // 继电器
};

const cId_t Smart_home_ClusterList_OUT[Smart_home_MAX_INCLUSTERS ] =
{
  Smart_home_CLUSTERID_MOTORCTRL,             //直流电机控制
  Smart_home_CLUSTERID_RELAYCTRL              //继电器控制
};

const SimpleDescriptionFormat_t Smart_home_SimpleDesc =
{
  Smart_home_ENDPOINT,              //  int   Endpoint;
  Smart_home_PROFID,                //  uint16 AppProfId[2];
  Smart_home_DEVICEID,              //  uint16 AppDeviceId[2];
  Smart_home_DEVICE_VERSION,        //  int   AppDevVer:4;
  Smart_home_FLAGS,                 //  int   AppFlags:4;
  Smart_home_MAX_INCLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Smart_home_ClusterList_IN,  //  byte *pAppInClusterList;
  Smart_home_MAX_OUTCLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)Smart_home_ClusterList_OUT   //  byte *pAppOutClusterList;
};

const endPointDesc_t Smart_home_epDesc =
{
  Smart_home_ENDPOINT,
  &Smart_home_TaskID,
  (SimpleDescriptionFormat_t *)&Smart_home_SimpleDesc,
  noLatencyReqs
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 Smart_home_TaskID;    // Task ID for internal task/event processing.

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*3.18 终端节点的设备详细信息缓存*/
static uint8 DeviceCnt[Smart_home_MAX_INCLUSTERS];
static int8 Ctrlcase = 0;    //0用来控制屏幕显示，1控制继电器，2控制电机
static int8 LCD_Page  =  0;  //终端状态显示
DeviceInfo DeviceList[Smart_home_MAX_INCLUSTERS];                      //设备列表  
/*********************************************************************************
//这些似乎会引起内存问题  3.22
DeviceInfo *Humit = DeviceList+Smart_home_CLUSTERID_HUMITMSG;          //温湿度缓存
DeviceInfo *TempLight = DeviceList+Smart_home_CLUSTERID_TEMPLIGHTMSG;  // 温度光照缓存
DeviceInfo *RfID = DeviceList+Smart_home_CLUSTERID_RFIDMSG;            // RFID 信息缓存
DeviceInfo *gasFlame = DeviceList+Smart_home_CLUSTERID_GASFLAMEMSG;    //气体火焰缓存
DeviceInfo *infrared = DeviceList+Smart_home_CLUSTERID_INFRAREDMSG;    //人体红外
DeviceInfo *motor = DeviceList+Smart_home_CLUSTERID_MOTORSTATUSMSG;     //电机状态
DeviceInfo *relay = DeviceList+Smart_home_CLUSTERID_RELAYSTATUSMSG;    //继电器状态
DeviceInfo *soundVb = DeviceList+Smart_home_CLUSTERID_SOUNDVBMSG;      //声音震动
修改如下
***********************************************************************************/
#define Humit     1     //温湿度缓存
#define TempLight 2     // 温度光照缓存
#define RfID      3     // RFID 信息缓存
#define gasFlame  4     //气体火焰缓存
#define infrared  5     //人体红外
#define motor     6     //电机状态
#define relay     7     //继电器状态
#define soundVb   8     //声音震动

/*3.14 消息发送部分*/
/*3.17短地址存储*/
static afAddrType_t Relay_addr;
static afAddrType_t Motor_addr;

/*消息发送数组*/
byte Coordinator_Msg[MSG_MAX_LEN];


/*3.19 协议栈中有但暂时不知道要他们干嘛的东西*/
/*3.21 现在我会了！！！*/
/*3.22 那些没用！删掉了！*/
//static uint8 Smart_home_MsgID;
static uint8 RelayTransID;  // This is the unique message ID (counter)
static uint8 MotorTransID;  // This is the unique message ID (counter)

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void Smart_home_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
//static void Smart_home_Key_add(uint8 Ctrlcase);
static void Smart_home_HandleKeys( byte shift, byte keys );
static void Smart_home_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
static void Smart_home_Device_check(void);
static void Smart_home_Relay_Ctl(uint8 cmd);
static void Smart_home_Motor_Ctl(uint8 cmd,uint8 speed);
static void Smart_home_Display(void);

/*********************************************************************
 * @fn      Smart_home_Init
 *
 * @brief   This is called during OSAL tasks' initialization.
 *
 * @param   task_id - the Task ID assigned by OSAL.
 *
 * @return  none
 */
void Smart_home_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  uint8 DeviceNum;                                    //DeviceList初始化用变量

  Smart_home_TaskID = task_id;
  MotorTransID = 0;
  RelayTransID = 0;

  afRegister( (endPointDesc_t *)&Smart_home_epDesc );

  RegisterForKeys( task_id );

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SMART_HOME_BAUD;
  uartConfig.flowControl          = TRUE;
  uartConfig.flowControlThreshold = SMART_HOME_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SMART_HOME_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SMART_HOME_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SMART_HOME_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  //uartConfig.callBackFunc         = Smart_home_CallBack;
  HalUARTOpen (SMART_HOME_PORT, &uartConfig);

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Smart_home", HAL_LCD_LINE_2 );
#endif
  
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, Match_Desc_rsp );
  
  //3.22 显示延时
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DISPLAY_EVT, 
                      SMART_HOME_DEVICE_DISPLAY_DELAY);
  
  
  //3.14 打开设备在线检测，第一次开启检测延时较长时间
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DEVICE_CHECK_EVT, 
                      SMART_HOME_DEVICE_CHECK_DELAY);
 
  //3.13 关闭LED灯(D4)，表示协调器默认不允许组网
  NLME_PermitJoiningRequest(0x00);
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  
  //3.21 初始化页面为0 第一页
  //LCD_Page=0;
  
  //3.13 设备离线状态检测初始化，初始化为离线
  //3.21 修改为DeviceList方式，初始化DeviceCnt
  for(DeviceNum=1;DeviceNum<Smart_home_MAX_INCLUSTERS;DeviceNum++) //只初始化终端
  {
    DeviceList[DeviceNum].deviceid = DeviceNum;
    DeviceList[DeviceNum].deviceStatus = DEVICE_OFFLINE;
    DeviceCnt[DeviceNum]=0;
  }
}

/*********************************************************************
 * @fn      Smart_home_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events   - Bit map of events to process.
 *
 * @return  Event flags of all unprocessed events.
 */
UINT16 Smart_home_ProcessEvent( uint8 task_id, UINT16 events )
{
  (void)task_id;  // Intentionally unreferenced parameter
  afDataConfirm_t *afDataConfirm;
  
  //3.13 数据确认消息字段
  ZStatus_t sentStatus;
  byte sentEP;
  
  if ( events & SYS_EVENT_MSG )   //3.13 系统消息事件
  {
    afIncomingMSGPacket_t *MSGpkt;

    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Smart_home_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:           //3.13 ZDO传入消息回调
          Smart_home_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
          
        case KEY_CHANGE:           //3.13 按键事件
          Smart_home_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
         
        case AF_DATA_CONFIRM_CMD:
        //3.13 接收到此消息，作为对发送的数据包的确认。
        //状态为ZStatus_t类型[在ZComDef.h中定义]
        //消息字段在AF.h中定义
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
 
          if ( (ZSuccess == sentStatus) &&
               (Smart_home_epDesc.endPoint == sentEP) )
          {  
            //3.13 在消息发送确认成功后绿灯闪烁一下
            HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
          }
          else
          {
            //3.13 红灯闪烁一下  硬件还没实现
            //HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK);
          }
          break;
          
        case AF_INCOMING_MSG_CMD:   //3.13 消息传入处理
          //3.13 在消息接收确认成功后绿灯闪烁一下
          Smart_home_ProcessMSGCmd( MSGpkt );
          break;
        default:
          break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
      
      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Smart_home_TaskID );
    }
    
    // Squash compiler warnings until values are used.
    (void)sentStatus;
    (void)sentEP;
    
    return ( events ^ SYS_EVENT_MSG );
  }

  if ( events & SMART_HOME_SEND_MSG_EVT )
  {
    //Smart_home_Send();
    return ( events ^ SMART_HOME_SEND_MSG_EVT );
  }
  
  if ( events & SMART_HOME_DEVICE_CHECK_EVT )  //终端设备检查事件
  {
    Smart_home_Device_check();  //调用终端设备检查函数
    
    osal_start_timerEx(Smart_home_TaskID,SMART_HOME_DEVICE_CHECK_EVT,
                       DEVICE_CHECK_DELAY);
    return (events ^ SMART_HOME_DEVICE_CHECK_EVT);
  }
  
  
  if( events & SMART_HOME_DISPLAY_EVT )
  {
    Smart_home_Display();
    //周期性刷新
    osal_start_timerEx( Smart_home_TaskID,events & SMART_HOME_DISPLAY_EVT,LCD_DISPLAY_TIMER);
    return (events ^ SMART_HOME_DISPLAY_EVT);
  }

  return ( 0 );  // Discard unknown events.
}

/*********************************************************************
 * @fn      Smart_home_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;
      
    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            /*
            Smart_home_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
            Smart_home_TxAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            Smart_home_TxAddr.endPoint = pRsp->epList[0];
            */
            
            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      Smart_home_Key_add
 *
 * @brief   3.21新增，用来标识目前处于的控制状态，是附加项
 *          0 控制屏幕 1 控制继电器 2 控制电机
 *            
 *
 * @param   Ctrlcase 目前的控制号
 *
 * @return  屏幕回显
 *
static void Smart_home_Key_add(uint8 Ctrlcase)
{
#if defined ( LCD_SUPPORTED )
  switch(Ctrlcase)
  {
    default:    
    case 0:
      HalLcdWriteString( "Flip use UP/DOWN", HAL_LCD_LINE_4 ); 
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      break;
    
    case 1:
      HalLcdWriteString( "Relay Contrling", HAL_LCD_LINE_4 ); 
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      if(DeviceList[relay].deviceStatus == DEVICE_ONLINE)  //设备在线回显
      {
        HalLcdWriteString( "Relay Online", HAL_LCD_LINE_1 ); 
      }
      if(DeviceList[relay].deviceStatus != DEVICE_ONLINE)
      {
        HalLcdWriteString( "Relay Offline", HAL_LCD_LINE_1 );      
      }
      break;
    
    case 2:
      HalLcdWriteString( "Motor Contrling", HAL_LCD_LINE_4 );
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      if(DeviceList[motor].deviceStatus == DEVICE_ONLINE)  //设备在线回显
      {
        HalLcdWriteString( "Motor Online", HAL_LCD_LINE_1 );
      }
      if(DeviceList[motor].deviceStatus != DEVICE_ONLINE)
      {
        HalLcdWriteString( "Motor Offline", HAL_LCD_LINE_1 );      
      }      
      break;
#endif     
  }
}
*/
/*********************************************************************
 * @fn      Smart_home_HandleKeys
 *
 * @brief   设置按键事件
 *          目前事件设置    3.19
 *
 *          HAL_KEY_SW_1--UP     (上翻一页)
 *          HAL_KEY_SW_2--RIGHT  (选项向右)
 *          HAL_KEY_SW_3--DOWN   (下翻一页)
 *          HAL_KEY_SW_4--LEFT   (选项向左)
 *          HAL_KEY_SW_5--OK     (组网开)
 *          HAL_KEY_SW_7--CANCEL (组网关) 
 * + 3.21 新增  选项总共有3种，控制屏幕显示，控制继电器和控制电机
 *              控制继电器时 UP控制1，DOWN控制2
 *              控制电机时   UP加速， DOWN减速
 *
 * @param   shift - true if in shift/alt.
 * @param   keys  - bit field for key events.
 *
 * @return  none
 */
void Smart_home_HandleKeys( byte shift, byte keys )
{
  //zAddrType_t txAddr;
  static int8 MotorSpeed = 0;
  static uint8 NetWorkAllow = 0;
  /*这里这么设置似乎会出现内存问题
  static uint8 Relay1_on = 0x02;  //两个默认都是关闭
  static uint8 Relay2_on = 0x10;
  */
  /*3.21 LCD 显示*/
  static uint8 Relay1_on = 0;  //两个默认都是关闭
  static uint8 Relay2_on = 0;
  
  
  if ( keys & HAL_KEY_SW_1 )  //UP
  {
    switch(Ctrlcase)
    {
      default:    
      case 0:
      {
        if(LCD_Page < LCD_PAGE_MAX)       {LCD_Page++;}
        if(LCD_Page > LCD_PAGE_MAX - 1)       {LCD_Page = 0;}
        break;
      }
      case 1:
      {  
        uint8 switch1;
        if(Relay1_on == 0)  {Relay1_on = 1 ; switch1 = 0x02;}
        else if(Relay1_on == 1)  {Relay1_on = 0; switch1 = 0x01;}
        else  {Relay1_on = 0; switch1 = 0x01;}       
        Smart_home_Relay_Ctl(switch1);  
        break;
      } 
      case 2:
      {
        if(MotorSpeed < 80)     {MotorSpeed += 10;}
        if(MotorSpeed >= 80)     
        {
#if defined ( LCD_SUPPORTED )
           HalLcdWriteString( "Motor max Speed", HAL_LCD_LINE_4 );
#endif 
           MotorSpeed = 80;
        }
        break;
      }
         
    }    
  }
  
  if ( keys & HAL_KEY_SW_3 )  //DOWN
  {
    switch(Ctrlcase)
    {
      default:    
      case 0:
      {
        if(LCD_Page >= 0)       {LCD_Page--;}
        if(LCD_Page < 0)        {LCD_Page = LCD_PAGE_MAX;}
        break;
      }
      case 1:  
      {
        uint8 switch2;
        if(Relay2_on == 0)  {Relay2_on = 1 ; switch2 = 0x20;}
        else if(Relay2_on == 1)  {Relay2_on = 0; switch2 = 0x10;}
        else  {Relay2_on = 0; switch2 = 0x01;}       
        Smart_home_Relay_Ctl(switch2);       
        break;
      } 
      case 2:
      {
        if(MotorSpeed > -80)     {MotorSpeed -= 10;}
        if(MotorSpeed <= -80)     
        {
#if defined ( LCD_SUPPORTED )
           HalLcdWriteString( "Motor min Speed", HAL_LCD_LINE_4 );
#endif 
           MotorSpeed = -80;
        }
        break; 
      }
    }
  }
  
  if ( keys & HAL_KEY_SW_2 )  //RIGHT  
  /*3.22 这里在调试时出现问题，具体问题为 点击up时会出现屏幕变为禁止组网
    控制方面变为控制电机，问题在于汇编中的key值会出现  keys & HAL_KEY_SW_2 = 2
    的情况，暂时禁止，需要等待研究  还有key_7*/
  {
     if(Ctrlcase < 3)   { Ctrlcase++;}
     if(Ctrlcase > 2)   { Ctrlcase=0;}
     //Smart_home_Key_add(Ctrlcase);
  }
  
  if ( keys & HAL_KEY_SW_4 )  //LEFT
  {
     if(Ctrlcase > -1)   { Ctrlcase--;}
     if(Ctrlcase < 0)    { Ctrlcase=2;}
     //Smart_home_Key_add(Ctrlcase);    
  }
  
  if ( keys & HAL_KEY_SW_5 )  //OK
  {

  }
  
  if ( keys & HAL_KEY_SW_7 )  //CENCEL
  {
     if(NetWorkAllow == 0)
     {
       NetWorkAllow = 1;
       NLME_PermitJoiningRequest(0xFF); // 组网，允许随时加入
       HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Allow networking", HAL_LCD_LINE_4 );
#endif
     }
     else
     {
       NetWorkAllow = 0;
       NLME_PermitJoiningRequest(0x00); // 不允许组网
       HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);  
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Ban   networking", HAL_LCD_LINE_4 );
#endif        
     }
  }
}

/*********************************************************************
 * @fn      Smart_home_ProcessMSGCmd
 *
 * @brief   Data message processor callback. This function processes
 *          any incoming data - probably from other devices. Based
 *          on the cluster ID, perform the intended action.
 *
 * @param   pkt - pointer to the incoming message packet
 *
 * @return  TRUE if the 'pkt' parameter is being used and will be freed later,
 *          FALSE otherwise.
 */
void Smart_home_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
  /***********************************************************************
    目前总共8种case
   #define Smart_home_CLUSTERID_HUMITMSG            1  // 温湿度
   #define Smart_home_CLUSTERID_TEMPLIGHTMSG        2  // 温度光照
   #define Smart_home_CLUSTERID_RFIDMSG             3  // 射频卡
   #define Smart_home_CLUSTERID_GASFLAMEMSG         4  // 烟雾火焰
   #define Smart_home_CLUSTERID_INFRAREDMSG         5  // 人体红外
   #define Smart_home_CLUSTERID_SOUNDVBMSG          6  // 声音振动
   #define Smart_home_CLUSTERID_MOTORSTATUSMSG      7  // 直流电机状态信息
   #define Smart_home_CLUSTERID_RELAYSTATUSMSG      8  // 继电器状态信息
    
   设备描述
   typedef struct DeviceInfo
   {
     uint8 deviceID;
     uint8 deviceStatus;
     uint8 data[5];
   } DeviceInfo; 
   ************************************************************************/ 
    //3.14 消息处理模块，共8个，行为类似 
    //3.21 修改为指针方法（DeviceList相关）
    case Smart_home_CLUSTERID_HUMITMSG:         // 温湿度
      DeviceList[Humit].deviceStatus = DEVICE_ONLINE;       //收到消息设置为在线
      DeviceList[Humit].data[0] = pkt->cmd.Data[4]; // 湿度 
      DeviceList[Humit].data[1] = pkt->cmd.Data[5]; // 温度
      break;
      
    case Smart_home_CLUSTERID_TEMPLIGHTMSG:     // 温度光照
      DeviceList[TempLight].deviceStatus = DEVICE_ONLINE;   //收到消息设置为在线
      DeviceList[TempLight].data[0] = pkt->cmd.Data[4]; // 温度整数
      DeviceList[TempLight].data[1] = pkt->cmd.Data[5]; // 温度小数
      DeviceList[TempLight].data[2] = pkt->cmd.Data[6]; // 光照
      DeviceList[TempLight].data[3] = pkt->cmd.Data[7]; // 光照
      break;
      
    case Smart_home_CLUSTERID_RFIDMSG:          // 射频卡
      DeviceList[RfID].deviceStatus = DEVICE_ONLINE;        //收到消息设置为在线
      DeviceList[RfID].data[0] = pkt->cmd.Data[4]; // 射频卡类型
      DeviceList[RfID].data[1] = pkt->cmd.Data[5]; // 4个字节的ID号
      DeviceList[RfID].data[2] = pkt->cmd.Data[6]; //
      DeviceList[RfID].data[3] = pkt->cmd.Data[7]; //
      DeviceList[RfID].data[4] = pkt->cmd.Data[8]; //  
      break;
      
    case Smart_home_CLUSTERID_GASFLAMEMSG:      // 烟雾火焰
      DeviceList[gasFlame].deviceStatus = DEVICE_ONLINE;    //收到消息设置为在线
      DeviceList[gasFlame].data[0] = pkt->cmd.Data[4]; // 烟雾与火焰报警信息
      break;
      
    case Smart_home_CLUSTERID_INFRAREDMSG:      // 人体红外
      DeviceList[infrared].deviceStatus = DEVICE_ONLINE;    //收到消息设置为在线
      DeviceList[infrared].data[0] = pkt->cmd.Data[4]; // 人体红外 
      break;

    case Smart_home_CLUSTERID_SOUNDVBMSG:       // 声音振动
      DeviceList[soundVb].deviceStatus = DEVICE_ONLINE;     //收到消息设置为在线
      DeviceList[soundVb].data[0] = pkt->cmd.Data[4]; // 声音震动信息
      break;
      
    case Smart_home_CLUSTERID_MOTORSTATUSMSG:   // 直流电机状态信息
      DeviceList[motor].deviceStatus = DEVICE_ONLINE;       //收到消息设置为在线
      // 储存电机设备的网络地址，用于发送控制命令
      Motor_addr.addrMode = (afAddrMode_t)Addr16Bit;
      Motor_addr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Motor_addr.endPoint = 1;  // 目的节点的端口号
      //TransmitApp_DstMotorAddr->endPoint = TRANSMITAPP_ENDPOINT;
     
      DeviceList[motor].data[0] = pkt->cmd.Data[4]; // 电机转速
      DeviceList[motor].data[1] = pkt->cmd.Data[5]; // 电机状态
      break;
      
    case Smart_home_CLUSTERID_RELAYSTATUSMSG:   // 继电器状态信息
      DeviceList[relay].deviceStatus = DEVICE_ONLINE;       //收到消息设置为在线    
      
      // 储存继电器设备的网络地址，用于发送控制命令
      Relay_addr.addrMode = (afAddrMode_t)Addr16Bit;
      Relay_addr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;    
      Relay_addr.endPoint = 1; // 目的节点的端口号
      DeviceList[relay].data[0] = pkt->cmd.Data[4]; 
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn     Smart_home_Device_check
 *
 * @brief   设备计数器，用来检查设备在开始组网后多久在线.
 *          目前搜索为4次HeartBeat  (DEVICE_HEART_BEAT 3)
 *          3.21
 * @param   none
 *
 * @return  none
 */
/*3.24 这里的检测时常有点长 改为变量外提的方式加快处理速度*/
static void Smart_home_Device_check(void)
{
  static uint8 DeviceID;
  static uint8 Device_status =  0;
  static uint8* counttmp;
  for(DeviceID=1;DeviceID<Smart_home_MAX_INCLUSTERS;DeviceID++)
  {  
    Device_status = DeviceList[DeviceID].deviceStatus;
    counttmp = & DeviceCnt[DeviceID];
    if(Device_status != DEVICE_ONLINE)    //设备离线
    {
      (*counttmp)++;
    }
    if(Device_status == DEVICE_ONLINE)    //设备在线
    {
      *counttmp = 0;
      Device_status = 0;
    }
    if((*counttmp) > DEVICE_HEART_BEAT)
    {
      *counttmp = DEVICE_HEART_BEAT;
      Device_status = DEVICE_OFFLINE;
    }
    DeviceList[DeviceID].deviceStatus = Device_status;
  }
  /*温湿度缓存 温度光照缓存 RFID 信息缓存 气体火焰缓存 
  人体红外 电机状态 继电器状态 声音震动*/
}


/*********************************************************************
 * @fn      Smart_home_Relay_Ctl
 *
 * @brief   发送继电器控制消息
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Relay_Ctl(uint8 cmd)
{
  uint8 tmp;
  
  // 只有设备在线时, 方发送控制命令
  if (DeviceList[relay].deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // 发送给继电器的控制命令 
    Coordinator_Msg[4] = cmd;
    /********************************************
    *   屏幕显示部分*
#if defined ( LCD_SUPPORTED )
    if(cmd == 0x02) {HalLcdWriteString( "K1:ON", HAL_LCD_LINE_2 );}
    if(cmd == 0x01) {HalLcdWriteString( "K1:OFF", HAL_LCD_LINE_2 );}
    if(cmd == 0x20) {HalLcdWriteString( "K2:ON", HAL_LCD_LINE_3 );}
    if(cmd == 0x10) {HalLcdWriteString( "K2:OFF", HAL_LCD_LINE_3 );}
#endif     
    ********************************************/
    tmp = AF_DataRequest( &Relay_addr,                         
                          (endPointDesc_t *)&Smart_home_epDesc,                  
                           Smart_home_CLUSTERID_RELAYCTRL,
                           RELAYSTATUSMSG_LEN,                 
                           Coordinator_Msg,                    
                          &RelayTransID,                       
                           AF_DISCV_ROUTE,                     
                           AF_DEFAULT_RADIUS );
  } 
}

/*********************************************************************
 * @fn      Smart_home_Motor_Ctl
 *
 * @brief   发送电机控制消息
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Motor_Ctl(uint8 cmd,uint8 speed)
{
  uint8 tmp;
  
  // 只有设备在线时, 方发送控制命令
  if (DeviceList[motor].deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // 发送给继电器的控制命令 
    Coordinator_Msg[4] = speed;
    Coordinator_Msg[5] = cmd;
    
    tmp = AF_DataRequest( &Motor_addr,                         
                          (endPointDesc_t *)&Smart_home_epDesc,                  
                           Smart_home_CLUSTERID_MOTORCTRL,
                           MOTORSTATUSMSG_LEN,                 
                           Coordinator_Msg,                    
                          &MotorTransID,                       
                           AF_DISCV_ROUTE,                     
                           AF_DEFAULT_RADIUS );
  }   
}


/*********************************************************************
 * @fn      Smart_home_Display
 *
 * @brief   屏幕回显函数，定义了内容如何显示.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void Smart_home_Display(void)
{
#if defined LCD_SUPPORTED
  switch(Ctrlcase)
  {
    default:    
    case 0:
      HalLcdWriteString( "      Menu      ", HAL_LCD_LINE_1 );
      //HalLcdWriteString( "Flip use UP/DOWN", HAL_LCD_LINE_4 ); 
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      
      switch(LCD_Page)
      {
        default:
        case 0:
        {
          static uint16 humit;
          static uint16 temper;
          static uint16 SoundVb; 
          
          DeviceInfo Devhum = DeviceList[Humit];
          DeviceInfo DevSound = DeviceList[soundVb];
          //第一页第二行显示温度
          //      第三行显示光照
          humit = (uint16)Devhum.data[0];
          temper = (uint16)Devhum.data[1];
          SoundVb = (uint16)DevSound.data[0];

          HalLcdWriteStringValueValue( "Hum:", humit, 10, "% Tem_1:", temper, 10, HAL_LCD_LINE_2 );
          
          if(SoundVb & 0x01 == 0x01) { HalLcdWriteString( "Sound: Voice", HAL_LCD_LINE_3 ); }
          else if(SoundVb & 0x02 == 0x02) { HalLcdWriteString( "Sound: Vibration", HAL_LCD_LINE_3 ); }
          else if(SoundVb & 0x03 == 0x03) { HalLcdWriteString( "Sound: All", HAL_LCD_LINE_3 ); }
          else {HalLcdWriteString( "Sound: None", HAL_LCD_LINE_3 );}
          
          //HalLcdWriteStringValue( "Sound:", SoundVb, 16, HAL_LCD_LINE_3 );
          
          
          //显示最下面的百分比条
          static uint8 percent;
          percent = (1 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
        }
        case 1:
        {
          static uint16 Light;
          static uint8 lightmp;
          static uint16 integer;
          static uint16 decimals;
          DeviceInfo Devtmp = DeviceList[TempLight];
          //第一页第二行显示温度
          //      第三行显示光照
          integer = (uint16)Devtmp.data[0];
          decimals = (uint16)Devtmp.data[1];
          lightmp = Devtmp.data[3];
          Light = (uint16)Devtmp.data[4];
          memcpy(&Light,&lightmp,sizeof(lightmp));
          
          HalLcdWriteStringValueValue( "Temper_2:", integer, 10, ".", decimals, 10, HAL_LCD_LINE_2 );
          HalLcdWriteStringValue( "Light:", Light, 10, HAL_LCD_LINE_3 );
          
          //显示最下面的百分比条
          static uint8 percent;
          percent = (2 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
        }
        
      case 2:
      {
          static uint8 kind;
          static uint8 tmp1,tmp2;
          static uint8 tmp3,tmp4;
          static uint16 Data1;
          static uint16 Data2;
          //显示最下面的百分比条
          DeviceInfo rfid = DeviceList[RfID];
          
          kind = rfid.data[0];
          tmp1 = rfid.data[1];
          tmp2 = rfid.data[2];
          tmp3 = rfid.data[3];
          tmp4 = rfid.data[4];
          
          Data1 = (tmp1 << 8) | tmp2;
          Data2 = (tmp3 << 8) | tmp4;
          
          if(kind == 0x01) {HalLcdWriteString( "MFOne-S50", HAL_LCD_LINE_2 );}
          else if(kind == 0x02) {HalLcdWriteString( "MFOne-S70", HAL_LCD_LINE_2 );}
          else if(kind == 0x03) {HalLcdWriteString( "MF-UltraLight", HAL_LCD_LINE_2 );}
          else if(kind == 0x04) {HalLcdWriteString( "MF-Pro", HAL_LCD_LINE_2 );}
          else if(kind == 0x05) {HalLcdWriteString( "MF-DesFire", HAL_LCD_LINE_2 );}
          else {HalLcdWriteString( "No Card", HAL_LCD_LINE_2 );}
          

          HalLcdWriteStringValueValue( "ID: ", Data1, 16, "-", Data2, 16, HAL_LCD_LINE_3 );
          
          static uint8 percent;
          percent = (3 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
      }
      
      case 3:
      {
          static uint16 GasF;
          static uint16 Infrared;         

          DeviceInfo DevGas = DeviceList[GasF];
          DeviceInfo DevInf = DeviceList[infrared]; 
          
          GasF = (uint16)DevGas.data[0];
          Infrared = (uint16)DevInf.data[0];
          
          if(GasF & 0x01 == 0x01) { HalLcdWriteString( "GasFlame: Flame", HAL_LCD_LINE_2 ); }         //0位是火焰
          else if(GasF & 0x02 == 0x02) { HalLcdWriteString( "GasFlame: Gas", HAL_LCD_LINE_2 ); }//1位是气体
          else if(GasF & 0x03 == 0x03) { HalLcdWriteString( "GasFlame: All", HAL_LCD_LINE_2 ); }
          else {HalLcdWriteString( "GasFlame: None", HAL_LCD_LINE_2 );}
          
          //HalLcdWriteStringValue("GasFlame: ", GasF, 16, HAL_LCD_LINE_2 );
          HalLcdWriteStringValue("Infrared: ", infrared, 10, HAL_LCD_LINE_3 );
          
          //显示最下面的百分比条
          static uint8 percent;
          percent = (4 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
      }
          
    }
      
      break;
    
    case 1:
      HalLcdWriteString( "Relay Contrling", HAL_LCD_LINE_4 ); 
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      //这里的deviceStatus 可能是0x00,所以会出现bug 这个0在Device检查时被设置 2.24
      if(DeviceList[relay].deviceStatus == DEVICE_OFFLINE)
      {
        HalLcdWriteString( "Relay Offline", HAL_LCD_LINE_1 );      
      }
      else  //设备在线回显  设备在线可能是 0/1
      {
        HalLcdWriteString( "Relay Online", HAL_LCD_LINE_1 ); 
        //继电器控制界面
        const uint8 cmd =  DeviceList[relay].data[0];

        if((cmd & 0x02) == 0x02) {HalLcdWriteString( "K1:ON", HAL_LCD_LINE_2 );}
        if((cmd & 0x01) == 0x01) {HalLcdWriteString( "K1:OFF", HAL_LCD_LINE_2 );}
        if((cmd & 0x20) == 0x20) {HalLcdWriteString( "K2:ON", HAL_LCD_LINE_3 );}
        if((cmd & 0x10) == 0x10) {HalLcdWriteString( "K2:OFF", HAL_LCD_LINE_3 );}      
      }

      break;
    
    case 2:
      HalLcdWriteString( "Motor Contrling", HAL_LCD_LINE_4 );
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      if(DeviceList[relay].deviceStatus == DEVICE_OFFLINE)
      {
        HalLcdWriteString( "Motor Offline", HAL_LCD_LINE_1 );      
      }
      else  //设备在线回显  设备在线可能是 0/1
      {
        HalLcdWriteString( "Motor Online", HAL_LCD_LINE_1 ); 
        //电机控制界面
        const uint16 speed =  (uint16) DeviceList[relay].data[0];
        const uint16 status = (uint16) DeviceList[relay].data[1];
        
        HalLcdWriteStringValue( "Status:", status, 10, HAL_LCD_LINE_2 );
        HalLcdWriteStringValue( "Speed:", speed, 10, HAL_LCD_LINE_3 );
      
      }
  }
  
#endif // LCD_SUPPORTED  
}


