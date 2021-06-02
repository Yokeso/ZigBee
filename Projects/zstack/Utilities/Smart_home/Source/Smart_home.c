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
  3.28 调试时间
  + 解决motor显示的bug
  4.4
  + 解决温湿度数据不回显的bug
  4.6
  + 正式拿到所有节点开始调试
  4.9
  + 解决掉串口Bug
  4.14
  + 发现在多终端加入时会出现断网或假死问题，开始寻找
  4.26
  + 提出几种假设，利用其他平台做抓包器尝试
  5.6
  + 不能应用NV操作，因为NV操作仅能烧写20000次，并需要固定位置读取。
  + 调试了N天，感觉是内存问题，开始对程序进行负优化，以时间换空间  （内存仅128字节）
  5.8
  + 串口传输原来的想法是边发边传，但是这样会导致系统崩溃，换用事件轮询定时发送的方式传输
  + 上位机联动效果开始构想
   
  经调试程序无bug，可以接收各个传感器消息。//3.22
  当我没说。。好像出了内存问题//3.22
  项目重构啦！，现在又只有一个小问题了/3.23
*********************************************************************/

/*******************************************************************************
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

/*******************************************************************************
 * MACROS
 */

// 设备离线定时计数临界值
#define  DEVICE_HEART_BEAT 3

// Send with or without APS ACKs
#define Smart_home_TX_OPTIONS              AF_DISCV_ROUTE

// 第一次LCD显示延时
#define Smart_home_DISPLAY_DELAY           10000
// LCD每次显示时间间隔
#define Smart_home_DISPLAY_TIMER           2000

#define Smart_home_DEVICE_CHECK_DELAY      5000
#define Smart_home_DEVICE_CHECK_TIMER      2000 
// not used here
#define Smart_home_MATCH_TIMER     
// not used here
#define Smart_home_BIND_TIMER      

#if defined ( Smart_home_FRAGMENTED )
#define Smart_home_MAX_DATA_LEN            225
#else
#define Smart_home_MAX_DATA_LEN            102
#endif

/*******************************************************************************
 * GLOBAL VARIABLES
 */

// This is the buffer that is sent out as data.
byte Coordinator_Msg[ Smart_home_MAX_DATA_LEN ];

// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t Smart_home_InClusterList[Smart_home_MAX_INCLUSTERS] =
{
  Smart_home_CLUSTERID_HUMITMSG,      // 温湿度
  Smart_home_CLUSTERID_TEMPLIGHTMSG,  // 温度光照
  Smart_home_CLUSTERID_RFIDMSG,       // 射频卡
  Smart_home_CLUSTERID_GASFLAMEMSG,   // 气体火焰
  Smart_home_CLUSTERID_INFRAREDMSG,   // 人体红外
  Smart_home_CLUSTERID_SOUNDVBMSG,    // 声音震动
  Smart_home_CLUSTERID_MOTORSTATUSMSG,// 电机状态
  Smart_home_CLUSTERID_RELAYSTATUSMSG // 继电器状态
};

const cId_t Smart_home_OutClusterList[Smart_home_MAX_OUTCLUSTERS] =
{
  Smart_home_CLUSTERID_TEXT,    
  Smart_home_CLUSTERID_MOTORCTRL,   // 继电器
  Smart_home_CLUSTERID_RELAYCTRL    // 直流电机
};

const SimpleDescriptionFormat_t Smart_home_SimpleDesc =
{
  Smart_home_ENDPOINT,                //  int    Endpoint;
  Smart_home_PROFID,                  //  uint16 AppProfId[2];
  Smart_home_DEVICEID,                //  uint16 AppDeviceId[2];
  Smart_home_DEVICE_VERSION,          //  int    AppDevVer:4;
  Smart_home_FLAGS,                   //  int    AppFlags:4;
  Smart_home_MAX_INCLUSTERS,          
  (cId_t *)Smart_home_InClusterList,  
  Smart_home_MAX_OUTCLUSTERS,         
  (cId_t *)Smart_home_OutClusterList  
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in Smart_home_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t Smart_home_epDesc;    // 定义节点


/*******************************************************************************
 *串口配置
 */
#define TRANSMIT_APP_PORT  0
// zstack default: 38400
#define TRANSMIT_APP_BAUD  HAL_UART_BR_38400
// When the Rx buf space is less than this threshold, invoke the Rx callback.
#define TRANSMIT_APP_THRESH  64
#define TRANSMIT_APP_RX_SZ  128
#define TRANSMIT_APP_TX_SZ  128
// Millisecs of idle time after a byte is received before invoking Rx callback.
#define TRANSMIT_APP_IDLE  6
// Loopback Rx bytes to Tx for throughput testing.
#define TRANSMIT_APP_LOOPBACK  FALSE

/*关于显示部分相关的宏*/
#define LCD_PAGE_MAX         4     //目前目录页最多4页

static int8 Ctrlcase = 0;    //0用来控制屏幕显示，1控制继电器，2控制电机
static int8 LCD_Page  =  0;  //终端状态显示
/* 直流电机状态(status)定义*/
#define  HAL_MOTOR_STOP            0x01
#define  HAL_MOTOR_FORWARD         0x02
#define  HAL_MOTOR_BACKWARD        0x03
#define  MOTOR_MAX_SPEED           2400

/*********************************************************************
 * LOCAL VARIABLES
 */

/*3.18 终端节点的设备详细信息缓存*/
/*5.6 尝试利用离散变量的方式*/
//DeviceInfo   DeviceList[Smart_home_MAX_INCLUSTERS];                      //设备列表 
// 温湿度缓存, 第一个字节是湿度，第二个字节是温度(均是整数)
DeviceInfo Humit;
// 温度光照缓存, 前两个字节是温度整数和小数, 后两个字节是光照的16位整数 
DeviceInfo TempLight;
// RFID 信息
DeviceInfo RfID;
//气体火焰
DeviceInfo gasFlame;
//人体红外
DeviceInfo infrared;
//电机状态
DeviceInfo motor;
//继电器状态
DeviceInfo relay;
//声音震动
DeviceInfo soundVb;

// Task ID for event processing - received when Smart_home_Init() is called.
byte Smart_home_TaskID;


static byte Smart_home_RelayTransID;  // This is the unique message ID (counter)
static byte Smart_home_MotorTransID;  // This is the unique message ID (counter)

afAddrType_t Smart_home_DstAddr;

afAddrType_t Smart_home_DstRelayAddr;

afAddrType_t Smart_home_DstMotorAddr;

// Max Data Request Length
uint16 Smart_home_MaxDataLength;


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
void Smart_home_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void Smart_home_HandleKeys( byte shift, byte keys );
void Smart_home_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void Smart_home_SendToRelayMSG( uint8 cmd );
void Smart_home_SendToMotorMSG( uint8 cmd,uint8 speed );
void TransmitAPP_CallBack(uint8 port, uint8 event);
void Smart_home_Device_check(void);
void Smart_home_Display(void);

/*******************************************************************************
 * @fn      Smart_home_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Smart_home_Init( byte task_id )
{
#if HAL_UART==TRUE
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = TRANSMIT_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = TRANSMIT_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = TRANSMIT_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = TRANSMIT_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = TRANSMIT_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = FALSE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = TransmitAPP_CallBack;
  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
#endif
  
#if !defined ( Smart_home_FRAGMENTED )
  afDataReqMTU_t mtu;
#endif
  uint16 i;

  Smart_home_TaskID = task_id;
  Smart_home_RelayTransID = 0;
  Smart_home_MotorTransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  //Smart_home_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  //Smart_home_DstAddr.endPoint = 0;
  //Smart_home_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  // 初始化节点
  Smart_home_epDesc.endPoint = Smart_home_ENDPOINT;   // 端口号
  Smart_home_epDesc.task_id = &Smart_home_TaskID;     // 任务ID
  Smart_home_epDesc.simpleDesc                         // 简单描述符来描述端口
            = (SimpleDescriptionFormat_t *)&Smart_home_SimpleDesc;
  Smart_home_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint/interface description with the AF
  afRegister( &Smart_home_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( Smart_home_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Smart_home", HAL_LCD_LINE_2 );
#endif

  // Set the data length
#if defined ( Smart_home_FRAGMENTED )
  Smart_home_MaxDataLength = Smart_home_MAX_DATA_LEN;
#else
  mtu.kvp        = FALSE;
  mtu.aps.secure = FALSE;
  Smart_home_MaxDataLength = afDataReqMTU( &mtu );
#endif

  // 发送数据清零
  for (i = 0; i < Smart_home_MaxDataLength; i++)
  {
    Coordinator_Msg[i] = 0;
  }

  // 注册两个MSG
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, Match_Desc_rsp );
  
  // 打开显示，第一次延时周期较长
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DISPLAY_EVT, 
                      Smart_home_DISPLAY_DELAY);
  
  // 打开设备在线检测，第一次开启检测延时较长时间
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DEVICE_CHECK_EVT, 
                      Smart_home_DEVICE_CHECK_DELAY);
  
  // 关闭LED灯(D4)，表示协调器默认不允许组网
  NLME_PermitJoiningRequest(0x00);
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  HalUARTWrite(HAL_UART_PORT_0, "GOT IT111!\n",   11);
  
  // 设备离线状态检测初始化，初始化为离线
  Humit.deviceStatus     = DEVICE_OFFLINE;
  TempLight.deviceStatus = DEVICE_OFFLINE;
  RfID.deviceStatus      = DEVICE_OFFLINE;
  gasFlame.deviceStatus  = DEVICE_OFFLINE;
  infrared.deviceStatus  = DEVICE_OFFLINE;
  soundVb.deviceStatus   = DEVICE_OFFLINE;
  motor.deviceStatus     = DEVICE_OFFLINE;
  relay.deviceStatus     = DEVICE_OFFLINE;
}

/*******************************************************************************
 * @fn      Smart_home_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 Smart_home_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  (void)task_id;  // Intentionally unreferenced parameter

  // Data Confirmation message fields
  ZStatus_t sentStatus;
  byte sentEP;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Smart_home_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      { 
        case ZDO_CB_MSG:
          Smart_home_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          Smart_home_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
 
          if ( (ZSuccess == sentStatus) &&
               (Smart_home_epDesc.endPoint == sentEP) )
          {  
          }
          break;

        case AF_INCOMING_MSG_CMD:
          Smart_home_MessageMSGCB( MSGpkt );
#if (HAL_UART == TRUE)
          HalUARTWrite(HAL_UART_PORT_0, MSGpkt->cmd.Data,   MSGpkt->cmd.DataLength-1);
#endif  
          break;

        case ZDO_STATE_CHANGE:
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Smart_home_TaskID );
    }

    // Squash compiler warnings until values are used.
    (void)sentStatus;
    (void)sentEP;

    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out, 周期性串口回调
  if ( events & SMART_HOME_SEND_MSG_EVT )
  {      
    // Return unprocessed events
    return (events ^ SMART_HOME_SEND_MSG_EVT);
  }
  
  // 设备状态检查事件
  if ( events & SMART_HOME_DEVICE_CHECK_EVT )
  {
    // 调用设备网络状态监测函数 
    Smart_home_Device_check();
    
    // 任务需要周期性运行
    osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DEVICE_CHECK_EVT, 
                        Smart_home_DEVICE_CHECK_TIMER);
    
    // Return unprocessed events
    return (events ^ SMART_HOME_DEVICE_CHECK_EVT);
  }
 

  // LCD显示事件
  if ( events & SMART_HOME_DISPLAY_EVT )
  {
    // 刷新显示数据
    //Smart_home_DisplayResults( dispPage, &scrollLine);
    Smart_home_Display();
    
    // 周期性的调用该事件来刷新显示数据
    osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DISPLAY_EVT, 
                        Smart_home_DISPLAY_TIMER );   
    // Return unprocessed events
    return (events ^ SMART_HOME_DISPLAY_EVT);
  }

  // Smart_home_MATCHRSP_EVT事件预留
  if ( events & SMART_HOME_MATCHRSP_EVT )
  {  
    return (events ^ SMART_HOME_MATCHRSP_EVT);
  }
  // Smart_home_BINDRSP_EVT事件预留
  if ( events & SMART_HOME_BINDRSP_EVT )
  {
    return (events ^ SMART_HOME_BINDRSP_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*******************************************************************************
 * Event Generation Functions
 */
/*******************************************************************************
 * @fn      Smart_home_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void Smart_home_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  /*
  switch ( inMsg->clusterID )
  {
    // 绑定信息处理
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        osal_stop_timerEx( Smart_home_TaskID, Smart_home_BINDRSP_EVT);
#if defined ( LCD_SUPPORTED )
        HalLcdWriteString( "BindSuccess", HAL_LCD_LINE_3 );
#endif
      }
      break;

    // 描述符匹配信息处理 
    case Match_Desc_rsp:
      ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
      if ( pRsp )
      {
        if ( pRsp->status == ZSuccess && pRsp->cnt )
        {
          osal_stop_timerEx( Smart_home_TaskID, Smart_home_MATCHRSP_EVT);

#if defined ( LCD_SUPPORTED )
          HalLcdWriteString( "MatchSuccess", HAL_LCD_LINE_3 );
#endif
         }
         osal_mem_free( pRsp );
      }
      break;
  }*/
}

/*********************************************************************
 * @fn      Smart_home_Motor_cmd
 * 
 * @brief   通过电机速度判断电机转向
 * 
 * 
 * @param   uint8 电机转速
 * 
 * @return  
 *          #define  HAL_MOTOR_STOP            0x01
 *          #define  HAL_MOTOR_FORWARD         0x02
 *          #define  HAL_MOTOR_BACKWARD        0x03
 */
void Smart_home_Motor_cmd(int8 speed)
{
  uint8 outspeed;
  uint8 cmd;
  if(speed == 0)
  {
    outspeed = 0;
    cmd = HAL_MOTOR_STOP;
  }   
  else if(speed > 0)
  {
    outspeed = speed;
    cmd = HAL_MOTOR_FORWARD;
  }
  
  else if(speed < 0)
  {
    outspeed = 0xff - speed;
    cmd = HAL_MOTOR_BACKWARD;
  }
  
  else
  {
    outspeed = 0;
    cmd = HAL_MOTOR_STOP;    
  }
  Smart_home_SendToMotorMSG(cmd,outspeed); 
}


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
        Smart_home_SendToRelayMSG(switch1);  
        break;
      } 
      case 2:
      {
        if(MotorSpeed < 50)     {MotorSpeed += 10;}
        if(MotorSpeed >= 50)     
        {
#if defined ( LCD_SUPPORTED )
           HalLcdWriteString( "Motor max Speed", HAL_LCD_LINE_4 );
#endif 
           MotorSpeed = 50;
        }
        Smart_home_Motor_cmd(MotorSpeed);
        
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
        Smart_home_SendToRelayMSG(switch2);       
        break;
      } 
      case 2:
      {
        /*
        #define  HAL_MOTOR_STOP            0x01
        #define  HAL_MOTOR_FORWORD         0x02
        #define  HAL_MOTOR_BACKWORD        0x03*/
        if(MotorSpeed > -50)     {MotorSpeed -= 10;}
        if(MotorSpeed <= -50)     
        {
#if defined ( LCD_SUPPORTED )
           HalLcdWriteString( "Motor min Speed", HAL_LCD_LINE_4 );
#endif 
           MotorSpeed = -80;
        }
        Smart_home_Motor_cmd(MotorSpeed);
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
       //HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Allow networking", HAL_LCD_LINE_4 );
#endif
     }
     else
     {
       NetWorkAllow = 0;
       NLME_PermitJoiningRequest(0x00); // 不允许组网
       HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);  
       //HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF); 
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Ban   networking", HAL_LCD_LINE_4 );
#endif        
     }
  }
}


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * @fn      Smart_home_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
  
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
  //HalUARTWrite(HAL_UART_PORT_0, "9",   1);
void Smart_home_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{

  switch ( pkt->clusterId )
  {
    // 温湿度传感器信息
    case Smart_home_CLUSTERID_HUMITMSG:
      Humit.deviceStatus = DEVICE_ONLINE;
      Humit.data[0] = pkt->cmd.Data[4]; // 湿度 
      Humit.data[1] = pkt->cmd.Data[5]; // 温度
      break;
    
    // 温度与光照度传感器信息  
    case Smart_home_CLUSTERID_TEMPLIGHTMSG:
      TempLight.deviceStatus = DEVICE_ONLINE;
      TempLight.data[0] = pkt->cmd.Data[4]; // 温度整数
      TempLight.data[1] = pkt->cmd.Data[5]; // 温度小数
      TempLight.data[2] = pkt->cmd.Data[6]; // 光照
      TempLight.data[3] = pkt->cmd.Data[7]; // 光照
      break;
    
    // RFID射频卡信息 
    case Smart_home_CLUSTERID_RFIDMSG:
      RfID.deviceStatus = DEVICE_ONLINE;
      RfID.data[0] = pkt->cmd.Data[4]; // 射频卡类型
      RfID.data[1] = pkt->cmd.Data[5]; // 4个字节的ID号
      RfID.data[2] = pkt->cmd.Data[6]; //
      RfID.data[3] = pkt->cmd.Data[7]; //
      RfID.data[4] = pkt->cmd.Data[8]; //           
      break;
    
    // 烟雾与火焰报警信息  
    case Smart_home_CLUSTERID_GASFLAMEMSG:
      gasFlame.deviceStatus = DEVICE_ONLINE;
      gasFlame.data[0] = pkt->cmd.Data[4]; // 烟雾与火焰报警信息
      break;
    
    // 人体红外检测信息  
    case Smart_home_CLUSTERID_INFRAREDMSG:
      infrared.deviceStatus = DEVICE_ONLINE;
      infrared.data[0] = pkt->cmd.Data[4]; // 人体红外 
      break;
    
    // 声音与振动传感器信息  
    case Smart_home_CLUSTERID_SOUNDVBMSG:
      soundVb.deviceStatus = DEVICE_ONLINE;
      soundVb.data[0] = pkt->cmd.Data[4]; // 声音震动信息
      break;
    
    // 电机状态信息  
    case Smart_home_CLUSTERID_MOTORSTATUSMSG:
      motor.deviceStatus = DEVICE_ONLINE;
      
      // 储存电机设备的网络地址，用于发送控制命令
      Smart_home_DstMotorAddr.addrMode = (afAddrMode_t)Addr16Bit;
      Smart_home_DstMotorAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Smart_home_DstMotorAddr.endPoint = 1;  // 目的节点的端口号
      //Smart_home_DstMotorAddr.endPoint = Smart_home_ENDPOINT;
     
      motor.data[0] = pkt->cmd.Data[4]; // 电机转速
      motor.data[1] = pkt->cmd.Data[5]; // 电机状态
      break;
    
    // 继电器状态信息   
    case Smart_home_CLUSTERID_RELAYSTATUSMSG:
      relay.deviceStatus = DEVICE_ONLINE;
      
      // 储存继电器设备的网络地址，用于发送控制命令
      Smart_home_DstRelayAddr.addrMode = (afAddrMode_t)Addr16Bit;
      Smart_home_DstRelayAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Smart_home_DstRelayAddr.endPoint = 1; // 目的节点的端口号
      //Smart_home_DstRelayAddr.endPoint = Smart_home_ENDPOINT;  
      
      relay.data[0] = pkt->cmd.Data[4]; 
      break;
      
    // 同上面一样，可以在将来添加更多的控制信息
    default:
      break;
  }
}


/*******************************************************************************
 * @fn      Smart_home_DeviceCheck
 *
 * @brief   check the device  status: online or offline.
 *          由宏定义Smart_home_DEVICE_CHECK_TIMER确定周期是2秒
 *
 * @param   none
 *
 * @return  none
 */
/*3.24 这里的检测时常有点长 改为变量外提的方式加快处理速度*/
uint8 Device_check(DeviceInfo dev,uint8 count)
{
  if(dev.deviceStatus != DEVICE_ONLINE)    //设备离线
  {
    count++;
  }
  if(dev.deviceStatus == DEVICE_ONLINE)    //设备在线
  {
    count = 0;
    dev.deviceStatus = 0;
  }
  if(count > DEVICE_HEART_BEAT)
  {
    count = DEVICE_HEART_BEAT;
    dev.deviceStatus = DEVICE_OFFLINE;
  }  
  return count;
}

void Smart_home_Device_check(void)
{
  static uint8 humitCnt, tempLightCnt, rfIDCnt, gasFlameCnt, infraredCnt;
  static uint8 motorCnt, relayStatusCnt, soundVbCnt;
  humitCnt = Device_check(Humit,humitCnt);
  tempLightCnt = Device_check(TempLight,tempLightCnt);
  rfIDCnt = Device_check(RfID,rfIDCnt);
  gasFlameCnt = Device_check(gasFlame,gasFlameCnt);
  infraredCnt = Device_check(infrared,infraredCnt);
  motorCnt = Device_check(motor,motorCnt);
  relayStatusCnt = Device_check(relay,relayStatusCnt);
  soundVbCnt = Device_check(soundVb,soundVbCnt);
  /*温湿度缓存 温度光照缓存 RFID 信息缓存 气体火焰缓存 
  人体红外 电机状态 继电器状态 声音震动*/
}

/*******************************************************************************
 * @fn      Smart_home_SendToRelayMSG
 *
 * @brief   Send control message to relay, if relay is in the network.
 *
 * @param   uint8 cmd: Relay control command
 *
 * @return  none
 */
void Smart_home_SendToRelayMSG( uint8 cmd )
{
  uint8 tmp;
  
  // 只有设备在线时, 方发送控制命令
  if (relay.deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( Smart_home_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( Smart_home_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // 发送给继电器的控制命令 
    Coordinator_Msg[4] = cmd;
    
    // Smart_home_epDesc.endPoint = Smart_home_CLUSTERID_RELAYCTLMSG; // 20201225
    tmp = AF_DataRequest( &Smart_home_DstRelayAddr,           \
                          &Smart_home_epDesc,                 \
                           Smart_home_CLUSTERID_RELAYCTRL,    \
                           RELAYSTATUSMSG_LEN,                \
                           Coordinator_Msg,                   \
                          &Smart_home_RelayTransID,           \
                           Smart_home_TX_OPTIONS,             \
                           AF_DEFAULT_RADIUS );
  }
}

/*******************************************************************************
 * @fn      Smart_home_SendToMotorMSG
 *
 * @brief   Send  message to motor.
 *
 * @param   uint8 cmd: motor command
 *
 * @return  none
 */
void Smart_home_SendToMotorMSG( uint8 cmd, uint8 speed )
{
  uint8 tmp;
  
  // 只有设备在线时, 方发送控制命令
  if (motor.deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( Smart_home_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( Smart_home_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    Coordinator_Msg[4] = speed; // 电机速度
    Coordinator_Msg[5] = cmd;   // 电机控制状态(方向)
  
    //Smart_home_epDesc.endPoint = Smart_home_CLUSTERID_MOTORCTLMSG; // 20201225
    tmp = AF_DataRequest( &Smart_home_DstMotorAddr,            \
                          &Smart_home_epDesc,                  \
                           Smart_home_CLUSTERID_MOTORCTRL,     \
                           MOTORSTATUSMSG_LEN,                 \
                           Coordinator_Msg,                    \
                          &Smart_home_MotorTransID,            \
                           Smart_home_TX_OPTIONS,              \
                           AF_DEFAULT_RADIUS );
  }
}

/*******************************************************************************
 * @fn      TransmitAPP_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
void TransmitAPP_CallBack(uint8 port, uint8 event)
{ 
}

/*******************************************************************************
*******************************************************************************/
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
  static uint8 percent;
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
          //DeviceInfo* Devhum = &DeviceList[Humit];
          //DeviceInfo* DevSound = &DeviceList[soundVb];
          //第一页第二行显示温度
          //      第三行显示光照
          if(Humit.deviceStatus != DEVICE_OFFLINE)
          {    
              HalLcdWriteStringValueValue( "Hum:", Humit.data[0], 10, "% T_1:", Humit.data[1], 10, HAL_LCD_LINE_2 );
          }
          else
          {
              HalLcdWriteString( "Hum:OFF T_1:OFF", HAL_LCD_LINE_2 ); 
          }
          
          
          if(soundVb.deviceStatus  != DEVICE_OFFLINE)
          {    
              if(soundVb.data[0] & 0x01 == 0x01) { HalLcdWriteString( "Sound: Voice", HAL_LCD_LINE_3 ); }
              else if(soundVb.data[0] & 0x02 == 0x02) { HalLcdWriteString( "Sound: Vibration", HAL_LCD_LINE_3 ); }
              else if(soundVb.data[0] & 0x03 == 0x03) { HalLcdWriteString( "Sound: All", HAL_LCD_LINE_3 ); }
              else {HalLcdWriteString( "Sound: None", HAL_LCD_LINE_3 );}
          }
          else
          {
              HalLcdWriteString( "Sound:OFFLINE", HAL_LCD_LINE_3 ); 
          }
       
          //HalLcdWriteStringValue( "Sound:", SoundVb, 16, HAL_LCD_LINE_3 );
          
          
          //显示最下面的百分比条
          percent = (1 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
        }
        case 1:
        {
          //第二页只有光照
          //第一页第二行显示温度
          //      第三行显示光照
          //DeviceInfo* Devtmp = &DeviceList[TempLight];
          if(TempLight.deviceStatus  != DEVICE_OFFLINE)
          {    
              static uint16 Light;
              static uint8 lightmp;
          

              lightmp = TempLight.data[3];
              Light = (uint16)TempLight.data[4];
              memcpy(&Light,&lightmp,sizeof(lightmp));
          
              HalLcdWriteStringValueValue( "Temper_2:", TempLight.data[0], 10, ".", TempLight.data[1], 10, HAL_LCD_LINE_2 );
              HalLcdWriteStringValue( "Light:", Light, 10, HAL_LCD_LINE_3 );
              
              osal_msg_deallocate((uint8*)Light);
          }
          else
          {
              HalLcdWriteString( "Temper_2:OFF", HAL_LCD_LINE_2 ); 
              HalLcdWriteString( "Light:OFF", HAL_LCD_LINE_3 ); 
          }
          
          //显示最下面的百分比条
          percent = (2 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
          
          
        }
        
      case 2:
      {
          static uint16 Data1;
          static uint16 Data2;
          //显示最下面的百分比条
          //DeviceInfo* rfid = &DeviceList[RfID];
          
          Data1 = ( RfID.data[1] << 8) | RfID.data[2];
          Data2 = ( RfID.data[3] << 8) | RfID.data[4];
          
          switch(RfID.data[0])
          {
          case 0x01:
            HalLcdWriteString( "MFOne-S50", HAL_LCD_LINE_2 );
            break;
          case 0x02:
            HalLcdWriteString( "MFOne-S70", HAL_LCD_LINE_2 );
            break;
          case 0x03:
            HalLcdWriteString( "MF-UltraLight", HAL_LCD_LINE_2 );
            break;
          case 0x04:
            HalLcdWriteString( "MF-Pro", HAL_LCD_LINE_2 );
            break;
          case 0x05:
            HalLcdWriteString( "MF-DesFire", HAL_LCD_LINE_2 );
            break;
            
          default:
            HalLcdWriteString( "No Card", HAL_LCD_LINE_2 );
            break;
          }
          
          HalLcdWriteStringValueValue( "ID: ", Data1, 16, "-", Data2, 16, HAL_LCD_LINE_3 );
          
          percent = (3 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          
          osal_msg_deallocate((uint8*)Data1);
          osal_msg_deallocate((uint8*)Data2);
          
          break;
      }
      
      case 3:
      {        
          //DeviceInfo* DevGas = &DeviceList[gasFlame];
          //DeviceInfo* DevInf = &DeviceList[infrared];         
          
          if(gasFlame.deviceStatus  != DEVICE_OFFLINE)
          {    
              if(gasFlame.data[0] & 0x01 == 0x01) { HalLcdWriteString( "GasFlame: Flame", HAL_LCD_LINE_2 ); }         //0位是火焰
              else if(gasFlame.data[0] & 0x02 == 0x02) { HalLcdWriteString( "GasFlame: Gas", HAL_LCD_LINE_2 ); }//1位是气体
              else if(gasFlame.data[0] & 0x03 == 0x03) { HalLcdWriteString( "GasFlame: All", HAL_LCD_LINE_2 ); }
              else {HalLcdWriteString( "GasFlame: None", HAL_LCD_LINE_2 );}
          }
          else
          {
              HalLcdWriteString( "GasFlame: OFF", HAL_LCD_LINE_2 ); 
          }
          
          
          if(infrared.deviceStatus  != DEVICE_OFFLINE)
          {    
              if(infrared.data[0] == 0x01)
              {
                HalLcdWriteString("Infrared: Human", HAL_LCD_LINE_3 );
              }
              else{
                HalLcdWriteString("Infrared: NoHuman", HAL_LCD_LINE_3 );
              }
          }
          else
          {
              HalLcdWriteString( "Infrared: OFF", HAL_LCD_LINE_3 ); 
          }         

          
          
          //显示最下面的百分比条
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
      if(relay.deviceStatus == DEVICE_OFFLINE)
      {
        HalLcdWriteString( "Relay Offline", HAL_LCD_LINE_1 );      
      }
      else  //设备在线回显  设备在线可能是 0/1
      {
        HalLcdWriteString( "Relay Online", HAL_LCD_LINE_1 ); 
        //继电器控制界面

        if((relay.data[0]& 0x02) == 0x02) {HalLcdWriteString( "K1:ON", HAL_LCD_LINE_2 );}
        if((relay.data[0] & 0x01) == 0x01) {HalLcdWriteString( "K1:OFF", HAL_LCD_LINE_2 );}
        if((relay.data[0] & 0x20) == 0x20) {HalLcdWriteString( "K2:ON", HAL_LCD_LINE_3 );}
        if((relay.data[0] & 0x10) == 0x10) {HalLcdWriteString( "K2:OFF", HAL_LCD_LINE_3 );}      
      }

      break;
    
    case 2:
      HalLcdWriteString( "Motor Contrling", HAL_LCD_LINE_4 );
      //清除屏幕显示
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      if(motor.deviceStatus == DEVICE_OFFLINE)
      {
        HalLcdWriteString( "Motor Offline", HAL_LCD_LINE_1 );      
      }
      else  //设备在线回显  设备在线可能是 0/1
      {
        HalLcdWriteString( "Motor Online", HAL_LCD_LINE_1 ); 
        //电机控制界面
        switch(motor.data[1])
        {
        default:
        case 1:
          HalLcdWriteString( "Status: STOP", HAL_LCD_LINE_2 );
          break;
        case 2:
          HalLcdWriteString( "Status: FORWARD", HAL_LCD_LINE_2 );
          break;
        case 3:
          HalLcdWriteString( "Status: BACKWARD", HAL_LCD_LINE_2 );
          break;
        }
        
        HalLcdWriteStringValue( "Speed:", motor.data[0], 10, HAL_LCD_LINE_3 );
      
      }
  }
  
#endif // LCD_SUPPORTED  
}

