/*******************************************************************************
  文 件 名：TransmitApp.c
  作    者：南京安宸博研电子科技有限公司
  创建时间：2013.9.20
  修改时间：2020.12.22
  IAR 版本：IAR for 8051 V8.10.1
  测试平台：MotherBoard V2.1

  说    明：
  本程序实现协调器组网，采集传感器信息并在LCD上显示，且可通过按键控制设备运行。
  协调器管理8个终端设备，具体如下：
      1、温度与光照度传感器；2、烟雾与火焰传感器；3、继电器；4、温湿度传感器；
      5、声音与震动传感器；  6、人体红外检测传感器；7、电机；8、RFID检测器。

  终端设备周期性发送信息给协调器，协调器将数据缓存起来，然后定时2s在LCD显示出来。
  在终端设备中，DHT11温湿度传感器是每隔5秒采集一次，其余都是每隔1秒采集一次。
 
  按键：
       (此处的按键标号是软件内部标号，与开发板上的标号并非一一对应)
       SW1(UP)    ：行显示++(页面上翻)；
       SW2(RIGHT) ：控制继电器，每按一下按键，继电器状态切换一次；
       SW3(DOWN)  ：行显示--(页面下翻)；
       SW4(LEFT)  ：控制直流电机，每按下一次按键，直流电机的状态切换一次；
       SW5(OK)    ：显示切换，同时打开屏幕滚动显示；显示信息分为两屏，
                    1、第一屏显示采集到的温湿度、光照信息、继电器和直流电机，
                      由于增加电机转速显示后，需要滚动显示；
                    2、取消第一屏的滚动显示；
                    3、第二屏显示告警信息，第二屏四行显示不全，需要滚动显示；
                    4、取消第二屏的滚动显示；
       SW7(CANCEL)：打开/关闭协调器组网功能，灯亮表示允许，灯灭表示不允许。
                    请注意：系统默认关闭组网功能，如有需要，请按SW7键打开。

  LCD:
      显示部分，传感器信息显示事件是自动运行的，默认显示的是第一屏信息，手动
      按键SW5可以切换到第二屏，第二屏默认滚动显示，再次按下按键SW5可以关闭滚
      动显示，此时按键SW1和SW3可以控制滚动屏幕向上或者向下，
      LCD显示举例说明：
               第一屏：H:OFF T:OFF(湿度、温度设备离线)
                       T:32.5C L:320L(温度32.5度，光照强度：320流明)
                       K1:off K2:on(继电器K1关闭，K2打开，如果是OFF表示离线)
                       Motor:Stop(电机停转，如果是OFF表示离线)
                       Speed:0RPS(电机目前转速为0转/分钟)

               第二屏：Gas:OFF(烟雾传感器离线)
                       Flame:OFF(火焰传感器离线)
                       Sound:ON(声音传感器在线，没有报警)
                       Vibrate:Alarm!(震动报警)
                       Infrared:ON(人体红外检测在线，没有报警)
                       Card:OFF(射频卡设备离线)
                               (此行空白，因为射频卡设备离线，所以没有ID信息)
  
  设备在线检测：
      主要是TransmitApp_DeviceNWKCheck(void)函数来检测各个设备是否在限定的时间内
      发送数据，如果超时，判定设备离线。如果设备发送数据，那么相应的deviceInfo_t
      结构体中的deviceNWKStatus标记为1，如果超时，标记为0，显示部分的函数只需读
      取设备的网络状态就可以判定是否离线了。
*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "nwk_util.h"

#include "TransmitApp.h"
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
#define  DEVICE_NWK_CNT_LIMIT 3

// Send with or without APS ACKs
#define TRANSMITAPP_TX_OPTIONS              AF_DISCV_ROUTE

// 第一次LCD显示延时
#define TRANSMITAPP_DISPLAY_DELAY           10000
// LCD每次显示时间间隔
#define TRANSMITAPP_DISPLAY_TIMER           2000

#define TRANSMITAPP_DEVICE_CHECK_DELAY      5000
#define TRANSMITAPP_DEVICE_CHECK_TIMER      2000 
// not used here
#define TRANSMITAPP_MATCH_TIMER     
// not used here
#define TRANSMITAPP_BIND_TIMER      

#if defined ( TRANSMITAPP_FRAGMENTED )
#define TRANSMITAPP_MAX_DATA_LEN            225
#else
#define TRANSMITAPP_MAX_DATA_LEN            102
#endif

/*******************************************************************************
 * GLOBAL VARIABLES
 */

// This is the buffer that is sent out as data.
byte TransmitApp_Msg[ TRANSMITAPP_MAX_DATA_LEN ];

// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t TransmitApp_InClusterList[TRANSMITAPP_MAX_INCLUSTERS] =
{
  TRANSMITAPP_CLUSTERID_HUMITMSG,      // 温湿度
  TRANSMITAPP_CLUSTERID_TEMPLIGHTMSG,  // 温度光照
  TRANSMITAPP_CLUSTERID_RFIDMSG,       // 射频卡
  TRANSMITAPP_CLUSTERID_GASFLAMEMSG,   // 气体火焰
  TRANSMITAPP_CLUSTERID_INFRAREDMSG,   // 人体红外
  TRANSMITAPP_CLUSTERID_SOUNDVBMSG,    // 声音震动
  TRANSMITAPP_CLUSTERID_MOTORSTATUSMSG,// 电机状态
  TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG // 继电器状态
};

const cId_t TransmitApp_OutClusterList[TRANSMITAPP_MAX_OUTCLUSTERS] =
{
  TRANSMITAPP_CLUSTERID_TESTMSG,    
  TRANSMITAPP_CLUSTERID_RELAYCTLMSG,   // 继电器
  TRANSMITAPP_CLUSTERID_MOTORCTLMSG    // 直流电机
};

const SimpleDescriptionFormat_t TransmitApp_SimpleDesc =
{
  TRANSMITAPP_ENDPOINT,                //  int    Endpoint;
  TRANSMITAPP_PROFID,                  //  uint16 AppProfId[2];
  TRANSMITAPP_DEVICEID,                //  uint16 AppDeviceId[2];
  TRANSMITAPP_DEVICE_VERSION,          //  int    AppDevVer:4;
  TRANSMITAPP_FLAGS,                   //  int    AppFlags:4;
  TRANSMITAPP_MAX_INCLUSTERS,          
  (cId_t *)TransmitApp_InClusterList,  
  TRANSMITAPP_MAX_OUTCLUSTERS,         
  (cId_t *)TransmitApp_OutClusterList  
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in TransmitApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t TransmitApp_epDesc;    // 定义节点


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
// 温湿度缓存, 第一个字节是湿度，第二个字节是温度(均是整数)
deviceInfo_t Humit;
// 温度光照缓存, 前两个字节是温度整数和小数, 后两个字节是光照的16位整数 
deviceInfo_t TempLight;
// RFID 信息
deviceInfo_t RfID;
//气体火焰
deviceInfo_t gasFlame;
//人体红外
deviceInfo_t infrared;
//电机状态
deviceInfo_t motor;
//继电器状态
deviceInfo_t relay;
//声音震动
deviceInfo_t soundVb;

// Task ID for event processing - received when TransmitApp_Init() is called.
byte TransmitApp_TaskID;

static byte TransmitApp_RelayTransID;  // This is the unique message ID (counter)
static byte TransmitApp_MotorTransID;  // This is the unique message ID (counter)

afAddrType_t TransmitApp_DstAddr;

afAddrType_t TransmitApp_DstRelayAddr;

afAddrType_t TransmitApp_DstMotorAddr;

// Max Data Request Length
uint16 TransmitApp_MaxDataLength;

// LCD显示
static uint8 dispPage=1;
static uint8 scrollLine=0;
//LCD屏幕滚动开关
static uint8 autoScrollEnable=1;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
void TransmitApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void TransmitApp_HandleKeys( byte shift, byte keys );
void TransmitApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void TransmitApp_SendToRelayMSG( uint8 cmd );
void TransmitApp_SendToMotorMSG( uint8 cmd,uint8 speed );
void TransmitAPP_CallBack(uint8 port, uint8 event);
void TransmitApp_DeviceNWKCheck(void);
void Smart_home_Display(void);

/*******************************************************************************
 * @fn      TransmitApp_Init
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
void TransmitApp_Init( byte task_id )
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
  
#if !defined ( TRANSMITAPP_FRAGMENTED )
  afDataReqMTU_t mtu;
#endif
  uint16 i;

  TransmitApp_TaskID = task_id;
  TransmitApp_RelayTransID = 0;
  TransmitApp_MotorTransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  //TransmitApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  //TransmitApp_DstAddr.endPoint = 0;
  //TransmitApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  // 初始化节点
  TransmitApp_epDesc.endPoint = TRANSMITAPP_ENDPOINT;   // 端口号
  TransmitApp_epDesc.task_id = &TransmitApp_TaskID;     // 任务ID
  TransmitApp_epDesc.simpleDesc                         // 简单描述符来描述端口
            = (SimpleDescriptionFormat_t *)&TransmitApp_SimpleDesc;
  TransmitApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint/interface description with the AF
  afRegister( &TransmitApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( TransmitApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "TransmitApp", HAL_LCD_LINE_2 );
#endif

  // Set the data length
#if defined ( TRANSMITAPP_FRAGMENTED )
  TransmitApp_MaxDataLength = TRANSMITAPP_MAX_DATA_LEN;
#else
  mtu.kvp        = FALSE;
  mtu.aps.secure = FALSE;
  TransmitApp_MaxDataLength = afDataReqMTU( &mtu );
#endif

  // 发送数据清零
  for (i = 0; i < TransmitApp_MaxDataLength; i++)
  {
    TransmitApp_Msg[i] = 0;
  }

  // 注册两个MSG
  // 绑定需要一种即可, 无需两种?????
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, Match_Desc_rsp );
  
  // 打开显示，第一次延时周期较长
  osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DISPLAY_EVT, 
                      TRANSMITAPP_DISPLAY_DELAY);
  
  // 打开设备在线检测，第一次开启检测延时较长时间
  osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DEVICE_CHECK_EVT, 
                      TRANSMITAPP_DEVICE_CHECK_DELAY);
  
  // 关闭LED灯(D4)，表示协调器默认不允许组网
  NLME_PermitJoiningRequest(0x00);
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  
  // 设备离线状态检测初始化，初始化为离线
  Humit.deviceNWKStatus     = DEVICE_NWK_OFFLINE;
  TempLight.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  RfID.deviceNWKStatus      = DEVICE_NWK_OFFLINE;
  gasFlame.deviceNWKStatus  = DEVICE_NWK_OFFLINE;
  infrared.deviceNWKStatus  = DEVICE_NWK_OFFLINE;
  soundVb.deviceNWKStatus   = DEVICE_NWK_OFFLINE;
  motor.deviceNWKStatus     = DEVICE_NWK_OFFLINE;
  relay.deviceNWKStatus     = DEVICE_NWK_OFFLINE;
}

/*******************************************************************************
 * @fn      TransmitApp_ProcessEvent
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
UINT16 TransmitApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  (void)task_id;  // Intentionally unreferenced parameter

  // Data Confirmation message fields
  ZStatus_t sentStatus;
  byte sentEP;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TransmitApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          TransmitApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          TransmitApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
 
          if ( (ZSuccess == sentStatus) &&
               (TransmitApp_epDesc.endPoint == sentEP) )
          {  
          }
          break;

        case AF_INCOMING_MSG_CMD:
          TransmitApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TransmitApp_TaskID );
    }

    // Squash compiler warnings until values are used.
    (void)sentStatus;
    (void)sentEP;

    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out, 本实验协调器没有周期发送数任务
  if ( events & TRANSMITAPP_SEND_MSG_EVT )
  {      
    // Return unprocessed events
    return (events ^ TRANSMITAPP_SEND_MSG_EVT);
  }
  
  // 设备状态检查事件
  if ( events & TRANSMITAPP_DEVICE_CHECK_EVT )
  {
    // 调用设备网络状态监测函数
    TransmitApp_DeviceNWKCheck(); 
    
    // 任务需要周期性运行
    osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DEVICE_CHECK_EVT, 
                        TRANSMITAPP_DEVICE_CHECK_TIMER);
    
    // Return unprocessed events
    return (events ^ TRANSMITAPP_DEVICE_CHECK_EVT);
  }
 

  // LCD显示事件
  if ( events & TRANSMITAPP_DISPLAY_EVT )
  {
    // 如果滚屏功能打开，自动向上滚动一行
    if (autoScrollEnable == 1)
    {
        scrollLine++;
    }
    // 刷新显示数据
    //TransmitApp_DisplayResults( dispPage, &scrollLine);
    Smart_home_Display();
    
    // 周期性的调用该事件来刷新显示数据
    osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DISPLAY_EVT, 
                        TRANSMITAPP_DISPLAY_TIMER );   
    // Return unprocessed events
    return (events ^ TRANSMITAPP_DISPLAY_EVT);
  }

  // TRANSMITAPP_MATCHRSP_EVT事件预留
  if ( events & TRANSMITAPP_MATCHRSP_EVT )
  {  
    return (events ^ TRANSMITAPP_MATCHRSP_EVT);
  }
  // TRANSMITAPP_BINDRSP_EVT事件预留
  if ( events & TRANSMITAPP_BINDRSP_EVT )
  {
    return (events ^ TRANSMITAPP_BINDRSP_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*******************************************************************************
 * Event Generation Functions
 */
/*******************************************************************************
 * @fn      TransmitApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  /*
  switch ( inMsg->clusterID )
  {
    // 绑定信息处理
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        osal_stop_timerEx( TransmitApp_TaskID, TRANSMITAPP_BINDRSP_EVT);
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
          osal_stop_timerEx( TransmitApp_TaskID, TRANSMITAPP_MATCHRSP_EVT);

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
  TransmitApp_SendToMotorMSG(cmd,outspeed); 
}


void TransmitApp_HandleKeys( byte shift, byte keys )
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
        TransmitApp_SendToRelayMSG(switch1);  
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
        TransmitApp_SendToRelayMSG(switch2);       
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
       //HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
       //HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Allow networking", HAL_LCD_LINE_4 );
#endif
     }
     else
     {
       NetWorkAllow = 0;
       NLME_PermitJoiningRequest(0x00); // 不允许组网
       //HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);  
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
 * @fn      TransmitApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{

  switch ( pkt->clusterId )
  {
    // 温湿度传感器信息
    case TRANSMITAPP_CLUSTERID_HUMITMSG:
      Humit.deviceNWKStatus = DEVICE_NWK_ONLINE;
      Humit.data[0] = pkt->cmd.Data[4]; // 湿度 
      Humit.data[1] = pkt->cmd.Data[5]; // 温度
      break;
    
    // 温度与光照度传感器信息  
    case TRANSMITAPP_CLUSTERID_TEMPLIGHTMSG:
      TempLight.deviceNWKStatus = DEVICE_NWK_ONLINE;
      TempLight.data[0] = pkt->cmd.Data[4]; // 温度整数
      TempLight.data[1] = pkt->cmd.Data[5]; // 温度小数
      TempLight.data[2] = pkt->cmd.Data[6]; // 光照
      TempLight.data[3] = pkt->cmd.Data[7]; // 光照
      break;
    
    // RFID射频卡信息 
    case TRANSMITAPP_CLUSTERID_RFIDMSG:
      RfID.deviceNWKStatus = DEVICE_NWK_ONLINE;
      RfID.data[0] = pkt->cmd.Data[4]; // 射频卡类型
      RfID.data[1] = pkt->cmd.Data[5]; // 4个字节的ID号
      RfID.data[2] = pkt->cmd.Data[6]; //
      RfID.data[3] = pkt->cmd.Data[7]; //
      RfID.data[4] = pkt->cmd.Data[8]; //           
      break;
    
    // 烟雾与火焰报警信息  
    case TRANSMITAPP_CLUSTERID_GASFLAMEMSG:
      gasFlame.deviceNWKStatus = DEVICE_NWK_ONLINE;
      gasFlame.data[0] = pkt->cmd.Data[4]; // 烟雾与火焰报警信息
      break;
    
    // 人体红外检测信息  
    case TRANSMITAPP_CLUSTERID_INFRAREDMSG:
      infrared.deviceNWKStatus = DEVICE_NWK_ONLINE;
      infrared.data[0] = pkt->cmd.Data[4]; // 人体红外 
      break;
    
    // 声音与振动传感器信息  
    case TRANSMITAPP_CLUSTERID_SOUNDVBMSG:
      soundVb.deviceNWKStatus = DEVICE_NWK_ONLINE;
      soundVb.data[0] = pkt->cmd.Data[4]; // 声音震动信息
      break;
    
    // 电机状态信息  
    case TRANSMITAPP_CLUSTERID_MOTORSTATUSMSG:
      motor.deviceNWKStatus = DEVICE_NWK_ONLINE;
      
      // 储存电机设备的网络地址，用于发送控制命令
      TransmitApp_DstMotorAddr.addrMode = (afAddrMode_t)Addr16Bit;
      TransmitApp_DstMotorAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      TransmitApp_DstMotorAddr.endPoint = 1;  // 目的节点的端口号
      //TransmitApp_DstMotorAddr.endPoint = TRANSMITAPP_ENDPOINT;
     
      motor.data[0] = pkt->cmd.Data[4]; // 电机转速
      motor.data[1] = pkt->cmd.Data[5]; // 电机状态
      break;
    
    // 继电器状态信息   
    case TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG:
      relay.deviceNWKStatus = DEVICE_NWK_ONLINE;
      
      // 储存继电器设备的网络地址，用于发送控制命令
      TransmitApp_DstRelayAddr.addrMode = (afAddrMode_t)Addr16Bit;
      TransmitApp_DstRelayAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      TransmitApp_DstRelayAddr.endPoint = 1; // 目的节点的端口号
      //TransmitApp_DstRelayAddr.endPoint = TRANSMITAPP_ENDPOINT;  
      
      relay.data[0] = pkt->cmd.Data[4]; 
      break;
      
    // 同上面一样，可以在将来添加更多的控制信息
    default:
      break;
  }
}

/*******************************************************************************
 * @fn      TransmitApp_DeviceNWKCheck
 *
 * @brief   check the device NWK status: online or offline.
 *          由宏定义TRANSMITAPP_DEVICE_CHECK_TIMER确定周期是2秒
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_DeviceNWKCheck(void)
{
  // 设备离线计数器，超过临界值判定设备离线
  static uint8 humitCnt, tempLightCnt, rfIDCnt, gasFlameCnt, infraredCnt;
  static uint8 motorCnt, relayStatusCnt, soundVbCnt;
  
  // 温湿度传感器
  if (Humit.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
    humitCnt++;
  }
  if (Humit.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
    humitCnt = 0;
    Humit.deviceNWKStatus = 0;
  }
  if (humitCnt > DEVICE_NWK_CNT_LIMIT)            // 超时, 判为离线
  {
    humitCnt = DEVICE_NWK_CNT_LIMIT;
    Humit.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
  
  // 温度与光照度传感器
  if (TempLight.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
    tempLightCnt++;
  }
  if (TempLight.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
    tempLightCnt = 0;
    TempLight.deviceNWKStatus = 0;
  }
  if (tempLightCnt > DEVICE_NWK_CNT_LIMIT)            // 超时, 判为离线
  {
    tempLightCnt = DEVICE_NWK_CNT_LIMIT;
    TempLight.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
  
  // RFID读卡器
  if (RfID.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
      rfIDCnt++;
  }
  if (RfID.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
      rfIDCnt = 0;
      RfID.deviceNWKStatus = 0;
  }
  if (rfIDCnt > DEVICE_NWK_CNT_LIMIT)            // 超时, 判为离线
  {
      rfIDCnt = DEVICE_NWK_CNT_LIMIT;
      RfID.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // 烟雾与火焰传感器  
  if (gasFlame.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
    gasFlameCnt++;
  }
  if (gasFlame.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
    gasFlameCnt = 0;
    gasFlame.deviceNWKStatus = 0;
  }
  if (gasFlameCnt > DEVICE_NWK_CNT_LIMIT)            // 超时, 判为离线
  {
    gasFlameCnt = DEVICE_NWK_CNT_LIMIT;
    gasFlame.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
  
  // 人体红外检测传感器
  if (infrared.deviceNWKStatus != DEVICE_NWK_ONLINE)  // 设备离线，计数+1
  {
    infraredCnt++;
  }
  if (infrared.deviceNWKStatus == DEVICE_NWK_ONLINE)  // 设备在线，计数清零
  {
    infraredCnt = 0;
    infrared.deviceNWKStatus = 0;
  }
  if (infraredCnt > DEVICE_NWK_CNT_LIMIT)             // 超时, 判为离线
  {
    infraredCnt = DEVICE_NWK_CNT_LIMIT;
    infrared.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // 声音振动传感器  
  if (soundVb.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
    soundVbCnt++;
  }
  if (soundVb.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
    soundVbCnt++;
    soundVb.deviceNWKStatus = 0;
  }
  if (soundVbCnt > DEVICE_NWK_CNT_LIMIT)            // 超时, 判为离线
  {
    soundVbCnt = DEVICE_NWK_CNT_LIMIT;
    soundVb.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // 微型直流电机RF-310T/QJT310AH 
  if (motor.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
    motorCnt++;
  }
  if (motor.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
    motorCnt = 0;
    motor.deviceNWKStatus = 0;
  }
  if (motorCnt > DEVICE_NWK_CNT_LIMIT)            // 超时, 判为离线
  {
    motorCnt = DEVICE_NWK_CNT_LIMIT;
    motor.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // 继电器  
  if (relay.deviceNWKStatus != DEVICE_NWK_ONLINE) // 设备离线，计数+1
  {
    relayStatusCnt++;
  }
  if (relay.deviceNWKStatus == DEVICE_NWK_ONLINE) // 设备在线，计数清零
  {
    relayStatusCnt = 0;
    relay.deviceNWKStatus = 0;
  }
  if (relayStatusCnt > DEVICE_NWK_CNT_LIMIT)      // 超时, 判为离线
  {
    relayStatusCnt = DEVICE_NWK_CNT_LIMIT;
    relay.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }  
}

/*******************************************************************************
 * @fn      TransmitApp_SendToRelayMSG
 *
 * @brief   Send control message to relay, if relay is in the network.
 *
 * @param   uint8 cmd: Relay control command
 *
 * @return  none
 */
void TransmitApp_SendToRelayMSG( uint8 cmd )
{
  uint8 tmp;
  
  // 只有设备在线时, 方发送控制命令
  if (relay.deviceNWKStatus != DEVICE_NWK_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( TransmitApp_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[2] = tmp;
    tmp = LO_UINT8( TransmitApp_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[3] = tmp;
    
    // 发送给继电器的控制命令 
    TransmitApp_Msg[4] = cmd;
    
    // TransmitApp_epDesc.endPoint = TRANSMITAPP_CLUSTERID_RELAYCTLMSG; // 20201225
    tmp = AF_DataRequest( &TransmitApp_DstRelayAddr,           \
                          &TransmitApp_epDesc,                 \
                           TRANSMITAPP_CLUSTERID_RELAYCTLMSG,  \
                           TRANSMITAPP_RELAY_DATA_LEN,         \
                           TransmitApp_Msg,                    \
                          &TransmitApp_RelayTransID,           \
                           TRANSMITAPP_TX_OPTIONS,             \
                           AF_DEFAULT_RADIUS );
  }
}

/*******************************************************************************
 * @fn      TransmitApp_SendToMotorMSG
 *
 * @brief   Send  message to motor.
 *
 * @param   uint8 cmd: motor command
 *
 * @return  none
 */
void TransmitApp_SendToMotorMSG( uint8 cmd, uint8 speed )
{
  uint8 tmp;
  
  // 只有设备在线时, 方发送控制命令
  if (motor.deviceNWKStatus != DEVICE_NWK_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( TransmitApp_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[2] = tmp;
    tmp = LO_UINT8( TransmitApp_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[3] = tmp;
    
    TransmitApp_Msg[4] = speed; // 电机速度
    TransmitApp_Msg[5] = cmd;   // 电机控制状态(方向)
  
    //TransmitApp_epDesc.endPoint = TRANSMITAPP_CLUSTERID_MOTORCTLMSG; // 20201225
    tmp = AF_DataRequest( &TransmitApp_DstMotorAddr,            \
                          &TransmitApp_epDesc,                  \
                           TRANSMITAPP_CLUSTERID_MOTORCTLMSG,   \
                           TRANSMITAPP_MOTOR_DATA_LEN,          \
                           TransmitApp_Msg,                     \
                          &TransmitApp_MotorTransID,            \
                           TRANSMITAPP_TX_OPTIONS,              \
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
          if(Humit.deviceNWKStatus != DEVICE_NWK_OFFLINE)
          {    
              HalLcdWriteStringValueValue( "Hum:", Humit.data[0], 10, "% T_1:", Humit.data[1], 10, HAL_LCD_LINE_2 );
          }
          else
          {
              HalLcdWriteString( "Hum:OFF T_1:OFF", HAL_LCD_LINE_2 ); 
          }
          
          
          if(soundVb.deviceNWKStatus  != DEVICE_NWK_OFFLINE)
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
          if(TempLight.deviceNWKStatus  != DEVICE_NWK_OFFLINE)
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
          
          if(gasFlame.deviceNWKStatus  != DEVICE_NWK_OFFLINE)
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
          
          
          if(infrared.deviceNWKStatus  != DEVICE_NWK_OFFLINE)
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
      if(relay.deviceNWKStatus == DEVICE_NWK_OFFLINE)
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
      if(motor.deviceNWKStatus == DEVICE_NWK_OFFLINE)
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

