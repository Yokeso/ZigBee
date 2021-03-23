/*******************************************************************************
  文 件 名：TransmitAppRelay.c
  作    者：南京安宸博研电子科技有限公司
  创建时间：2013.9.20
  修改时间：2020.12.22
  IAR 版本：IAR for 8051 V8.10.1
  测试平台：Sensor MotherBoard V2.3

  说    明：
  1、程序的驱动文件在hal_relay.c。

  2、本部分的程序使用了以下的编译选项：
     HAL_UART=TRUE
     HAL_UART_TEST
     xPOWER_SAVING (该选项没有打开)
     
     HAL_UART_TEST是为用户增加的编译选项，该选项主要是为了解决在没有LCD的设备上
     观察Z-Stack输出的显示数据。在有LCD的设备上打开LCD_SUPPORTED可以观察网络ID等
     一些基本的信息，打开这个编译选项，有类似的效果。可借助串口显示一些原本在LCD
     上显示的信息，如果串口有其他用途，可通过关闭此编译选项来关闭串口输出的数据。
     
     xPOWER_SAVING选项是终端设备(EndDevice)可以打开的编译选项。

  3、程序使用了以下两个事件：
     TRANSMITAPP_SEND_MSG_EVT(发送数据)
     TRANSMITAPP_MATCHRSP_EVT(描述符匹配)

  4、按键处理函数在TransmitApp_HandleKeys( byte shift, byte keys )，
     本部分程序使用了按键SW5，按键实现了离线情况下的继电器控制功能。

  5、设备自动加入网络，通过描述符匹配事件对目标设备的网络地址进行查询，当匹配成
     功后，关闭该事件的定时器，保存目标设备的网络地址，接着打开发送数据事件，周
     期性地发送数据。发送数据的时间间隔由宏定义TRANSMITAPP_SEND_DELAY设置，修改
     此宏定义即可修改设备发送数据的周期。
     注意：设备发送数据的周期与协调器的离线检测功能是关联的，如果此处的发送间隔
     时间过长(大于6s)，那么协调器就会认定设备离线。因此，修改时间间隔需要与协调
     器的离线检测功能协调一致。
*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "TransmitApp.h"
#include "OnBoard.h"

#include "DebugTrace.h"

#include <string.h>

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_relay.h"

/*******************************************************************************
 * MACROS
 */
#define TRANSMITAPP_SEND_DELAY   1000
#define TRANSMITAPP_MATCH_DELAY  1000

// Send with or without APS ACKs
#define TRANSMITAPP_TX_OPTIONS    AF_DISCV_ROUTE

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

// This is the buffer that is sent out as data.
byte TransmitApp_Msg[ TRANSMITAPP_RELAY_DATA_LEN ];

// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t TransmitApp_InClusterList[1] =
{
  TRANSMITAPP_CLUSTERID_RELAYCTLMSG      // 继电器接收的控制命令ID
};

const cId_t TransmitApp_OutClusterList[1] =
{
  TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG   // 继电器上报状态信息命令ID
};

const SimpleDescriptionFormat_t TransmitApp_SimpleDesc =
{
  TRANSMITAPP_ENDPOINT,                  //  int    Endpoint;
  TRANSMITAPP_PROFID,                    //  uint16 AppProfId[2];
  TRANSMITAPP_DEVICEID,                  //  uint16 AppDeviceId[2];
  TRANSMITAPP_DEVICE_VERSION,            //  int    AppDevVer:4;
  TRANSMITAPP_FLAGS,                     //  int    AppFlags:4;
  TRANSMITAPP_MAX_INCLUSTERS,            //  byte   AppNumInClusters;
  (cId_t *)TransmitApp_InClusterList,    //  byte   *pAppInClusterList;
  TRANSMITAPP_MAX_OUTCLUSTERS,           //  byte   AppNumOutClusters;
  (cId_t *)TransmitApp_OutClusterList    //  byte   *pAppOutClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in TransmitApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t TransmitApp_epDesc;

/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 * EXTERNAL FUNCTIONS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */
#if (HAL_UART == TRUE)
#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_38400
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
#endif

#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  80
#endif
#endif // HAL_UART == TURE

// Task ID for event processing - received when TransmitApp_Init() is called.
byte TransmitApp_TaskID;

static byte TransmitApp_TransID;  // This is the unique message ID (counter)

afAddrType_t TransmitApp_DstAddr;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
void TransmitApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void TransmitApp_HandleKeys( byte shift, byte keys );
void TransmitApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void TransmitApp_SendTheMessage( void );
void TransmitApp_CallBack(uint8 port, uint8 event);

/*******************************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

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
  uint16 i;
  
#if (HAL_UART == TRUE)
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = FALSE;             // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = TransmitApp_CallBack;
  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
#endif
    
  TransmitApp_TaskID = task_id;
  TransmitApp_TransID = 0;   
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  
  TransmitApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  TransmitApp_DstAddr.endPoint = 0;
  TransmitApp_DstAddr.addr.shortAddr = 0;
    
  // Fill out the endpoint description.
  TransmitApp_epDesc.endPoint = TRANSMITAPP_ENDPOINT;
  TransmitApp_epDesc.task_id = &TransmitApp_TaskID;
  TransmitApp_epDesc.simpleDesc
          = (SimpleDescriptionFormat_t *)&TransmitApp_SimpleDesc;
  TransmitApp_epDesc.latencyReq = noLatencyReqs;
  
  // Register the endpoint/interface description with the AF
  afRegister( &TransmitApp_epDesc );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( TransmitApp_TaskID );
 
  // Generate the data
  for (i = 0; i < TRANSMITAPP_RELAY_DATA_LEN; i++)
  {
    TransmitApp_Msg[i] = 0;
  }
  
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, Match_Desc_rsp );
  
  // 初始化继电器，两个都处于关闭状态
  HalRelayCtl(0x11);
  
  // 打开定时器，描述符匹配事件
  osal_start_reload_timer( TransmitApp_TaskID, TRANSMITAPP_MATCHRSP_EVT, 
                                               TRANSMITAPP_MATCH_DELAY );
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

  // Send a message out
  if ( events & TRANSMITAPP_SEND_MSG_EVT )
  {
      TransmitApp_SendTheMessage();

    // Return unprocessed events
    return (events ^ TRANSMITAPP_SEND_MSG_EVT);
  }
  
  // 描述匹配事件
  if ( events & TRANSMITAPP_MATCHRSP_EVT )
  {
     zAddrType_t dstAddr;
     dstAddr.addrMode = AddrBroadcast;
     dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
     
     ZDP_MatchDescReq( &dstAddr, 
                        NWK_BROADCAST_SHORTADDR,
                        TRANSMITAPP_PROFID,
                        TRANSMITAPP_MAX_OUTCLUSTERS, 
                        (cId_t *)TransmitApp_OutClusterList,
                        TRANSMITAPP_MAX_INCLUSTERS, 
                        (cId_t *)TransmitApp_InClusterList,
                        FALSE );

    // Return unprocessed events
    return (events ^ TRANSMITAPP_MATCHRSP_EVT);
  }

  // Discard unknown events
  return 0;
}

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
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
#if (HAL_UART == TRUE)
        HalUARTWrite(HAL_UART_PORT_0, "Bind Success!\n",   strlen("Bind Success!\n"));
#endif
      }
      break;
    
    case Match_Desc_rsp:
      ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
      if ( pRsp )
      {
        if ( pRsp->status == ZSuccess && pRsp->cnt )
        {
          // 此处存储绑定对方的地址信息用于发送
          TransmitApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
          TransmitApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
          // Take the first endpoint, Can be changed to search through endpoints
          TransmitApp_DstAddr.endPoint = pRsp->epList[0];  // ok
          // 匹配成功，关闭描述符匹配事件定时器
          osal_stop_timerEx( TransmitApp_TaskID, TRANSMITAPP_MATCHRSP_EVT );
  
#if (HAL_UART == TRUE)
          HalUARTWrite(HAL_UART_PORT_0, "Match Success!\n",   strlen("Match Success!\n"));
#endif
          // 开启定时发送数据给协调器的事件
          osal_start_reload_timer( TransmitApp_TaskID, TRANSMITAPP_SEND_MSG_EVT, 
                                   TRANSMITAPP_SEND_DELAY );
        }
    
        osal_mem_free( pRsp );
      }
      break;
      
    default:
      break;
  }
}

/*******************************************************************************
 * @fn      TransmitApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. 
 *
 * @return  none
 */
void TransmitApp_HandleKeys( byte shift, byte keys )
{   
  static uint8 relayStatusCnt;
  uint8 relayCmd;
  
  if ( keys & HAL_KEY_SW_5 )
  {
    relayStatusCnt++;
    // 继电器有四种状态
    relayStatusCnt = relayStatusCnt % 4;
    
    switch(relayStatusCnt)
    {
      case 0:
        relayCmd = 0x01;  // K1开，K2没有操作(状态保持不变)
        break;
      case 1:
        relayCmd = 0x02;  // K1关，K2没有操作(状态保持不变)
        break;
      case 2:
        relayCmd = 0x20;  // K1不变，K2开
        break;
      case 3:
        relayCmd = 0x10;  // K1不变，K2关
        break;
      default:
        break;
    }
    
    // 发送继电器控制命令
    HalRelayCtl(relayCmd);
  }
}

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
  uint8 mode;
  
  switch ( pkt->clusterId )
  {
    case TRANSMITAPP_CLUSTERID_RELAYCTLMSG:
    {
      // 收到的继电器控制命令
      mode = pkt->cmd.Data[4];
      HalRelayCtl(mode);
      // 测试收到的命令
#if (HAL_UART == TRUE)
      HalUARTWrite(HAL_UART_PORT_0, & mode,  1);
#endif      
      break;
    } 
    // Could receive control messages in the future.
    default:
      break;
  }
}

/*******************************************************************************
 * @fn      TransmitApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_SendTheMessage( void )
{
  uint8 tmp;
  
  do {
      // put the sequence number in the message
      tmp = HI_UINT8( TransmitApp_TransID );
      tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
      TransmitApp_Msg[2] = tmp;
      tmp = LO_UINT8( TransmitApp_TransID );
      tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
      TransmitApp_Msg[3] = tmp;
  
      // 查询继电器当前状态
      TransmitApp_Msg[4] = HalRelayStatus();
      
      tmp = AF_DataRequest( &TransmitApp_DstAddr,                  \
                            &TransmitApp_epDesc,                   \
                             TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG, \
                             TRANSMITAPP_RELAY_DATA_LEN,           \
                             TransmitApp_Msg,                      \
                            &TransmitApp_TransID,                  \
                             TRANSMITAPP_TX_OPTIONS,               \
                             AF_DEFAULT_RADIUS );

  } while (afStatus_SUCCESS == tmp);
}

/*********************************************************************
 * @fn      TransmitApp_CallBack
 *
 * @brief   串口回调函数.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void TransmitApp_CallBack(uint8 port, uint8 event)
{

}