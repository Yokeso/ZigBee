/**************************************************************************************************
  文件名：Smart_homeRouter.c
  作 者： 柳成林
  功 能： 
            (1)路由器的主要功能提供组网服务，当协调器组建好网络后，路由设备加入，
               并接手维持网络的任务。此时，如果协调器离网，不会导致终端设备离线；
               如果协调器需要回到原来的网络中，协调器端必须打开NV编译选项。
            (2)按键(SW5)可打开或关闭组网功能，同时控制LED灯。
               LED灯亮表示可以组网，灯灭表示不支持组网。
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "Smart_home.h"
#include "OnBoard.h"

#include "DebugTrace.h"


#include <string.h>

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_38400
//#define SERIAL_APP_BAUD  HAL_UART_BR_115200
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

#define SERIAL_APP_RSP_CNT  4

// This list should be filled with Application specific Cluster IDs.

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

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void Smart_home_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void Smart_home_HandleKeys( uint8 shift, uint8 keys );
static void Smart_home_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
static void Smart_home_Send(void);
static void Smart_home_Resp(void);
static void Smart_home_CallBack(uint8 port, uint8 event);

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

  Smart_home_TaskID = task_id;

  RegisterForKeys( task_id );

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = TRUE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = Smart_home_CallBack;
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Smart_home", HAL_LCD_LINE_2 );
#endif
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
    
  // Data Confirmation message fields
  ZStatus_t sentStatus;
  byte sentEP;

  
  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt;
    afDataConfirm_t *afDataConfirm;
    
    
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Smart_home_TaskID )) )
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
        break;        
        
      case AF_INCOMING_MSG_CMD:
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
      }
      break;
      
    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
          }
          else
          {
          }
          osal_mem_free( pRsp );
        }
      }
      break;
    default:
      break;
  }
}

/*********************************************************************
 * @fn      Smart_home_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys  - bit field for key events.
 *
 * @return  none
 */
void Smart_home_HandleKeys( uint8 shift, uint8 keys )
{
  static uint8 networkPermitEn;           // 组网功能开关标志
  
  if ( keys & HAL_KEY_SW_5 )
  { 
    networkPermitEn++;
    networkPermitEn = networkPermitEn % 2;
    
    if (networkPermitEn == 0)
    {
      NLME_PermitJoiningRequest(0x00);    // 不允许组网
      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    }
    else if (networkPermitEn == 1)
    {
      NLME_PermitJoiningRequest(0xFF);    // 允许组网
      HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
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
  uint8 stat;
  uint8 seqnb;
  uint8 delay;

  switch ( pkt->clusterId )
  {
    default:
      break;
  }
}

/*********************************************************************
 * @fn      Smart_home_Send
 *
 * @brief   Send data OTA.
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Send(void)
{
}

/*********************************************************************
 * @fn      Smart_home_Resp
 *
 * @brief   Send data OTA.
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Resp(void)
{
}

/*********************************************************************
 * @fn      Smart_home_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void Smart_home_CallBack(uint8 port, uint8 event)
{
}

/*********************************************************************
*********************************************************************/
