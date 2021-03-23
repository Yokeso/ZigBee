/**************************************************************************************************
  Filename:       Smart_home.c
  Revised:        $Date: 2009-03-29 10:51:47 -0700 (Sun, 29 Mar 2009) $
  Revision:       $Revision: 19585 $

  Description -   Serial Transfer Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
  This sample application is basically a cable replacement
  and it should be customized for your application. A PC
  (or other device) sends data via the serial port to this
  application's device.  This device transmits the message
  to another device with the same application running. The
  other device receives the over-the-air message and sends
  it to a PC (or other device) connected to its serial port.
				
  This application doesn't have a profile, so it handles everything directly.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "Smart_home.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "hal_drivers.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
  #include "hal_lcd.h"
#endif
#include "hal_led.h"
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
const cId_t Smart_home_ClusterList[Smart_home_MAX_CLUSTERS] =
{
  Smart_home_CLUSTERID1,
  Smart_home_CLUSTERID2
};

const SimpleDescriptionFormat_t Smart_home_SimpleDesc =
{
  Smart_home_ENDPOINT,              //  int   Endpoint;
  Smart_home_PROFID,                //  uint16 AppProfId[2];
  Smart_home_DEVICEID,              //  uint16 AppDeviceId[2];
  Smart_home_DEVICE_VERSION,        //  int   AppDevVer:4;
  Smart_home_FLAGS,                 //  int   AppFlags:4;
  Smart_home_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Smart_home_ClusterList,  //  byte *pAppInClusterList;
  Smart_home_MAX_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)Smart_home_ClusterList   //  byte *pAppOutClusterList;
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

static uint8 Smart_home_MsgID;

static afAddrType_t Smart_home_TxAddr;
static uint8 Smart_home_TxSeq;
static uint8 Smart_home_TxBuf[SERIAL_APP_TX_MAX+1];
static uint8 Smart_home_TxLen;

static afAddrType_t Smart_home_RxAddr;
static uint8 Smart_home_RxSeq;
static uint8 Smart_home_RspBuf[SERIAL_APP_RSP_CNT];

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
  Smart_home_RxSeq = 0xC3;

  afRegister( (endPointDesc_t *)&Smart_home_epDesc );

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
  
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, Match_Desc_rsp );
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
  
  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt;

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

      case AF_INCOMING_MSG_CMD:
        Smart_home_ProcessMSGCmd( MSGpkt );
        break;

      default:
        break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    return ( events ^ SYS_EVENT_MSG );
  }

  if ( events & Smart_home_SEND_EVT )
  {
    Smart_home_Send();
    return ( events ^ Smart_home_SEND_EVT );
  }

  if ( events & Smart_home_RESP_EVT )
  {
    Smart_home_Resp();
    return ( events ^ Smart_home_RESP_EVT );
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
            Smart_home_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
            Smart_home_TxAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            Smart_home_TxAddr.endPoint = pRsp->epList[0];
            
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
  zAddrType_t txAddr;
  
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      
      // Initiate an End Device Bind Request for the mandatory endpoint
      txAddr.addrMode = Addr16Bit;
      txAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &txAddr, NLME_GetShortAddr(), 
                            Smart_home_epDesc.endPoint,
                            Smart_home_PROFID,
                            Smart_home_MAX_CLUSTERS, (cId_t *)Smart_home_ClusterList,
                            Smart_home_MAX_CLUSTERS, (cId_t *)Smart_home_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      
      // Initiate a Match Description Request (Service Discovery)
      txAddr.addrMode = AddrBroadcast;
      txAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &txAddr, NWK_BROADCAST_SHORTADDR,
                        Smart_home_PROFID,
                        Smart_home_MAX_CLUSTERS, (cId_t *)Smart_home_ClusterList,
                        Smart_home_MAX_CLUSTERS, (cId_t *)Smart_home_ClusterList,
                        FALSE );
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
  // A message with a serial data block to be transmitted on the serial port.
  case Smart_home_CLUSTERID1:
    // Store the address for sending and retrying.
    osal_memcpy(&Smart_home_RxAddr, &(pkt->srcAddr), sizeof( afAddrType_t ));

    seqnb = pkt->cmd.Data[0];

    // Keep message if not a repeat packet
    if ( (seqnb > Smart_home_RxSeq) ||                    // Normal
        ((seqnb < 0x80 ) && ( Smart_home_RxSeq > 0x80)) ) // Wrap-around
    {
      // Transmit the data on the serial port.
      if ( HalUARTWrite( SERIAL_APP_PORT, pkt->cmd.Data+1, (pkt->cmd.DataLength-1) ) )
      {
        // Save for next incoming message
        Smart_home_RxSeq = seqnb;
        stat = OTA_SUCCESS;
      }
      else
      {
        stat = OTA_SER_BUSY;
      }
    }
    else
    {
      stat = OTA_DUP_MSG;
    }

    // Select approproiate OTA flow-control delay.
    delay = (stat == OTA_SER_BUSY) ? Smart_home_NAK_DELAY : Smart_home_ACK_DELAY;

    // Build & send OTA response message.
    Smart_home_RspBuf[0] = stat;
    Smart_home_RspBuf[1] = seqnb;
    Smart_home_RspBuf[2] = LO_UINT16( delay );
    Smart_home_RspBuf[3] = HI_UINT16( delay );
    osal_set_event( Smart_home_TaskID, Smart_home_RESP_EVT );
    osal_stop_timerEx(Smart_home_TaskID, Smart_home_RESP_EVT);
    break;

  // A response to a received serial data block.
  case Smart_home_CLUSTERID2:
    if ((pkt->cmd.Data[1] == Smart_home_TxSeq) &&
       ((pkt->cmd.Data[0] == OTA_SUCCESS) || (pkt->cmd.Data[0] == OTA_DUP_MSG)))
    {
      Smart_home_TxLen = 0;
      osal_stop_timerEx(Smart_home_TaskID, Smart_home_SEND_EVT);
    }
    else
    {
      // Re-start timeout according to delay sent from other device.
      delay = BUILD_UINT16( pkt->cmd.Data[2], pkt->cmd.Data[3] );
      osal_start_timerEx( Smart_home_TaskID, Smart_home_SEND_EVT, delay );
    }
    break;

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
#if SERIAL_APP_LOOPBACK
  if (Smart_home_TxLen < SERIAL_APP_TX_MAX)
  {
    Smart_home_TxLen += HalUARTRead(SERIAL_APP_PORT, Smart_home_TxBuf+Smart_home_TxLen+1,
                                                    SERIAL_APP_TX_MAX-Smart_home_TxLen);
  }

  if (Smart_home_TxLen)
  {
    (void)Smart_home_TxAddr;
    if (HalUARTWrite(SERIAL_APP_PORT, Smart_home_TxBuf+1, Smart_home_TxLen))
    {
      Smart_home_TxLen = 0;
    }
    else
    {
      osal_set_event(Smart_home_TaskID, Smart_home_SEND_EVT);
    }
  }
#else
  if (!Smart_home_TxLen && 
      (Smart_home_TxLen = HalUARTRead(SERIAL_APP_PORT, Smart_home_TxBuf+1, SERIAL_APP_TX_MAX)))
  {
    // Pre-pend sequence number to the Tx message.
    Smart_home_TxBuf[0] = ++Smart_home_TxSeq;
  }

  if (Smart_home_TxLen)
  {
    if (afStatus_SUCCESS != AF_DataRequest(&Smart_home_TxAddr,
                                           (endPointDesc_t *)&Smart_home_epDesc,
                                            Smart_home_CLUSTERID1,
                                            Smart_home_TxLen+1, Smart_home_TxBuf,
                                            &Smart_home_MsgID, 0, AF_DEFAULT_RADIUS))
    {
      osal_set_event(Smart_home_TaskID, Smart_home_SEND_EVT);
    }
  }
#endif
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
  if (afStatus_SUCCESS != AF_DataRequest(&Smart_home_RxAddr,
                                         (endPointDesc_t *)&Smart_home_epDesc,
                                          Smart_home_CLUSTERID2,
                                          SERIAL_APP_RSP_CNT, Smart_home_RspBuf,
                                         &Smart_home_MsgID, 0, AF_DEFAULT_RADIUS))
  {
    osal_set_event(Smart_home_TaskID, Smart_home_RESP_EVT);
  }
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
  (void)port;

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (Smart_home_TxLen < SERIAL_APP_TX_MAX))
#else
      !Smart_home_TxLen)
#endif
  {
    Smart_home_Send();
  }
}

/*********************************************************************
*********************************************************************/
