/*******************************************************************
  �ļ�����Smart_homeRelay.c
  �� �ߣ� ������
  �� �ܣ� �̵������ܽڵ㣬ʵ�������ؽڵ�ķ����Լ��������ؽڵ����Ϣ
  ������־(2021)
  3.28
  + ���� �������������
         Smart_home_HandleKeys()      //��������
         Smart_home_ProcessMSGCmd();  //������ƺ��� 
  + �޸� Smart_home_ProcessEvt()      //�¼�������
         Smart_home_Init()            //��ʼ������
         Smart_home_Send()            //��Ϣ���ͺ���
  3.23
  + �޸� #define SMART_HOME_SEND_DELAY   500 
         �����Ļ��ʾ��Сbug


ʹ���¼� 
     SMART_HOME_SEND_MSG_EVT(��������)
     SMART_HOME_MATCHRSP_EVT(������ƥ��)
   
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

#include "hal_led.h"
#include "hal_uart.h"
#include "hal_relay.h"

/*********************************************************************
 * MACROS
 */
//3.23  ��Ļ��ʾ��Сbug
#define SMART_HOME_SEND_DELAY   5000
#define SMART_HOME_MATCH_DELAY  1000
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

uint8 Smart_home_TaskID;    // Task ID for internal task/event processing.

//3.21 �������������
// This list should be filled with Application specific Cluster IDs.
const cId_t Smart_home_ClusterList_IN[1] =
{
  Smart_home_CLUSTERID_RELAYCTRL         //�̵������յ���Ϣ��������
};

const cId_t Smart_home_ClusterList_OUT[1] =
{
  Smart_home_CLUSTERID_RELAYSTATUSMSG         //�̵������͵�״̬��Ϣ
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
  Smart_home_MAX_OUTCLUSTERS ,          //  byte  AppNumOutClusters;
  (cId_t *)Smart_home_ClusterList_OUT   //  byte *pAppOutClusterList;
};

const endPointDesc_t Smart_home_epDesc =
{
  Smart_home_ENDPOINT,
 &Smart_home_TaskID,
  (SimpleDescriptionFormat_t *)&Smart_home_SimpleDesc,
  noLatencyReqs
};

afAddrType_t Coordinator_DstAddr;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

byte Coordinator_Msg[RELAYSTATUSMSG_LEN];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static byte Smart_home_MsgID;  // This is the unique message ID (counter)


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
  Smart_home_MsgID = 0;

  // Register the endpoint/interface description with the AF
  afRegister( (endPointDesc_t *)&Smart_home_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( task_id );
  
  //��������
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

  //��ʼ����Э�������͵ĵ�ַ
  Coordinator_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  Coordinator_DstAddr.endPoint = 0;
  Coordinator_DstAddr.addr.shortAddr = 0;
  
  //��ʼ���̵���Ϊȫ��
  HalRelayCtl(0x11);
  
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, Match_Desc_rsp );
  
  // �򿪶�ʱ����������ƥ���¼�
  osal_start_reload_timer( Smart_home_TaskID, SMART_HOME_MATCHRSP_EVT, 
                                               SMART_HOME_SEND_DELAY );
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
  
  // ����ƥ���¼�
  if ( events & SMART_HOME_MATCHRSP_EVT )
  {
     zAddrType_t dstAddr;
     dstAddr.addrMode = AddrBroadcast;
     dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
     
     ZDP_MatchDescReq( &dstAddr, 
                        NWK_BROADCAST_SHORTADDR,
                        Smart_home_PROFID,
                        Smart_home_MAX_OUTCLUSTERS, 
                        (cId_t *)Smart_home_ClusterList_IN,
                        Smart_home_MAX_INCLUSTERS, 
                        (cId_t *)Smart_home_ClusterList_OUT,
                        FALSE );

    // Return unprocessed events
    return (events ^ SMART_HOME_MATCHRSP_EVT);
  }

  if ( events & SMART_HOME_SEND_MSG_EVT )
  {
    Smart_home_Send();
    return ( events ^ SMART_HOME_SEND_MSG_EVT );
  }

  if ( events & SMART_HOME_BINDRSP_EVT )
  {
    Smart_home_Resp();
    return ( events ^ SMART_HOME_BINDRSP_EVT );
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
          // �˴��洢�󶨶Է��ĵ�ַ��Ϣ���ڷ���
          Coordinator_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
          Coordinator_DstAddr.addr.shortAddr = pRsp->nwkAddr;
          // Take the first endpoint, Can be changed to search through endpoints
          Coordinator_DstAddr.endPoint = pRsp->epList[0];  // ok
          // ƥ��ɹ����ر�������ƥ���¼���ʱ��
          osal_stop_timerEx( Smart_home_TaskID, SMART_HOME_MATCHRSP_EVT );
  
#if (HAL_UART == TRUE)
          HalUARTWrite(HAL_UART_PORT_0, "Match Success!\n",   strlen("Match Success!\n"));
#endif
          // ������ʱ�������ݸ�Э�������¼�
          osal_start_reload_timer( Smart_home_TaskID, SMART_HOME_SEND_MSG_EVT, 
                                   SMART_HOME_SEND_DELAY );
        }
    
        osal_mem_free( pRsp );
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
  static uint8 relayStatusCnt;
  uint8 relayCtl;
  
  if ( keys & HAL_KEY_SW_5 )
  {
    relayStatusCnt++;
    // �̵���������״̬
    relayStatusCnt = relayStatusCnt % 4;
    
    switch(relayStatusCnt)
    {
      case 0:
        relayCtl = 0x01;  // K1����K2û�в���(״̬���ֲ���)
        break;
      case 1:
        relayCtl = 0x02;  // K1�أ�K2û�в���(״̬���ֲ���)
        break;
      case 2:
        relayCtl = 0x20;  // K1���䣬K2��
        break;
      case 3:
        relayCtl = 0x10;  // K1���䣬K2��
        break;
      default:
        break;
    }
    
    // ���ͼ̵�����������
    HalRelayCtl(relayCtl);
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
  uint8 mode;
  
  switch ( pkt->clusterId )
  {
    case Smart_home_CLUSTERID_RELAYCTRL:
    {
      // �յ��ļ̵�����������
      mode = pkt->cmd.Data[4];
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
      HalRelayCtl(mode);
      // �����յ�������
#if (HAL_UART == TRUE)
      HalUARTWrite(HAL_UART_PORT_0, & mode,  1);
#endif      
      break;
    } 
    // Could receive control messages in the future.
    default:
      break;
  }
  Smart_home_Send();
  //�յ�֮������������ģʽ�����̷���ֵ�������ӳ�
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
  uint8 tmp;
  
  do{
    // put the sequence number in the message
    tmp = HI_UINT8( Smart_home_MsgID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( Smart_home_MsgID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // ���͸�Э�������� 
    Coordinator_Msg[4] = HalRelayStatus();
    tmp = AF_DataRequest( &Coordinator_DstAddr,                         
                          (endPointDesc_t *)&Smart_home_epDesc,                  
                           Smart_home_CLUSTERID_RELAYSTATUSMSG,
                           RELAYSTATUSMSG_LEN,                 
                           Coordinator_Msg,                    
                          &Smart_home_MsgID,                       
                           AF_DISCV_ROUTE,                     
                           AF_DEFAULT_RADIUS );
  }while (afStatus_SUCCESS == tmp);   
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
  /*
  if (afStatus_SUCCESS != AF_DataRequest(&Smart_home_RxAddr,
                                         (endPointDesc_t *)&Smart_home_epDesc,
                                          Smart_home_CLUSTERID2,
                                          SERIAL_APP_RSP_CNT, Smart_home_RspBuf,
                                         &Smart_home_MsgID, 0, AF_DEFAULT_RADIUS))
  {
    osal_set_event(Smart_home_TaskID, SMART_HOME_BINDRSP_EVT);
  }
  */
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
