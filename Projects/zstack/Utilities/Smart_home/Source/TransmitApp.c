/*******************************************************************************
  �� �� ����TransmitApp.c
  ��    �ߣ��Ͼ���已��е��ӿƼ����޹�˾
  ����ʱ�䣺2013.9.20
  �޸�ʱ�䣺2020.12.22
  IAR �汾��IAR for 8051 V8.10.1
  ����ƽ̨��MotherBoard V2.1

  ˵    ����
  ������ʵ��Э�����������ɼ���������Ϣ����LCD����ʾ���ҿ�ͨ�����������豸���С�
  Э��������8���ն��豸���������£�
      1���¶�����նȴ�������2����������洫������3���̵�����4����ʪ�ȴ�������
      5���������𶯴�������  6����������⴫������7�������8��RFID�������

  �ն��豸�����Է�����Ϣ��Э������Э���������ݻ���������Ȼ��ʱ2s��LCD��ʾ������
  ���ն��豸�У�DHT11��ʪ�ȴ�������ÿ��5��ɼ�һ�Σ����඼��ÿ��1��ɼ�һ�Ρ�
 
  ������
       (�˴��İ������������ڲ���ţ��뿪�����ϵı�Ų���һһ��Ӧ)
       SW1(UP)    ������ʾ++(ҳ���Ϸ�)��
       SW2(RIGHT) �����Ƽ̵�����ÿ��һ�°������̵���״̬�л�һ�Σ�
       SW3(DOWN)  ������ʾ--(ҳ���·�)��
       SW4(LEFT)  ������ֱ�������ÿ����һ�ΰ�����ֱ�������״̬�л�һ�Σ�
       SW5(OK)    ����ʾ�л���ͬʱ����Ļ������ʾ����ʾ��Ϣ��Ϊ������
                    1����һ����ʾ�ɼ�������ʪ�ȡ�������Ϣ���̵�����ֱ�������
                      �������ӵ��ת����ʾ����Ҫ������ʾ��
                    2��ȡ����һ���Ĺ�����ʾ��
                    3���ڶ�����ʾ�澯��Ϣ���ڶ���������ʾ��ȫ����Ҫ������ʾ��
                    4��ȡ���ڶ����Ĺ�����ʾ��
       SW7(CANCEL)����/�ر�Э�����������ܣ�������ʾ���������ʾ������
                    ��ע�⣺ϵͳĬ�Ϲر��������ܣ�������Ҫ���밴SW7���򿪡�

  LCD:
      ��ʾ���֣���������Ϣ��ʾ�¼����Զ����еģ�Ĭ����ʾ���ǵ�һ����Ϣ���ֶ�
      ����SW5�����л����ڶ������ڶ���Ĭ�Ϲ�����ʾ���ٴΰ��°���SW5���Թرչ�
      ����ʾ����ʱ����SW1��SW3���Կ��ƹ�����Ļ���ϻ������£�
      LCD��ʾ����˵����
               ��һ����H:OFF T:OFF(ʪ�ȡ��¶��豸����)
                       T:32.5C L:320L(�¶�32.5�ȣ�����ǿ�ȣ�320����)
                       K1:off K2:on(�̵���K1�رգ�K2�򿪣������OFF��ʾ����)
                       Motor:Stop(���ͣת�������OFF��ʾ����)
                       Speed:0RPS(���Ŀǰת��Ϊ0ת/����)

               �ڶ�����Gas:OFF(������������)
                       Flame:OFF(���洫��������)
                       Sound:ON(�������������ߣ�û�б���)
                       Vibrate:Alarm!(�𶯱���)
                       Infrared:ON(������������ߣ�û�б���)
                       Card:OFF(��Ƶ���豸����)
                               (���пհף���Ϊ��Ƶ���豸���ߣ�����û��ID��Ϣ)
  
  �豸���߼�⣺
      ��Ҫ��TransmitApp_DeviceNWKCheck(void)�������������豸�Ƿ����޶���ʱ����
      �������ݣ������ʱ���ж��豸���ߡ�����豸�������ݣ���ô��Ӧ��deviceInfo_t
      �ṹ���е�deviceNWKStatus���Ϊ1�������ʱ�����Ϊ0����ʾ���ֵĺ���ֻ���
      ȡ�豸������״̬�Ϳ����ж��Ƿ������ˡ�
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

// �豸���߶�ʱ�����ٽ�ֵ
#define  DEVICE_NWK_CNT_LIMIT 3

// Send with or without APS ACKs
#define TRANSMITAPP_TX_OPTIONS              AF_DISCV_ROUTE

// ��һ��LCD��ʾ��ʱ
#define TRANSMITAPP_DISPLAY_DELAY           10000
// LCDÿ����ʾʱ����
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
  TRANSMITAPP_CLUSTERID_HUMITMSG,      // ��ʪ��
  TRANSMITAPP_CLUSTERID_TEMPLIGHTMSG,  // �¶ȹ���
  TRANSMITAPP_CLUSTERID_RFIDMSG,       // ��Ƶ��
  TRANSMITAPP_CLUSTERID_GASFLAMEMSG,   // �������
  TRANSMITAPP_CLUSTERID_INFRAREDMSG,   // �������
  TRANSMITAPP_CLUSTERID_SOUNDVBMSG,    // ������
  TRANSMITAPP_CLUSTERID_MOTORSTATUSMSG,// ���״̬
  TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG // �̵���״̬
};

const cId_t TransmitApp_OutClusterList[TRANSMITAPP_MAX_OUTCLUSTERS] =
{
  TRANSMITAPP_CLUSTERID_TESTMSG,    
  TRANSMITAPP_CLUSTERID_RELAYCTLMSG,   // �̵���
  TRANSMITAPP_CLUSTERID_MOTORCTLMSG    // ֱ�����
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
endPointDesc_t TransmitApp_epDesc;    // ����ڵ�


/*******************************************************************************
 *��������
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

/*������ʾ������صĺ�*/
#define LCD_PAGE_MAX         4     //ĿǰĿ¼ҳ���4ҳ

static int8 Ctrlcase = 0;    //0����������Ļ��ʾ��1���Ƽ̵�����2���Ƶ��
static int8 LCD_Page  =  0;  //�ն�״̬��ʾ
/* ֱ�����״̬(status)����*/
#define  HAL_MOTOR_STOP            0x01
#define  HAL_MOTOR_FORWARD         0x02
#define  HAL_MOTOR_BACKWARD        0x03
#define  MOTOR_MAX_SPEED           2400

/*********************************************************************
 * LOCAL VARIABLES
 */
// ��ʪ�Ȼ���, ��һ���ֽ���ʪ�ȣ��ڶ����ֽ����¶�(��������)
deviceInfo_t Humit;
// �¶ȹ��ջ���, ǰ�����ֽ����¶�������С��, �������ֽ��ǹ��յ�16λ���� 
deviceInfo_t TempLight;
// RFID ��Ϣ
deviceInfo_t RfID;
//�������
deviceInfo_t gasFlame;
//�������
deviceInfo_t infrared;
//���״̬
deviceInfo_t motor;
//�̵���״̬
deviceInfo_t relay;
//������
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

// LCD��ʾ
static uint8 dispPage=1;
static uint8 scrollLine=0;
//LCD��Ļ��������
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
  // ��ʼ���ڵ�
  TransmitApp_epDesc.endPoint = TRANSMITAPP_ENDPOINT;   // �˿ں�
  TransmitApp_epDesc.task_id = &TransmitApp_TaskID;     // ����ID
  TransmitApp_epDesc.simpleDesc                         // ���������������˿�
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

  // ������������
  for (i = 0; i < TransmitApp_MaxDataLength; i++)
  {
    TransmitApp_Msg[i] = 0;
  }

  // ע������MSG
  // ����Ҫһ�ּ���, ��������?????
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, Match_Desc_rsp );
  
  // ����ʾ����һ����ʱ���ڽϳ�
  osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DISPLAY_EVT, 
                      TRANSMITAPP_DISPLAY_DELAY);
  
  // ���豸���߼�⣬��һ�ο��������ʱ�ϳ�ʱ��
  osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DEVICE_CHECK_EVT, 
                      TRANSMITAPP_DEVICE_CHECK_DELAY);
  
  // �ر�LED��(D4)����ʾЭ����Ĭ�ϲ���������
  NLME_PermitJoiningRequest(0x00);
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  
  // �豸����״̬����ʼ������ʼ��Ϊ����
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

  // Send a message out, ��ʵ��Э����û�����ڷ���������
  if ( events & TRANSMITAPP_SEND_MSG_EVT )
  {      
    // Return unprocessed events
    return (events ^ TRANSMITAPP_SEND_MSG_EVT);
  }
  
  // �豸״̬����¼�
  if ( events & TRANSMITAPP_DEVICE_CHECK_EVT )
  {
    // �����豸����״̬��⺯��
    TransmitApp_DeviceNWKCheck(); 
    
    // ������Ҫ����������
    osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DEVICE_CHECK_EVT, 
                        TRANSMITAPP_DEVICE_CHECK_TIMER);
    
    // Return unprocessed events
    return (events ^ TRANSMITAPP_DEVICE_CHECK_EVT);
  }
 

  // LCD��ʾ�¼�
  if ( events & TRANSMITAPP_DISPLAY_EVT )
  {
    // ����������ܴ򿪣��Զ����Ϲ���һ��
    if (autoScrollEnable == 1)
    {
        scrollLine++;
    }
    // ˢ����ʾ����
    //TransmitApp_DisplayResults( dispPage, &scrollLine);
    Smart_home_Display();
    
    // �����Եĵ��ø��¼���ˢ����ʾ����
    osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_DISPLAY_EVT, 
                        TRANSMITAPP_DISPLAY_TIMER );   
    // Return unprocessed events
    return (events ^ TRANSMITAPP_DISPLAY_EVT);
  }

  // TRANSMITAPP_MATCHRSP_EVT�¼�Ԥ��
  if ( events & TRANSMITAPP_MATCHRSP_EVT )
  {  
    return (events ^ TRANSMITAPP_MATCHRSP_EVT);
  }
  // TRANSMITAPP_BINDRSP_EVT�¼�Ԥ��
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
    // ����Ϣ����
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        osal_stop_timerEx( TransmitApp_TaskID, TRANSMITAPP_BINDRSP_EVT);
#if defined ( LCD_SUPPORTED )
        HalLcdWriteString( "BindSuccess", HAL_LCD_LINE_3 );
#endif
      }
      break;

    // ������ƥ����Ϣ���� 
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
 * @brief   ͨ������ٶ��жϵ��ת��
 * 
 * 
 * @param   uint8 ���ת��
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
  /*������ô�����ƺ�������ڴ�����
  static uint8 Relay1_on = 0x02;  //����Ĭ�϶��ǹر�
  static uint8 Relay2_on = 0x10;
  */
  /*3.21 LCD ��ʾ*/
  static uint8 Relay1_on = 0;  //����Ĭ�϶��ǹر�
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
  /*3.22 �����ڵ���ʱ�������⣬��������Ϊ ���upʱ�������Ļ��Ϊ��ֹ����
    ���Ʒ����Ϊ���Ƶ�����������ڻ���е�keyֵ�����  keys & HAL_KEY_SW_2 = 2
    ���������ʱ��ֹ����Ҫ�ȴ��о�  ����key_7*/
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
       NLME_PermitJoiningRequest(0xFF); // ������������ʱ����
       //HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
       //HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Allow networking", HAL_LCD_LINE_4 );
#endif
     }
     else
     {
       NetWorkAllow = 0;
       NLME_PermitJoiningRequest(0x00); // ����������
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
    // ��ʪ�ȴ�������Ϣ
    case TRANSMITAPP_CLUSTERID_HUMITMSG:
      Humit.deviceNWKStatus = DEVICE_NWK_ONLINE;
      Humit.data[0] = pkt->cmd.Data[4]; // ʪ�� 
      Humit.data[1] = pkt->cmd.Data[5]; // �¶�
      break;
    
    // �¶�����նȴ�������Ϣ  
    case TRANSMITAPP_CLUSTERID_TEMPLIGHTMSG:
      TempLight.deviceNWKStatus = DEVICE_NWK_ONLINE;
      TempLight.data[0] = pkt->cmd.Data[4]; // �¶�����
      TempLight.data[1] = pkt->cmd.Data[5]; // �¶�С��
      TempLight.data[2] = pkt->cmd.Data[6]; // ����
      TempLight.data[3] = pkt->cmd.Data[7]; // ����
      break;
    
    // RFID��Ƶ����Ϣ 
    case TRANSMITAPP_CLUSTERID_RFIDMSG:
      RfID.deviceNWKStatus = DEVICE_NWK_ONLINE;
      RfID.data[0] = pkt->cmd.Data[4]; // ��Ƶ������
      RfID.data[1] = pkt->cmd.Data[5]; // 4���ֽڵ�ID��
      RfID.data[2] = pkt->cmd.Data[6]; //
      RfID.data[3] = pkt->cmd.Data[7]; //
      RfID.data[4] = pkt->cmd.Data[8]; //           
      break;
    
    // ��������汨����Ϣ  
    case TRANSMITAPP_CLUSTERID_GASFLAMEMSG:
      gasFlame.deviceNWKStatus = DEVICE_NWK_ONLINE;
      gasFlame.data[0] = pkt->cmd.Data[4]; // ��������汨����Ϣ
      break;
    
    // �����������Ϣ  
    case TRANSMITAPP_CLUSTERID_INFRAREDMSG:
      infrared.deviceNWKStatus = DEVICE_NWK_ONLINE;
      infrared.data[0] = pkt->cmd.Data[4]; // ������� 
      break;
    
    // �������񶯴�������Ϣ  
    case TRANSMITAPP_CLUSTERID_SOUNDVBMSG:
      soundVb.deviceNWKStatus = DEVICE_NWK_ONLINE;
      soundVb.data[0] = pkt->cmd.Data[4]; // ��������Ϣ
      break;
    
    // ���״̬��Ϣ  
    case TRANSMITAPP_CLUSTERID_MOTORSTATUSMSG:
      motor.deviceNWKStatus = DEVICE_NWK_ONLINE;
      
      // �������豸�������ַ�����ڷ��Ϳ�������
      TransmitApp_DstMotorAddr.addrMode = (afAddrMode_t)Addr16Bit;
      TransmitApp_DstMotorAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      TransmitApp_DstMotorAddr.endPoint = 1;  // Ŀ�Ľڵ�Ķ˿ں�
      //TransmitApp_DstMotorAddr.endPoint = TRANSMITAPP_ENDPOINT;
     
      motor.data[0] = pkt->cmd.Data[4]; // ���ת��
      motor.data[1] = pkt->cmd.Data[5]; // ���״̬
      break;
    
    // �̵���״̬��Ϣ   
    case TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG:
      relay.deviceNWKStatus = DEVICE_NWK_ONLINE;
      
      // ����̵����豸�������ַ�����ڷ��Ϳ�������
      TransmitApp_DstRelayAddr.addrMode = (afAddrMode_t)Addr16Bit;
      TransmitApp_DstRelayAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      TransmitApp_DstRelayAddr.endPoint = 1; // Ŀ�Ľڵ�Ķ˿ں�
      //TransmitApp_DstRelayAddr.endPoint = TRANSMITAPP_ENDPOINT;  
      
      relay.data[0] = pkt->cmd.Data[4]; 
      break;
      
    // ͬ����һ���������ڽ�����Ӹ���Ŀ�����Ϣ
    default:
      break;
  }
}

/*******************************************************************************
 * @fn      TransmitApp_DeviceNWKCheck
 *
 * @brief   check the device NWK status: online or offline.
 *          �ɺ궨��TRANSMITAPP_DEVICE_CHECK_TIMERȷ��������2��
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_DeviceNWKCheck(void)
{
  // �豸���߼������������ٽ�ֵ�ж��豸����
  static uint8 humitCnt, tempLightCnt, rfIDCnt, gasFlameCnt, infraredCnt;
  static uint8 motorCnt, relayStatusCnt, soundVbCnt;
  
  // ��ʪ�ȴ�����
  if (Humit.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
    humitCnt++;
  }
  if (Humit.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
    humitCnt = 0;
    Humit.deviceNWKStatus = 0;
  }
  if (humitCnt > DEVICE_NWK_CNT_LIMIT)            // ��ʱ, ��Ϊ����
  {
    humitCnt = DEVICE_NWK_CNT_LIMIT;
    Humit.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
  
  // �¶�����նȴ�����
  if (TempLight.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
    tempLightCnt++;
  }
  if (TempLight.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
    tempLightCnt = 0;
    TempLight.deviceNWKStatus = 0;
  }
  if (tempLightCnt > DEVICE_NWK_CNT_LIMIT)            // ��ʱ, ��Ϊ����
  {
    tempLightCnt = DEVICE_NWK_CNT_LIMIT;
    TempLight.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
  
  // RFID������
  if (RfID.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
      rfIDCnt++;
  }
  if (RfID.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
      rfIDCnt = 0;
      RfID.deviceNWKStatus = 0;
  }
  if (rfIDCnt > DEVICE_NWK_CNT_LIMIT)            // ��ʱ, ��Ϊ����
  {
      rfIDCnt = DEVICE_NWK_CNT_LIMIT;
      RfID.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // ��������洫����  
  if (gasFlame.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
    gasFlameCnt++;
  }
  if (gasFlame.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
    gasFlameCnt = 0;
    gasFlame.deviceNWKStatus = 0;
  }
  if (gasFlameCnt > DEVICE_NWK_CNT_LIMIT)            // ��ʱ, ��Ϊ����
  {
    gasFlameCnt = DEVICE_NWK_CNT_LIMIT;
    gasFlame.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
  
  // ��������⴫����
  if (infrared.deviceNWKStatus != DEVICE_NWK_ONLINE)  // �豸���ߣ�����+1
  {
    infraredCnt++;
  }
  if (infrared.deviceNWKStatus == DEVICE_NWK_ONLINE)  // �豸���ߣ���������
  {
    infraredCnt = 0;
    infrared.deviceNWKStatus = 0;
  }
  if (infraredCnt > DEVICE_NWK_CNT_LIMIT)             // ��ʱ, ��Ϊ����
  {
    infraredCnt = DEVICE_NWK_CNT_LIMIT;
    infrared.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // �����񶯴�����  
  if (soundVb.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
    soundVbCnt++;
  }
  if (soundVb.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
    soundVbCnt++;
    soundVb.deviceNWKStatus = 0;
  }
  if (soundVbCnt > DEVICE_NWK_CNT_LIMIT)            // ��ʱ, ��Ϊ����
  {
    soundVbCnt = DEVICE_NWK_CNT_LIMIT;
    soundVb.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // ΢��ֱ�����RF-310T/QJT310AH 
  if (motor.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
    motorCnt++;
  }
  if (motor.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
    motorCnt = 0;
    motor.deviceNWKStatus = 0;
  }
  if (motorCnt > DEVICE_NWK_CNT_LIMIT)            // ��ʱ, ��Ϊ����
  {
    motorCnt = DEVICE_NWK_CNT_LIMIT;
    motor.deviceNWKStatus = DEVICE_NWK_OFFLINE;
  }
    
  // �̵���  
  if (relay.deviceNWKStatus != DEVICE_NWK_ONLINE) // �豸���ߣ�����+1
  {
    relayStatusCnt++;
  }
  if (relay.deviceNWKStatus == DEVICE_NWK_ONLINE) // �豸���ߣ���������
  {
    relayStatusCnt = 0;
    relay.deviceNWKStatus = 0;
  }
  if (relayStatusCnt > DEVICE_NWK_CNT_LIMIT)      // ��ʱ, ��Ϊ����
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
  
  // ֻ���豸����ʱ, �����Ϳ�������
  if (relay.deviceNWKStatus != DEVICE_NWK_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( TransmitApp_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[2] = tmp;
    tmp = LO_UINT8( TransmitApp_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[3] = tmp;
    
    // ���͸��̵����Ŀ������� 
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
  
  // ֻ���豸����ʱ, �����Ϳ�������
  if (motor.deviceNWKStatus != DEVICE_NWK_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( TransmitApp_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[2] = tmp;
    tmp = LO_UINT8( TransmitApp_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[3] = tmp;
    
    TransmitApp_Msg[4] = speed; // ����ٶ�
    TransmitApp_Msg[5] = cmd;   // �������״̬(����)
  
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
 * @brief   ��Ļ���Ժ��������������������ʾ.
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
      //�����Ļ��ʾ
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      
      switch(LCD_Page)
      {
        default:
        case 0:
        { 
          //DeviceInfo* Devhum = &DeviceList[Humit];
          //DeviceInfo* DevSound = &DeviceList[soundVb];
          //��һҳ�ڶ�����ʾ�¶�
          //      ��������ʾ����
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
          
          
          //��ʾ������İٷֱ���
          percent = (1 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
        }
        case 1:
        {
          //�ڶ�ҳֻ�й���
          //��һҳ�ڶ�����ʾ�¶�
          //      ��������ʾ����
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
          
          //��ʾ������İٷֱ���
          percent = (2 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
          
          
        }
        
      case 2:
      {
          static uint16 Data1;
          static uint16 Data2;
          //��ʾ������İٷֱ���
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
              if(gasFlame.data[0] & 0x01 == 0x01) { HalLcdWriteString( "GasFlame: Flame", HAL_LCD_LINE_2 ); }         //0λ�ǻ���
              else if(gasFlame.data[0] & 0x02 == 0x02) { HalLcdWriteString( "GasFlame: Gas", HAL_LCD_LINE_2 ); }//1λ������
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

          
          
          //��ʾ������İٷֱ���
          percent = (4 * 100) / LCD_PAGE_MAX;
          HalLcdDisplayPercentBar("",percent);
          break;
      }
          
    }
      
      break;
    
    case 1:
      HalLcdWriteString( "Relay Contrling", HAL_LCD_LINE_4 ); 
      //�����Ļ��ʾ
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      //�����deviceStatus ������0x00,���Ի����bug ���0��Device���ʱ������ 2.24
      if(relay.deviceNWKStatus == DEVICE_NWK_OFFLINE)
      {
        HalLcdWriteString( "Relay Offline", HAL_LCD_LINE_1 );      
      }
      else  //�豸���߻���  �豸���߿����� 0/1
      {
        HalLcdWriteString( "Relay Online", HAL_LCD_LINE_1 ); 
        //�̵������ƽ���

        if((relay.data[0]& 0x02) == 0x02) {HalLcdWriteString( "K1:ON", HAL_LCD_LINE_2 );}
        if((relay.data[0] & 0x01) == 0x01) {HalLcdWriteString( "K1:OFF", HAL_LCD_LINE_2 );}
        if((relay.data[0] & 0x20) == 0x20) {HalLcdWriteString( "K2:ON", HAL_LCD_LINE_3 );}
        if((relay.data[0] & 0x10) == 0x10) {HalLcdWriteString( "K2:OFF", HAL_LCD_LINE_3 );}      
      }

      break;
    
    case 2:
      HalLcdWriteString( "Motor Contrling", HAL_LCD_LINE_4 );
      //�����Ļ��ʾ
      HalLcdWriteString( " ", HAL_LCD_LINE_2 ); 
      HalLcdWriteString( " ", HAL_LCD_LINE_3 );
      if(motor.deviceNWKStatus == DEVICE_NWK_OFFLINE)
      {
        HalLcdWriteString( "Motor Offline", HAL_LCD_LINE_1 );      
      }
      else  //�豸���߻���  �豸���߿����� 0/1
      {
        HalLcdWriteString( "Motor Online", HAL_LCD_LINE_1 ); 
        //������ƽ���
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

