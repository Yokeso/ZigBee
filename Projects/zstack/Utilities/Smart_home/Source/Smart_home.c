/**************************************************************************************************
  �ļ�����Smart_home.c
  �� �ߣ� ������
  �� �ܣ� ��Ϊ���ؽڵ㣬ʵ�ִ�������Ϣ�Ĳɼ���������ͨ�������һЩ�ڵ�
          �ն��豸��Ҫ��Ϊ8�����ֱ�Ϊ
     1���¶�����նȴ�������2����������洫������3���̵�����4����ʪ�ȴ�������
     5���������𶯴�������  6����������⴫������7�������8��RFID�������
  ������־(2021)
  3.13
  + ���� 8�ִ�������Project�����úñ������
  + �޸� Smart_home_ProcessEvent() �����Ӹ��ִ��������¼���
         Smart_home_Init()         ����ʼ�������޸ģ�����豸Ĭ�����ߣ�
  3.14
  + �޸� Smart_home_ProcessMSGCmd()������ն˽ڵ������жϣ����崦�����գ�
  + ���� Hal����ִ���������(ͷ�ļ�)
         ClusterList (���8���նˣ��ֿ�������)
  3.17
  + �޸� Smart_home_ProcessMSGCmd() ����ɽ��պ���ڸ��ն˴���
  + ��� Smart_home_HandleKeys()     (�޸�ԭ������������δ��ȫ)
  + ɾ�� static void Smart_home_Send(void);
         static void Smart_home_Resp(void);
         ��������
  3.19
  + ��� ���ֵ�warning������ʶ
  + �޸� Smart_home_HandleKeys() ����ȫ��ƺ�����
  + ���� Smart_home_Device_check() (�豸�Ƿ����߼��)

  3.21
  + ���� DeviceList�Է����ʼ�����豸���߼�飨���ر������֣�
         Smart_home_Display()    ����Ļ��ʾ��δ��ȫ��
         Smart_home_Key_add()     (���ư�����������)
  + �޸� Smart_home_Device_check()
         Smart_home_Init()
         Smart_home_ProcessMSGCmd()
         �����й�DeviceList���豸�б����ֵ��޸�
         ֱ��ʹ������λ�ú���Ϊ�豸���
         �������Ʒ�ʽ���ð����������
  3.22
  + �޸� Smart_home_Display()    ����Ļ��ʾ����ȫ��
         Smart_home_Key_add()     (��Ļ��ʾ�еĵ����̵������ּ�������)
         ��д������ʽ�����ڴ����
         �����޷���ʼ�������⣬��Ҫ��Ŀ�ع�
  3.23
  + Warn �����ʼ��������ֱ�ӽ���key�����,��debug
         OK�������޷��޸�������ʹ��OK�����޸�ΪCencel���� ��ֹ����
  3.23 ��ʽ������ؽڵ㲿�֣�����ʾ�������ֳ��⣩
  3.24 
  + �޸� Smart_home_Key_add()��display��������
  + bug:��ÿʮ��һ�ε��õ����ڼ�麯���лᵼ����ʾ������һ��ʱ�䲻��ʾ
    �ѽ���� ��״̬0��ʱ����Ϊ���߼���
  + ��� ��ʾ��������
  3.28 ����ʱ��
  + ���motor��ʾ��bug
  4.4
  + �����ʪ�����ݲ����Ե�bug
  4.6
  + ��ʽ�õ����нڵ㿪ʼ����
  4.9
  + ���������Bug
  4.14
  + �����ڶ��ն˼���ʱ����ֶ�����������⣬��ʼѰ��
  4.26
  + ������ּ��裬��������ƽ̨��ץ��������
  5.6
  + ����Ӧ��NV��������ΪNV����������д20000�Σ�����Ҫ�̶�λ�ö�ȡ��
  + ������N�죬�о����ڴ����⣬��ʼ�Գ�����и��Ż�����ʱ�任�ռ�  ���ڴ��128�ֽڣ�
  5.8
  + ���ڴ���ԭ�����뷨�Ǳ߷��ߴ������������ᵼ��ϵͳ�����������¼���ѯ��ʱ���͵ķ�ʽ����
  + ��λ������Ч����ʼ����
   
  �����Գ�����bug�����Խ��ո�����������Ϣ��//3.22
  ����û˵������������ڴ�����//3.22
  ��Ŀ�ع�������������ֻ��һ��С������/3.23
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

// �豸���߶�ʱ�����ٽ�ֵ
#define  DEVICE_HEART_BEAT 3

// Send with or without APS ACKs
#define Smart_home_TX_OPTIONS              AF_DISCV_ROUTE

// ��һ��LCD��ʾ��ʱ
#define Smart_home_DISPLAY_DELAY           10000
// LCDÿ����ʾʱ����
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
  Smart_home_CLUSTERID_HUMITMSG,      // ��ʪ��
  Smart_home_CLUSTERID_TEMPLIGHTMSG,  // �¶ȹ���
  Smart_home_CLUSTERID_RFIDMSG,       // ��Ƶ��
  Smart_home_CLUSTERID_GASFLAMEMSG,   // �������
  Smart_home_CLUSTERID_INFRAREDMSG,   // �������
  Smart_home_CLUSTERID_SOUNDVBMSG,    // ������
  Smart_home_CLUSTERID_MOTORSTATUSMSG,// ���״̬
  Smart_home_CLUSTERID_RELAYSTATUSMSG // �̵���״̬
};

const cId_t Smart_home_OutClusterList[Smart_home_MAX_OUTCLUSTERS] =
{
  Smart_home_CLUSTERID_TEXT,    
  Smart_home_CLUSTERID_MOTORCTRL,   // �̵���
  Smart_home_CLUSTERID_RELAYCTRL    // ֱ�����
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
endPointDesc_t Smart_home_epDesc;    // ����ڵ�


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

/*3.18 �ն˽ڵ���豸��ϸ��Ϣ����*/
/*5.6 ����������ɢ�����ķ�ʽ*/
//DeviceInfo   DeviceList[Smart_home_MAX_INCLUSTERS];                      //�豸�б� 
// ��ʪ�Ȼ���, ��һ���ֽ���ʪ�ȣ��ڶ����ֽ����¶�(��������)
DeviceInfo Humit;
// �¶ȹ��ջ���, ǰ�����ֽ����¶�������С��, �������ֽ��ǹ��յ�16λ���� 
DeviceInfo TempLight;
// RFID ��Ϣ
DeviceInfo RfID;
//�������
DeviceInfo gasFlame;
//�������
DeviceInfo infrared;
//���״̬
DeviceInfo motor;
//�̵���״̬
DeviceInfo relay;
//������
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
  // ��ʼ���ڵ�
  Smart_home_epDesc.endPoint = Smart_home_ENDPOINT;   // �˿ں�
  Smart_home_epDesc.task_id = &Smart_home_TaskID;     // ����ID
  Smart_home_epDesc.simpleDesc                         // ���������������˿�
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

  // ������������
  for (i = 0; i < Smart_home_MaxDataLength; i++)
  {
    Coordinator_Msg[i] = 0;
  }

  // ע������MSG
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Smart_home_TaskID, Match_Desc_rsp );
  
  // ����ʾ����һ����ʱ���ڽϳ�
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DISPLAY_EVT, 
                      Smart_home_DISPLAY_DELAY);
  
  // ���豸���߼�⣬��һ�ο��������ʱ�ϳ�ʱ��
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DEVICE_CHECK_EVT, 
                      Smart_home_DEVICE_CHECK_DELAY);
  
  // �ر�LED��(D4)����ʾЭ����Ĭ�ϲ���������
  NLME_PermitJoiningRequest(0x00);
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  HalUARTWrite(HAL_UART_PORT_0, "GOT IT111!\n",   11);
  
  // �豸����״̬����ʼ������ʼ��Ϊ����
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

  // Send a message out, �����Դ��ڻص�
  if ( events & SMART_HOME_SEND_MSG_EVT )
  {      
    // Return unprocessed events
    return (events ^ SMART_HOME_SEND_MSG_EVT);
  }
  
  // �豸״̬����¼�
  if ( events & SMART_HOME_DEVICE_CHECK_EVT )
  {
    // �����豸����״̬��⺯�� 
    Smart_home_Device_check();
    
    // ������Ҫ����������
    osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DEVICE_CHECK_EVT, 
                        Smart_home_DEVICE_CHECK_TIMER);
    
    // Return unprocessed events
    return (events ^ SMART_HOME_DEVICE_CHECK_EVT);
  }
 

  // LCD��ʾ�¼�
  if ( events & SMART_HOME_DISPLAY_EVT )
  {
    // ˢ����ʾ����
    //Smart_home_DisplayResults( dispPage, &scrollLine);
    Smart_home_Display();
    
    // �����Եĵ��ø��¼���ˢ����ʾ����
    osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DISPLAY_EVT, 
                        Smart_home_DISPLAY_TIMER );   
    // Return unprocessed events
    return (events ^ SMART_HOME_DISPLAY_EVT);
  }

  // Smart_home_MATCHRSP_EVT�¼�Ԥ��
  if ( events & SMART_HOME_MATCHRSP_EVT )
  {  
    return (events ^ SMART_HOME_MATCHRSP_EVT);
  }
  // Smart_home_BINDRSP_EVT�¼�Ԥ��
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
    // ����Ϣ����
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        osal_stop_timerEx( Smart_home_TaskID, Smart_home_BINDRSP_EVT);
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
  Smart_home_SendToMotorMSG(cmd,outspeed); 
}


void Smart_home_HandleKeys( byte shift, byte keys )
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
       HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
       //HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Allow networking", HAL_LCD_LINE_4 );
#endif
     }
     else
     {
       NetWorkAllow = 0;
       NLME_PermitJoiningRequest(0x00); // ����������
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
    Ŀǰ�ܹ�8��case
   #define Smart_home_CLUSTERID_HUMITMSG            1  // ��ʪ��
   #define Smart_home_CLUSTERID_TEMPLIGHTMSG        2  // �¶ȹ���
   #define Smart_home_CLUSTERID_RFIDMSG             3  // ��Ƶ��
   #define Smart_home_CLUSTERID_GASFLAMEMSG         4  // �������
   #define Smart_home_CLUSTERID_INFRAREDMSG         5  // �������
   #define Smart_home_CLUSTERID_SOUNDVBMSG          6  // ������
   #define Smart_home_CLUSTERID_MOTORSTATUSMSG      7  // ֱ�����״̬��Ϣ
   #define Smart_home_CLUSTERID_RELAYSTATUSMSG      8  // �̵���״̬��Ϣ
    
   �豸����
   typedef struct DeviceInfo
   {
     uint8 deviceID;
     uint8 deviceStatus;
     uint8 data[5];
   } DeviceInfo; 
   ************************************************************************/ 
    //3.14 ��Ϣ����ģ�飬��8������Ϊ���� 
  //HalUARTWrite(HAL_UART_PORT_0, "9",   1);
void Smart_home_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{

  switch ( pkt->clusterId )
  {
    // ��ʪ�ȴ�������Ϣ
    case Smart_home_CLUSTERID_HUMITMSG:
      Humit.deviceStatus = DEVICE_ONLINE;
      Humit.data[0] = pkt->cmd.Data[4]; // ʪ�� 
      Humit.data[1] = pkt->cmd.Data[5]; // �¶�
      break;
    
    // �¶�����նȴ�������Ϣ  
    case Smart_home_CLUSTERID_TEMPLIGHTMSG:
      TempLight.deviceStatus = DEVICE_ONLINE;
      TempLight.data[0] = pkt->cmd.Data[4]; // �¶�����
      TempLight.data[1] = pkt->cmd.Data[5]; // �¶�С��
      TempLight.data[2] = pkt->cmd.Data[6]; // ����
      TempLight.data[3] = pkt->cmd.Data[7]; // ����
      break;
    
    // RFID��Ƶ����Ϣ 
    case Smart_home_CLUSTERID_RFIDMSG:
      RfID.deviceStatus = DEVICE_ONLINE;
      RfID.data[0] = pkt->cmd.Data[4]; // ��Ƶ������
      RfID.data[1] = pkt->cmd.Data[5]; // 4���ֽڵ�ID��
      RfID.data[2] = pkt->cmd.Data[6]; //
      RfID.data[3] = pkt->cmd.Data[7]; //
      RfID.data[4] = pkt->cmd.Data[8]; //           
      break;
    
    // ��������汨����Ϣ  
    case Smart_home_CLUSTERID_GASFLAMEMSG:
      gasFlame.deviceStatus = DEVICE_ONLINE;
      gasFlame.data[0] = pkt->cmd.Data[4]; // ��������汨����Ϣ
      break;
    
    // �����������Ϣ  
    case Smart_home_CLUSTERID_INFRAREDMSG:
      infrared.deviceStatus = DEVICE_ONLINE;
      infrared.data[0] = pkt->cmd.Data[4]; // ������� 
      break;
    
    // �������񶯴�������Ϣ  
    case Smart_home_CLUSTERID_SOUNDVBMSG:
      soundVb.deviceStatus = DEVICE_ONLINE;
      soundVb.data[0] = pkt->cmd.Data[4]; // ��������Ϣ
      break;
    
    // ���״̬��Ϣ  
    case Smart_home_CLUSTERID_MOTORSTATUSMSG:
      motor.deviceStatus = DEVICE_ONLINE;
      
      // �������豸�������ַ�����ڷ��Ϳ�������
      Smart_home_DstMotorAddr.addrMode = (afAddrMode_t)Addr16Bit;
      Smart_home_DstMotorAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Smart_home_DstMotorAddr.endPoint = 1;  // Ŀ�Ľڵ�Ķ˿ں�
      //Smart_home_DstMotorAddr.endPoint = Smart_home_ENDPOINT;
     
      motor.data[0] = pkt->cmd.Data[4]; // ���ת��
      motor.data[1] = pkt->cmd.Data[5]; // ���״̬
      break;
    
    // �̵���״̬��Ϣ   
    case Smart_home_CLUSTERID_RELAYSTATUSMSG:
      relay.deviceStatus = DEVICE_ONLINE;
      
      // ����̵����豸�������ַ�����ڷ��Ϳ�������
      Smart_home_DstRelayAddr.addrMode = (afAddrMode_t)Addr16Bit;
      Smart_home_DstRelayAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Smart_home_DstRelayAddr.endPoint = 1; // Ŀ�Ľڵ�Ķ˿ں�
      //Smart_home_DstRelayAddr.endPoint = Smart_home_ENDPOINT;  
      
      relay.data[0] = pkt->cmd.Data[4]; 
      break;
      
    // ͬ����һ���������ڽ�����Ӹ���Ŀ�����Ϣ
    default:
      break;
  }
}


/*******************************************************************************
 * @fn      Smart_home_DeviceCheck
 *
 * @brief   check the device  status: online or offline.
 *          �ɺ궨��Smart_home_DEVICE_CHECK_TIMERȷ��������2��
 *
 * @param   none
 *
 * @return  none
 */
/*3.24 ����ļ��ʱ���е㳤 ��Ϊ��������ķ�ʽ�ӿ촦���ٶ�*/
uint8 Device_check(DeviceInfo dev,uint8 count)
{
  if(dev.deviceStatus != DEVICE_ONLINE)    //�豸����
  {
    count++;
  }
  if(dev.deviceStatus == DEVICE_ONLINE)    //�豸����
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
  /*��ʪ�Ȼ��� �¶ȹ��ջ��� RFID ��Ϣ���� ������滺�� 
  ������� ���״̬ �̵���״̬ ������*/
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
  
  // ֻ���豸����ʱ, �����Ϳ�������
  if (relay.deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( Smart_home_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( Smart_home_RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // ���͸��̵����Ŀ������� 
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
  
  // ֻ���豸����ʱ, �����Ϳ�������
  if (motor.deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( Smart_home_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( Smart_home_MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    Coordinator_Msg[4] = speed; // ����ٶ�
    Coordinator_Msg[5] = cmd;   // �������״̬(����)
  
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
          
          if(gasFlame.deviceStatus  != DEVICE_OFFLINE)
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
      if(relay.deviceStatus == DEVICE_OFFLINE)
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
      if(motor.deviceStatus == DEVICE_OFFLINE)
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

