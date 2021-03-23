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
         Smart_home_Init()         ����ʼ�������޸ģ������豸Ĭ�����ߣ�
  3.14
  + �޸� Smart_home_ProcessMSGCmd()�������ն˽ڵ������жϣ����崦�����գ�
  + ���� Hal����ִ���������(ͷ�ļ�)
         ClusterList (����8���նˣ��ֿ�������)
  3.17
  + �޸� Smart_home_ProcessMSGCmd() ����ɽ��պ���ڸ��ն˴�����
  + ���� Smart_home_HandleKeys()     (�޸�ԭ�������������δ��ȫ)
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
         �����й�DeviceList���豸�б������ֵ��޸�
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
   
  �����Գ�����bug�����Խ��ո�����������Ϣ��//3.22
  ����û˵������������ڴ�����//3.22
  ��Ŀ�ع�������������ֻ��һ��С������/3.23
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
/*�豸�����������*/
/*�ж��豸�Ƿ����ߵ�������*/
#define DEVICE_HEART_BEAT    3
#define DEVICE_CHECK_DELAY   5000

/*LCD�������*/
/*������ʾʱ��   MS*/
#define LCD_DISPLAY_LENGTH   10000
#define LCD_DISPLAY_TIMER    3000  //��ø���һ�� 

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
  Smart_home_CLUSTERID_HUMITMSG,              // ��ʪ��
  Smart_home_CLUSTERID_TEMPLIGHTMSG,          // �¶ȹ���
  Smart_home_CLUSTERID_RFIDMSG,               // ��Ƶ��
  Smart_home_CLUSTERID_GASFLAMEMSG,           // ��������
  Smart_home_CLUSTERID_INFRAREDMSG,           // �������
  Smart_home_CLUSTERID_SOUNDVBMSG,            // ������
  Smart_home_CLUSTERID_MOTORSTATUSMSG,        // ֱ�����״̬��Ϣ
  Smart_home_CLUSTERID_RELAYSTATUSMSG         // �̵���
};

const cId_t Smart_home_ClusterList_OUT[Smart_home_MAX_INCLUSTERS ] =
{
  Smart_home_CLUSTERID_MOTORCTRL,             //ֱ���������
  Smart_home_CLUSTERID_RELAYCTRL              //�̵�������
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

/*3.18 �ն˽ڵ���豸��ϸ��Ϣ����*/
static uint8 DeviceCnt[Smart_home_MAX_INCLUSTERS];
DeviceInfo DeviceList[Smart_home_MAX_INCLUSTERS];                      //�豸�б�  
/*********************************************************************************
//��Щ�ƺ��������ڴ�����  3.22
DeviceInfo *Humit = DeviceList+Smart_home_CLUSTERID_HUMITMSG;          //��ʪ�Ȼ���
DeviceInfo *TempLight = DeviceList+Smart_home_CLUSTERID_TEMPLIGHTMSG;  // �¶ȹ��ջ���
DeviceInfo *RfID = DeviceList+Smart_home_CLUSTERID_RFIDMSG;            // RFID ��Ϣ����
DeviceInfo *gasFlame = DeviceList+Smart_home_CLUSTERID_GASFLAMEMSG;    //������滺��
DeviceInfo *infrared = DeviceList+Smart_home_CLUSTERID_INFRAREDMSG;    //�������
DeviceInfo *motor = DeviceList+Smart_home_CLUSTERID_MOTORSTATUSMSG;     //���״̬
DeviceInfo *relay = DeviceList+Smart_home_CLUSTERID_RELAYSTATUSMSG;    //�̵���״̬
DeviceInfo *soundVb = DeviceList+Smart_home_CLUSTERID_SOUNDVBMSG;      //������
�޸�����
***********************************************************************************/
#define Humit     1     //��ʪ�Ȼ���
#define TempLight 2     // �¶ȹ��ջ���
#define RfID      3     // RFID ��Ϣ����
#define gasFlame  4     //������滺��
#define infrared  5     //�������
#define motor     6     //���״̬
#define relay     7     //�̵���״̬
#define soundVb   8     //������

/*3.14 ��Ϣ���Ͳ���*/
/*3.17�̵�ַ�洢*/
static afAddrType_t Relay_addr;
static afAddrType_t Motor_addr;

/*��Ϣ��������*/
byte Coordinator_Msg[MSG_MAX_LEN];


/*3.19 Э��ջ���е���ʱ��֪��Ҫ���Ǹ���Ķ���*/
/*3.21 �����һ��ˣ�����*/
/*3.22 ��Щû�ã�ɾ���ˣ�*/
//static uint8 Smart_home_MsgID;
static uint8 RelayTransID;  // This is the unique message ID (counter)
static uint8 MotorTransID;  // This is the unique message ID (counter)
static afAddrType_t Smart_home_TxAddr;
static uint8 Smart_home_TxSeq;
static uint8 Smart_home_TxBuf[SMART_HOME_TX_MAX+1];
static uint8 Smart_home_TxLen;

static afAddrType_t Smart_home_RxAddr;
static uint8 Smart_home_RxSeq;
static uint8 Smart_home_RspBuf[SMART_HOME_RSP_CNT];

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void Smart_home_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void Smart_home_Key_add(uint8 Ctrlcase);
static void Smart_home_HandleKeys( byte shift, byte keys );
static void Smart_home_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
static void Smart_home_Device_check(void);
static void SerialApp_Send(void);
static void SerialApp_Resp(void);
static void Smart_home_Relay_Ctl(uint8 cmd);
static void Smart_home_Motor_Ctl(uint8 cmd,uint8 speed);
static void Smart_home_Display(uint8 page);
static void SerialApp_CallBack(uint8 port, uint8 event);

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
  uint8 DeviceNum;                                    //DeviceList��ʼ���ñ���

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
  
  //3.22 ��ʾ��ʱ
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DISPLAY_EVT, 
                      SMART_HOME_DEVICE_DISPLAY_DELAY);
  
  
  //3.14 ���豸���߼�⣬��һ�ο��������ʱ�ϳ�ʱ��
  osal_start_timerEx( Smart_home_TaskID, SMART_HOME_DEVICE_CHECK_EVT, 
                      SMART_HOME_DEVICE_CHECK_DELAY);
 
  //3.13 �ر�LED��(D4)����ʾЭ����Ĭ�ϲ���������
  NLME_PermitJoiningRequest(0x00);
  HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
  
  //3.21 ��ʼ��ҳ��Ϊ0 ��һҳ
  //LCD_Page=0;
  
  //3.13 �豸����״̬����ʼ������ʼ��Ϊ����
  //3.21 �޸�ΪDeviceList��ʽ����ʼ��DeviceCnt
  for(DeviceNum=1;DeviceNum<Smart_home_MAX_INCLUSTERS;DeviceNum++) //ֻ��ʼ���ն�
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
  
  //3.13 ����ȷ����Ϣ�ֶ�
  ZStatus_t sentStatus;
  byte sentEP;
  
  if ( events & SYS_EVENT_MSG )   //3.13 ϵͳ��Ϣ�¼�
  {
    afIncomingMSGPacket_t *MSGpkt;

    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Smart_home_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:           //3.13 ZDO������Ϣ�ص�
          Smart_home_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
          
        case KEY_CHANGE:           //3.13 �����¼�
          Smart_home_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
         
        case AF_DATA_CONFIRM_CMD:
        //3.13 ���յ�����Ϣ����Ϊ�Է��͵����ݰ���ȷ�ϡ�
        //״̬ΪZStatus_t����[��ZComDef.h�ж���]
        //��Ϣ�ֶ���AF.h�ж���
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
 
          if ( (ZSuccess == sentStatus) &&
               (Smart_home_epDesc.endPoint == sentEP) )
          {  
            //3.13 ����Ϣ����ȷ�ϳɹ����̵���˸һ��
            HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
          }
          else
          {
            //3.13 �����˸һ��  Ӳ����ûʵ��
            //HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK);
          }
          break;
          
        case AF_INCOMING_MSG_CMD:   //3.13 ��Ϣ���봦��
          //3.13 ����Ϣ����ȷ�ϳɹ����̵���˸һ��
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
  
  if ( events & SMART_HOME_DEVICE_CHECK_EVT )  //�ն��豸����¼�
  {
    Smart_home_Device_check();  //�����ն��豸��麯��
    
    osal_start_timerEx(Smart_home_TaskID,SMART_HOME_DEVICE_CHECK_EVT,
                       DEVICE_CHECK_DELAY);
    return (events ^ SMART_HOME_DEVICE_CHECK_EVT);
  }
  
  if( events & SMART_HOME_DISPLAY_EVT )
  {
    //Smart_home_Display( dispPage);
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
 * @brief   3.21������������ʶĿǰ���ڵĿ���״̬���Ǹ�����
 *          0 ������Ļ 1 ���Ƽ̵��� 2 ���Ƶ��
 *            
 *
 * @param   Ctrlcase Ŀǰ�Ŀ��ƺ�
 *
 * @return  ��Ļ����
 */
static void Smart_home_Key_add(uint8 Ctrlcase)
{
#if defined ( LCD_SUPPORTED )
  switch(Ctrlcase)
  {
    default:    
    case 0:
      HalLcdWriteString( "Flip use UP/DOWN", HAL_LCD_LINE_4 ); 
      break;
    
    case 1:
      HalLcdWriteString( "Relay Contrling", HAL_LCD_LINE_4 ); 
      if(DeviceList[relay].deviceStatus == DEVICE_ONLINE)  //�豸���߻���
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
      if(DeviceList[motor].deviceStatus == DEVICE_ONLINE)  //�豸���߻���
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

/*********************************************************************
 * @fn      Smart_home_HandleKeys
 *
 * @brief   ���ð����¼�
 *          Ŀǰ�¼�����    3.19
 *
 *          HAL_KEY_SW_1--UP     (�Ϸ�һҳ)
 *          HAL_KEY_SW_2--RIGHT  (ѡ������)
 *          HAL_KEY_SW_3--DOWN   (�·�һҳ)
 *          HAL_KEY_SW_4--LEFT   (ѡ������)
 *          HAL_KEY_SW_5--OK     (������)
 *          HAL_KEY_SW_7--CANCEL (������) 
 * + 3.21 ����  ѡ���ܹ���3�֣�������Ļ��ʾ�����Ƽ̵����Ϳ��Ƶ��
 *              ���Ƽ̵���ʱ UP����1��DOWN����2
 *              ���Ƶ��ʱ   UP���٣� DOWN����
 *
 * @param   shift - true if in shift/alt.
 * @param   keys  - bit field for key events.
 *
 * @return  none
 */
void Smart_home_HandleKeys( byte shift, byte keys )
{
  //zAddrType_t txAddr;
  static int8 Ctrlcase = 0;    //0����������Ļ��ʾ��1���Ƽ̵�����2���Ƶ��
  static uint8 MotorSpeed = 0;
  static uint8 NetWorkAllow = 0;
  /*������ô�����ƺ�������ڴ�����
  static uint8 Relay1_on = 0x02;  //����Ĭ�϶��ǹر�
  static uint8 Relay2_on = 0x10;
  */
  /*3.21 LCD ��ʾ*/
  static uint8 LCD_Page  =  0;
  static uint8 Relay1_on = 0;  //����Ĭ�϶��ǹر�
  static uint8 Relay2_on = 0;
  
  
  if ( keys & HAL_KEY_SW_1 )  //UP
  {
    switch(Ctrlcase)
    {
      default:    
      case 0:
      {
        if(LCD_Page < 3)       {LCD_Page++;}
        if(LCD_Page > 2)       {LCD_Page=0;}
        Smart_home_Display(LCD_Page);
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
        if(LCD_Page < 0)        {LCD_Page=2;}
        Smart_home_Display(LCD_Page);
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
  /*3.22 �����ڵ���ʱ�������⣬��������Ϊ ���upʱ�������Ļ��Ϊ��ֹ����
    ���Ʒ����Ϊ���Ƶ�����������ڻ���е�keyֵ�����  keys & HAL_KEY_SW_2 = 2
    ���������ʱ��ֹ����Ҫ�ȴ��о�  ����key_7*/
  {
     if(Ctrlcase < 3)   { Ctrlcase++;}
     if(Ctrlcase > 2)   { Ctrlcase=0;}
     Smart_home_Key_add(Ctrlcase);
  }
  
  if ( keys & HAL_KEY_SW_4 )  //LEFT
  {
     if(Ctrlcase > -1)   { Ctrlcase--;}
     if(Ctrlcase < 0)    { Ctrlcase=2;}
     Smart_home_Key_add(Ctrlcase);    
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
#if defined ( LCD_SUPPORTED )
       HalLcdWriteString( "Allow networking", HAL_LCD_LINE_4 );
#endif
     }
     else
     {
       NetWorkAllow = 0;
       NLME_PermitJoiningRequest(0x00); // ����������
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
    Ŀǰ�ܹ�8��case
   #define Smart_home_CLUSTERID_HUMITMSG            1  // ��ʪ��
   #define Smart_home_CLUSTERID_TEMPLIGHTMSG        2  // �¶ȹ���
   #define Smart_home_CLUSTERID_RFIDMSG             3  // ��Ƶ��
   #define Smart_home_CLUSTERID_GASFLAMEMSG         4  // ��������
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
    //3.21 �޸�Ϊָ�뷽����DeviceList��أ�
    case Smart_home_CLUSTERID_HUMITMSG:         // ��ʪ��
      DeviceList[Humit].deviceStatus = DEVICE_ONLINE;       //�յ���Ϣ����Ϊ����
      DeviceList[Humit].data[0] = pkt->cmd.Data[4]; // ʪ�� 
      DeviceList[Humit].data[1] = pkt->cmd.Data[5]; // �¶�
      break;
      
    case Smart_home_CLUSTERID_TEMPLIGHTMSG:     // �¶ȹ���
      DeviceList[TempLight].deviceStatus = DEVICE_ONLINE;   //�յ���Ϣ����Ϊ����
      DeviceList[TempLight].data[0] = pkt->cmd.Data[4]; // �¶�����
      DeviceList[TempLight].data[1] = pkt->cmd.Data[5]; // �¶�С��
      DeviceList[TempLight].data[2] = pkt->cmd.Data[6]; // ����
      DeviceList[TempLight].data[3] = pkt->cmd.Data[7]; // ����
      break;
      
    case Smart_home_CLUSTERID_RFIDMSG:          // ��Ƶ��
      DeviceList[RfID].deviceStatus = DEVICE_ONLINE;        //�յ���Ϣ����Ϊ����
      DeviceList[RfID].data[0] = pkt->cmd.Data[4]; // ��Ƶ������
      DeviceList[RfID].data[1] = pkt->cmd.Data[5]; // 4���ֽڵ�ID��
      DeviceList[RfID].data[2] = pkt->cmd.Data[6]; //
      DeviceList[RfID].data[3] = pkt->cmd.Data[7]; //
      DeviceList[RfID].data[4] = pkt->cmd.Data[8]; //  
      break;
      
    case Smart_home_CLUSTERID_GASFLAMEMSG:      // ��������
      DeviceList[gasFlame].deviceStatus = DEVICE_ONLINE;    //�յ���Ϣ����Ϊ����
      DeviceList[gasFlame].data[0] = pkt->cmd.Data[4]; // ��������汨����Ϣ
      break;
      
    case Smart_home_CLUSTERID_INFRAREDMSG:      // �������
      DeviceList[infrared].deviceStatus = DEVICE_ONLINE;    //�յ���Ϣ����Ϊ����
      DeviceList[infrared].data[0] = pkt->cmd.Data[4]; // ������� 
      break;

    case Smart_home_CLUSTERID_SOUNDVBMSG:       // ������
      DeviceList[soundVb].deviceStatus = DEVICE_ONLINE;     //�յ���Ϣ����Ϊ����
      DeviceList[soundVb].data[0] = pkt->cmd.Data[4]; // ��������Ϣ
      break;
      
    case Smart_home_CLUSTERID_MOTORSTATUSMSG:   // ֱ�����״̬��Ϣ
      DeviceList[motor].deviceStatus = DEVICE_ONLINE;       //�յ���Ϣ����Ϊ����
      // �������豸�������ַ�����ڷ��Ϳ�������
      Motor_addr.addrMode = (afAddrMode_t)Addr16Bit;
      Motor_addr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Motor_addr.endPoint = 1;  // Ŀ�Ľڵ�Ķ˿ں�
      //TransmitApp_DstMotorAddr->endPoint = TRANSMITAPP_ENDPOINT;
     
      DeviceList[motor].data[0] = pkt->cmd.Data[4]; // ���ת��
      DeviceList[motor].data[1] = pkt->cmd.Data[5]; // ���״̬
      break;
      
    case Smart_home_CLUSTERID_RELAYSTATUSMSG:   // �̵���״̬��Ϣ
      DeviceList[relay].deviceStatus = DEVICE_ONLINE;       //�յ���Ϣ����Ϊ����    
      
      // ����̵����豸�������ַ�����ڷ��Ϳ�������
      Relay_addr.addrMode = (afAddrMode_t)Addr16Bit;
      Relay_addr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
      
      Relay_addr.endPoint = 1; // Ŀ�Ľڵ�Ķ˿ں�
      
      DeviceList[relay].data[0] = pkt->cmd.Data[4]; 
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn     Smart_home_Device_check
 *
 * @brief   �豸����������������豸�ڿ�ʼ������������.
 *          Ŀǰ����Ϊ4��HeartBeat  (DEVICE_HEART_BEAT 3)
 *          3.21
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Device_check(void)
{
  static uint8 DeviceID;
  for(DeviceID=1;DeviceID<Smart_home_MAX_INCLUSTERS;DeviceID++)
  {
    if(DeviceList[DeviceID].deviceStatus != DEVICE_ONLINE)    //�豸����
    {
      DeviceCnt[DeviceID]++;
    }
    if(DeviceList[DeviceID].deviceStatus == DEVICE_ONLINE)    //�豸����
    {
      DeviceCnt[DeviceID] = 0;
      DeviceList[DeviceID].deviceStatus = 0;
    }
    if(DeviceCnt[DeviceID] > DEVICE_HEART_BEAT)
    {
      DeviceCnt[DeviceID] = DEVICE_HEART_BEAT;
      DeviceList[DeviceID].deviceStatus = DEVICE_OFFLINE;
    }
  }
  /*��ʪ�Ȼ��� �¶ȹ��ջ��� RFID ��Ϣ���� ������滺�� 
  ������� ���״̬ �̵���״̬ ������*/
}


/*********************************************************************
 * @fn      Smart_home_Relay_Ctl
 *
 * @brief   ���ͼ̵���������Ϣ
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Relay_Ctl(uint8 cmd)
{
  uint8 tmp;
  
  // ֻ���豸����ʱ, �����Ϳ�������
  if (DeviceList[relay].deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // ���͸��̵����Ŀ������� 
    Coordinator_Msg[4] = cmd;
    /********************************************
    *   ��Ļ��ʾ����*/
#if defined ( LCD_SUPPORTED )
    if(cmd == 0x02) {HalLcdWriteString( "K1:ON", HAL_LCD_LINE_2 );}
    if(cmd == 0x01) {HalLcdWriteString( "K1:OFF", HAL_LCD_LINE_2 );}
    if(cmd == 0x20) {HalLcdWriteString( "K2:ON", HAL_LCD_LINE_3 );}
    if(cmd == 0x10) {HalLcdWriteString( "K2:OFF", HAL_LCD_LINE_3 );}
#endif     
    /********************************************/
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
 * @brief   ���͵��������Ϣ
 *
 * @param   none
 *
 * @return  none
 */
static void Smart_home_Motor_Ctl(uint8 cmd,uint8 speed)
{
  uint8 tmp;
  
  // ֻ���豸����ʱ, �����Ϳ�������
  if (DeviceList[motor].deviceStatus != DEVICE_OFFLINE)
  {
    // put the sequence number in the message
    tmp = HI_UINT8( MotorTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[2] = tmp;
    tmp = LO_UINT8( RelayTransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    Coordinator_Msg[3] = tmp;
    
    // ���͸��̵����Ŀ������� 
    Coordinator_Msg[4] = speed;
    Coordinator_Msg[5] = cmd;
    /*********************************************
    *   ��Ļ��ʾ����
#if defined ( LCD_SUPPORTED )
    char* spe_c     = &speed;
    char* cmd_c     = &cmd;
    const char SpeStr[3]  = {"ON"};
    const char CmdStr[4] = {"OFF"};
    char* LCD_Display1;
    char* LCD_Display2;
    
    if(cmd == 0x02) {LCD_Display1 = strcat(K1,OnStr);}
    if(cmd == 0x01) {LCD_Display1 = strcat(K1,OffStr);}
    if(cmd == 0x10) {LCD_Display2 = strcat(K2,OnStr);}
    if(cmd == 0x20) {LCD_Display2 = strcat(K2,OffStr);}
    
    HalLcdWriteString( LCD_Display1, HAL_LCD_LINE_2 );
    HalLcdWriteString( LCD_Display2, HAL_LCD_LINE_3 );
#endif     
    ********************************************/
    
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
 * @brief   ��Ļ���Ժ��������������������ʾ.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void Smart_home_Display(uint8 page)
{
   
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
      (Smart_home_TxLen = HalUARTRead(SMART_HOME_PORT, Smart_home_TxBuf+1, SMART_HOME_TX_MAX)))
  {
    // Pre-pend sequence number to the Tx message.
    Smart_home_TxBuf[0] = ++Smart_home_TxSeq;
  }

  if (Smart_home_TxLen)
  {
    /*
    if (afStatus_SUCCESS != AF_DataRequest(&Smart_home_TxAddr,
                                           (endPointDesc_t *)&Smart_home_epDesc,
                                            Smart_home_CLUSTERID1,
                                            Smart_home_TxLen+1, Smart_home_TxBuf,
                                            &Smart_home_MsgID, 0, AF_DEFAULT_RADIUS))
    {
      osal_set_event(Smart_home_TaskID, Smart_home_SEND_EVT);
    }
    */
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
  /*
  if (afStatus_SUCCESS != AF_DataRequest(&Smart_home_RxAddr,
                                         (endPointDesc_t *)&Smart_home_epDesc,
                                          Smart_home_CLUSTERID2,
                                          SERIAL_APP_RSP_CNT, Smart_home_RspBuf,
                                         &Smart_home_MsgID, 0, AF_DEFAULT_RADIUS))
  {
    osal_set_event(Smart_home_TaskID, Smart_home_RESP_EVT);
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