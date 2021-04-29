/**************************************************************************************************
  �ļ�����Smart_home.h
  �� �ߣ� ������
  �� �ܣ� ���������ط����ף�·�������ն˽ڵ㶼Ҫ��ѭ����غ��볣����
          ����������������־
  ������־(2021)
  3.13
  + ���� �豸����״̬��غ�
         ����������غ�
         �豸���ݳ�����غ�  �����趨���ƣ����峤����Ҫ���ģ�
  3.14
  + �޸� �豸��ش�ת��Ϊ���������������
  + ��� �豸����������ݽṹ

  3.17
  + ���� �¼���غ�
         �豸��ʼ������ڼ��ʱ���
  3.21
  + �޸� �豸������ز��� �����������Ϊ0���豸 ��
**************************************************************************************************/

#ifndef Smart_home_H
#define Smart_home_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

#define Smart_home_ENDPOINT           1

#define Smart_home_PROFID             0x0F05
#define Smart_home_DEVICEID           0x0001
#define Smart_home_DEVICE_VERSION     0
#define Smart_home_FLAGS              0

/*���ݷ�����غ�*/
/*�豸������*/
//���벿��   3.14
//3.21 DeviceList����
#define Smart_home_MAX_INCLUSTERS                9
#define Smart_home_CLUSTERID_COORDINATOR         0  // ���ذ�
#define Smart_home_CLUSTERID_HUMITMSG            1  // ��ʪ��
#define Smart_home_CLUSTERID_TEMPLIGHTMSG        2  // �¶ȹ���
#define Smart_home_CLUSTERID_RFIDMSG             3  // ��Ƶ��
#define Smart_home_CLUSTERID_GASFLAMEMSG         4  // �������
#define Smart_home_CLUSTERID_INFRAREDMSG         5  // �������
#define Smart_home_CLUSTERID_SOUNDVBMSG          6  // ������
#define Smart_home_CLUSTERID_MOTORSTATUSMSG      7  // ֱ�����״̬��Ϣ
#define Smart_home_CLUSTERID_RELAYSTATUSMSG      8  // �̵���״̬��Ϣ

//�������
#define Smart_home_MAX_OUTCLUSTERS               3  
#define Smart_home_CLUSTERID_TEXT                12  //������
#define Smart_home_CLUSTERID_MOTORCTRL           11  // ֱ�����������Ϣ
#define Smart_home_CLUSTERID_RELAYCTRL           10  // �̵���������Ϣ  
 
/*���ݷ��ͳ���*/
/*��δ��ȫȷ��*/
#define MSG_MAX_LEN                   102 //������ݳ���
#define HUMITMSG_LEN                  8  // ��ʪ��
#define TEMPLIGHTMSG_LEN              8  // �¶ȹ���
#define RFIDMSG_LEN                   9  // ��Ƶ��
#define GASFLAMEMSG_LEN               5  // �������
#define INFRAREDMSG_LEN               5  // �������
#define SOUNDVBMSG_LEN                5  // ������
#define MOTORSTATUSMSG_LEN            6  // ֱ�����״̬��Ϣ
#define RELAYSTATUSMSG_LEN            5  // �̵���״̬��Ϣ
 
  
//3.17 �¼����
#define SMART_HOME_SEND_MSG_EVT        0x0001  // ���ڷ����¼�
#define SMART_HOME_RCVTIMER_EVT        0x0002  // û��ʹ��
#define SMART_HOME_SEND_ERR_EVT        0x0004  // ����ʧ�ܴ����¼�
#define SMART_HOME_MATCHRSP_EVT        0x0008  // ������ƥ���¼�
#define SMART_HOME_DISPLAY_EVT         0x0010  // LCD��ʾ�¼�
#define SMART_HOME_BINDRSP_EVT         0x0020  // ���¼���
#define SMART_HOME_HALCHECK_EVT        0x0040  // ��������豸�Ķ�ʱ��ѯ�¼�
#define SMART_HOME_DEVICE_CHECK_EVT    0x0080  // �豸���߼���¼�(Э����)

// OTA Flow Control Delays
#define Smart_home_ACK_DELAY          1
#define Smart_home_NAK_DELAY          16

// OTA Flow Control Status
#define OTA_SUCCESS                  ZSuccess
#define OTA_DUP_MSG                 (ZSuccess+1)
#define OTA_SER_BUSY                (ZSuccess+2)
  
/*********************************************************************
 * MACROS
 */
/*3.14 �豸����*/
typedef struct DeviceInfo
{
  uint8 deviceid;
  uint8 deviceStatus;
  uint8 data[5];
} DeviceInfo;  

/*�豸����״̬*/
#define DEVICE_ONLINE                 1  //����
#define DEVICE_OFFLINE                2  //�豸������

/*ÿ�μ���豸����ʱ*/
#define SMART_HOME_DEVICE_CHECK_DELAY         5000
#define SMART_HOME_DEVICE_DISPLAY_DELAY       10000

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern byte Smart_home_TaskID;
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Smart_home Application
 */
extern void Smart_home_Init( byte task_id );

/*
 * Task Event Processor for the Smart_home Application
 */
extern UINT16 Smart_home_ProcessEvent( byte task_id, UINT16 events );

extern void Smart_home_ChangeState( void );
extern void Smart_home_SetSendEvt( void );
/*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* Smart_home_H */
