/**************************************************************************************************
    Filename:       TransmitApp.h
    Revised:        $Date: 2013-10-2 $
    Revision:       $ $

    Description:    This file contains the Transmit Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef TRANSMITAPP_H
#define TRANSMITAPP_H

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

// These constants are only for example and should be changed to the
// device's needs
#define TRANSMITAPP_ENDPOINT           1

#define TRANSMITAPP_PROFID             0x0F05
#define TRANSMITAPP_DEVICEID           0x0001
#define TRANSMITAPP_DEVICE_VERSION     0
#define TRANSMITAPP_FLAGS              0
  
#define TRANSMITAPP_MAX_INCLUSTERS           8
#define TRANSMITAPP_CLUSTERID_HUMITMSG       1  // ��ʪ��
#define TRANSMITAPP_CLUSTERID_TEMPLIGHTMSG   2  // �¶ȹ���
#define TRANSMITAPP_CLUSTERID_RFIDMSG        3  // ��Ƶ��
#define TRANSMITAPP_CLUSTERID_GASFLAMEMSG    4  // �������
#define TRANSMITAPP_CLUSTERID_INFRAREDMSG    5  // �������
#define TRANSMITAPP_CLUSTERID_SOUNDVBMSG     6  // ������
#define TRANSMITAPP_CLUSTERID_MOTORSTATUSMSG 7  // ֱ�����״̬��Ϣ
#define TRANSMITAPP_CLUSTERID_RELAYSTATUSMSG 8  // �̵���״̬��Ϣ

  
#define TRANSMITAPP_MAX_OUTCLUSTERS        3
#define TRANSMITAPP_CLUSTERID_TESTMSG      9    // ����Ԥ��
#define TRANSMITAPP_CLUSTERID_RELAYCTLMSG  10   // �̵�������
#define TRANSMITAPP_CLUSTERID_MOTORCTLMSG  11   // ֱ���������
  
// Application Events (OSAL) - These are bit weighted definitions.
#define TRANSMITAPP_SEND_MSG_EVT        0x0001  // ���ڷ����¼�
#define TRANSMITAPP_RCVTIMER_EVT        0x0002  // û��ʹ��
#define TRANSMITAPP_SEND_ERR_EVT        0x0004  // ����ʧ�ܴ����¼�����ʵ��û��ʹ��
#define TRANSMITAPP_MATCHRSP_EVT        0x0008  // ������ƥ���¼�
#define TRANSMITAPP_DISPLAY_EVT         0x0010  // LCD��ʾ�¼�
#define TRANSMITAPP_BINDRSP_EVT         0x0020  // ���¼�����ʵ��û��ʹ��
#define TRANSMITAPP_HALCHECK_EVT        0x0040  // ��������豸�Ķ�ʱ��ѯ�¼�
#define TRANSMITAPP_DEVICE_CHECK_EVT    0x0080  // �豸���߼���¼�(Э����)
  
#define TRANSMITAPP_MOTOR_DATA_LEN        6     // ��������豸�����ݳ���
#define TRANSMITAPP_RELAY_DATA_LEN        5     // �̵��������豸�����ݳ���
#define TRANSMITAPP_TEMPLIGHT_DATA_LEN    8     // �¶Ⱥ͹��նȲ����豸�����ݳ���
#define TRANSMITAPP_DHT11_DATA_LEN        8     // �¶Ⱥ�ʪ�Ȳ����豸�����ݳ���  
#define TRANSMITAPP_RC522_DATA_LEN        9     // RFID�����������Ϣ�����ݳ���
#define TRANSMITAPP_GASFLAME_DATA_LEN     5     // ������������豸�����ݳ���
#define TRANSMITAPP_D203S_DATA_LEN        5     // �������͵����豸�����ݳ��� 
#define TRANSMITAPP_SOUNDVB_DATA_LEN      5     // �����������豸�����ݳ��� 
  
/*********************************************************************
 * MACROS
 */

// Define to enable fragmentation transmit test
//#define TRANSMITAPP_FRAGMENTED

typedef struct deviceInfo
{
  //uint8 deviceID;
  uint8 deviceNWKStatus;
  uint8 data[5];
} deviceInfo_t;

// �豸���ߡ����߱�־
#define  DEVICE_NWK_ONLINE   1
#define  DEVICE_NWK_OFFLINE  2
  
#define ZB_BINDING_ADDR               INVALID_NODE_ADDR
#define ZB_BROADCAST_ADDR             0xFFFF
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Transmit Application
 */
extern void TransmitApp_Init( byte task_id );

/*
 * Task Event Processor for the Transmit Application
 */
extern UINT16 TransmitApp_ProcessEvent( byte task_id, UINT16 events );

extern void TransmitApp_ChangeState( void );
extern void TransmitApp_SetSendEvt( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TRANSMITAPP_H */
