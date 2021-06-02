//*****************************************************************************/
//
// �� �� ����hal_relay.c
//
// ��    ��: ������
// ����ʱ��: 2021.4.6
//
// ˵    �����̵����������򣬼̵���ģ��������IO�ܽ�������P1_2(�̵���K1)��P1_3(�̵���K2)��
//           �ܽ��øߵ�ƽ���̵���ģ�鿪�ش򿪣��෴���ܽŵ͵�ƽ���̵���ģ��رգ�
//                        
//
//*****************************************************************************/

#include "iocc2530.h"
#include "hal_types.h"
#include "hal_defs.h"

//*****************************************************************************/
#define  RELAY_MODULE_1_PIN_SET  P1DIR |= BV(2) 
#define  RELAY_MODULE_2_PIN_SET  P1DIR |= BV(3)
#define  RELAY_MODULE_1_TOGGLE   P1_2  ^= 1
#define  RELAY_MODULE_2_TOGGLE   P1_3  ^= 1

#define  RELAY_MODULE_1_PIN      P1_2
#define  RELAY_MODULE_2_PIN      P1_3

#define  RELAY_SWITCH_ON         1
#define  RELAY_SWITCH_OFF        0

//*****************************************************************************/
//���ر���
static uint8 relayStatus; // �̵�����ǰ״̬

//*****************************************************************************/
//��������HalRelayCtl
//��  �ܣ��̵�������ģʽ����
//��  ����uint8 mode���̵���ģʽ�л���
//              0x11��K1�ر�(1), K2�ر�(1)
//              0x12��K1��(2), K2�ر�(1)
//              0x21��K1�ر�(1), K2��(2)
//              0x22��K1��(2), K2��(2)
//              0x00�����κβ���
//����ֵ����
//*****************************************************************************/
void HalRelayCtl(uint8 mode)
{
    RELAY_MODULE_1_PIN_SET;
    RELAY_MODULE_2_PIN_SET;
  
    // �̵���K1״̬�л�
    switch(mode & 0x0F)
    {
    case 0x02: // ��
      {
        RELAY_MODULE_1_PIN =  RELAY_SWITCH_ON;
        relayStatus = (relayStatus&0xF0) | 0x02;
        break;
      }
    case 0x01: // �ر�
      {
        RELAY_MODULE_1_PIN =  RELAY_SWITCH_OFF;
        relayStatus = (relayStatus&0xF0) | 0x01;
        break;
      }
    default:
        break;
    }
    
    // �̵���K2״̬�л�
    switch(mode & 0xF0)
    {
    case 0x10: // �ر�
      {
        RELAY_MODULE_2_PIN =  RELAY_SWITCH_OFF;
        relayStatus = (relayStatus&0x0F) | 0x10;
        break;
      }
    case 0x20: // ��
      {
        RELAY_MODULE_2_PIN =  RELAY_SWITCH_ON;
        relayStatus = (relayStatus&0x0F) | 0x20;
        break;
      }
    default:
        break;
    }
}

//*****************************************************************************/
//��������HalRelayStatus
//��  �ܣ����洢�ļ̵�����״̬ͨ����������
//��  ������
//����ֵ�����ؼ̵������õ�״̬
//*****************************************************************************/
uint8 HalRelayStatus()
{
    return relayStatus;
}