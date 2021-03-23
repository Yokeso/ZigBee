//*****************************************************************************/
//
// �� �� ����hal_soundvibration.c
//
// ��    �ߣ�yizedxl
//
// ����ʱ�䣺2013.10
//
// �޸�ʱ�䣺2013.11.26
//
// IAR �汾��IAR for 8051 V8.10.1
//
// ����ƽ̨��Sensor Battery MotherBoard V2.0
//
// ˵    ���������񶯼��ģ�飬��ģ�����ӵ���P1_1������ģ�����ӵ���P1_2��
//           ����ģ�������IO��⵽�͵�ƽʱ��ʾ�������������񶯳��֡�
//
//*****************************************************************************/

#include <iocc2530.h>
#include "hal_defs.h"
#include "hal_types.h"

//*****************************************************************************/
//
//��������HalSoundVbInit
//
//��  �ܣ�������ģ��IO��ʼ��
//
//��  ������
//
//����ֵ����
//
//*****************************************************************************/
void HalSoundVbInit()
{
    P1DIR &= ~(BV(1) + BV(2));
}

//*****************************************************************************/
//
//��������HalSoundCheck
//
//��  �ܣ������������ģ���IO��ƽ
//
//��  ������
//
//����ֵ��0-û��������
//        1-��������
//
//*****************************************************************************/
uint8 HalSoundCheck()
{
    if (P1_2)
    {
        return 0;
    }
    else 
    {
        return 1;
    }
}

//*****************************************************************************/
//
//��������HalVibrationCheck
//
//��  �ܣ����������ģ���IO��ƽ
//
//��  ������
//
//����ֵ��0-û����
//        1-����
//
//*****************************************************************************/
uint8 HalVibrationCheck()
{
    if (P1_1)
    {
        return 0;
    }
    else 
    {
        return 1;
    }
}