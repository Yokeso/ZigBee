/*******************************************************************************
 * ��   ��   ����hal_DHT11.c
 *
 * ��        �ߣ�yizedxl
 *
 * ��  �� �� �ڣ�2013.4.30
 *
 * ���һ���޸ģ�2013.9.13
 *
 * ˵        ����1��ʵ��DHT11��ʪ�ȴ����������ݶ�ȡ,����������Ǩ��ZstackЭ��ջ;
 *               2��������Գ�������(����ȫ0)�����������DHT11_ReadData(void)����1����;
 *               3�������ֲ᣺ÿ�ζ�������ʪ����ֵ����һ�β����Ľ��������ȡʵʱ����,
 *                            ��������ȡ���Σ���������������ζ�ȡ��������ÿ�ζ�ȡ��
 *                            �����������5�뼴�ɻ��׼ȷ�����ݡ�
 *               4��ʪ�ȷ�Χ��20%~90%; �¶ȷ�Χ��0~50C;
 *               5���������ɼ���������ֻ���������֣���ʱû��С�����֣�
 *               6������һ��5���ֽڣ���һ��ʪ���������ڶ���ʪ��С����
 *                                   �������¶����������ģ��¶�С����
 *                                   ���壺У��(У��ֵΪǰ�ĸ��ֽڵĺ�)��
 *
 ********************************************************************************/

#include <ioCC2530.h>
#include "hal_types.h"
#include "osal.h"
#include "hal_mcu.h"
#include "hal_defs.h"

//*****************************************************************************/
//���ź궨��
#define DHT11_SET_PIN_WRITE    P1DIR |= BV(1)
#define DHT11_SET_PIN_READ     P1DIR &= ~BV(1)
#define DHT11_WRITE_H          P1_1   = 1
#define DHT11_WRITE_L          P1_1   = 0
#define DHT11_READ_DATA        P1_1

//*****************************************************************************/
// �궨��
// ����ȴ�ʱ��
#define TIMEOUT_LIMIT 100

//*****************************************************************************/
//��������DHT11_DelayNus
//��  �ܣ���ʱn΢��, (�������ԣ��÷���������ȷ)
//��  ����n
//����ֵ����
//*****************************************************************************/
void DHT11_DelayNus(uint16 n)
{
    while(n--)
    {
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
        asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
        asm("NOP");asm("NOP");
    }
}

//*****************************************************************************/
//��������DHT11_Start
//��  �ܣ�����DHT11
//��  ������
//����ֵ����
//*****************************************************************************/
uint8 DHT11_Start(void)
{
    uint8 i;
    
    DHT11_SET_PIN_WRITE;
    DHT11_WRITE_L;
    DHT11_DelayNus(22000); // �͵�ƽ����ʱ�䲻��С��18ms
    DHT11_WRITE_H;
    DHT11_SET_PIN_READ;
    
    //DHT11��� 80΢��ĵ͵�ƽ��ΪӦ���ź�
    for (i = 0; i < TIMEOUT_LIMIT; i++)
    {
        if (!DHT11_READ_DATA)
        {
            break;
        }
        DHT11_DelayNus(1);
    }
    
    if (i == TIMEOUT_LIMIT)
    {
        return 1;
    }
    
    for (i = 0; i < TIMEOUT_LIMIT; i++)
    {
        if (DHT11_READ_DATA)
        {
            break;
        }
        DHT11_DelayNus(1);
    }
    
    if (i == TIMEOUT_LIMIT)
    {
        return 1;
    }
    // ��������� 80 ΢��ĸߵ�ƽ֪ͨ����׼����������
    for (i = 0; i < TIMEOUT_LIMIT; i++)
    {
        if (!DHT11_READ_DATA)
        {
            break;
        }
        DHT11_DelayNus(1);
    }
    
    if (i == TIMEOUT_LIMIT)
    {
        return 1;
    }
    return 0;
}

//*****************************************************************************/
//��������DHT11_ReadData
//��  �ܣ���DHT11��ȡ����
//��  ������
//����ֵ����ȡ������
//˵ �� ��λ���ݡ�0���ĸ�ʽΪ: 50΢��ĵ͵�ƽ��26-28΢��ĸߵ�ƽ��
//        λ���ݡ�1���ĸ�ʽΪ: 50΢��ĵ͵�ƽ��70΢��ĸߵ�ƽ	
//*****************************************************************************/
uint8 DHT11_ReadData(void)
{
    uint8 i, j;
    uint8 data = 0;
    halIntState_t intState;
    
    DHT11_SET_PIN_READ;
    for (i = 0; i < 8; i++)
    {
        HAL_ENTER_CRITICAL_SECTION(intState); // ���ж�
        for (j = 0; j < TIMEOUT_LIMIT; j++)
        {
            if (DHT11_READ_DATA)
            {
                break;
            }
            DHT11_DelayNus(1);
        }        
    
        if (j == TIMEOUT_LIMIT) // ��ʱ���
        {
            return 0;
        }
                   
        for (j = 0; j < TIMEOUT_LIMIT; j++)
        {
            if (!DHT11_READ_DATA)
            {
                break;
            }
            DHT11_DelayNus(1);
        }
        HAL_EXIT_CRITICAL_SECTION(intState); // ���ж�  
    
        if (j == TIMEOUT_LIMIT) // ��ʱ���
        {
            return 0;
        }
        
        data <<= 1;
        
        // ������Գ�������(�����������ȫ0)��������������1����;
        // Ŀǰ���Է�Χ5~10������������󣬵����˴�����1����
        if (j > 10) 
        {
            data |=  0x01;
        }
        else 
        {
            data |= 0x00;
        }
    }
    return data;
}

//*****************************************************************************/
//��������DHT11_Convert
//��  �ܣ�һ�����������ݲɼ�����
//��  ����data����д�������
//����ֵ��error��1-����0-������ȡ��
//*****************************************************************************/
uint8 HalDht11_Convert(uint8 *data)
{
    uint8 error, dataCheck;
    uint8 datatmp[5];
    
    error = 0;
    
    error = DHT11_Start();
    if (error)
    {
        return error;
    }
      
    datatmp[0] = DHT11_ReadData();
    datatmp[1] = DHT11_ReadData();
    datatmp[2] = DHT11_ReadData();
    datatmp[3] = DHT11_ReadData();
    datatmp[4] = DHT11_ReadData();
    
    // ����У��ֵ
    dataCheck = (datatmp[0] + datatmp[1] + datatmp[2] + datatmp[3]);
    
    // ��֤У��ֵ
    if (datatmp[4] == dataCheck)
    {
        error = 0;
        osal_memcpy(data, datatmp, 4);
    }
    else 
    {
        error = 1;
    }
    
    DHT11_SET_PIN_WRITE;
    DHT11_WRITE_H;
    return error;
}
