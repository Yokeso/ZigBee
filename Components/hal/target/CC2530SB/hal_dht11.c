/*****************************************************************************
*
* �� �� ����hal_dht11.c

* ��    ��: ������

* ����ʱ��: 2021.3.2

* IAR �汾: IAR for 8051 V8.10.1

* ����ƽ̨: Sensor MotherBoard V2.3

* ˵    ��: 1. ʵ��DHT11��ʪ�ȴ����������ݶ�ȡ, ����������Ǩ��ZstackЭ��ջ.
*           2. ������Գ�������(��ȫ0), ���������DHT11_ReadData(void)����1����.
*           3. �����ֲ�, ÿ�ζ�������ʪ����ֵ����һ�β����Ľ��, ����ȡʵʱ����,
*              ��������ȡ����, ��������������ζ�ȡ������, ÿ�ζ�ȡ���������
*              ����5�뼴�ɻ��׼ȷ������.
*           4. ʪ�ȷ�Χ: 20%~90%; �¶ȷ�Χ: 0~50C.
*           5. �������ɼ���������ֻ����������, ��ʱû��С������(������չ).
*           6. ����һ��5���ֽ�: ��һ��Ϊʪ������, �ڶ���Ϊʪ��С��, ������Ϊ�¶�
*              ����, ���ĸ�Ϊ�¶�С��,�����У���ֽ�(У��ֵΪǰ�ĸ��ֽڵĺ�).
*                               
*****************************************************************************/

// ͷ�ļ�
#include <ioCC2530.h>
#include "hal_mcu.h"

// ���ź궨��, ����/��������MCU����
#define SET_DHT11_PIN_OUTPUT    (P1DIR |= 0x02)
#define SET_DHT11_PIN_HIGH      (P1_1   = 1)
#define SET_DHT11_PIN_LOW       (P1_1   = 0)
#define SET_DHT11_PIN_INPUT     (P1DIR &= ~0x02)
#define GET_DHT11_PIN_DATA      (P1_1) 

// �����궨��
#define TIMEOUT_LIMIT 1000       // ����ȴ�ʱ��1000ms

/*****************************************************************************
 * @fn          DHT11_Delay100us
 *
 * @brief       ��������¡�ʱ��Ƶ��Ϊ32MHzʱ��ʱ100΢������(����ȷ).
 *
 * @param       none
 *
 * @return      none
 */
void DHT11_Delay100us(void)
{
    unsigned char cnt = 85;
    while (cnt--)
    {
        asm("NOP");  
        asm("NOP");
        asm("NOP");
    }
}

/*****************************************************************************
 * @fn          DHT11_Delay10us
 *
 * @brief       ��������¡�ʱ��Ƶ��Ϊ32MHzʱ��ʱ10΢������(����ȷ).
 *
 * @param       none
 *
 * @return      none
 */
void DHT11_Delay10us(void)
{
    unsigned char cnt = 9;
    while (cnt--)
    {
        asm("NOP");  
        asm("NOP");
        asm("NOP");
    }
}

/*****************************************************************************
 * @fn          DHT11_Start
 *
 * @brief       ����DHT11
 *
 * @param       none
 *
 * @return      none
 */
void DHT11_Start(void)
{
    unsigned char timeCnt;
    
    SET_DHT11_PIN_OUTPUT;   // ��������Ϊ���
    SET_DHT11_PIN_LOW;      // ����͵�ƽ, ��������
    // �͵�ƽ����ʱ�䲻��С��18ms, ����DHT11�޷�����
    for(timeCnt=0; timeCnt<200; timeCnt++)
    {
        DHT11_Delay100us();
    }
    
    SET_DHT11_PIN_HIGH;     // ����ߵ�ƽ
    SET_DHT11_PIN_INPUT;    // ����Ϊ��������(������������, ���Ա���Ϊ�ߵ�ƽ)    

    // �ߵ�ƽ����20~40us
    for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
    {
        if (!GET_DHT11_PIN_DATA)
        {
            break;  // �͵�ƽ�˳�ѭ��
        }
        DHT11_Delay10us();
    }
    // ��ʱ����
    if (timeCnt == TIMEOUT_LIMIT)
    {
        return;
    }
    // DHT11���80΢��ĵ͵�ƽ��ΪӦ���ź�
    for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
    {
        if (GET_DHT11_PIN_DATA)
        {
            break;  // �ߵ�ƽ�˳�ѭ��
        }
        DHT11_Delay10us();
    }
    // ��ʱ����
    if (timeCnt == TIMEOUT_LIMIT)
    {
        return;
    }
    // DHT11���������80΢��ĸߵ�ƽ֪ͨ����׼����������
    for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
    {
        if (!GET_DHT11_PIN_DATA)    // �͵�ƽ�˳�ѭ��
        {
            break;
        }
        DHT11_Delay10us();
    }
    // ��ʱ����
    if (timeCnt == TIMEOUT_LIMIT)
    {
        return;
    }
}

/*****************************************************************************
 * @fn          DHT11_ReadData
 *
 * @brief       ��DHT11��ȡһ���ֽ�����
 *
 * @param       none
 *
 * @return      ��ȡ������, ����0���ʾ��ʱ����.
 *              λ����"0"�ĸ�ʽΪ50΢��ĵ͵�ƽ��26-28΢��ĸߵ�ƽ��
 *              λ����"1"�ĸ�ʽΪ50΢��ĵ͵�ƽ��70΢��ĸߵ�ƽ.	
 * 
 */
unsigned char DHT11_ReadData(void)
{
    unsigned char bitCnt, timeCnt;
    unsigned char byteVal = 0;
    halIntState_t intState;
    
    for (bitCnt = 0; bitCnt < 8; bitCnt++)
    {
        // DHT11���50us�͵�ƽ, ������������λ�ѿ�ʼ����
        for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
        {
            if (GET_DHT11_PIN_DATA)    // �ߵ�ƽ�˳�ѭ��
            {
                break;
            }
            DHT11_Delay10us();
        }        
        if (timeCnt == TIMEOUT_LIMIT)   // ��ʱ����, ����0
        {
            return 0;
        }
        
        HAL_ENTER_CRITICAL_SECTION(intState);
        // DHT11����ߵ�ƽ��͵�ƽ                   
        for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
        {
            if (!GET_DHT11_PIN_DATA)    // �͵�ƽ�˳�ѭ��
            {
                break;
            }
            DHT11_Delay10us();
        }
        if (timeCnt == TIMEOUT_LIMIT)   // ��ʱ����, ����0
        {
            return 0;
        }
        HAL_EXIT_CRITICAL_SECTION(intState);
        
        // �洢����λ, DHT11���ȷ���λ
        byteVal <<= 1;
        // ���ݸߵ�ƽ�Ŀ���ж���0����1
        if (timeCnt > 4) 
        {
            byteVal |=  0x01;
        }
        else 
        {
            byteVal |= 0x00;
        }
    }
    return byteVal;
}

/*****************************************************************************
 * @fn          HalDht11_Convert
 *
 * @brief       ����DHT11, �����һ��ת��.
 *
 * @param       none
 *
 * @return      ת��ֵ����conversionVal������, ��������򲻸�������. 
 *              ����ʱ, �ɰ����¼������м��:
 *              1. �����ʱ�������Ƿ�׼ȷ;
 *              2. ���DHT11�Ƿ�����;
 *              3. ����б�"0"��"1"����ֵ�Ƿ����;
 *              4. ���У�����Ƿ���ȷ.
 * 
 */
void HalDht11_Convert(unsigned char conversionVal[4])
{
    unsigned char checkSum;
    unsigned char tempData[5];
    
    DHT11_Start();      // ����DHT11
    
    // ��ȡ����
    tempData[0] = DHT11_ReadData();
    tempData[1] = DHT11_ReadData();
    tempData[2] = DHT11_ReadData();
    tempData[3] = DHT11_ReadData();
    tempData[4] = DHT11_ReadData();
    
    // ����У��ֵ
    checkSum = (tempData[0] + tempData[1] + tempData[2] + tempData[3]);
    
    // ��֤У��ֵ, �����򲻸�������
    if (tempData[4] == checkSum)
    {
        conversionVal[0] = tempData[0];
        conversionVal[1] = tempData[1];
        conversionVal[2] = tempData[2];
        conversionVal[3] = tempData[3];
    }
    
    SET_DHT11_PIN_OUTPUT;   // ����DHT11�Ŀ�������Ϊ�������
    SET_DHT11_PIN_HIGH;     // �øߵ�ƽ, ʹDHT11���ڵ͹��Ĵ���ģʽ
}
