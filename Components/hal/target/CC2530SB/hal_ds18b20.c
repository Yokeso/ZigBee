/*****************************************************************************
*
* �� �� ����hal_ds18b20.c

* ��    ��: �Ͼ���已��е��ӿƼ����޹�˾

* ����ʱ��: 2019.04.01

* �޸�ʱ��: 2019.04.21

* IAR �汾: IAR for 8051 V8.10.1

* ����ƽ̨: Sensor MotherBoard V2.3

* ˵    ��: 1. DS18B20��������ͨ��, CC2530���ͨ��P1_1�ܽŽ���ͨ��.
*           2. DS18B20����״̬ʱ, ����Ϊ�ߵ�ƽ.
*           3. ��������480us���Ը�λDS18B20.
*              ��������ȡ����, ��������������ζ�ȡ������, ÿ�ζ�ȡ���������
*              ����5�뼴�ɻ��׼ȷ������.
*           4. �������¶���ֵ12λ����: ��4λ��С��, ǰ���5λ��, �м��7λ��
*              ��������.
*                               
*****************************************************************************/

// ͷ�ļ�
#include <ioCC2530.h>
#include "hal_types.h"
#include "hal_mcu.h"

// ���ź궨��, ����/��������MCU����
#define SET_DS18B20_PIN_HIGH    (P1_1 = 1)
#define SET_DS18B20_PIN_LOW     (P1_1 = 0)
#define SET_DS18B20_PIN_INPUT   (P1DIR &= ~0x02)
#define SET_DS18B20_PIN_OUTPUT  (P1DIR |= 0x02)
#define GET_DS18B20_PIN_DATA    (P1_1)


// DS18B20�������
#define CONVERT_T_CMD           0x44
#define SKIP_ROM_CMD            0xCC
#define READ_SCRATCHPAD_CMD     0xBE
#define WRITE_SCRATCHPAD_CMD    0x4E

/*****************************************************************************
 * @fn          HalDs18b20DelayNus
 *
 * @brief       ��������¡�ʱ��Ƶ��Ϊ32MHzʱʵ��N��΢�����ʱ(����ȷ).
 *
 * @param       n--��ʱ����
 *
 * @return      none
 */
void HalDs18b20DelayNus(uint16 cnt)
{
    while (cnt--)
    {
        asm("NOP");  
        asm("NOP");
        asm("NOP");
    }
}

/*****************************************************************************
 * @fn          DS18B20_Init
 *
 * @brief       ��DS18B20���и�λ����.
 *
 * @param       none
 *
 * @return      1--�ɹ�, 0--ʧ��
 */
uint8 DS18B20_Init(void)
{
    uint8 opSts = 0;
    halIntState_t intState;
    
    SET_DS18B20_PIN_OUTPUT;
    SET_DS18B20_PIN_LOW;
    HalDs18b20DelayNus(750);    // 480us���ϵĵ͵�ƽ��λ����
    
    SET_DS18B20_PIN_HIGH; 
    SET_DS18B20_PIN_INPUT;      // �ߵ�ƽ��������, �ȴ�DS18B20��������
    
    HAL_ENTER_CRITICAL_SECTION(intState);
    HalDs18b20DelayNus(80);     // �ȴ�80us
    if(GET_DS18B20_PIN_DATA)    // DS18B20����60~240us�ĵ͵�ƽ��������  
    {
        opSts = 0;  // ��ʼ��ʧ��, ������û���豸
    }
    else
    {
        opSts = 1;  // ��ʼ���ɹ�, ���������豸��������
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
    
    // �ͷ�����   
    SET_DS18B20_PIN_OUTPUT;
    SET_DS18B20_PIN_HIGH;
    HalDs18b20DelayNus(500);
    
    return(opSts);
}

/*****************************************************************************
 * @fn          DS18B20_WriteCMD
 *
 * @brief       ��DS18B20д��һ���ֽڵ�����, ��λ�ȷ���.
 *
 * @param       wdata--д�������
 *
 * @return      none
 */
void DS18B20_WriteCMD(uint8 wdata)
{
    uint8 i;
    halIntState_t intState;
    
    for (i = 0; i < 8; i++)
    {
        HAL_ENTER_CRITICAL_SECTION(intState);
        SET_DS18B20_PIN_LOW;
        HalDs18b20DelayNus(15);            
        
        if(wdata & 0x01)    
        {
            SET_DS18B20_PIN_HIGH;
        }
        else
        {
            SET_DS18B20_PIN_LOW;
        }
        
        wdata >>= 1;
        HalDs18b20DelayNus(30); // DS18B20����ʱ��15~45us
        HAL_EXIT_CRITICAL_SECTION(intState);
        
        // �ͷ�����
        SET_DS18B20_PIN_HIGH;
        HalDs18b20DelayNus(10); // ���߻ָ�ʱ�� 
    }
}

/*****************************************************************************
 * @fn          DS18B20_ReadByte
 *
 * @brief       ��DS18B20��ȡһ���ֽڵ�����.
 *
 * @param       none
 *
 * @return      ������һ���ֽ�����
 */
uint8 DS18B20_ReadByte(void)
{
    uint8 bitCnt;
    halIntState_t intState;
    uint8 tmpData = 0;
    
    for (bitCnt = 0; bitCnt < 8; bitCnt++)
    {
        HAL_ENTER_CRITICAL_SECTION(intState);
        tmpData >>= 1;
        
        // ׼��������
        SET_DS18B20_PIN_OUTPUT;
        SET_DS18B20_PIN_LOW;
        HalDs18b20DelayNus(2);  // ��ʱ2us
        // �ȴ��豸��Ӧ
        SET_DS18B20_PIN_HIGH;
        SET_DS18B20_PIN_INPUT;
        HalDs18b20DelayNus(15); // ������ʱ15us���������
        if (GET_DS18B20_PIN_DATA)   
        {
            tmpData |= 0x80;
        }
        HalDs18b20DelayNus(30); // �ȴ��ӻ��������ݽ���
        HAL_EXIT_CRITICAL_SECTION(intState);
        
        // ��������Ϊ���, Ȼ���ͷ�����
        SET_DS18B20_PIN_OUTPUT;
        SET_DS18B20_PIN_HIGH;
        HalDs18b20DelayNus(10); // ���߻ָ�ʱ��           
    }

    return  tmpData;
}

/*****************************************************************************
 * @fn          DS18B20_ReadTemp
 *
 * @brief       ��DS18B20��ScratchPad��ȡ�¶�ת�����.
 *
 * @param       none
 *
 * @return      ��ȡ���¶���ֵ
 */
uint16 DS18B20_ReadTemp(void)
{
    uint8   tmpLowByte;
    uint16  tmpData;
    
    tmpLowByte  = DS18B20_ReadByte();           // �����ֽ�
    tmpData     = DS18B20_ReadByte() & 0x00FF;  // �����ֽ�
    tmpData     = ((tmpData << 8) | tmpLowByte);
    
    return  tmpData;
}

/*****************************************************************************
 * @fn          DS18B20_ConvertTemp
 *
 * @brief       ����DS18B20���һ���¶�ת��.
 *
 * @param       none
 *
 * @return      �������¶���ֵ, ��4λ��С��, ǰ���5λ��, �м��7λ����������.
 *              ����0��ʾ����, �������ʵ����������޸ĺ��ʵ���ֵ.
 */
uint16 DS18B20_ConvertTemp(void)
{
    uint8 timeCnt; 

    // ��ʼ���豸
    if (0 == DS18B20_Init())
    {
        return(0);  // ��ʼ��ʧ��
    }
    
    DS18B20_WriteCMD(SKIP_ROM_CMD);         // ����ROM����
    DS18B20_WriteCMD(CONVERT_T_CMD);        // �¶�ת������
    // ��ʱ800ms����, ȷ��ת�����
    for (timeCnt=0; timeCnt<80; timeCnt++)  
    {
        HalDs18b20DelayNus(10000);          //  ��ʱ10ms
    }
    
    // �ٴθ�λ�豸
    if (0 == DS18B20_Init())
    {
        return(0);  // ��ʼ��ʧ��
    }
    
    DS18B20_WriteCMD(SKIP_ROM_CMD);         // ����ROM����
    DS18B20_WriteCMD(READ_SCRATCHPAD_CMD);  // ��ȡת�����
    return(DS18B20_ReadTemp());             // �����¶�ֵ
}