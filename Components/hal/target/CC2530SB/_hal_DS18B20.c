//*****************************************************************************/
//��    �ߣ�yizedxl
//�޸�ʱ�䣺2013.7.18
//˵    ����(1)DS18B20������ͨ�ţ���CC2530ͨ��P1_1�ܽ�ͨ�ţ�
//          (2)DS18B20����״̬ʱ���߸ߵ�ƽ��
//          (3)��������480us���Ը�λDS18B20��
//          (4)�������¶���ֵ12λ���ȣ���4λ��С����ǰ���5λ�գ��м��7λ���������֡�
//*****************************************************************************/


#include <ioCC2530.h>
#include "hal_types.h"
#include "hal_mcu.h"

#define DS18B20_DATA_H    P1_1 = 1
#define DS18B20_DATA_L    P1_1 = 0
#define DS18B20_DATA_IN   P1DIR &= ~0x02
#define DS18B20_DATA_OUT  P1DIR |= 0x02
#define DS18B20_DATA      P1_1

//************************************************************/
//����
#define DS18B20_CMD_ConvertTemp     0x44
#define DS18B20_CMD_SkipROMCMD      0xCC
#define DS18B20_CMD_ReadScratchpad  0xBE

/*******************************************
�������ƣ�HalDs18b20DelayNus
��    �ܣ�ʵ��N��΢�����ʱ(����ȷ)
��    ����n--��ʱ����
����ֵ  ����
˵��    ��
********************************************/
void HalDs18b20DelayNus(uint16 n)
{
    uint16 i;
    for (i = 0; i < n; i++)
    {
        asm("NOP"); 
    }
}
/*******************************************
�������ƣ�DS18B20_Init
��    �ܣ���DS18B20���и�λ����
��    ������
����ֵ  ����ʼ��״̬��־��1--ʧ�ܣ�0--�ɹ�
********************************************/
uint8 DS18B20_Init(void)
{
    uint8 Error;
    halIntState_t intState;

    DS18B20_DATA_OUT;
    
    DS18B20_DATA_L;
    HalDs18b20DelayNus(500); // 480us���ϵĸ�λ����
    
    DS18B20_DATA_H;
    
    HAL_ENTER_CRITICAL_SECTION(intState);
    DS18B20_DATA_IN;
    HalDs18b20DelayNus(60);  // DS18B20�ȴ�
    
    if(DS18B20_DATA)    // DS18B20����60~240us�ĵ͵�ƽ��������  
    {
        Error = 1;          //��ʼ��ʧ��
    }
    else
    {
        Error = 0;          //��ʼ���ɹ�
    }
    
    HAL_EXIT_CRITICAL_SECTION(intState);
    DS18B20_DATA_OUT;
    DS18B20_DATA_H;
   
    HalDs18b20DelayNus(500);
    
    return Error;
}
/*******************************************
�������ƣ�DS18B20_WriteCMD
��    �ܣ���DS18B20д��һ���ֽڵ�����
��    ����wdata--д�������
����ֵ  ����
********************************************/
void DS18B20_WriteCMD(uint8 wdata)
{
    uint8 i;
    
    for (i = 0; i < 8; i++)
    {
        DS18B20_DATA_L;
        HalDs18b20DelayNus(15);            
        
        if(wdata & 0X01)    
        {
            DS18B20_DATA_H;
        }
        else
        {
            DS18B20_DATA_L;
        }
        
        wdata >>= 1;
        HalDs18b20DelayNus(50); // DS18B20����ʱ��15~45us
        
        DS18B20_DATA_H;
        HalDs18b20DelayNus(10);           
    }
}
/*******************************************
�������ƣ�DS18B20_ReadByte
��    �ܣ���DS18B20��ȡһ���ֽڵ�����
��    ������
����ֵ  ��������һ���ֽ�����
********************************************/
uint8 DS18B20_ReadByte(void)
{
    uint8 i;
    uint8 temp = 0;
    
    for (i = 0; i < 8; i++)
    {
        temp >>= 1;
        DS18B20_DATA_L;
        HalDs18b20DelayNus(6);            //��ʱ6us
        DS18B20_DATA_H;
        HalDs18b20DelayNus(8);            //��ʱ9us��15us������������
        DS18B20_DATA_IN;
        if (DS18B20_DATA)   
        {
            temp |= 0x80;
        }
        
        HalDs18b20DelayNus(50);
        
        DS18B20_DATA_OUT;
        DS18B20_DATA_H;
        HalDs18b20DelayNus(10);           
    }
    
    return  temp;
}

/*******************************************
�������ƣ�DS18B20_ReadTemp
��    �ܣ���DS18B20��ScratchPad��ȡ�¶�ת�����
��    ������
����ֵ  ����ȡ���¶���ֵ
********************************************/
uint16 DS18B20_ReadTemp(void)
{
    uint8 temp_low;
    uint16  temp;
    
    temp_low = DS18B20_ReadByte(); // ����λ
    temp     = DS18B20_ReadByte(); // ����λ
    temp     = ((temp << 8) | temp_low) & 0x07FF;
    
    return  temp;
}

/*******************************************************************************
�������ƣ�DS18B20_ConvertTemp
��    �ܣ�����DS18B20���һ���¶�ת��
��    ������
����ֵ  ���������¶���ֵ����4λ��С����ǰ���5λ�գ��м��7λ���������֡�
          ����0��ʾ�����������ʵ����������޸ĺ��ʵ���ֵ
*******************************************************************************/
uint16 DS18B20_ConvertTemp(void)
{
    uint8 i; 

    i = DS18B20_Init();
    if (i)
    {
        return(0);
    }
    
    DS18B20_WriteCMD(DS18B20_CMD_SkipROMCMD);
    DS18B20_WriteCMD(DS18B20_CMD_ConvertTemp);
    
    for (i = 80; i > 0; i--)  //��ʱ800ms����
    {
        HalDs18b20DelayNus(10000); 
    }
    
    
    i = DS18B20_Init();
    if (i)
    {
        return(0);
    }
    
    DS18B20_WriteCMD(DS18B20_CMD_SkipROMCMD);
    DS18B20_WriteCMD(DS18B20_CMD_ReadScratchpad);
    return DS18B20_ReadTemp();
}
