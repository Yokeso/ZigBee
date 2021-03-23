//*****************************************************************************/
//作    者：yizedxl
//修改时间：2013.7.18
//说    明：(1)DS18B20单总线通信，和CC2530通过P1_1管脚通信；
//          (2)DS18B20空闲状态时总线高电平；
//          (3)拉低总线480us可以复位DS18B20；
//          (4)测量的温度数值12位精度：后4位是小数，前面的5位空，中间的7位是整数部分。
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
//命令
#define DS18B20_CMD_ConvertTemp     0x44
#define DS18B20_CMD_SkipROMCMD      0xCC
#define DS18B20_CMD_ReadScratchpad  0xBE

/*******************************************
函数名称：HalDs18b20DelayNus
功    能：实现N个微秒的延时(不精确)
参    数：n--延时长度
返回值  ：无
说明    ：
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
函数名称：DS18B20_Init
功    能：对DS18B20进行复位操作
参    数：无
返回值  ：初始化状态标志：1--失败，0--成功
********************************************/
uint8 DS18B20_Init(void)
{
    uint8 Error;
    halIntState_t intState;

    DS18B20_DATA_OUT;
    
    DS18B20_DATA_L;
    HalDs18b20DelayNus(500); // 480us以上的复位脉冲
    
    DS18B20_DATA_H;
    
    HAL_ENTER_CRITICAL_SECTION(intState);
    DS18B20_DATA_IN;
    HalDs18b20DelayNus(60);  // DS18B20等待
    
    if(DS18B20_DATA)    // DS18B20发出60~240us的低电平存在脉冲  
    {
        Error = 1;          //初始化失败
    }
    else
    {
        Error = 0;          //初始化成功
    }
    
    HAL_EXIT_CRITICAL_SECTION(intState);
    DS18B20_DATA_OUT;
    DS18B20_DATA_H;
   
    HalDs18b20DelayNus(500);
    
    return Error;
}
/*******************************************
函数名称：DS18B20_WriteCMD
功    能：向DS18B20写入一个字节的数据
参    数：wdata--写入的数据
返回值  ：无
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
        HalDs18b20DelayNus(50); // DS18B20采样时长15~45us
        
        DS18B20_DATA_H;
        HalDs18b20DelayNus(10);           
    }
}
/*******************************************
函数名称：DS18B20_ReadByte
功    能：从DS18B20读取一个字节的数据
参    数：无
返回值  ：读出的一个字节数据
********************************************/
uint8 DS18B20_ReadByte(void)
{
    uint8 i;
    uint8 temp = 0;
    
    for (i = 0; i < 8; i++)
    {
        temp >>= 1;
        DS18B20_DATA_L;
        HalDs18b20DelayNus(6);            //延时6us
        DS18B20_DATA_H;
        HalDs18b20DelayNus(8);            //延时9us，15us后建议主机采样
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
函数名称：DS18B20_ReadTemp
功    能：从DS18B20的ScratchPad读取温度转换结果
参    数：无
返回值  ：读取的温度数值
********************************************/
uint16 DS18B20_ReadTemp(void)
{
    uint8 temp_low;
    uint16  temp;
    
    temp_low = DS18B20_ReadByte(); // 读低位
    temp     = DS18B20_ReadByte(); // 读高位
    temp     = ((temp << 8) | temp_low) & 0x07FF;
    
    return  temp;
}

/*******************************************************************************
函数名称：DS18B20_ConvertTemp
功    能：控制DS18B20完成一次温度转换
参    数：无
返回值  ：测量的温度数值，后4位是小数，前面的5位空，中间的7位是整数部分。
          返回0表示出错，这个根据实际情况可以修改合适的数值
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
    
    for (i = 80; i > 0; i--)  //延时800ms以上
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
