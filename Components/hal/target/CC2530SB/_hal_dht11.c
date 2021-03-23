/*******************************************************************************
 * 文   件   名：hal_DHT11.c
 *
 * 作        者：yizedxl
 *
 * 制  作 日 期：2013.4.30
 *
 * 最近一次修改：2013.9.13
 *
 * 说        明：1、实现DHT11温湿度传感器的数据读取,驱动将来搬迁到Zstack协议栈;
 *               2、如果测试出现问题(例如全0)，请调整函数DHT11_ReadData(void)的判1区间;
 *               3、根据手册：每次读出的温湿度数值是上一次测量的结果，欲获取实时数据,
 *                            需连续读取两次，但不建议连续多次读取传感器，每次读取传
 *                            感器间隔大于5秒即可获得准确的数据。
 *               4、湿度范围：20%~90%; 温度范围：0~50C;
 *               5、传感器采集到的数据只有整数部分，暂时没有小数部分；
 *               6、数据一共5个字节：第一：湿度整数；第二：湿度小数；
 *                                   第三：温度整数；第四：温度小数；
 *                                   第五：校验(校验值为前四个字节的和)；
 *
 ********************************************************************************/

#include <ioCC2530.h>
#include "hal_types.h"
#include "osal.h"
#include "hal_mcu.h"
#include "hal_defs.h"

//*****************************************************************************/
//引脚宏定义
#define DHT11_SET_PIN_WRITE    P1DIR |= BV(1)
#define DHT11_SET_PIN_READ     P1DIR &= ~BV(1)
#define DHT11_WRITE_H          P1_1   = 1
#define DHT11_WRITE_L          P1_1   = 0
#define DHT11_READ_DATA        P1_1

//*****************************************************************************/
// 宏定义
// 脉冲等待时限
#define TIMEOUT_LIMIT 100

//*****************************************************************************/
//函数名：DHT11_DelayNus
//功  能：延时n微秒, (经过测试，该方法并不精确)
//参  数：n
//返回值：无
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
//函数名：DHT11_Start
//功  能：启动DHT11
//参  数：无
//返回值：无
//*****************************************************************************/
uint8 DHT11_Start(void)
{
    uint8 i;
    
    DHT11_SET_PIN_WRITE;
    DHT11_WRITE_L;
    DHT11_DelayNus(22000); // 低电平保持时间不能小于18ms
    DHT11_WRITE_H;
    DHT11_SET_PIN_READ;
    
    //DHT11输出 80微秒的低电平作为应答信号
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
    // 紧接着输出 80 微秒的高电平通知外设准备接收数据
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
//函数名：DHT11_ReadData
//功  能：从DHT11读取数据
//参  数：无
//返回值：读取的数据
//说 明 ：位数据“0”的格式为: 50微秒的低电平和26-28微秒的高电平，
//        位数据“1”的格式为: 50微秒的低电平和70微秒的高电平	
//*****************************************************************************/
uint8 DHT11_ReadData(void)
{
    uint8 i, j;
    uint8 data = 0;
    halIntState_t intState;
    
    DHT11_SET_PIN_READ;
    for (i = 0; i < 8; i++)
    {
        HAL_ENTER_CRITICAL_SECTION(intState); // 关中断
        for (j = 0; j < TIMEOUT_LIMIT; j++)
        {
            if (DHT11_READ_DATA)
            {
                break;
            }
            DHT11_DelayNus(1);
        }        
    
        if (j == TIMEOUT_LIMIT) // 超时溢出
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
        HAL_EXIT_CRITICAL_SECTION(intState); // 开中断  
    
        if (j == TIMEOUT_LIMIT) // 超时溢出
        {
            return 0;
        }
        
        data <<= 1;
        
        // 如果测试出现问题(例如输出数据全0)，请调整下面的判1区间;
        // 目前测试范围5~10，如果数据有误，调整此处的判1区间
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
//函数名：DHT11_Convert
//功  能：一次完整的数据采集过程
//参  数：data：待写入的数据
//返回值：error：1-出错；0-正常读取；
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
    
    // 计算校验值
    dataCheck = (datatmp[0] + datatmp[1] + datatmp[2] + datatmp[3]);
    
    // 验证校验值
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
