/*****************************************************************************
*
* 文 件 名：hal_ds18b20.c

* 作    者: 南京安宸博研电子科技有限公司

* 创建时间: 2019.04.01

* 修改时间: 2019.04.21

* IAR 版本: IAR for 8051 V8.10.1

* 测试平台: Sensor MotherBoard V2.3

* 说    明: 1. DS18B20单线总线通信, CC2530与此通过P1_1管脚进行通信.
*           2. DS18B20空闲状态时, 总线为高电平.
*           3. 拉低总线480us可以复位DS18B20.
*              需连续读取两次, 但不建议连续多次读取传感器, 每次读取传感器间隔
*              大于5秒即可获得准确的数据.
*           4. 测量的温度数值12位精度: 后4位是小数, 前面的5位空, 中间的7位是
*              整数部分.
*                               
*****************************************************************************/

// 头文件
#include <ioCC2530.h>
#include "hal_types.h"
#include "hal_mcu.h"

// 引脚宏定义, 输入/输出是针对MCU而言
#define SET_DS18B20_PIN_HIGH    (P1_1 = 1)
#define SET_DS18B20_PIN_LOW     (P1_1 = 0)
#define SET_DS18B20_PIN_INPUT   (P1DIR &= ~0x02)
#define SET_DS18B20_PIN_OUTPUT  (P1DIR |= 0x02)
#define GET_DS18B20_PIN_DATA    (P1_1)


// DS18B20相关命令
#define CONVERT_T_CMD           0x44
#define SKIP_ROM_CMD            0xCC
#define READ_SCRATCHPAD_CMD     0xBE
#define WRITE_SCRATCHPAD_CMD    0x4E

/*****************************************************************************
 * @fn          HalDs18b20DelayNus
 *
 * @brief       裸机条件下、时钟频率为32MHz时实现N个微秒的延时(不精确).
 *
 * @param       n--延时长度
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
 * @brief       对DS18B20进行复位操作.
 *
 * @param       none
 *
 * @return      1--成功, 0--失败
 */
uint8 DS18B20_Init(void)
{
    uint8 opSts = 0;
    halIntState_t intState;
    
    SET_DS18B20_PIN_OUTPUT;
    SET_DS18B20_PIN_LOW;
    HalDs18b20DelayNus(750);    // 480us以上的低电平复位脉冲
    
    SET_DS18B20_PIN_HIGH; 
    SET_DS18B20_PIN_INPUT;      // 高电平持续存在, 等待DS18B20拉低总线
    
    HAL_ENTER_CRITICAL_SECTION(intState);
    HalDs18b20DelayNus(80);     // 等待80us
    if(GET_DS18B20_PIN_DATA)    // DS18B20发出60~240us的低电平存在脉冲  
    {
        opSts = 0;  // 初始化失败, 总线上没有设备
    }
    else
    {
        opSts = 1;  // 初始化成功, 总线上有设备拉低总线
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
    
    // 释放总线   
    SET_DS18B20_PIN_OUTPUT;
    SET_DS18B20_PIN_HIGH;
    HalDs18b20DelayNus(500);
    
    return(opSts);
}

/*****************************************************************************
 * @fn          DS18B20_WriteCMD
 *
 * @brief       向DS18B20写入一个字节的数据, 低位先发送.
 *
 * @param       wdata--写入的数据
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
        HalDs18b20DelayNus(30); // DS18B20采样时长15~45us
        HAL_EXIT_CRITICAL_SECTION(intState);
        
        // 释放总线
        SET_DS18B20_PIN_HIGH;
        HalDs18b20DelayNus(10); // 总线恢复时间 
    }
}

/*****************************************************************************
 * @fn          DS18B20_ReadByte
 *
 * @brief       从DS18B20读取一个字节的数据.
 *
 * @param       none
 *
 * @return      读出的一个字节数据
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
        
        // 准备读数据
        SET_DS18B20_PIN_OUTPUT;
        SET_DS18B20_PIN_LOW;
        HalDs18b20DelayNus(2);  // 延时2us
        // 等待设备响应
        SET_DS18B20_PIN_HIGH;
        SET_DS18B20_PIN_INPUT;
        HalDs18b20DelayNus(15); // 主机延时15us后采样数据
        if (GET_DS18B20_PIN_DATA)   
        {
            tmpData |= 0x80;
        }
        HalDs18b20DelayNus(30); // 等待从机发送数据结束
        HAL_EXIT_CRITICAL_SECTION(intState);
        
        // 设置引脚为输出, 然后释放总线
        SET_DS18B20_PIN_OUTPUT;
        SET_DS18B20_PIN_HIGH;
        HalDs18b20DelayNus(10); // 总线恢复时间           
    }

    return  tmpData;
}

/*****************************************************************************
 * @fn          DS18B20_ReadTemp
 *
 * @brief       从DS18B20的ScratchPad读取温度转换结果.
 *
 * @param       none
 *
 * @return      读取的温度数值
 */
uint16 DS18B20_ReadTemp(void)
{
    uint8   tmpLowByte;
    uint16  tmpData;
    
    tmpLowByte  = DS18B20_ReadByte();           // 读低字节
    tmpData     = DS18B20_ReadByte() & 0x00FF;  // 读高字节
    tmpData     = ((tmpData << 8) | tmpLowByte);
    
    return  tmpData;
}

/*****************************************************************************
 * @fn          DS18B20_ConvertTemp
 *
 * @brief       控制DS18B20完成一次温度转换.
 *
 * @param       none
 *
 * @return      测量的温度数值, 后4位是小数, 前面的5位空, 中间的7位是整数部分.
 *              返回0表示出错, 这个根据实际情况可以修改合适的数值.
 */
uint16 DS18B20_ConvertTemp(void)
{
    uint8 timeCnt; 

    // 初始化设备
    if (0 == DS18B20_Init())
    {
        return(0);  // 初始化失败
    }
    
    DS18B20_WriteCMD(SKIP_ROM_CMD);         // 跳过ROM命令
    DS18B20_WriteCMD(CONVERT_T_CMD);        // 温度转换命令
    // 延时800ms以上, 确保转换完成
    for (timeCnt=0; timeCnt<80; timeCnt++)  
    {
        HalDs18b20DelayNus(10000);          //  延时10ms
    }
    
    // 再次复位设备
    if (0 == DS18B20_Init())
    {
        return(0);  // 初始化失败
    }
    
    DS18B20_WriteCMD(SKIP_ROM_CMD);         // 跳过ROM命令
    DS18B20_WriteCMD(READ_SCRATCHPAD_CMD);  // 读取转换结果
    return(DS18B20_ReadTemp());             // 返回温度值
}
