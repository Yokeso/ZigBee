/*****************************************************************************
*
* 文 件 名：hal_dht11.c

* 作    者: 柳成林

* 创建时间: 2021.3.2

* IAR 版本: IAR for 8051 V8.10.1

* 测试平台: Sensor MotherBoard V2.3

* 说    明: 1. 实现DHT11温湿度传感器的数据读取, 驱动将来搬迁到Zstack协议栈.
*           2. 如果测试出现问题(如全0), 请调整函数DHT11_ReadData(void)的判1区间.
*           3. 根据手册, 每次读出的温湿度数值是上一次测量的结果, 欲获取实时数据,
*              需连续读取两次, 但不建议连续多次读取传感器, 每次读取传感器间隔
*              大于5秒即可获得准确的数据.
*           4. 湿度范围: 20%~90%; 温度范围: 0~50C.
*           5. 传感器采集到的数据只有整数部分, 暂时没有小数部分(留着扩展).
*           6. 数据一共5个字节: 第一个为湿度整数, 第二个为湿度小数, 第三个为温度
*              整数, 第四个为温度小数,第五个校验字节(校验值为前四个字节的和).
*                               
*****************************************************************************/

// 头文件
#include <ioCC2530.h>
#include "hal_mcu.h"

// 引脚宏定义, 输入/输出是针对MCU而言
#define SET_DHT11_PIN_OUTPUT    (P1DIR |= 0x02)
#define SET_DHT11_PIN_HIGH      (P1_1   = 1)
#define SET_DHT11_PIN_LOW       (P1_1   = 0)
#define SET_DHT11_PIN_INPUT     (P1DIR &= ~0x02)
#define GET_DHT11_PIN_DATA      (P1_1) 

// 参数宏定义
#define TIMEOUT_LIMIT 1000       // 脉冲等待时限1000ms

/*****************************************************************************
 * @fn          DHT11_Delay100us
 *
 * @brief       裸机条件下、时钟频率为32MHz时延时100微秒左右(不精确).
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
 * @brief       裸机条件下、时钟频率为32MHz时延时10微秒左右(不精确).
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
 * @brief       启动DHT11
 *
 * @param       none
 *
 * @return      none
 */
void DHT11_Start(void)
{
    unsigned char timeCnt;
    
    SET_DHT11_PIN_OUTPUT;   // 设置引脚为输出
    SET_DHT11_PIN_LOW;      // 输出低电平, 启动总线
    // 低电平保持时间不能小于18ms, 否则DHT11无法启动
    for(timeCnt=0; timeCnt<200; timeCnt++)
    {
        DHT11_Delay100us();
    }
    
    SET_DHT11_PIN_HIGH;     // 输出高电平
    SET_DHT11_PIN_INPUT;    // 设置为输入引脚(因有上拉电阻, 所以保持为高电平)    

    // 高电平持续20~40us
    for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
    {
        if (!GET_DHT11_PIN_DATA)
        {
            break;  // 低电平退出循环
        }
        DHT11_Delay10us();
    }
    // 超时返回
    if (timeCnt == TIMEOUT_LIMIT)
    {
        return;
    }
    // DHT11输出80微秒的低电平作为应答信号
    for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
    {
        if (GET_DHT11_PIN_DATA)
        {
            break;  // 高电平退出循环
        }
        DHT11_Delay10us();
    }
    // 超时返回
    if (timeCnt == TIMEOUT_LIMIT)
    {
        return;
    }
    // DHT11紧接着输出80微秒的高电平通知外设准备接收数据
    for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
    {
        if (!GET_DHT11_PIN_DATA)    // 低电平退出循环
        {
            break;
        }
        DHT11_Delay10us();
    }
    // 超时返回
    if (timeCnt == TIMEOUT_LIMIT)
    {
        return;
    }
}

/*****************************************************************************
 * @fn          DHT11_ReadData
 *
 * @brief       从DHT11读取一个字节数据
 *
 * @param       none
 *
 * @return      读取的数据, 返回0则表示超时报错.
 *              位数据"0"的格式为50微秒的低电平和26-28微秒的高电平，
 *              位数据"1"的格式为50微秒的低电平和70微秒的高电平.	
 * 
 */
unsigned char DHT11_ReadData(void)
{
    unsigned char bitCnt, timeCnt;
    unsigned char byteVal = 0;
    halIntState_t intState;
    
    for (bitCnt = 0; bitCnt < 8; bitCnt++)
    {
        // DHT11输出50us低电平, 提醒主机数据位已开始传送
        for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
        {
            if (GET_DHT11_PIN_DATA)    // 高电平退出循环
            {
                break;
            }
            DHT11_Delay10us();
        }        
        if (timeCnt == TIMEOUT_LIMIT)   // 超时报错, 返回0
        {
            return 0;
        }
        
        HAL_ENTER_CRITICAL_SECTION(intState);
        // DHT11输出高电平或低电平                   
        for (timeCnt = 0; timeCnt < TIMEOUT_LIMIT; timeCnt++)
        {
            if (!GET_DHT11_PIN_DATA)    // 低电平退出循环
            {
                break;
            }
            DHT11_Delay10us();
        }
        if (timeCnt == TIMEOUT_LIMIT)   // 超时报错, 返回0
        {
            return 0;
        }
        HAL_EXIT_CRITICAL_SECTION(intState);
        
        // 存储数据位, DHT11是先发高位
        byteVal <<= 1;
        // 根据高电平的宽度判断是0还是1
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
 * @brief       启动DHT11, 并完成一次转换.
 *
 * @param       none
 *
 * @return      转换值存在conversionVal数组中, 如果出错则不更新数据. 
 *              出错时, 可按以下几步进行检查:
 *              1. 检查延时函数的是否准确;
 *              2. 检查DHT11是否启动;
 *              3. 检查判别"0"与"1"的阈值是否合理;
 *              4. 检查校验码是否正确.
 * 
 */
void HalDht11_Convert(unsigned char conversionVal[4])
{
    unsigned char checkSum;
    unsigned char tempData[5];
    
    DHT11_Start();      // 启动DHT11
    
    // 读取数据
    tempData[0] = DHT11_ReadData();
    tempData[1] = DHT11_ReadData();
    tempData[2] = DHT11_ReadData();
    tempData[3] = DHT11_ReadData();
    tempData[4] = DHT11_ReadData();
    
    // 计算校验值
    checkSum = (tempData[0] + tempData[1] + tempData[2] + tempData[3]);
    
    // 验证校验值, 出错则不更新数据
    if (tempData[4] == checkSum)
    {
        conversionVal[0] = tempData[0];
        conversionVal[1] = tempData[1];
        conversionVal[2] = tempData[2];
        conversionVal[3] = tempData[3];
    }
    
    SET_DHT11_PIN_OUTPUT;   // 设置DHT11的控制引脚为输出引脚
    SET_DHT11_PIN_HIGH;     // 置高电平, 使DHT11处于低功耗待机模式
}
