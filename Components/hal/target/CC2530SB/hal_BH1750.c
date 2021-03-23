/*****************************************************************************
*
* 文 件 名：hal_BH1750.c

* 作    者: 南京安宸博研电子科技有限公司

* 创建时间: 2019.04.01

* 修改时间: 2019.04.21

* IAR 版本: IAR for 8051 V8.10.1

* 测试平台: Sensor MotherBoard V2.3

* 说    明: 1. 设置I2C接口涉及的相关GPIO.
*           2. 利用GPIO口模拟I2C总线的读写等操作.
*                               
*****************************************************************************/

#include "iocc2530.h"
#include "hal_types.h"
#include "hal_mcu.h"

// 功能函数的宏定义
#define st(x)                             do { x } while (__LINE__ == -1)
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )
#define HAL_IO_GET(port, pin)             HAL_IO_GET_PREP( port,pin)
#define HAL_IO_GET_PREP(port, pin)        (P##port##_##pin)

// I2C总线相关GPIO口宏定义, 针对MCU而言
#define SET_SDA_PIN_OUTPUT     (P1DIR |=0x10  )
#define SET_SDA_PIN_INPUT      (P1DIR &=~0x10 )
#define SET_SCL_PIN_OUTPUT     (P1DIR |=0x04  )
#define SET_DVI_PIN_OUTPUT     (P1DIR |=0x08  )

#define SET_SCL_LOW             HAL_IO_SET(1,2,0)
#define SET_SCL_HIGH            HAL_IO_SET(1,2,1)
#define SET_SDA_LOW             HAL_IO_SET(1,4,0)
#define SET_SDA_HIGH            HAL_IO_SET(1,4,1)
#define SET_DVI_LOW             HAL_IO_SET(1,3,0)
#define SET_DVI_HIGH            HAL_IO_SET(1,3,1)

#define GET_SDA_DATA            HAL_IO_GET(1,4)

// ADDR引脚接地, 最低位为读写位
#define BH1750_ADDR     0x46
#define READ_CMD        0x01

// BH1750操作命令
#define POWER_DOWN      0x00    // 断电
#define POWER_ON        0x01    // 上电
#define RESET           0x07    // 复位
#define CON_HR_MODE     0x10    // 连续H分辨率, 1lx分辨率, 测量时间一120ms
#define CON_HR_MODE2    0x11    // 连续H分辨率2, 0.5lx分辨率, 测量时间为120ms
#define CON_LR_MODE     0x13    // 连续低分辨, 4lx分辨率, 测量时间一16ms
// 一次测量后, 自动处于断电模式
#define ONCE_HR_MODE    0x20    // 一次H分辨率, 1lx分辨率, 测量时间一120ms
#define ONCE_HR_MODE2   0x21    // 一次H分辨率2, 0.5lx分辨率, 测量时间为120ms
#define ONCE_LR_MODE    0x23    // 一次L分辨率模式4lx分辨率, 测量时间一16ms

/*****************************************************************************
 * @fn          I2C_DelayNus
 *
 * @brief       裸机条件下、时钟频率为32MHz时实现N个微秒的延时(不精确).
 *
 * @param       n--延时长度
 *
 * @return      none
 */
void I2C_DelayNus(uint16 cnt)
{
    while (cnt--)
    {
        asm("NOP");  
        asm("NOP");
        asm("NOP");
    }
}

/*****************************************************************************
 * @fn          I2C_Start
 *
 * @brief       启动I2C, 占用总线, 数据在时钟高电平的时候从高往低跃变.
 *
 * @param       none
 *
 * @return      none
 */
void I2C_Start(void)
{
    SET_SDA_PIN_OUTPUT;
    SET_SDA_HIGH;
    I2C_DelayNus(10);
    SET_SCL_HIGH;
    I2C_DelayNus(10);
    SET_SDA_LOW;
    I2C_DelayNus(10);
    SET_SCL_LOW;
    I2C_DelayNus(10);
}

/*****************************************************************************
 * @fn          I2C_Stop
 *
 * @brief       结束I2C, 释放总线, 数据在时钟高电平的时候从低往高跃变.
 *
 * @param       none
 *
 * @return      none
 */
void I2C_Stop(void)
{
    SET_SDA_PIN_OUTPUT ;
    SET_SCL_LOW ;
    I2C_DelayNus(10);
    SET_SDA_LOW ;
    I2C_DelayNus(10);
    SET_SCL_HIGH ;
    I2C_DelayNus(10);
    SET_SDA_HIGH ;
    I2C_DelayNus(10);
    SET_SCL_LOW ;
    I2C_DelayNus(10);
}

/*****************************************************************************
 * @fn          I2C_Write
 *
 * @brief       发送字节并且判断是否收到ACK(低电平), 高位先发送.
 *
 * @param       none
 *
 * @return      0: 收到ACK; 1: 未收到ACK.
 */
uint8 I2C_Write(uint8 val)                 
{
    uint8 tmpData, rtnData;
    uint8 cnt;
    halIntState_t intState;
    
    tmpData = 0x80;
    SET_SDA_PIN_OUTPUT;

    // 循环发送8位数据   
    for(cnt=0; cnt<8; cnt++)
    {
        if(val & tmpData)
        {
            SET_SDA_HIGH;
        }
        else
        {
            SET_SDA_LOW;
        }
        I2C_DelayNus(10);
        SET_SCL_HIGH ; 
        I2C_DelayNus(10);
        SET_SCL_LOW ;
        I2C_DelayNus(10);
        tmpData = tmpData>>1;   // 右移一位             
     }

    // 接收并判别ACK信号
    I2C_DelayNus(10);
    HAL_ENTER_CRITICAL_SECTION(intState);
    SET_SDA_PIN_INPUT;
    I2C_DelayNus(10);
    SET_SCL_HIGH; 
    I2C_DelayNus(10);
    if(GET_SDA_DATA)
    {
        rtnData = 1; // 未收到ACK信号
    }
    else
    {
        rtnData = 0; // 收到ACK信号
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
    I2C_DelayNus(10);
    SET_SCL_LOW;

    return(rtnData); 
}

/*****************************************************************************
 * @fn          I2C_Read
 *
 * @brief       MCU读取一个字节，根据mode参数, 发送ACK(低电平)或NACK(高电平).
 *
 * @param       mode : 1--数据读取未结束, 0--结束数据读取.
 *
 * @return      读取的数据
 */
uint8 I2C_Read(uint8 mode)
{
    uint8 cnt;
    uint8 val, tmpData;
    halIntState_t intState;
    
    val = 0;
    tmpData = 0x80;
    SET_SDA_HIGH;
    // 循环读取8位数据
    for(cnt=0; cnt<8; cnt++)
    {       
        SET_SCL_HIGH;
        HAL_ENTER_CRITICAL_SECTION(intState);
        SET_SDA_PIN_INPUT;
        I2C_DelayNus(10);
        if(GET_SDA_DATA)
        {
            val |= tmpData;
        }
        HAL_EXIT_CRITICAL_SECTION(intState);
        tmpData = tmpData>>1;
        I2C_DelayNus(10);
        SET_SCL_LOW ;
        I2C_DelayNus(10);                    
    }
    // MCU发送ACK或NACK信号
    SET_SDA_PIN_OUTPUT;
    if(mode)
    {
        SET_SDA_LOW;
    }
    else
    {
        SET_SDA_HIGH;
    }
    I2C_DelayNus(10);
    SET_SCL_HIGH ;
    I2C_DelayNus(10);
    SET_SCL_LOW ;   // 置时钟线为空闲状态
    SET_SDA_HIGH;   // 置数据线为空闲状态
    
    return val;
}

/*********************************************************************
 * @fn      BH1750_Init
 *
 * @brief   BH1750初始化
 *
 * @param   none
 *
 * @return  none
 */
void BH1750_Init(void)
{
    SET_SCL_PIN_OUTPUT;
    SET_DVI_PIN_OUTPUT;
    // 复位芯片
    SET_DVI_LOW;
    I2C_DelayNus(10);
    SET_DVI_HIGH;
    I2C_DelayNus(100);
}

/*********************************************************************
 * @fn      BH1750_ConvertLight
 *
 * @brief   读取BH1750光照信息
 *
 * @param   none
 *
 * @return  光照度值(1~65535lx)
 */
uint16 BH1750_ConvertLight(void)
{        
    uint8 tmpData;
    uint16 lightData = 0;
    
    // 初始化BH1750及其相关引脚
    BH1750_Init();
    
    // 启动总线
    I2C_Start();
    if(I2C_Write(BH1750_ADDR))
    {
        return(0);  // 未收到ACK, 返回０
    }
    // 上电
    if(I2C_Write(POWER_ON))
    {
         return(0);  // 未收到ACK, 返回０
    }
    I2C_Stop();
    
    // 开始测量
    I2C_Start();
    if(I2C_Write(BH1750_ADDR))
    {
        return(0);  // 未收到ACK, 返回０
    }
    // 设置为连续Ｈ分辨率模式
    if(I2C_Write(CON_HR_MODE))
    {
         return(0);  // 未收到ACK, 返回０
    }
    I2C_Stop();

    // 转换时间120ms
    I2C_DelayNus(20000);
    
    I2C_Start();
    if(I2C_Write(BH1750_ADDR + READ_CMD))
    {
        return(0);  // 未收到ACK, 返回０
    }
    tmpData = I2C_Read(1);      // 高字节数据
    lightData = I2C_Read(0);    // 低字节数据
    I2C_Stop();
    
    lightData = (uint16)(((tmpData << 8) | lightData) / 1.2);
    return lightData;
}