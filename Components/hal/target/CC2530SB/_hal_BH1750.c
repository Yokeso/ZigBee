//*****************************************************************************/
//
// 文 件 名：hal_bh1750.c
//
// 作    者：yizedxl
//
// 创建时间：2013.9.29
//
// 修改时间：2013.11.3
//
// IAR 版本：IAR for 8051 V8.10.1
//
// 测试平台：Sensor Battery MotherBoard V2.0
//
// 说    明：bh1750光照度检测，模拟I2C接口，I2C驱动文件在hal_I2C.c；
//           光照度范围0~65535lx，
//
//*****************************************************************************/

#include "hal_i2c.h"
#include "iocc2530.h"
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

/**************************
测量光张强度
***************************/

#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin = val; )
#define HAL_IO_GET(port, pin)             HAL_IO_GET_PREP( port,pin)
#define HAL_IO_GET_PREP(port, pin)        (P##port##_##pin)

#define Write_SCL_0         HAL_IO_SET(1,2,0)
#define Write_SCL_1         HAL_IO_SET(1,2,1)
#define Write_SDA_0         HAL_IO_SET(1,4,0)
#define Write_SDA_1         HAL_IO_SET(1,4,1)

#define Get_SDA             HAL_IO_GET(1,4)

#define SDA_Write (P1DIR |= BV(4)  )
#define SDA_Read  (P1DIR &= ~BV(4) )
#define SCL_Write (P1DIR |= BV(2)  )

#define BH1750_Addr 0x46    //addr接地时,最低位为读写位

/*****************command***************************/
#define POWRDOWN    0X00         //断电
#define POWERON     0X01         //SHANG DIAN
#define RESET       0X07         //CHONG ZHI
#define CHMODE      0X10         //连续H分辨率
#define CHMODE2     0X11         //连续H分辨率2
#define CLMODE      0X13        //连续低分辨
#define H1MODE      0X20        //一次H分辨率
#define H1MODE2     0X21         //一次H分辨率2
#define L1MODE      0X23       //一次L分辨率模式

/*********************************************************************
 * @fn      Hal_BH1750_DelayNus
 *
 * @brief   n us delay
 *
 * @param   none
 *
 * @return  none
 */
void Hal_BH1750_DelayNus(uint16 n)
{
    while (n--)
    {
        asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
        asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
        asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
        asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
        asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
        asm("nop");asm("nop");
    }
}

/*********************************************************************
 * @fn      BH1750_ConvertLight
 *
 * @brief   读取BH1750光照信息
 *
 * @param   none
 *
 * @return  none
 */
uint16 BH1750_ConvertLight(void)
{        
    uint8 ack=1;
    uint8 tempL, tempH;
    uint16 temp;
    uint8 i;
    
    SCL_Write;
    
    Hal_BH1750_DelayNus(100);

    //上电
    I2C_Start();
    ack = I2C_Send(BH1750_Addr);
    
    if(ack)
        return 255;
    
    ack = I2C_Send(POWERON);
    
    if(ack)
        return 254;
    
    I2C_Stop();
    
    //开始测量
    I2C_Start();
    ack = I2C_Send(BH1750_Addr);
    
    if(ack)
       return 253;
    
    ack = I2C_Send(CHMODE);
    
    if(ack)
       return 252;
    
    I2C_Stop();
                    
    for (i = 0; i < 20; i++)//延时120ms以上
    {
        Hal_BH1750_DelayNus(1000);
    }
    
    I2C_Start();
    ack = I2C_Send(BH1750_Addr + 1);   //发送地址
    
    if(ack)
        return 251;
                    
    tempH = I2C_Read(1);
    tempL = I2C_Read(0);
    I2C_Stop();
    temp = (uint16)(((tempH << 8) | tempL) / 1.2);
    return temp;
}