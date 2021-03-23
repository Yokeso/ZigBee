

#include "hal_types.h"
#include "iocc2530.h"

/************************************************
ע�⣺I2C�ӿ���������BH1750
*************************************************/


#define st(x)                             do { x } while (__LINE__ == -1)
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin = val; )
#define HAL_IO_GET(port, pin)             HAL_IO_GET_PREP( port,pin)
#define HAL_IO_GET_PREP(port, pin)        (P##port##_##pin)

#define Write_SCL_0         HAL_IO_SET(1,2,0)
#define Write_SCL_1         HAL_IO_SET(1,2,1)
#define Write_SDA_0         HAL_IO_SET(1,4,0)
#define Write_SDA_1         HAL_IO_SET(1,4,1)

#define Get_SDA             HAL_IO_GET(1,4)

#define SDA_Write (P1DIR |=0x10  )
#define SDA_Read  (P1DIR &=~0x10 )
#define SCL_Write (P1DIR |=0x04  )

void Hal_I2C_DelayNus(uint16 n)
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

/****************************
���ܣ�����I2C
       �������߱�ռ��
˵����������ʱ�Ӹߵ�ƽ��ʱ��Ӹ�����Ծ��

*****************************/

void I2C_Start(void)
{
        SDA_Write;
        Write_SDA_1;
        Hal_I2C_DelayNus(10);
        Write_SCL_1;
        Hal_I2C_DelayNus(10);
        Write_SDA_0;
        Hal_I2C_DelayNus(10);
        Write_SCL_0;
        Hal_I2C_DelayNus(10);
}

/********************************

���ܣ�����I2C
      �ͷ����ߣ��������߿���
˵����������ʱ�Ӹߵ�ƽ��ʱ��ӵ�����Ծ��
********************************/

void I2C_Stop(void)
{
        SDA_Write ;
        Write_SCL_0 ;
        Hal_I2C_DelayNus(10);
        Write_SDA_0 ;
        Hal_I2C_DelayNus(10);
        Write_SCL_1 ;
        Hal_I2C_DelayNus(10);
        Write_SDA_1 ;
        Hal_I2C_DelayNus(10);
        Write_SCL_0 ;
        Hal_I2C_DelayNus(10);
}



/******************************
�����ֽڲ����ж��Ƿ��յ�ACK
���յ�ACK����Ϊ0�����򷵻�Ϊ1

******************************/


uint8 I2C_Send(uint8 val)                 
{
        uint8 i,t;
        uint8 error=0;
        SDA_Write;
        i=0x80;
        for(t=0;t<8;t++)
        {
            if(val&i)
                    Write_SDA_1;
            else
                    Write_SDA_0;
            Hal_I2C_DelayNus(10);
            Write_SCL_1 ; 
            Hal_I2C_DelayNus(10);
            Write_SCL_0 ;
            Hal_I2C_DelayNus(10);
            i=i>>1;             
         }
        //ACK
        Hal_I2C_DelayNus(10);
        SDA_Read;
        Hal_I2C_DelayNus(10);
        Write_SCL_1; 
        Hal_I2C_DelayNus(10);
        if(Get_SDA)
          error=1;
        Hal_I2C_DelayNus(10);
        Write_SCL_0;
        return error;
}

/***************************

��ȡI2C���ֽڣ����ҷ���ACK
������Ϊ1��ʱ����һ��ACK(�͵�ƽ)


***************************/

uint8 I2C_Read(uint8 ack)
{
    uint8 i;
    uint8 val=0;
    Write_SDA_1;
    
    for(i=0x80;i>0;i/=2)
    {
            
            Write_SCL_1 ;
            Hal_I2C_DelayNus(10);
            SDA_Read;
            if(Get_SDA)
                    val=(val|i);
            Hal_I2C_DelayNus(10);
            Write_SCL_0 ;
            Hal_I2C_DelayNus(10);
            
            
    }
    SDA_Write;
    if(ack)
            Write_SDA_0;
    else
            Write_SDA_1;
    Hal_I2C_DelayNus(10);
    Write_SCL_1 ;
    Hal_I2C_DelayNus(10);
    Write_SCL_0 ;
    Write_SDA_1;
    return val;
    
}