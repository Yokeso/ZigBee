/*****************************************************************************
*
* �� �� ����hal_BH1750.c

* ��    ��: �Ͼ���已��е��ӿƼ����޹�˾

* ����ʱ��: 2019.04.01

* �޸�ʱ��: 2019.04.21

* IAR �汾: IAR for 8051 V8.10.1

* ����ƽ̨: Sensor MotherBoard V2.3

* ˵    ��: 1. ����I2C�ӿ��漰�����GPIO.
*           2. ����GPIO��ģ��I2C���ߵĶ�д�Ȳ���.
*                               
*****************************************************************************/

#include "iocc2530.h"
#include "hal_types.h"
#include "hal_mcu.h"

// ���ܺ����ĺ궨��
#define st(x)                             do { x } while (__LINE__ == -1)
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )
#define HAL_IO_GET(port, pin)             HAL_IO_GET_PREP( port,pin)
#define HAL_IO_GET_PREP(port, pin)        (P##port##_##pin)

// I2C�������GPIO�ں궨��, ���MCU����
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

// ADDR���Žӵ�, ���λΪ��дλ
#define BH1750_ADDR     0x46
#define READ_CMD        0x01

// BH1750��������
#define POWER_DOWN      0x00    // �ϵ�
#define POWER_ON        0x01    // �ϵ�
#define RESET           0x07    // ��λ
#define CON_HR_MODE     0x10    // ����H�ֱ���, 1lx�ֱ���, ����ʱ��һ120ms
#define CON_HR_MODE2    0x11    // ����H�ֱ���2, 0.5lx�ֱ���, ����ʱ��Ϊ120ms
#define CON_LR_MODE     0x13    // �����ͷֱ�, 4lx�ֱ���, ����ʱ��һ16ms
// һ�β�����, �Զ����ڶϵ�ģʽ
#define ONCE_HR_MODE    0x20    // һ��H�ֱ���, 1lx�ֱ���, ����ʱ��һ120ms
#define ONCE_HR_MODE2   0x21    // һ��H�ֱ���2, 0.5lx�ֱ���, ����ʱ��Ϊ120ms
#define ONCE_LR_MODE    0x23    // һ��L�ֱ���ģʽ4lx�ֱ���, ����ʱ��һ16ms

/*****************************************************************************
 * @fn          I2C_DelayNus
 *
 * @brief       ��������¡�ʱ��Ƶ��Ϊ32MHzʱʵ��N��΢�����ʱ(����ȷ).
 *
 * @param       n--��ʱ����
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
 * @brief       ����I2C, ռ������, ������ʱ�Ӹߵ�ƽ��ʱ��Ӹ�����Ծ��.
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
 * @brief       ����I2C, �ͷ�����, ������ʱ�Ӹߵ�ƽ��ʱ��ӵ�����Ծ��.
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
 * @brief       �����ֽڲ����ж��Ƿ��յ�ACK(�͵�ƽ), ��λ�ȷ���.
 *
 * @param       none
 *
 * @return      0: �յ�ACK; 1: δ�յ�ACK.
 */
uint8 I2C_Write(uint8 val)                 
{
    uint8 tmpData, rtnData;
    uint8 cnt;
    halIntState_t intState;
    
    tmpData = 0x80;
    SET_SDA_PIN_OUTPUT;

    // ѭ������8λ����   
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
        tmpData = tmpData>>1;   // ����һλ             
     }

    // ���ղ��б�ACK�ź�
    I2C_DelayNus(10);
    HAL_ENTER_CRITICAL_SECTION(intState);
    SET_SDA_PIN_INPUT;
    I2C_DelayNus(10);
    SET_SCL_HIGH; 
    I2C_DelayNus(10);
    if(GET_SDA_DATA)
    {
        rtnData = 1; // δ�յ�ACK�ź�
    }
    else
    {
        rtnData = 0; // �յ�ACK�ź�
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
    I2C_DelayNus(10);
    SET_SCL_LOW;

    return(rtnData); 
}

/*****************************************************************************
 * @fn          I2C_Read
 *
 * @brief       MCU��ȡһ���ֽڣ�����mode����, ����ACK(�͵�ƽ)��NACK(�ߵ�ƽ).
 *
 * @param       mode : 1--���ݶ�ȡδ����, 0--�������ݶ�ȡ.
 *
 * @return      ��ȡ������
 */
uint8 I2C_Read(uint8 mode)
{
    uint8 cnt;
    uint8 val, tmpData;
    halIntState_t intState;
    
    val = 0;
    tmpData = 0x80;
    SET_SDA_HIGH;
    // ѭ����ȡ8λ����
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
    // MCU����ACK��NACK�ź�
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
    SET_SCL_LOW ;   // ��ʱ����Ϊ����״̬
    SET_SDA_HIGH;   // ��������Ϊ����״̬
    
    return val;
}

/*********************************************************************
 * @fn      BH1750_Init
 *
 * @brief   BH1750��ʼ��
 *
 * @param   none
 *
 * @return  none
 */
void BH1750_Init(void)
{
    SET_SCL_PIN_OUTPUT;
    SET_DVI_PIN_OUTPUT;
    // ��λоƬ
    SET_DVI_LOW;
    I2C_DelayNus(10);
    SET_DVI_HIGH;
    I2C_DelayNus(100);
}

/*********************************************************************
 * @fn      BH1750_ConvertLight
 *
 * @brief   ��ȡBH1750������Ϣ
 *
 * @param   none
 *
 * @return  ���ն�ֵ(1~65535lx)
 */
uint16 BH1750_ConvertLight(void)
{        
    uint8 tmpData;
    uint16 lightData = 0;
    
    // ��ʼ��BH1750�����������
    BH1750_Init();
    
    // ��������
    I2C_Start();
    if(I2C_Write(BH1750_ADDR))
    {
        return(0);  // δ�յ�ACK, ���أ�
    }
    // �ϵ�
    if(I2C_Write(POWER_ON))
    {
         return(0);  // δ�յ�ACK, ���أ�
    }
    I2C_Stop();
    
    // ��ʼ����
    I2C_Start();
    if(I2C_Write(BH1750_ADDR))
    {
        return(0);  // δ�յ�ACK, ���أ�
    }
    // ����Ϊ�����ȷֱ���ģʽ
    if(I2C_Write(CON_HR_MODE))
    {
         return(0);  // δ�յ�ACK, ���أ�
    }
    I2C_Stop();

    // ת��ʱ��120ms
    I2C_DelayNus(20000);
    
    I2C_Start();
    if(I2C_Write(BH1750_ADDR + READ_CMD))
    {
        return(0);  // δ�յ�ACK, ���أ�
    }
    tmpData = I2C_Read(1);      // ���ֽ�����
    lightData = I2C_Read(0);    // ���ֽ�����
    I2C_Stop();
    
    lightData = (uint16)(((tmpData << 8) | lightData) / 1.2);
    return lightData;
}