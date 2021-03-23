/*******************************************************************************
  文 件 名: hal_motor.c
  作    者：南京安宸博研电子科技有限公司
  创建时间：2013.9.16
  修改时间：2020.12.22
  说    明：
    (1) CC2530的P1_2输出PWMA波控制直流电机正反转，50%占空比停机，小于50%反转，
        大于50%正转；请将直流电机板子上的PWM波选择跳线短接在sin，本实验使用
        单路PWM波输出控制。
    (2) 电机顺时针转动，先过P1_3，然后通过P1_4，此时方向为正方向；
    (3) 本部分程序使用了CC2530的定时器1和定时器3，分别用来产生直流电机的PWM波，
        和1s的定时中断，1s的定时中断作为节拍来控制转速的测量和转速的调节；
    (4) 电机的转速单位：转/秒；目前支持的转速调节范围是：1~50(Max)转/秒；
    (5) 电机控制方法采用PID控制中的P控制，经过测试可以实现基本调速功能。
    (6) 通过修改定时器的计数值，可以调节电机测速和控制的节拍，具体看定时器
        的中断说明，这里有测试比较的结果：当测速和控制的节奏提高后，速度的
        精确度不高，之前是4个脉冲代表1转，提高到1/4秒节拍后是1个脉冲代表1转，
        在观察中发现虽然可以迅速(4倍于1秒的节拍)的调整速度，但是速度稳定性不高，
        总结的主要原因应该是测速模块的稳定性问题。
    (7) 测速模块主要是：在IO中断内的脉冲计数；以及定时器中断内的转速转换；
        测速精度有±1RPM的误差(使用目前1/4秒的测速、控制节拍), 或者
        ±0.25RPM的误差(使用1秒的测速、控制节拍)。
    (8) 关于PID：P = 设定转速 - 当前转速；
                 I += p；
                 D = 当前转速 - 上一个当前转速；
                 上一个当前转速 = 当前转速；
                 然后选择合适的kp、ki、kd
                 调整量 = kp*P + ki*I + kd*D；
*******************************************************************************/

#include <iocc2530.h>
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_motor.h"

//宏定义--电机控制与测速的相关引脚
#define  HAL_MOTOR_SPEED1          P1_4
#define  HAL_MOTOR_SPEED2          P1_3
#define  HAL_MOTOR_SPEED_PIN_SET() st( P1DIR &= ~(BV(3) + BV(4)); \
                                       P1IEN |= BV(4) + BV(3);    \
                                       IEN2 |= BV(4);             \
                                       P1IFG = 0;)
#define  HAL_MOTOR_PWMA_OUT        P1_2
#define  HAL_MOTOR_PWMA_PIN_SET()  st( P1DIR |= BV(2); P1SEL |= BV(2);)

//*****************************************************************************/
//定时器
/*****************************************
//T3配置定义
*****************************************/
// Where _timer_ must be either 3 or 4
// Macro for initialising timer 3 or 4
#define TIMER34_INIT(timer)    st(    T##timer##CTL   = 0x06; \
                                      T##timer##CCTL0 = 0x00; \
                                      T##timer##CC0   = 0x00; \
                                      T##timer##CCTL1 = 0x00; \
                                      T##timer##CC1   = 0x00; \
                                  ) 

//Macro for enabling overflow interrupt
#define TIMER34_ENABLE_OVERFLOW_INT(timer,val) \
   (T##timer##CTL =  (val) ? T##timer##CTL | 0x08 : T##timer##CTL & ~0x08)



// Macro for configuring channel 1 of timer 3 or 4 for PWM mode.
#define TIMER34_PWM_CONFIG(timer)    st ( T##timer##CCTL1 = 0x24;                     \
                                          if(timer == 3){                             \
                                             if(PERCFG & 0x20) {                      \
                                                IO_FUNC_PORT_PIN(1,7,IO_FUNC_PERIPH); \
                                             }                                        \
                                             else {                                   \
                                                IO_FUNC_PORT_PIN(1,4,IO_FUNC_PERIPH); \
                                             }                                        \
                                          }                                           \
                                          else {                                      \
                                             if(PERCFG & 0x10) {                      \
                                                 IO_FUNC_PORT_PIN(2,3,IO_FUNC_PERIPH);\
                                             }                                        \
                                             else {                                   \
                                                IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH); \
                                             }                                        \
                                          }                                           \
                                       ) 

// Macro for setting pulse length of the timer in PWM mode
#define  TIMER34_SET_PWM_PULSE_LENGTH(timer, value)  \
         st( T##timer##CC1 = (BYTE)value; )


// Macro for setting timer 3 or 4 as a capture timer
#define TIMER34_CAPTURE_TIMER(timer,edge)          \
   st(                                             \
      T##timer##CCTL1 = edge;                      \
      if(timer == 3){                              \
         if(PERCFG & 0x20) {                       \
            IO_FUNC_PORT_PIN(1,7,IO_FUNC_PERIPH);  \
         }                                         \
         else {                                    \
             IO_FUNC_PORT_PIN(1,4,IO_FUNC_PERIPH); \
         }                                         \
      }                                            \
      else                                         \
      {                                            \
         if(PERCFG & 0x10) {                       \
            IO_FUNC_PORT_PIN(2,3,IO_FUNC_PERIPH);  \
         }                                         \
        else {                                     \
           IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH);   \
        }                                          \
      } )

//Macro for setting the clock tick for timer3 or 4
#define  TIMER34_START(timer)  (T##timer##CTL = T##timer##CTL | 0X10)

#define  TIMER34_STOP(timer)   (T##timer##CTL &= ~0X10)

#define TIMER34_SET_CLOCK_DIVIDE(timer,val)                       \
        st(                                                       \
              T##timer##CTL &= ~0XE0;                             \
              (val==2) ? (T##timer##CTL|=0X20):                   \
              (val==4) ? (T##timer##CTL|=0x40):                   \
              (val==8) ? (T##timer##CTL|=0X60):                   \
              (val==16)? (T##timer##CTL|=0x80):                   \
              (val==32)? (T##timer##CTL|=0xa0):                   \
              (val==64) ? (T##timer##CTL|=0xc0):                  \
              (val==128) ? (T##timer##CTL|=0XE0):                 \
              (T##timer##CTL|=0X00);             /* 1 */          \
           )

//Macro for setting the mode of timer3 or 4
#define  TIMER34_SET_MODE(timer,val)                              \
         st(                                                      \
            T##timer##CTL &= ~0X03;                               \
            (val==1)?(T##timer##CTL|=0X01):  /*DOWN        */     \
            (val==2)?(T##timer##CTL|=0X02):  /*Modulo      */     \
            (val==3)?(T##timer##CTL|=0X03):  /*UP / DOWN   */     \
            (T##timer##CTL|=0X00);           /*free runing */     \
            )

#define  HAL_MOTOR_PWMA_T1CFG      0x40
#define  HAL_T1MODE_FREE_RUN       0x01
#define  HAL_T1_COMPARE_MODE       0x04
#define  HAL_T1_OUTPUT_CLEAR_MODE  0x20
       
#define  HAL_T3_SET()              st( T3CTL |= 0xA0 + 0x06; \
                                       T3CCTL0 = 0;          \
                                       T3CCTL1 = 0;          \
                                       T3CC0 = 100;          \
                                       T3CTL |= 0x10;        \
                                       T3IE = 1; )

//*****************************************************************************/
//调速
//#define  HAL_MOTOR_PWMA_SPEED(x)     st( T1CC0H = 128;   \
                                         T1CC0L = 0; )
#define  HAL_MOTOR_PWMA_SPEED(x)   st( T1CC0H = (uint8)(((uint16)x >> 8) & 0xFF);   \
                                       T1CC0L = (uint8)((uint16)x & 0xFF); )

#define  Ka  210
#define  Kb  -100
#define  Kc  10
//*****************************************************************************/
// 本地变量

// 电机的方向状态
static uint8 motorStatus;

// 电机的当前速度
static uint8 motorCurrentSpeed;

// 电机的测速脉冲数
static uint16 motorPulseCnt1;
static uint16 motorPulseCnt2;

// 电机的预置速度
static uint8 motorSetSpeed;

//*****************************************************************************/
//内部函数
/*****************************************
//函数声明
*****************************************/
void Timer3Init(void);
void Timer1Init(void);

//*****************************************************************************/
//函数名：Timer3Init
//功  能：timer3初始化
//参  数：无
//返回值：无
//*****************************************************************************/
void Timer1Init(void)
{
  T1CTL |= HAL_T1MODE_FREE_RUN;
  T1CCTL0 |= HAL_T1_COMPARE_MODE + HAL_T1_OUTPUT_CLEAR_MODE;
}

//*****************************************************************************/
//函数名：Timer3Init
//功  能：timer3初始化
//参  数：无
//返回值：无
//*****************************************************************************/
void Timer3Init(void)
{
  TIMER34_INIT(3);                    //初始化T4
  TIMER34_ENABLE_OVERFLOW_INT(3, 1);  //开T4中断
  EA = 1;
  T3IE = 1;
  
  TIMER34_SET_CLOCK_DIVIDE(3, 32);
  TIMER34_SET_MODE(3, 1);                 
  T3CC0 = 100;
  TIMER34_START(3);                    //启动
}

//*****************************************************************************/
//函数名：P1_ISR
//功  能：P1口中断函数，直流电机测速，P1_3、P1_4两个IO口中断
//参  数：无
//返回值：无
//*****************************************************************************/
#pragma vector = P1INT_VECTOR
__interrupt void P1_ISR(void)
{
  if (P1IFG & BV(3))         
  {
    P1IFG = 0;
    motorPulseCnt1++;
  }
  if (P1IFG & BV(4))         
  {
    P1IFG = 0;
    motorPulseCnt2++;
  }
  P1IF = 0;          //清中断标志
}

//*****************************************************************************/
//函数名：T3_ISR
//功  能：Timer3中断，中断100us产生一次，时钟节拍1s：测转速，调整转速;
//参  数：无
//返回值：无
//*****************************************************************************/
#pragma vector = T3_VECTOR
__interrupt void T3_ISR(void)
{
  static uint16 timerCnt;
  static int16  Ek, Ek1, Ek2; 
  int16 speedDiff;
  uint16 tmpT1CC0H, tmpT1CC0L;
  int32 speedToT1CC0;
  
  // 此处的计数值决定了测速和控制的节拍，2500表示1/4秒测一次速，调一次速；
  // 10000表示1秒测一次速，调一次速；
  if (timerCnt < 2500)
  {
    timerCnt++;
  }
  else
  {
    timerCnt = 0;
    
    // 此处的脉冲数与测速的周期有关，如果1秒测速一次，那么此处/4;
    // 如果1/4秒测速一次，那么正好等于转速；
    motorCurrentSpeed = (motorPulseCnt1 + motorPulseCnt2)/2;
    motorPulseCnt1 = 0;
    motorPulseCnt2 = 0;

    tmpT1CC0H = T1CC0H;
    tmpT1CC0L = T1CC0L;
    speedToT1CC0 = tmpT1CC0H << 8 | tmpT1CC0L;

    if (motorSetSpeed == 0)
    {
      HAL_MOTOR_PWMA_SPEED(32768);
    }
    else 
    {
      speedDiff = motorSetSpeed - motorCurrentSpeed;
      Ek2 = Ek1;
      Ek1 = Ek;
      Ek = speedDiff;
      if (motorStatus == HAL_MOTOR_FORWORD)
      {                
        speedToT1CC0 += Ka * Ek + Kb * Ek1 + Kc * Ek2;
         
        // 电机正转，确保PWM波占空比大于50%;
        if (speedToT1CC0 < 32768)
        {
          speedToT1CC0 = 32768;        
        } 
        // 防止上溢出
        if (speedToT1CC0 > 0xFFFF)
        {
          speedToT1CC0 = 0xFFFF;
        }
        HAL_MOTOR_PWMA_SPEED(speedToT1CC0);
      }            
      else if (motorStatus == HAL_MOTOR_BACKWORD)
      {                
        speedToT1CC0 -= Ka * Ek + Kb * Ek1 + Kc * Ek2;
        
        // 反转，PWM占空比小于50%
        if (speedToT1CC0 > 32768)
        {
          speedToT1CC0 = 32768;        
        } 
        // 防止下溢出
        if (speedToT1CC0 < 1)
        {
          speedToT1CC0 = 1;
        }
        HAL_MOTOR_PWMA_SPEED(speedToT1CC0);
      }
    }
  }
  
  TIMER34_START(3); // 打开Timer3启动
}


//*****************************************************************************/
//*****************************************************************************/
//外部函数
//*****************************************************************************/
//函数名：HalMotorInit
//功  能：直流电机初始化配置
//参  数：无
//返回值：无
//*****************************************************************************/
void HalMotorInit(void)
{
  PERCFG |= HAL_MOTOR_PWMA_T1CFG;
  HAL_MOTOR_PWMA_PIN_SET(); 
  Timer1Init();                   // T1定时器用来输出PWM波
  HalMotorCtl(HAL_MOTOR_STOP, 0); // 电机初始化状态：停机
  HAL_MOTOR_SPEED_PIN_SET();
  Timer3Init();                   // T3定时器用来产生系统控制周期定时
  EA = 1;
}

//*****************************************************************************/
//函数名：HalMotorCtl
//功  能：直流电机上层控制;
//参  数：dir  : 方向，停止、前进、后退；
//        speed: 速度，在停止状态下，速度设置无效，默认设置为0；
//返回值：无
//*****************************************************************************/
void HalMotorCtl(uint8 dir, uint8 speed)
{
  switch(dir)
  {
    case HAL_MOTOR_STOP:
    {
      // 忽略函数的速度设置，停止状态速度为0
      motorSetSpeed = 0;
      break;
    }
    case HAL_MOTOR_FORWORD:
    {
      motorSetSpeed = speed;
      break;
    }
    case HAL_MOTOR_BACKWORD:
    {
      motorSetSpeed = speed;
      break;
    }
    default:
    {
      motorSetSpeed = 0;
      break;
    }
  }
  motorStatus = dir;
}

//*****************************************************************************/
//函数名：HalMotorStatusCheck
//功  能：返回当前电机的方向状态；
//参  数：无
//返回值：返回电机的状态信息；
//*****************************************************************************/
uint8 HalMotorStatusCheck(void)
{
  return motorStatus;
}

//*****************************************************************************/
//函数名：HalMotorSpeedCheck
//功  能：返回电机当前的速度；
//参  数：无
//返回值：电机当前的速度，单位：转/秒；
//*****************************************************************************/
uint8 HalMotorSpeedCheck(void)
{
  return ((uint8)motorCurrentSpeed);
}

//*****************************************************************************/
//函数名：HalTimerTest
//功  能：测试用途，为了检测定时器配置，T1CC0与PWM波的占空比有关，直观反映了转速的关系
//参  数：无
//返回值：16位的T1CC0寄存器数值
//*****************************************************************************/
uint16 HalTimerTest()
{
  uint16 tmpT1CC0H, tmpT1CC0L;

  tmpT1CC0H = T1CC0H;
  tmpT1CC0L = T1CC0L;
  return (tmpT1CC0H << 8 | tmpT1CC0L);
}

//*****************************************************************************/
//*****************************************************************************/