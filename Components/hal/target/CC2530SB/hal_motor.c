/*******************************************************************************
  �� �� ��: hal_motor.c
  ��    �ߣ��Ͼ���已��е��ӿƼ����޹�˾
  ����ʱ�䣺2013.9.16
  �޸�ʱ�䣺2020.12.22
  ˵    ����
    (1) CC2530��P1_2���PWMA������ֱ���������ת��50%ռ�ձ�ͣ����С��50%��ת��
        ����50%��ת���뽫ֱ����������ϵ�PWM��ѡ�����߶̽���sin����ʵ��ʹ��
        ��·PWM��������ơ�
    (2) ���˳ʱ��ת�����ȹ�P1_3��Ȼ��ͨ��P1_4����ʱ����Ϊ������
    (3) �����ֳ���ʹ����CC2530�Ķ�ʱ��1�Ͷ�ʱ��3���ֱ���������ֱ�������PWM����
        ��1s�Ķ�ʱ�жϣ�1s�Ķ�ʱ�ж���Ϊ����������ת�ٵĲ�����ת�ٵĵ��ڣ�
    (4) �����ת�ٵ�λ��ת/�룻Ŀǰ֧�ֵ�ת�ٵ��ڷ�Χ�ǣ�1~50(Max)ת/�룻
    (5) ������Ʒ�������PID�����е�P���ƣ��������Կ���ʵ�ֻ������ٹ��ܡ�
    (6) ͨ���޸Ķ�ʱ���ļ���ֵ�����Ե��ڵ�����ٺͿ��ƵĽ��ģ����忴��ʱ��
        ���ж�˵���������в��ԱȽϵĽ���������ٺͿ��ƵĽ�����ߺ��ٶȵ�
        ��ȷ�Ȳ��ߣ�֮ǰ��4���������1ת����ߵ�1/4����ĺ���1���������1ת��
        �ڹ۲��з�����Ȼ����Ѹ��(4����1��Ľ���)�ĵ����ٶȣ������ٶ��ȶ��Բ��ߣ�
        �ܽ����Ҫԭ��Ӧ���ǲ���ģ����ȶ������⡣
    (7) ����ģ����Ҫ�ǣ���IO�ж��ڵ�����������Լ���ʱ���ж��ڵ�ת��ת����
        ���پ����С�1RPM�����(ʹ��Ŀǰ1/4��Ĳ��١����ƽ���), ����
        ��0.25RPM�����(ʹ��1��Ĳ��١����ƽ���)��
    (8) ����PID��P = �趨ת�� - ��ǰת�٣�
                 I += p��
                 D = ��ǰת�� - ��һ����ǰת�٣�
                 ��һ����ǰת�� = ��ǰת�٣�
                 Ȼ��ѡ����ʵ�kp��ki��kd
                 ������ = kp*P + ki*I + kd*D��
*******************************************************************************/

#include <iocc2530.h>
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_motor.h"

//�궨��--�����������ٵ��������
#define  HAL_MOTOR_SPEED1          P1_4
#define  HAL_MOTOR_SPEED2          P1_3
#define  HAL_MOTOR_SPEED_PIN_SET() st( P1DIR &= ~(BV(3) + BV(4)); \
                                       P1IEN |= BV(4) + BV(3);    \
                                       IEN2 |= BV(4);             \
                                       P1IFG = 0;)
#define  HAL_MOTOR_PWMA_OUT        P1_2
#define  HAL_MOTOR_PWMA_PIN_SET()  st( P1DIR |= BV(2); P1SEL |= BV(2);)

//*****************************************************************************/
//��ʱ��
/*****************************************
//T3���ö���
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
//����
//#define  HAL_MOTOR_PWMA_SPEED(x)     st( T1CC0H = 128;   \
                                         T1CC0L = 0; )
#define  HAL_MOTOR_PWMA_SPEED(x)   st( T1CC0H = (uint8)(((uint16)x >> 8) & 0xFF);   \
                                       T1CC0L = (uint8)((uint16)x & 0xFF); )

#define  Ka  210
#define  Kb  -100
#define  Kc  10
//*****************************************************************************/
// ���ر���

// ����ķ���״̬
static uint8 motorStatus;

// ����ĵ�ǰ�ٶ�
static uint8 motorCurrentSpeed;

// ����Ĳ���������
static uint16 motorPulseCnt1;
static uint16 motorPulseCnt2;

// �����Ԥ���ٶ�
static uint8 motorSetSpeed;

//*****************************************************************************/
//�ڲ�����
/*****************************************
//��������
*****************************************/
void Timer3Init(void);
void Timer1Init(void);

//*****************************************************************************/
//��������Timer3Init
//��  �ܣ�timer3��ʼ��
//��  ������
//����ֵ����
//*****************************************************************************/
void Timer1Init(void)
{
  T1CTL |= HAL_T1MODE_FREE_RUN;
  T1CCTL0 |= HAL_T1_COMPARE_MODE + HAL_T1_OUTPUT_CLEAR_MODE;
}

//*****************************************************************************/
//��������Timer3Init
//��  �ܣ�timer3��ʼ��
//��  ������
//����ֵ����
//*****************************************************************************/
void Timer3Init(void)
{
  TIMER34_INIT(3);                    //��ʼ��T4
  TIMER34_ENABLE_OVERFLOW_INT(3, 1);  //��T4�ж�
  EA = 1;
  T3IE = 1;
  
  TIMER34_SET_CLOCK_DIVIDE(3, 32);
  TIMER34_SET_MODE(3, 1);                 
  T3CC0 = 100;
  TIMER34_START(3);                    //����
}

//*****************************************************************************/
//��������P1_ISR
//��  �ܣ�P1���жϺ�����ֱ��������٣�P1_3��P1_4����IO���ж�
//��  ������
//����ֵ����
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
  P1IF = 0;          //���жϱ�־
}

//*****************************************************************************/
//��������T3_ISR
//��  �ܣ�Timer3�жϣ��ж�100us����һ�Σ�ʱ�ӽ���1s����ת�٣�����ת��;
//��  ������
//����ֵ����
//*****************************************************************************/
#pragma vector = T3_VECTOR
__interrupt void T3_ISR(void)
{
  static uint16 timerCnt;
  static int16  Ek, Ek1, Ek2; 
  int16 speedDiff;
  uint16 tmpT1CC0H, tmpT1CC0L;
  int32 speedToT1CC0;
  
  // �˴��ļ���ֵ�����˲��ٺͿ��ƵĽ��ģ�2500��ʾ1/4���һ���٣���һ���٣�
  // 10000��ʾ1���һ���٣���һ���٣�
  if (timerCnt < 2500)
  {
    timerCnt++;
  }
  else
  {
    timerCnt = 0;
    
    // �˴�������������ٵ������йأ����1�����һ�Σ���ô�˴�/4;
    // ���1/4�����һ�Σ���ô���õ���ת�٣�
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
         
        // �����ת��ȷ��PWM��ռ�ձȴ���50%;
        if (speedToT1CC0 < 32768)
        {
          speedToT1CC0 = 32768;        
        } 
        // ��ֹ�����
        if (speedToT1CC0 > 0xFFFF)
        {
          speedToT1CC0 = 0xFFFF;
        }
        HAL_MOTOR_PWMA_SPEED(speedToT1CC0);
      }            
      else if (motorStatus == HAL_MOTOR_BACKWORD)
      {                
        speedToT1CC0 -= Ka * Ek + Kb * Ek1 + Kc * Ek2;
        
        // ��ת��PWMռ�ձ�С��50%
        if (speedToT1CC0 > 32768)
        {
          speedToT1CC0 = 32768;        
        } 
        // ��ֹ�����
        if (speedToT1CC0 < 1)
        {
          speedToT1CC0 = 1;
        }
        HAL_MOTOR_PWMA_SPEED(speedToT1CC0);
      }
    }
  }
  
  TIMER34_START(3); // ��Timer3����
}


//*****************************************************************************/
//*****************************************************************************/
//�ⲿ����
//*****************************************************************************/
//��������HalMotorInit
//��  �ܣ�ֱ�������ʼ������
//��  ������
//����ֵ����
//*****************************************************************************/
void HalMotorInit(void)
{
  PERCFG |= HAL_MOTOR_PWMA_T1CFG;
  HAL_MOTOR_PWMA_PIN_SET(); 
  Timer1Init();                   // T1��ʱ���������PWM��
  HalMotorCtl(HAL_MOTOR_STOP, 0); // �����ʼ��״̬��ͣ��
  HAL_MOTOR_SPEED_PIN_SET();
  Timer3Init();                   // T3��ʱ����������ϵͳ�������ڶ�ʱ
  EA = 1;
}

//*****************************************************************************/
//��������HalMotorCtl
//��  �ܣ�ֱ������ϲ����;
//��  ����dir  : ����ֹͣ��ǰ�������ˣ�
//        speed: �ٶȣ���ֹͣ״̬�£��ٶ�������Ч��Ĭ������Ϊ0��
//����ֵ����
//*****************************************************************************/
void HalMotorCtl(uint8 dir, uint8 speed)
{
  switch(dir)
  {
    case HAL_MOTOR_STOP:
    {
      // ���Ժ������ٶ����ã�ֹͣ״̬�ٶ�Ϊ0
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
//��������HalMotorStatusCheck
//��  �ܣ����ص�ǰ����ķ���״̬��
//��  ������
//����ֵ�����ص����״̬��Ϣ��
//*****************************************************************************/
uint8 HalMotorStatusCheck(void)
{
  return motorStatus;
}

//*****************************************************************************/
//��������HalMotorSpeedCheck
//��  �ܣ����ص����ǰ���ٶȣ�
//��  ������
//����ֵ�������ǰ���ٶȣ���λ��ת/�룻
//*****************************************************************************/
uint8 HalMotorSpeedCheck(void)
{
  return ((uint8)motorCurrentSpeed);
}

//*****************************************************************************/
//��������HalTimerTest
//��  �ܣ�������;��Ϊ�˼�ⶨʱ�����ã�T1CC0��PWM����ռ�ձ��йأ�ֱ�۷�ӳ��ת�ٵĹ�ϵ
//��  ������
//����ֵ��16λ��T1CC0�Ĵ�����ֵ
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