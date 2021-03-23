/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2010-09-15 19:02:45 -0700 (Wed, 15 Sep 2010) $
  Revision:       $Revision: 23815 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_board.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1

#define HAL_KEY_DEBOUNCE_VALUE  25

/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_1_IF P1IF

/* SW_7(CANCEL) is at P1.3 */
#define HAL_KEY_SW_7_PORT   P1
#define HAL_KEY_SW_7_BIT    BV(3)
#define HAL_KEY_SW_7_SEL    P1SEL
#define HAL_KEY_SW_7_DIR    P1DIR

/* edge interrupt */
#define HAL_KEY_SW_7_EDGEBIT  BV(1) /* P1.0 - P1.3�жϱ��ؿ���λ */
#define HAL_KEY_SW_7_EDGE     HAL_KEY_FALLING_EDGE

/* SW_7 interrupts */
#define HAL_KEY_SW_7_IEN      IEN2  /* P1���ж����μĴ��� */
#define HAL_KEY_SW_7_IENBIT   BV(4) /* ���P1�ڵ��ж����ο���λ */
#define HAL_KEY_SW_7_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_7_ICTLBIT  BV(3) /* P1IEN - P1.3 enable/disable bit */
#define HAL_KEY_SW_7_PXIFG    P1IFG /* Interrupt flag at source */

/* SW_5(OK) is at P1.2 */
#define HAL_KEY_SW_5_PORT   P1
#define HAL_KEY_SW_5_BIT    BV(2) 
#define HAL_KEY_SW_5_SEL    P1SEL
#define HAL_KEY_SW_5_DIR    P1DIR

/* edge interrupt */
#define HAL_KEY_SW_5_EDGEBIT  BV(1) /* P1.0 - P1.3�жϱ��ؿ���λ */
#define HAL_KEY_SW_5_EDGE     HAL_KEY_FALLING_EDGE

/* SW_5 interrupts */
#define HAL_KEY_SW_5_IEN      IEN2  /* P1���ж����μĴ��� */
#define HAL_KEY_SW_5_IENBIT   BV(4) /* ���P1�ڵ��ж����ο���λ */
#define HAL_KEY_SW_5_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_5_ICTLBIT  BV(2) /* P1IEN - P1.2 enable/disable bit */
#define HAL_KEY_SW_5_PXIFG    P1IFG /* Interrupt flag at source */

/* Joy stick move at P0.6 */
#define HAL_KEY_JOY_MOVE_PORT   P0
#define HAL_KEY_JOY_MOVE_BIT    BV(6)
#define HAL_KEY_JOY_MOVE_SEL    P0SEL
#define HAL_KEY_JOY_MOVE_DIR    P0DIR

/* edge interrupt */
#define HAL_KEY_JOY_MOVE_EDGEBIT  BV(0) /* P0.0 - P0.7�жϱ��ؿ���λ */
#define HAL_KEY_JOY_MOVE_EDGE     HAL_KEY_FALLING_EDGE

/* Joy move interrupts */
#define HAL_KEY_JOY_MOVE_IEN      IEN1  /* P0���ж����μĴ��� */
#define HAL_KEY_JOY_MOVE_IENBIT   BV(5) /* ���P0�ڵ��ж����ο���λ */
#define HAL_KEY_JOY_MOVE_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_JOY_MOVE_ICTLBIT  BV(6) /* P0IEN - P0.6 enable/disable bit */
#define HAL_KEY_JOY_MOVE_PXIFG    P0IFG /* Interrupt flag at source */

#define HAL_KEY_JOY_CHN   HAL_ADC_CHANNEL_6


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt(void);
uint8 halGetJoyKeyInput(void);



/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
    /* Initialize previous key to 0 */
    halKeySavedKeys = 0;

    HAL_KEY_SW_5_SEL &= ~(HAL_KEY_SW_5_BIT);
    HAL_KEY_SW_5_DIR &= ~(HAL_KEY_SW_5_BIT);

    HAL_KEY_SW_7_SEL &= ~(HAL_KEY_SW_7_BIT);    /* Set pin function to GPIO */
    HAL_KEY_SW_7_DIR &= ~(HAL_KEY_SW_7_BIT);    /* Set pin direction to Input */

    HAL_KEY_JOY_MOVE_SEL &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin function to GPIO */
    HAL_KEY_JOY_MOVE_DIR &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin direction to Input */

    /* Initialize callback function */
    pHalKeyProcessFunction  = NULL;

    /* Start with key is not configured */
    HalKeyConfigured = FALSE;
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
    /* Enable/Disable Interrupt or */
    Hal_KeyIntEnable = interruptEnable;

    /* Register the callback fucntion */
    pHalKeyProcessFunction = cback;

    /* Determine if interrupt is enable or not */
    if (Hal_KeyIntEnable)
    {
        /* Rising/Falling edge configuratinn */
        PICTL &= ~(HAL_KEY_SW_7_EDGEBIT);    /* Clear the edge bit */
        PICTL &= ~(HAL_KEY_SW_5_EDGEBIT);    /* Clear the edge bit */
        /* For falling edge, the bit must be set. */
        #if (HAL_KEY_SW_7_EDGE == HAL_KEY_FALLING_EDGE)
        PICTL |= HAL_KEY_SW_7_EDGEBIT;
        #endif
        #if (HAL_KEY_SW_5_EDGE == HAL_KEY_FALLING_EDGE)
        PICTL |= HAL_KEY_SW_5_EDGEBIT;
        #endif
        
        /* Interrupt configuration:
         * - Enable interrupt generation at the port
         * - Enable CPU interrupt
         * - Clear any pending interrupt
         */
        HAL_KEY_SW_7_ICTL |= HAL_KEY_SW_7_ICTLBIT;
        HAL_KEY_SW_7_IEN |= HAL_KEY_SW_7_IENBIT;
        HAL_KEY_SW_7_PXIFG = ~(HAL_KEY_SW_7_BIT);
        
        HAL_KEY_SW_5_ICTL |= HAL_KEY_SW_5_ICTLBIT;
        HAL_KEY_SW_5_IEN |= HAL_KEY_SW_5_IENBIT;
        HAL_KEY_SW_5_PXIFG = ~(HAL_KEY_SW_5_BIT);

        /* Do this only after the hal_key is configured - to work with sleep stuff */
        if (HalKeyConfigured == TRUE)
        {
            osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
        }
    }
    else    /* Interrupts NOT enabled */
    {
        HAL_KEY_SW_7_ICTL &= ~(HAL_KEY_SW_7_ICTLBIT); /* don't generate interrupt */
        HAL_KEY_SW_7_IEN &= ~(HAL_KEY_SW_7_IENBIT);   /* Clear interrupt enable bit */
        HAL_KEY_SW_5_ICTL &= ~(HAL_KEY_SW_5_ICTLBIT); /* don't generate interrupt */
        HAL_KEY_SW_5_IEN &= ~(HAL_KEY_SW_5_IENBIT);   /* Clear interrupt enable bit */
        
        osal_set_event(Hal_TaskID, HAL_KEY_EVENT);    /* ���ð����¼� */
    }

    /* Key now is configured */
    HalKeyConfigured = TRUE;
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
    uint8 keys = 0;
    // CANCEL
    if (HAL_PUSH_BUTTON1())
    {
        keys |= HAL_KEY_SW_7;
    }
    // OK
    if (HAL_PUSH_BUTTON2())
    {
        keys |= HAL_KEY_SW_5;
    }

    keys |= halGetJoyKeyInput();

    return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll (void)
{
    uint8 keys = 0;

    keys = halGetJoyKeyInput();

    if (HAL_PUSH_BUTTON1())
    {
        keys |= HAL_KEY_SW_7;
    }
     
    if (HAL_PUSH_BUTTON2())
    {
        keys |= HAL_KEY_SW_5;
    }
  
    /* If interrupts are not enabled, previous key status and current key status
     * are compared to find out if a key has changed status.
     */
    if (!Hal_KeyIntEnable)
    {
        if (keys == halKeySavedKeys)
        {
            /* Exit - since no keys have changed */
            return;
        }
        /* Store the current keys for comparation next time */
        halKeySavedKeys = keys;
    }
    else
    {
        /* Key interrupt handled here */
    }

    /* Invoke Callback if new keys were depressed */
    if (keys && (pHalKeyProcessFunction))
    {
        (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
    }
}

/**************************************************************************************************
 * @fn      halGetJoyKeyInput
 *
 * @brief   Map the ADC value to its corresponding key.
 *
 * @param   None
 *
 * @return  keys - current joy key status
 **************************************************************************************************/
uint8 halGetJoyKeyInput(void)
{
    /* The joystick control is encoded as an analog voltage.
     * Read the JOY_LEVEL analog value and map it to joy movement.
     */
    uint8 adc;
    uint8 ksave0 = 0;
    uint8 ksave1;

    /* Keep on reading the ADC until two consecutive key decisions are the same. */
    do
    {
        ksave1 = ksave0;    /* save previouse key reading */
    
        adc = HalAdcRead (HAL_KEY_JOY_CHN, HAL_ADC_RESOLUTION_8);
        
        if ((adc >= 88) && (adc <= 104))
        {
            ksave0 |= HAL_KEY_UP;
        }
        else if (adc < 10)
        {
            ksave0 |= HAL_KEY_RIGHT;
        }
        else if ((adc >= 24) && (adc <= 46))
        {
            ksave0 |= HAL_KEY_LEFT;
        }
        else if ((adc >= 56) && (adc <= 78))
        {
            ksave0 |= HAL_KEY_DOWN;
        }
        else
        {
        }
        /*
        if ((adc >= 0x58) && (adc <= 0x68))
        {
            ksave0 |= HAL_KEY_UP;
        }
        else if (adc < 0x0A)
        {
            ksave0 |= HAL_KEY_RIGHT;
        }
        else if ((adc >= 0x18) && (adc <= 0x2E))
        {
            ksave0 |= HAL_KEY_LEFT;
        }
        else if ((adc >= 0x38) && (adc <= 0x4E))
        {
            ksave0 |= HAL_KEY_DOWN;
        }
        else
        {
        }*/
        /*
        if ((adc >= 100) && (adc <= 110))
        {
            ksave0 |= HAL_KEY_UP;
        }
        else if (adc < 10)
        {
            ksave0 |= HAL_KEY_RIGHT;
        }
        else if ((adc >= 60) && (adc <= 70))
        {
            ksave0 |= HAL_KEY_LEFT;
        }
        else if ((adc >= 80) && (adc <= 95))
        {
            ksave0 |= HAL_KEY_DOWN;
        }
        else
        {
        }*/
    } while (ksave0 != ksave1);

    return ksave0;
}

/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{
    bool valid=FALSE;

    if (HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT)  /* Interrupt Flag has been set */
    {
        HAL_KEY_SW_7_PXIFG = ~(HAL_KEY_SW_7_BIT); /* Clear Interrupt Flag */
        valid = TRUE;
    }
    
    if (HAL_KEY_SW_5_PXIFG & HAL_KEY_SW_5_BIT)  /* Interrupt Flag has been set */
    {
        HAL_KEY_SW_5_PXIFG = ~(HAL_KEY_SW_5_BIT); /* Clear Interrupt Flag */
        valid = TRUE;
    }

    if (HAL_KEY_JOY_MOVE_PXIFG & HAL_KEY_JOY_MOVE_BIT)  /* Interrupt Flag has been set */
    {
        HAL_KEY_JOY_MOVE_PXIFG = ~(HAL_KEY_JOY_MOVE_BIT); /* Clear Interrupt Flag */
        valid = TRUE;
    }

    if (valid)
    {
        osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
    }
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
    /* Wake up and read keys */
    return ( HalKeyRead () );
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halKeyPort1Isr
 *
 * @brief   Port1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
    HAL_ENTER_ISR();

    if ( (HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT) || (HAL_KEY_SW_5_PXIFG & HAL_KEY_SW_5_BIT) )
    {
        halProcessKeyInterrupt();
    }

    /*
      Clear the CPU interrupt flag for Port_1
      PxIFG has to be cleared before PxIF
    */
    HAL_KEY_SW_7_PXIFG = 0;
    HAL_KEY_SW_5_PXIFG = 0;
    HAL_KEY_CPU_PORT_1_IF = 0;

    CLEAR_SLEEP_MODE();
    HAL_EXIT_ISR();
}


/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
    HAL_ENTER_ISR();

    if (HAL_KEY_JOY_MOVE_PXIFG & HAL_KEY_JOY_MOVE_BIT)
    {
        halProcessKeyInterrupt();
    }

    /*
      Clear the CPU interrupt flag for Port_0
      PxIFG has to be cleared before PxIF
    */
    HAL_KEY_JOY_MOVE_PXIFG = 0;
    HAL_KEY_CPU_PORT_0_IF = 0;

    CLEAR_SLEEP_MODE();
    HAL_EXIT_ISR();
}

#else


void HalKeyInit(void){}
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback){}
uint8 HalKeyRead(void){ return 0;}
void HalKeyPoll(void){}

#endif /* HAL_KEY */





/**************************************************************************************************
**************************************************************************************************/



