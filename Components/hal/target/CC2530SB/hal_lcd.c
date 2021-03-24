/****************************************************************************************
  Filename:       hal_lcd.c
  Revised:        $Date: 2019-03-16 $
  
  Description:    
  This file contains the interface to the HAL LCD Service.


  Copyright 2007-2019 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/
//#include <ioCC2530.h>
#include "hal_types.h"
#include "hal_lcd.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_assert.h"
#include "hal_defs.h"
#include "font.h"

#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
    #include "DebugTrace.h"
#endif

/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/
/*
    LCD pins
    // control
    P1.1 - LCD_BACKLIGHT
    P2.0 - LCD_RESET
    
    // spi
    P1.7 - LCD_CS
    P1.5 - RS
    P1.4 - SCLK
    P1.6 - SDA
*/

/* LCD Control lines */
// control
#define HAL_LCD_BACKLIGHT_PORT 1
#define HAL_LCD_BACKLIGHT_PIN  1

#define HAL_LCD_RESET_PORT 2
#define HAL_LCD_RESET_PIN  0

// spi
#define HAL_LCD_CS_PORT 1
#define HAL_LCD_CS_PIN  7

#define HAL_LCD_RS_PORT 1
#define HAL_LCD_RS_PIN  5

#define HAL_LCD_SCLK_PORT 1
#define HAL_LCD_SCLK_PIN  4

#define HAL_LCD_SDA_PORT 1
#define HAL_LCD_SDA_PIN  6

/* LCD Parameters */
#define LCD_MAX_LINE_COUNT              4
#define LCD_MAX_LINE_LENGTH             16
#define LCD_MAX_BUF                     25

/* Defines for HW LCD */

/* Bias control */
#define BIAS_1_7                        0x01
#define BIAS_1_9                        0x00
#define SET_BIAS_CTRL(bias)             HalLcd_HW_Control(0xA2 | (bias))

/* Power control */
#define LCD_POWER_CONTROL_1             0x04
#define LCD_POWER_CONTROL_2             0x06
#define LCD_POWER_CONTROL_3             0x07
#define LCD_POWER_CONTROL(options)      HalLcd_HW_Control(0x28 + options)

/* LCD commands */
#define LCD_SOFT_RESET                  0xE2
#define LCD_CMD_SET_CONTRAST            0x20


/**************************************************************************************************
 *                                           MACROS
 **************************************************************************************************/

#define HAL_IO_SET(port, pin, val)                HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)           st( P##port##_##pin = val; )

#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin = val; \
                                                      P##port##DIR |= BV(pin); )

#define HAL_CONFIG_IO_PERIPHERAL(port, pin)       HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)  st( P##port##SEL |= BV(pin); )


/* Control macros */
#define LCD_BACKLIGHT_OFF()     HAL_IO_SET(HAL_LCD_BACKLIGHT_PORT, HAL_LCD_BACKLIGHT_PIN, 1);
#define LCD_BACKLIGHT_ON()      HAL_IO_SET(HAL_LCD_BACKLIGHT_PORT, HAL_LCD_BACKLIGHT_PIN, 0);

#define LCD_SCLK_H              HAL_IO_SET(HAL_LCD_SCLK_PORT, HAL_LCD_SCLK_PIN, 1);
#define LCD_SCLK_L              HAL_IO_SET(HAL_LCD_SCLK_PORT, HAL_LCD_SCLK_PIN, 0);

#define LCD_SDA_H               HAL_IO_SET(HAL_LCD_SDA_PORT, HAL_LCD_SDA_PIN, 1);
#define LCD_SDA_L               HAL_IO_SET(HAL_LCD_SDA_PORT, HAL_LCD_SDA_PIN, 0);

#define LCD_ACTIVATE_CS()       HAL_IO_SET(HAL_LCD_CS_PORT, HAL_LCD_CS_PIN, 0);
#define LCD_RELEASE_CS()        HAL_IO_SET(HAL_LCD_CS_PORT, HAL_LCD_CS_PIN, 1);

#define LCD_RS_CMD()            HAL_IO_SET(HAL_LCD_RS_PORT, HAL_LCD_RS_PIN, 0);
#define LCD_RS_DATA()           HAL_IO_SET(HAL_LCD_RS_PORT, HAL_LCD_RS_PIN, 1);

#define LCD_ACTIVATE_RESET()    HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 0);//低电平复位
#define LCD_RELEASE_RESET()     HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);

#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                       LOCAL VARIABLES
 **************************************************************************************************/

static uint8 *Lcd_Line1;

/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/

void HalLcd_HW_Init(void);
void HalLcd_HW_WaitUs(uint16 i);
void HalLcd_HW_SET_DDRAM_ADDR(uint8 page, uint8 column);
void HalLcd_HW_Clear(void);
void HalLcd_HW_Control(uint8 cmd);
void HalLcd_HW_Write(uint8 data);
void HalLcd_HW_SetContrast(uint8 value);
void HalLcd_HW_SetVoltage(uint8 value);
void HalLcd_HW_WriteChar(uint8 line, uint8 col, char text);
void HalLcd_HW_WriteLine(uint8 line, const char *pText);
#endif //LCD

/**************************************************************************************************
 * @fn      HalLcdInit
 *
 * @brief   Initilize LCD Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 **************************************************************************************************/
void HalLcdInit(void)
{
#if (HAL_LCD == TRUE)
    Lcd_Line1 = NULL;
    HalLcd_HW_Init();
#endif
}

/*************************************************************************************************
 *                    LCD EMULATION FUNCTIONS
 *
 * Some evaluation boards are equipped with Liquid Crystal Displays
 * (LCD) which may be used to display diagnostic information. These
 * functions provide LCD emulation, sending the diagnostic strings
 * to Z-Tool via the RS232 serial port. These functions are enabled
 * when the "LCD_SUPPORTED" compiler flag is placed in the makefile.
 *
 * Most applications update both lines (1 and 2) of the LCD whenever
 * text is posted to the device. This emulator assumes that line 1 is
 * updated first (saved locally) and the formatting and send operation
 * is triggered by receipt of line 2. Nothing will be transmitted if
 * only line 1 is updated.
 *
 *************************************************************************************************/

/**************************************************************************************************
 * @fn      HalLcdWriteString
 *
 * @brief   Write a string to the LCD
 *
 * @param   str    - pointer to the string that will be displayed
 *          option - display options
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteString ( char *str, uint8 option)
{
#if (HAL_LCD == TRUE)
    uint8 strLen = 0;
    uint8 totalLen = 0;
    uint8 *buf;
    uint8 tmpLen;
  
    if ( Lcd_Line1 == NULL )
    {
        Lcd_Line1 = osal_mem_alloc( HAL_LCD_MAX_CHARS+1 );
        HalLcdWriteString( "TexasInstruments", 1 );
    }
  
    strLen = (uint8)osal_strlen( (char*)str );
  
    /* Check boundries */
    if ( strLen > HAL_LCD_MAX_CHARS )
        strLen = HAL_LCD_MAX_CHARS;
  
    if ( option == HAL_LCD_LINE_1 )
    {
        /* Line 1 gets saved for later */
        osal_memcpy( Lcd_Line1, str, strLen );
        Lcd_Line1[strLen] = '\0';
    }
    else
    {
        /* Line 2 triggers action */
        tmpLen = (uint8)osal_strlen( (char*)Lcd_Line1 );
        totalLen =  tmpLen + 1 + strLen + 1;
        buf = osal_mem_alloc( totalLen );
        if ( buf != NULL )
        {
            /* Concatenate strings */
            osal_memcpy( buf, Lcd_Line1, tmpLen );
            buf[tmpLen++] = ' ';
            osal_memcpy( &buf[tmpLen], str, strLen );
            buf[tmpLen+strLen] = '\0';
  
            /* Send it out */
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
#if defined(SERIAL_DEBUG_SUPPORTED)
            debug_str( (uint8*)buf );
#endif //LCD_SUPPORTED
#endif //ZTOOL_P1
            /* Free mem */
            osal_mem_free( buf );
        }
    }
  
    /* Display the string */
    HalLcd_HW_WriteLine (option, str);
#endif //HAL_LCD
}

/**************************************************************************************************
 * @fn      HalLcdWriteValue
 *
 * @brief   Write a value to the LCD
 *
 * @param   value  - value that will be displayed
 *          radix  - 8, 10, 16(进制)
 *          option - display options
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option)
{
#if (HAL_LCD == TRUE)
    uint8 buf[LCD_MAX_BUF];

    // 将value按照radix的进制转换存到buf中
    _ltoa( value, &buf[0], radix ); 
    HalLcdWriteString( (char*)buf, option );
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteScreen
 *
 * @brief   Write a value to the LCD
 *
 * @param   line1  - string that will be displayed on line 1
 *          line2  - string that will be displayed on line 2
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteScreen( char *line1, char *line2 )
{
#if (HAL_LCD == TRUE)
    HalLcdWriteString( line1, 1 );
    HalLcdWriteString( line2, 2 );
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteStringValue
 *
 * @brief   Write a string followed by a value to the LCD
 *
 * @param   title  - Title that will be displayed before the value
 *          value  - value
 *          format - redix
 *          line   - line number
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
#if (HAL_LCD == TRUE)
    uint8 tmpLen;
    uint8 buf[LCD_MAX_BUF];
    uint32 err;
  
    tmpLen = (uint8)osal_strlen( (char*)title );
    osal_memcpy( buf, title, tmpLen );
    buf[tmpLen] = ' ';
    err = (uint32)(value);
    _ltoa( err, &buf[tmpLen+1], format );
    HalLcdWriteString( (char*)buf, line );		
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteStringValueValue
 *
 * @brief   Write a string followed by a value to the LCD
 *
 * @param   title   - Title that will be displayed before the value
 *          value1  - value #1
 *          format1 - redix of value #1
 *          value2  - value #2
 *          format2 - redix of value #2
 *          line    - line number
 *
 * @return  None
 * 修改为显示两个title + 2个Value
 **************************************************************************************************/
void HalLcdWriteStringValueValue( char *title1, uint16 value1, uint8 format1,
                                  char *title2,uint16 value2, uint8 format2, uint8 line )
{
#if (HAL_LCD == TRUE)
    uint8 tmpLen;
    uint8 buf[LCD_MAX_BUF];
    uint32 err;
  
    tmpLen = (uint8)osal_strlen( (char*)title1 );
    if ( tmpLen )
    {
        osal_memcpy( buf, title1, tmpLen );
    }
  
    err = (uint32)(value1);
    _ltoa( err, &buf[tmpLen], format1 );
    tmpLen = (uint8)osal_strlen( (char*)buf );
  
    tmpLen += (uint8)osal_strlen( (char*)title2 );
    if ( tmpLen )
    {
        strcat( buf, title2);
    }
    
    err = (uint32)(value2);
    _ltoa( err, &buf[tmpLen], format2 );
  
    HalLcdWriteString( (char *)buf, line );		
#endif
}

/**************************************************************************************************
 * @fn      HalLcdDisplayPercentBar
 *
 * @brief   Display percentage bar on the LCD
 *
 * @param   title   -
 *          value   -
 *
 * @return  None
 **************************************************************************************************/
void HalLcdDisplayPercentBar( char *title, uint8 value )
{
#if (HAL_LCD == TRUE)
    uint8 percent;
    uint8 leftOver;
    uint8 buf[17];
    uint32 err;
    uint8 x;
  
    /* Write the title: */
    HalLcdWriteString( title, HAL_LCD_LINE_1 );
  
    if ( value > 100 )
      value = 100;
  
    /* convert to blocks */
    percent = (uint8)(value / 10);
    leftOver = (uint8)(value % 10);
  
    /* Make window */
    osal_memcpy( buf, "[          ]  ", 15 );
  
    for ( x = 0; x < percent; x ++ )
    {
        buf[1+x] = '>';
    }
  
    if ( leftOver >= 5 )
      buf[1+x] = '+';
  
    err = (uint32)value;
    _ltoa( err, (uint8*)&buf[13], 10 );
  
    HalLcdWriteString( (char*)buf, HAL_LCD_LINE_2 );
#endif
}                     


#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                    HARDWARE LCD
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      halLcd_ConfigIO
 *
 * @brief   Configure IO lines needed for LCD control.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
static void halLcd_ConfigIO(void)
{
    /* GPIO configuration */
    HAL_CONFIG_IO_OUTPUT(HAL_LCD_BACKLIGHT_PORT, HAL_LCD_BACKLIGHT_PIN, 1);
    HAL_CONFIG_IO_OUTPUT(HAL_LCD_RESET_PORT,     HAL_LCD_RESET_PIN,     1);
    HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT,        HAL_LCD_CS_PIN,        1);
    HAL_CONFIG_IO_OUTPUT(HAL_LCD_SCLK_PORT,      HAL_LCD_SCLK_PIN,      1);
    HAL_CONFIG_IO_OUTPUT(HAL_LCD_RS_PORT,        HAL_LCD_RS_PIN,        1);
    HAL_CONFIG_IO_OUTPUT(HAL_LCD_SDA_PORT,       HAL_LCD_SDA_PIN,       1);
}

/**************************************************************************************************
 * @fn      HalLcd_HW_SET_DDRAM_ADDR
 *
 * @brief   Set DDRAM address, 一共128列，8页(每页8行)
 *
 * @param   uint8 page, uint8 column
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_SET_DDRAM_ADDR(uint8 page, uint8 column)
{
    LCD_ACTIVATE_CS();
    page   -= 1; 
    column -= 1;
    HalLcd_HW_Control(0xb0 + page);
    HalLcd_HW_Control(((column >> 4) & 0x0F) + 0x10);
    HalLcd_HW_Control(column & 0x0F);
    // LCD_RELEASE_CS();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Control
 *
 * @brief   Write 1 command to the LCD
 *
 * @param   uint8 cmd - command to be written to the LCD
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Control(uint8 cmd)
{
    uint8 i;
    
    LCD_ACTIVATE_CS();
    LCD_RS_CMD();
    
    for (i = 0; i < 8; i++) 
    { 
        LCD_SCLK_L; 
        if (cmd & 0x80) 
        {
            LCD_SDA_H;
        }
        else 
        {
            LCD_SDA_L;
        }
        
        LCD_SCLK_H; 
        cmd <<= 1; 
    }
    // LCD_RELEASE_CS();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Write
 *
 * @brief   Write 1 byte to the LCD
 *
 * @param   uint8 data - data to be written to the LCD
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Write(uint8 data)
{
    uint8 i; 
    
    LCD_ACTIVATE_CS();
    LCD_RS_DATA(); 
    
    for (i = 0; i < 8; i++) 
    { 
        LCD_SCLK_L; 
        if (data & 0x80) 
        {
            LCD_SDA_H;
        }
        else 
        {
            LCD_SDA_L;
        }
        
        LCD_SCLK_H; 
        data <<= 1; 
    }
    // LCD_RELEASE_CS();
}

/**************************************************************************************************
 * @fn          HalLcd_HW_SetContrast
 *
 * @brief       Set display contrast
 *
 * @param       uint8 value - contrast value: 0~7;
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_SetContrast(uint8 value)
{
    if (value <= 7)
    {
        // 粗调对比度，可设置范围 0x20～0x27
        HalLcd_HW_Control(LCD_CMD_SET_CONTRAST + value);  
    }
    else 
    {
        // 输入数值溢出，默认3  
        HalLcd_HW_Control(LCD_CMD_SET_CONTRAST + 3); 
    }
}

/**************************************************************************************************
 * @fn          HalLcd_HW_SetVoltage
 *
 * @brief       Set LCD Voltage
 *
 * @param       uint8 value - voltage value: 0x00~0x3F;
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_SetVoltage(uint8 value)
{
    HalLcd_HW_Control(0x81); 
    if (value <= 0x3F)
    {
        // 0x1a,微调对比度的值，可设置范围 0x00～0x3f   
        HalLcd_HW_Control(value);  
    }
    else
    {
        // 数据溢出，默认0x28;  
        HalLcd_HW_Control(0x28);   
    }
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Clear
 *
 * @brief   Clear the HW LCD
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Clear(void)
{
    LCD_ACTIVATE_CS();
    uint8 i,j;
    
    for (i = 0; i < 9; i++) 
    { 
        HalLcd_HW_SET_DDRAM_ADDR((i + 1), 1); // 第一页，第一列
        for( j = 0; j < 132; j++) 
        { 
            HalLcd_HW_Write(0x00); 
        } 
    }
    // LCD_RELEASE_CS();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Init
 *
 * @brief   Initilize HW LCD Driver.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Init(void)
{
    /* Initialize LCD IO lines */
    halLcd_ConfigIO();
    
    LCD_ACTIVATE_CS();
    
    LCD_BACKLIGHT_ON(); // 打开LCD背光灯
  
    /* Perform reset */
    LCD_ACTIVATE_RESET();
    HalLcd_HW_WaitUs(1500); // >1ms
    LCD_RELEASE_RESET();
    HalLcd_HW_WaitUs(5000); // >5ms
  
    HalLcd_HW_Control(LCD_SOFT_RESET);
    HalLcd_HW_WaitUs(500);  

    /* Perform the Power initialization sequence */
    LCD_POWER_CONTROL(LCD_POWER_CONTROL_1);
    HalLcd_HW_WaitUs(500);   
    LCD_POWER_CONTROL(LCD_POWER_CONTROL_2);
    HalLcd_HW_WaitUs(500); 
    LCD_POWER_CONTROL(LCD_POWER_CONTROL_3);  
    HalLcd_HW_WaitUs(500); 
   
    /* Set contrast 对比度*/
    HalLcd_HW_SetContrast(3);

    /* Set power */
    //内部电阻微调
    HalLcd_HW_SetVoltage(0x28);
    
    SET_BIAS_CTRL(BIAS_1_9);
    HalLcd_HW_WaitUs(1000);  // 1 ms
    
    HalLcd_HW_Control(0xc8); // 行扫描顺序：从上到下 
    HalLcd_HW_Control(0xa0); // 列扫描顺序：从左到右 
    HalLcd_HW_Control(0x40); // 起始行：第一行开始
    HalLcd_HW_Control(0xaf); // 打开显示 

    /* Clear the display */
    HalLcd_HW_Clear();

    LCD_RELEASE_CS();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_WriteChar
 *
 * @brief   Write one char to the display
 *
 * @param   uint8 line - line number that the char will be displayed
 *          uint8 col - colum where the char will be displayed
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_WriteChar(uint8 line, uint8 col, char text)
{
    uint8 j, k, n;
    
    if (col <= LCD_MAX_LINE_LENGTH)
    {   
        //字符大小：16(行)x8(列)
        if ((text >= 0x20) && (text <= 0x7e)) 
        { 
            j = text - 0x20; 
            for (n = 0; n < 2; n++) // 两页
            {    
                HalLcd_HW_SET_DDRAM_ADDR((line * 2 - 1 + n), ((col - 1) * 8 + 1)); // 页和列 
                for (k = 0; k < 8; k++) 
                { 
                    // j为字符对应的，
                    // 写数据到 LCD, 每写一次(即写一个 8 位的数据)后列地址自动加 1
                    HalLcd_HW_Write(ascii_table_8x16[j][k + 8 * n]);
                } 
            }  
        } 
    }
    else
    {
        return;
    }
}

/**************************************************************************************************
 * @fn          halLcdWriteLine
 *
 * @brief       Write one line on display
 *
 * @param       uint8 line - display line
 *              char *pText - text buffer to write
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_WriteLine(uint8 line, const char *pText)
{
    uint8 count;
    uint8 totalLength = (uint8)osal_strlen( (char *)pText );
  
    /* Write the content first */
    for (count = 0; count < totalLength; count++)
    {
        HalLcd_HW_WriteChar(line, (count + 1), (*(pText++)));
    }
  
    /* Write blank spaces to rest of the line */
    for (count = totalLength; count < LCD_MAX_LINE_LENGTH; count++)
    {
       HalLcd_HW_WriteChar(line, (count + 1), ' ');
    }
}

/**************************************************************************************************
 * @fn      HalLcd_HW_WaitUs
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_WaitUs(uint16 microSecs)
{
    while(microSecs--)
    {
        // 32 NOPs == 1 usecs
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop");
    }
}
#endif
