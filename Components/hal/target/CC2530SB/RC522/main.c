
#include <ioCC2530.h>
#include "Global.h"
#include "UART.h"
#include "PIN_DEF.H"
#include "RC522.H"
#include <ctype.h> 

uchar UID[7], Temp[4];
uchar RF_Buffer[18];
uchar Password_Buffer[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Mifare One 缺省密码

uchar MBRX[30];
uchar MBKeyTP[30];
uchar Event;
uchar DISP_MODE, i; // 编辑控件显示模式
uchar des_on = 0; // DES加密标志
void Key_TP_Task(void);

//***************************************************************************//
//                                                                           //
//                 初始化主时钟: MCLK = XT1×(FLL_FACTOR+1)                  //
//                                                                           //
//***************************************************************************//
void Init_CLK(void)
{
    CLKCONCMD &= ~0x40;             // 晶振
    while(!(SLEEPSTA & 0x40));      // 等待晶振稳定
    CLKCONCMD = 0x00;                // 32MHz,TICHSPD不分频，CLKSPD不分频
    SLEEPCMD |= 0x04; 		    // 关闭不用的RC振荡器
}

void Delay(unsigned int time)
{
    unsigned int i,k;
    for(i = 0; i < 255; i++)
    {
        for(k = 0; k < time; k++);
    }
}

void Auto_Reader(void)
{
    while(1)
    {
        
        if(PcdRequest(0x52,Temp) == MI_OK)
        {
            if(Temp[0] == 0x04 && Temp[1] == 0x00)  
            {
                PutString("MFOne-S50\r\n");
                if(PcdAnticoll(UID) == MI_OK)
                { 
                    PutString("Card Id is:");
                    tochar(UID[0]);
                    tochar(UID[1]);
                    tochar(UID[2]);
                    tochar(UID[3]);
                    PutString("\r\n");
                    RED_LED_ON ;
                    Delay(4000);
                    RED_LED_OFF;
                    Delay(4000);
                } 
            }
            else if(Temp[0] == 0x02 && Temp[1] == 0x00)
            {
                PutString("MFOne-S70\r\n");
                if(PcdAnticoll(UID) == MI_OK)
                { 
                    PutString("Card Id is:");
                    tochar(UID[0]);
                    tochar(UID[1]);
                    tochar(UID[2]);
                    tochar(UID[3]);
                    PutString("\r\n");
                    RED_LED_ON ;
                    Delay(4000);
                    RED_LED_OFF;
                    Delay(4000);
                } 
            }
            else if(Temp[0] == 0x44 && Temp[1] == 0x00)
            {
                PutString("MF-UltraLight\r\n");
                if(PcdAnticoll(UID) == MI_OK)
                { 
                    PutString("Card Id is:");
                    tochar(UID[0]);
                    tochar(UID[1]);
                    tochar(UID[2]);
                    tochar(UID[3]);
                    PutString("\r\n");
                    RED_LED_ON ;
                    Delay(4000);
                    RED_LED_OFF;
                    Delay(4000);
                } 
            }
            else if(Temp[0] == 0x08 && Temp[1] == 0x00)
            {
                PutString("MF-Pro\r\n");
                if(PcdAnticoll(UID) == MI_OK)
                { 
                    PutString("Card Id is:");
                    tochar(UID[0]);
                    tochar(UID[1]);
                    tochar(UID[2]);
                    tochar(UID[3]);
                    PutString("\r\n");
                    RED_LED_ON ;
                    Delay(4000);
                    RED_LED_OFF;
                    Delay(4000);
                } 
            }
            else if(Temp[0] == 0x44 && Temp[1] == 0x03)
            {
                PutString("MF DesFire\r\n");
                if(PcdAnticoll(UID) == MI_OK)
                { 
                    PutString("Card Id is:");
                    tochar(UID[0]);
                    tochar(UID[1]);
                    tochar(UID[2]);
                    tochar(UID[3]);
                    PutString("\r\n");
                    RED_LED_ON ;
                    Delay(4000);
                    RED_LED_OFF;
                    Delay(4000);
                } 
            }
            else
              PutString("Unknown\r\n");
          
            
        }
    } 
}

void Find_Card(void)
{
    if (PcdRequest(0x52,Temp) == MI_OK)
    {
        if (Temp[0] == 0x04 && Temp[1] == 0x00)  
            PutString("MFOne-S50\r\n");
        else if (Temp[0] == 0x02 && Temp[1] == 0x00)
            PutString("MFOne-S70\r\n");
        else if (Temp[0] == 0x44 && Temp[1] == 0x00)
            PutString("MF-UltraLight\r\n");
        else if (Temp[0] == 0x08 && Temp[1] == 0x00)
            PutString("MF-Pro\r\n");
        else if (Temp[0] == 0x44 && Temp[1] == 0x03)
            PutString("MF DesFire\r\n");        
        else
            PutString("Unknown\r\n");
        
        PutString("SUCCESS!\r\n");
    }
    else PutString("Failed!\r\n");                                             
}

void Init_Port(void)
{
  P1DIR |= BIT6 + BIT5 + BIT4 + BIT3 + BIT2 + BIT0;
  P1DIR &= ~BIT7;
  P2DIR |= BIT0;
  P1_3 = 1;
  P1_2 = 0;
}
/*******************************************
函数名称：HandleConfigMenu
功    能：处理PC的配置函
参    数：inputvalue--接收到的来自PC机的字符
返回值  ：无
********************************************/
void HandleConfigMenu(uchar inputvalue)
{
    switch(toupper(inputvalue)) 
    {
    case 'A':
              Auto_Reader();
              break;
    case 'F':
              Find_Card();
              break;
    default:
              PutString("Recieve it.But it is not right.\r\n");  
    }
}


void main( void )
{

  Init_CLK();
  Init_Port();
  InitUART();
  PcdReset();//复位RC522
  PcdAntennaOn();//开启天线发射 
  DisplayConfigMenu();
  while(1)
  {
        //Send1Char('>');
        //Send1Char('\n');
        /*i=Get1Char();
        PutString("Recieve it.\r\n");
        HandleConfigMenu(i);*/
        
        Find_Card();
        Delay(6000);
        Auto_Reader();
  }
}
