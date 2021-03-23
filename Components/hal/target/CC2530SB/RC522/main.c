
#include <ioCC2530.h>
#include "Global.h"
#include "UART.h"
#include "PIN_DEF.H"
#include "RC522.H"
#include <ctype.h> 

uchar UID[7], Temp[4];
uchar RF_Buffer[18];
uchar Password_Buffer[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Mifare One ȱʡ����

uchar MBRX[30];
uchar MBKeyTP[30];
uchar Event;
uchar DISP_MODE, i; // �༭�ؼ���ʾģʽ
uchar des_on = 0; // DES���ܱ�־
void Key_TP_Task(void);

//***************************************************************************//
//                                                                           //
//                 ��ʼ����ʱ��: MCLK = XT1��(FLL_FACTOR+1)                  //
//                                                                           //
//***************************************************************************//
void Init_CLK(void)
{
    CLKCONCMD &= ~0x40;             // ����
    while(!(SLEEPSTA & 0x40));      // �ȴ������ȶ�
    CLKCONCMD = 0x00;                // 32MHz,TICHSPD����Ƶ��CLKSPD����Ƶ
    SLEEPCMD |= 0x04; 		    // �رղ��õ�RC����
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
�������ƣ�HandleConfigMenu
��    �ܣ�����PC�����ú�
��    ����inputvalue--���յ�������PC�����ַ�
����ֵ  ����
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
  PcdReset();//��λRC522
  PcdAntennaOn();//�������߷��� 
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
