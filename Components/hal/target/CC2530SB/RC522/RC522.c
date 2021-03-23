/*******************************************************************
*文 件 名：RC522.c
*作    者：yizedxl
*修改日期：2013.5.23
*******************************************************************/
#include <ioCC2530.h>
#include "RC522.H"
#include "PIN_DEF.H"
#include "Global.h"

//******************************************************************/
//功    能：读RC522寄存器
//参数说明：Address[IN]:寄存器地址
//返    回：读出的值
//******************************************************************/
uchar ReadRawRC(uchar Address)
{
  uchar i, ucAddr;
  uchar ucResult=0;
  NSS522_0;
  SCK522_0;
  ucAddr = ((Address<<1)&0x7E)|0x80;
  for (i = 8; i > 0; i--)
  {
    if ((ucAddr & 0x80) == 0x80)
      SI522_1;
    else
      SI522_0;
    SCK522_1;
    ucAddr <<= 1;
    SCK522_0;
  }
  for (i = 8; i > 0;i--)
  {
    SCK522_1;
    ucResult <<= 1;
    ucResult |= SO522;
    SCK522_0;
  }
  SCK522_0;
  NSS522_1;
  return ucResult;
}

//******************************************************************/
//功    能：写RC522寄存器
//参数说明：Address[IN]:寄存器地址
//          value[IN]:写入的值
//******************************************************************/
void WriteRawRC(uchar Address, uchar value)
{  
   uchar i, ucAddr;
   NSS522_0;
   SCK522_0;
   ucAddr = (Address<<1)&0x7E;
   for (i = 8; i > 0; i--)
   {
     if ((ucAddr & 0x80) == 0x80)
       SI522_1;
     else
       SI522_0;
     SCK522_1;
     ucAddr <<= 1;
     SCK522_0;
   }
   for (i = 8; i > 0; i--)
   {
     if ((value & 0x80) == 0x80)
       SI522_1;
     else
       SI522_0;
     SCK522_1;
     value <<= 1;
     SCK522_0;
   }
   SCK522_0;
   NSS522_1;
}

//******************************************************************/
//功    能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
//******************************************************************/
void SetBitMask(uchar reg,uchar mask)  
{
  char tmp = 0x0;
  tmp = ReadRawRC(reg)| mask;
  WriteRawRC(reg, tmp);  // set bit mask
}

//******************************************************************/
//功    能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
//******************************************************************/
void ClearBitMask(uchar reg,uchar mask)  
{
  char tmp = 0x0;
  tmp = ReadRawRC(reg)&(~mask);
  WriteRawRC(reg, tmp);  // clear bit mask
} 

//******************************************************************/
//功    能：复位RC522
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdReset()
{
  RF_POWER_ON;
  RST522_1;
  Delay(1);
  RST522_0;
  Delay(1);
  RST522_1;
  Delay(1);
  WriteRawRC(CommandReg,PCD_RESETPHASE);
  Delay(1);
  WriteRawRC(ModeReg,0x3D);
  WriteRawRC(TReloadRegL,30);
  WriteRawRC(TReloadRegH,0);
  WriteRawRC(TModeReg,0x8D);
  WriteRawRC(TPrescalerReg,0x3E);   
  return MI_OK; 
}

//******************************************************************/
//开启天线发射  
//每次启动或关闭天线发射之间应至少有1ms的间隔
//******************************************************************/
void PcdAntennaOn()
{
  uchar i;
  WriteRawRC(TxASKReg,0x40);
  Delay(10);
  i = ReadRawRC(TxControlReg);
  if(!(i&0x03))
    SetBitMask(TxControlReg, 0x03);
  i=ReadRawRC(TxASKReg);
}

//******************************************************************/
//开启天线发射  
//每次启动或关闭天险发射之间应至少有1ms的间隔
//******************************************************************/
void PcdAntennaTestOn()
{
//*
  RST522_1;
  Delay(15); // 2010.10.09 ???? FOR DEBUG
//  Delay(5); // 2010.10.09 ???? FOR DEBUG
  
  WriteRawRC(TxControlReg,0x02);
//*/
/*
  Delay(1); 
  SetBitMask(TxControlReg, 0x03);// FOR DEBUG 
*/
}


//******************************************************************/
//关闭天线发射
//******************************************************************/
void PcdAntennaOff()
{
  ClearBitMask(TxControlReg, 0x03);
}

//******************************************************************/
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
//******************************************************************/
char PcdComMF522(uchar Command  ,uchar *pInData , 
                 uchar InLenByte,uchar *pOutData, 
                 unsigned int  *pOutLenBit)
{
  char status   = MI_ERR;
  uchar irqEn   = 0x00;
  uchar waitFor = 0x00;
  uchar lastBits;
  uchar n;
  unsigned int  i;
  
  switch (Command)
  {
    case PCD_AUTHENT:
      irqEn   = 0x12;
      waitFor = 0x10;
      break;
    case PCD_TRANSCEIVE:
      irqEn   = 0x77;
      waitFor = 0x30;
      break;
    default:
      break;
  }
  WriteRawRC(ComIEnReg,irqEn|0x80); //
  ClearBitMask(ComIrqReg,0x80);
  WriteRawRC(CommandReg,PCD_IDLE);
  SetBitMask(FIFOLevelReg,0x80); // 清空FIFO 
  
  for(i=0; i<InLenByte; i++)
    WriteRawRC(FIFODataReg,pInData[i]); // 数据写入FIFO 
  
  WriteRawRC(CommandReg, Command); // 命令写入命令寄存器
  if(Command == PCD_TRANSCEIVE)
    SetBitMask(BitFramingReg,0x80); // 开始发送     
  
  i = 25000; //根据时钟频率调整，操作M1卡最大等待时间25ms
  do 
  {
    n = ReadRawRC(ComIrqReg);
    i--;
  }
  while ((i != 0) && !(n & 0x01) && !(n & waitFor));
  
  ClearBitMask(BitFramingReg,0x80);
  if(i!=0)
  {
    if(!(ReadRawRC(ErrorReg)&0x1B))
    {
      status = MI_OK;
      if (n&irqEn&0x01)
        status = MI_NOTAGERR;
      if(Command==PCD_TRANSCEIVE)
      {
        n = ReadRawRC(FIFOLevelReg);
        lastBits = ReadRawRC(ControlReg)&0x07;
        if(lastBits)
          *pOutLenBit = (n-1)*8 + lastBits;
        else
          *pOutLenBit = n*8;
        if(n==0)
          n = 1;
        if(n>MAXRLEN)
          n = MAXRLEN;
        for (i=0; i<n; i++)
          pOutData[i] = ReadRawRC(FIFODataReg); 
      }
    }
    else
      status = MI_ERR;        
  }
  SetBitMask(ControlReg, 0x80);// stop timer now
  WriteRawRC(CommandReg, PCD_IDLE); 
  return status;
}

//******************************************************************/
//功    能：寻卡                                                    /
//参数说明: req_code[IN]:寻卡方式                                   /
//                0x52 = 寻感应区内所有符合14443A标准的卡           /
//                0x26 = 寻未进入休眠状态的卡                       /
//          pTagType[OUT]：卡片类型代码                             /
//                0x4400 = Mifare_UltraLight                        /
//                0x0400 = Mifare_One(S50)                          /
//                0x0200 = Mifare_One(S70)                          /
//                0x0800 = Mifare_Pro(X)                            /
//                0x4403 = Mifare_DESFire                           /
//返    回: 成功返回MI_OK                                           /
//******************************************************************/
char PcdRequest(uchar req_code,uchar *pTagType)
{
  char status;  
  unsigned int  unLen;
  uchar ucComMF522Buf[MAXRLEN]; 

  ClearBitMask(Status2Reg,0x08);
  WriteRawRC(BitFramingReg,0x07);
  SetBitMask(TxControlReg,0x03);
 
  ucComMF522Buf[0] = req_code;

  status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);
  
  if ((status == MI_OK) && (unLen == 0x10))
  {    
    *pTagType     = ucComMF522Buf[0];
    *(pTagType+1) = ucComMF522Buf[1];
  }
  else
    status = MI_ERR;
  return status;
}

//******************************************************************/
//功    能：防冲撞                                                  /
//参数说明: pSnr[OUT]:卡片序列号，4字节                             /
//返    回: 成功返回MI_OK                                           /
//******************************************************************/
char PcdAnticoll(uchar *pSnr)
{
    char status;
    uchar i,snr_check=0;
    unsigned int  unLen;
    uchar ucComMF522Buf[MAXRLEN]; 
    
    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
    	 for (i=0; i<unLen/8-1; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(uchar *pSnr)
{
    char status;
    uchar i;
    unsigned int  unLen;
    uchar ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

//******************************************************************/
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdAuthState(uchar auth_mode,uchar addr,
                  uchar *pKey,uchar *pSnr    )
{
    char status;
    unsigned int  unLen;
    uchar i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

//******************************************************************/
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          pData[OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdRead(uchar addr,uchar *pData)
{
    char status;
    unsigned int  unLen;
    uchar i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,
                         ucComMF522Buf,&unLen           );
    if ((status == MI_OK) && (unLen == 0x90))
    {
        for (i=0; i<16; i++)
            *(pData+i) = ucComMF522Buf[i];   
    }
    else
      status = MI_ERR;       
    return status;
}

//******************************************************************/
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          pData[OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
//******************************************************************/
char Read_Block(uchar Block,uchar *Buf)
{
  char result;
  result = PcdAuthState(0x60,Block,Password_Buffer,UID);
  if(result!=MI_OK)
    return result;
  result = PcdRead(Block,Buf);
//  return result; // 2011.01.03
  
  if(result!=MI_OK)     return   result;
  if(Block!=0x00&&des_on)
  {
    Des_Decrypt((char *)Buf    ,KK,(char *)Buf    );
    Des_Decrypt((char *)&Buf[8],KK,(char *)&Buf[8]);  
  }
  return SUCCESS; 
}

//******************************************************************/
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdWrite(uchar addr,uchar *pData)
{
  char status;
  unsigned int  unLen;
  uchar i,ucComMF522Buf[MAXRLEN]; 
    
  ucComMF522Buf[0] = PICC_WRITE;
  ucComMF522Buf[1] = addr;
  CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,
                       ucComMF522Buf,&unLen          );
  if(  ( status != MI_OK)||(unLen != 4)
     ||((ucComMF522Buf[0]&0x0F)!= 0x0A))
    status = MI_ERR;           
  if (status == MI_OK)
  {
    for (i=0; i<16; i++)
      ucComMF522Buf[i] = *(pData+i);  
    CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,
                         18,ucComMF522Buf,&unLen     );
    if(  (status != MI_OK)||(unLen != 4 )
       ||((ucComMF522Buf[0]&0x0F)!= 0x0A))
      status = MI_ERR;   
  }    
  return status;
}
//******************************************************************/
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
//******************************************************************/

char Write_Block(uchar Block)
{
  char result;
  if(des_on)
  {
    Des_Encrypt((char *)RF_Buffer    ,KK,
                (char *)RF_Buffer        );// for debug
    Des_Encrypt((char *)&RF_Buffer[8],KK,
                (char *)&RF_Buffer[8]    );// for debug  
  }
  result = PcdAuthState(0x60,Block,Password_Buffer,UID);
  if(result!=MI_OK)
    return result;  
  result = PcdWrite(Block,RF_Buffer);
  return result;  
}

//******************************************************************/
//功    能：扣款和充值
//参数说明: dd_mode[IN]：命令字
//               0xC0 = 扣款
//               0xC1 = 充值
//          addr[IN]：钱包地址
//          pValue[IN]：4字节增(减)值，低位在前
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdValue(uchar dd_mode,uchar addr,uchar *pValue)
{
    char status;
    unsigned int  unLen;
    uchar i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

//******************************************************************/
//功    能：备份钱包
//参数说明: sourceaddr[IN]：源地址
//          goaladdr[IN]：目标地址
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdBakValue(uchar sourceaddr, uchar goaladdr)
{
    char status;
    unsigned int  unLen;
    uchar ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}


//******************************************************************/
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
//******************************************************************/
char PcdHalt(void)
{
    char status;
    unsigned int  unLen;
    uchar ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    return status;
//    return MI_OK;
}

//******************************************************************/
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
//******************************************************************/
char MIF_Halt(void)
{
    char status;
    unsigned int  unLen;
    uchar ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    return status;  
}



//******************************************************************/
//用MF522计算CRC16函数
//******************************************************************/
void CalulateCRC(uchar *pIndata,uchar len,uchar *pOutData)
{
    uchar i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}

