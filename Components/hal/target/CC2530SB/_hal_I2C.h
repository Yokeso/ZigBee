#ifndef I2C_H
#define I2C_H

#include "hal_types.h"
 
extern  void  I2C_Start(void);
extern  void  I2C_Stop(void);
extern  uint8 I2C_Read(uint8 ack);
extern  uint8 I2C_Send(uint8 val);

#endif