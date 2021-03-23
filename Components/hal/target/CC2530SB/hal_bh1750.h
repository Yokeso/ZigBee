
#ifndef HAL_BH1750_H
#define HAL_BH1750_H

#include "hal_types.h"

void   BH1750_Init(void);
uint16 BH1750_ConvertLight(void);

#endif