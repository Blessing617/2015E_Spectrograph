#ifndef _Data_handle_H
#define _Data_handle_H

#include "main.h"
#include "delay.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "myusart1.h"
#include "OLED_I2C.h"

void ADC_Init(void);
void ADC_Get(void);
void Fre_Control(uint32_t Fre);
void Data_handle(void);

#endif
