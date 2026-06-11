#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "main.h"
#include <stdint.h>

void Encoder_Init(void);
int16_t Encoder_GetCount(void);
void Encoder_Clear(void);
int16_t Encoder_GetDelta(void);
uint16_t Encoder_GetAbsDelta(void);
uint16_t Encoder_CalcRPM(uint16_t pulse_delta, uint16_t sample_ms);
uint16_t Encoder_GetRPM(uint16_t sample_ms);

#endif
