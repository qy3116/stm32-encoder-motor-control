#ifndef __BSP_SOFT_I2C_H
#define __BSP_SOFT_I2C_H

#include "main.h"
#include <stdint.h>

void SoftI2C_Init(void);
void SoftI2C_Start(void);
void SoftI2C_Stop(void);
uint8_t SoftI2C_WaitAck(void);
void SoftI2C_SendAck(void);
void SoftI2C_SendNack(void);
void SoftI2C_SendByte(uint8_t data);
uint8_t SoftI2C_ReadByte(uint8_t ack);

#endif
