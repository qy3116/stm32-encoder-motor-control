#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"
#include <stdint.h>

#define MOTOR_PWM_MAX 1000

void Motor_Init(void);
void Motor_SetPWM(uint16_t pwm);
void Motor_SetSpeed(int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);
void Motor_SoftStart(int16_t target_speed);

#endif
