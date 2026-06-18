#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "main.h"
#include <stdint.h>

/* TIM3 period is 1000 - 1, so valid CCR values are 0..1000. */
#define MOTOR_PWM_MAX            1000U
#define MOTOR_SPEED_MIN_PERCENT  (-100)
#define MOTOR_SPEED_MAX_PERCENT  100

void Motor_Init(void);
void Motor_SetPWM(uint16_t pwm);
void Motor_SetSpeed(int16_t speed);
void Motor_Stop(void);

#endif /* BSP_MOTOR_H */
