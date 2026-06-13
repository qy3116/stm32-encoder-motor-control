#include "bsp_motor.h"
#include "tim.h"

/*
 * TB6612FNG wiring:
 * PWMA -> PA6 / TIM3_CH1
 * AIN1 -> PB0
 * AIN2 -> PB1
 * STBY -> PB10
 */

/* Start PWM output, enable TB6612, and leave the motor stopped. */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    Motor_Stop();
}

/* Set raw PWM compare value and clamp it to the TIM3 period. */
void Motor_SetPWM(uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX)
    {
        pwm = MOTOR_PWM_MAX;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

/*
 * Set motor direction and speed percentage.
 * speed > 0: forward
 * speed < 0: reverse
 * speed = 0: coast stop
 */
void Motor_SetSpeed(int16_t speed)
{
    uint16_t pwm;

    if (speed > MOTOR_SPEED_MAX_PERCENT)
    {
        speed = MOTOR_SPEED_MAX_PERCENT;
    }

    if (speed < MOTOR_SPEED_MIN_PERCENT)
    {
        speed = MOTOR_SPEED_MIN_PERCENT;
    }

    if (speed > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        pwm = (uint16_t)((speed * (int16_t)MOTOR_PWM_MAX) / 100);
        Motor_SetPWM(pwm);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

        pwm = (uint16_t)(((-speed) * (int16_t)MOTOR_PWM_MAX) / 100);
        Motor_SetPWM(pwm);
    }
    else
    {
        Motor_Stop();
    }
}

/* Coast stop: both direction inputs are low and PWM is zero. */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    Motor_SetPWM(0);
}
