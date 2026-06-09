#include "bsp_motor.h"
#include "tim.h"

/*
    TB6612 接线：
    PWMA -> PA6 / TIM3_CH1
    AIN1 -> PB0
    AIN2 -> PB1
    STBY -> PB10
*/

/* PWM最大值
 * 需要与TIM3的ARR保持一致
 * 例如ARR=99，则PWM最大值为100
 */
#define MOTOR_PWM_MAX 1000




/**
 * @brief 电机初始化
 * @note  启动PWM输出，并使能TB6612
 */
void Motor_Init(void)
{
    /* 开启TIM3通道1 PWM输出 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    /* STBY拉高，退出待机模式 */
    HAL_GPIO_WritePin(STBY_GPIO_Port,
                      STBY_Pin,
                      GPIO_PIN_SET);

    /* 初始化后先停止电机 */
    Motor_Stop();
}


/**
 * @brief 设置PWM占空比
 * @param pwm PWM值(0~MOTOR_PWM_MAX)
 * @note  实际控制电机驱动能力
 */
void Motor_SetPWM(uint16_t pwm)
{
    /* 防止PWM超过最大值 */
    if(pwm > MOTOR_PWM_MAX)
    {
        pwm = MOTOR_PWM_MAX;
    }

    /* 修改CCR值改变占空比 */
    __HAL_TIM_SET_COMPARE(&htim3,
                          TIM_CHANNEL_1,
                          pwm);
}


/**
 * @brief 设置电机速度及方向
 * @param speed
 *        100  -> 正转100%
 *         50  -> 正转50%
 *          0  -> 停止
 *        -50  -> 反转50%
 *       -100  -> 反转100%
 */
void Motor_SetSpeed(int16_t speed)
{
    uint16_t pwm;

    /* 限幅处理 */
    if(speed > 100)
    {
        speed = 100;
    }

    if(speed < -100)
    {
        speed = -100;
    }

    /* 正转 */
    if(speed > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port,
                          AIN1_Pin,
                          GPIO_PIN_SET);

        HAL_GPIO_WritePin(AIN2_GPIO_Port,
                          AIN2_Pin,
                          GPIO_PIN_RESET);

        /* 百分比转换成PWM值 */
        pwm = speed * MOTOR_PWM_MAX / 100;

        Motor_SetPWM(pwm);
    }

    /* 反转 */
    else if(speed < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port,
                          AIN1_Pin,
                          GPIO_PIN_RESET);

        HAL_GPIO_WritePin(AIN2_GPIO_Port,
                          AIN2_Pin,
                          GPIO_PIN_SET);

        pwm = (-speed) * MOTOR_PWM_MAX / 100;

        Motor_SetPWM(pwm);
    }

    /* 停止 */
    else
    {
        Motor_Stop();
    }
}


/**
 * @brief 电机滑行停止
 * @note  电机自由减速停止
 *        AIN1=0
 *        AIN2=0
 */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port,
                      AIN1_Pin,
                      GPIO_PIN_RESET);

    HAL_GPIO_WritePin(AIN2_GPIO_Port,
                      AIN2_Pin,
                      GPIO_PIN_RESET);

    Motor_SetPWM(0);
}


/**
 * @brief 电机制动
 * @note  利用H桥短接实现快速停车
 *        AIN1=1
 *        AIN2=1
 */
void Motor_Brake(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port,
                      AIN1_Pin,
                      GPIO_PIN_SET);

    HAL_GPIO_WritePin(AIN2_GPIO_Port,
                      AIN2_Pin,
                      GPIO_PIN_SET);

    Motor_SetPWM(0);
}


/**
 * @brief 电机缓启动
 * @param target_speed 目标速度(-100~100)
 * @note  防止电机瞬间启动电流过大
 */
void Motor_SoftStart(int16_t target_speed)
{
    int16_t i;

    /* 限幅 */
    if(target_speed > 100)
    {
        target_speed = 100;
    }

    if(target_speed < -100)
    {
        target_speed = -100;
    }

    /* 正转缓启动 */
    if(target_speed >= 0)
    {
        for(i = 0; i <= target_speed; i++)
        {
            Motor_SetSpeed(i);

            /* 每20ms增加一次速度 */
            HAL_Delay(20);
        }
    }

    /* 反转缓启动 */
    else
    {
        for(i = 0; i >= target_speed; i--)
        {
            Motor_SetSpeed(i);

            HAL_Delay(20);
        }
    }
}
