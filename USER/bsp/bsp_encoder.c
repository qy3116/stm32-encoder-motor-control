#include "bsp_encoder.h"
#include "tim.h"

#define ENCODER_MOTOR_PPR        2U
#define MOTOR_REDUCTION_RATIO    30U
#define ENCODER_OUTPUT_PPR       (ENCODER_MOTOR_PPR * MOTOR_REDUCTION_RATIO)

static int16_t encoder_last_count = 0;

/* Start TIM4 encoder mode and reset the software delta baseline. */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    Encoder_Clear();
}

int16_t Encoder_GetCount(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/* Clear both the hardware counter and the previous-count snapshot. */
void Encoder_Clear(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    encoder_last_count = 0;
}

int16_t Encoder_GetDelta(void)
{
    int16_t now_count;
    int16_t delta;

    now_count = Encoder_GetCount();
    delta = (int16_t)(now_count - encoder_last_count);
    encoder_last_count = now_count;

    return delta;
}

/* Use a wider type when negating to avoid the int16_t minimum edge case. */
uint16_t Encoder_GetAbsDelta(void)
{
    int16_t delta;

    delta = Encoder_GetDelta();

    if (delta < 0)
    {
        return (uint16_t)(-(int32_t)delta);
    }

    return (uint16_t)delta;
}

/* Convert a previously sampled pulse delta to output-shaft RPM. */
uint16_t Encoder_CalcRPM(uint16_t pulse_delta, uint16_t sample_ms)
{
    uint32_t rpm;

    if (sample_ms == 0)
    {
        return 0;
    }

    rpm = ((uint32_t)pulse_delta * 60000U) /
          ((uint32_t)ENCODER_OUTPUT_PPR * sample_ms);

    if (rpm > UINT16_MAX)
    {
        rpm = UINT16_MAX;
    }

    return (uint16_t)rpm;
}

uint16_t Encoder_GetRPM(uint16_t sample_ms)
{
    return Encoder_CalcRPM(Encoder_GetAbsDelta(), sample_ms);
}
