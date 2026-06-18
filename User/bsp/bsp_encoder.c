#include "bsp_encoder.h"
#include "tim.h"

static int16_t encoder_last_count = 0;

/* Start TIM4 encoder mode and reset the software delta baseline. */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    Encoder_Clear();
}

/* Return the raw signed view of TIM4->CNT. This is useful for OLED diagnosis. */
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

/*
 * Return signed count change since the previous call.
 * The int16_t subtraction is intentional because TIM4 is a 16-bit counter and
 * this form handles normal wraparound as long as one sample is below 32768 counts.
 */
int16_t Encoder_GetDelta(void)
{
    int16_t now_count;
    int16_t delta;

    now_count = Encoder_GetCount();
    delta = (int16_t)(now_count - encoder_last_count);
    encoder_last_count = now_count;

#if ENCODER_REVERSE_DIRECTION
    delta = (int16_t)(-delta);
#endif

    return delta;
}

/* Convert a signed delta to magnitude without overflowing at INT16_MIN. */
uint16_t Encoder_AbsDelta(int16_t delta)
{
    if (delta < 0)
    {
        return (uint16_t)(-(int32_t)delta);
    }

    return (uint16_t)delta;
}

/* Convert a sampled pulse delta to output-shaft RPM. */
uint16_t Encoder_CalcRPM(uint16_t pulse_delta, uint16_t sample_ms)
{
    uint32_t rpm;

    if ((sample_ms == 0U) || (ENCODER_OUTPUT_PPR == 0U))
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

/* Read raw phase A level so wiring can be checked directly on the OLED. */
uint8_t Encoder_GetPhaseA(void)
{
    return (HAL_GPIO_ReadPin(ENCODER_PHASE_A_PORT, ENCODER_PHASE_A_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Read raw phase B level so wiring can be checked directly on the OLED. */
uint8_t Encoder_GetPhaseB(void)
{
    return (HAL_GPIO_ReadPin(ENCODER_PHASE_B_PORT, ENCODER_PHASE_B_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}
