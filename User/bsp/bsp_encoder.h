#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "main.h"
#include <stdint.h>

/*
 * New motor encoder wiring:
 * Phase A -> PB6 / TIM4_CH1
 * Phase B -> PB7 / TIM4_CH2
 *
 * TIM4 should be configured as TIM_ENCODERMODE_TI12 so both phases participate
 * in quadrature counting. If the forward direction is opposite from what you
 * expect, either swap A/B wires or set ENCODER_REVERSE_DIRECTION to 1.
 */
#define ENCODER_PHASE_A_PORT              GPIOB
#define ENCODER_PHASE_A_PIN               GPIO_PIN_6
#define ENCODER_PHASE_B_PORT              GPIOB
#define ENCODER_PHASE_B_PIN               GPIO_PIN_7
#define ENCODER_REVERSE_DIRECTION         0U

/*
 * RPM conversion parameters.
 *
 * ENCODER_LINES_PER_MOTOR_REV is the number of A-phase pulses per motor-shaft
 * revolution before quadrature multiplication. Replace this value with the
 * datasheet value of the new motor.
 *
 * In TIM_ENCODERMODE_TI12, STM32 counts both A and B phase edges, so a normal
 * quadrature encoder contributes 4 timer counts per encoder line.
 */
#define ENCODER_LINES_PER_MOTOR_REV       11U
#define ENCODER_QUADRATURE_MULTIPLIER     4U
#define ENCODER_GEAR_RATIO                30U
#define ENCODER_OUTPUT_PPR                (ENCODER_LINES_PER_MOTOR_REV * \
                                           ENCODER_QUADRATURE_MULTIPLIER * \
                                           ENCODER_GEAR_RATIO)

void Encoder_Init(void);
void Encoder_Clear(void);
int16_t Encoder_GetCount(void);
int16_t Encoder_GetDelta(void);
uint16_t Encoder_AbsDelta(int16_t delta);
uint16_t Encoder_CalcRPM(uint16_t pulse_delta, uint16_t sample_ms);
uint8_t Encoder_GetPhaseA(void);
uint8_t Encoder_GetPhaseB(void);

#endif /* BSP_ENCODER_H */
