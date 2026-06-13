#ifndef BSP_BOARD_H
#define BSP_BOARD_H

/*
 * STM32F103RCT6 wiring map for the current motor test project.
 *
 * Motor driver: TB6612FNG
 * VM   -> external motor power
 * VCC  -> STM32 3.3V
 * GND  -> common ground
 * PWMA -> PA6 / TIM3_CH1 / PWM
 * AIN1 -> PB0 / direction input 1
 * AIN2 -> PB1 / direction input 2
 * STBY -> PB10 / driver enable
 *
 * New encoder motor:
 * Encoder VCC -> 3.3V
 * Encoder GND -> GND
 * Encoder A   -> PB6 / TIM4_CH1
 * Encoder B   -> PB7 / TIM4_CH2
 *
 * OLED:
 * SCL -> PC8 / software I2C clock
 * SDA -> PC9 / software I2C data
 *
 * Independent keys:
 * KEY_ADD -> PC4, active low
 * KEY_SUB -> PC5, active low
 *
 * Notes:
 * 1. STM32 GND, motor driver GND, motor power GND, encoder GND, and OLED GND
 *    must be connected together.
 * 2. TIM4 should use encoder mode TI12 for two-phase quadrature counting.
 * 3. If the count direction is reversed, swap A/B wires or change
 *    ENCODER_REVERSE_DIRECTION in bsp_encoder.h.
 */

#endif /* BSP_BOARD_H */
