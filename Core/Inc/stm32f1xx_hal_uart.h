#ifndef CORE_INC_STM32F1XX_HAL_UART_FORWARD_H
#define CORE_INC_STM32F1XX_HAL_UART_FORWARD_H

/*
 * 兜底转发头文件。
 *
 * 正常情况下，Keil 会从 ../Drivers/STM32F1xx_HAL_Driver/Inc 找到
 * stm32f1xx_hal_uart.h。若工程缓存或 include path 没刷新，编译器会先在
 * Core/Inc 中查找本文件，再由这里转发到真正的 HAL UART 官方头文件。
 */
#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h"

#endif /* CORE_INC_STM32F1XX_HAL_UART_FORWARD_H */
