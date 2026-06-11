#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "main.h"
#include <stdint.h>

/*
 * Independent keys, active low:
 * KEY_ADD -> PC4
 * KEY_SUB -> PC5
 *
 * Hardware connection:
 * GPIO pin ---- key ---- GND
 */

#define KEY_ADD_PORT      GPIOC
#define KEY_ADD_PIN       GPIO_PIN_4

#define KEY_SUB_PORT      GPIOC
#define KEY_SUB_PIN       GPIO_PIN_5

typedef enum
{
    KEY_NONE = 0,
    KEY_ADD,
    KEY_SUB
} Key_Value_t;

Key_Value_t Key_Scan(void);

#endif
