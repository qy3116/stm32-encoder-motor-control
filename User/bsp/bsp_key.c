#include "bsp_key.h"

#define KEY_DEBOUNCE_MS 20U

/**
 * @brief Scan independent keys.
 * @return KEY_NONE if no key is pressed.
 *         KEY_ADD if PC4 is pressed.
 *         KEY_SUB if PC5 is pressed.
 *
 * @note Keys are active low and single-triggered. Holding a key down will not
 *       repeatedly trigger until all keys are released.
 */
Key_Value_t Key_Scan(void)
{
    static uint8_t waiting_release = 0;
    static uint8_t debounce_active = 0;
    static uint32_t debounce_start_ms = 0;
    uint8_t add_pressed;
    uint8_t sub_pressed;
    uint32_t now_ms;

    add_pressed = (HAL_GPIO_ReadPin(KEY_ADD_PORT, KEY_ADD_PIN) == GPIO_PIN_RESET);
    sub_pressed = (HAL_GPIO_ReadPin(KEY_SUB_PORT, KEY_SUB_PIN) == GPIO_PIN_RESET);
    now_ms = HAL_GetTick();

    /* All keys released: arm the scanner for the next single key event. */
    if (!add_pressed && !sub_pressed)
    {
        waiting_release = 0;
        debounce_active = 0;
        return KEY_NONE;
    }

    /* A key event was already reported; wait until every key is released. */
    if (waiting_release)
    {
        return KEY_NONE;
    }

    /* Start debounce timing without blocking the main loop. */
    if (!debounce_active)
    {
        debounce_active = 1;
        debounce_start_ms = now_ms;
        return KEY_NONE;
    }

    /* Ignore unstable input until it has stayed pressed long enough. */
    if ((uint32_t)(now_ms - debounce_start_ms) < KEY_DEBOUNCE_MS)
    {
        return KEY_NONE;
    }

    debounce_active = 0;
    waiting_release = 1;

    /* Re-read after debounce so the reported key is the stable one. */
    if (HAL_GPIO_ReadPin(KEY_ADD_PORT, KEY_ADD_PIN) == GPIO_PIN_RESET)
    {
        return KEY_ADD;
    }

    if (HAL_GPIO_ReadPin(KEY_SUB_PORT, KEY_SUB_PIN) == GPIO_PIN_RESET)
    {
        return KEY_SUB;
    }

    waiting_release = 0;
    return KEY_NONE;
}
