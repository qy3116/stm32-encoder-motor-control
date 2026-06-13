#include "app.h"

#include "main.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include <stdint.h>

/* Encoder sampling period. RPM is calculated from the real elapsed time. */
#define APP_ENCODER_SAMPLE_PERIOD_MS 100U

/*
 * OLED software I2C is slow, so the display must not be refreshed every encoder
 * sample. Keeping it slower makes encoder sampling more stable.
 */
#define APP_DISPLAY_PERIOD_MS        500U

/* Motor speed command step and valid command range. */
#define APP_SPEED_STEP       10
#define APP_SPEED_MIN        (-100)
#define APP_SPEED_MAX        100

/* Application state shown on OLED and used by the control task. */
static int16_t encoder_cnt = 0;
static int16_t encoder_raw_delta = 0;
static int16_t encoder_delta_100ms = 0;
static uint16_t encoder_abs_delta = 0;
static int16_t target_speed = 0;
static uint16_t motor_rpm = 0;
static uint8_t encoder_phase_a = 0;
static uint8_t encoder_phase_b = 0;
static uint16_t sample_elapsed_ms = APP_ENCODER_SAMPLE_PERIOD_MS;
static uint32_t last_encoder_sample_time = 0;
static uint32_t last_display_time = 0;

static void App_HandleKey(void);
static void App_SetTargetSpeed(int16_t speed);
static void App_DrawStaticDisplay(void);
static void App_UpdateEncoder(uint16_t elapsed_ms);
static void App_UpdateDisplay(void);
static int16_t App_NormalizeDeltaTo100ms(int16_t delta, uint16_t elapsed_ms);
static int16_t App_ClampSpeed(int16_t speed);

/* Initialize board support modules and draw the first OLED frame. */
void App_Init(void)
{
  Motor_Init();
  Encoder_Init();

  OLED_Init();
  OLED_Clear();
  App_DrawStaticDisplay();

  App_SetTargetSpeed(target_speed);
  last_encoder_sample_time = HAL_GetTick();
  last_display_time = last_encoder_sample_time;

  App_UpdateDisplay();
}

/* Main application scheduler. Encoder sampling and OLED display use separate periods. */
void App_Task(void)
{
  uint32_t now_ms;
  uint32_t elapsed_ms;

  App_HandleKey();

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - last_encoder_sample_time;

  if (elapsed_ms >= APP_ENCODER_SAMPLE_PERIOD_MS)
  {
    /*
     * Use the real elapsed time instead of assuming the loop always returns
     * exactly after 100 ms. OLED software I2C can make the loop late, and RPM
     * must be calculated from the real sampling interval.
     */
    last_encoder_sample_time = now_ms;

    if (elapsed_ms > UINT16_MAX)
    {
      elapsed_ms = UINT16_MAX;
    }

    App_UpdateEncoder((uint16_t)elapsed_ms);
  }

  now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - last_display_time) >= APP_DISPLAY_PERIOD_MS)
  {
    last_display_time = now_ms;
    App_UpdateDisplay();
  }
}

/* Convert one key event into a bounded motor speed command. */
static void App_HandleKey(void)
{
  Key_Value_t key_value;
  int16_t new_speed;

  key_value = Key_Scan();
  new_speed = target_speed;

  if (key_value == KEY_ADD)
  {
    new_speed += APP_SPEED_STEP;
  }
  else if (key_value == KEY_SUB)
  {
    new_speed -= APP_SPEED_STEP;
  }
  else
  {
    return;
  }

  new_speed = App_ClampSpeed(new_speed);

  if (new_speed != target_speed)
  {
    App_SetTargetSpeed(new_speed);
    App_UpdateDisplay();
  }
}

/*
 * Apply a signed speed command.
 * Positive speed drives forward, negative speed drives reverse, and zero stops.
 */
static void App_SetTargetSpeed(int16_t speed)
{
  target_speed = App_ClampSpeed(speed);
  Motor_SetSpeed(target_speed);
}

/* Draw fixed OLED labels once, then only refresh numeric fields in the loop. */
static void App_DrawStaticDisplay(void)
{
  OLED_ShowString(0, 0, "Speed:");
  OLED_ShowString(0, 1, "Cnt:");
  OLED_ShowString(0, 2, "D100:");
  OLED_ShowString(0, 3, "RPM:");
  OLED_ShowString(0, 4, "A:");
  OLED_ShowString(30, 4, "B:");
  OLED_ShowString(0, 5, "T:");
  OLED_ShowString(42, 5, "ms");
  OLED_ShowString(0, 6, "Raw:");
}

/* Sample the encoder once per application period and derive RPM from delta. */
static void App_UpdateEncoder(uint16_t elapsed_ms)
{
  encoder_cnt = Encoder_GetCount();
  encoder_raw_delta = Encoder_GetDelta();
  encoder_delta_100ms = App_NormalizeDeltaTo100ms(encoder_raw_delta, elapsed_ms);
  encoder_abs_delta = Encoder_AbsDelta(encoder_raw_delta);
  motor_rpm = Encoder_CalcRPM(encoder_abs_delta, elapsed_ms);
  encoder_phase_a = Encoder_GetPhaseA();
  encoder_phase_b = Encoder_GetPhaseB();
  sample_elapsed_ms = elapsed_ms;
}

/* Refresh the OLED values used to validate motor command and encoder wiring. */
static void App_UpdateDisplay(void)
{
  OLED_ShowSignedNum(42, 0, target_speed, 3);

  OLED_ShowSignedNum(42, 1, encoder_cnt, 5);

  OLED_ShowSignedNum(36, 2, encoder_delta_100ms, 5);

  OLED_ShowNum(30, 3, motor_rpm, 5);

  OLED_ShowNum(12, 4, encoder_phase_a, 1);
  OLED_ShowNum(42, 4, encoder_phase_b, 1);

  OLED_ShowNum(12, 5, sample_elapsed_ms, 4);

  OLED_ShowSignedNum(30, 6, encoder_raw_delta, 5);
}

/*
 * Convert the real measured delta to an equivalent 100 ms delta for display.
 * Example: if the loop was delayed to 400 ms, Raw may be about 4 times larger
 * than the 100 ms count. D100 keeps the value comparable with hand calculation.
 */
static int16_t App_NormalizeDeltaTo100ms(int16_t delta, uint16_t elapsed_ms)
{
  int32_t normalized_delta;

  if (elapsed_ms == 0U)
  {
    return 0;
  }

  normalized_delta = ((int32_t)delta * (int32_t)APP_ENCODER_SAMPLE_PERIOD_MS) /
                     (int32_t)elapsed_ms;

  if (normalized_delta > INT16_MAX)
  {
    return INT16_MAX;
  }

  if (normalized_delta < INT16_MIN)
  {
    return INT16_MIN;
  }

  return (int16_t)normalized_delta;
}

/* Keep target speed inside the range accepted by Motor_SetSpeed(). */
static int16_t App_ClampSpeed(int16_t speed)
{
  if (speed > APP_SPEED_MAX)
  {
    return APP_SPEED_MAX;
  }

  if (speed < APP_SPEED_MIN)
  {
    return APP_SPEED_MIN;
  }

  return speed;
}
