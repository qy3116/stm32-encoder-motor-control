#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include <stdint.h>

/* EncoderTask 按这个周期采样编码器。 */
#define APP_ENCODER_SAMPLE_PERIOD_MS 100U

/* OLED 使用软件 I2C，刷新会占用较多 CPU，所以 OLEDTask 刷新周期故意放慢。 */
#define APP_DISPLAY_PERIOD_MS        500U

/* KeyTask 只需要满足人手按键响应，不必高频空转浪费 CPU。 */
#define APP_KEY_SCAN_PERIOD_MS       10U

/* MotorTask 用适中的周期把目标速度应用到电机。 */
#define APP_MOTOR_UPDATE_PERIOD_MS   20U

/* 电机速度命令的步进值和允许范围。 */
#define APP_SPEED_STEP       10
#define APP_SPEED_MIN        (-100)
#define APP_SPEED_MAX        100

typedef struct
{
  int16_t target_speed;
  int16_t encoder_cnt;
  int16_t encoder_raw_delta;
  int16_t encoder_delta_100ms;
  uint16_t motor_rpm;
  uint8_t encoder_phase_a;
  uint8_t encoder_phase_b;
  uint16_t sample_elapsed_ms;
} App_DisplaySnapshot_t;

/*
 * 应用层共享状态。
 * 多个 FreeRTOS 任务会读写这些变量，所以成组读写时用很短的临界区保护。
 */
static volatile int16_t target_speed = 0;
static volatile int16_t encoder_cnt = 0;
static volatile int16_t encoder_raw_delta = 0;
static volatile int16_t encoder_delta_100ms = 0;
static volatile uint16_t motor_rpm = 0;
static volatile uint8_t encoder_phase_a = 0;
static volatile uint8_t encoder_phase_b = 0;
static volatile uint16_t sample_elapsed_ms = APP_ENCODER_SAMPLE_PERIOD_MS;

static void App_DrawStaticDisplay(void);
static void App_UpdateDisplay(const App_DisplaySnapshot_t *snapshot);
static void App_ReadDisplaySnapshot(App_DisplaySnapshot_t *snapshot);
static void App_PublishEncoderSample(int16_t cnt,
                                     int16_t raw_delta,
                                     int16_t delta_100ms,
                                     uint16_t rpm,
                                     uint8_t phase_a,
                                     uint8_t phase_b,
                                     uint16_t elapsed_ms);
static int16_t App_ReadTargetSpeed(void);
static void App_WriteTargetSpeed(int16_t speed);
static int16_t App_NormalizeDeltaTo100ms(int16_t delta, uint16_t elapsed_ms);
static int16_t App_ClampSpeed(int16_t speed);

/* 在各个任务开始运行前，先复位共享的软件状态。 */
void App_Init(void)
{
  /*
   * MX_FREERTOS_Init() 会在调度器真正启动任务之前调用这里，
   * 此时还没有任务并发，所以不需要进入临界区。
   */
  target_speed = 0;
  encoder_cnt = 0;
  encoder_raw_delta = 0;
  encoder_delta_100ms = 0;
  motor_rpm = 0;
  encoder_phase_a = 0;
  encoder_phase_b = 0;
  sample_elapsed_ms = APP_ENCODER_SAMPLE_PERIOD_MS;
}

/*
 * EncoderTask 专门负责 TIM4 编码器采样。
 * 它不操作 OLED，也不直接控制电机，所以编码器采样不会被慢速显示刷新
 * 或按键消抖逻辑拖慢。
 */
void App_EncoderTask(void)
{
  uint32_t last_sample_time;

  Encoder_Init();
  last_sample_time = HAL_GetTick();

  for (;;)
  {
    uint32_t now_ms;
    uint32_t elapsed_ms;
    uint16_t elapsed_ms_16;
    int16_t cnt;
    int16_t raw_delta;
    int16_t delta_100ms;
    uint16_t abs_delta;
    uint16_t rpm;
    uint8_t phase_a;
    uint8_t phase_b;

    osDelay(APP_ENCODER_SAMPLE_PERIOD_MS);

    now_ms = HAL_GetTick();
    elapsed_ms = now_ms - last_sample_time;
    last_sample_time = now_ms;

    if (elapsed_ms > UINT16_MAX)
    {
      elapsed_ms = UINT16_MAX;
    }

    elapsed_ms_16 = (uint16_t)elapsed_ms;
    cnt = Encoder_GetCount();
    raw_delta = Encoder_GetDelta();
    delta_100ms = App_NormalizeDeltaTo100ms(raw_delta, elapsed_ms_16);
    abs_delta = Encoder_AbsDelta(raw_delta);
    rpm = Encoder_CalcRPM(abs_delta, elapsed_ms_16);
    phase_a = Encoder_GetPhaseA();
    phase_b = Encoder_GetPhaseB();

    App_PublishEncoderSample(cnt, raw_delta, delta_100ms, rpm, phase_a, phase_b, elapsed_ms_16);
  }
}

/*
 * OLEDTask 专门负责 OLED 绘制。
 * 所有 OLED 调用集中在一个任务里，可以避免多个任务同时操作软件 I2C 总线。
 */
void App_OLEDTask(void)
{
  OLED_Init();
  OLED_Clear();
  App_DrawStaticDisplay();

  for (;;)
  {
    App_DisplaySnapshot_t snapshot;

    App_ReadDisplaySnapshot(&snapshot);
    App_UpdateDisplay(&snapshot);
    osDelay(APP_DISPLAY_PERIOD_MS);
  }
}

/*
 * KeyTask 把按键事件转换成目标速度命令。
 * 它只修改“想要的速度”，真正输出到电机由 MotorTask 负责。
 */
void App_KeyTask(void)
{
  for (;;)
  {
    Key_Value_t key_value;
    int16_t new_speed;

    key_value = Key_Scan();

    if (key_value != KEY_NONE)
    {
      new_speed = App_ReadTargetSpeed();

      if (key_value == KEY_ADD)
      {
        new_speed += APP_SPEED_STEP;
      }
      else if (key_value == KEY_SUB)
      {
        new_speed -= APP_SPEED_STEP;
      }

      App_WriteTargetSpeed(new_speed);
    }

    osDelay(APP_KEY_SCAN_PERIOD_MS);
  }
}

/*
 * MotorTask 专门负责 TB6612 输出。
 * 它读取最新目标速度，只有命令变化时才重新调用 Motor_SetSpeed()。
 */
void App_MotorTask(void)
{
  int16_t applied_speed = (int16_t)(APP_SPEED_MIN - 1);

  Motor_Init();

  for (;;)
  {
    int16_t desired_speed;

    desired_speed = App_ReadTargetSpeed();

    if (desired_speed != applied_speed)
    {
      Motor_SetSpeed(desired_speed);
      applied_speed = desired_speed;
    }

    osDelay(APP_MOTOR_UPDATE_PERIOD_MS);
  }
}

/* 固定的 OLED 标签只画一次，后面 OLEDTask 只刷新数字字段。 */
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

/* 使用绘制前抓取的快照刷新 OLED 数值。 */
static void App_UpdateDisplay(const App_DisplaySnapshot_t *snapshot)
{
  OLED_ShowSignedNum(42, 0, snapshot->target_speed, 3);

  OLED_ShowSignedNum(42, 1, snapshot->encoder_cnt, 5);

  OLED_ShowSignedNum(36, 2, snapshot->encoder_delta_100ms, 5);

  OLED_ShowNum(30, 3, snapshot->motor_rpm, 5);

  OLED_ShowNum(12, 4, snapshot->encoder_phase_a, 1);
  OLED_ShowNum(42, 4, snapshot->encoder_phase_b, 1);

  OLED_ShowNum(12, 5, snapshot->sample_elapsed_ms, 4);

  OLED_ShowSignedNum(30, 6, snapshot->encoder_raw_delta, 5);
}

/* 在一个很短的临界区里复制所有显示字段，避免一帧数据新旧混杂。 */
static void App_ReadDisplaySnapshot(App_DisplaySnapshot_t *snapshot)
{
  taskENTER_CRITICAL();
  snapshot->target_speed = target_speed;
  snapshot->encoder_cnt = encoder_cnt;
  snapshot->encoder_raw_delta = encoder_raw_delta;
  snapshot->encoder_delta_100ms = encoder_delta_100ms;
  snapshot->motor_rpm = motor_rpm;
  snapshot->encoder_phase_a = encoder_phase_a;
  snapshot->encoder_phase_b = encoder_phase_b;
  snapshot->sample_elapsed_ms = sample_elapsed_ms;
  taskEXIT_CRITICAL();
}

/* 发布一组完整的编码器采样结果，供 OLEDTask 显示。 */
static void App_PublishEncoderSample(int16_t cnt,
                                     int16_t raw_delta,
                                     int16_t delta_100ms,
                                     uint16_t rpm,
                                     uint8_t phase_a,
                                     uint8_t phase_b,
                                     uint16_t elapsed_ms)
{
  taskENTER_CRITICAL();
  encoder_cnt = cnt;
  encoder_raw_delta = raw_delta;
  encoder_delta_100ms = delta_100ms;
  motor_rpm = rpm;
  encoder_phase_a = phase_a;
  encoder_phase_b = phase_b;
  sample_elapsed_ms = elapsed_ms;
  taskEXIT_CRITICAL();
}

/* 读取 KeyTask、MotorTask、OLEDTask 共享的目标速度命令。 */
static int16_t App_ReadTargetSpeed(void)
{
  int16_t speed;

  taskENTER_CRITICAL();
  speed = target_speed;
  taskEXIT_CRITICAL();

  return speed;
}

/* 保存限幅后的目标速度，等待 MotorTask 应用到电机。 */
static void App_WriteTargetSpeed(int16_t speed)
{
  speed = App_ClampSpeed(speed);

  taskENTER_CRITICAL();
  target_speed = speed;
  taskEXIT_CRITICAL();
}

/*
 * 把实际采样间隔下的增量折算成等效 100 ms 增量。
 * Raw 显示真实采样计数；D100 用来在 EncoderTask 被调度器稍微延迟唤醒时
 * 仍然能和手算值做对比。
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

/* 确保目标速度保持在 Motor_SetSpeed() 能接受的范围内。 */
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
