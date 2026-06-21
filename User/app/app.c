#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_vofa.h"
#include "pid.h"
#include <stdint.h>

/* EncoderTask 按这个周期采样编码器，也是速度 PID 的计算周期。 */
#define APP_ENCODER_SAMPLE_PERIOD_MS 100U

/* OLED 使用软件 I2C，刷新会占用较多 CPU，所以 OLEDTask 刷新周期故意放慢。 */
#define APP_DISPLAY_PERIOD_MS        500U

/* KeyTask 只需要满足人手按键响应，不必高频空转浪费 CPU。 */
#define APP_KEY_SCAN_PERIOD_MS       10U

/* MotorTask 做速度闭环，周期和编码器速度采样保持一致，避免用旧速度反复计算 PID。 */
#define APP_MOTOR_UPDATE_PERIOD_MS   APP_ENCODER_SAMPLE_PERIOD_MS

/* VofaTask 只负责串口发调试数据，50 ms 一帧足够观察 PID 曲线。 */
#define APP_VOFA_SEND_PERIOD_MS      50U

/* 按键修改的是“目标百分比”，后面再换算成目标 RPM。 */
#define APP_SPEED_STEP               10
#define APP_SPEED_MIN                (-100)
#define APP_SPEED_MAX                100

/*
 * 100% 命令对应的目标转速。
 * 这个值不是硬件极限，只是 PID 学习起点；如果你的电机实测满速明显不同，可以再调整。
 */
#define APP_TARGET_RPM_MAX           330

/*
 * 速度 PID 的初始参数。
 * Kp: 当前误差越大，输出越大。
 * Ki: 长时间达不到目标时，慢慢补偿静差。
 * Kd: 先设为 0，初学调速度环时先把 P/I 调明白。
 */
#define APP_PID_KP                   0.3f
#define APP_PID_KI                   0.25f
#define APP_PID_KD                   0.0f

typedef struct
{
  int16_t target_speed;        /* 按键给出的目标百分比，范围 -100..100。 */
  int16_t target_rpm;          /* 目标转速，PID 的 setpoint。 */
  int16_t actual_rpm;          /* 编码器测得的实际转速，PID 的 measurement。 */
  int16_t pid_output;          /* PID 输出，最后交给 Motor_SetSpeed()。 */
  int16_t encoder_cnt;
  int16_t encoder_raw_delta;
  int16_t encoder_delta_100ms;
  uint8_t encoder_phase_a;
  uint8_t encoder_phase_b;
  uint16_t sample_elapsed_ms;
} App_DisplaySnapshot_t;

/*
 * 应用层共享状态。
 * 多个 FreeRTOS 任务会读写这些变量，成组读写时用很短的临界区保护。
 */
static volatile int16_t target_speed = 0;
static volatile int16_t target_rpm = 0;
static volatile int16_t actual_rpm = 0;
static volatile int16_t pid_output = 0;
static volatile int16_t encoder_cnt = 0;
static volatile int16_t encoder_raw_delta = 0;
static volatile int16_t encoder_delta_100ms = 0;
static volatile uint8_t encoder_phase_a = 0;
static volatile uint8_t encoder_phase_b = 0;
static volatile uint16_t sample_elapsed_ms = APP_ENCODER_SAMPLE_PERIOD_MS;
static volatile float vofa_target_rpm = 0.0f;
static volatile float vofa_actual_rpm = 0.0f;
static volatile float vofa_pid_output = 0.0f;
static volatile float vofa_error = 0.0f;
static volatile float vofa_p_term = 0.0f;
static volatile float vofa_i_term = 0.0f;
static volatile float vofa_d_term = 0.0f;

static void App_DrawStaticDisplay(void);
static void App_UpdateDisplay(const App_DisplaySnapshot_t *snapshot);
static void App_ReadDisplaySnapshot(App_DisplaySnapshot_t *snapshot);
static void App_ReadControlSnapshot(int16_t *target_rpm_snapshot,
                                    int16_t *actual_rpm_snapshot);
static void App_ReadVofaSnapshot(Vofa_PidFrame_t *snapshot);
static void App_PublishEncoderSample(int16_t cnt,
                                     int16_t raw_delta,
                                     int16_t delta_100ms,
                                     int16_t rpm,
                                     uint8_t phase_a,
                                     uint8_t phase_b,
                                     uint16_t elapsed_ms);
static void App_PublishPidOutput(int16_t output);
static void App_PublishVofaSnapshot(float target,
                                    float actual,
                                    float output,
                                    float error,
                                    float p_term,
                                    float i_term,
                                    float d_term);
static int16_t App_ReadTargetSpeed(void);
static void App_WriteTargetSpeed(int16_t speed);
static int16_t App_SpeedToTargetRpm(int16_t speed);
static int16_t App_MakeSignedRpm(uint16_t rpm, int16_t raw_delta);
static int16_t App_RoundPidOutput(float output);
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
  target_rpm = 0;
  actual_rpm = 0;
  pid_output = 0;
  encoder_cnt = 0;
  encoder_raw_delta = 0;
  encoder_delta_100ms = 0;
  encoder_phase_a = 0;
  encoder_phase_b = 0;
  sample_elapsed_ms = APP_ENCODER_SAMPLE_PERIOD_MS;
  vofa_target_rpm = 0.0f;
  vofa_actual_rpm = 0.0f;
  vofa_pid_output = 0.0f;
  vofa_error = 0.0f;
  vofa_p_term = 0.0f;
  vofa_i_term = 0.0f;
  vofa_d_term = 0.0f;
}

/*
 * EncoderTask 专门负责 TIM4 编码器采样。
 * 它只发布“实际速度”和编码器原始信息，不直接刷新 OLED，也不直接控制电机。
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
    uint16_t rpm_unsigned;
    int16_t rpm_signed;
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
    rpm_unsigned = Encoder_CalcRPM(abs_delta, elapsed_ms_16);
    rpm_signed = App_MakeSignedRpm(rpm_unsigned, raw_delta);
    phase_a = Encoder_GetPhaseA();
    phase_b = Encoder_GetPhaseB();

    App_PublishEncoderSample(cnt,
                             raw_delta,
                             delta_100ms,
                             rpm_signed,
                             phase_a,
                             phase_b,
                             elapsed_ms_16);
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
 * KeyTask 把按键事件转换成目标速度。
 * 注意：按键只改“我想要多少转速”，真正给电机多少输出由 MotorTask 的 PID 决定。
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
 * MotorTask 专门负责速度闭环。
 *
 * 闭环的核心是三步：
 * 1. 读取目标 RPM 和实际 RPM。
 * 2. 用 PID 根据误差算出输出百分比。
 * 3. 把输出百分比交给 Motor_SetSpeed()。
 */
void App_MotorTask(void)
{
  PID_Controller_t speed_pid;
  int16_t previous_target_rpm = 0;

  Motor_Init();
  PID_Init(&speed_pid,
           APP_PID_KP,
           APP_PID_KI,
           APP_PID_KD,
           (float)APP_SPEED_MIN,
           (float)APP_SPEED_MAX);

  for (;;)
  {
    int16_t target_rpm_snapshot;
    int16_t actual_rpm_snapshot;
    int16_t output_speed;

    App_ReadControlSnapshot(&target_rpm_snapshot, &actual_rpm_snapshot);

    if (target_rpm_snapshot == 0)
    {
      /*
       * 目标为 0 时直接停机，并清掉积分。
       * 这样下次重新启动不会带着上一次累积的积分量突然冲一下。
       */
      PID_Reset(&speed_pid);
      output_speed = 0;
      previous_target_rpm = 0;
      App_PublishVofaSnapshot((float)target_rpm_snapshot,
                              (float)actual_rpm_snapshot,
                              (float)output_speed,
                              (float)(target_rpm_snapshot - actual_rpm_snapshot),
                              0.0f,
                              0.0f,
                              0.0f);
    }
    else
    {
      float output_float;

      if (((previous_target_rpm > 0) && (target_rpm_snapshot < 0)) ||
          ((previous_target_rpm < 0) && (target_rpm_snapshot > 0)))
      {
        /* 正反转方向切换时清积分，避免旧方向的积分继续影响新方向。 */
        PID_Reset(&speed_pid);
      }

      output_float = PID_Update(&speed_pid,
                                (float)target_rpm_snapshot,
                                (float)actual_rpm_snapshot,
                                (float)APP_MOTOR_UPDATE_PERIOD_MS / 1000.0f);
      output_speed = App_RoundPidOutput(output_float);
      previous_target_rpm = target_rpm_snapshot;
      App_PublishVofaSnapshot((float)target_rpm_snapshot,
                              (float)actual_rpm_snapshot,
                              (float)output_speed,
                              speed_pid.last_error,
                              speed_pid.last_p_term,
                              speed_pid.last_i_term,
                              speed_pid.last_d_term);
    }

    Motor_SetSpeed(output_speed);
    App_PublishPidOutput(output_speed);
    osDelay(APP_MOTOR_UPDATE_PERIOD_MS);
  }
}

/*
 * VofaTask 专门负责把 PID 调试数据发给电脑。
 * 串口发送和控制计算分开，避免调试输出影响编码器采样节奏。
 */
void App_VofaTask(void)
{
  Vofa_PidFrame_t snapshot;

  for (;;)
  {
    App_ReadVofaSnapshot(&snapshot);
    Vofa_SendPidFrame(&snapshot);
    osDelay(APP_VOFA_SEND_PERIOD_MS);
  }
}

/* 固定的 OLED 标签只画一次，后面 OLEDTask 只刷新数字字段。 */
static void App_DrawStaticDisplay(void)
{
  OLED_ShowString(0, 0, "Cmd:");
  OLED_ShowString(60, 0, "%");
  OLED_ShowString(0, 1, "Tgt:");
  OLED_ShowString(0, 2, "Act:");
  OLED_ShowString(0, 3, "Out:");
  OLED_ShowString(60, 3, "%");
  OLED_ShowString(0, 4, "D100:");
  OLED_ShowString(0, 5, "A:");
  OLED_ShowString(30, 5, "B:");
  OLED_ShowString(0, 6, "T:");
  OLED_ShowString(42, 6, "ms");
  OLED_ShowString(0, 7, "Raw:");
}

/* 使用绘制前抓取的快照刷新 OLED 数值。 */
static void App_UpdateDisplay(const App_DisplaySnapshot_t *snapshot)
{
  OLED_ShowSignedNum(30, 0, snapshot->target_speed, 3);

  OLED_ShowSignedNum(30, 1, snapshot->target_rpm, 4);

  OLED_ShowSignedNum(30, 2, snapshot->actual_rpm, 4);

  OLED_ShowSignedNum(30, 3, snapshot->pid_output, 3);

  OLED_ShowSignedNum(36, 4, snapshot->encoder_delta_100ms, 5);

  OLED_ShowNum(12, 5, snapshot->encoder_phase_a, 1);
  OLED_ShowNum(42, 5, snapshot->encoder_phase_b, 1);

  OLED_ShowNum(12, 6, snapshot->sample_elapsed_ms, 4);

  OLED_ShowSignedNum(30, 7, snapshot->encoder_raw_delta, 5);
}

/* 在一个很短的临界区里复制所有显示字段，避免一帧数据新旧混杂。 */
static void App_ReadDisplaySnapshot(App_DisplaySnapshot_t *snapshot)
{
  taskENTER_CRITICAL();
  snapshot->target_speed = target_speed;
  snapshot->target_rpm = target_rpm;
  snapshot->actual_rpm = actual_rpm;
  snapshot->pid_output = pid_output;
  snapshot->encoder_cnt = encoder_cnt;
  snapshot->encoder_raw_delta = encoder_raw_delta;
  snapshot->encoder_delta_100ms = encoder_delta_100ms;
  snapshot->encoder_phase_a = encoder_phase_a;
  snapshot->encoder_phase_b = encoder_phase_b;
  snapshot->sample_elapsed_ms = sample_elapsed_ms;
  taskEXIT_CRITICAL();
}

/* MotorTask 只需要目标 RPM 和实际 RPM，单独做一个更小的快照函数。 */
static void App_ReadControlSnapshot(int16_t *target_rpm_snapshot,
                                    int16_t *actual_rpm_snapshot)
{
  taskENTER_CRITICAL();
  *target_rpm_snapshot = target_rpm;
  *actual_rpm_snapshot = actual_rpm;
  taskEXIT_CRITICAL();
}

/* VofaTask 读取 PID 调试快照，一次复制完整 7 个通道，避免曲线数据新旧混在一起。 */
static void App_ReadVofaSnapshot(Vofa_PidFrame_t *snapshot)
{
  taskENTER_CRITICAL();
  snapshot->target_rpm = vofa_target_rpm;
  snapshot->actual_rpm = vofa_actual_rpm;
  snapshot->pid_output = vofa_pid_output;
  snapshot->error = vofa_error;
  snapshot->p_term = vofa_p_term;
  snapshot->i_term = vofa_i_term;
  snapshot->d_term = vofa_d_term;
  taskEXIT_CRITICAL();
}

/* 发布一组完整的编码器采样结果，供 OLEDTask 显示，也供 MotorTask 闭环使用。 */
static void App_PublishEncoderSample(int16_t cnt,
                                     int16_t raw_delta,
                                     int16_t delta_100ms,
                                     int16_t rpm,
                                     uint8_t phase_a,
                                     uint8_t phase_b,
                                     uint16_t elapsed_ms)
{
  taskENTER_CRITICAL();
  encoder_cnt = cnt;
  encoder_raw_delta = raw_delta;
  encoder_delta_100ms = delta_100ms;
  actual_rpm = rpm;
  encoder_phase_a = phase_a;
  encoder_phase_b = phase_b;
  sample_elapsed_ms = elapsed_ms;
  taskEXIT_CRITICAL();
}

/* 发布 PID 输出，让 OLED 上能看到闭环最后给了电机多少百分比。 */
static void App_PublishPidOutput(int16_t output)
{
  taskENTER_CRITICAL();
  pid_output = output;
  taskEXIT_CRITICAL();
}

/* 发布 VOFA 要显示的 PID 调试数据。 */
static void App_PublishVofaSnapshot(float target,
                                    float actual,
                                    float output,
                                    float error,
                                    float p_term,
                                    float i_term,
                                    float d_term)
{
  taskENTER_CRITICAL();
  vofa_target_rpm = target;
  vofa_actual_rpm = actual;
  vofa_pid_output = output;
  vofa_error = error;
  vofa_p_term = p_term;
  vofa_i_term = i_term;
  vofa_d_term = d_term;
  taskEXIT_CRITICAL();
}

/* 读取 KeyTask、OLEDTask 共享的目标百分比命令。 */
static int16_t App_ReadTargetSpeed(void)
{
  int16_t speed;

  taskENTER_CRITICAL();
  speed = target_speed;
  taskEXIT_CRITICAL();

  return speed;
}

/* 保存限幅后的目标速度，并同步换算出 PID 要用的目标 RPM。 */
static void App_WriteTargetSpeed(int16_t speed)
{
  int16_t rpm;

  speed = App_ClampSpeed(speed);
  rpm = App_SpeedToTargetRpm(speed);

  taskENTER_CRITICAL();
  target_speed = speed;
  target_rpm = rpm;
  taskEXIT_CRITICAL();
}

/* 把 -100..100 的目标百分比换算成目标 RPM。 */
static int16_t App_SpeedToTargetRpm(int16_t speed)
{
  int32_t rpm;

  rpm = ((int32_t)speed * (int32_t)APP_TARGET_RPM_MAX) / APP_SPEED_MAX;

  if (rpm > INT16_MAX)
  {
    return INT16_MAX;
  }

  if (rpm < INT16_MIN)
  {
    return INT16_MIN;
  }

  return (int16_t)rpm;
}

/*
 * Encoder_CalcRPM() 只算大小，不带方向。
 * raw_delta 为负说明这段时间编码器反向计数，所以这里把 RPM 补成有符号值。
 */
static int16_t App_MakeSignedRpm(uint16_t rpm, int16_t raw_delta)
{
  if (rpm > (uint16_t)INT16_MAX)
  {
    rpm = (uint16_t)INT16_MAX;
  }

  if (raw_delta < 0)
  {
    return -(int16_t)rpm;
  }

  return (int16_t)rpm;
}

/* PID 输出是 float，电机驱动接口使用整数百分比，这里做四舍五入和保险限幅。 */
static int16_t App_RoundPidOutput(float output)
{
  int16_t speed;

  if (output >= 0.0f)
  {
    output += 0.5f;
  }
  else
  {
    output -= 0.5f;
  }

  speed = (int16_t)output;
  return App_ClampSpeed(speed);
}

/*
 * 把实际采样间隔下的增量折算成等效 100 ms 增量。
 * Raw 显示真实采样计数；D100 用来在 EncoderTask 被调度器稍微延迟唤醒时，
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

/* 确保速度命令保持在 Motor_SetSpeed() 能接受的范围内。 */
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
