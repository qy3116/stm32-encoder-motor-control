#include "pid.h"

static float PID_ClampFloat(float value, float min_value, float max_value);

/*
 * 初始化 PID 控制器。
 * kp/ki/kd 是三个参数；output_min/output_max 是最终输出范围。
 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_min = output_min;
  pid->output_max = output_max;

  PID_Reset(pid);
}

/* 清空积分和上一次误差，常用于停机或正反转切换。 */
void PID_Reset(PID_Controller_t *pid)
{
  if (pid == 0)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->first_update = 1U;
  pid->last_error = 0.0f;
  pid->last_derivative = 0.0f;
  pid->last_p_term = 0.0f;
  pid->last_i_term = 0.0f;
  pid->last_d_term = 0.0f;
  pid->last_output = 0.0f;
}

/*
 * 根据目标值和实际值计算一次 PID 输出。
 *
 * setpoint    : 目标值，例如目标 RPM。
 * measurement : 实际值，例如编码器测得的实际 RPM。
 * dt_s        : 两次计算之间的时间，单位秒。
 */
float PID_Update(PID_Controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt_s)
{
  float error;
  float derivative;
  float output;
  float p_term;
  float i_term;
  float d_term;

  if ((pid == 0) || (dt_s <= 0.0f))
  {
    return 0.0f;
  }

  error = setpoint - measurement;

  /*
   * I 项：把误差按时间累加。
   * 如果输出已经有限幅，积分也要做保护，否则长期堵转时积分会越积越大。
   */
  pid->integral += error * dt_s;

  if (pid->ki != 0.0f)
  {
    float integral_min;
    float integral_max;

    integral_min = pid->output_min / pid->ki;
    integral_max = pid->output_max / pid->ki;

    if (integral_min > integral_max)
    {
      float temp;

      temp = integral_min;
      integral_min = integral_max;
      integral_max = temp;
    }

    pid->integral = PID_ClampFloat(pid->integral, integral_min, integral_max);
  }

  /*
   * D 项：看误差变化速度。
   * 第一次计算没有“上一次误差”，所以 D 项先按 0 处理。
   */
  if (pid->first_update != 0U)
  {
    derivative = 0.0f;
    pid->first_update = 0U;
  }
  else
  {
    derivative = (error - pid->previous_error) / dt_s;
  }

  p_term = pid->kp * error;
  i_term = pid->ki * pid->integral;
  d_term = pid->kd * derivative;
  output = p_term + i_term + d_term;
  output = PID_ClampFloat(output, pid->output_min, pid->output_max);

  pid->previous_error = error;
  pid->last_error = error;
  pid->last_derivative = derivative;
  pid->last_p_term = p_term;
  pid->last_i_term = i_term;
  pid->last_d_term = d_term;
  pid->last_output = output;

  return output;
}

/* 小工具：把 float 限制在指定范围内。 */
static float PID_ClampFloat(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}
