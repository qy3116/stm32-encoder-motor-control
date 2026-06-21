#ifndef PID_H
#define PID_H

#include <stdint.h>

/*
 * 一个最基础的 PID 控制器。
 *
 * setpoint  : 目标值，比如目标转速 RPM。
 * measurement: 实际值，比如编码器算出来的实际 RPM。
 * output    : 控制输出，比如电机速度百分比 -100..100。
 */
typedef struct
{
  float kp;
  float ki;
  float kd;

  float output_min;
  float output_max;

  float integral;
  float previous_error;
  uint8_t first_update;

  /*
   * 下面这些是调试用快照。
   * 控制本身只需要上面的状态；这些字段方便 OLED/VOFA 观察 PID 每一项。
   */
  float last_error;
  float last_derivative;
  float last_p_term;
  float last_i_term;
  float last_d_term;
  float last_output;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);
void PID_Reset(PID_Controller_t *pid);
float PID_Update(PID_Controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt_s);

#endif /* PID_H */
