#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#include <stdint.h>

/*
 * VOFA JustFloat 一帧发送 7 个 float：
 * 0 target_rpm : 目标转速
 * 1 actual_rpm : 实际转速
 * 2 pid_output : PID 输出百分比
 * 3 error      : 目标 - 实际
 * 4 p_term     : P 项
 * 5 i_term     : I 项
 * 6 d_term     : D 项
 */
typedef struct
{
  float target_rpm;
  float actual_rpm;
  float pid_output;
  float error;
  float p_term;
  float i_term;
  float d_term;
} Vofa_PidFrame_t;

void Vofa_SendPidFrame(const Vofa_PidFrame_t *frame);

#endif /* BSP_VOFA_H */
