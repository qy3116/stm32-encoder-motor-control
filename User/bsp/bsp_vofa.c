#include "bsp_vofa.h"

#include "usart.h"
#include <string.h>

#define VOFA_PID_CHANNEL_COUNT 7U
#define VOFA_TAIL_SIZE         4U

/*
 * JustFloat 协议帧尾：00 00 80 7F。
 * VOFA 看到这个帧尾后，就知道前面的字节是一组小端 float 数据。
 */
static const uint8_t vofa_tail[VOFA_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};

/* 按 VOFA JustFloat 协议发送一帧 PID 调试数据。 */
void Vofa_SendPidFrame(const Vofa_PidFrame_t *frame)
{
  float channels[VOFA_PID_CHANNEL_COUNT];
  uint8_t tx_buffer[(VOFA_PID_CHANNEL_COUNT * sizeof(float)) + VOFA_TAIL_SIZE];

  if (frame == 0)
  {
    return;
  }

  channels[0] = frame->target_rpm;
  channels[1] = frame->actual_rpm;
  channels[2] = frame->pid_output;
  channels[3] = frame->error;
  channels[4] = frame->p_term;
  channels[5] = frame->i_term;
  channels[6] = frame->d_term;

  /*
  内存拷贝函数
  */
  memcpy(tx_buffer, channels, sizeof(channels));
  memcpy(&tx_buffer[sizeof(channels)], vofa_tail, VOFA_TAIL_SIZE);

  /*
   * 32 字节在 115200 波特率下约 2.8 ms。
   * 这里放在低优先级 VofaTask 中发送，不阻塞编码器采样任务。
   */
  (void)HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 20U);
}
