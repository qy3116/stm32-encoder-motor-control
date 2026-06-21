#ifndef APP_H
#define APP_H

/* App_Init() 在任务启动前复位共享软件状态。 */
void App_Init(void);

/* 下面每个函数都由一个 FreeRTOS 任务调用，进入后不会主动返回。 */
void App_EncoderTask(void);
void App_OLEDTask(void);
void App_KeyTask(void);
void App_MotorTask(void);
void App_VofaTask(void);

#endif /* APP_H */
