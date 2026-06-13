#ifndef APP_H
#define APP_H

/* App_Init() is called once after CubeMX has initialized GPIO, timers, and HAL. */
void App_Init(void);

/* App_Task() is called repeatedly in while(1) and must not block for a long time. */
void App_Task(void);

#endif /* APP_H */
