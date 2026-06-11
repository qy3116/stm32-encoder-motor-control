/*
 * ============================================================
 * STM32F103RCT6 电机项目接口对应表
 * ============================================================
 *
 * 一、TB6612FNG 电机驱动模块
 *
 * TB6612 VM    -> 外部 12V 电源正极
 * TB6612 VCC   -> STM32 3.3V
 * TB6612 GND   -> STM32 GND / 12V电源负极，共地
 *
 * TB6612 PWMA  -> PA6  / TIM3_CH1 / PWM调速
 * TB6612 AIN1  -> PB0  / GPIO_Output / 方向控制1
 * TB6612 AIN2  -> PB1  / GPIO_Output / 方向控制2
 * TB6612 STBY  -> PB10 / GPIO_Output / TB6612使能，高电平工作
 *
 * TB6612 AO1   -> 电机动力线1
 * TB6612 AO2   -> 电机动力线2
 *
 *
 * 二、JGB37-520 霍尔编码器电机
 *
 * 电机红线      -> TB6612 AO1 或 AO2
 * 电机白线      -> TB6612 AO2 或 AO1
 *
 * 编码器蓝线    -> STM32 3.3V
 * 编码器黑线    -> GND
 * 编码器黄线    -> PB6 / TIM4_CH1 / 编码器A相
 * 编码器绿线    -> PB7 / TIM4_CH2 / 编码器B相
 *
 *
 * 三、0.96寸四针 IIC OLED，软件IIC
 *
 * OLED GND     -> GND
 * OLED VCC     -> STM32 3.3V
 * OLED SCL     -> PC8 / GPIO_Output Open Drain / 软件IIC时钟线
 * OLED SDA     -> PC9 / GPIO_Output Open Drain / 软件IIC数据线
 *
 *
 * 四、串口调试 USART1
 *
 * CH340 TXD    -> PA10 / USART1_RX
 * CH340 RXD    -> PA9  / USART1_TX
 *
 * 电脑通过开发板 ISP/CH340 USB 口查看串口打印。
 *
 *
 * 五、SWD下载调试接口
 *
 * ST-Link SWDIO -> PA13 / SWDIO
 * ST-Link SWCLK -> PA14 / SWCLK
 * ST-Link GND   -> GND
 * ST-Link 3.3V  -> 3.3V，可选，用于电压检测
 *
 *
 * 六、供电和共地注意事项
 *
 * 1. STM32 GND、TB6612 GND、12V电源负极、编码器GND、OLED GND 必须全部连通。
 * 2. TB6612 的 VM 接 12V，VCC 接 3.3V，不能接反。
 * 3. 面包板上下电源轨可能是断开的，使用前用万用表蜂鸣档确认是否导通。
 * 4. PA13、PA14 是 SWD 下载调试口，不要配置成普通GPIO。
 * 5. CubeMX 中 SYS -> Debug 必须选择 Serial Wire。
 *
 * ============================================================
 */