#include "bsp_soft_i2c.h"

/*
    软件 IIC/I2C 驱动说明：

    本文件通过普通 GPIO 模拟 I2C 时序，不使用 STM32 硬件 I2C 外设。
    I2C 总线空闲时 SCL/SDA 都保持高电平；在开漏模式下，写高电平等价于释放总线。
    数据线 SDA 只在 SCL 低电平期间改变，在 SCL 高电平期间保持稳定供对端采样。

    引脚定义：

    OLED_SCL -> PC8
    OLED_SDA -> PC9

    CubeMX 中配置：
    PC8: GPIO_Output, Open Drain, Pull-up
    PC9: GPIO_Output, Open Drain, Pull-up

    如果外设板上没有上拉电阻，需要确认内部上拉是否足够；高速或长线场景建议使用外部上拉。
*/

#define SOFT_I2C_SCL_PORT     GPIOC
#define SOFT_I2C_SCL_PIN      GPIO_PIN_8

#define SOFT_I2C_SDA_PORT     GPIOC
#define SOFT_I2C_SDA_PIN      GPIO_PIN_9


/*
    GPIO 操作封装：
    SCL/SDA 配成开漏输出，SDA_HIGH 实际是释放 SDA，总线会被上拉为高电平。
    这样从机才能在 ACK 或读数据时把 SDA 拉低。
*/
#define SOFT_I2C_SCL_HIGH()   HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define SOFT_I2C_SCL_LOW()    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)

#define SOFT_I2C_SDA_HIGH()   HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define SOFT_I2C_SDA_LOW()    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)

#define SOFT_I2C_SDA_READ()   HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)


/*
    简单延时。
    每次 SCL/SDA 翻转后都调用它，给电平留出稳定时间。
    软件 IIC 不需要特别精确，只要 SCL/SDA 时序不要太快即可。
    如果总线波形过快或 OLED 应答不稳定，可适当增大循环次数。
*/
static void SoftI2C_Delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < 80; i++)
    {
        __NOP();
    }
}


/*
    初始化软件 I2C 总线。
    总线空闲状态要求 SCL/SDA 都为高电平，所以这里先释放两根线。
    GPIO 模式应已在 CubeMX 生成的 MX_GPIO_Init() 中完成配置。
*/
void SoftI2C_Init(void)
{
    SOFT_I2C_SCL_HIGH();
    SOFT_I2C_SDA_HIGH();
    SoftI2C_Delay();
}


/*
    IIC 起始信号：

    SCL 为高电平时，SDA 从高变低。
    起始信号之后拉低 SCL，方便后续在 SCL 低电平期间准备第一个数据位。
*/
void SoftI2C_Start(void)
{
    SOFT_I2C_SDA_HIGH();
    SOFT_I2C_SCL_HIGH();
    SoftI2C_Delay();

    SOFT_I2C_SDA_LOW();
    SoftI2C_Delay();

    SOFT_I2C_SCL_LOW();
    SoftI2C_Delay();
}


/*
    IIC 停止信号：

    SCL 为高电平时，SDA 从低变高。
    停止后 SDA/SCL 都回到高电平，总线重新进入空闲状态。
*/
void SoftI2C_Stop(void)
{
    SOFT_I2C_SDA_LOW();
    SOFT_I2C_SCL_LOW();
    SoftI2C_Delay();

    SOFT_I2C_SCL_HIGH();
    SoftI2C_Delay();

    SOFT_I2C_SDA_HIGH();
    SoftI2C_Delay();
}


/*
    等待从机 ACK。

    ACK 是第 9 个时钟周期：主机发送完 8bit 后释放 SDA，再拉高 SCL 采样。
    从机如果应答，会把 SDA 拉低；如果 SDA 仍为高电平，则表示未应答。
    超时后主动发送 STOP，防止总线一直卡在等待状态。

    返回值：
    0：收到 ACK
    1：未收到 ACK
*/
uint8_t SoftI2C_WaitAck(void)
{
    uint8_t timeout = 0;

    SOFT_I2C_SDA_HIGH();      // 释放 SDA
    SoftI2C_Delay();

    SOFT_I2C_SCL_HIGH();
    SoftI2C_Delay();

    while (SOFT_I2C_SDA_READ() == GPIO_PIN_SET)
    {
        timeout++;

        if (timeout > 250)
        {
            SoftI2C_Stop();
            return 1;
        }
    }

    SOFT_I2C_SCL_LOW();
    SoftI2C_Delay();

    return 0;
}


/*
    主机发送 ACK：SDA 拉低。
    用于连续读取多个字节，告诉从机后面还要继续读。
*/
void SoftI2C_SendAck(void)
{
    SOFT_I2C_SCL_LOW();
    SOFT_I2C_SDA_LOW();
    SoftI2C_Delay();

    SOFT_I2C_SCL_HIGH();
    SoftI2C_Delay();

    SOFT_I2C_SCL_LOW();
    SoftI2C_Delay();

    SOFT_I2C_SDA_HIGH();
}


/*
    主机发送 NACK：SDA 释放为高。
    通常用于读取最后一个字节，告诉从机本次读取结束。
*/
void SoftI2C_SendNack(void)
{
    SOFT_I2C_SCL_LOW();
    SOFT_I2C_SDA_HIGH();
    SoftI2C_Delay();

    SOFT_I2C_SCL_HIGH();
    SoftI2C_Delay();

    SOFT_I2C_SCL_LOW();
    SoftI2C_Delay();
}


/*
    发送 1 字节，高位先发。
    每一位都是先在 SCL 低电平期间设置 SDA，再拉高 SCL 让从机采样。
    发送完 8 位后释放 SDA，下一拍交给从机返回 ACK。
*/
void SoftI2C_SendByte(uint8_t data)
{
    uint8_t i;

    SOFT_I2C_SCL_LOW();

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            SOFT_I2C_SDA_HIGH();
        }
        else
        {
            SOFT_I2C_SDA_LOW();
        }

        data <<= 1;

        SoftI2C_Delay();

        SOFT_I2C_SCL_HIGH();
        SoftI2C_Delay();

        SOFT_I2C_SCL_LOW();
        SoftI2C_Delay();
    }

    SOFT_I2C_SDA_HIGH();      // 释放 SDA，准备读取 ACK
}


/*
    读取 1 字节。

    主机先释放 SDA，由从机驱动数据线；主机在 SCL 高电平期间读取当前位。
    每次读取后由 ack 参数决定第 9 个时钟周期返回 ACK 还是 NACK。

    ack = 1：读完后发送 ACK
    ack = 0：读完后发送 NACK
*/
uint8_t SoftI2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    SOFT_I2C_SDA_HIGH();      // 释放 SDA

    for (i = 0; i < 8; i++)
    {
        data <<= 1;

        SOFT_I2C_SCL_LOW();
        SoftI2C_Delay();

        SOFT_I2C_SCL_HIGH();
        SoftI2C_Delay();

        if (SOFT_I2C_SDA_READ() == GPIO_PIN_SET)
        {
            data |= 0x01;
        }
    }

    SOFT_I2C_SCL_LOW();

    if (ack)
    {
        SoftI2C_SendAck();
    }
    else
    {
        SoftI2C_SendNack();
    }

    return data;
}
