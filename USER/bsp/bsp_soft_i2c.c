#include "bsp_soft_i2c.h"

/*
    软件 IIC 引脚定义：

    OLED_SCL -> PC8
    OLED_SDA -> PC9

    CubeMX 中配置：
    PC8: GPIO_Output, Open Drain, Pull-up
    PC9: GPIO_Output, Open Drain, Pull-up
*/

#define SOFT_I2C_SCL_PORT     GPIOC
#define SOFT_I2C_SCL_PIN      GPIO_PIN_8

#define SOFT_I2C_SDA_PORT     GPIOC
#define SOFT_I2C_SDA_PIN      GPIO_PIN_9


#define SOFT_I2C_SCL_HIGH()   HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define SOFT_I2C_SCL_LOW()    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)

#define SOFT_I2C_SDA_HIGH()   HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define SOFT_I2C_SDA_LOW()    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)

#define SOFT_I2C_SDA_READ()   HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)


/*
    简单延时。
    软件 IIC 不需要特别精确，只要 SCL/SDA 时序不要太快即可。
*/
static void SoftI2C_Delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < 80; i++)
    {
        __NOP();
    }
}


void SoftI2C_Init(void)
{
    SOFT_I2C_SCL_HIGH();
    SOFT_I2C_SDA_HIGH();
    SoftI2C_Delay();
}


/*
    IIC 起始信号：

    SCL 为高电平时，SDA 从高变低。
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

    主机发送完 8bit 后，释放 SDA。
    从机如果应答，会把 SDA 拉低。

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
    主机发送 ACK：SDA 拉低
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
    主机发送 NACK：SDA 释放为高
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
