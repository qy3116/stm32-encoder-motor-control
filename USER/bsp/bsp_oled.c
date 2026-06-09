#include "bsp_oled.h"
#include "bsp_soft_i2c.h"
#include "oledfont.h"

#define OLED_ADDR      0x78    // 0x3C << 1


void OLED_WriteCmd(uint8_t cmd)
{
    SoftI2C_Start();

    SoftI2C_SendByte(OLED_ADDR);
    SoftI2C_WaitAck();

    SoftI2C_SendByte(0x00);    // 0x00 表示后面发送的是命令
    SoftI2C_WaitAck();

    SoftI2C_SendByte(cmd);
    SoftI2C_WaitAck();

    SoftI2C_Stop();
}


void OLED_WriteData(uint8_t data)
{
    SoftI2C_Start();

    SoftI2C_SendByte(OLED_ADDR);
    SoftI2C_WaitAck();

    SoftI2C_SendByte(0x40);    // 0x40 表示后面发送的是数据
    SoftI2C_WaitAck();

    SoftI2C_SendByte(data);
    SoftI2C_WaitAck();

    SoftI2C_Stop();
}


void OLED_SetCursor(uint8_t page, uint8_t col)
{
    OLED_WriteCmd(0xB0 + page);                 // 设置页地址，page范围 0~7
    OLED_WriteCmd(0x00 + (col & 0x0F));         // 设置列地址低4位
    OLED_WriteCmd(0x10 + ((col >> 4) & 0x0F));  // 设置列地址高4位
}


void OLED_Clear(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0; page < 8; page++)
    {
        OLED_SetCursor(page, 0);

        for (col = 0; col < 128; col++)
        {
            OLED_WriteData(0x00);
        }
    }
}


void OLED_Init(void)
{
    HAL_Delay(100);

    SoftI2C_Init();

    OLED_WriteCmd(0xAE);    // 关闭显示

    OLED_WriteCmd(0x20);    // 设置内存地址模式
    OLED_WriteCmd(0x10);    // Page Addressing Mode

    OLED_WriteCmd(0xB0);    // 设置页地址

    OLED_WriteCmd(0xC8);    // COM扫描方向
    OLED_WriteCmd(0x00);    // 列低地址
    OLED_WriteCmd(0x10);    // 列高地址

    OLED_WriteCmd(0x40);    // 起始行地址

    OLED_WriteCmd(0x81);    // 对比度
    OLED_WriteCmd(0x7F);

    OLED_WriteCmd(0xA1);    // 段重映射
    OLED_WriteCmd(0xA6);    // 正常显示，不反色

    OLED_WriteCmd(0xA8);    // 多路复用率
    OLED_WriteCmd(0x3F);

    OLED_WriteCmd(0xA4);    // 输出遵循RAM内容

    OLED_WriteCmd(0xD3);    // 显示偏移
    OLED_WriteCmd(0x00);

    OLED_WriteCmd(0xD5);    // 显示时钟分频
    OLED_WriteCmd(0x80);

    OLED_WriteCmd(0xD9);    // 预充电周期
    OLED_WriteCmd(0xF1);

    OLED_WriteCmd(0xDA);    // COM引脚配置
    OLED_WriteCmd(0x12);

    OLED_WriteCmd(0xDB);    // VCOMH
    OLED_WriteCmd(0x40);

    OLED_WriteCmd(0x8D);    // 电荷泵
    OLED_WriteCmd(0x14);

    OLED_WriteCmd(0xAF);    // 开启显示

    OLED_Clear();
}

void OLED_ShowChar(uint8_t x, uint8_t y, char chr)
{
    uint8_t i;
    uint8_t char_index;

    /*
        F6x8 字库：
        一个字符宽 6 列，高 8 像素，只占 OLED 的 1 页。

        x: 0~127
        y: 0~7
    */

    if (x > 122 || y > 7)
    {
        return;
    }

    if (chr < ' ' || chr > '~')
    {
        chr = ' ';
    }

    char_index = chr - ' ';

    OLED_SetCursor(y, x);

    for (i = 0; i < 6; i++)
    {
        OLED_WriteData(F6x8[char_index][i]);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    while (*str != '\0')
    {
        OLED_ShowChar(x, y, *str);

        x += 6;

        /*
            128 / 6 = 21 个字符左右。
            如果当前行放不下，就换下一页。
        */
        if (x > 122)
        {
            x = 0;
            y++;
        }

        if (y > 7)
        {
            break;
        }

        str++;
    }
}
