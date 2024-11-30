
#include "OLED_Font.h"
#include "oled.h"
uint32_t OLED_Pow(uint32_t X, uint32_t Y);  // 添加这一行
/* 引脚定义 */
#define OLED_SCL_GPIO_PORT    GPIOB
#define OLED_SCL_PIN          GPIO_PIN_8

#define OLED_SDA_GPIO_PORT    GPIOB
#define OLED_SDA_PIN          GPIO_PIN_9

/* 引脚操作宏 */
#define OLED_SCL_Set()        HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_Reset()      HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)

#define OLED_SDA_Set()        HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_Reset()      HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)

/* 延时函数，微秒级 */
void OLED_DelayUs(uint16_t us)
{
    uint32_t delay = us * (SystemCoreClock / 1000000 / 5);
    while (delay--) __NOP();
}

/* 引脚初始化 */
void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 启用GPIO时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置SCL引脚 */
    GPIO_InitStruct.Pin = OLED_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* 配置SDA引脚 */
    GPIO_InitStruct.Pin = OLED_SDA_PIN;
    HAL_GPIO_Init(OLED_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* 初始化引脚状态 */
    OLED_SCL_Set();
    OLED_SDA_Set();
}

/* I2C开始信号 */
void OLED_I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();
    OLED_DelayUs(4);
    OLED_SDA_Reset();
    OLED_DelayUs(4);
    OLED_SCL_Reset();
}

/* I2C停止信号 */
void OLED_I2C_Stop(void)
{
    OLED_SCL_Reset();
    OLED_SDA_Reset();
    OLED_DelayUs(4);
    OLED_SCL_Set();
    OLED_DelayUs(4);
    OLED_SDA_Set();
    OLED_DelayUs(4);
}

/* I2C发送一个字节 */
void OLED_I2C_SendByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_SCL_Reset();
        if (byte & 0x80)
            OLED_SDA_Set();
        else
            OLED_SDA_Reset();
        byte <<= 1;
        OLED_DelayUs(2);
        OLED_SCL_Set();
        OLED_DelayUs(2);
    }
    /* 接收应答 */
    OLED_SCL_Reset();
    OLED_SDA_Set();  // 释放SDA
    OLED_DelayUs(2);
    OLED_SCL_Set();
    OLED_DelayUs(2);
    OLED_SCL_Reset();
}

/* OLED写命令 */
void OLED_WriteCommand(uint8_t command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    // 从机地址，具体根据你的OLED模块调整
    OLED_I2C_SendByte(0x00);    // 写命令
    OLED_I2C_SendByte(command);
    OLED_I2C_Stop();
}

/* OLED写数据 */
void OLED_WriteData(uint8_t data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    // 从机地址
    OLED_I2C_SendByte(0x40);    // 写数据
    OLED_I2C_SendByte(data);
    OLED_I2C_Stop();
}

/* OLED设置光标位置 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    // 设置Y位置
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    // 设置X位置高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));           // 设置X位置低4位
}

/* OLED清屏 */
void OLED_Clear(void)
{  
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/* OLED显示字符 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{       
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);    // 设置光标位置在上半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);       // 显示上半部分内容
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);    // 设置光标位置在下半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);   // 显示下半部分内容
    }
}

/* OLED显示字符串 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/* OLED显示数字（十进制） */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)                            
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* OLED显示数字（带符号的十进制） */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)                            
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* OLED显示数字（十六进制） */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)                            
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/* OLED显示数字（二进制） */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)                            
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* OLED次方函数 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/* OLED初始化 */
void OLED_Init(void)
{
    uint32_t i, j;
    
    for (i = 0; i < 1000; i++)            // 上电延时
    {
        for (j = 0; j < 1000; j++);
    }
    
    OLED_GPIO_Init();            // 端口初始化
    
    OLED_WriteCommand(0xAE);    // 关闭显示
    
    OLED_WriteCommand(0xD5);    // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);
    
    OLED_WriteCommand(0xA8);    // 设置多路复用率
    OLED_WriteCommand(0x3F);
    
    OLED_WriteCommand(0xD3);    // 设置显示偏移
    OLED_WriteCommand(0x00);
    
    OLED_WriteCommand(0x40);    // 设置显示开始行
    
    OLED_WriteCommand(0xA1);    // 设置左右方向，0xA1正常 0xA0左右反置
    
    OLED_WriteCommand(0xC8);    // 设置上下方向，0xC8正常 0xC0上下反置
 
    OLED_WriteCommand(0xDA);    // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    
    OLED_WriteCommand(0x81);    // 设置对比度控制
    OLED_WriteCommand(0xCF);
 
    OLED_WriteCommand(0xD9);    // 设置预充电周期
    OLED_WriteCommand(0xF1);
 
    OLED_WriteCommand(0xDB);    // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);
 
    OLED_WriteCommand(0xA4);    // 设置整个显示打开/关闭
 
    OLED_WriteCommand(0xA6);    // 设置正常/倒转显示
 
    OLED_WriteCommand(0x8D);    // 设置充电泵
    OLED_WriteCommand(0x14);
 
    OLED_WriteCommand(0xAF);    // 开启显示
        
    OLED_Clear();                // OLED清屏
}
