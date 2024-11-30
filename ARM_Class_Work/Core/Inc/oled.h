
#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"  // 使用HAL库的头文件

/* OLED驱动相关函数声明 */
void OLED_Init(void);
void OLED_Clear(void);
uint32_t OLED_Pow(uint32_t X, uint32_t Y);  // 添加这一行
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif  /* __OLED_H */
