/**
 ****************************************************************************************************
 * @file        atk_md0430_touch.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0430模块触摸驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M144Z-M3最小系统板STM32F103版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MD0430_TOUCH_H
#define __ATK_MD0430_TOUCH_H

#include "atk_md0430.h"

#if (ATK_MD0430_USING_TOUCH != 0)

/* 引脚定义 */
#define ATK_MD0430_TOUCH_PEN_GPIO_PORT          GPIOF
#define ATK_MD0430_TOUCH_PEN_GPIO_PIN           GPIO_PIN_10
#define ATK_MD0430_TOUCH_PEN_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)
#define ATK_MD0430_TOUCH_TCS_GPIO_PORT          GPIOF
#define ATK_MD0430_TOUCH_TCS_GPIO_PIN           GPIO_PIN_11
#define ATK_MD0430_TOUCH_TCS_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* IO操作 */
#define ATK_MD0430_TOUCH_READ_PEN()             HAL_GPIO_ReadPin(ATK_MD0430_TOUCH_PEN_GPIO_PORT, ATK_MD0430_TOUCH_PEN_GPIO_PIN)
#define ATK_MD0430_TOUCH_PEN(x)                 do{ x ?                                                                                                 \
                                                    HAL_GPIO_WritePin(ATK_MD0430_TOUCH_PEN_GPIO_PORT, ATK_MD0430_TOUCH_PEN_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(ATK_MD0430_TOUCH_PEN_GPIO_PORT, ATK_MD0430_TOUCH_PEN_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)
#define ATK_MD0430_TOUCH_TCS(x)                 do{ x ?                                                                                                 \
                                                    HAL_GPIO_WritePin(ATK_MD0430_TOUCH_TCS_GPIO_PORT, ATK_MD0430_TOUCH_TCS_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(ATK_MD0430_TOUCH_TCS_GPIO_PORT, ATK_MD0430_TOUCH_TCS_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

/* ATK-MD0430触摸IIC通讯地址枚举 */
typedef enum
{
    ATK_MD0430_TOUCH_IIC_ADDR_14 = 0x14,    /* 0x14 */
    ATK_MD0430_TOUCH_IIC_ADDR_5D = 0x5D,    /* 0x5D */
} atk_md0430_touch_iic_addr_t;

/* 触摸点坐标数据结构 */
typedef struct
{
    uint16_t x;     /* 触摸点X坐标 */
    uint16_t y;     /* 触摸点Y坐标 */
    uint16_t size;  /* 触摸点大小 */
} atk_md0430_touch_point_t;

/* 错误代码 */
#define ATK_MD0430_TOUCH_EOK                    0   /* 没有错误 */
#define ATK_MD0430_TOUCH_ERROR                  1   /* 错误 */

/* 操作函数 */
uint8_t atk_md0430_touch_init(void);                                            /* ATK-MD0430模块触摸初始化 */
uint8_t atk_md0430_touch_scan(atk_md0430_touch_point_t *point, uint8_t cnt);    /* ATK-MD0430模块触摸扫描 */

#endif /* ATK_MD0430_USING_TOUCH */

#endif
