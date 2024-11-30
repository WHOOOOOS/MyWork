/**
 ****************************************************************************************************
 * @file        atk_md0430_touch_iic.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0430模块触摸IIC接口驱动代码
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

#ifndef __ATK_MD0430_TOUCH_IIC_H
#define __ATK_MD0430_TOUCH_IIC_H

#include "atk_md0430.h"

#if (ATK_MD0430_USING_TOUCH != 0)

/* 引脚定义 */
#define ATK_MD0430_TOUCH_IIC_SCL_GPIO_PORT          GPIOB
#define ATK_MD0430_TOUCH_IIC_SCL_GPIO_PIN           GPIO_PIN_1
#define ATK_MD0430_TOUCH_IIC_SCL_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
#define ATK_MD0430_TOUCH_IIC_SDA_GPIO_PORT          GPIOF
#define ATK_MD0430_TOUCH_IIC_SDA_GPIO_PIN           GPIO_PIN_9
#define ATK_MD0430_TOUCH_IIC_SDA_GPIO_CLK_ENABLE()  do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* IO操作 */
#define ATK_MD0430_TOUCH_IIC_SCL(x)                 do{ x ?                                                                                                         \
                                                        HAL_GPIO_WritePin(ATK_MD0430_TOUCH_IIC_SCL_GPIO_PORT, ATK_MD0430_TOUCH_IIC_SCL_GPIO_PIN, GPIO_PIN_SET) :    \
                                                        HAL_GPIO_WritePin(ATK_MD0430_TOUCH_IIC_SCL_GPIO_PORT, ATK_MD0430_TOUCH_IIC_SCL_GPIO_PIN, GPIO_PIN_RESET);   \
                                                    }while(0)

#define ATK_MD0430_TOUCH_IIC_SDA(x)                 do{ x ?                                                                                                         \
                                                        HAL_GPIO_WritePin(ATK_MD0430_TOUCH_IIC_SDA_GPIO_PORT, ATK_MD0430_TOUCH_IIC_SDA_GPIO_PIN, GPIO_PIN_SET) :    \
                                                        HAL_GPIO_WritePin(ATK_MD0430_TOUCH_IIC_SDA_GPIO_PORT, ATK_MD0430_TOUCH_IIC_SDA_GPIO_PIN, GPIO_PIN_RESET);   \
                                                    }while(0)

#define ATK_MD0430_TOUCH_IIC_READ_SDA()             HAL_GPIO_ReadPin(ATK_MD0430_TOUCH_IIC_SDA_GPIO_PORT, ATK_MD0430_TOUCH_IIC_SDA_GPIO_PIN)

/* 错误代码 */
#define ATK_MD0430_TOUCH_IIC_EOK    0   /* 没有错误 */
#define ATK_MD0430_TOUCH_IIC_ERROR  1   /* 错误 */

/* 操作函数 */
void atk_md0430_touch_iic_init(uint8_t iic_addr);                                   /* 初始化IIC接口 */
uint8_t atk_md0430_touch_iic_write_reg(uint16_t reg, uint8_t *buf, uint8_t len);    /* 写ATK-MD0430模块触摸寄存器 */
void atk_md0430_touch_iic_read_reg(uint16_t reg, uint8_t *buf, uint8_t len);        /* 读ATK-MD0430模块触摸寄存器 */

#endif /* ATK_MD0430_USING_TOUCH */

#endif
