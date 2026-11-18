/**
 * @file periph_io.h
 * @brief GPIO 通用 IO 对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-06
 */

#ifndef PERIPH_IO_H
#define PERIPH_IO_H

#include "stdinc.h"

/* ==================== IO 数量宏定义 ==================== */
#define IO_NUM   20

/* ==================== IO 编号枚举 ==================== */
typedef enum {
    IO_PC1 = 0,
    IO_PA1,
    IO_PA3,
    IO_PA4,
    IO_PB0,
    IO_PB3,
    IO_PA15,
    IO_PA12,
    IO_PA11,
    IO_PA8,
    IO_PC9,
    IO_PC8,
    IO_PC7,
    IO_PC6,
    IO_PB12,   /* CS0 */
    IO_PC4,    /* CS1 */
    IO_PB1,    /* CS2 */
    IO_PA2,    /* CS3 */
    IO_PC3,    /* ADC_CHANNEL_13 */
    IO_PC5,    /* ADC_CHANNEL_15 */
    IO_MAX
} IoIndex_t;

/* ==================== GPIO 模式枚举 ==================== */
typedef enum {
    IO_MODE_INPUT_FLOATING,      /* 浮空输入 */
    IO_MODE_INPUT_PULLUP,        /* 上拉输入 */
    IO_MODE_INPUT_PULLDOWN,      /* 下拉输入 */
    IO_MODE_OUTPUT_PP,           /* 推挽输出 */
    IO_MODE_OUTPUT_OD,           /* 开漏输出 */
    IO_MODE_AF_PP,               /* 复用推挽 */
    IO_MODE_AF_OD,               /* 复用开漏 */
    IO_MODE_ANALOG               /* 模拟输入 */
} IoMode_t;

/* ==================== GPIO 对象结构体定义 ==================== */
typedef struct Periph_Gpio
{
    uint32_t    port;
    uint32_t    pin;
    IoMode_t    mode;
    uint8_t     is_initialized;

    void     (*init)(IoMode_t mode);
    void     (*deinit)(void);

    void     (*write)(uint8_t level);
    /* 高电平 1，低电平 0，未初始化 -1 */
    int8_t   (*read)(void);

    void     (*toggle)(void);

} Periph_Gpio_t;

extern Periph_Gpio_t periph_io[IO_NUM];
Periph_Gpio_t* mcal_io_get(IoIndex_t index);

#endif /* PERIPH_IO_H */
