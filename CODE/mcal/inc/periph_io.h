/**
 * @file periph_io.h
 * @brief GPIO 通用 IO 对象头文件 (MCAL 层)
 * @author 
 * @date 2026-07-06
 */

#ifndef PERIPH_IO_H
#define PERIPH_IO_H

#include <stdint.h>
#include "gd32f30x.h"

/* ==================== GPIO 模式枚举 ==================== */
typedef enum {
    IO_MODE_INPUT_FLOATING,      // 浮空输入
    IO_MODE_INPUT_PULLUP,        // 上拉输入
    IO_MODE_INPUT_PULLDOWN,      // 下拉输入
    IO_MODE_OUTPUT_PP,           // 推挽输出
    IO_MODE_OUTPUT_OD,           // 开漏输出
    IO_MODE_AF_PP,               // 复用推挽
    IO_MODE_AF_OD,               // 复用开漏
    IO_MODE_ANALOG               // 模拟输入
} IoMode_t;

/* ==================== GPIO 对象结构体定义 ==================== */
typedef struct Periph_Gpio
{
    uint32_t    port;                // GPIO 端口 (GPIOA, GPIOB, ...)
    uint32_t    pin;                 // GPIO 引脚号
    IoMode_t    mode;                // 当前模式
    uint8_t     is_initialized;      // 初始化完成标志

    void     (*init)(struct Periph_Gpio *self, uint32_t port, uint32_t pin, IoMode_t mode);
    void     (*deinit)(struct Periph_Gpio *self);

    void     (*write)(struct Periph_Gpio *self, uint8_t level);
    uint8_t  (*read)(struct Periph_Gpio *self);

    void     (*toggle)(struct Periph_Gpio *self);

} Periph_Gpio_t;

/* ==================== IO 数量宏定义 ==================== */
#define IO_NUM   18

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
    IO_PB12,   // CS0
    IO_PC4,    // CS1
    IO_PB1,    // CS2
    IO_PA2,    // CS3
    IO_MAX
} IoIndex_t;

/* ==================== IO 引脚数组声明 ==================== */
extern Periph_Gpio_t periph_io[IO_NUM];   
/* ==================== 通过枚举获取 IO 指针 ==================== */
Periph_Gpio_t* io_get(IoIndex_t index);

#endif /* PERIPH_IO_H */
