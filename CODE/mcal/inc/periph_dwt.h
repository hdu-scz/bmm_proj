/**
 * @file periph_dwt.h
 * @brief DWT 微秒级延时对象头文件 (MCAL 层)
 * @author 
 * @date 2026-07-03
 */

#ifndef PERIPH_DWT_H
#define PERIPH_DWT_H

#include <stdint.h>

/* DWT 对象结构体定义 */
typedef struct Periph_DWT
{
    uint32_t    core_clock_hz;       // 系统核心时钟频率 (Hz)
    uint8_t     is_initialized;      // 初始化完成标志

    void     (*init)(struct Periph_DWT *self, uint32_t core_clock_hz);
    void     (*deinit)(struct Periph_DWT *self);
    void     (*delay_us)(struct Periph_DWT *self, uint32_t us);

} Periph_Dwt_t;

/* 全局 DWT 对象声明 */
extern Periph_Dwt_t periph_dwt;

#endif /* PERIPH_DWT_H */
