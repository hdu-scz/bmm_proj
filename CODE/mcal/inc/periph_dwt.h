/**
 * @file periph_dwt.h
 * @brief DWT 微秒级延时对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-03
 */

#ifndef PERIPH_DWT_H
#define PERIPH_DWT_H

#include "stdinc.h"



typedef struct Periph_DWT
{
    uint32_t    core_clock_hz;
    uint8_t     is_initialized;
    uint8_t     counter_ready;

    void     (*init)(void);
    void     (*deinit)(void);
    void     (*delay_us)(uint32_t us);

} Periph_Dwt_t;

extern Periph_Dwt_t periph_dwt;

#endif /* PERIPH_DWT_H */
