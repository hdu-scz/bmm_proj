/**
 * @file periph_adc.h
 * @brief ADC 外设对象头文件 (MCAL 层)
 * @author 
 * @date 2026-07-06
 */

#ifndef PERIPH_ADC_H
#define PERIPH_ADC_H

#include <stdint.h>
#include "gd32f30x.h"

/* ==================== ADC 对象结构体定义 ==================== */
typedef struct Periph_Adc
{
    uint32_t    adc_periph;          // ADC 外设基地址 (ADC0 / ADC1 / ADC2)
    uint8_t     channel;             // ADC 通道号
    uint32_t    port;                // GPIO 端口
    uint32_t    pin;                 // GPIO 引脚
    uint8_t     is_initialized;      // 初始化完成标志

    void     (*init)(struct Periph_Adc *self);
    void     (*deinit)(struct Periph_Adc *self);

    uint16_t (*get_value)(struct Periph_Adc *self);

} Periph_Adc_t;

/* 全局 ADC 对象声明 */
extern Periph_Adc_t periph_adc1;   // ADC1, PC0
extern Periph_Adc_t periph_adc2;   // ADC2, PC2

#endif /* PERIPH_ADC_H */
