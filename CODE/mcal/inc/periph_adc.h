/**
 * @file periph_adc.h
 * @brief ADC0 外设对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-06
 *
 * 用法（由具体器件对象组合，不单独做 BSP ADC 设备）：
 *   1. 器件持有 Periph_Adc_t *（通常 &periph_adc0）及自己的 channel
 *   2. 器件用 periph_io 将感测脚配成 IO_MODE_ANALOG，再 adc->init()
 *   3. 器件读电压：get_value(channel, &raw)
 */

#ifndef PERIPH_ADC_H
#define PERIPH_ADC_H

#include "stdinc.h"

typedef struct Periph_Adc
{
    uint32_t    adc_periph;
    uint8_t     is_initialized;
    uint8_t     user_count;          /* 引用计数，供多器件共用 init/deinit */

    void     (*init)(void);
    void     (*deinit)(void);

    /**
     * @brief 软件触发采样指定通道
     * @param channel ADC 通道号，如 ADC_CHANNEL_10
     * @param value 输出原始码值，12 位右对齐
     * @retval 1 成功
     * @retval -1 转换超时
     * @retval 0 参数无效或未初始化
     */
    int8_t   (*get_value)(uint8_t channel, uint16_t *value);

} Periph_Adc_t;

extern Periph_Adc_t periph_adc0;

#endif /* PERIPH_ADC_H */
