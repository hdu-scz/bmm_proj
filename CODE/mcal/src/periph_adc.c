/**
 * @file periph_adc.c
 * @brief ADC 外设对象实现 (MCAL 层)
 * @author 
 * @date 2026-07-06
 */

#include "periph_adc.h"
#include "gd32f30x_adc.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"

/* ==================== ADC1 引脚映射 (PC0) ==================== */
#define ADC1_PORT   GPIOC
#define ADC1_PIN    GPIO_PIN_0
#define ADC1_CHANNEL ADC_CHANNEL_10

/* ==================== ADC2 引脚映射 (PC2) ==================== */
#define ADC2_PORT   GPIOC
#define ADC2_PIN    GPIO_PIN_2
#define ADC2_CHANNEL ADC_CHANNEL_12

/* ==================== 全局 ADC 对象实例 ==================== */
Periph_Adc_t periph_adc1;
Periph_Adc_t periph_adc2;

/* ==================== 私有函数声明 ==================== */
static void adc_init(Periph_Adc_t *self);
static void adc_deinit(Periph_Adc_t *self);
static uint16_t adc_get_value(Periph_Adc_t *self);

/* ==================== ADC 对象静态初始化 ==================== */
Periph_Adc_t periph_adc1 = {
    .adc_periph     = ADC0,              // GD32 中 ADC0 对应 ADC1
    .channel        = ADC1_CHANNEL,
    .port           = ADC1_PORT,
    .pin            = ADC1_PIN,
    .is_initialized = 0,
    .init           = adc_init,
    .deinit         = adc_deinit,
    .get_value      = adc_get_value
};

Periph_Adc_t periph_adc2 = {
    .adc_periph     = ADC0,              // GD32 中 ADC0 对应 ADC2（共用 ADC0）
    .channel        = ADC2_CHANNEL,
    .port           = ADC2_PORT,
    .pin            = ADC2_PIN,
    .is_initialized = 0,
    .init           = adc_init,
    .deinit         = adc_deinit,
    .get_value      = adc_get_value
};

/* ==================== 方法实现 ==================== */

static void adc_init(Periph_Adc_t *self)
{
    self->is_initialized = 0;

    /* 使能 ADC 和 GPIO 时钟 */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* 配置 GPIO 为模拟输入 */
    gpio_init(self->port, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, self->pin);

    /* ADC 参数配置 */
    adc_deinit(self->adc_periph);
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(self->adc_periph, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(self->adc_periph, ADC_SCAN_MODE, DISABLE);
    adc_data_alignment_config(self->adc_periph, ADC_DATAALIGN_RIGHT);

    /* 配置通道 */
    adc_channel_length_config(self->adc_periph, ADC_REGULAR_CHANNEL, 1);
    adc_regular_channel_config(self->adc_periph, 0, self->channel, ADC_SAMPLETIME_55POINT5);

    /* 使能 ADC */
    adc_enable(self->adc_periph);
    /* 等待 ADC 稳定 */
    for (volatile uint32_t i = 0; i < 1000; i++);

    /* 校准 ADC */
    adc_calibration_enable(self->adc_periph);

    self->is_initialized = 1;
}

static void adc_deinit(Periph_Adc_t *self)
{
    adc_deinit(self->adc_periph);
    self->is_initialized = 0;
}

static uint16_t adc_get_value(Periph_Adc_t *self)
{
    if (!self->is_initialized) return 0;

    adc_regular_channel_config(self->adc_periph, 0, self->channel, ADC_SAMPLETIME_55POINT5);
    adc_software_trigger_enable(self->adc_periph, ADC_REGULAR_CHANNEL);

    while (!adc_flag_get(self->adc_periph, ADC_FLAG_EOC));
    adc_flag_clear(self->adc_periph, ADC_FLAG_EOC);

    return (uint16_t)adc_regular_data_read(self->adc_periph);
}