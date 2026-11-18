/**
 * @file periph_adc.c
 * @brief ADC0 外设对象实现 (MCAL 层)
 * @author
 * @date 2026-07-06
 *
 * APB2 = 48 MHz，ADC 时钟取 APB2/4 = 12 MHz。
 * 常规序列使用软件触发（ETSRC = NONE）。
 * 感测脚模拟输入由器件通过 periph_io 配置，本对象只做 ADC 转换。
 */

#include "stdinc.h"
#include "periph_adc.h"
#include "periph_dwt.h"
#include "gd32f30x_adc.h"
#include "gd32f30x_rcu.h"

#define ADC0_APB2_CLOCK_HZ      48000000UL
#define ADC0_CLOCK_HZ           12000000UL
#define ADC0_RCU_PSC            RCU_CKADC_CKAPB2_DIV4

#if ((ADC0_APB2_CLOCK_HZ / 4UL) != ADC0_CLOCK_HZ)
#error "ADC0_CLOCK_HZ must be APB2 / 4"
#endif

#define ADC0_STAB_DELAY_US      1000U
#define ADC0_EOC_TIMEOUT_US     1000U

static void periph_adc_init(Periph_Adc_t *self);
static void periph_adc_deinit(Periph_Adc_t *self);
static int8_t periph_adc_get_value(Periph_Adc_t *self, uint8_t channel, uint16_t *value);

static void adc0_init(void) { periph_adc_init(&periph_adc0); }
static void adc0_deinit(void) { periph_adc_deinit(&periph_adc0); }
static int8_t adc0_get_value(uint8_t channel, uint16_t *value){return periph_adc_get_value(&periph_adc0, channel, value);}

Periph_Adc_t periph_adc0 = {
    .adc_periph     = ADC0,
    .is_initialized = 0,
    .user_count     = 0,
    .init           = adc0_init,
    .deinit         = adc0_deinit,
    .get_value      = adc0_get_value
};

static void periph_adc_init(Periph_Adc_t *self)
{
    if (self == NULL) {
        return;
    }

    if (self->is_initialized != 0U) {
        if (self->user_count < 255U) {
            self->user_count++;
        }
        return;
    }

    rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(ADC0_RCU_PSC);

    adc_deinit(self->adc_periph);
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(self->adc_periph, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(self->adc_periph, ADC_SCAN_MODE, DISABLE);
    adc_data_alignment_config(self->adc_periph, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(self->adc_periph, ADC_ROUTINE_CHANNEL, 1U);

    adc_external_trigger_source_config(self->adc_periph, ADC_ROUTINE_CHANNEL,
                                       ADC0_1_2_EXTTRIG_ROUTINE_NONE);
    adc_external_trigger_config(self->adc_periph, ADC_ROUTINE_CHANNEL, ENABLE);

    adc_enable(self->adc_periph);
    periph_dwt.delay_us(ADC0_STAB_DELAY_US);
    adc_calibration_enable(self->adc_periph);

    self->user_count = 1U;
    self->is_initialized = 1U;
}

static void periph_adc_deinit(Periph_Adc_t *self)
{
    if ((self == NULL) || (self->is_initialized == 0U)) {
        return;
    }

    if (self->user_count > 1U) {
        self->user_count--;
        return;
    }

    adc_deinit(self->adc_periph);
    self->user_count = 0U;
    self->is_initialized = 0U;
}

static int8_t periph_adc_get_value(Periph_Adc_t *self, uint8_t channel, uint16_t *value)
{
    uint32_t wait_us;

    if ((self == NULL) || (self->is_initialized == 0U) || (value == NULL)) {
        return 0;
    }

    adc_routine_channel_config(self->adc_periph, 0U, channel, ADC_SAMPLETIME_55POINT5);
    adc_software_trigger_enable(self->adc_periph, ADC_ROUTINE_CHANNEL);

    wait_us = ADC0_EOC_TIMEOUT_US;
    while (adc_flag_get(self->adc_periph, ADC_FLAG_EOC) == RESET) {
        if (wait_us == 0U) {
            return -1;
        }
        periph_dwt.delay_us(1U);
        wait_us--;
    }

    adc_flag_clear(self->adc_periph, ADC_FLAG_EOC);
    *value = adc_routine_data_read(self->adc_periph);
    return 1;
}
