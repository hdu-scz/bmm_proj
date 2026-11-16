/**
 * @file periph_dwt.c
 * @brief DWT 微秒级延时对象实现 (MCAL 层)
 * @author 
 * @date 2026-07-03
 */

#include "periph_dwt.h"
#include "core_cm4.h"

/* ==================== 全局 DWT 对象实例 ==================== */
Periph_Dwt_t periph_dwt;

/* ==================== 私有函数声明 ==================== */
static void dwt_init(Periph_Dwt_t *self, uint32_t core_clock_hz);
static void dwt_deinit(Periph_Dwt_t *self);
static void delay_us(Periph_Dwt_t *self, uint32_t us);

/* ==================== DWT 对象静态初始化 ==================== */
Periph_Dwt_t periph_dwt = {
    .core_clock_hz = 48000000,
    .is_initialized = 0,
    .init          = dwt_init,
    .deinit        = dwt_deinit,
    .delay_us      = delay_us
};

/* ==================== 方法实现 ==================== */

static void dwt_init(Periph_Dwt_t *self, uint32_t core_clock_hz)
{
    self->core_clock_hz = core_clock_hz;
    self->is_initialized = 0;

    /* 1. 使能 DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* 2. 清零 CYCCNT */
    DWT->CYCCNT = 0;

    /* 3. 使能 CYCCNT */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    self->is_initialized = 1;
}

static void dwt_deinit(Periph_Dwt_t *self)
{
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    self->is_initialized = 0;
}

static void delay_us(Periph_Dwt_t *self, uint32_t us)
{
    if (!self->is_initialized)
    {
        self->init(self, self->core_clock_hz);
    }

    uint32_t cycles = us * (self->core_clock_hz / 1000000UL);
    uint32_t start = DWT->CYCCNT;

    /* 防止卡死*/
    while (1)
    {
        uint32_t now = DWT->CYCCNT;
        uint32_t diff = (uint32_t)(now - start);

        if (diff >= cycles)
            break;
    }
}