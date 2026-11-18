/**
 * @file periph_dwt.c
 * @brief DWT 微秒级延时对象实现 (MCAL 层)
 * @author
 * @date 2026-07-03
 */

#include "periph_dwt.h"
#include "gd32f30x.h"

/* 系统内核时钟 48 MHz */
#define DWT_CORE_CLOCK_HZ    48000000UL

/* Cortex-M4 上一次减法循环大约 4 个时钟，只在 DWT 不可用时使用 */
#define DWT_SOFT_CYCLES_PER_LOOP    4UL
#define DWT_STALL_LIMIT             64UL

static void periph_dwt_init(Periph_Dwt_t *self);
static void periph_dwt_deinit(Periph_Dwt_t *self);
static void periph_dwt_delay_us(Periph_Dwt_t *self, uint32_t us);
static void periph_dwt_soft_wait(uint32_t cycles);

static void dwt_init(void) { periph_dwt_init(&periph_dwt); }
static void dwt_deinit(void) { periph_dwt_deinit(&periph_dwt); }
static void dwt_delay_us(uint32_t us) { periph_dwt_delay_us(&periph_dwt, us); }

Periph_Dwt_t periph_dwt = {
    .core_clock_hz  = DWT_CORE_CLOCK_HZ,
    .is_initialized = 0,
    .counter_ready  = 0,
    .init           = dwt_init,
    .deinit         = dwt_deinit,
    .delay_us       = dwt_delay_us
};

/**
 * @brief CYCCNT 不可用时的软件循环等待
 * @param cycles 目标内核周期数
 * @retval none
 */
static void periph_dwt_soft_wait(uint32_t cycles)
{
    volatile uint32_t left;

    left = cycles / DWT_SOFT_CYCLES_PER_LOOP;
    while (left > 0U) {
        __NOP();
        left--;
    }
}

/**
 * @brief 使能 DWT CYCCNT，并探测计数器是否在走
 * @param self DWT 对象
 * @retval none counter_ready 表示硬件延时可依赖 CYCCNT
 */
static void periph_dwt_init(Periph_Dwt_t *self)
{
    uint32_t sample;

    if (self == NULL) {
        return;
    }

    self->core_clock_hz = DWT_CORE_CLOCK_HZ;
    self->is_initialized = 0U;
    self->counter_ready = 0U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    __DSB();
    __ISB();

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();

    sample = DWT->CYCCNT;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    if (DWT->CYCCNT != sample) {
        self->counter_ready = 1U;
    }
    self->is_initialized = 1U;
}

/**
 * @brief 关闭 CYCCNT 并清标志
 * @param self DWT 对象
 * @retval none
 */
static void periph_dwt_deinit(Periph_Dwt_t *self)
{
    if (self == NULL) {
        return;
    }

    if (self->counter_ready != 0U) {
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    }
    self->counter_ready = 0U;
    self->is_initialized = 0U;
}

/**
 * @brief 微秒延时；未 init 时自动 init；CYCCNT 异常时回退软等待
 * @param self DWT 对象
 * @param us 微秒数，0 直接返回
 * @retval none
 */
static void periph_dwt_delay_us(Periph_Dwt_t *self, uint32_t us)
{
    uint32_t cycles;
    uint32_t start;
    uint32_t now;
    uint32_t previous;
    uint32_t stall;
    uint32_t done;

    if ((self == NULL) || (us == 0U) || (DWT_CORE_CLOCK_HZ < 1000000UL)) {
        return;
    }
    if (self->is_initialized == 0U) {
        periph_dwt_init(self);
    }

    cycles = us * (DWT_CORE_CLOCK_HZ / 1000000UL);
    if (self->counter_ready == 0U) {
        periph_dwt_soft_wait(cycles);
        return;
    }

    start = DWT->CYCCNT;
    previous = start;
    stall = 0U;
    while ((DWT->CYCCNT - start) < cycles) {
        now = DWT->CYCCNT;
        if (now == previous) {
            stall++;
            if (stall >= DWT_STALL_LIMIT) {
                done = now - start;
                if (done < cycles) {
                    periph_dwt_soft_wait(cycles - done);
                }
                return;
            }
        } else {
            stall = 0U;
            previous = now;
        }
    }
}
