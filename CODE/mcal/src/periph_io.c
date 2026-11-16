/**
 * @file periph_io.c
 * @brief GPIO 通用 IO 对象实现 (MCAL 层)
 * @author 
 * @date 2026-07-06
 */

#include "periph_io.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_rcu.h"

/* ==================== 全局 GPIO 对象数组（18 个引脚） ==================== */
Periph_Gpio_t periph_io[IO_NUM];

/* ==================== 私有函数声明 ==================== */
static void gpio_init_func(Periph_Gpio_t *self, uint32_t port, uint32_t pin, IoMode_t mode);
static void gpio_deinit(Periph_Gpio_t *self);
static void gpio_write(Periph_Gpio_t *self, uint8_t level);
static uint8_t gpio_read(Periph_Gpio_t *self);
static void gpio_toggle(Periph_Gpio_t *self);

/* ==================== GPIO 对象数组静态初始化（18 个引脚） ==================== */
Periph_Gpio_t periph_io[IO_NUM] = {
    /* 0  */ { .port = GPIOC, .pin = GPIO_PIN_1,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 1  */ { .port = GPIOA, .pin = GPIO_PIN_1,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 2  */ { .port = GPIOA, .pin = GPIO_PIN_3,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 3  */ { .port = GPIOA, .pin = GPIO_PIN_4,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 4  */ { .port = GPIOB, .pin = GPIO_PIN_0,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 5  */ { .port = GPIOB, .pin = GPIO_PIN_3,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 6  */ { .port = GPIOA, .pin = GPIO_PIN_15, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 7  */ { .port = GPIOA, .pin = GPIO_PIN_12, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 8  */ { .port = GPIOA, .pin = GPIO_PIN_11, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 9  */ { .port = GPIOA, .pin = GPIO_PIN_8,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 10 */ { .port = GPIOC, .pin = GPIO_PIN_9,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 11 */ { .port = GPIOC, .pin = GPIO_PIN_8,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 12 */ { .port = GPIOC, .pin = GPIO_PIN_7,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 13 */ { .port = GPIOC, .pin = GPIO_PIN_6,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 14 */ { .port = GPIOB, .pin = GPIO_PIN_12, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,   // IO_PB12 (CS0)
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 15 */ { .port = GPIOC, .pin = GPIO_PIN_4,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,   // IO_PC4  (CS1)
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 16 */ { .port = GPIOB, .pin = GPIO_PIN_1,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,   // IO_PB1  (CS2)
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
    /* 17 */ { .port = GPIOA, .pin = GPIO_PIN_2,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0,   // IO_PA2  (CS3)
               .init = gpio_init_func, .deinit = gpio_deinit, .write = gpio_write, .read = gpio_read, .toggle = gpio_toggle },
};

/* ==================== 方法实现 ==================== */

static void gpio_init_func(Periph_Gpio_t *self, uint32_t port, uint32_t pin, IoMode_t mode)
{
    self->port = port;
    self->pin  = pin;
    self->mode = mode;
    self->is_initialized = 0;

    /* 使能 GPIO 时钟 */
    if (port == GPIOA) rcu_periph_clock_enable(RCU_GPIOA);
    else if (port == GPIOB) rcu_periph_clock_enable(RCU_GPIOB);
    else if (port == GPIOC) rcu_periph_clock_enable(RCU_GPIOC);
    else if (port == GPIOD) rcu_periph_clock_enable(RCU_GPIOD);

    uint32_t gpio_mode = GPIO_MODE_AIN;

    switch (mode) {
        case IO_MODE_INPUT_FLOATING:  gpio_mode = GPIO_MODE_IN_FLOATING; break;
        case IO_MODE_INPUT_PULLUP:    gpio_mode = GPIO_MODE_IPU; break;
        case IO_MODE_INPUT_PULLDOWN:  gpio_mode = GPIO_MODE_IPD; break;
        case IO_MODE_OUTPUT_PP:       gpio_mode = GPIO_MODE_OUT_PP; break;
        case IO_MODE_OUTPUT_OD:       gpio_mode = GPIO_MODE_OUT_OD; break;
        case IO_MODE_AF_PP:           gpio_mode = GPIO_MODE_AF_PP; break;
        case IO_MODE_AF_OD:           gpio_mode = GPIO_MODE_AF_OD; break;
        case IO_MODE_ANALOG:          gpio_mode = GPIO_MODE_AIN; break;
    }

    gpio_init(port, gpio_mode, GPIO_OSPEED_50MHZ, pin);
    self->is_initialized = 1;
}

static void gpio_deinit(Periph_Gpio_t *self)
{
    gpio_init(self->port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->pin);
    self->is_initialized = 0;
}

static void gpio_write(Periph_Gpio_t *self, uint8_t level)
{
    if (!self->is_initialized) return;

    if (level) {
        gpio_bit_set(self->port, self->pin);
    } else {
        gpio_bit_reset(self->port, self->pin);
    }
}

static uint8_t gpio_read(Periph_Gpio_t *self)
{
    if (!self->is_initialized) return 0;
    return gpio_input_bit_get(self->port, self->pin);
}

static void gpio_toggle(Periph_Gpio_t *self)
{
    if (!self->is_initialized) return;
    gpio_bit_write(self->port, self->pin, (gpio_input_bit_get(self->port, self->pin) == RESET) ? SET : RESET);
}

/* ==================== 通过枚举获取 IO 指针 ==================== */
Periph_Gpio_t* mcal_io_get(IoIndex_t index)
{
    if (index >= IO_MAX) return NULL;
    return &periph_io[index];
}