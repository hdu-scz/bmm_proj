/**
 * @file periph_io.c
 * @brief GPIO 通用 IO 对象实现 (MCAL 层)
 * @author
 * @date 2026-07-06
 */

#include "stdinc.h"
#include "periph_io.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_rcu.h"

static void periph_io_init(Periph_Gpio_t *self, IoMode_t mode);
static void periph_io_deinit(Periph_Gpio_t *self);
static void periph_io_write(Periph_Gpio_t *self, uint8_t level);
static int8_t periph_io_read(Periph_Gpio_t *self);
static void periph_io_toggle(Periph_Gpio_t *self);
static void periph_io_release_jtag(const Periph_Gpio_t *self);

/* 各引脚包装：对外方法无 self，内部绑定到 periph_io[n] */
#define DEFINE_IO_WRAPPERS(n) \
static void io##n##_init(IoMode_t mode) { periph_io_init(&periph_io[n], mode); } \
static void io##n##_deinit(void) { periph_io_deinit(&periph_io[n]); } \
static void io##n##_write(uint8_t level) { periph_io_write(&periph_io[n], level); } \
static int8_t io##n##_read(void) { return periph_io_read(&periph_io[n]); } \
static void io##n##_toggle(void) { periph_io_toggle(&periph_io[n]); }

#define IO_METHODS(n) \
    .init = io##n##_init, \
    .deinit = io##n##_deinit, \
    .write = io##n##_write, \
    .read = io##n##_read, \
    .toggle = io##n##_toggle

DEFINE_IO_WRAPPERS(0)
DEFINE_IO_WRAPPERS(1)
DEFINE_IO_WRAPPERS(2)
DEFINE_IO_WRAPPERS(3)
DEFINE_IO_WRAPPERS(4)
DEFINE_IO_WRAPPERS(5)
DEFINE_IO_WRAPPERS(6)
DEFINE_IO_WRAPPERS(7)
DEFINE_IO_WRAPPERS(8)
DEFINE_IO_WRAPPERS(9)
DEFINE_IO_WRAPPERS(10)
DEFINE_IO_WRAPPERS(11)
DEFINE_IO_WRAPPERS(12)
DEFINE_IO_WRAPPERS(13)
DEFINE_IO_WRAPPERS(14)
DEFINE_IO_WRAPPERS(15)
DEFINE_IO_WRAPPERS(16)
DEFINE_IO_WRAPPERS(17)
DEFINE_IO_WRAPPERS(18)
DEFINE_IO_WRAPPERS(19)

Periph_Gpio_t periph_io[IO_NUM] = {
    /* 0  IO_PC1  */ { .port = GPIOC, .pin = GPIO_PIN_1,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(0) },
    /* 1  IO_PA1  */ { .port = GPIOA, .pin = GPIO_PIN_1,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(1) },
    /* 2  IO_PA3  */ { .port = GPIOA, .pin = GPIO_PIN_3,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(2) },
    /* 3  IO_PA4  */ { .port = GPIOA, .pin = GPIO_PIN_4,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(3) },
    /* 4  IO_PB0  */ { .port = GPIOB, .pin = GPIO_PIN_0,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(4) },
    /* 5  IO_PB3  */ { .port = GPIOB, .pin = GPIO_PIN_3,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(5) },
    /* 6  IO_PA15 */ { .port = GPIOA, .pin = GPIO_PIN_15, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(6) },
    /* 7  IO_PA12 */ { .port = GPIOA, .pin = GPIO_PIN_12, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(7) },
    /* 8  IO_PA11 */ { .port = GPIOA, .pin = GPIO_PIN_11, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(8) },
    /* 9  IO_PA8  */ { .port = GPIOA, .pin = GPIO_PIN_8,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(9) },
    /* 10 IO_PC9  */ { .port = GPIOC, .pin = GPIO_PIN_9,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(10) },
    /* 11 IO_PC8  */ { .port = GPIOC, .pin = GPIO_PIN_8,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(11) },
    /* 12 IO_PC7  */ { .port = GPIOC, .pin = GPIO_PIN_7,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(12) },
    /* 13 IO_PC6  */ { .port = GPIOC, .pin = GPIO_PIN_6,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(13) },
    /* 14 IO_PB12 CS0 */ { .port = GPIOB, .pin = GPIO_PIN_12, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(14) },
    /* 15 IO_PC4  CS1 */ { .port = GPIOC, .pin = GPIO_PIN_4,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(15) },
    /* 16 IO_PB1  CS2 */ { .port = GPIOB, .pin = GPIO_PIN_1,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(16) },
    /* 17 IO_PA2  CS3 */ { .port = GPIOA, .pin = GPIO_PIN_2,  .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(17) },
    /* 18 IO_PC3  ADC CH13 */ { .port = GPIOC, .pin = GPIO_PIN_3, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(18) },
    /* 19 IO_PC5  ADC CH15 */ { .port = GPIOC, .pin = GPIO_PIN_5, .mode = IO_MODE_INPUT_FLOATING, .is_initialized = 0, IO_METHODS(19) },
};

/**
 * @brief PA15 / PB3 作 GPIO 时释放 JTAG，保留 SWD
 * @param self IO 对象
 * @retval none
 */
static void periph_io_release_jtag(const Periph_Gpio_t *self)
{
    if (((self->port == GPIOA) && (self->pin == GPIO_PIN_15)) ||
        ((self->port == GPIOB) && (self->pin == GPIO_PIN_3))) {
        rcu_periph_clock_enable(RCU_AF);
        gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    }
}

/**
 * @brief 配置引脚模式并置 is_initialized
 * @param self IO 对象
 * @param mode IoMode_t
 * @retval none
 */
static void periph_io_init(Periph_Gpio_t *self, IoMode_t mode)
{
    uint32_t gpio_mode = GPIO_MODE_AIN;

    self->mode = mode;
    self->is_initialized = 0;

    if (self->port == GPIOA) {
        rcu_periph_clock_enable(RCU_GPIOA);
    } else if (self->port == GPIOB) {
        rcu_periph_clock_enable(RCU_GPIOB);
    } else if (self->port == GPIOC) {
        rcu_periph_clock_enable(RCU_GPIOC);
    } else if (self->port == GPIOD) {
        rcu_periph_clock_enable(RCU_GPIOD);
    }

    periph_io_release_jtag(self);

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

    gpio_init(self->port, gpio_mode, GPIO_OSPEED_50MHZ, self->pin);
    self->is_initialized = 1;
}

/**
 * @brief 引脚恢复为浮空输入
 * @param self IO 对象
 * @retval none
 */
static void periph_io_deinit(Periph_Gpio_t *self)
{
    gpio_init(self->port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->pin);
    self->is_initialized = 0;
}

/**
 * @brief 写输出电平；未初始化则忽略
 * @param self IO 对象
 * @param level 非 0 为高
 * @retval none
 */
static void periph_io_write(Periph_Gpio_t *self, uint8_t level)
{
    if (self->is_initialized == 0U) {
        return;
    }

    if (level != 0U) {
        gpio_bit_set(self->port, self->pin);
    } else {
        gpio_bit_reset(self->port, self->pin);
    }
}

/**
 * @brief 读输入电平
 * @param self IO 对象
 * @retval 1 高，0 低，-1 未初始化
 */
static int8_t periph_io_read(Periph_Gpio_t *self)
{
    if ((self == 0) || (self->is_initialized == 0U)) {
        return -1;
    }
    if (gpio_input_bit_get(self->port, self->pin) == SET) {
        return 1;
    }
    return 0;
}

/**
 * @brief 翻转输出；未初始化则忽略
 * @param self IO 对象
 * @retval none
 */
static void periph_io_toggle(Periph_Gpio_t *self)
{
    if (self->is_initialized == 0U) {
        return;
    }
    if (gpio_input_bit_get(self->port, self->pin) == RESET) {
        gpio_bit_set(self->port, self->pin);
    } else {
        gpio_bit_reset(self->port, self->pin);
    }
}

/**
 * @brief 按枚举取 IO 对象指针
 * @param index IoIndex_t
 * @retval 对象指针；非法索引返回 NULL
 */
Periph_Gpio_t* mcal_io_get(IoIndex_t index)
{
    if (index >= IO_MAX) {
        return NULL;
    }
    return &periph_io[index];
}
