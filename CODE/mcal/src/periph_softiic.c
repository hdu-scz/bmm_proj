/**
 * @file periph_softiic.c
 * @brief 软件模拟 I2C 外设对象实现 (MCAL 层)
 * @author
 * @date 2026-07-02
 */

#include "periph_softiic.h"
#include "periph_dwt.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_rcu.h"

#define SOFTI2C0_SCL_PORT     GPIOB
#define SOFTI2C0_SCL_PIN      GPIO_PIN_6
#define SOFTI2C0_SDA_PORT     GPIOB
#define SOFTI2C0_SDA_PIN      GPIO_PIN_7

/* 从机拉住 SCL 时最多再等 1 ms */
#define SOFTI2C_STRETCH_US    1000U

static void softi2c_init(Periph_SoftI2c_t *self);
static void softi2c_deinit(Periph_SoftI2c_t *self);
static void softi2c_set_speed_mode(Periph_SoftI2c_t *self, I2cSpeedMode_t mode);
static int8_t softi2c_write(Periph_SoftI2c_t *self, uint8_t dev_addr, uint16_t mem_addr,
                            uint8_t addr_len, const uint8_t *data, uint16_t len);
static int8_t softi2c_read(Periph_SoftI2c_t *self, uint8_t dev_addr, uint16_t mem_addr,
                           uint8_t addr_len, uint8_t *data, uint16_t len);

static void softi2c_apply_speed(Periph_SoftI2c_t *self);
static void softi2c_gpio_clock_enable(uint32_t port);
static void softi2c_delay(Periph_SoftI2c_t *self);
static int8_t softi2c_scl_high(Periph_SoftI2c_t *self);
static void softi2c_scl_low(Periph_SoftI2c_t *self);
static void softi2c_sda_high(Periph_SoftI2c_t *self);
static void softi2c_sda_low(Periph_SoftI2c_t *self);
static uint8_t softi2c_sda_read(Periph_SoftI2c_t *self);
static int8_t softi2c_start(Periph_SoftI2c_t *self);
static int8_t softi2c_stop(Periph_SoftI2c_t *self);
static int8_t softi2c_write_byte(Periph_SoftI2c_t *self, uint8_t data);
static int8_t softi2c_read_byte(Periph_SoftI2c_t *self, uint8_t ack, uint8_t *data);
static int8_t softi2c_write_addr(Periph_SoftI2c_t *self, uint8_t dev_addr, uint16_t mem_addr, uint8_t addr_len);

Periph_SoftI2c_t periph_softi2c0 = {
    .scl_port   = SOFTI2C0_SCL_PORT,
    .scl_pin    = SOFTI2C0_SCL_PIN,
    .sda_port   = SOFTI2C0_SDA_PORT,
    .sda_pin    = SOFTI2C0_SDA_PIN,
    .delayUs_timevVal = 5U,
    .speed_mode = I2C_SPEED_REGULAR,
    .init       = softi2c_init,
    .deinit     = softi2c_deinit,
    .set_speed_mode = softi2c_set_speed_mode,
    .write      = softi2c_write,
    .read       = softi2c_read
};

static void softi2c_apply_speed(Periph_SoftI2c_t *self)
{
    switch (self->speed_mode) {
    case I2C_SPEED_LOW:
        self->delayUs_timevVal = 10U;
        break;
    case I2C_SPEED_FAST:
        self->delayUs_timevVal = 2U;
        break;
    case I2C_SPEED_REGULAR:
    default:
        self->delayUs_timevVal = 5U;
        break;
    }
}

static void softi2c_gpio_clock_enable(uint32_t port)
{
    if (port == GPIOA) {
        rcu_periph_clock_enable(RCU_GPIOA);
    } else if (port == GPIOB) {
        rcu_periph_clock_enable(RCU_GPIOB);
    } else if (port == GPIOC) {
        rcu_periph_clock_enable(RCU_GPIOC);
    }
}

static void softi2c_delay(Periph_SoftI2c_t *self)
{
    periph_dwt.delay_us(&periph_dwt, self->delayUs_timevVal);
}

static void softi2c_scl_low(Periph_SoftI2c_t *self)
{
    gpio_bit_reset(self->scl_port, self->scl_pin);
}

static void softi2c_sda_high(Periph_SoftI2c_t *self)
{
    gpio_bit_set(self->sda_port, self->sda_pin);
}

static void softi2c_sda_low(Periph_SoftI2c_t *self)
{
    gpio_bit_reset(self->sda_port, self->sda_pin);
}

static uint8_t softi2c_sda_read(Periph_SoftI2c_t *self)
{
    return (uint8_t)gpio_input_bit_get(self->sda_port, self->sda_pin);
}

/* 开漏释放 SCL 后，等引脚真正变高。从机把 SCL 拉低时就停在这里。 */
static int8_t softi2c_scl_high(Periph_SoftI2c_t *self)
{
    uint16_t wait_us = SOFTI2C_STRETCH_US;

    gpio_bit_set(self->scl_port, self->scl_pin);
    while (gpio_input_bit_get(self->scl_port, self->scl_pin) == RESET) {
        if (wait_us == 0U) {
            return -1;
        }
        periph_dwt.delay_us(&periph_dwt, 1U);
        wait_us--;
    }
    return 1;
}

static void softi2c_init(Periph_SoftI2c_t *self)
{
    softi2c_apply_speed(self);
    periph_dwt.init(&periph_dwt, periph_dwt.core_clock_hz);

    softi2c_gpio_clock_enable(self->scl_port);
    softi2c_gpio_clock_enable(self->sda_port);

    gpio_init(self->scl_port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, self->scl_pin);
    gpio_init(self->sda_port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, self->sda_pin);

    softi2c_scl_high(self);
    softi2c_sda_high(self);
}

static void softi2c_deinit(Periph_SoftI2c_t *self)
{
    gpio_init(self->scl_port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->scl_pin);
    gpio_init(self->sda_port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->sda_pin);
}

static void softi2c_set_speed_mode(Periph_SoftI2c_t *self, I2cSpeedMode_t mode)
{
    self->speed_mode = mode;
    softi2c_apply_speed(self);
}

static int8_t softi2c_start(Periph_SoftI2c_t *self)
{
    softi2c_sda_high(self);
    if (softi2c_scl_high(self) != 1) {
        return -1;
    }
    softi2c_delay(self);

    softi2c_sda_low(self);
    softi2c_delay(self);
    softi2c_scl_low(self);
    softi2c_delay(self);
    return 1;
}

static int8_t softi2c_stop(Periph_SoftI2c_t *self)
{
    softi2c_sda_low(self);
    softi2c_scl_low(self);
    softi2c_delay(self);

    if (softi2c_scl_high(self) != 1) {
        return -1;
    }
    softi2c_delay(self);
    softi2c_sda_high(self);
    softi2c_delay(self);
    return 1;
}

static int8_t softi2c_write_byte(Periph_SoftI2c_t *self, uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0U; i < 8U; i++) {
        softi2c_scl_low(self);
        softi2c_delay(self);
        if ((data & 0x80U) != 0U) {
            softi2c_sda_high(self);
        } else {
            softi2c_sda_low(self);
        }
        data <<= 1;
        if (softi2c_scl_high(self) != 1) {
            return -1;
        }
        softi2c_delay(self);
    }

    softi2c_scl_low(self);
    softi2c_sda_high(self);
    softi2c_delay(self);
    if (softi2c_scl_high(self) != 1) {
        return -1;
    }
    softi2c_delay(self);
    ack = softi2c_sda_read(self);
    softi2c_scl_low(self);

    return (ack == 0U) ? 1 : -1;
}

static int8_t softi2c_read_byte(Periph_SoftI2c_t *self, uint8_t ack, uint8_t *data)
{
    uint8_t i;
    uint8_t value = 0U;

    softi2c_sda_high(self);
    for (i = 0U; i < 8U; i++) {
        softi2c_scl_low(self);
        softi2c_delay(self);
        if (softi2c_scl_high(self) != 1) {
            return -1;
        }
        softi2c_delay(self);
        value <<= 1;
        if (softi2c_sda_read(self) != 0U) {
            value |= 0x01U;
        }
    }

    softi2c_scl_low(self);
    if (ack != 0U) {
        softi2c_sda_low(self);
    } else {
        softi2c_sda_high(self);
    }
    softi2c_delay(self);
    if (softi2c_scl_high(self) != 1) {
        return -1;
    }
    softi2c_delay(self);
    softi2c_scl_low(self);
    *data = value;
    return 1;
}

static int8_t softi2c_write_addr(Periph_SoftI2c_t *self, uint8_t dev_addr, uint16_t mem_addr, uint8_t addr_len)
{
    if (softi2c_write_byte(self, (uint8_t)(dev_addr << 1)) != 1) {
        return -1;
    }
    if (addr_len == 2U) {
        if (softi2c_write_byte(self, (uint8_t)(mem_addr >> 8)) != 1) {
            return -1;
        }
    }
    if (addr_len >= 1U) {
        if (softi2c_write_byte(self, (uint8_t)mem_addr) != 1) {
            return -1;
        }
    }
    return 1;
}

static int8_t softi2c_write(Periph_SoftI2c_t *self, uint8_t dev_addr, uint16_t mem_addr,
                            uint8_t addr_len, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((self == 0) || (addr_len > 2U) || ((len > 0U) && (data == 0))) {
        return 0;
    }

    if (softi2c_start(self) != 1) {
        return -1;
    }
    if (softi2c_write_addr(self, dev_addr, mem_addr, addr_len) != 1) {
        (void)softi2c_stop(self);
        return -1;
    }
    for (i = 0U; i < len; i++) {
        if (softi2c_write_byte(self, data[i]) != 1) {
            (void)softi2c_stop(self);
            return -1;
        }
    }
    if (softi2c_stop(self) != 1) {
        return -1;
    }
    return 1;
}

static int8_t softi2c_read(Periph_SoftI2c_t *self, uint8_t dev_addr, uint16_t mem_addr,
                           uint8_t addr_len, uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((self == 0) || (data == 0) || (len == 0U) || (addr_len > 2U)) {
        return 0;
    }

    if (addr_len > 0U) {
        if (softi2c_start(self) != 1) {
            return -1;
        }
        if (softi2c_write_addr(self, dev_addr, mem_addr, addr_len) != 1) {
            (void)softi2c_stop(self);
            return -1;
        }
    }

    if (softi2c_start(self) != 1) {
        return -1;
    }
    if (softi2c_write_byte(self, (uint8_t)((dev_addr << 1) | 0x01U)) != 1) {
        (void)softi2c_stop(self);
        return -1;
    }
    for (i = 0U; i < len; i++) {
        if (softi2c_read_byte(self, (i == (len - 1U)) ? 0U : 1U, &data[i]) != 1) {
            (void)softi2c_stop(self);
            return -1;
        }
    }
    if (softi2c_stop(self) != 1) {
        return -1;
    }
    return 1;
}
