/**
 * @file periph_softiic.c
 * @brief 软件模拟 I2C 主机 (MCAL 层)
 * @author
 * @date 2026-07-02
 */

#include "gd32f30x.h"
#include "periph_softiic.h"
#include "periph_dwt.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_rcu.h"

#define SOFTI2C0_SCL_PORT     GPIOB
#define SOFTI2C0_SCL_PIN      GPIO_PIN_6
#define SOFTI2C0_SDA_PORT     GPIOB
#define SOFTI2C0_SDA_PIN      GPIO_PIN_7

#define SOFTI2C_STRETCH_US    1000U

static void softi2c_init(Periph_SoftI2c_t *self);
static void softi2c_deinit(Periph_SoftI2c_t *self);
static void softi2c_set_speed_mode(Periph_SoftI2c_t *self, I2cSpeedMode_t mode);
static int8_t softi2c_transfer(Periph_SoftI2c_t *self, uint8_t dev_addr,
                               const uint8_t *tx, uint16_t tx_len,
                               uint8_t *rx, uint16_t rx_len);

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

static void softi2c0_init(void) { softi2c_init(&periph_softi2c0); }
static void softi2c0_deinit(void) { softi2c_deinit(&periph_softi2c0); }
static void softi2c0_set_speed_mode(I2cSpeedMode_t mode) { softi2c_set_speed_mode(&periph_softi2c0, mode); }
static int8_t softi2c0_transfer(uint8_t dev_addr,
                                const uint8_t *tx, uint16_t tx_len,
                                uint8_t *rx, uint16_t rx_len) { return softi2c_transfer(&periph_softi2c0, dev_addr, tx, tx_len, rx, rx_len); }

Periph_SoftI2c_t periph_softi2c0 = {
    .scl_port   = SOFTI2C0_SCL_PORT,
    .scl_pin    = SOFTI2C0_SCL_PIN,
    .sda_port   = SOFTI2C0_SDA_PORT,
    .sda_pin    = SOFTI2C0_SDA_PIN,
    .delayUs_timevVal = 5U,
    .speed_mode = I2C_SPEED_REGULAR,
    .is_initialized = 0,
    .init       = softi2c0_init,
    .deinit     = softi2c0_deinit,
    .set_speed_mode = softi2c0_set_speed_mode,
    .transfer   = softi2c0_transfer
};

/**
 * @brief 按 speed_mode 写入半周期微秒数
 * @param self 软件 I2C 对象
 * @retval none
 */
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

/**
 * @brief 按端口打开 GPIO 时钟
 * @param port GPIOA、GPIOB 或 GPIOC
 * @retval none
 */
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

/**
 * @brief 半周期延时
 * @param self 软件 I2C 对象
 * @retval none
 */
static void softi2c_delay(Periph_SoftI2c_t *self)
{
    periph_dwt.delay_us(self->delayUs_timevVal);
}

/** @brief SCL 拉低 */
static void softi2c_scl_low(Periph_SoftI2c_t *self)
{
    gpio_bit_reset(self->scl_port, self->scl_pin);
}

/** @brief SDA 释放/拉高 */
static void softi2c_sda_high(Periph_SoftI2c_t *self)
{
    gpio_bit_set(self->sda_port, self->sda_pin);
}

/** @brief SDA 拉低 */
static void softi2c_sda_low(Periph_SoftI2c_t *self)
{
    gpio_bit_reset(self->sda_port, self->sda_pin);
}

/**
 * @brief 读 SDA 电平
 * @retval 0 低，非 0 高
 */
static uint8_t softi2c_sda_read(Periph_SoftI2c_t *self)
{
    return (uint8_t)gpio_input_bit_get(self->sda_port, self->sda_pin);
}

/**
 * @brief SCL 释放并等待时钟拉伸结束
 * @retval 1 成功，-1 超时
 */
static int8_t softi2c_scl_high(Periph_SoftI2c_t *self)
{
    uint16_t wait_us = SOFTI2C_STRETCH_US;

    gpio_bit_set(self->scl_port, self->scl_pin);
    while (gpio_input_bit_get(self->scl_port, self->scl_pin) == RESET) {
        if (wait_us == 0U) {
            return -1;
        }
        periph_dwt.delay_us(1U);
        wait_us--;
    }
    return 1;
}

/**
 * @brief 初始化软件 I2C：开漏 SCL/SDA，空闲拉高
 * @param self 软件 I2C 对象
 * @retval none
 */
static void softi2c_init(Periph_SoftI2c_t *self)
{
    if (self == NULL) {
        return;
    }

    self->is_initialized = 0;
    softi2c_apply_speed(self);
    periph_dwt.init();

    softi2c_gpio_clock_enable(self->scl_port);
    softi2c_gpio_clock_enable(self->sda_port);

    gpio_init(self->scl_port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, self->scl_pin);
    gpio_init(self->sda_port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, self->sda_pin);

    (void)softi2c_scl_high(self);
    softi2c_sda_high(self);
    self->is_initialized = 1;
}

/**
 * @brief 释放 SCL/SDA 为浮空输入
 * @param self 软件 I2C 对象
 * @retval none
 */
static void softi2c_deinit(Periph_SoftI2c_t *self)
{
    if (self == NULL) {
        return;
    }

    gpio_init(self->scl_port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->scl_pin);
    gpio_init(self->sda_port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->sda_pin);
    self->is_initialized = 0;
}

/**
 * @brief 设置速率档位并刷新半周期延时
 * @param self 软件 I2C 对象
 * @param mode LOW / REGULAR / FAST
 * @retval none
 */
static void softi2c_set_speed_mode(Periph_SoftI2c_t *self, I2cSpeedMode_t mode)
{
    if (self == NULL) {
        return;
    }
    self->speed_mode = mode;
    softi2c_apply_speed(self);
}

/**
 * @brief 产生起始条件
 * @retval 1 成功，-1 SCL 拉伸超时
 */
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

/**
 * @brief 产生停止条件
 * @retval 1 成功，-1 SCL 拉伸超时
 */
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

/**
 * @brief 写一字节并检查 ACK
 * @retval 1 收到 ACK，-1 NACK 或拉伸超时
 */
static int8_t softi2c_write_byte(Periph_SoftI2c_t *self, uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0U; i < 8U; i++) {
        softi2c_scl_low(self);
        if ((data & 0x80U) != 0U) {
            softi2c_sda_high(self);
        } else {
            softi2c_sda_low(self);
        }
        data <<= 1;
        softi2c_delay(self);
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

/**
 * @brief 读一字节并回 ACK/NACK
 * @param ack 非 0 发 ACK，0 发 NACK
 * @retval 1 成功，-1 拉伸超时
 */
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

/**
 * @brief 主机一次传输：可选写后重复起始再读
 * @param self 软件 I2C 对象
 * @param dev_addr 7 位从机地址
 * @param tx 写缓冲，tx_len 为 0 可为 NULL
 * @param tx_len 写长度
 * @param rx 读缓冲，rx_len 为 0 可为 NULL
 * @param rx_len 读长度
 * @retval 1 成功
 * @retval -1 无应答或 SCL 拉伸超时
 * @retval 0 未初始化或参数无效
 */
static int8_t softi2c_transfer(Periph_SoftI2c_t *self, uint8_t dev_addr,
                               const uint8_t *tx, uint16_t tx_len,
                               uint8_t *rx, uint16_t rx_len)
{
    uint16_t i;

    if ((self == NULL) || (self->is_initialized == 0U) || (dev_addr > 0x7FU)) {
        return 0;
    }
    if ((tx_len > 0U) && (tx == 0)) {
        return 0;
    }
    if ((rx_len > 0U) && (rx == 0)) {
        return 0;
    }

    if (softi2c_start(self) != 1) {
        return -1;
    }

    if (tx_len == 0U) {
        if (softi2c_write_byte(self, (uint8_t)((dev_addr << 1) | ((rx_len > 0U) ? 0x01U : 0x00U))) != 1) {
            (void)softi2c_stop(self);
            return -1;
        }
    } else {
        if (softi2c_write_byte(self, (uint8_t)(dev_addr << 1)) != 1) {
            (void)softi2c_stop(self);
            return -1;
        }
        for (i = 0U; i < tx_len; i++) {
            if (softi2c_write_byte(self, tx[i]) != 1) {
                (void)softi2c_stop(self);
                return -1;
            }
        }
        if (rx_len > 0U) {
            if (softi2c_start(self) != 1) {
                (void)softi2c_stop(self);
                return -1;
            }
            if (softi2c_write_byte(self, (uint8_t)((dev_addr << 1) | 0x01U)) != 1) {
                (void)softi2c_stop(self);
                return -1;
            }
        }
    }

    for (i = 0U; i < rx_len; i++) {
        if (softi2c_read_byte(self, (i == (rx_len - 1U)) ? 0U : 1U, &rx[i]) != 1) {
            (void)softi2c_stop(self);
            return -1;
        }
    }

    if (softi2c_stop(self) != 1) {
        return -1;
    }
    return 1;
}
