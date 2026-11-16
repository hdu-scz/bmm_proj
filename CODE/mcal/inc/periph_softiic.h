/**
 * @file periph_softiic.h
 * @brief 软件模拟 I2C 外设对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-02
 */

#ifndef PERIPH_SOFTIIC_H
#define PERIPH_SOFTIIC_H

#include <stdint.h>
#include "gd32f30x.h"

/* 半周期取整到微秒后的实际速率。48 MHz 下 DWT 每微秒 48 个周期。
   400 kHz 需要 1.25 us，这里用 2 us，实际约 250 kHz。 */
typedef enum {
    I2C_SPEED_LOW     = 0,   /* 半周期 10 us，约 50 kHz */
    I2C_SPEED_REGULAR = 1,   /* 半周期 5 us，约 100 kHz */
    I2C_SPEED_FAST    = 2    /* 半周期 2 us，约 250 kHz */
} I2cSpeedMode_t;

typedef struct Periph_SoftI2c
{
    uint32_t    scl_port;
    uint32_t    scl_pin;
    uint32_t    sda_port;
    uint32_t    sda_pin;

    uint32_t        delayUs_timevVal;    /* 半周期，单位 us */
    I2cSpeedMode_t  speed_mode;

    void     (*init)(struct Periph_SoftI2c *self);
    void     (*deinit)(struct Periph_SoftI2c *self);
    void     (*set_speed_mode)(struct Periph_SoftI2c *self, I2cSpeedMode_t mode);

    /* dev_addr 为 7 位地址。addr_len：0 无存储地址，1 为 AT24C01～16，2 为 AT24C32 及以上。
       返回 1 成功，-1 无 ACK 或从机拉住 SCL 超时，0 参数无效。 */
    int8_t   (*write)(struct Periph_SoftI2c *self, uint8_t dev_addr, uint16_t mem_addr,
                      uint8_t addr_len, const uint8_t *data, uint16_t len);
    int8_t   (*read)(struct Periph_SoftI2c *self, uint8_t dev_addr, uint16_t mem_addr,
                     uint8_t addr_len, uint8_t *data, uint16_t len);

} Periph_SoftI2c_t;

extern Periph_SoftI2c_t periph_softi2c0;

#endif /* PERIPH_SOFTIIC_H */
