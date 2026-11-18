/**
 * @file periph_softiic.h
 * @brief 软件模拟 I2C 主机 (MCAL 层)
 * @author
 * @date 2026-07-02
 *
 * 半周期取整到微秒。48 MHz 下一位时间约 2 × delayUs_timevVal。
 * FAST 用 2 us，约 250 kHz。
 */

#ifndef PERIPH_SOFTIIC_H
#define PERIPH_SOFTIIC_H

#include "stdinc.h"

typedef enum {
    I2C_SPEED_LOW     = 0,   /* 10 us，约 50 kHz */
    I2C_SPEED_REGULAR = 1,   /* 5 us，约 100 kHz */
    I2C_SPEED_FAST    = 2    /* 2 us，约 250 kHz */
} I2cSpeedMode_t;

typedef struct Periph_SoftI2c
{
    uint32_t    scl_port;
    uint32_t    scl_pin;
    uint32_t    sda_port;
    uint32_t    sda_pin;

    uint32_t        delayUs_timevVal;
    I2cSpeedMode_t  speed_mode;
    uint8_t         is_initialized;

    void     (*init)(void);
    void     (*deinit)(void);
    void     (*set_speed_mode)(I2cSpeedMode_t mode);

    /* 7 位从机地址。tx_len 与 rx_len 都为 0 时只发送写地址，用于探测应答。
       两者都大于 0 时，写完 tx 后发重复起始再读 rx。
       返回 1 成功，-1 无应答或 SCL 被拉住超时，0 未初始化或参数无效。 */
    int8_t   (*transfer)(uint8_t dev_addr,
                         const uint8_t *tx, uint16_t tx_len,
                         uint8_t *rx, uint16_t rx_len);

} Periph_SoftI2c_t;

extern Periph_SoftI2c_t periph_softi2c0;

#endif /* PERIPH_SOFTIIC_H */
