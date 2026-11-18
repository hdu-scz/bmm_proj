/**
 * @file periph_spi.h
 * @brief SPI 外设对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-02
 */

#ifndef PERIPH_SPI_H
#define PERIPH_SPI_H

#include "stdinc.h"

typedef struct {
    uint32_t port;
    uint32_t pin;
} SpiPin_t;

typedef struct {
    SpiPin_t sck;
    SpiPin_t miso;
    SpiPin_t mosi;
} SpiPins_t;

typedef struct Periph_Spi
{
    uint32_t    spi_periph;
    uint32_t    baudrate_prescaler;
    uint32_t    clock_polarity;
    uint32_t    frame_size;
    uint32_t    direction;
    SpiPins_t   physical_pins;
    uint8_t     is_initialized;

    void     (*init)(void);
    void     (*deinit)(void);

    /* 成功 1，超时 -1，未初始化或参数无效 0 */
    int8_t   (*send_array)(const uint8_t *data, uint16_t size, uint32_t timeout);
    int8_t   (*read_array)(uint8_t *data, uint16_t size, uint32_t timeout);

} Periph_Spi_t;

extern Periph_Spi_t periph_spi0;

#endif /* PERIPH_SPI_H */
