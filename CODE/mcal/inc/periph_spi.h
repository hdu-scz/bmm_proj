/**
 * @file periph_spi.h
 * @brief SPI 外设对象头文件 (MCAL 层)
 * @author 
 * @date 2026-07-02
 */

#ifndef PERIPH_SPI_H
#define PERIPH_SPI_H

#include <stdint.h>
#include "gd32f30x.h"   // HAL 库头文件

/* ==================== SPI 引脚结构体 ==================== */
typedef struct {
    uint32_t port;
    uint32_t pin;
} SpiPin_t;

/* ==================== SPI 引脚组结构体 ==================== */
typedef struct {
    SpiPin_t sck;
    SpiPin_t miso;
    SpiPin_t mosi;
} SpiPins_t;

/* ==================== SPI 对象结构体定义 ==================== */
typedef struct Periph_Spi
{
    /* 硬件配置 */
    uint32_t    spi_periph;              // SPI 外设基地址 (固定为 SPI0)
    uint32_t    baudrate_prescaler;      // 波特率分频系数
    uint32_t    clock_polarity;          // CPOL 与 CPHA 的合成值，如 SPI_CK_PL_HIGH_PH_2EDGE
    uint32_t    frame_size;              // 数据帧长度 (8bit / 16bit)
    uint32_t    direction;               // 通信方向

    /* SPI0 引脚组（使用结构体封装） */
    SpiPins_t   physical_pins;

    /* 状态与错误 */
    uint8_t     state;                   // 当前状态
    uint32_t    error_code;              // 错误码

    /* 方法指针 */
    void     (*init)(struct Periph_Spi *self);
    void     (*deinit)(struct Periph_Spi *self);

    int8_t   (*send_array)(struct Periph_Spi *self, const uint8_t *data, uint16_t size, uint32_t timeout);
    int8_t   (*read_array)(struct Periph_Spi *self, uint8_t *data, uint16_t size, uint32_t timeout);

} Periph_Spi_t;

/* 全局 SPI0 对象声明 */
extern Periph_Spi_t periph_spi0;

#endif /* PERIPH_SPI_H */
