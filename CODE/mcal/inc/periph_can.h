/**
 * @file periph_can.h
 * @brief CAN 外设对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-03
 */

#ifndef PERIPH_CAN_H
#define PERIPH_CAN_H

#include "stdinc.h"

typedef struct {
    uint32_t port;
    uint32_t pin;
} CanPin_t;

typedef struct {
    CanPin_t rx;
    CanPin_t tx;
} CanPins_t;

/* 29 位扩展帧 ID、数据、长度 */
typedef void (*CanRxCallback_t)(uint32_t ext_id, uint8_t *data, uint8_t len);

typedef struct Periph_Can
{
    uint32_t    can_periph;
    uint32_t    baudrate;
    CanPins_t   physical_pins;
    uint8_t     is_initialized;

    CanRxCallback_t rx_callback;

    void     (*init)(void);
    void     (*deinit)(void);

    /* 成功 1，邮箱满或初始化失败 -1，参数无效 0。接收时 FIFO 为空返回 0。 */
    int8_t   (*transmit)(uint32_t ext_id, const uint8_t *data, uint8_t len);
    int8_t   (*receive)(uint32_t *ext_id, uint8_t *data, uint8_t *len);

    void     (*enable_rx_interrupt)(void);
    void     (*disable_rx_interrupt)(void);
    void     (*register_rx_callback)(CanRxCallback_t callback);

} Periph_Can_t;

extern Periph_Can_t periph_can0;

#endif /* PERIPH_CAN_H */
