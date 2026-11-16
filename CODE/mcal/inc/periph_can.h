/**
 * @file periph_can.h
 * @brief CAN 外设对象头文件 (MCAL 层)
 * @author 
 * @date 2026-07-03
 */

#ifndef PERIPH_CAN_H
#define PERIPH_CAN_H

#include <stdint.h>
#include "gd32f30x.h"

/* ==================== CAN 接收回调函数类型 ==================== */
typedef void (*CanRxCallback_t)(uint32_t std_id, uint8_t *data, uint8_t len);

/* ==================== CAN 对象结构体定义 ==================== */
typedef struct Periph_Can
{
    uint32_t    can_periph;          // CAN 外设基地址 (CAN0 / CAN1)
    uint32_t    baudrate;            // 波特率 (bps)
    uint32_t    rx_port, rx_pin;     // RX 引脚
    uint32_t    tx_port, tx_pin;     // TX 引脚
    uint8_t     is_initialized;      // 初始化完成标志

    /* 中断接收相关 */
    CanRxCallback_t rx_callback;     // 接收回调函数

    void     (*init)(struct Periph_Can *self);
    void     (*deinit)(struct Periph_Can *self);

    uint8_t  (*transmit)(struct Periph_Can *self, uint32_t std_id, uint8_t *data, uint8_t len);
    uint8_t  (*receive)(struct Periph_Can *self, uint32_t *std_id, uint8_t *data, uint8_t *len);

    void     (*enable_rx_interrupt)(struct Periph_Can *self);
    void     (*disable_rx_interrupt)(struct Periph_Can *self);
    void     (*register_rx_callback)(struct Periph_Can *self, CanRxCallback_t callback);

} Periph_Can_t;

/* 全局 CAN0 对象声明 */
extern Periph_Can_t periph_can0;

#endif /* PERIPH_CAN_H */
