/**
 * @file dev_can.h
 * @brief CAN 设备对象头文件 (BSP 层)
 * @author 
 * @date 2026-07-09
 */

#ifndef DEV_CAN_H
#define DEV_CAN_H

#include <stdint.h>
#include "periph_can.h"

/* ==================== CAN 帧结构体定义 ==================== */
typedef struct {
    uint32_t    canid;       // CAN ID（支持 29 位扩展 ID）
    uint8_t     data[8];     // 数据内容
    uint8_t     len;         // 数据长度
} CanFrame_t;

/* ==================== CAN 接收回调函数类型 ==================== */
typedef void (*CanFrameRxCallback_t)(CanFrame_t *frame);

/* ==================== CAN 设备对象结构体定义 ==================== */
typedef struct Dev_Can {
    uint8_t     is_initialized;          // 初始化完成标志
    Periph_Can_t *periph;                // 指向 MCAL 层 CAN 对象
    CanFrame_t          frame;           // 接收帧缓存

    void     (*init)(struct Dev_Can *self);
    void     (*deinit)(struct Dev_Can *self);

    uint8_t  (*send)(struct Dev_Can *self, CanFrame_t *frame);
    
    void     (*hook_to_mcal_rx)(uint32_t std_id, uint8_t *data, uint8_t len);  // 内部钩子函数，用于接收 MCAL 层数据
    void     (*register_rx_frame_callback)(struct Dev_Can *self, CanFrameRxCallback_t callback);
    CanFrameRxCallback_t rx_frame_callback;  // 上层注册的帧回调函数
} Dev_Can_t;

/* 全局 CAN 设备对象声明 */
extern Dev_Can_t dev_can0;

#endif /* DEV_CAN_H */