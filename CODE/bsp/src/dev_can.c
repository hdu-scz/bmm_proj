/**
 * @file dev_can.c
 * @brief CAN 设备对象实现 (BSP 层)
 * @author 
 * @date 2026-07-09
 */

#include "dev_can.h"

/* ==================== 全局 CAN 设备对象实例 ==================== */
Dev_Can_t dev_can0;

/* ==================== 私有函数声明 ==================== */
static void dev_can_init(Dev_Can_t *self);
static void dev_can_deinit(Dev_Can_t *self);
static uint8_t dev_can_send(Dev_Can_t *self, CanFrame_t *frame);
static void dev_can0_hook_to_mcal_rx(uint32_t std_id, uint8_t *data, uint8_t len);
static void dev_can_register_rx_frame_callback(Dev_Can_t *self, CanFrameRxCallback_t callback);

/* ==================== CAN 设备对象静态初始化 ==================== */
Dev_Can_t dev_can0 = {
    .is_initialized = 0,
    .periph         = &periph_can0,  // 指向 MCAL 层 CAN0 对象
    .frame          = {0},
    .init           = dev_can_init,
    .deinit         = dev_can_deinit,
    .send           = dev_can_send,
    .hook_to_mcal_rx = dev_can0_hook_to_mcal_rx,
    .register_rx_frame_callback = dev_can_register_rx_frame_callback,
    .rx_frame_callback = NULL
};

/* ==================== 方法实现 ==================== */

static void dev_can_init(Dev_Can_t *self)
{
    if (self->periph != NULL) {
        self->periph->init();
        self->periph->register_rx_callback(self->hook_to_mcal_rx);
        self->periph->enable_rx_interrupt();
        self->is_initialized = 1;
    }
}


static void dev_can_deinit(Dev_Can_t *self)
{
    if (self->periph != NULL) {
        self->periph->disable_rx_interrupt();
        self->periph->register_rx_callback(NULL);
        self->periph->deinit();
    }
    self->is_initialized = 0;
    self->rx_frame_callback = NULL;
}

static uint8_t dev_can_send(Dev_Can_t *self, CanFrame_t *frame)
{
    if (!self->is_initialized || self->periph == NULL || frame == NULL) {
        return 0;
    }

    return self->periph->transmit(frame->canid, frame->data, frame->len);
}


/* ==================== 注册帧回调函数 ==================== */
static void dev_can_register_rx_frame_callback(Dev_Can_t *self, CanFrameRxCallback_t callback)
{
    self->rx_frame_callback = callback;
}


/* ==================== MCAL 层回调适配函数（内部钩子） ==================== */
static void dev_can0_hook_to_mcal_rx(uint32_t std_id, uint8_t *data, uint8_t len)
{
    /* 将 MCAL 层原始数据封装成 CanFrame_t */
    dev_can0.frame.canid = std_id;
    dev_can0.frame.len   = len;
    for (uint8_t i = 0; i < len && i < 8; i++) 
    {
        dev_can0.frame.data[i] = data[i];
    }
    if (dev_can0.rx_frame_callback != NULL) 
    {
        dev_can0.rx_frame_callback(&dev_can0.frame);// 调用上层注册的回调函数
    }
}
