/**
 * @file periph_can.c
 * @brief CAN 外设对象实现 (MCAL 层)
 * @author 
 * @date 2026-07-03
 */

#include "periph_can.h"
#include "gd32f30x_can.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"

/* ==================== CAN0 物理引脚映射 ==================== */
#define CAN0_RX_PORT    GPIOB
#define CAN0_RX_PIN     GPIO_PIN_8
#define CAN0_TX_PORT    GPIOB
#define CAN0_TX_PIN     GPIO_PIN_9



/* ==================== 全局 CAN0 对象实例 ==================== */
Periph_Can_t periph_can0;

/* ==================== 私有函数声明 ==================== */
static void can_init(Periph_Can_t *self);
static void can_deinit(Periph_Can_t *self);
static uint8_t can_transmit(Periph_Can_t *self, uint32_t std_id, uint8_t *data, uint8_t len);
static uint8_t can_receive(Periph_Can_t *self, uint32_t *std_id, uint8_t *data, uint8_t *len);

/* ==================== CAN0 对象静态初始化 ==================== */
Periph_Can_t periph_can0 = {
    .can_periph     = CAN0,
    .baudrate       = 500000,        // 默认 500kbps
    .rx_port        = CAN0_RX_PORT,
    .rx_pin         = CAN0_RX_PIN,
    .tx_port        = CAN0_TX_PORT,
    .tx_pin         = CAN0_TX_PIN,
    .is_initialized = 0,
    .init           = can_init,
    .deinit         = can_deinit,
    .transmit       = can_transmit,
    .receive        = can_receive,
    .enable_rx_interrupt   = can_enable_rx_interrupt,
    .disable_rx_interrupt  = can_disable_rx_interrupt,
    .register_rx_callback  = can_register_rx_callback,
    .rx_callback           = NULL
};

/* ==================== 方法实现 ==================== */

static void can_init(Periph_Can_t *self)
{
    self->is_initialized = 0;

    /* 使能 CAN 时钟和 GPIO 时钟 */
    if (self->can_periph == CAN0) {
        rcu_periph_clock_enable(RCU_CAN0);
        rcu_periph_clock_enable(RCU_GPIOB);
    } else if (self->can_periph == CAN1) {
        rcu_periph_clock_enable(RCU_CAN1);
        rcu_periph_clock_enable(RCU_GPIOB);
    }

    /* 配置 CAN 引脚（基于结构体中的 rx/tx 引脚） */
    gpio_init(self->rx_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, self->rx_pin);
    gpio_init(self->tx_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, self->tx_pin);

    /* CAN 参数配置 */
    can_parameter_struct can_init_struct;
    can_struct_para_init(&can_init_struct);

    can_init_struct.working_mode = CAN_NORMAL_MODE;
    can_init_struct.resync_jump_width = CAN_BT_SJW_1TQ;
    can_init_struct.time_segment_1 = CAN_BT_BS1_6TQ;
    can_init_struct.time_segment_2 = CAN_BT_BS2_3TQ;
    can_init_struct.prescaler = 9;   // 72MHz / (1+6+3) / 9 = 500kbps

    can_init(can_periph, &can_init_struct);
    can_filter_init(can_periph, 0, CAN_FILTERMODE_MASK, CAN_FILTERSCALE_32BIT,
                    0x00000000, 0x00000000, CAN_FILTER_FIFO0, ENABLE);

    self->is_initialized = 1;
}

static void can_deinit(Periph_Can_t *self)
{
    can_deinit(self->can_periph);
    self->is_initialized = 0;
}

static uint8_t can_transmit(Periph_Can_t *self, uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (!self->is_initialized) return 0;

    can_transmit_message_struct tx_msg = {0};

    tx_msg.stdid = std_id;
    tx_msg.extid = 0;
    tx_msg.ff = CAN_FF_EXTENDED;
    tx_msg.rtr = CAN_RTR_DATA;
    tx_msg.dlc = len;
    for (uint8_t i = 0; i < len; i++) {
        tx_msg.data[i] = data[i];
    }

    return (can_message_transmit(self->can_periph, &tx_msg) != CAN_NOMAILBOX);
}

static uint8_t can_receive(Periph_Can_t *self, uint32_t *std_id, uint8_t *data, uint8_t *len)
{
    if (!self->is_initialized) return 0;

    can_receive_message_struct rx_msg;
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx_msg);

    if (can_message_receive(self->can_periph, CAN_FIFO0, &rx_msg) == SUCCESS) {
        *std_id = rx_msg.stdid;
        *len = rx_msg.dlc;
        for (uint8_t i = 0; i < rx_msg.dlc; i++) {
            data[i] = rx_msg.data[i];
        }
        return 1;
    }
    return 0;
}

/* ==================== 中断接收相关方法 ==================== */

static void can_enable_rx_interrupt(Periph_Can_t *self)
{
    if (!self->is_initialized) return;

    if (self->can_periph == CAN0) {
        can_interrupt_enable(self->can_periph, CAN_INT_RFNE0);
        nvic_irq_enable(CAN0_RX0_IRQn, 1, 0);
    } else if (self->can_periph == CAN1) {
        can_interrupt_enable(self->can_periph, CAN_INT_RFNE1);
        nvic_irq_enable(CAN1_RX0_IRQn, 1, 1);
    }
}

static void can_disable_rx_interrupt(Periph_Can_t *self)
{
    if (!self->is_initialized) return;

    if (self->can_periph == CAN0) {
        can_interrupt_disable(self->can_periph, CAN_INT_RFNE0);
        nvic_irq_disable(CAN0_RX0_IRQn);
    } else if (self->can_periph == CAN1) {
        can_interrupt_disable(self->can_periph, CAN_INT_RFNE1);
        nvic_irq_disable(CAN1_RX0_IRQn);
    }
}

static void can_register_rx_callback(Periph_Can_t *self, CanRxCallback_t callback)
{
    self->rx_callback = callback;
}

/* ==================== CAN0 接收中断服务函数 ==================== */
void CAN0_RX0_IRQHandler(void)
{
    can_receive_message_struct rx_msg;
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx_msg);

    if (can_message_receive(CAN0, CAN_FIFO0, &rx_msg) == SUCCESS) {
        if (periph_can0.rx_callback != NULL) {
            periph_can0.rx_callback(rx_msg.stdid, rx_msg.data, rx_msg.dlc);
        }
    }
}