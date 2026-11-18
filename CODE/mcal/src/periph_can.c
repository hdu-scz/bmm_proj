/**
 * @file periph_can.c
 * @brief CAN0 外设对象实现 (MCAL 层)
 * @author
 * @date 2026-07-03
 *
 * 系统时钟 48 MHz，APB1 二分频后 CAN 时钟为 24 MHz。
 * 位时间固定 8TQ：SJW = 1TQ，BS1 = 5TQ，BS2 = 2TQ。
 * 预分频 = 24 MHz / (CAN0_BAUDRATE × 8)。
 */

#include "periph_can.h"
#include "gd32f30x_can.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_misc.h"

#define CAN0_RX_PORT    GPIOB
#define CAN0_RX_PIN     GPIO_PIN_8
#define CAN0_TX_PORT    GPIOB
#define CAN0_TX_PIN     GPIO_PIN_9

#define CAN0_EXT_ID_MAX   0x1FFFFFFFU

/* CAN 时钟为 APB1 24 MHz，位时间为 8TQ。波特率须能整除 24 MHz / 8 */
#define CAN0_CLOCK_HZ     24000000UL
#define CAN0_BIT_TQ       8UL
#define CAN0_BAUDRATE     500000U
#define CAN0_PRESCALER    (CAN0_CLOCK_HZ / (CAN0_BAUDRATE * CAN0_BIT_TQ))

#if ((CAN0_CLOCK_HZ % (CAN0_BAUDRATE * CAN0_BIT_TQ)) != 0U)
#error "CAN0_BAUDRATE cannot divide the 24 MHz CAN clock into 8 time quanta"
#endif

static void periph_can_init(Periph_Can_t *self);
static void periph_can_deinit(Periph_Can_t *self);
static int8_t periph_can_transmit(Periph_Can_t *self, uint32_t ext_id, const uint8_t *data, uint8_t len);
static int8_t periph_can_receive(Periph_Can_t *self, uint32_t *ext_id, uint8_t *data, uint8_t *len);
static void periph_can_enable_rx_interrupt(Periph_Can_t *self);
static void periph_can_disable_rx_interrupt(Periph_Can_t *self);
static void periph_can_register_rx_callback(Periph_Can_t *self, CanRxCallback_t callback);

static void can_gpio_clock_enable(uint32_t port);
static void can_apply_pin_remap(const Periph_Can_t *self);

static void can0_init(void) { periph_can_init(&periph_can0); }
static void can0_deinit(void) { periph_can_deinit(&periph_can0); }
static int8_t can0_transmit(uint32_t ext_id, const uint8_t *data, uint8_t len) { return periph_can_transmit(&periph_can0, ext_id, data, len); }
static int8_t can0_receive(uint32_t *ext_id, uint8_t *data, uint8_t *len) { return periph_can_receive(&periph_can0, ext_id, data, len); }
static void can0_enable_rx_interrupt(void) { periph_can_enable_rx_interrupt(&periph_can0); }
static void can0_disable_rx_interrupt(void) { periph_can_disable_rx_interrupt(&periph_can0); }
static void can0_register_rx_callback(CanRxCallback_t callback) { periph_can_register_rx_callback(&periph_can0, callback); }

Periph_Can_t periph_can0 = {
    .can_periph = CAN0,
    .baudrate   = CAN0_BAUDRATE,
    .physical_pins = {
        .rx = {CAN0_RX_PORT, CAN0_RX_PIN},
        .tx = {CAN0_TX_PORT, CAN0_TX_PIN},
    },
    .is_initialized = 0,
    .rx_callback = NULL,
    .init = can0_init,
    .deinit = can0_deinit,
    .transmit = can0_transmit,
    .receive = can0_receive,
    .enable_rx_interrupt = can0_enable_rx_interrupt,
    .disable_rx_interrupt = can0_disable_rx_interrupt,
    .register_rx_callback = can0_register_rx_callback
};

/**
 * @brief 按端口打开 GPIO 时钟
 * @param port GPIOA、GPIOB 或 GPIOC
 * @retval none
 */
static void can_gpio_clock_enable(uint32_t port)
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
 * @brief PB8/PB9 做 CAN0 部分重映射
 * @param self CAN0 对象，使用其中的 physical_pins
 * @retval none
 */
static void can_apply_pin_remap(const Periph_Can_t *self)
{
    if ((self->physical_pins.rx.port == GPIOB) &&
        (self->physical_pins.rx.pin == GPIO_PIN_8) &&
        (self->physical_pins.tx.port == GPIOB) &&
        (self->physical_pins.tx.pin == GPIO_PIN_9)) {
        rcu_periph_clock_enable(RCU_AF);
        gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);
    }
}

/**
 * @brief 初始化 CAN0：时钟、引脚、500 kbps、过滤器
 * @param self CAN0 对象
 * @retval none 失败时 is_initialized 保持 0
 * @note 不开启接收中断；须先 register_rx_callback，再 enable_rx_interrupt
 */
static void periph_can_init(Periph_Can_t *self)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    self->is_initialized = 0;

    rcu_periph_clock_enable(RCU_CAN0);
    can_gpio_clock_enable(self->physical_pins.rx.port);
    can_gpio_clock_enable(self->physical_pins.tx.port);
    can_apply_pin_remap(self);

    gpio_init(self->physical_pins.rx.port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
              self->physical_pins.rx.pin);
    gpio_init(self->physical_pins.tx.port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              self->physical_pins.tx.pin);

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    can_parameter.prescaler = (uint16_t)CAN0_PRESCALER;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_retrans = ENABLE;
    if (can_init(self->can_periph, &can_parameter) != SUCCESS) {
        return;
    }

    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_filter.filter_number = 0U;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000U;
    can_filter.filter_list_low = 0x0000U;
    can_filter.filter_mask_high = 0x0000U;
    can_filter.filter_mask_low = 0x0000U;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);

    self->is_initialized = 1;
}

/**
 * @brief 关闭接收中断并复位 CAN0
 * @param self CAN0 对象
 * @retval none
 */
static void periph_can_deinit(Periph_Can_t *self)
{
    periph_can_disable_rx_interrupt(self);
    can_deinit(self->can_periph);
    self->is_initialized = 0;
}

/**
 * @brief 把一帧扩展数据帧放入发送邮箱
 * @param self CAN0 对象
 * @param ext_id 29 位扩展帧 ID，最大 0x1FFFFFFF
 * @param data 数据指针，len 为 0 时可以为空
 * @param len 数据长度，0 到 8
 * @retval 1 已写入邮箱
 * @retval -1 三个邮箱都满
 * @retval 0 参数无效或尚未初始化
 */
static int8_t periph_can_transmit(Periph_Can_t *self, uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    can_transmit_message_struct tx_msg;
    uint8_t i;
    uint8_t mailbox;

    if ((self == 0) || (self->is_initialized == 0U) || (ext_id > CAN0_EXT_ID_MAX) || (len > 8U)) {
        return 0;
    }
    if ((len > 0U) && (data == 0)) {
        return 0;
    }

    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_msg);
    tx_msg.tx_efid = ext_id;
    tx_msg.tx_ff = (uint8_t)CAN_FF_EXTENDED;
    tx_msg.tx_ft = (uint8_t)CAN_FT_DATA;
    tx_msg.tx_dlen = len;
    for (i = 0U; i < len; i++) {
        tx_msg.tx_data[i] = data[i];
    }

    mailbox = can_message_transmit(self->can_periph, &tx_msg);
    if (mailbox == CAN_NOMAILBOX) {
        return -1;
    }
    return 1;
}

/**
 * @brief 从 FIFO0 读出一帧
 * @param self CAN0 对象
 * @param ext_id 输出 ID。扩展帧为 29 位 ID，标准帧为 11 位 ID
 * @param data 输出数据，至少 8 字节
 * @param len 输出数据长度
 * @retval 1 读到一帧
 * @retval -1 数据长度异常
 * @retval 0 参数无效、尚未初始化或 FIFO 为空
 */
static int8_t periph_can_receive(Periph_Can_t *self, uint32_t *ext_id, uint8_t *data, uint8_t *len)
{
    can_receive_message_struct rx_msg;
    uint8_t i;

    if ((self == 0) || (self->is_initialized == 0U) || (ext_id == 0) || (data == 0) || (len == 0)) {
        return 0;
    }
    if (can_receive_message_length_get(self->can_periph, CAN_FIFO0) == 0U) {
        return 0;
    }

    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx_msg);
    can_message_receive(self->can_periph, CAN_FIFO0, &rx_msg);
    if (rx_msg.rx_dlen > 8U) {
        return -1;
    }

    if (rx_msg.rx_ff == (uint8_t)CAN_FF_EXTENDED) {
        *ext_id = rx_msg.rx_efid;
    } else {
        *ext_id = rx_msg.rx_sfid;
    }
    *len = rx_msg.rx_dlen;
    for (i = 0U; i < rx_msg.rx_dlen; i++) {
        data[i] = rx_msg.rx_data[i];
    }
    return 1;
}

/**
 * @brief 使能 FIFO0 非空中断
 * @param self CAN0 对象
 * @retval none 未初始化时不操作
 */
static void periph_can_enable_rx_interrupt(Periph_Can_t *self)
{
    if ((self == 0) || (self->is_initialized == 0U)) {
        return;
    }

    can_interrupt_enable(self->can_periph, CAN_INT_RFNE0);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1U, 0U);
}

/**
 * @brief 关闭 FIFO0 非空中断
 * @param self CAN0 对象
 * @retval none
 */
static void periph_can_disable_rx_interrupt(Periph_Can_t *self)
{
    if (self == 0) {
        return;
    }

    can_interrupt_disable(self->can_periph, CAN_INT_RFNE0);
    nvic_irq_disable(USBD_LP_CAN0_RX0_IRQn);
}

/**
 * @brief 注册接收回调（不开关中断）
 * @param self CAN0 对象
 * @param callback 收到一帧后调用，传空则取消回调
 * @retval none
 */
static void periph_can_register_rx_callback(Periph_Can_t *self, CanRxCallback_t callback)
{
    if (self == 0) {
        return;
    }
    self->rx_callback = callback;
}

/**
 * @brief CAN0 FIFO0 接收中断
 * @param none
 * @retval none
 */
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    can_receive_message_struct rx_msg;

    if (can_interrupt_flag_get(CAN0, CAN_INT_FLAG_RFL0) == RESET) {
        return;
    }

    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx_msg);
    can_message_receive(CAN0, CAN_FIFO0, &rx_msg);
    if ((rx_msg.rx_dlen > 8U) || (periph_can0.rx_callback == 0)) {
        return;
    }
    if (rx_msg.rx_ff == (uint8_t)CAN_FF_EXTENDED) {
        periph_can0.rx_callback(rx_msg.rx_efid, rx_msg.rx_data, rx_msg.rx_dlen);
    } else {
        periph_can0.rx_callback(rx_msg.rx_sfid, rx_msg.rx_data, rx_msg.rx_dlen);
    }
}
