/**
 * @file periph_uart.c
 * @brief UART 外设对象实现 (MCAL 层)
 * @author
 * @date 2026-07-06
 *
 * USART0: PA9 TX / PA10 RX
 * USART2: PB10 TX / PB11 RX
 * UART3:  PC10 TX / PC11 RX
 * UART4:  PC12 TX / PD2 RX
 * 默认波特率 UART_BAUDRATE。发送带超时，未 init 返回 0。
 */

#include "stdinc.h"
#include "periph_uart.h"
#include "gd32f30x_usart.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_misc.h"

#define UART_BAUDRATE   9600U

#define UART0_TX_PORT   GPIOA
#define UART0_TX_PIN    GPIO_PIN_9
#define UART0_RX_PORT   GPIOA
#define UART0_RX_PIN    GPIO_PIN_10

#define UART2_TX_PORT   GPIOB
#define UART2_TX_PIN    GPIO_PIN_10
#define UART2_RX_PORT   GPIOB
#define UART2_RX_PIN    GPIO_PIN_11

#define UART3_TX_PORT   GPIOC
#define UART3_TX_PIN    GPIO_PIN_10
#define UART3_RX_PORT   GPIOC
#define UART3_RX_PIN    GPIO_PIN_11

#define UART4_TX_PORT   GPIOC
#define UART4_TX_PIN    GPIO_PIN_12
#define UART4_RX_PORT   GPIOD
#define UART4_RX_PIN    GPIO_PIN_2

static void periph_uart_init(Periph_Uart_t *self);
static void periph_uart_deinit(Periph_Uart_t *self);
static int8_t periph_uart_send_byte(Periph_Uart_t *self, uint8_t data, uint32_t timeout);
static int8_t periph_uart_send_array(Periph_Uart_t *self, const uint8_t *data, uint16_t len, uint32_t timeout);
static void periph_uart_enable_rx_interrupt(Periph_Uart_t *self);
static void periph_uart_disable_rx_interrupt(Periph_Uart_t *self);
static void periph_uart_register_rx_callback(Periph_Uart_t *self, UartRxCallback_t callback);

static void uart_gpio_clock_enable(uint32_t port);

static void uart0_init(void) { periph_uart_init(&periph_uart0); }
static void uart0_deinit(void) { periph_uart_deinit(&periph_uart0); }
static int8_t uart0_send_byte(uint8_t data, uint32_t timeout) { return periph_uart_send_byte(&periph_uart0, data, timeout); }
static int8_t uart0_send_array(const uint8_t *data, uint16_t len, uint32_t timeout) { return periph_uart_send_array(&periph_uart0, data, len, timeout); }
static void uart0_enable_rx_interrupt(void) { periph_uart_enable_rx_interrupt(&periph_uart0); }
static void uart0_disable_rx_interrupt(void) { periph_uart_disable_rx_interrupt(&periph_uart0); }
static void uart0_register_rx_callback(UartRxCallback_t callback) { periph_uart_register_rx_callback(&periph_uart0, callback); }

static void uart2_init(void) { periph_uart_init(&periph_uart2); }
static void uart2_deinit(void) { periph_uart_deinit(&periph_uart2); }
static int8_t uart2_send_byte(uint8_t data, uint32_t timeout) { return periph_uart_send_byte(&periph_uart2, data, timeout); }
static int8_t uart2_send_array(const uint8_t *data, uint16_t len, uint32_t timeout) { return periph_uart_send_array(&periph_uart2, data, len, timeout); }
static void uart2_enable_rx_interrupt(void) { periph_uart_enable_rx_interrupt(&periph_uart2); }
static void uart2_disable_rx_interrupt(void) { periph_uart_disable_rx_interrupt(&periph_uart2); }
static void uart2_register_rx_callback(UartRxCallback_t callback) { periph_uart_register_rx_callback(&periph_uart2, callback); }

static void uart3_init(void) { periph_uart_init(&periph_uart3); }
static void uart3_deinit(void) { periph_uart_deinit(&periph_uart3); }
static int8_t uart3_send_byte(uint8_t data, uint32_t timeout) { return periph_uart_send_byte(&periph_uart3, data, timeout); }
static int8_t uart3_send_array(const uint8_t *data, uint16_t len, uint32_t timeout) { return periph_uart_send_array(&periph_uart3, data, len, timeout); }
static void uart3_enable_rx_interrupt(void) { periph_uart_enable_rx_interrupt(&periph_uart3); }
static void uart3_disable_rx_interrupt(void) { periph_uart_disable_rx_interrupt(&periph_uart3); }
static void uart3_register_rx_callback(UartRxCallback_t callback) { periph_uart_register_rx_callback(&periph_uart3, callback); }

static void uart4_init(void) { periph_uart_init(&periph_uart4); }
static void uart4_deinit(void) { periph_uart_deinit(&periph_uart4); }
static int8_t uart4_send_byte(uint8_t data, uint32_t timeout) { return periph_uart_send_byte(&periph_uart4, data, timeout); }
static int8_t uart4_send_array(const uint8_t *data, uint16_t len, uint32_t timeout) { return periph_uart_send_array(&periph_uart4, data, len, timeout); }
static void uart4_enable_rx_interrupt(void) { periph_uart_enable_rx_interrupt(&periph_uart4); }
static void uart4_disable_rx_interrupt(void) { periph_uart_disable_rx_interrupt(&periph_uart4); }
static void uart4_register_rx_callback(UartRxCallback_t callback) { periph_uart_register_rx_callback(&periph_uart4, callback); }

Periph_Uart_t periph_uart0 = {
    .uart_periph = USART0,
    .baudrate    = UART_BAUDRATE,
    .physical_pins = {
        .rx = {UART0_RX_PORT, UART0_RX_PIN},
        .tx = {UART0_TX_PORT, UART0_TX_PIN},
    },
    .is_initialized = 0,
    .rx_callback = NULL,
    .init = uart0_init,
    .deinit = uart0_deinit,
    .send_byte = uart0_send_byte,
    .send_array = uart0_send_array,
    .enable_rx_interrupt = uart0_enable_rx_interrupt,
    .disable_rx_interrupt = uart0_disable_rx_interrupt,
    .register_rx_callback = uart0_register_rx_callback
};

Periph_Uart_t periph_uart2 = {
    .uart_periph = USART2,
    .baudrate    = UART_BAUDRATE,
    .physical_pins = {
        .rx = {UART2_RX_PORT, UART2_RX_PIN},
        .tx = {UART2_TX_PORT, UART2_TX_PIN},
    },
    .is_initialized = 0,
    .rx_callback = NULL,
    .init = uart2_init,
    .deinit = uart2_deinit,
    .send_byte = uart2_send_byte,
    .send_array = uart2_send_array,
    .enable_rx_interrupt = uart2_enable_rx_interrupt,
    .disable_rx_interrupt = uart2_disable_rx_interrupt,
    .register_rx_callback = uart2_register_rx_callback
};

Periph_Uart_t periph_uart3 = {
    .uart_periph = UART3,
    .baudrate    = UART_BAUDRATE,
    .physical_pins = {
        .rx = {UART3_RX_PORT, UART3_RX_PIN},
        .tx = {UART3_TX_PORT, UART3_TX_PIN},
    },
    .is_initialized = 0,
    .rx_callback = NULL,
    .init = uart3_init,
    .deinit = uart3_deinit,
    .send_byte = uart3_send_byte,
    .send_array = uart3_send_array,
    .enable_rx_interrupt = uart3_enable_rx_interrupt,
    .disable_rx_interrupt = uart3_disable_rx_interrupt,
    .register_rx_callback = uart3_register_rx_callback
};

Periph_Uart_t periph_uart4 = {
    .uart_periph = UART4,
    .baudrate    = UART_BAUDRATE,
    .physical_pins = {
        .rx = {UART4_RX_PORT, UART4_RX_PIN},
        .tx = {UART4_TX_PORT, UART4_TX_PIN},
    },
    .is_initialized = 0,
    .rx_callback = NULL,
    .init = uart4_init,
    .deinit = uart4_deinit,
    .send_byte = uart4_send_byte,
    .send_array = uart4_send_array,
    .enable_rx_interrupt = uart4_enable_rx_interrupt,
    .disable_rx_interrupt = uart4_disable_rx_interrupt,
    .register_rx_callback = uart4_register_rx_callback
};

/**
 * @brief 按端口打开 GPIO 时钟
 * @param port GPIOA～GPIOD
 * @retval none
 */
static void uart_gpio_clock_enable(uint32_t port)
{
    if (port == GPIOA) {
        rcu_periph_clock_enable(RCU_GPIOA);
    } else if (port == GPIOB) {
        rcu_periph_clock_enable(RCU_GPIOB);
    } else if (port == GPIOC) {
        rcu_periph_clock_enable(RCU_GPIOC);
    } else if (port == GPIOD) {
        rcu_periph_clock_enable(RCU_GPIOD);
    }
}

/**
 * @brief 初始化 UART：时钟、引脚、8N1、收发使能
 * @param self UART 对象
 * @retval none 外设非法时 is_initialized 保持 0
 * @note 不开启接收中断
 */
static void periph_uart_init(Periph_Uart_t *self)
{
    if (self == NULL) {
        return;
    }

    self->is_initialized = 0;

    if (self->uart_periph == USART0) {
        rcu_periph_clock_enable(RCU_USART0);
    } else if (self->uart_periph == USART2) {
        rcu_periph_clock_enable(RCU_USART2);
    } else if (self->uart_periph == UART3) {
        rcu_periph_clock_enable(RCU_UART3);
    } else if (self->uart_periph == UART4) {
        rcu_periph_clock_enable(RCU_UART4);
    } else {
        return;
    }

    uart_gpio_clock_enable(self->physical_pins.tx.port);
    uart_gpio_clock_enable(self->physical_pins.rx.port);

    gpio_init(self->physical_pins.tx.port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              self->physical_pins.tx.pin);
    gpio_init(self->physical_pins.rx.port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
              self->physical_pins.rx.pin);

    usart_deinit(self->uart_periph);
    usart_baudrate_set(self->uart_periph, self->baudrate);
    usart_word_length_set(self->uart_periph, USART_WL_8BIT);
    usart_stop_bit_set(self->uart_periph, USART_STB_1BIT);
    usart_parity_config(self->uart_periph, USART_PM_NONE);
    usart_hardware_flow_rts_config(self->uart_periph, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(self->uart_periph, USART_CTS_DISABLE);
    usart_receive_config(self->uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(self->uart_periph, USART_TRANSMIT_ENABLE);
    usart_enable(self->uart_periph);

    self->is_initialized = 1;
}

/**
 * @brief 关闭接收中断并复位 UART
 * @param self UART 对象
 * @retval none
 */
static void periph_uart_deinit(Periph_Uart_t *self)
{
    if (self == NULL) {
        return;
    }

    periph_uart_disable_rx_interrupt(self);
    usart_deinit(self->uart_periph);
    self->is_initialized = 0;
}

/**
 * @brief 发送一字节，等待 TBE
 * @param self UART 对象
 * @param data 数据
 * @param timeout 等待 TBE 的循环上限
 * @retval 1 成功
 * @retval -1 超时
 * @retval 0 未初始化或参数无效
 */
static int8_t periph_uart_send_byte(Periph_Uart_t *self, uint8_t data, uint32_t timeout)
{
    if ((self == NULL) || (self->is_initialized == 0U) || (timeout == 0U)) {
        return 0;
    }

    usart_data_transmit(self->uart_periph, data);
    while (RESET == usart_flag_get(self->uart_periph, USART_FLAG_TBE)) {
        if (timeout == 0U) {
            return -1;
        }
        timeout--;
    }
    return 1;
}

/**
 * @brief 发送若干字节
 * @param self UART 对象
 * @param data 数据缓冲
 * @param len 长度
 * @param timeout 整次发送共用的 TBE 等待上限
 * @retval 1 全部成功
 * @retval -1 中途超时
 * @retval 0 未初始化或参数无效
 */
static int8_t periph_uart_send_array(Periph_Uart_t *self, const uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint16_t i;
    int8_t ret;

    if ((self == NULL) || (self->is_initialized == 0U) || (data == NULL) ||
        (len == 0U) || (timeout == 0U)) {
        return 0;
    }

    for (i = 0U; i < len; i++) {
        ret = periph_uart_send_byte(self, data[i], timeout);
        if (ret != 1) {
            return ret;
        }
    }
    return 1;
}

/**
 * @brief 使能 RBNE 中断与对应 NVIC
 * @param self UART 对象
 * @retval none 未初始化时不操作
 */
static void periph_uart_enable_rx_interrupt(Periph_Uart_t *self)
{
    if ((self == NULL) || (self->is_initialized == 0U)) {
        return;
    }

    usart_interrupt_enable(self->uart_periph, USART_INT_RBNE);

    if (self->uart_periph == USART0) {
        nvic_irq_enable(USART0_IRQn, 2U, 0U);
    } else if (self->uart_periph == USART2) {
        nvic_irq_enable(USART2_IRQn, 2U, 0U);
    } else if (self->uart_periph == UART3) {
        nvic_irq_enable(UART3_IRQn, 2U, 0U);
    } else if (self->uart_periph == UART4) {
        nvic_irq_enable(UART4_IRQn, 2U, 0U);
    }
}

/**
 * @brief 关闭 RBNE 中断与对应 NVIC
 * @param self UART 对象
 * @retval none
 */
static void periph_uart_disable_rx_interrupt(Periph_Uart_t *self)
{
    if (self == NULL) {
        return;
    }

    usart_interrupt_disable(self->uart_periph, USART_INT_RBNE);

    if (self->uart_periph == USART0) {
        nvic_irq_disable(USART0_IRQn);
    } else if (self->uart_periph == USART2) {
        nvic_irq_disable(USART2_IRQn);
    } else if (self->uart_periph == UART3) {
        nvic_irq_disable(UART3_IRQn);
    } else if (self->uart_periph == UART4) {
        nvic_irq_disable(UART4_IRQn);
    }
}

/**
 * @brief 注册接收回调（不开关中断）
 * @param self UART 对象
 * @param callback 收到一字节后调用，传空则取消
 * @retval none
 */
static void periph_uart_register_rx_callback(Periph_Uart_t *self, UartRxCallback_t callback)
{
    if (self == NULL) {
        return;
    }
    self->rx_callback = callback;
}

/**
 * @brief USART0 接收中断：读一字节并回调
 */
void USART0_IRQHandler(void)
{
    uint8_t data;

    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) {
        data = (uint8_t)usart_data_receive(USART0);
        if (periph_uart0.rx_callback != NULL) {
            periph_uart0.rx_callback(data);
        }
    }
}

/**
 * @brief USART2 接收中断：读一字节并回调
 */
void USART2_IRQHandler(void)
{
    uint8_t data;

    if (RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)) {
        data = (uint8_t)usart_data_receive(USART2);
        if (periph_uart2.rx_callback != NULL) {
            periph_uart2.rx_callback(data);
        }
    }
}

/**
 * @brief UART3 接收中断：读一字节并回调
 */
void UART3_IRQHandler(void)
{
    uint8_t data;

    if (RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE)) {
        data = (uint8_t)usart_data_receive(UART3);
        if (periph_uart3.rx_callback != NULL) {
            periph_uart3.rx_callback(data);
        }
    }
}

/**
 * @brief UART4 接收中断：读一字节并回调
 */
void UART4_IRQHandler(void)
{
    uint8_t data;

    if (RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_RBNE)) {
        data = (uint8_t)usart_data_receive(UART4);
        if (periph_uart4.rx_callback != NULL) {
            periph_uart4.rx_callback(data);
        }
    }
}
