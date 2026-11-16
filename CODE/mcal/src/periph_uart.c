/**
 * @file periph_uart.c
 * @brief UART 外设对象实现 (MCAL 层)
 * @author 
 * @date 2026-07-06
 */

#include "periph_uart.h"
#include "gd32f30x_usart.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"

/* ==================== UART0 引脚映射 (PA10 RX, PA9 TX) ==================== */
#define UART0_RX_PORT   GPIOA
#define UART0_RX_PIN    GPIO_PIN_10
#define UART0_TX_PORT   GPIOA
#define UART0_TX_PIN    GPIO_PIN_9

/* ==================== UART2 引脚映射 (PB10 RX, PB11 TX) ==================== */
#define UART2_RX_PORT   GPIOB
#define UART2_RX_PIN    GPIO_PIN_10
#define UART2_TX_PORT   GPIOB
#define UART2_TX_PIN    GPIO_PIN_11

/* ==================== UART1 引脚映射 (PC11 RX, PC10 TX) ==================== */
#define UART1_RX_PORT   GPIOC
#define UART1_RX_PIN    GPIO_PIN_11
#define UART1_TX_PORT   GPIOC
#define UART1_TX_PIN    GPIO_PIN_10

/* ==================== UART3 引脚映射 (PD2 RX, PC12 TX) ==================== */
#define UART3_RX_PORT   GPIOD
#define UART3_RX_PIN    GPIO_PIN_2
#define UART3_TX_PORT   GPIOC
#define UART3_TX_PIN    GPIO_PIN_12

/* ==================== 全局 UART 对象实例 ==================== */
Periph_Uart_t periph_uart0;
Periph_Uart_t periph_uart2;
Periph_Uart_t periph_uart1;
Periph_Uart_t periph_uart3;

/* ==================== 私有函数声明 ==================== */
static void uart_init(Periph_Uart_t *self);
static void uart_deinit(Periph_Uart_t *self);
static void uart_send_byte(Periph_Uart_t *self, uint8_t data);
static void uart_send_array(Periph_Uart_t *self, uint8_t *data, uint16_t len);
static void uart_enable_rx_interrupt(Periph_Uart_t *self);
static void uart_disable_rx_interrupt(Periph_Uart_t *self);
static void uart_register_rx_callback(Periph_Uart_t *self, UartRxCallback_t callback);

/* ==================== UART 对象静态初始化 ==================== */
Periph_Uart_t periph_uart0 = {
    .uart_periph = USART0,
    .baudrate    = 115200,
    .rx_port     = UART0_RX_PORT,
    .rx_pin      = UART0_RX_PIN,
    .tx_port     = UART0_TX_PORT,
    .tx_pin      = UART0_TX_PIN,
    .is_initialized = 0,
    .rx_callback = NULL,
    .init        = uart_init,
    .deinit      = uart_deinit,
    .send_byte   = uart_send_byte,
    .send_array  = uart_send_array,
    .enable_rx_interrupt  = uart_enable_rx_interrupt,
    .disable_rx_interrupt = uart_disable_rx_interrupt,
    .register_rx_callback = uart_register_rx_callback
};

Periph_Uart_t periph_uart2 = {
    .uart_periph = USART2,
    .baudrate    = 115200,
    .rx_port     = UART2_RX_PORT,
    .rx_pin      = UART2_RX_PIN,
    .tx_port     = UART2_TX_PORT,
    .tx_pin      = UART2_TX_PIN,
    .is_initialized = 0,
    .rx_callback = NULL,
    .init        = uart_init,
    .deinit      = uart_deinit,
    .send_byte   = uart_send_byte,
    .send_array  = uart_send_array,
    .enable_rx_interrupt  = uart_enable_rx_interrupt,
    .disable_rx_interrupt = uart_disable_rx_interrupt,
    .register_rx_callback = uart_register_rx_callback
};

Periph_Uart_t periph_uart1 = {
    .uart_periph = USART1,
    .baudrate    = 115200,
    .rx_port     = UART1_RX_PORT,
    .rx_pin      = UART1_RX_PIN,
    .tx_port     = UART1_TX_PORT,
    .tx_pin      = UART1_TX_PIN,
    .is_initialized = 0,
    .rx_callback = NULL,
    .init        = uart_init,
    .deinit      = uart_deinit,
    .send_byte   = uart_send_byte,
    .send_array  = uart_send_array,
    .enable_rx_interrupt  = uart_enable_rx_interrupt,
    .disable_rx_interrupt = uart_disable_rx_interrupt,
    .register_rx_callback = uart_register_rx_callback
};

Periph_Uart_t periph_uart3 = {
    .uart_periph = UART3,
    .baudrate    = 115200,
    .rx_port     = UART3_RX_PORT,
    .rx_pin      = UART3_RX_PIN,
    .tx_port     = UART3_TX_PORT,
    .tx_pin      = UART3_TX_PIN,
    .is_initialized = 0,
    .rx_callback = NULL,
    .init        = uart_init,
    .deinit      = uart_deinit,
    .send_byte   = uart_send_byte,
    .send_array  = uart_send_array,
    .enable_rx_interrupt  = uart_enable_rx_interrupt,
    .disable_rx_interrupt = uart_disable_rx_interrupt,
    .register_rx_callback = uart_register_rx_callback
};

/* ==================== 方法实现 ==================== */

static void uart_init(Periph_Uart_t *self)
{
    self->is_initialized = 0;

    /* 使能 UART 时钟和 GPIO 时钟 */
    if (self->uart_periph == USART0) {
        rcu_periph_clock_enable(RCU_USART0);
        rcu_periph_clock_enable(RCU_GPIOA);
    } else if (self->uart_periph == USART2) {
        rcu_periph_clock_enable(RCU_USART2);
        rcu_periph_clock_enable(RCU_GPIOB);
    } else if (self->uart_periph == USART1) {
        rcu_periph_clock_enable(RCU_USART1);
        rcu_periph_clock_enable(RCU_GPIOC);
    } else if (self->uart_periph == UART3) {
        rcu_periph_clock_enable(RCU_UART3);
        rcu_periph_clock_enable(RCU_GPIOC);
        rcu_periph_clock_enable(RCU_GPIOD);
    }

    /* 配置引脚 */
    gpio_init(self->tx_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, self->tx_pin);
    gpio_init(self->rx_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, self->rx_pin);

    /* UART 参数配置 */
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

static void uart_deinit(Periph_Uart_t *self)
{
    usart_deinit(self->uart_periph);
    self->is_initialized = 0;
}

static void uart_send_byte(Periph_Uart_t *self, uint8_t data)
{
    if (!self->is_initialized) return;
    usart_data_transmit(self->uart_periph, data);
    while (RESET == usart_flag_get(self->uart_periph, USART_FLAG_TBE));
}

static void uart_send_array(Periph_Uart_t *self, uint8_t *data, uint16_t len)
{
    if (!self->is_initialized) return;
    for (uint16_t i = 0; i < len; i++) {
        uart_send_byte(self, data[i]);
    }
}

static void uart_enable_rx_interrupt(Periph_Uart_t *self)
{
    if (!self->is_initialized) return;

    usart_interrupt_enable(self->uart_periph, USART_INT_RBNE);

    if (self->uart_periph == USART0)      nvic_irq_enable(USART0_IRQn, 2, 0);
    else if (self->uart_periph == USART2) nvic_irq_enable(USART2_IRQn, 2, 0);
    else if (self->uart_periph == USART1) nvic_irq_enable(USART1_IRQn, 2, 0);
    else if (self->uart_periph == UART3)  nvic_irq_enable(UART3_IRQn, 2, 0);
}

static void uart_disable_rx_interrupt(Periph_Uart_t *self)
{
    if (!self->is_initialized) return;

    usart_interrupt_disable(self->uart_periph, USART_INT_RBNE);

    if (self->uart_periph == USART0)      nvic_irq_disable(USART0_IRQn);
    else if (self->uart_periph == USART2) nvic_irq_disable(USART2_IRQn);
    else if (self->uart_periph == USART1) nvic_irq_disable(USART1_IRQn);
    else if (self->uart_periph == UART3)  nvic_irq_disable(UART3_IRQn);
}

static void uart_register_rx_callback(Periph_Uart_t *self, UartRxCallback_t callback)
{
    self->rx_callback = callback;
}

/* ==================== UART 中断服务函数 ==================== */

void USART0_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART0);
        if (periph_uart0.rx_callback != NULL) {
            periph_uart0.rx_callback(data);
        }
    }
}

void USART2_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART2);
        if (periph_uart2.rx_callback != NULL) {
            periph_uart2.rx_callback(data);
        }
    }
}

void USART1_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(USART1);
        if (periph_uart1.rx_callback != NULL) {
            periph_uart1.rx_callback(data);
        }
    }
}

void UART3_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(UART3);
        if (periph_uart3.rx_callback != NULL) {
            periph_uart3.rx_callback(data);
        }
    }
}