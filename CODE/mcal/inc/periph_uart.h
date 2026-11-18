/**
 * @file periph_uart.h
 * @brief UART 外设对象头文件 (MCAL 层)
 * @author
 * @date 2026-07-06
 */

#ifndef PERIPH_UART_H
#define PERIPH_UART_H

#include "stdinc.h"
#include "gd32f30x.h"

typedef void (*UartRxCallback_t)(uint8_t data);

typedef struct {
    uint32_t port;
    uint32_t pin;
} UartPin_t;

typedef struct {
    UartPin_t rx;
    UartPin_t tx;
} UartPins_t;

typedef struct Periph_Uart
{
    uint32_t    uart_periph;
    uint32_t    baudrate;
    UartPins_t  physical_pins;
    uint8_t     is_initialized;

    UartRxCallback_t rx_callback;

    void     (*init)(void);
    void     (*deinit)(void);

    /* 成功 1，等待 TBE 超时 -1，未初始化或参数无效 0 */
    int8_t   (*send_byte)(uint8_t data, uint32_t timeout);
    int8_t   (*send_array)(const uint8_t *data, uint16_t len, uint32_t timeout);

    void     (*enable_rx_interrupt)(void);
    void     (*disable_rx_interrupt)(void);
    void     (*register_rx_callback)(UartRxCallback_t callback);

} Periph_Uart_t;

extern Periph_Uart_t periph_uart0;   /* USART0: PA9 TX, PA10 RX */
extern Periph_Uart_t periph_uart2;   /* USART2: PB10 TX, PB11 RX */
extern Periph_Uart_t periph_uart3;   /* UART3:  PC10 TX, PC11 RX */
extern Periph_Uart_t periph_uart4;   /* UART4:  PC12 TX, PD2 RX  */

#endif /* PERIPH_UART_H */
