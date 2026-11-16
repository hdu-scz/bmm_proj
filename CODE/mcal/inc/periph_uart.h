/**
 * @file periph_uart.h
 * @brief UART 外设对象头文件 (MCAL 层)
 * @author 
 * @date 2026-07-06
 */

#ifndef PERIPH_UART_H
#define PERIPH_UART_H

#include <stdint.h>
#include "gd32f30x.h"

/* ==================== UART 接收回调函数类型 ==================== */
typedef void (*UartRxCallback_t)(uint8_t data);

/* ==================== UART 对象结构体定义 ==================== */
typedef struct Periph_Uart
{
    uint32_t    uart_periph;         // UART 外设基地址
    uint32_t    baudrate;            // 波特率
    uint32_t    rx_port, rx_pin;     // RX 引脚
    uint32_t    tx_port, tx_pin;     // TX 引脚
    uint8_t     is_initialized;      // 初始化完成标志

    /* 中断接收相关 */
    UartRxCallback_t rx_callback;    // 接收回调函数

    void     (*init)(struct Periph_Uart *self);
    void     (*deinit)(struct Periph_Uart *self);

    void     (*send_byte)(struct Periph_Uart *self, uint8_t data);
    void     (*send_array)(struct Periph_Uart *self, uint8_t *data, uint16_t len);

    void     (*enable_rx_interrupt)(struct Periph_Uart *self);
    void     (*disable_rx_interrupt)(struct Periph_Uart *self);
    void     (*register_rx_callback)(struct Periph_Uart *self, UartRxCallback_t callback);

} Periph_Uart_t;

/* 全局 UART 对象声明 */
extern Periph_Uart_t periph_uart0;   // PA10(RX), PA9(TX)
extern Periph_Uart_t periph_uart2;   // PB10(RX), PB11(TX)
extern Periph_Uart_t periph_uart1;   // PC11(RX), PC10(TX)
extern Periph_Uart_t periph_uart3;   // PD2(RX), PC12(TX)

#endif /* PERIPH_UART_H */
