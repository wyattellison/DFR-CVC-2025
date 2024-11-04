/*
 * uart.h
 *
 * Created on November 3rd, 2024
 * Wyatt Ellison
 */
#ifndef CVC_UART_H
#define CVC_UART_H

#define ENABLE_UART_PRINTOUTS

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void uart_printf(const char *format, ...);

#endif  // CVC_CAN_H