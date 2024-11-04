/*
 * uart.c
 *
 * Created on Nov 3 2024
 * Wyatt Ellison
 */

#include <stdio.h>
#include <stdarg.h>
#include <main.h>
#include <cvc/uart.h>


extern UART_HandleTypeDef huart3;

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#ifdef ENABLE_UART_PRINTOUTS
void uart_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
#else
void uart_printf(const char *format, ...) {
    //do nothing if ENABLE_UART_PRINTOUTS is not defined
}
#endif

