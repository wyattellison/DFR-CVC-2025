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

// Main uart sampling data vector
// Data array stored as single 64-bit integer
volatile uint32_t uart_time_last_sampled[UART_NUM_MESSAGES];
volatile uint32_t uart_sample_rates[UART_NUM_MESSAGES];

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

void uart_printSamplingRates(void) {
  static uint32_t last = 0;
  if ( HAL_GetTick() - last < UART_PRINT_SAMPLE_PERIOD ) {
      return;
  }
  last = HAL_GetTick();

  #ifdef CSV_PRINTOUTS
    uart_printf("%d,%d,%d,%d\n", 
    uart_sample_rates[UART_INVERTER1_MOTOR_SPEED], 
    uart_sample_rates[UART_INVERTER2_MOTOR_SPEED], 
    uart_sample_rates[UART_THROTTLE], 
    uart_sample_rates[UART_INVERTER1_TEMP1]);
  #else
    uart_printf("sampling rates: inverter1: %d, inverter2: %d, throttle: %d, i1_temp1: %d (all in ms)\n\r", 
    uart_sample_rates[UART_INVERTER1_MOTOR_SPEED], 
    uart_sample_rates[UART_INVERTER2_MOTOR_SPEED], 
    uart_sample_rates[UART_THROTTLE], 
    uart_sample_rates[UART_INVERTER1_TEMP1]);
  #endif


}

void uart_update_sampling_rate(uint32_t index) {
  uint32_t last_time = uart_time_last_sampled[index];
  uart_time_last_sampled[index] = HAL_GetTick();

  uart_sample_rates[index] = uart_time_last_sampled[index] - last_time;
}
