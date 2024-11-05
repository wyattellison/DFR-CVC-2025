/*
 * uart.h
 *
 * Created on November 3rd, 2024
 * Wyatt Ellison
 */
#ifndef CVC_UART_H
#define CVC_UART_H

#define ENABLE_UART_PRINTOUTS
//#define CSV_PRINTOUTS
#define UART_PRINT_SAMPLE_PERIOD 1000 // in ms

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void uart_printf(const char *format, ...);

void uart_printSamplingRates(void);

void uart_update_sampling_rate(uint32_t index);

typedef enum {
    UART_INVERTER1_MOTOR_SPEED,
    UART_INVERTER2_MOTOR_SPEED,
    UART_THROTTLE,
    UART_INVERTER1_TEMP1,
    // === Length of uart sample data array ===
    // This must be the last value in the enum
    UART_NUM_MESSAGES,
} UART_sample_index;

// Main uart sampling data vector
// Data array stored as single 64-bit integer
extern volatile uint32_t uart_time_last_sampled[UART_NUM_MESSAGES];
extern volatile uint32_t uart_sample_rates[UART_NUM_MESSAGES];

#endif  // CVC_CAN_H