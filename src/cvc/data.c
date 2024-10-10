/*
* data.c
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/

#include <cvc/data.h>

// Main CAN data vector
// Data array stored as single 64-bit integer
volatile uint64_t CAN_data[NUM_MESSAGES];
// CAN data parsed flag vector
volatile bool CAN_data_parsed[NUM_MESSAGES];
// CVC data vector
// Data stored as single 64-bit integer
volatile uint64_t CVC_data[NUM_DATA_VALUES];