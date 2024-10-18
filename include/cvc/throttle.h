/*
 * throttle.h
 *
 * Created on September 20, 2024
 * Andrei Gerashchenko
 */
#ifndef CVC_THROTTLE_H
#define CVC_THROTTLE_H

#define THROTTLE_INVALID_TIMER 250                  // Time before throttle is marked invalid in milliseconds
#define MAX_THROTTLE_DIFFERENCE (int)(0.05 * 4095)  // 5% of 12-bit ADC range
#define MIN_THROTTLE_RANGE 400                      // Lower bound for valid throttle range
#define MAX_THROTTLE_RANGE 4095                     // Upper bound for valid throttle range
#define THROTTLE_ZERO 700                           // 0% throttle
#define THROTTLE_FULL 2650                          // 100% throttle
#define MAX_THROTTLE_CONTINUITY 1.00                // Maximum throttle continuity analog input voltage

void Throttle_ProcessThrottle(void);

#endif  // CVC_THROTTLE_H