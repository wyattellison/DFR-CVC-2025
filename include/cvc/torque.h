/*
 * torque.h
 *
 * Created on September 20, 2024
 * Andrei Gerashchenko
 */
#ifndef CVC_TORQUE_H
#define CVC_TORQUE_H

#include <cvc/data.h>

#define TORQUE_PERIOD 10            // ms
#define NOMINAL_TORQUE 121          // Nm
#define REVERSE_TORQUE_LIMIT 20.0   // Percentage of nominal torque (0.0 - 100.0)
#define TORQUE_LIMIT 100.0          // Percentage of nominal torque (0.0 - 100.0)
#define DISABLE_ON_ZERO_THROTTLE 0  // Disable inverters when throttle is at 0%
#define INVERTER_ACCEL_AVERAGE 3    // Number of acceleration values to average

#define LEFT_MOTOR_ENABLE 1   // Left motor enable bit
#define RIGHT_MOTOR_ENABLE 1  // Right motor enable bit

#define TORQUE_VECTORING_GAIN 0.25  // Gain for torque vectoring
#define STEERING_POT_LEFT 0.50      // Left-most position of steering potentiometer
#define STEERING_POT_RIGHT 4.10     // Right-most position of steering potentiometer

#define ACCEL_INT_FLOAT_SCALING 10000     // Scaling factor for acceleration values
#define TRACTION_CONTROL_MAX_ACCEL 0.351  // Maximum acceleration in RPM/ms
#define TRACTION_CONTROL_GAIN 0.0         // Gain for traction control

#define BATTERY_CURRENT_LIMIT 100.0               // Amps
#define BATTERY_INTERNAL_RESISTANCE 0.013         // Ohms
#define BATTERY_CELLS_PARALLEL 12                 // Number of cells in parallel
#define BATTERY_CELLS_SERIES 60                   // Number of cells in series
#define UNDERVOLTAGE_SCALING_FACTOR 0.1047197551  // 2 * pi / 60

void Torque_CalculateAcceleration(void);

void Torque_CalculateTorque(void);

void Torque_SendTorque(void);

void Torque_CalculateAvailableTorque(void);

#endif  // CVC_TORQUE_H