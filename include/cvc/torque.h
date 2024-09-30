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
#define TORQUE_LIMIT 100.0           // Percentage of nominal torque (0.0 - 100.0)
#define DISABLE_ON_ZERO_THROTTLE 0  // Disable inverters when throttle is at 0%
#define INVERTER_ACCEL_AVERAGE 3    // Number of acceleration values to average

#define TORQUE_VECTORING_GAIN 0.75 // Gain for torque vectoring
#define STEERING_POT_LEFT 0.56     // Left-most position of steering potentiometer
#define STEERING_POT_RIGHT 4.09    // Right-most position of steering potentiometer

#define ACCEL_INT_FLOAT_SCALING 10000     // Scaling factor for acceleration values
#define TRACTION_CONTROL_MAX_ACCEL 0.351  // Maximum acceleration in RPM/ms
#define TRACTION_CONTROL_GAIN 1.0         // Gain for traction control

void Torque_CalculateAcceleration(void);

void Torque_CalculateTorque(void);

void Torque_SendTorque(void);

#endif  // CVC_TORQUE_H