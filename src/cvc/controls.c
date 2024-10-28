/*
 * controls.c
 *
 * Created on October 22, 2024
 * Wyatt Ellison
 */

#include <cvc/can.h>
#include <cvc/data.h>
#include <cvc/parse.h>
#include <cvc/statemachine.h>
#include <cvc/torque.h>
#include <cvc/controls.h>
#include <main.h>
#include <stdbool.h>


void Controls_ProcessInputs() {

  Torque_CalculateAvailableTorque(); // calculate the available torque and put it in CVC data
  Throttle_ProcessThrottle(); // process throttle input from CAN bus and put it in CVC data

}

void Controls_ProcessOutputs() {
  
  Torque_CalculateAcceleration(); // calculates acceleration
  Torque_CalculateTorque(); // calcualtes the standard, open-loop torque

}

