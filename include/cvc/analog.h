/*
* analog.h
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/
#ifndef CVC_ANALOG_H
#define CVC_ANALOG_H

/**
 * @brief Starts ADC conversion.
 * @param None
 * @retval None
 */
void Analog_Configure(void);

/**
 * @brief Reads throttle position from ADC and stores it in global CVC data.
 * @param None
 * @retval None
 */
void Analog_ReadThrottle(void);

/**
 * @brief Reads LV voltage from ADC and stores it in global CVC data.
 * @param None
 * @retval None 
 */
void Analog_ReadLV(void);

#endif // CVC_ANALOG_H