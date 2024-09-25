/*
* digital.h
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/
#ifndef CVC_DIGITAL_H
#define CVC_DIGITAL_H
#include <stm32f767xx.h>

// Enum for identifying 12V sense lines
typedef enum {
    PrechargeBtn = 0,
    LVChargeState,
    AIR_12V,
    AIR_Power,
    DCDCConverter,
    CockpitBRB,
    BOT,
} CVC_12V_id_t;

/**
 * @brief Function for reading state of 12V sense line
 * @param id - 12V sense line to check
 * @return 1 if 12V sense line is high, 0 if 12V sense line is low 
 */
uint8_t CVC_12V_Read(CVC_12V_id_t id);

/**
 * @brief Function for reading IMD latching relay state
 * @param None
 * @retval uint8_t IMD relay state 
 */
uint8_t CVC_12V_IMD_State(void);

/**
 * @brief Function for reading BMS latching relay state
 * @param None
 * @retval uint8_t BMS relay state 
 */
uint8_t CVC_12V_BMS_State(void);

/**
 * @brief Reads state of each 12V sense line and stores it in global CVC data
 * @param None
 * @retval None
 */
void CVC_12V_ReadAll(void);

#endif // CVC_DIGITAL_H