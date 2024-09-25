/*
* digital.c
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/
#include <stm32f7xx_hal_gpio.h>
#include <main.h>
#include <cvc/digital.h>
#include <cvc/data.h>

uint8_t CVC_12V_Read(CVC_12V_id_t id) {
    switch (id) {
        case PrechargeBtn:
            return HAL_GPIO_ReadPin(SENSE_12V_7_GPIO_Port, SENSE_12V_7_Pin) ? 0 : 1;
        case LVChargeState:
            return HAL_GPIO_ReadPin(SENSE_12V_6_GPIO_Port, SENSE_12V_6_Pin) ? 0 : 1;
        case AIR_12V:
            return HAL_GPIO_ReadPin(SENSE_12V_5_GPIO_Port, SENSE_12V_5_Pin) ? 0 : 1;
        case AIR_Power:
            return HAL_GPIO_ReadPin(SENSE_12V_4_GPIO_Port, SENSE_12V_4_Pin) ? 0 : 1;
        case DCDCConverter:
            return HAL_GPIO_ReadPin(SENSE_12V_3_GPIO_Port, SENSE_12V_3_Pin) ? 0 : 1;
        case CockpitBRB:
            return HAL_GPIO_ReadPin(SENSE_12V_2_GPIO_Port, SENSE_12V_2_Pin) ? 0 : 1;
        case BOT:
            return HAL_GPIO_ReadPin(SENSE_12V_1_GPIO_Port, SENSE_12V_1_Pin) ? 0 : 1;
        default:
            return 0;
    }
}

uint8_t CVC_12V_IMD_State() {
    return HAL_GPIO_ReadPin(IMD_STATE_GPIO_Port, IMD_STATE_Pin);
}

uint8_t CVC_12V_BMS_State() {
    return HAL_GPIO_ReadPin(BMS_STATE_GPIO_Port, BMS_STATE_Pin);
}

void CVC_12V_ReadAll(void) {
    CVC_data[CVC_PRECHARGE_BTN] = CVC_12V_Read(PrechargeBtn);
    CVC_data[CVC_LV_CHARGE_STATE] = CVC_12V_Read(LVChargeState);
    CVC_data[CVC_AIR_1_STATE] = CVC_12V_Read(AIR_Power);
    CVC_data[CVC_AIR_2_STATE] = CVC_12V_Read(AIR_12V);
    CVC_data[CVC_DCDC_STATE] = CVC_12V_Read(DCDCConverter);
    CVC_data[CVC_COCKPIT_BRB_STATE] = CVC_12V_Read(CockpitBRB);
    CVC_data[CVC_BOT_STATE] = CVC_12V_Read(BOT);
    CVC_data[CVC_IMD_STATE] = CVC_12V_IMD_State();
    CVC_data[CVC_BMS_STATE] = CVC_12V_BMS_State();
}