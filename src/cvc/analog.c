/*
* analog.c
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/
#include <stm32f7xx_hal_adc.h>
#include <cvc/data.h>
#include <main.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void Analog_Configure() {
    uint32_t adc_value = 0;
    CVC_data[CVC_LV_VOLTAGE] = adc_value;
    CVC_data[CVC_THROTTLE_ADC] = adc_value;
}

void Analog_ReadLV() {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        CVC_data[CVC_LV_VOLTAGE] = adc_value;
    }
}

void Analog_ReadThrottle() {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc2);
        CVC_data[CVC_THROTTLE_ADC] = adc_value;
    }
}
