/*
 * MQ136.c
 *
 *  Created on: Oct 10, 2023
 *      Author: MSV
 */


#include "MQ136.h"
#include <math.h>

// Define your coefficients and load resistance here

#define COEFFICIENT_A_H2S 0.1546
#define COEFFICIENT_B_H2S -3.678

#define R_LOAD 10.0

void MQ136_Init(MQ136* mq136, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input) {
	mq136->hadc = hadc;
	mq136->adc_channel = adc_channel;
	mq136->v_in = v_input;
}

float MQ136_GetPPM_H2S(const MQ136* mq136) {
    return (float)(COEFFICIENT_A_H2S * pow(MQ136_GetRatio(mq136), COEFFICIENT_B_H2S));
}

float MQ136_VoltageConversion(uint16_t value, float v_in) {
    return (float)value * (v_in / 4095.0); // Assuming 12-bit ADC
}

float MQ136_GetRatio(const MQ136* mq136) {
    uint16_t value;
    HAL_ADC_Start(mq136->hadc);
    HAL_ADC_PollForConversion(mq136->hadc, HAL_MAX_DELAY);
    value = HAL_ADC_GetValue(mq136->hadc);
    HAL_ADC_Stop(mq136->hadc);

    float v_out = MQ136_VoltageConversion(value, mq136->v_in);
    return (mq136->v_in - v_out) / v_out;
}

float MQ136_GetSensorResistance(const MQ136* mq136) {
    return R_LOAD * MQ136_GetRatio(mq136);
}
