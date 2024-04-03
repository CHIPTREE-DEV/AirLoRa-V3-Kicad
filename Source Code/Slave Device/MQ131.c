/*
 * MQ131.c
 *
 *  Created on: Oct 10, 2023
 *      Author: MSV
 */


#include "MQ131.h"
#include <math.h>

// Define your coefficients and load resistance here

#define COEFFICIENT_A_O3 28.62
#define COEFFICIENT_B_O3 -0.9015


#define R_LOAD_O3 10.0

void MQ131_Init(MQ131* mq131, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input) {
	mq131->hadc = hadc;
	mq131->adc_channel = adc_channel;
	mq131->v_in = v_input;
}

float MQ131_GetPPM_O3(const MQ131* mq131) {
    return (float)(COEFFICIENT_A_O3 * pow(MQ131_GetRatio(mq131), COEFFICIENT_B_O3));
}


float MQ131_VoltageConversion(uint16_t value, float v_in) {
    return (float)value * (v_in / 4095.0); // Assuming 12-bit ADC
}

float MQ131_GetRatio(const MQ131* mq131) {
    uint16_t value;
    HAL_ADC_Start(mq131->hadc);
    HAL_ADC_PollForConversion(mq131->hadc, HAL_MAX_DELAY);
    value = HAL_ADC_GetValue(mq131->hadc);
    HAL_ADC_Stop(mq131->hadc);

    float v_out = MQ131_VoltageConversion(value, mq131->v_in);
    return (mq131->v_in - v_out) / v_out;
}

float MQ131_GetSensorResistance(const MQ131* mq131) {
    return R_LOAD_O3 * MQ131_GetRatio(mq131);
}
