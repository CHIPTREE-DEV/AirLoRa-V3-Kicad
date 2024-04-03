#include "MQ135.h"
#include <math.h>

// Define your coefficients and load resistance here

#define COEFFICIENT_A_CO2 5.8549
#define COEFFICIENT_B_CO2 -0.373

#define COEFFICIENT_A_NH4 6.6814
#define COEFFICIENT_B_NH4 -0.408

#define R_LOAD 10.0

void MQ135_Init(MQ135* mq135, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input) {
	mq135->hadc = hadc;
	mq135->adc_channel = adc_channel;
	mq135->v_in = v_input;
}

float MQ135_GetPPM_CO2(const MQ135* mq135) {
    return (float)(COEFFICIENT_A_CO2 * pow(MQ135_GetRatio(mq135), COEFFICIENT_B_CO2));
}
float MQ135_GetPPM_NH4(const MQ135* mq135) {
    return (float)(COEFFICIENT_A_NH4 * pow(MQ135_GetRatio(mq135), COEFFICIENT_B_NH4));
}

float MQ135_VoltageConversion(uint16_t value, float v_in) {
    return (float)value * (v_in / 4095.0); // Assuming 12-bit ADC
}

float MQ135_GetRatio(const MQ135* mq135) {
    uint16_t value;
    HAL_ADC_Start(mq135->hadc);
    HAL_ADC_PollForConversion(mq135->hadc, HAL_MAX_DELAY);
    value = HAL_ADC_GetValue(mq135->hadc);
    HAL_ADC_Stop(mq135->hadc);

    float v_out = MQ135_VoltageConversion(value, mq135->v_in);
    return (mq135->v_in - v_out) / v_out;
}

float MQ135_GetSensorResistance(const MQ135* mq135) {
    return R_LOAD * MQ135_GetRatio(mq135);
}
