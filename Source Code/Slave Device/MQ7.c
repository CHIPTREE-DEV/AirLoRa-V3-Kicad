#include <math.h>
#include <MQ7.h>

// Define your coefficients and load resistance here
#define COEFFICIENT_A_CO 24.01
#define COEFFICIENT_B_CO -0.675

//------- COEFFICIENT Parameter of Liquefied petroleum gas ------
#define COEFFICIENT_A_LPG 14.878
#define COEFFICIENT_B_LPG -0.132
//------- COEFFICIENT Parameter of Hydrogen ------
#define COEFFICIENT_A_H2 19.887
#define COEFFICIENT_B_H2 -0.714

#define R_LOAD 10.0

void MQ7_Init(MQ7* mq7, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input) {
    mq7->hadc = hadc;
    mq7->adc_channel = adc_channel;
    mq7->v_in = v_input;
}

//=================== PPM of CO ============================
float MQ7_GetPPM_CO(const MQ7* mq7) {
    return (float)(COEFFICIENT_A_CO * pow(MQ7_GetRatio(mq7), COEFFICIENT_B_CO));
}
//=================== PPM of LPG ============================
float MQ7_GetPPM_LPG(const MQ7* mq7) {
    return (float)(COEFFICIENT_A_LPG * pow(MQ7_GetRatio(mq7), COEFFICIENT_B_LPG));
}
//=================== PPM of H2 ============================
float MQ7_GetPPM_H2(const MQ7* mq7) {
    return (float)(COEFFICIENT_A_H2 * pow(MQ7_GetRatio(mq7), COEFFICIENT_B_H2));
}
//==========================================================

float MQ7_VoltageConversion(uint16_t value, float v_in) {
    return (float)value * (v_in / 4095.0); // Assuming 12-bit ADC
}

float MQ7_GetRatio(const MQ7* mq7) {
    uint16_t value;
    HAL_ADC_Start(mq7->hadc);
    HAL_ADC_PollForConversion(mq7->hadc, HAL_MAX_DELAY);
    value = HAL_ADC_GetValue(mq7->hadc);
    HAL_ADC_Stop(mq7->hadc);

    float v_out = MQ7_VoltageConversion(value, mq7->v_in);
    return (mq7->v_in - v_out) / v_out;
}

float MQ7_GetSensorResistance(const MQ7* mq7) {
    return R_LOAD * MQ7_GetRatio(mq7);
}
