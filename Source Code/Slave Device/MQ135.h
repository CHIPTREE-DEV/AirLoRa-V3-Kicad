/*
 * MQ135.h
 *
 *  Created on: Oct 5, 2023
 *      Author: MSV
 */
#ifndef MQ135_H_
#define MQ135_H_

#include "stm32f4xx_hal.h"

// Define the MQ7 structure
typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t adc_channel;
    float v_in;
}MQ135;

// Function prototypes
void MQ135_Init(MQ135* mq135, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input);
float MQ135_GetPPM_NH4(const MQ135* mq135);
float MQ135_GetPPM_CO2(const MQ135* mq135);
float MQ135_GetSensorResistance(const MQ135* mq135);
float MQ135_GetRatio(const MQ135* mq135);

#endif /* INC_MQ135_H_ */
