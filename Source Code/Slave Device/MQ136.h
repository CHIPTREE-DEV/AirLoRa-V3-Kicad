/*
 * MQ136.h
 *
 *  Created on: Oct 10, 2023
 *      Author: MSV
 */

#ifndef INC_MQ136_H_
#define INC_MQ136_H_

#include "stm32f4xx_hal.h"

// Define the MQ7 structure
typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t adc_channel;
    float v_in;
}MQ136;

// Function prototypes
void MQ136_Init(MQ136* mq136, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input);
float MQ136_GetPPM_H2S(const MQ136* mq136);
float MQ136_GetSensorResistance(const MQ136* mq136);
float MQ136_GetRatio(const MQ136* mq136);

#endif /* INC_MQ136_H_ */
