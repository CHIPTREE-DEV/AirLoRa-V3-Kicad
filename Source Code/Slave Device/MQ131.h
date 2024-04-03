/*
 * MQ131.h
 *
 *  Created on: Oct 10, 2023
 *      Author: MSV
 */

#ifndef INC_MQ131_H_
#define INC_MQ131_H_

#include "stm32f4xx_hal.h"

// Define the MQ7 structure
typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t adc_channel;
    float v_in;
}MQ131;

// Function prototypes
void MQ131_Init(MQ131* mq131, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input);
float MQ131_GetPPM_O3(const MQ131* mq131);
float MQ131_GetSensorResistance(const MQ131* mq131);
float MQ131_GetRatio(const MQ131* mq131);

#endif /* INC_MQ131_H_ */
