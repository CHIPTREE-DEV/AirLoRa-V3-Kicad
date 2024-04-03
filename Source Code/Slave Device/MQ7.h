#ifndef MQ7_CO_H_
#define MQ7_CO_H_

#include "stm32f4xx_hal.h"

// Define the MQ7 structure
typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t adc_channel;
    float v_in;
} MQ7;

// Function prototypes
void MQ7_Init(MQ7* mq7, ADC_HandleTypeDef* hadc, uint32_t adc_channel, float v_input);
float MQ7_GetPPM_CO (const MQ7* mq7);
float MQ7_GetPPM_LPG (const MQ7* mq7);
float MQ7_GetPPM_H2 (const MQ7* mq7);
float MQ7_GetSensorResistance(const MQ7* mq7);
float MQ7_GetRatio(const MQ7* mq7);

#endif /* MQ7_CO_H_ */
