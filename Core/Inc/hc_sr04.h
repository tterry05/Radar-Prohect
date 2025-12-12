/*
 * hc_sr04.h
 *
 *  Created on: Oct 23, 2025
 *      Author: Tyler W
 */

#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_



#include "stm32f4xx_hal.h"

typedef struct
{
    GPIO_TypeDef* triggerPort;
    uint16_t                triggerPin;
    uint32_t                inputCaptureTimerChannel;
    TIM_HandleTypeDef* inputCaptureTimer;
    HAL_TIM_ActiveChannel   activeChannel;
    uint32_t                interruptType;
} HC_SR04_HandleTypeDef;

void HCSR04_Init(HC_SR04_HandleTypeDef* hhc_sr04,
                GPIO_TypeDef* triggerPort, uint16_t triggerPin,
                TIM_HandleTypeDef* inputCaptureTimer,
                uint32_t inputCaptureTimerChannel,
                TIM_HandleTypeDef* hcsr_04_delayTimer);

void HCSR04_Trigger(HC_SR04_HandleTypeDef* hhc_sr04);
float HCSR04_ReadDistance(HC_SR04_HandleTypeDef* hhc_sr04);
void HCSR04_IC_ISR(HC_SR04_HandleTypeDef* hhc_sr04, TIM_HandleTypeDef* htim);


#endif /* INC_HC_SR04_H_ */
