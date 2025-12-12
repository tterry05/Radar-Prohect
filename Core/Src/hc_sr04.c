/*
 * hc_sr04.c
 *
 *  Created on: Oct 23, 2025
 *      Author: Tyler W
 */

#include "hc_sr04.h"

// 32-bit timer
#define MAX_COUNTER_VALUE 0xffffffff

// The first capture timer counter value
static uint32_t firstInputCaptureValue = 0;
// The second capture timer counter value
static uint32_t secondInputCaptureValue = 0;
// The difference between two capture value
static uint32_t captureValueDifference = 0;
// Is the first value captured?
static uint8_t isFirstCaptured = 0;
// The measure distance
static float distance = 0;
// Delay function
static TIM_HandleTypeDef* delayTimer;


static void delay(TIM_HandleTypeDef* delayTimer, uint32_t delayTime);

static void delay(TIM_HandleTypeDef* delayTimer, uint32_t delayTime)
{
    __HAL_TIM_SET_COUNTER(delayTimer, 0);
    while(__HAL_TIM_GET_COUNTER(delayTimer) < delayTime)
    {
        ;
    }
}

void HCSR04_Init(HC_SR04_HandleTypeDef* hhc_sr04,
                GPIO_TypeDef* triggerPort, uint16_t triggerPin,
                TIM_HandleTypeDef* inputCaptureTimer, uint32_t inputCaptureTimerChannel,
                TIM_HandleTypeDef* hcsr_04_delayTimer)
{
    hhc_sr04->triggerPort = triggerPort;
    hhc_sr04->triggerPin = triggerPin;
    hhc_sr04->inputCaptureTimer = inputCaptureTimer;
    hhc_sr04->inputCaptureTimerChannel = inputCaptureTimerChannel;
    delayTimer = hcsr_04_delayTimer;

    if ( (hhc_sr04->inputCaptureTimerChannel) == TIM_CHANNEL_1) {
        hhc_sr04->activeChannel = HAL_TIM_ACTIVE_CHANNEL_1;
        hhc_sr04->interruptType = TIM_IT_CC1;
    } else if ( (hhc_sr04->inputCaptureTimerChannel) == TIM_CHANNEL_2) {
        hhc_sr04->activeChannel = HAL_TIM_ACTIVE_CHANNEL_2;
        hhc_sr04->interruptType = TIM_IT_CC2;
    } else if ( (hhc_sr04->inputCaptureTimerChannel) == TIM_CHANNEL_3) {
        hhc_sr04->activeChannel = HAL_TIM_ACTIVE_CHANNEL_3;
        hhc_sr04->interruptType = TIM_IT_CC3;
    } else if ( (hhc_sr04->inputCaptureTimerChannel) == TIM_CHANNEL_4) {
        hhc_sr04->activeChannel = HAL_TIM_ACTIVE_CHANNEL_4;
        hhc_sr04->interruptType = TIM_IT_CC4;
    }

    // state delay timer
    HAL_TIM_Base_Start(delayTimer);

    // start input capture timer
    HAL_TIM_IC_Start_IT(inputCaptureTimer, inputCaptureTimerChannel);
}

void HCSR04_Trigger(HC_SR04_HandleTypeDef* hhc_sr04)
{
    HAL_GPIO_WritePin(hhc_sr04->triggerPort, hhc_sr04->triggerPin, GPIO_PIN_RESET);
    //delay(delayTimer, 3);

    // pull the Trigger pin HIGH
    HAL_GPIO_WritePin(hhc_sr04->triggerPort, hhc_sr04->triggerPin, GPIO_PIN_SET);
    delay(delayTimer, 10); // wait for 10us

    // pull the Trigger pin LOW
    HAL_GPIO_WritePin(hhc_sr04->triggerPort, hhc_sr04->triggerPin, GPIO_PIN_RESET);
    //delay(delayTimer, 3);

    // enable input capture interrupt
    __HAL_TIM_ENABLE_IT(hhc_sr04->inputCaptureTimer, hhc_sr04->interruptType);
}

void HCSR04_IC_ISR(HC_SR04_HandleTypeDef* hhc_sr04, TIM_HandleTypeDef* htim)
{
    // Check if the interrupt is from the correct Timer Instance and Channel
    if ((htim->Instance == hhc_sr04->inputCaptureTimer->Instance) &&
        (htim->Channel == hhc_sr04->activeChannel))
    {

        if (isFirstCaptured == 0) // if the first value is not captured
        {
            // read the first value
            firstInputCaptureValue = HAL_TIM_ReadCapturedValue(htim, hhc_sr04->inputCaptureTimerChannel);

            isFirstCaptured = 1; // set the first captured flag to true

            // Now change the polarity to falling edge, to capture falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, hhc_sr04->inputCaptureTimerChannel,
                                            TIM_INPUTCHANNELPOLARITY_FALLING);
        }


        else if (isFirstCaptured == 1) // when the falling edge is captured
        {
            // read the second value
            secondInputCaptureValue = HAL_TIM_ReadCapturedValue(htim, hhc_sr04->inputCaptureTimerChannel);
            __HAL_TIM_SET_COUNTER(htim, 0); // reset the counter

            // Calculate the time difference, handling potential timer overflow
            if (secondInputCaptureValue > firstInputCaptureValue)
            {
                captureValueDifference = secondInputCaptureValue - firstInputCaptureValue;
            }
            // The timer overflow scenario
            else if (firstInputCaptureValue > secondInputCaptureValue)
            {
                captureValueDifference = (MAX_COUNTER_VALUE - firstInputCaptureValue) + secondInputCaptureValue;
            }



            // calculate the distance
            // distance = (high level time x velocity of sound (340m/s) / 2
            // 340m/s * 1us * 100cm/m = 0.034 cm/us
            distance = captureValueDifference * 0.034 / 2;

            isFirstCaptured = 0; // set the first captured back to false
            firstInputCaptureValue = 0;
            secondInputCaptureValue = 0;

            // Set polarity back to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, hhc_sr04->inputCaptureTimerChannel,
                                            TIM_INPUTCHANNELPOLARITY_RISING);

            // Disable the input capture interrupt
            __HAL_TIM_DISABLE_IT(htim, hhc_sr04->interruptType);
        }
    }


}


float HCSR04_ReadDistance (HC_SR04_HandleTypeDef* hhc_sr04)
{
    return distance;
}

