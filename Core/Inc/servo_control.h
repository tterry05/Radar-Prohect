/**
 ******************************************************************************
 * @file    servo_control.h
 * @brief   Servo motor control for SG90 radar sweep
 * @author  Team Lane 6 - Tommy Terry
 ******************************************************************************
 * @attention
 *
 * Servo sweeps 180 degrees in back-and-forth pattern:
 * Start: 90° → Right to 180° → Left to 0° → Right to 180° → repeat
 *
 * PWM Configuration:
 * - Timer: TIM2, Channel 1 (PA5 - Arduino A3 header)
 * - Frequency: 50Hz (20ms period)
 * - Pulse width: 1ms (0°) to 2ms (180°)
 *
 * NOTE: PA0 is the USER BUTTON on STM32F413H-Discovery, so we use PA5 instead!
 *
 ******************************************************************************
 */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_


#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Servo Parameters - Extended range for full 180° rotation */
#define SERVO_MIN_PULSE_US      500     // 0.5ms = 0 degrees
#define SERVO_MAX_PULSE_US      2500    // 2.5ms = 180 degrees
#define SERVO_CENTER_PULSE_US   1500    // 1.5ms = 90 degrees

#define SERVO_MIN_ANGLE         0
#define SERVO_MAX_ANGLE         180
#define SERVO_CENTER_ANGLE      90

#define SERVO_STEP_ANGLE        2       // Degrees per step (90 steps for full sweep)

/* Sweep Direction */
typedef enum {
    SWEEP_RIGHT,    // Moving from center (90°) toward 180°
    SWEEP_LEFT      // Moving from 180° toward 0° (or 0° toward 180°)
} SweepDirection_t;

/* Servo State Structure */
typedef struct {
    uint16_t currentAngle;          // Current angle (0-180)
    SweepDirection_t direction;     // Current sweep direction
    bool initialized;               // Initialization flag
} ServoState_t;

/* Function Prototypes */

/**
 * @brief  Initialize servo motor and move to center position (90°)
 * @param  htim: pointer to TIM handle (TIM2)
 * @retval HAL status
 */
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim);

/**
 * @brief  Set servo to specific angle (0-180 degrees)
 * @param  angle: target angle in degrees
 * @retval HAL status
 */
HAL_StatusTypeDef Servo_SetAngle(uint16_t angle);

/**
 * @brief  Perform one step of the radar sweep pattern
 * @note   Call this every 100ms from FreeRTOS task
 * @retval Current angle after step
 */
uint16_t Servo_SweepStep(void);

/**
 * @brief  Get current servo angle
 * @retval Current angle (0-180)
 */
uint16_t Servo_GetCurrentAngle(void);

/**
 * @brief  Get current sweep direction
 * @retval SWEEP_RIGHT or SWEEP_LEFT
 */
SweepDirection_t Servo_GetDirection(void);

/**
 * @brief  Reset servo to center position and reset sweep pattern
 * @retval HAL status
 */
HAL_StatusTypeDef Servo_Reset(void);

#endif /* INC_SERVO_CONTROL_H_ */
