/**
 ******************************************************************************
 * @file    servo_control.c
 * @brief   Servo motor control implementation for SG90 radar sweep
 * @author  Team Lane 6 - Tommy Terry
 ******************************************************************************
 */

#include "servo_control.h"

/* Private Variables */
static TIM_HandleTypeDef *htim_servo = NULL;
static ServoState_t servoState = {
    .currentAngle = SERVO_CENTER_ANGLE,
    .direction = SWEEP_RIGHT,
    .initialized = false
};

/* Private Function Prototypes */
static uint16_t Servo_AngleToPulse(uint16_t angle);

/**
 * @brief  Initialize servo motor and move to center position
 */
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim) {
    if (htim == NULL) {
        return HAL_ERROR;
    }

    htim_servo = htim;

    // Start PWM on TIM2 Channel 1
    if (HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Move to center position (90 degrees)
    servoState.currentAngle = SERVO_CENTER_ANGLE;
    servoState.direction = SWEEP_RIGHT;
    servoState.initialized = true;

    return Servo_SetAngle(SERVO_CENTER_ANGLE);
}

/**
 * @brief  Set servo to specific angle
 */
HAL_StatusTypeDef Servo_SetAngle(uint16_t angle) {
    if (htim_servo == NULL || !servoState.initialized) {
        return HAL_ERROR;
    }

    // Clamp angle to valid range
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    // Convert angle to pulse width (in timer counts)
    uint16_t pulse = Servo_AngleToPulse(angle);

    // Update PWM duty cycle
    __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_1, pulse);

    servoState.currentAngle = angle;

    return HAL_OK;
}

/**
 * @brief  Perform one step of the radar sweep pattern
 *
 * Sweep Pattern:
 * Start: 90° (center)
 * Phase 1: 90° → 180° (SWEEP_RIGHT)
 * Phase 2: 180° → 0° (SWEEP_LEFT)
 * Phase 3: 0° → 180° (SWEEP_LEFT continues)
 * Repeat from Phase 2
 */
uint16_t Servo_SweepStep(void) {
    if (!servoState.initialized) {
        return servoState.currentAngle;
    }

    uint16_t nextAngle = servoState.currentAngle;

    if (servoState.direction == SWEEP_RIGHT) {
        // Moving right (increasing angle)
        nextAngle += SERVO_STEP_ANGLE;

        if (nextAngle >= SERVO_MAX_ANGLE) {
            nextAngle = SERVO_MAX_ANGLE;
            servoState.direction = SWEEP_LEFT;  // Reverse direction at max
        }
    }
    else {  // SWEEP_LEFT
        // Moving left (decreasing angle)
        if (nextAngle >= SERVO_STEP_ANGLE) {
            nextAngle -= SERVO_STEP_ANGLE;
        } else {
            nextAngle = SERVO_MIN_ANGLE;
        }

        if (nextAngle <= SERVO_MIN_ANGLE) {
            nextAngle = SERVO_MIN_ANGLE;
            servoState.direction = SWEEP_RIGHT;  // Reverse direction at min
        }
    }

    Servo_SetAngle(nextAngle);
    return nextAngle;
}

/**
 * @brief  Get current servo angle
 */
uint16_t Servo_GetCurrentAngle(void) {
    return servoState.currentAngle;
}

/**
 * @brief  Get current sweep direction
 */
SweepDirection_t Servo_GetDirection(void) {
    return servoState.direction;
}

/**
 * @brief  Reset servo to center position
 */
HAL_StatusTypeDef Servo_Reset(void) {
    servoState.currentAngle = SERVO_CENTER_ANGLE;
    servoState.direction = SWEEP_RIGHT;

    return Servo_SetAngle(SERVO_CENTER_ANGLE);
}

/**
 * @brief  Convert angle (0-180) to PWM pulse width in timer counts
 *
 * @note   Assumes TIM2 configured for 50Hz (20ms period):
 *         - For APB1 timer clock of 16MHz (HSI) with no prescaler on APB1:
 *           Prescaler = 16-1 = 15 (1MHz timer clock, 1 count = 1µs)
 *           ARR = 20000-1 = 19999 (20ms period at 1MHz)
 *         - Pulse width: 1000 counts = 1ms (0°), 2000 counts = 2ms (180°)
 *
 * @param  angle: angle in degrees (0-180)
 * @retval pulse width in timer counts
 */
static uint16_t Servo_AngleToPulse(uint16_t angle) {
    // Linear mapping: angle (0-180) -> pulse (1000-2000 us)
    // pulse_us = 1000 + (angle * 1000 / 180)
    // pulse_counts = pulse_us (since timer runs at 1MHz, 1 count = 1us)

    uint32_t pulse_us = SERVO_MIN_PULSE_US +
                       ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;

    return (uint16_t)pulse_us;
}
