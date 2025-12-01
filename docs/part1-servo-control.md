# Part 1: Servo Motor Control - Explained

This document provides a detailed explanation of the servo motor control implementation for the STM32 Radar Project.

## 📚 Table of Contents

1. [Overview](#overview)
2. [Hardware Setup](#hardware-setup)
3. [PWM Theory](#pwm-theory)
4. [Timer Configuration](#timer-configuration)
5. [Code Walkthrough](#code-walkthrough)
6. [FreeRTOS Integration](#freertos-integration)
7. [API for GUI Integration](#api-for-gui-integration)
8. [Troubleshooting](#troubleshooting)

---

## Overview

The servo control module provides precise angular positioning for an SG90 servo motor using hardware PWM. The servo sweeps continuously from 0° to 180° and back, simulating a radar scanning pattern.

### Key Specifications

| Parameter | Value |
|-----------|-------|
| PWM Frequency | 50 Hz (20ms period) |
| Pulse Range | 500µs - 2500µs |
| Angular Range | 0° - 180° |
| Step Size | 2° per step |
| Update Rate | 100ms (10 Hz) |
| Full Sweep Time | ~9 seconds |

---

## Hardware Setup

### Why PA5 and NOT PA0?

On the STM32F413H-Discovery board, **PA0 is connected to the USER BUTTON**. If you try to use PA0 for PWM output, the button circuitry will interfere with the signal.

We use **PA5** instead, which:
- Maps to Arduino header **A3**
- Supports TIM2_CH1 alternate function (AF1)
- Is freely available for PWM output

### Wiring

```
┌─────────────────────────────────────────────────────────┐
│                  STM32F413H-Discovery                    │
│                                                          │
│   Arduino Headers:                                       │
│   ┌─────────────────────────────────────────────────┐   │
│   │  A0   A1   A2   A3   A4   A5                    │   │
│   │ (PC0)(PA1)(PA2)(PA5)(PB1)(PC4)                  │   │
│   │             ▲                                    │   │
│   │             │                                    │   │
│   │         SERVO SIGNAL                             │   │
│   └─────────────────────────────────────────────────┘   │
│                                                          │
│   Power:                                                 │
│   ┌─────────────────────────────────────────────────┐   │
│   │  5V ──────────────── Servo VCC (Red)            │   │
│   │  GND ─────────────── Servo GND (Brown)          │   │
│   └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

---

## PWM Theory

### How Servo PWM Works

Hobby servos use a specific PWM signal format:
- **Period:** 20ms (50 Hz)
- **Pulse Width:** Determines the angle

```
        │◄────────── 20ms Period ──────────►│
        │                                    │
        ┌────┐                               ┌────┐
   HIGH │    │                               │    │
        │    │                               │    │
   LOW  ┘    └───────────────────────────────┘    └───────
        │◄──►│
        Pulse Width
        (500µs to 2500µs)
```

### Pulse Width to Angle Mapping

| Pulse Width | Angle |
|-------------|-------|
| 500 µs | 0° |
| 1000 µs | 45° |
| 1500 µs | 90° (center) |
| 2000 µs | 135° |
| 2500 µs | 180° |

### The Formula

```c
pulse_µs = 500 + (angle × 2000 / 180)
```

Or in code:
```c
uint32_t pulse_us = SERVO_MIN_PULSE_US +
    ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
```

---

## Timer Configuration

### Clock Tree

```
                    ┌─────────────────┐
   HSI (16 MHz) ───►│   APB1 Bus      │───► TIM2 Clock (16 MHz)
                    │   (No divider)  │
                    └─────────────────┘
```

### TIM2 Settings

| Register | Value | Purpose |
|----------|-------|---------|
| Prescaler (PSC) | 15 | Divides 16MHz → 1MHz (1µs resolution) |
| Auto-Reload (ARR) | 19999 | Period = 20000 counts = 20ms |
| Capture/Compare 1 (CCR1) | 500-2500 | Pulse width in µs |
| Mode | PWM Mode 1 | Output HIGH when CNT < CCR1 |

### Frequency Calculation

```
Timer Clock = 16 MHz / (PSC + 1) = 16 MHz / 16 = 1 MHz
PWM Period = (ARR + 1) / Timer Clock = 20000 / 1 MHz = 20ms
PWM Frequency = 1 / 20ms = 50 Hz ✓
```

---

## Code Walkthrough

### File: `servo_control.h`

```c
/* Servo pulse width parameters */
#define SERVO_MIN_PULSE_US      500     // 0.5ms = 0°
#define SERVO_MAX_PULSE_US      2500    // 2.5ms = 180°
#define SERVO_CENTER_PULSE_US   1500    // 1.5ms = 90°

#define SERVO_STEP_ANGLE        2       // Degrees per 100ms step
```

### File: `servo_control.c`

#### Initialization

```c
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim) {
    htim_servo = htim;
    
    // Start PWM output on TIM2 Channel 1
    if (HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Move to center position (90°)
    servoState.currentAngle = SERVO_CENTER_ANGLE;
    servoState.direction = SWEEP_RIGHT;
    servoState.initialized = true;
    
    return Servo_SetAngle(SERVO_CENTER_ANGLE);
}
```

#### Setting Angle

```c
HAL_StatusTypeDef Servo_SetAngle(uint16_t angle) {
    // Clamp to valid range
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    
    // Convert angle to pulse width
    uint16_t pulse = Servo_AngleToPulse(angle);
    
    // Update timer compare register
    __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_1, pulse);
    
    servoState.currentAngle = angle;
    return HAL_OK;
}
```

#### Sweep Logic

```c
uint16_t Servo_SweepStep(void) {
    uint16_t nextAngle = servoState.currentAngle;
    
    if (servoState.direction == SWEEP_RIGHT) {
        nextAngle += SERVO_STEP_ANGLE;
        if (nextAngle >= SERVO_MAX_ANGLE) {
            nextAngle = SERVO_MAX_ANGLE;
            servoState.direction = SWEEP_LEFT;  // Reverse at max
        }
    } else {
        nextAngle -= SERVO_STEP_ANGLE;
        if (nextAngle <= SERVO_MIN_ANGLE) {
            nextAngle = SERVO_MIN_ANGLE;
            servoState.direction = SWEEP_RIGHT;  // Reverse at min
        }
    }
    
    Servo_SetAngle(nextAngle);
    return nextAngle;
}
```

---

## FreeRTOS Integration

### Task Definition

The servo is controlled by a dedicated FreeRTOS task:

```c
void StartServoTask(void *argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms
    
    for(;;) {
        // Toggle LED to show task is running
        HAL_GPIO_TogglePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin);
        
        // Step the servo
        Servo_SweepStep();
        
        // Wait exactly 100ms from last wake
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

### Why `vTaskDelayUntil`?

Using `vTaskDelayUntil` instead of `vTaskDelay` ensures **consistent timing**:

```
vTaskDelay(100):           Variable period (100ms + execution time)
├──────100ms──────┤├─exec─┤├──────100ms──────┤
                   ▲ drift accumulates

vTaskDelayUntil:           Fixed period (exactly 100ms)
├──────────────100ms──────────────┤├────────100ms────────┤
                                   ▲ no drift
```

---

## API for LCD Display Integration

### Getting Current State

For the on-board LCD display, you need to read the servo's current state:

```c
// Get the current angle (0-180)
uint16_t angle = Servo_GetCurrentAngle();

// Get sweep direction
SweepDirection_t direction = Servo_GetDirection();
// SWEEP_RIGHT = 0 (moving toward 180°)
// SWEEP_LEFT  = 1 (moving toward 0°)
```

### LCD Display Usage

The STM32F413H-Discovery has a 240x240 pixel LCD. To draw the radar sweep:

```c
// In your LCD display task:
uint16_t angle = Servo_GetCurrentAngle();

// Convert polar to cartesian for LCD
// Center point at (120, 240) - bottom center of screen
#define CENTER_X 120
#define CENTER_Y 240
#define RADIUS   200

// Calculate sweep line endpoint
float radians = (angle - 90) * 3.14159f / 180.0f;  // -90 to +90 from vertical
int16_t end_x = CENTER_X + (int16_t)(RADIUS * cosf(radians));
int16_t end_y = CENTER_Y - (int16_t)(RADIUS * sinf(radians));  // Y inverted for LCD

// Draw line from center to endpoint
BSP_LCD_DrawLine(CENTER_X, CENTER_Y, end_x, end_y);
```

### State Machine Diagram

```
                    ┌───────────────────────────────────┐
                    │                                   │
                    ▼                                   │
              ┌───────────┐                             │
   START ───► │  90° CTR  │                             │
              └─────┬─────┘                             │
                    │                                   │
                    ▼                                   │
              ┌───────────┐    angle >= 180°    ┌───────────┐
              │  SWEEP    │ ──────────────────► │  SWEEP    │
              │  RIGHT    │                     │  LEFT     │
              │  (0→180)  │ ◄────────────────── │  (180→0)  │
              └───────────┘    angle <= 0°      └───────────┘
                    │                                   │
                    └───────────────────────────────────┘
```

---

## Troubleshooting

### Servo Not Moving

| Symptom | Cause | Solution |
|---------|-------|----------|
| No movement, no sound | Wrong pin | Verify servo is on PA5 (Arduino A3) |
| No movement, no sound | PWM not started | Check `HAL_TIM_PWM_Start()` returns `HAL_OK` |
| No movement, buzzing | Wrong frequency | Verify prescaler=15, ARR=19999 |

### Only 90° Range

The default servo pulse range (1000-2000µs) only gives ~90° on most servos. Use extended range:

```c
#define SERVO_MIN_PULSE_US      500   // Was 1000
#define SERVO_MAX_PULSE_US      2500  // Was 2000
```

### Servo Jittering

- Check power supply - servo needs clean 5V
- Add a 100µF capacitor across servo power pins
- Ensure common ground between servo and STM32

### Grinding Noise at Extremes

The pulse range may exceed your servo's physical limits. Reduce the range:

```c
#define SERVO_MIN_PULSE_US      600   // Increase minimum
#define SERVO_MAX_PULSE_US      2400  // Decrease maximum
```

---

## Next Steps

In **Part 2**, we'll add an ultrasonic distance sensor (HC-SR04) to measure distances at each angle.

In **Part 3**, we'll implement the LCD display to show a real-time radar visualization using the on-board 240x240 LCD.

---

*Document Version: 1.0 | Last Updated: December 2025*
