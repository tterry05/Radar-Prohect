# 🛰️ STM32 Radar Project

A radar scanning system built on the **STM32F413H-Discovery** board using a servo motor for 180° sweeping motion. This project is designed for educational purposes and demonstrates embedded systems development with FreeRTOS.

![STM32F413H-Discovery](https://www.st.com/bin/ecommerce/api/image.PF264808.en.feature-description-include-personalized-no-498x249.jpg)

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Software Architecture](#software-architecture)
- [Building & Flashing](#building--flashing)
- [API Reference](#api-reference)
- [Project Parts](#project-parts)
- [Team](#team)

## Overview

This radar project implements a servo-controlled scanning mechanism that sweeps 180 degrees in a continuous back-and-forth motion. The system is built on FreeRTOS for real-time task management.

### Features

- ✅ 180° servo sweep using hardware PWM (TIM2)
- ✅ FreeRTOS task-based architecture
- ✅ Configurable sweep speed and step size
- ✅ Real-time angle tracking
- 🔄 *Coming Soon:* Distance sensor integration
- 🔄 *Coming Soon:* On-board LCD radar display

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **Board** | STM32F413H-Discovery |
| **Servo** | SG90 or compatible (5V, PWM) |
| **Power** | USB or external 5V for servo |

### Wiring Diagram

```
STM32F413H-Discovery          SG90 Servo
├── Arduino A3 (PA5) ──────── Signal (Orange/Yellow)
├── 5V ────────────────────── VCC (Red)
└── GND ───────────────────── GND (Brown/Black)
```

> ⚠️ **Important:** PA0 is the USER BUTTON on this board. Do NOT use it for PWM output!

## Pin Configuration

| Function | Pin | Arduino Header | Timer |
|----------|-----|----------------|-------|
| Servo PWM | PA5 | A3 | TIM2_CH1 |
| User LED (Green) | PC5 | - | GPIO |
| User LED (Red) | PE3 | - | GPIO |
| User Button | PA0 | - | GPIO (EXTI) |

## Software Architecture

```
radar-project/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Main header with pin definitions
│   │   ├── servo_control.h     # Servo API declarations
│   │   └── FreeRTOSConfig.h    # RTOS configuration
│   └── Src/
│       ├── main.c              # Application entry point
│       ├── servo_control.c     # Servo motor driver
│       └── freertos.c          # FreeRTOS hooks
├── Drivers/                    # STM32 HAL drivers
├── Middlewares/                # FreeRTOS kernel
└── docs/                       # Documentation
    └── part1-servo-control.md  # Part 1 detailed explanation
```

### Task Overview

| Task | Priority | Stack | Period | Description |
|------|----------|-------|--------|-------------|
| ServoTask | Normal | 512B | 100ms | Controls servo sweep |

## Building & Flashing

### Prerequisites

- STM32CubeIDE 1.19.0 or later
- ST-Link drivers installed

### Build Steps

1. Open STM32CubeIDE
2. Import project: `File → Import → Existing Projects into Workspace`
3. Select the `radar-project` folder
4. Build: `Project → Build Project` (or Ctrl+B)
5. Flash: `Run → Debug` (or F11)

## API Reference

### Servo Control API

```c
#include "servo_control.h"

// Initialize servo and move to center (90°)
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim);

// Set servo to specific angle (0-180)
HAL_StatusTypeDef Servo_SetAngle(uint16_t angle);

// Perform one sweep step (call every 100ms)
uint16_t Servo_SweepStep(void);

// Get current angle
uint16_t Servo_GetCurrentAngle(void);

// Get sweep direction
SweepDirection_t Servo_GetDirection(void);

// Reset to center position
HAL_StatusTypeDef Servo_Reset(void);
```

### Getting Servo Data (for LCD Display)

To read the current servo state for the LCD radar display:

```c
// In your display task:
uint16_t currentAngle = Servo_GetCurrentAngle();      // 0-180
SweepDirection_t dir = Servo_GetDirection();          // SWEEP_LEFT or SWEEP_RIGHT

// Use angle to draw radar sweep line on LCD
// Convert angle to x,y coordinates for the 240x240 display
```

### Configuration Parameters

Edit `servo_control.h` to customize:

```c
#define SERVO_MIN_PULSE_US      500     // 0° pulse width (µs)
#define SERVO_MAX_PULSE_US      2500    // 180° pulse width (µs)
#define SERVO_STEP_ANGLE        2       // Degrees per step
```

## Project Parts

| Part | Description | Status |
|------|-------------|--------|
| [Part 1: Servo Control](docs/part1-servo-control.md) | PWM setup, FreeRTOS task, sweep logic | ✅ Complete |
| Part 2: Distance Sensor | HC-SR04 ultrasonic integration | 🔄 Planned |
| Part 3: LCD Display | On-board 240x240 LCD radar visualization | 🔄 Planned |

## Team

**Team Lane 6**
- Tommy Terry

## License

This project is for educational purposes. Feel free to use and modify for learning!

---

*Built with ❤️ using STM32CubeIDE and FreeRTOS*
