/*
 * radar_draw.h
 *
 * Radar UI drawing library for STM32F413H-Discovery
 */

#ifndef RADAR_DRAW_H
#define RADAR_DRAW_H

#include "stm32f413h_discovery_lcd.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

// Radar Configuration
#define RADAR_MAX_DIST_CM    100.0f  // Max distance shown on screen
#define RADAR_CENTER_X       120     // Center of the screen (Width 240 / 2)
#define RADAR_CENTER_Y       239     // Bottom of the screen (Height 240)
#define RADAR_MAX_RADIUS     220     // Length of the sweep line in pixels

// Colors
#define COLOR_GRID           LCD_COLOR_DARKGREEN
#define COLOR_SWEEP          LCD_COLOR_GREEN
#define COLOR_OBJECT         LCD_COLOR_RED
#define COLOR_BACKGROUND     LCD_COLOR_BLACK
#define COLOR_TEXT           LCD_COLOR_WHITE

// Public Functions
void Radar_InitUI(void);
void Radar_DrawGrid(void);
void Radar_UpdateSweep(uint16_t sweep_angle, uint16_t obj_angle, float distance_cm);
void Radar_ShowStatus(bool armed);

#endif /* RADAR_DRAW_H */
