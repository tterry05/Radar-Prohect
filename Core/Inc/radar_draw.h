/* radar_draw.h */

#ifndef RADAR_DRAW_H
#define RADAR_DRAW_H

#include "stm32f413h_discovery_lcd.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

// Dimensions
#define RADAR_CENTER_X      120
#define RADAR_CENTER_Y      220
#define RADAR_MAX_RADIUS    200

// --- CHANGE THIS VALUE ---
// 150cm (1.5 meters) is much better for desk testing than 400cm.
// Objects at 1.5m will now hit the very edge of the screen.
#define RADAR_MAX_DIST_CM   150.0f

// Colors
#define COLOR_BACKGROUND    LCD_COLOR_BLACK
#define COLOR_GRID          LCD_COLOR_DARKGREEN
#define COLOR_SWEEP         LCD_COLOR_GREEN
#define COLOR_OBJECT        LCD_COLOR_RED
#define COLOR_TEXT          LCD_COLOR_WHITE

// Shared Flags
extern volatile bool g_ReloadGrid;

// Prototypes
void Radar_InitUI(void);
void Radar_DrawGrid(void);
void Radar_DrawLoading(uint8_t progress_percent);
void Radar_UpdateSweep(uint16_t sweep_angle, uint16_t obj_angle, float distance_cm);

#endif
