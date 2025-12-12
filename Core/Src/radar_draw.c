/*
 * radar_draw.c
 */

#include "radar_draw.h"
#include <math.h>    // Required for cos, sin, fabs
#include <stdbool.h> // Required for bool

// --- GLOBAL FLAGS ---
// This flag is set by the Servo Task in main.c when it hits the edge (0 or 180)
volatile bool g_ReloadGrid = false;

// Store previous line coordinates to erase them efficiently
static int16_t prev_x = RADAR_CENTER_X;
static int16_t prev_y = RADAR_CENTER_Y;

// Add these static variables at the top (under prev_x/prev_y) to track the dot
static int16_t prev_obj_x = -1;
static int16_t prev_obj_y = -1;

// Track the LAST drawn data so we don't redraw the dot unnecessarily
static uint16_t last_drawn_obj_angle = 999;
static float last_drawn_distance = -1.0;

/**
 * @brief Initializes the LCD and draws the static grid
 */
void Radar_InitUI(void) {
    BSP_LCD_Init();
    BSP_LCD_Clear(COLOR_BACKGROUND);
    BSP_LCD_SetBackColor(COLOR_BACKGROUND);
    BSP_LCD_SetFont(&Font12);
    Radar_DrawGrid();
}

/**
 * @brief Draws the static semicircles and labels
 */
void Radar_DrawGrid(void) {
	BSP_LCD_SetTextColor(COLOR_GRID);

	// Draw Concentric Circles
	BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_MAX_RADIUS / 4);     // 50
	BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_MAX_RADIUS / 2);     // 100
	BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_MAX_RADIUS * 3 / 4); // 150
	BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_MAX_RADIUS);         // 200

    // Draw Angled Lines (Dynamic calculation based on Radius)
    // 45 degrees
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y,
                     RADAR_CENTER_X + (int)(RADAR_MAX_RADIUS * cos(45 * 3.14/180)),
                     RADAR_CENTER_Y - (int)(RADAR_MAX_RADIUS * sin(45 * 3.14/180)));
    // 135 degrees
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y,
                     RADAR_CENTER_X + (int)(RADAR_MAX_RADIUS * cos(135 * 3.14/180)),
                     RADAR_CENTER_Y - (int)(RADAR_MAX_RADIUS * sin(135 * 3.14/180)));
    // 90 degrees (Vertical)
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_CENTER_X, RADAR_CENTER_Y - RADAR_MAX_RADIUS);
}

/**
 * @brief Displays Armed/Disarmed status text
 * (Kept for compatibility, though currently unused in the auto-run version)
 */
void Radar_ShowStatus(bool armed) {
    BSP_LCD_SetTextColor(COLOR_TEXT);
    BSP_LCD_SetBackColor(COLOR_BACKGROUND);

    // Clear the status area (top left)
    BSP_LCD_SetTextColor(COLOR_BACKGROUND);
    BSP_LCD_FillRect(0, 0, 240, 20);

    if (armed) {
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_DisplayStringAt(0, 5, (uint8_t*)"SYSTEM ARMED", CENTER_MODE);
    } else {
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_DisplayStringAt(0, 5, (uint8_t*)"SYSTEM DISARMED", CENTER_MODE);
    }
}

// Helper Functions to prevent off-screen coordinates
int16_t ClipX(int16_t x) {
    if (x < 0) return 0;
    if (x > BSP_LCD_GetXSize() - 1) return BSP_LCD_GetXSize() - 1;
    return x;
}

int16_t ClipY(int16_t y) {
    if (y < 0) return 0;
    if (y > BSP_LCD_GetYSize() - 1) return BSP_LCD_GetYSize() - 1;
    return y;
}

void Radar_UpdateSweep(uint16_t sweep_angle, uint16_t obj_angle, float distance_cm) {

    // --- PART 1: BACKGROUND REPAIR (Driven by Flag) ---
    // The Servo Task sets this flag when it hits 0 or 180.
    // This ensures we repair the grid at the exact right moment.
    if (g_ReloadGrid) {
        Radar_DrawGrid();
        g_ReloadGrid = false; // Reset the flag
    }

    // --- PART 2: SWEEP LINE ANIMATION ---

    // 1. Erase Previous Sweep Line (Draw Black)
    BSP_LCD_SetTextColor(COLOR_BACKGROUND);
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y, prev_x, prev_y);

    // 2. Calculate New Coordinates
    float rad_sweep = (float)sweep_angle * 3.14159f / 180.0f;

    int16_t raw_x = RADAR_CENTER_X + (int16_t)(RADAR_MAX_RADIUS * cos(rad_sweep));
    int16_t raw_y = RADAR_CENTER_Y - (int16_t)(RADAR_MAX_RADIUS * sin(rad_sweep));

    int16_t line_x = ClipX(raw_x);
    int16_t line_y = ClipY(raw_y);

    // 3. Draw New Sweep Line (Draw Green)
    BSP_LCD_SetTextColor(COLOR_SWEEP);
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y, line_x, line_y);

    prev_x = line_x;
    prev_y = line_y;


    // --- PART 3: OBJECT DOT (Stabilized) ---

    // We only want to update the dot if the sensor data has CHANGED.
    // This prevents the dot from updating too much or flickering.

    bool dataChanged = (obj_angle != last_drawn_obj_angle) ||
                       (fabs(distance_cm - last_drawn_distance) > 2.0f); // 2cm tolerance

    if (dataChanged) {

        // A. Erase OLD Dot (Only if we are moving it)
        if (prev_obj_x != -1) {
            BSP_LCD_SetTextColor(COLOR_BACKGROUND);
            BSP_LCD_FillCircle(prev_obj_x, prev_obj_y, 4);
            BSP_LCD_FillRect(prev_obj_x + 6, prev_obj_y - 8, 60, 16); // Erase text
            prev_obj_x = -1;
        }

        // B. Draw NEW Dot (If valid)
        if (distance_cm > 2.0f && distance_cm < RADAR_MAX_DIST_CM) {

            float rad_obj = (float)obj_angle * 3.14159f / 180.0f;
            float pixels_per_cm = (float)RADAR_MAX_RADIUS / RADAR_MAX_DIST_CM;
            int16_t obj_radius_px = (int16_t)(distance_cm * pixels_per_cm);

            int16_t raw_obj_x = RADAR_CENTER_X + (int16_t)(obj_radius_px * cos(rad_obj));
            int16_t raw_obj_y = RADAR_CENTER_Y - (int16_t)(obj_radius_px * sin(rad_obj));

            int16_t obj_x = ClipX(raw_obj_x);
            int16_t obj_y = ClipY(raw_obj_y);

            // Draw Dot
            BSP_LCD_SetTextColor(COLOR_OBJECT);
            BSP_LCD_FillCircle(obj_x, obj_y, 4);

            // Draw Text
            char distStr[16];
            sprintf(distStr, "%dcm", (int)distance_cm);

            BSP_LCD_SetFont(&Font12);
            BSP_LCD_SetBackColor(COLOR_BACKGROUND);
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

            // Text positioning logic (Flip to left if near right edge)
            int text_x = obj_x + 6;
            if (text_x > BSP_LCD_GetXSize() - 50) text_x = obj_x - 45;

            BSP_LCD_DisplayStringAt(text_x, obj_y - 8, (uint8_t*)distStr, LEFT_MODE);

            prev_obj_x = obj_x;
            prev_obj_y = obj_y;
        }

        // Update history
        last_drawn_obj_angle = obj_angle;
        last_drawn_distance = distance_cm;
    }
}

/**
 * @brief Draws a loading screen with a progress bar and blinking text.
 * @param progress_percent A value from 0 to 100 representing completion.
 */
void Radar_DrawLoading(uint8_t progress_percent) {
    // 1. Draw Loading Text
    BSP_LCD_SetBackColor(COLOR_BACKGROUND);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_SetFont(&Font16);

    // Simple blinking effect based on progress
    if (progress_percent % 10 < 5) {
        BSP_LCD_DisplayStringAt(0, RADAR_CENTER_Y - 50, (uint8_t*)"INITIALIZING SYSTEM...", CENTER_MODE);
    } else {
        // Draw the string in background color to "erase" it (blinking)
        BSP_LCD_SetTextColor(COLOR_BACKGROUND);
        BSP_LCD_DisplayStringAt(0, RADAR_CENTER_Y - 50, (uint8_t*)"INITIALIZING SYSTEM...", CENTER_MODE);
    }

    // 2. Draw Loading Bar Container
    int bar_width = 200;
    int bar_height = 20;
    int bar_x = (BSP_LCD_GetXSize() - bar_width) / 2;
    int bar_y = RADAR_CENTER_Y + 20;

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawRect(bar_x, bar_y, bar_width, bar_height);

    // 3. Draw Progress Filler
    int fill_width = (bar_width - 4) * progress_percent / 100;
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_FillRect(bar_x + 2, bar_y + 2, fill_width, bar_height - 4);
}
