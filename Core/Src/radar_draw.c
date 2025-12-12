/*
 * radar_draw.c
 */

#include "radar_draw.h"

// Store previous line coordinates to erase them efficiently
static uint16_t prev_x = RADAR_CENTER_X;
static uint16_t prev_y = RADAR_CENTER_Y;

// Add these static variables at the top (under prev_x/prev_y) to track the dot
static int16_t prev_obj_x = -1;
static int16_t prev_obj_y = -1;

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

    // Draw concentric semi-circles (Range Rings)
    BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, 50);
    BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, 100);
    BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, 150);
    BSP_LCD_DrawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, 200);

    // Draw angled lines (every 45 degrees)
    // 45 degrees
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y,
                     RADAR_CENTER_X + (int)(200 * cos(45 * 3.14/180)),
                     RADAR_CENTER_Y - (int)(200 * sin(45 * 3.14/180)));
    // 135 degrees
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y,
                     RADAR_CENTER_X + (int)(200 * cos(135 * 3.14/180)),
                     RADAR_CENTER_Y - (int)(200 * sin(135 * 3.14/180)));
    // 90 degrees (Vertical)
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_CENTER_X, RADAR_CENTER_Y - 200);
}

/**
 * @brief Displays Armed/Disarmed status text
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

void Radar_UpdateSweep(uint16_t sweep_angle, uint16_t obj_angle, float distance_cm) {

    // --- PART 1: UPDATE THE GREEN SWEEP LINE (Smooth Animation) ---

    // 1. Erase Previous Line
    BSP_LCD_SetTextColor(COLOR_BACKGROUND);
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y, prev_x, prev_y);

    // 2. Calculate New Line Coordinates
    float rad_sweep = (float)sweep_angle * 3.14159f / 180.0f;
    int16_t line_x = RADAR_CENTER_X + (int16_t)(RADAR_MAX_RADIUS * cos(rad_sweep));
    int16_t line_y = RADAR_CENTER_Y - (int16_t)(RADAR_MAX_RADIUS * sin(rad_sweep));

    // 3. Draw New Sweep Line
    BSP_LCD_SetTextColor(COLOR_SWEEP);
    BSP_LCD_DrawLine(RADAR_CENTER_X, RADAR_CENTER_Y, line_x, line_y);

    // Save line coordinates
    prev_x = line_x;
    prev_y = line_y;


    // --- PART 2: UPDATE THE OBJECT BLIP & TEXT ---

    // 1. ERASE PREVIOUS Object and Text
    if (prev_obj_x != -1) {
        BSP_LCD_SetTextColor(COLOR_BACKGROUND);

        // Erase the dot
        BSP_LCD_FillCircle(prev_obj_x, prev_obj_y, 4);

        // Erase the text box next to it
        // We assume the text was drawn to the right and slightly up
        BSP_LCD_FillRect(prev_obj_x + 6, prev_obj_y - 8, 50, 16);

        prev_obj_x = -1; // Reset
    }

    // 2. DRAW NEW Object if within valid range
    if (distance_cm > 2.0f && distance_cm < RADAR_MAX_DIST_CM) {

        float rad_obj = (float)obj_angle * 3.14159f / 180.0f;

        // Map distance to pixels (Scale automatically adjusts to RADAR_MAX_DIST_CM)
        float pixels_per_cm = (float)RADAR_MAX_RADIUS / RADAR_MAX_DIST_CM;
        int16_t obj_radius_px = (int16_t)(distance_cm * pixels_per_cm);

        int16_t obj_x = RADAR_CENTER_X + (int16_t)(obj_radius_px * cos(rad_obj));
        int16_t obj_y = RADAR_CENTER_Y - (int16_t)(obj_radius_px * sin(rad_obj));

        // Draw Red Dot
        BSP_LCD_SetTextColor(COLOR_OBJECT);
        BSP_LCD_FillCircle(obj_x, obj_y, 4);

        // Draw Distance Text
        char distStr[16];
        sprintf(distStr, "%dcm", (int)distance_cm);

        BSP_LCD_SetFont(&Font12);
        BSP_LCD_SetBackColor(COLOR_BACKGROUND);
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE); // White text is easier to read

        // Draw text 6 pixels to the right, 8 pixels up to center it vertically
        BSP_LCD_DisplayStringAt(obj_x + 6, obj_y - 8, (uint8_t*)distStr, LEFT_MODE);

        // Store coordinates for next erase
        prev_obj_x = obj_x;
        prev_obj_y = obj_y;
    }
}

/**
 * @brief Draws a loading screen with a progress bar and blinking text.
 * @param progress_percent A value from 0 to 100 representing completion.
 */
void Radar_DrawLoading(uint8_t progress_percent) {
    // 1. Set Background (Only clear if needed, otherwise it flickers)
    // For this simple implementation, we assume the background was cleared once on startup
    // or we just overwrite the specific area.

    // 2. Draw Loading Text
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

    // 3. Draw Loading Bar Container
    int bar_width = 200;
    int bar_height = 20;
    int bar_x = (BSP_LCD_GetXSize() - bar_width) / 2;
    int bar_y = RADAR_CENTER_Y + 20;

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawRect(bar_x, bar_y, bar_width, bar_height);

    // 4. Draw Progress Filler
    int fill_width = (bar_width - 4) * progress_percent / 100;
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_FillRect(bar_x + 2, bar_y + 2, fill_width, bar_height - 4);
}
