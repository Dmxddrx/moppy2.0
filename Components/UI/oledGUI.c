#include "oledGUI.h"
#include <stdio.h>

/* Global variable to control the current page (0 to 3) */
uint8_t current_gui_page = 0;

/* PAGE 0: M1 & M2 (Main Wheels & Encoders) */
static void render_page_main_wheels(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(10, 0, "M1(L)");
    OLED_Print(78, 0, "M2(R)");
    OLED_DrawLine(0, 10, 128, 10);
    OLED_DrawLine(64, 0, 64, 64);

    uint16_t wl_pwm = MainWheels_GetLeftSpeed();
    int8_t   wl_dir = MainWheels_GetLeftDirection();
    float dist_L = total_dist_L_cm;

    uint16_t wr_pwm = MainWheels_GetRightSpeed();
    int8_t   wr_dir = MainWheels_GetRightDirection();
    float dist_R = total_dist_R_cm;

    char *l_dir_s = (wl_dir == 1) ? "FWD" : ((wl_dir == -1) ? "REV" : "STP");
    char *r_dir_s = (wr_dir == 1) ? "FWD" : ((wr_dir == -1) ? "REV" : "STP");

    /* Left Wheel (M1) */
    snprintf(buf, sizeof(buf), "P:%-3u", wl_pwm);
    OLED_Print(2, 16, buf);
    snprintf(buf, sizeof(buf), "D:%s", l_dir_s);
    OLED_Print(2, 32, buf);
    snprintf(buf, sizeof(buf), "E:%.1fcm", dist_L);
    OLED_Print(2, 48, buf);

    /* Right Wheel (M2) */
    snprintf(buf, sizeof(buf), "P:%-3u", wr_pwm);
    OLED_Print(68, 16, buf);
    snprintf(buf, sizeof(buf), "D:%s", r_dir_s);
    OLED_Print(68, 32, buf);
    snprintf(buf, sizeof(buf), "E:%.1fcm", dist_R);
    OLED_Print(68, 48, buf);

    OLED_Update();
}

/* PAGE 1: Brushes (M3, M4, M5) */
static void render_page_brushes(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(22, 0, "BRUSH STATUS");
    OLED_DrawLine(0, 10, 128, 10);

    // Read real-time hardware PWM values
    uint16_t m3_pwm = SideBrushes_GetLeftSpeed();
    uint16_t m4_pwm = SideBrushes_GetRightSpeed();
    uint16_t m5_pwm = RollerBrush_GetSpeed();

    // Determine basic ON/OFF string states based on PWM
    const char *m3_stat = (m3_pwm > 0) ? "ON " : "OFF";
    const char *m4_stat = (m4_pwm > 0) ? "ON " : "OFF";
    const char *m5_stat = (m5_pwm > 0) ? "ON " : "OFF";

    /* Left Side Brush (M3) */
    snprintf(buf, sizeof(buf), "M3(L): %s | PWM:%-3u", m3_stat, m3_pwm);
    OLED_Print(2, 16, buf);

    /* Right Side Brush (M4) */
    snprintf(buf, sizeof(buf), "M4(R): %s | PWM:%-3u", m4_stat, m4_pwm);
    OLED_Print(2, 32, buf);

    /* Main Roller Brush (M5) */
    snprintf(buf, sizeof(buf), "M5(M): %s | PWM:%-3u", m5_stat, m5_pwm);
    OLED_Print(2, 48, buf);

    OLED_Update();
}

/* MASTER ROUTER */
void render_dashboard_page(void)
{
    switch (current_gui_page) {
        case 0:
        	render_page_main_wheels();
        	break;

        case 1:
        	render_page_brushes();
			break;

		case 2:
			// Placeholder for Page 2
			OLED_Clear();
			OLED_Print(10, 20, "Page 2: Diagnostics");
			OLED_Update();
			break;

		case 3:
			// Placeholder for Page 3
			OLED_Clear();
			OLED_Print(10, 20, "Page 3: Settings");
			OLED_Update();
			break;

        default:
            current_gui_page = 0;
            render_page_main_wheels();
            break;
    }
}
