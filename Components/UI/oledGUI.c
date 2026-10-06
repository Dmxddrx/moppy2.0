#include "oledGUI.h"
#include "chassis_control.h"
#include <stdio.h>

/* Change this value (0, 1, or 2) to manually switch pages right now. */
uint8_t current_gui_page = 1;

/* ═══════════════════════════════════════════════════════════════ */
/* PAGE 0: MAIN WHEELS (SPLIT SCREEN UI)                           */
/* ═══════════════════════════════════════════════════════════════ */
static void render_page_main_wheels(void)
{
    char buf[32];
    extern float total_distance_cm;
    extern float total_dist_L_cm;
    extern float total_dist_R_cm;

    OLED_Clear();

    /* Fetch Wheel Data */
    uint16_t wl_pwm = MainWheels_GetLeftSpeed();
    int8_t   wl_dir = MainWheels_GetLeftDirection();
    int16_t  enc_L  = MainWheels_GetLeftEncoder();

    uint16_t wr_pwm = MainWheels_GetRightSpeed();
    int8_t   wr_dir = MainWheels_GetRightDirection();
    int16_t  enc_R  = MainWheels_GetRightEncoder();

    /* Convert directions to strings */
    char *l_dir_s = (wl_dir == 1) ? "FWD" : ((wl_dir == -1) ? "REV" : "STP");
    char *r_dir_s = (wr_dir == 1) ? "FWD" : ((wr_dir == -1) ? "REV" : "STP");

    /* 1. Draw UI Framework */
    OLED_Print(10, 0, "L-WHL");               // Left Title
    OLED_Print(78, 0, "R-WHL");               // Right Title
    OLED_DrawLine(0, 10, 128, 10);            // Top Header Line
    OLED_DrawLine(64, 0, 64, 46);             // Center Vertical Divider
    OLED_DrawLine(0, 46, 128, 46);            // Bottom Footer Line

    /* 2. Left Wheel Data Block (Y spacing = 8 pixels) */
    snprintf(buf, sizeof(buf), "P:%-3u", wl_pwm);
    OLED_Print(2, 13, buf);

    snprintf(buf, sizeof(buf), "D:%s", l_dir_s);
    OLED_Print(2, 21, buf);

    snprintf(buf, sizeof(buf), "E:%-5d", enc_L);
    OLED_Print(2, 29, buf);

    snprintf(buf, sizeof(buf), "%.1fcm", total_dist_L_cm);
    OLED_Print(2, 37, buf);

    /* 3. Right Wheel Data Block (Y spacing = 8 pixels) */
    snprintf(buf, sizeof(buf), "P:%-3u", wr_pwm);
    OLED_Print(68, 13, buf);

    snprintf(buf, sizeof(buf), "D:%s", r_dir_s);
    OLED_Print(68, 21, buf);

    snprintf(buf, sizeof(buf), "E:%-5d", enc_R);
    OLED_Print(68, 29, buf);

    snprintf(buf, sizeof(buf), "%.1fcm", total_dist_R_cm);
    OLED_Print(68, 37, buf);

    /* 4. Global Distance Footer */
    snprintf(buf, sizeof(buf), "TOT DIST: %.1f cm", total_distance_cm);
    OLED_Print(10, 52, buf); // Centered at the bottom

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* PAGE 1: VACUUM BLOWER (PWM, REAL-TIME RPM)                      */
/* ═══════════════════════════════════════════════════════════════ */
static void render_page_vacuum(void)
{
    char buf[32];
    OLED_Clear();

    OLED_Print(0, 0, "-- VACUUM BLOWER --");
    OLED_DrawLine(0, 9, 128, 9);

    /* Fetch Blower Data */
    uint16_t vac_pwm = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
    uint32_t vac_rpm = VacuumBlower_GetRPM();

    snprintf(buf, sizeof(buf), "TARGET PWM:");
    OLED_Print(0, 20, buf);
    snprintf(buf, sizeof(buf), "-> %-3u", vac_pwm);
    OLED_Print(0, 30, buf);

    snprintf(buf, sizeof(buf), "TACHOMETER RPM:");
    OLED_Print(0, 45, buf);
    snprintf(buf, sizeof(buf), "-> %-6lu RPM", vac_rpm);
    OLED_Print(0, 55, buf);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* PAGE 2: CLEANING BRUSHES (SIDE BRUSHES & ROLLER)                */
/* ═══════════════════════════════════════════════════════════════ */
static void render_page_brushes(void)
{
    char buf[32];
    OLED_Clear();

    OLED_Print(0, 0, "--- SWEEP BRUSHES ---");
    OLED_DrawLine(0, 9, 128, 9);

    /* Fetch Brush Data */
    uint16_t sb_l = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
    uint16_t sb_r = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
    uint16_t rol_pwm = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);

    /* Side Brushes Block */
    OLED_Print(0, 15, "SIDE BRUSHES:");
    snprintf(buf, sizeof(buf), "L: %-3u", sb_l);
    OLED_Print(10, 25, buf);
    snprintf(buf, sizeof(buf), "R: %-3u", sb_r);
    OLED_Print(64, 25, buf);

    /* Main Roller Block */
    OLED_Print(0, 42, "MAIN ROLLER:");
    snprintf(buf, sizeof(buf), "PWM: %-3u", rol_pwm);
    OLED_Print(10, 52, buf);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* PAGE 3: IMU & COMPASS CALIBRATION                               */
/* ═══════════════════════════════════════════════════════════════ */
static void render_page_sensors(void)
{
    char buf[32];
    extern HMC5883L_RawData current_mag;       //[cite: 12]
    extern MPU6050_PhysData current_mpu_phys;  //[cite: 14]
    extern float current_heading;

    OLED_Clear();

    /* 1. Draw UI Framework */
    OLED_Print(0, 0, " GYRO(deg/s)");
    OLED_Print(70, 0, "MAG(raw)");
    OLED_DrawLine(0, 10, 128, 10);
    OLED_DrawLine(64, 0, 64, 46);
    OLED_DrawLine(0, 46, 128, 46);

    /* 2. Left Block: MPU6050 Gyroscope Data */
    snprintf(buf, sizeof(buf), "X:%5.1f", current_mpu_phys.gx);
    OLED_Print(2, 14, buf);

    snprintf(buf, sizeof(buf), "Y:%5.1f", current_mpu_phys.gy);
    OLED_Print(2, 24, buf);

    snprintf(buf, sizeof(buf), "Z:%5.1f", current_mpu_phys.gz);
    OLED_Print(2, 34, buf);

    /* 3. Right Block: HMC5883L Magnetometer Data */
    snprintf(buf, sizeof(buf), "X:%-5d", current_mag.mx);
    OLED_Print(68, 14, buf);

    snprintf(buf, sizeof(buf), "Y:%-5d", current_mag.my);
    OLED_Print(68, 24, buf);

    snprintf(buf, sizeof(buf), "Z:%-5d", current_mag.mz);
    OLED_Print(68, 34, buf);

    /* 4. Global Heading Footer */
    snprintf(buf, sizeof(buf), "HEADING: %3.0f deg", current_heading);
    OLED_Print(10, 52, buf);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* MASTER ROUTER: Displays the page matching 'current_gui_page'    */
/* ═══════════════════════════════════════════════════════════════ */
void render_dashboard_page(void)
{
    if (current_gui_page == 0) {
        render_page_main_wheels();
    }
    else if (current_gui_page == 1) {
        render_page_vacuum();
    }
    else if (current_gui_page == 2) {
        render_page_brushes();
    }
    else if (current_gui_page == 3) { // <--- NEW: Route to Sensor Page
        render_page_sensors();
    }
    else {
        current_gui_page = 0;
    }
}
