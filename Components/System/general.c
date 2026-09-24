#include "general.h"

// Define some dummy variables so oledGUI.c compiles without errors
uint16_t s_lidar_raw[4] = {0, 0, 0, 0};
float s_current_yaw = 0.0f;
int s_disp_l_pwm = 0;
int s_disp_r_pwm = 0;
typedef enum { CMD_STOP, CMD_DRIVE_FORWARD } CoverageCmd;
CoverageCmd s_current_cmd = CMD_STOP;

// Odometry and PID State Variables
float total_distance_cm = 0.0f;
float total_dist_L_cm = 0.0f;
float total_dist_R_cm = 0.0f;

static int16_t last_enc_L = 0;
static int16_t last_enc_R = 0;

static float integral_L = 0.0f, integral_R = 0.0f;
static int16_t last_error_L = 0, last_error_R = 0;

// Moppy 2.0 Physical Constants
const float WHEEL_CIRCUMFERENCE_CM = 21.04867f; // Example: 6.5cm diameter * 3.14159
const float TICKS_PER_REVOLUTION   = 943.0f; // Example: Motor gear ratio * Encoder pulses

// PID Tuning Constants
const float Kp = 2.0f;
const float Ki = 0.05f;
const float Kd = 0.5f;

void General_Init(void)
{
    MainWheels_Init();
    SideBrushes_Init();
    RollerBrush_Init();
    VacuumBlower_Init();

    OLED_Init(&hi2c2);

    OLED_Clear();
    OLED_Print(10, 20, "MOPPY 2.0 BOOTING");
    OLED_Update();
    for (int i = 0; i < 10; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
}

void System_RunAllForwardMax(void)
{
    MainWheels_Forward(999);
    SideBrushes_Start(999);
    RollerBrush_Start(999);
    VacuumBlower_SetSpeed(999);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MSD_RPM_Pin)
    {
        blower_pulse_count++;
    }
}

void General_Run(void)
{
    HAL_IWDG_Refresh(&hiwdg);

    static uint32_t last_ramp_time = 0;
    static uint16_t blower_speed = 0;
    static uint32_t last_display_update = 0;
    static uint32_t last_pid_time = 0;

    uint32_t current_time = HAL_GetTick();

    // 1. UPDATE THE DISPLAY (10 FPS)
    if ((current_time - last_display_update) >= 100) {
        last_display_update = current_time;
        render_dashboard_page();
    }

    // 2. MOTOR LOGIC
    if (current_time < 3000) {
        // Phase 1: ARMING
        blower_speed = 0;
        VacuumBlower_SetSpeed(blower_speed);

        MainWheels_Stop();
        SideBrushes_Stop();
        RollerBrush_Stop();
    }
    else {
        // Phase 2: DYNAMIC RAMP-UP
        if ((current_time - last_ramp_time) >= 50) {
            last_ramp_time = current_time;
            if (blower_speed < 800) {
                blower_speed += 10;
                // VacuumBlower_SetSpeed(blower_speed);
                VacuumBlower_Stop(); // Temporarily stopped for testing
            }
        }

        // Phase 3: PID & DISTANCE CONTROL LOOP (50ms)
        if ((current_time - last_pid_time) >= 50) {
            last_pid_time = current_time;

            int16_t current_enc_L = MainWheels_GetLeftEncoder();
            int16_t current_enc_R = MainWheels_GetRightEncoder();

            int16_t delta_L = current_enc_L - last_enc_L;
            int16_t delta_R = current_enc_R - last_enc_R;

            last_enc_L = current_enc_L;
            last_enc_R = current_enc_R;

            float dist_L_cm = ((float)delta_L / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;
            float dist_R_cm = ((float)delta_R / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;

            total_dist_L_cm += dist_L_cm;
			total_dist_R_cm += dist_R_cm;

            total_distance_cm += (dist_L_cm + dist_R_cm) / 2.0f;

            // Set Target Speed (Encoder ticks per 50ms)
            int16_t target_ticks_per_loop = 40;

            int16_t error_L = target_ticks_per_loop - delta_L;
            int16_t error_R = target_ticks_per_loop - delta_R;

            integral_L += error_L;
            integral_R += error_R;
            if (integral_L > 1000) integral_L = 1000; else if (integral_L < -1000) integral_L = -1000;
            if (integral_R > 1000) integral_R = 1000; else if (integral_R < -1000) integral_R = -1000;

            int16_t deriv_L = error_L - last_error_L;
            int16_t deriv_R = error_R - last_error_R;
            last_error_L = error_L;
            last_error_R = error_R;

            int16_t pwm_L = (int16_t)((Kp * error_L) + (Ki * integral_L) + (Kd * deriv_L));
            int16_t pwm_R = (int16_t)((Kp * error_R) + (Ki * integral_R) + (Kd * deriv_R));

            if (pwm_L > 999) pwm_L = 999; else if (pwm_L < 0) pwm_L = 0;
            if (pwm_R > 999) pwm_R = 999; else if (pwm_R < 0) pwm_R = 0;

            MainWheels_SetLeftSpeed(pwm_L);
            MainWheels_SetRightSpeed(pwm_R);
        }
    }
}
