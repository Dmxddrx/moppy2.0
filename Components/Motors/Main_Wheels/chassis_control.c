#include "chassis_control.h"

// Odometry and PID State Variables
float total_distance_cm = 0.0f;
float total_dist_L_cm = 0.0f;
float total_dist_R_cm = 0.0f;

static int16_t last_enc_L = 0;
static int16_t last_enc_R = 0;

static float integral_L = 0.0f, integral_R = 0.0f;
static int16_t last_error_L = 0, last_error_R = 0;

// Moppy 2.0 Physical Constants
const float WHEEL_CIRCUMFERENCE_CM = 21.04867f;
const float TICKS_PER_REVOLUTION   = 943.0f;

// PID Tuning Constants
const float Kp = 2.0f;
const float Ki = 0.05f;
const float Kd = 0.5f;

void Chassis_Init(void)
{
    // Continuously sync the baseline encoders while the robot is disarmed
    // so the PID loop doesn't jerk if the wheels were moved by hand.
    last_enc_L = MainWheels_GetLeftEncoder();
    last_enc_R = MainWheels_GetRightEncoder();
}

void Chassis_Update(void)
{
    // --- READ ENCODERS ---
    int16_t current_enc_L = MainWheels_GetLeftEncoder();
    int16_t current_enc_R = MainWheels_GetRightEncoder();

    int16_t delta_L = current_enc_L - last_enc_L;
    int16_t delta_R = current_enc_R - last_enc_R;

    last_enc_L = current_enc_L;
    last_enc_R = current_enc_R;

    // --- CALCULATE DISTANCE (ODOMETRY) ---
    float dist_L_cm = ((float)delta_L / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;
    float dist_R_cm = ((float)delta_R / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;

    total_dist_L_cm += dist_L_cm;
    total_dist_R_cm += dist_R_cm;
    total_distance_cm += (dist_L_cm + dist_R_cm) / 2.0f;

    // --- PID SPEED CONTROLLER ---
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
