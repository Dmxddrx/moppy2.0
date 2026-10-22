#include "chassis_control.h"

// Global Odometry Variables[cite: 16]
float total_distance_cm = 0.0f;
float total_dist_L_cm   = 0.0f;
float total_dist_R_cm   = 0.0f;

// Trajectory and Setpoint Tracking
static int16_t final_target_ticks_L = 0;
static int16_t final_target_ticks_R = 0;
static float current_target_ticks_L = 0.0f;
static float current_target_ticks_R = 0.0f;

// Encoder Tracking State[cite: 16]
static int16_t last_enc_L = 0;
static int16_t last_enc_R = 0;

// PID Controller State[cite: 16]
static float integral_L = 0.0f, integral_R = 0.0f;
static int16_t last_error_L = 0, last_error_R = 0;

// Moppy 2.0 Physical Constants[cite: 16]
const float WHEEL_CIRCUMFERENCE_CM = 21.04867f;
const float TICKS_PER_REVOLUTION   = 943.0f;

// PID & Trajectory Constants
const float Kp = 2.0f; //[cite: 16]
const float Ki = 0.05f; //[cite: 16]
const float Kd = 0.5f; //[cite: 16]
const float ACCEL_STEP = 2.0f; // Max target change per 20ms loop (Higher = steeper acceleration)

void Chassis_Init(void)
{
    // 1. Synchronize initial encoder baselines[cite: 16]
    last_enc_L = MainWheels_GetLeftEncoder();
    last_enc_R = MainWheels_GetRightEncoder();

    // 2. Reset PID controller states[cite: 16]
    integral_L = 0.0f;
    integral_R = 0.0f;
    last_error_L = 0;
    last_error_R = 0;

    // 3. Reset setpoints and trajectory profiles
    final_target_ticks_L = 0;
    final_target_ticks_R = 0;
    current_target_ticks_L = 0.0f;
    current_target_ticks_R = 0.0f;
}

void Chassis_SetTargetSpeed(int16_t left_ticks_per_loop, int16_t right_ticks_per_loop)
{
    // Register the final desired speed. The Update loop will smoothly ramp towards this.[cite: 16]
    final_target_ticks_L = left_ticks_per_loop;
    final_target_ticks_R = right_ticks_per_loop;
}

void Chassis_ResetOdometry(void)
{
    total_distance_cm = 0.0f; //[cite: 16]
    total_dist_L_cm   = 0.0f; //[cite: 16]
    total_dist_R_cm   = 0.0f; //[cite: 16]
}

void Chassis_Update(void)
{
    // --- READ ENCODERS ---[cite: 16]
    int16_t current_enc_L = MainWheels_GetLeftEncoder();
    int16_t current_enc_R = MainWheels_GetRightEncoder();

    // 16-bit signed subtraction handles counter overflow/underflow automatically[cite: 16]
    int16_t delta_L = current_enc_L - last_enc_L;
    int16_t delta_R = current_enc_R - last_enc_R;

    last_enc_L = current_enc_L; //[cite: 16]
    last_enc_R = current_enc_R; //[cite: 16]

    // --- CALCULATE DISTANCE (ODOMETRY) ---[cite: 16]
    float dist_L_cm = ((float)delta_L / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;
    float dist_R_cm = ((float)delta_R / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;

    total_dist_L_cm   += dist_L_cm; //[cite: 16]
    total_dist_R_cm   += dist_R_cm; //[cite: 16]
    total_distance_cm += (dist_L_cm + dist_R_cm) / 2.0f; //[cite: 16]

    // --- TRAJECTORY GENERATOR (SOFT START / SOFT STOP) ---
    // Smoothly ramp Left target towards final target
    if (current_target_ticks_L < final_target_ticks_L) {
        current_target_ticks_L += ACCEL_STEP;
        if (current_target_ticks_L > final_target_ticks_L) current_target_ticks_L = final_target_ticks_L;
    } else if (current_target_ticks_L > final_target_ticks_L) {
        current_target_ticks_L -= ACCEL_STEP;
        if (current_target_ticks_L < final_target_ticks_L) current_target_ticks_L = final_target_ticks_L;
    }

    // Smoothly ramp Right target towards final target
    if (current_target_ticks_R < final_target_ticks_R) {
        current_target_ticks_R += ACCEL_STEP;
        if (current_target_ticks_R > final_target_ticks_R) current_target_ticks_R = final_target_ticks_R;
    } else if (current_target_ticks_R > final_target_ticks_R) {
        current_target_ticks_R -= ACCEL_STEP;
        if (current_target_ticks_R < final_target_ticks_R) current_target_ticks_R = final_target_ticks_R;
    }

    // --- PID SPEED CONTROLLER ---
    // Calculate error using the smoothed current target instead of the raw final target
    int16_t error_L = (int16_t)current_target_ticks_L - delta_L;
    int16_t error_R = (int16_t)current_target_ticks_R - delta_R;

    // Accumulate integral with anti-windup clamping[cite: 16]
    integral_L += error_L;
    integral_R += error_R;
    if (integral_L > 1000.0f)  integral_L = 1000.0f;  else if (integral_L < -1000.0f) integral_L = -1000.0f;
    if (integral_R > 1000.0f)  integral_R = 1000.0f;  else if (integral_R < -1000.0f) integral_R = -1000.0f;

    // Derivative calculation[cite: 16]
    int16_t deriv_L = error_L - last_error_L;
    int16_t deriv_R = error_R - last_error_R;
    last_error_L = error_L;
    last_error_R = error_R;

    // Compute PID output[cite: 16]
    int16_t pwm_L = (int16_t)((Kp * error_L) + (Ki * integral_L) + (Kd * deriv_L));
    int16_t pwm_R = (int16_t)((Kp * error_R) + (Ki * integral_R) + (Kd * deriv_R));

    // Clamp output to full signed motor driver range (-999 to +999)[cite: 16]
    if (pwm_L > 999)  pwm_L = 999;  else if (pwm_L < -999) pwm_L = -999;
    if (pwm_R > 999)  pwm_R = 999;  else if (pwm_R < -999) pwm_R = -999;

    // Apply motor speeds[cite: 16]
    MainWheels_SetLeftSpeed(pwm_L);
    MainWheels_SetRightSpeed(pwm_R);
}
