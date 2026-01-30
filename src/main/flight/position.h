#pragma once

#include "common/time.h"

#define TASK_ALTITUDE_RATE_HZ 100

// --- 1D EKF for vertical axis:
// x = [ z (m), v (m/s), ba (m/s^2), bb (m) ]
// where:
//  - ba is accelerometer bias in world-Z acceleration (random walk)
//  - bb is barometer altitude bias/offset (random walk)
typedef struct positionAltEKF_s {
    float z;        // altitude (m)
    float v;        // vertical velocity (m/s)
    float ba;       // accel bias (m/s^2)
    float bb;       // baro bias/offset (m)

    float P[4][4];  // covariance

    // process/measurement noises
    float Qz;       // (m^2)    — usually 0
    float Qv;       // (m^2/s^2)
    float Qba;      // (m^2/s^4)
    float Qbb;      // (m^2/s)  - baro bias RW spectral density
    float R;        // (m^2)    — baro variance

    float R_eff;    // (m^2)    - effective baro variance after adaptation

    // debug taps
    float aWorldZ;      // input accel in world-Z (m/s^2)
    float innovZ;       // measurement innovation (m)
    float r33;          // cos(theta)*cos(phi)
    uint8_t gateRejected; // 0/1
} positionAltEKF_t;

typedef struct positionConfig_s {
    uint8_t  altitude_source;
    uint8_t  altitude_prefer_baro;
    uint16_t altitude_lpf;           // (value / 100) Hz
    uint16_t altitude_d_lpf;         // (value / 100) Hz

    // --- EKF tunables (scaled for CLI/EEPROM) ---
    uint16_t ekf_qv_centi;           // Qv = (ekf_qv_centi/100)^2  [ (m/s^2)^2 ]
    uint16_t ekf_qba_centi;          // Qba = (ekf_qba_centi/100)^2 [ (m/s^2)^2 per sec ]
    uint16_t ekf_qbb_centi;          // Qbb = (ekf_qbb_centi/100)^2 [ (m)^2 per sec ]
    uint16_t ekf_r_centi;            // R = (ekf_r_centi/100)^2     [ m^2 ]
    uint8_t  ekf_gate_sigma_x10;     // gate sigma = /10 (e.g. 30 -> 3.0σ)
    uint8_t  ekf_enable_adapt_r;     // 0/1 (optional, future use)

    // --- Altitude hold tuning (BARO_MODE) ---
    uint16_t alt_hold_kz_x100;       // outer loop: vSet = kz * zErr  (kz = /100, 1/s)
    uint16_t alt_hold_kpv;           // inner loop: throttle correction per (m/s) error (PWM units per m/s)
    uint16_t alt_hold_kiv;           // inner loop: throttle integral per (m/s*s) error (PWM units per m/s/s)
    uint16_t alt_hold_vmax_cms;      // max vertical speed command (cm/s)
    uint8_t  alt_hold_vstick_slope_x1000; // stick slope (m/s per PWM), m = val/1000 (1..20 => 0.001..0.020)
    uint8_t  alt_hold_vff_gain_x100; // velocity feedforward gain = /100 (e.g. 100 => 1.00)
    uint16_t alt_hold_thrust_zero_pwm;    // PWM at ~0 thrust (e.g. 1150) for tilt compensation linearization
    uint16_t alt_hold_hover_pwm;          // PWM at hover (mg ~= thrust) used as BARO-mode throttle base
    uint16_t alt_hold_i_limit_cms;        // integral limit in (m/s*s) scaled as cm (e.g. 500 -> 5.00)
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

// lifecycle
void positionInit(void);
void calculateEstimatedAltitude(void);
void positionUpdateAltEKFTunables(void);

// public getters (legacy API preserved)
int32_t getEstimatedAltitudeCm(void);
float   getAltitude(void);
int16_t getEstimatedVario(void);
