#pragma once

#include "common/time.h"

#define TASK_ALTITUDE_RATE_HZ 100

typedef struct positionConfig_s {
    uint8_t  altitude_source;
    uint8_t  altitude_prefer_baro;
    uint16_t altitude_lpf;           // (value / 100) Hz
    uint16_t altitude_d_lpf;         // (value / 100) Hz

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

// public getters
int32_t getEstimatedAltitudeCm(void);
float   getAltitude(void);
int16_t getEstimatedVario(void);
