#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

#define ALTITUDE_ESTIMATOR_VERSION 1

typedef enum {
    ALT_EST_CONFIG_DELAYED_FUSION = (1 << 0),
    ALT_EST_CONFIG_ADAPTIVE_R     = (1 << 1),
    ALT_EST_CONFIG_BARO_STEP      = (1 << 2),
} altitudeEstimatorConfigFlags_e;

typedef enum {
    ALT_EST_STATUS_ARMED       = (1 << 0),
    ALT_EST_STATUS_BARO_VALID  = (1 << 1),
    ALT_EST_STATUS_BARO_FUSED  = (1 << 2),
    ALT_EST_STATUS_BARO_REJECT = (1 << 3),
    ALT_EST_STATUS_RECOVERY    = (1 << 4),
    ALT_EST_STATUS_RESET       = (1 << 5),
    ALT_EST_STATUS_BARO_STEP   = (1 << 6),
    ALT_EST_STATUS_HISTORY_MISS = (1 << 7),
    ALT_EST_STATUS_BIAS_LIMIT  = (1 << 8),
} altitudeEstimatorStatusFlags_e;

typedef struct altitudeEstimatorConfig_s {
    uint16_t alt_est_flags;
    uint16_t alt_est_accel_noise_cms2;
    uint16_t alt_est_accel_bias_noise_cms2;
    uint16_t alt_est_accel_bias_limit_cms2;
    uint16_t alt_est_baro_noise_cm;
    uint16_t alt_est_baro_delay_ms;
    uint16_t alt_est_innov_var_floor_cm2;
    uint16_t alt_est_history_ms;
    uint8_t alt_est_gate_sigma_x10;
    uint16_t alt_est_recovery_start_frames;
    uint16_t alt_est_recovery_r_scale_x10;
    uint8_t alt_est_recovery_decay_tc_frames;
    uint16_t alt_est_step_innov_thresh_cm;
    uint16_t alt_est_step_rate_thresh_cms;
    uint16_t alt_est_step_rate_filter_tau_ms;
    uint8_t alt_est_step_streak_frames;
    uint16_t alt_est_step_offset_alpha_x1000;
    uint16_t alt_est_step_offset_limit_cm;
    uint16_t alt_est_height_rate_lpf_hz_x100;
} altitudeEstimatorConfig_t;

typedef struct altitudeEstimatorStatus_s {
    timeUs_t timeUs;
    uint32_t flags;

    float altitudeCm;
    float velocityCms;
    float positionRateCms;
    float baroAltitudeCm;
    float accelWorldZCms2;
    float accelBiasCms2;
    float innovationCm;
    float gateCm;
    float rEffCm2;
    float sCm2;
    float baroOffsetCm;
    float baroAgeMs;
    float configuredDelayMs;

    uint16_t rejectStreak;
    uint16_t recoveryFrames;
    uint16_t stepStreak;
} altitudeEstimatorStatus_t;

PG_DECLARE(altitudeEstimatorConfig_t, altitudeEstimatorConfig);

void altitudeEstimatorInit(void);
void altitudeEstimatorUpdateConfig(void);
void altitudeEstimatorUpdate(timeUs_t nowUs, bool armed, bool haveBaro, float baroAltitudeCm, timeUs_t baroTimeUs, bool haveAccel, float accelWorldZ);

const altitudeEstimatorStatus_t *altitudeEstimatorGetStatus(void);
int32_t altitudeEstimatorGetAltitudeCm(void);
int16_t altitudeEstimatorGetVelocityCms(void);
float altitudeEstimatorGetAltitudeCmFloat(void);
float altitudeEstimatorGetVelocityCmsFloat(void);
float altitudeEstimatorGetPositionRateCmsFloat(void);
