#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

#define ALTITUDE_ESTIMATOR_VERSION 2

typedef enum {
    ALT_EST_CONFIG_DELAYED_FUSION = (1 << 0),
    ALT_EST_CONFIG_ADAPTIVE_R     = (1 << 1),
    ALT_EST_CONFIG_BARO_STEP      = (1 << 2),
    ALT_EST_CONFIG_TILT_R         = (1 << 3),
    ALT_EST_CONFIG_ACCEL_R        = (1 << 4),
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
    ALT_EST_STATUS_GPS_ALT_VALID = (1 << 9),
    ALT_EST_STATUS_RANGEFINDER_VALID = (1 << 10),
    ALT_EST_STATUS_TILT_R_INFLATED = (1 << 11),
    ALT_EST_STATUS_ACCEL_R_INFLATED = (1 << 12),
} altitudeEstimatorStatusFlags_e;

typedef enum {
    ALT_EST_MEAS_SOURCE_NONE = 0,
    ALT_EST_MEAS_SOURCE_BAROMETER,
    ALT_EST_MEAS_SOURCE_GPS,
    ALT_EST_MEAS_SOURCE_RANGEFINDER,
} altitudeEstimatorMeasurementSource_e;

typedef enum {
    ALT_EST_MEAS_TYPE_NONE = 0,
    ALT_EST_MEAS_TYPE_ALTITUDE,
    ALT_EST_MEAS_TYPE_SURFACE_DISTANCE,
} altitudeEstimatorMeasurementType_e;

typedef enum {
    ALT_EST_MEAS_FLAG_VALID = (1 << 0),
} altitudeEstimatorMeasurementFlags_e;

typedef struct altitudeEstimatorMeasurement_s {
    timeUs_t timeUs;
    altitudeEstimatorMeasurementSource_e source;
    altitudeEstimatorMeasurementType_e type;
    // ALTITUDE is positive-up height; SURFACE_DISTANCE is positive distance to the surface below.
    float valueCm;
    // Measurement variance. Values <= 0 use the source's configured default.
    float varianceCm2;
    // 0 means unknown/lowest quality, 255 means best quality.
    uint8_t quality;
    uint16_t flags;
} altitudeEstimatorMeasurement_t;

typedef struct altitudeEstimatorInput_s {
    bool haveAccel;
    float accelWorldZ;
    bool haveTilt;
    float cosTilt;
} altitudeEstimatorInput_t;

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
    uint16_t alt_est_pz_floor_cm;
    uint16_t alt_est_pv_floor_cms;
    uint16_t alt_est_pba_floor_cms2;
    uint8_t alt_est_tilt_r_start_deg;
    uint8_t alt_est_tilt_r_end_deg;
    uint16_t alt_est_tilt_r_scale_x10;
    uint16_t alt_est_accel_r_start_cms2;
    uint16_t alt_est_accel_r_end_cms2;
    uint16_t alt_est_accel_r_scale_x10;
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
void altitudeEstimatorUpdateWithMeasurements(timeUs_t nowUs, bool armed, const altitudeEstimatorMeasurement_t *measurements, uint8_t measurementCount, const altitudeEstimatorInput_t *input);
void altitudeEstimatorUpdate(timeUs_t nowUs, bool armed, bool haveBaro, float baroAltitudeCm, timeUs_t baroTimeUs, bool haveAccel, float accelWorldZ);

const altitudeEstimatorStatus_t *altitudeEstimatorGetStatus(void);
int32_t altitudeEstimatorGetAltitudeCm(void);
int16_t altitudeEstimatorGetVelocityCms(void);
float altitudeEstimatorGetAltitudeCmFloat(void);
float altitudeEstimatorGetVelocityCmsFloat(void);
float altitudeEstimatorGetPositionRateCmsFloat(void);
