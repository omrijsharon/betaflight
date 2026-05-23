#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/altitude_estimator.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "pg/pg_ids.h"

static float displayAltitudeCm = 0.0f;

#ifdef USE_VARIO
static int16_t estimatedVario = 0;
#endif

static pt2Filter_t altitudeLpf;
static pt2Filter_t altitudeDerivativeLpf;

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altitudeSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 12);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = DEFAULT,
    .altitude_prefer_baro = 100,
    .altitude_lpf = 300,
    .altitude_d_lpf = 100,
    .ekf_qv_centi = 80,
    .ekf_qba_centi = 2,
    .ekf_qbb_centi = 3,
    .ekf_r_centi = 25,
    .ekf_gate_sigma_x10 = 30,
    .ekf_enable_adapt_r = 1,
    .ekf_s_min_m2_x1000 = 40,
    .ekf_reject_recovery_start_frames = 30,
    .ekf_recovery_r_scale_x10 = 500,
    .ekf_recovery_decay_tc_frames = 20,
    .ekf_step_innov_thresh_cm = 100,
    .ekf_step_rate_thresh_cms = 100,
    .ekf_step_rate_filter_tau_ms = 150,
    .ekf_step_streak_frames = 5,
    .ekf_step_bb_alpha_x1000 = 100,
    .ekf_step_bb_max_adjust_cm = 300,
    .alt_hold_kz_x100 = 80,
    .alt_hold_kpv = 200,
    .alt_hold_kiv = 60,
    .alt_hold_vmax_cms = 300,
    .alt_hold_vstick_slope_x1000 = 10,
    .alt_hold_vff_gain_x100 = 100,
    .alt_hold_thrust_zero_pwm = 1150,
    .alt_hold_hover_pwm = 1300,
    .alt_hold_i_limit_cms = 500,
);

void positionUpdateAltEKFTunables(void)
{
    altitudeEstimatorUpdateConfig();
}

void positionInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
    pt2FilterInit(&altitudeLpf, pt2FilterGain(altitudeCutoffHz, sampleTimeS));

    const float altitudeDerivativeCutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    pt2FilterInit(&altitudeDerivativeLpf, pt2FilterGain(altitudeDerivativeCutoffHz, sampleTimeS));

    altitudeEstimatorInit();
    mixerSetThrottleAltitudeCorrection(0);
}

void calculateEstimatedAltitude(void)
{
    const timeUs_t nowUs = micros();
    float baroAltCm = 0.0f;
    bool haveBaro = false;

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        baroAltCm = getBaroAltitude();
        haveBaro = true;
    }
#endif

    float accelWorldZ = 0.0f;
    bool haveAccel = false;
    if (sensors(SENSOR_ACC)) {
        const float accelScale = acc.dev.acc_1G_rec * 9.80665f;
        const float ax = acc.accADC[X] * accelScale;
        const float ay = acc.accADC[Y] * accelScale;
        const float az = acc.accADC[Z] * accelScale;
        accelWorldZ = rMat[2][0] * ax + rMat[2][1] * ay + rMat[2][2] * az - 9.80665f;
        haveAccel = true;
    }

    altitudeEstimatorUpdate(nowUs, ARMING_FLAG(ARMED), haveBaro, baroAltCm, nowUs, haveAccel, accelWorldZ);
    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();

    displayAltitudeCm = pt2FilterApply(&altitudeLpf, status->altitudeCm);
#ifdef USE_VARIO
    estimatedVario = (int16_t)constrain(lrintf(pt2FilterApply(&altitudeDerivativeLpf, status->velocityCms)), INT16_MIN, INT16_MAX);
#endif

    mixerSetThrottleAltitudeCorrection(0);

    DEBUG_SET(DEBUG_Z_EKF, 0, lrintf(status->accelWorldZCms2));
    DEBUG_SET(DEBUG_Z_EKF, 1, lrintf(status->baroAltitudeCm));
    DEBUG_SET(DEBUG_Z_EKF, 2, lrintf(status->altitudeCm));
    DEBUG_SET(DEBUG_Z_EKF, 3, lrintf(status->velocityCms));
    DEBUG_SET(DEBUG_Z_EKF, 4, lrintf(sqrtf(MAX(0.0f, status->rEffCm2))));
    DEBUG_SET(DEBUG_Z_EKF, 5, lrintf(status->innovationCm));
    DEBUG_SET(DEBUG_Z_EKF, 6, lrintf(status->gateCm));
    DEBUG_SET(DEBUG_Z_EKF, 7, (int16_t)constrain((status->flags & ALT_EST_STATUS_BARO_REJECT) ? status->rejectStreak : 0, INT16_MIN, INT16_MAX));

#ifdef USE_BARO
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf((haveBaro ? (baroAltCm - status->baroOffsetCm) : 0.0f) / 10.0f));
#endif
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm / 10.0f));
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

float getAltitude(void)
{
    return altitudeEstimatorGetAltitudeCmFloat();
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
