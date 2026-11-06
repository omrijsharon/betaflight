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
#ifdef USE_ALTITUDE_CONTROL
#include "config/feature.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "flight/failsafe.h"
#include "pg/rx.h"
#include "rx/rx.h"
#endif

#include "fc/runtime_config.h"

#include "flight/altitude_estimator.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"
#ifdef USE_ALTITUDE_CONTROL
#include "flight/altitude_control.h"
#endif

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "pg/pg_ids.h"

static float displayAltitudeCm = 0.0f;
#ifdef USE_BARO
static uint32_t lastBaroSampleNumber;
#endif

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

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 13);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = DEFAULT,
    .altitude_prefer_baro = 100,
    .altitude_lpf = 300,
    .altitude_d_lpf = 100,
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

void positionInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
    pt2FilterInit(&altitudeLpf, pt2FilterGain(altitudeCutoffHz, sampleTimeS));

    const float altitudeDerivativeCutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    pt2FilterInit(&altitudeDerivativeLpf, pt2FilterGain(altitudeDerivativeCutoffHz, sampleTimeS));

    altitudeEstimatorInit();
#ifdef USE_BARO
    lastBaroSampleNumber = baro.sampleNumber;
#endif
}

void calculateEstimatedAltitude(void)
{
    const timeUs_t nowUs = micros();
    float baroAltCm = 0.0f;
    bool haveBaro = false;
    timeUs_t baroTimeUs = nowUs;

#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baroIsCalibrated() && baro.sampleNumber != lastBaroSampleNumber) {
        lastBaroSampleNumber = baro.sampleNumber;
        // Driver receipt time minus configured sensor latency, exactly once.
        baroTimeUs = baro.sampleTimeUs - altitudeEstimatorConfig()->alt_est_baro_delay_ms * 1000u;
        baroAltCm = getBaroAltitude();
        haveBaro = true;
    }
#endif

    float accelWorldZ = 0.0f;
    bool haveAccel = false;
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce && accIsCalibrationComplete()) {
        const float accelScale = acc.dev.acc_1G_rec * 9.80665f;
        const float ax = acc.accADC[X] * accelScale;
        const float ay = acc.accADC[Y] * accelScale;
        const float az = acc.accADC[Z] * accelScale;
        accelWorldZ = rMat[2][0] * ax + rMat[2][1] * ay + rMat[2][2] * az - 9.80665f;
        haveAccel = true;
    }

    altitudeEstimatorUpdate(nowUs, ARMING_FLAG(ARMED), haveBaro, baroAltCm, baroTimeUs, haveAccel, accelWorldZ);
    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();

    displayAltitudeCm = pt2FilterApply(&altitudeLpf, status->altitudeCm);
#ifdef USE_VARIO
    estimatedVario = (int16_t)constrain(lrintf(pt2FilterApply(&altitudeDerivativeLpf, status->velocityCms)), INT16_MIN, INT16_MAX);
#endif

#ifdef USE_ALTITUDE_CONTROL
    static timeUs_t lastFusionUs;
    static bool haveFusion;
    if (!ARMING_FLAG(ARMED) || (status->flags & ALT_EST_STATUS_RESET)) {
        haveFusion = false;
    }
    if (status->flags & ALT_EST_STATUS_BARO_FUSED) {
        lastFusionUs = nowUs;
        haveFusion = true;
    }
    if ((uint32_t)(nowUs - lastFusionUs) > 500000) {
        haveFusion = false;
    }
    const bool healthy = haveFusion && haveAccel && (uint32_t)(nowUs - acc.sampleTimeUs) <= 50000 &&
        sensors(SENSOR_BARO) &&
        !(status->flags & (ALT_EST_STATUS_NO_DATUM | ALT_EST_STATUS_INVALID_INPUT | ALT_EST_STATUS_BAD_TIME | ALT_EST_STATUS_RESET));
    const bool allowed = failsafeIsReceivingRxData() && !failsafeIsActive() && !FLIGHT_MODE(GPS_RESCUE_MODE) && !IS_RC_MODE_ACTIVE(BOXGPSRESCUE) &&
        !isLaunchControlActive() && !isFlipOverAfterCrashActive() && !featureIsEnabled(FEATURE_3D) &&
        !isFixedWing() && !isUsingSticksForArming() && rMat[2][2] > 0.5f;
    altitudeControlStep(nowUs, ARMING_FLAG(ARMED), IS_RC_MODE_ACTIVE(BOXALTITUDE), allowed, healthy,
        rcData[THROTTLE] - rxConfig()->midrc, status->altitudeCm * 0.01f, status->velocityCms * 0.01f,
        rMat[2][2], mixerGetThrottle(), mixerAltitudeCeiling());
#endif

    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 0, lrintf(status->accelWorldZCms2));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 1, lrintf(status->baroAltitudeCm));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 2, lrintf(status->altitudeCm));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 3, lrintf(status->velocityCms));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 4, lrintf(sqrtf(MAX(0.0f, status->rEffCm2))));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 5, lrintf(status->innovationCm));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 6, lrintf(status->gateCm));
    DEBUG_SET(DEBUG_ALTITUDE_ESTIMATOR, 7, (int16_t)constrain((status->flags & ALT_EST_STATUS_BARO_REJECT) ? status->rejectStreak : 0, INT16_MIN, INT16_MAX));

#ifdef USE_BARO
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf((haveBaro ? baroAltCm : 0.0f) / 10.0f));
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
