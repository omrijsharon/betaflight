#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"             // rMat[3][3]
#include "flight/pid.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"   // acc.accADC[], acc.dev.acc_1G_rec

#include "pg/pg.h"
#include "pg/pg_ids.h"

// ------------------ Local storage ------------------

static float displayAltitudeCm = 0.0f;   // what OSD shows (cm)
static float zeroedAltitudeCm  = 0.0f;   // relative altitude (cm), legacy interface

#ifdef USE_VARIO
static int16_t estimatedVario = 0;       // cm/s (from EKF.v, cosmetically filtered)
#endif

static pt2Filter_t altitudeLpf;          // cosmetic display LPF
static pt2Filter_t altitudeDerivativeLpf;// cosmetic vario LPF

// EKF instance
static positionAltEKF_t zEkf;

// ------------------ Config ------------------

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altitudeSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 5);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source       = DEFAULT,
    .altitude_prefer_baro  = 100,
    .altitude_lpf          = 300,   // 3.00 Hz
    .altitude_d_lpf        = 100,   // 1.00 Hz
    // EKF defaults (roughly what we hardcoded before):
    .ekf_qv_centi          = 40,    // -> Qv = (0.60)^2
    .ekf_qba_centi         = 1,     // -> Qba = (0.02)^2 per sec
    .ekf_r_centi           = 10,    // -> R = (0.25)^2
    .ekf_gate_sigma_x10    = 40,    // -> 3.0σ gate
    .ekf_enable_adapt_r    = 0
);

// ------------------ EKF helpers ------------------

static inline void ekfInit(positionAltEKF_t *ekf, float z0)
{
    ekf->z  = z0;
    ekf->v  = 0.0f;
    ekf->ba = 0.0f;

    // initial covariance
    const float pz0  = 0.10f * 0.10f; // 10 cm std
    const float pv0  = 0.50f * 0.50f; // 0.5 m/s std
    const float pba0 = 1.00f * 1.00f; // 1 m/s^2 std

    ekf->P[0][0] = pz0;  ekf->P[0][1] = 0;     ekf->P[0][2] = 0;
    ekf->P[1][0] = 0;    ekf->P[1][1] = pv0;   ekf->P[1][2] = 0;
    ekf->P[2][0] = 0;    ekf->P[2][1] = 0;     ekf->P[2][2] = pba0;

        // Pull from CLI-config (centi-scaling)
    const positionConfig_t *pcfg = positionConfig();

    const float qv   = (pcfg->ekf_qv_centi  / 100.0f);
    const float qba  = (pcfg->ekf_qba_centi / 100.0f);
    const float rstd = (pcfg->ekf_r_centi   / 100.0f);

    // noises (tunable)
    ekf->Qz  = 0.0f;
    ekf->Qv  = qv  * qv;     // (m/s^2)^2
    ekf->Qba = qba * qba;    // (m/s^2)^2 per sec
    ekf->R   = rstd * rstd;  // m^2

    ekf->aWorldZ = 0.0f;
    ekf->innovZ  = 0.0f;
    ekf->r33     = 1.0f;
    ekf->gateRejected = 0;
}

static inline void ekfPredict(positionAltEKF_t *ekf, float aWorldZ, float dt)
{
    // state prediction
    const float z = ekf->z;
    const float v = ekf->v;
    const float ba= ekf->ba;

    ekf->z  = z + v * dt;
    ekf->v  = v + (aWorldZ - ba) * dt;
    ekf->ba = ba; // RW in covariance

    // F matrix
    const float F00 = 1.0f, F01 = dt,   F02 = 0.0f;
    const float F10 = 0.0f, F11 = 1.0f, F12 = -dt;
    const float F20 = 0.0f, F21 = 0.0f, F22 = 1.0f;

    // P update: P = F P F^T + Qd (diag approx)
    float P00 = ekf->P[0][0], P01 = ekf->P[0][1], P02 = ekf->P[0][2];
    float P10 = ekf->P[1][0], P11 = ekf->P[1][1], P12 = ekf->P[1][2];
    float P20 = ekf->P[2][0], P21 = ekf->P[2][1], P22 = ekf->P[2][2];

    const float FP00 = F00*P00 + F01*P10 + F02*P20;
    const float FP01 = F00*P01 + F01*P11 + F02*P21;
    const float FP02 = F00*P02 + F01*P12 + F02*P22;

    const float FP10 = F10*P00 + F11*P10 + F12*P20;
    const float FP11 = F10*P01 + F11*P11 + F12*P21;
    const float FP12 = F10*P02 + F11*P12 + F12*P22;

    const float FP20 = F20*P00 + F21*P10 + F22*P20;
    const float FP21 = F20*P01 + F21*P11 + F22*P21;
    const float FP22 = F20*P02 + F21*P12 + F22*P22;

    ekf->P[0][0] = FP00*F00 + FP01*F01 + FP02*F02;
    ekf->P[0][1] = FP00*F10 + FP01*F11 + FP02*F12;
    ekf->P[0][2] = FP00*F20 + FP01*F21 + FP02*F22;

    ekf->P[1][0] = FP10*F00 + FP11*F01 + FP12*F02;
    ekf->P[1][1] = FP10*F10 + FP11*F11 + FP12*F12;
    ekf->P[1][2] = FP10*F20 + FP11*F21 + FP12*F22;

    ekf->P[2][0] = FP20*F00 + FP21*F01 + FP22*F02;
    ekf->P[2][1] = FP20*F10 + FP21*F11 + FP22*F12;
    ekf->P[2][2] = FP20*F20 + FP21*F21 + FP22*F22;

    ekf->P[0][0] += ekf->Qz;
    ekf->P[1][1] += ekf->Qv * dt;
    ekf->P[2][2] += ekf->Qba * dt;

    ekf->aWorldZ = aWorldZ;
}

static inline void ekfUpdateZ(positionAltEKF_t *ekf, float zMeas)
{
    // S, K
    const float P00 = ekf->P[0][0], P01 = ekf->P[0][1], P02 = ekf->P[0][2];
    const float P10 = ekf->P[1][0], P11 = ekf->P[1][1], P12 = ekf->P[1][2];
    const float P20 = ekf->P[2][0], P21 = ekf->P[2][1], P22 = ekf->P[2][2];

    const float S    = P00 + ekf->R;
    const float invS = 1.0f / MAX(1e-9f, S);

    const float Kz  = P00 * invS;
    const float Kv  = P10 * invS;
    const float Kba = P20 * invS;

    const float innov = zMeas - ekf->z;
    ekf->innovZ = innov;

    // state update
    ekf->z  += Kz  * innov;
    ekf->v  += Kv  * innov;
    ekf->ba += Kba * innov;

    // covariance update
    const float IminusK = 1.0f - Kz;

    float nP00 = IminusK * P00;
    float nP01 = IminusK * P01;
    float nP02 = IminusK * P02;

    float nP10 = P10 - Kv * P00;
    float nP11 = P11 - Kv * P01;
    float nP12 = P12 - Kv * P02;

    float nP20 = P20 - Kba * P00;
    float nP21 = P21 - Kba * P01;
    float nP22 = P22 - Kba * P02;

    ekf->P[0][0] = nP00; ekf->P[0][1] = nP01; ekf->P[0][2] = nP02;
    ekf->P[1][0] = nP10; ekf->P[1][1] = nP11; ekf->P[1][2] = nP12;
    ekf->P[2][0] = nP20; ekf->P[2][1] = nP21; ekf->P[2][2] = nP22;
}

// ------------------ Position task ------------------

static inline float metersFromCm(float cm) { return cm * 0.01f; }
static inline float cmFromMeters(float m)  { return m * 100.0f;  }

void positionInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    // cosmetic filters
    const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
    const float altitudeGain     = pt2FilterGain(altitudeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeLpf, altitudeGain);

    const float altitudeDerivativeCutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    const float altitudeDerivativeGain     = pt2FilterGain(altitudeDerivativeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeDerivativeLpf, altitudeDerivativeGain);

    ekfInit(&zEkf, 0.0f);
}

#if defined(USE_BARO) || defined(USE_GPS)
void calculateEstimatedAltitude(void)
{
    static bool  wasArmed = false;
    static float baroAltOffsetCm = 0.0f;
    static float newBaroAltOffsetCm = 0.0f;

    // ---- 1) read sensors ----
    float baroAltCm = 0.0f;
    bool  haveBaroAlt = false;

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        baroAltCm   = getBaroAltitude();  // cm, absolute-ish
        haveBaroAlt = true;
    }
#endif

    // --- compute aWorldZ every loop (and run EKF predict) ------------
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (sensors(SENSOR_ACC)) {
        const float k = acc.dev.acc_1G_rec * 9.80665f; // g -> m/s^2
        ax = acc.accADC[X] * k;
        ay = acc.accADC[Y] * k;
        az = acc.accADC[Z] * k;
    }

    const float aWorldZ = rMat[2][0]*ax + rMat[2][1]*ay + rMat[2][2]*az - 9.80665f;
    zEkf.r33 = rMat[2][2]; // for debug

    const float dt = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    // Allow bench verification of gravity compensation and attitude coupling while disarmed.
    // When debug_mode = Z_EKF:
    //  - debug[0] shows r33*100
    //  - debug[1] shows aWorldZ*100 (centi m/s^2), should be ~0 at rest/level
    DEBUG_SET(DEBUG_Z_EKF, 0, lrintf(zEkf.r33 * 100.0f));
    DEBUG_SET(DEBUG_Z_EKF, 1, lrintf(aWorldZ * 100.0f));
    if (!ARMING_FLAG(ARMED)) {
        DEBUG_SET(DEBUG_Z_EKF, 2, 0);
        DEBUG_SET(DEBUG_Z_EKF, 3, 0);
        DEBUG_SET(DEBUG_Z_EKF, 4, 0);
        DEBUG_SET(DEBUG_Z_EKF, 5, 0);
        DEBUG_SET(DEBUG_Z_EKF, 6, 0);
        DEBUG_SET(DEBUG_Z_EKF, 7, 0);
    }

    ekfPredict(&zEkf, aWorldZ, dt);

    // ---- 2) arm/disarm handling & zeroing ----
    if (!ARMING_FLAG(ARMED)) {
        if (wasArmed) {
            wasArmed = false;
        }

        newBaroAltOffsetCm = 0.2f * baroAltCm + 0.8f * newBaroAltOffsetCm;
        displayAltitudeCm  = baroAltCm - baroAltOffsetCm;   // show recent baro zero
        zeroedAltitudeCm   = 0.0f;

    } else {
        // armed
        if (!wasArmed) {
            baroAltOffsetCm = newBaroAltOffsetCm;
            wasArmed = true;
            float z0m = haveBaroAlt ? metersFromCm(baroAltCm - baroAltOffsetCm) : 0.0f;
            ekfInit(&zEkf, z0m);
        }

        // measurement (relative baro → meters)
        float zMeas_m = 0.0f;
        bool  zMeasValid = false;

        if (haveBaroAlt) {
            zMeas_m   = metersFromCm(baroAltCm - baroAltOffsetCm);
            zMeasValid= true;
        }

        // ---- 4) EKF update with baro (if valid & gated) ----
        if (zMeasValid) {
            const float S = zEkf.P[0][0] + zEkf.R;
            const float innov = zMeas_m - zEkf.z;
            const float gateSigma = positionConfig()->ekf_gate_sigma_x10 / 10.0f;
            const float limit = gateSigma * sqrtf(MAX(1e-9f, S));
            zEkf.gateRejected = (fabsf(innov) > limit) ? 1 : 0;
            if (!zEkf.gateRejected) {
                ekfUpdateZ(&zEkf, zMeas_m);
            }
            zEkf.innovZ = innov; // keep for debug
        }

        // ---- 5) outputs & cosmetic filters ----
        float zCmFiltered = pt2FilterApply(&altitudeLpf, cmFromMeters(zEkf.z));
        displayAltitudeCm = zCmFiltered;

#ifdef USE_VARIO
        float varioCms = zEkf.v * 100.0f;
        varioCms = pt2FilterApply(&altitudeDerivativeLpf, varioCms);
        estimatedVario = lrintf(varioCms);
        estimatedVario = applyDeadband(estimatedVario, 10); // 0.1 m/s deadband
#endif

    // ---- 6) debug channels (guarded) ----
    // Debug is int16_t: scale for resolution
    DEBUG_SET(DEBUG_Z_EKF, 0, lrintf(zEkf.r33 * 100.0f));     // 0..100
    DEBUG_SET(DEBUG_Z_EKF, 1, lrintf(aWorldZ * 100.0f));      // centi m/s^2
    DEBUG_SET(DEBUG_Z_EKF, 2, lrintf(zMeas_m * 100.0f));      // cm
    DEBUG_SET(DEBUG_Z_EKF, 3, lrintf(zEkf.z * 100.0f));       // cm
    DEBUG_SET(DEBUG_Z_EKF, 4, lrintf(zEkf.v * 100.0f));       // cm/s
    DEBUG_SET(DEBUG_Z_EKF, 5, lrintf(zEkf.ba * 100.0f));      // centi m/s^2
    DEBUG_SET(DEBUG_Z_EKF, 6, lrintf(zEkf.innovZ * 100.0f));  // cm
    DEBUG_SET(DEBUG_Z_EKF, 7, (int16_t)(zEkf.gateRejected ? 1 : 0));

    }

#ifdef USE_BARO
    // legacy: relative baro (cm/10) on DEBUG_ALTITUDE[1]
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf((haveBaroAlt ? (baroAltCm - baroAltOffsetCm) : 0.0f) / 10.0f));
#endif
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm / 10.0f));
}
#endif // USE_BARO || USE_GPS

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

float getAltitude(void)
{
    // expose EKF z (m) as cm for legacy callers
    return cmFromMeters(zEkf.z);
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
