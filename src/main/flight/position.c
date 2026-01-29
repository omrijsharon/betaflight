#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"             // rMat[3][3]
#include "flight/mixer.h"
#include "flight/pid.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"   // acc.accADC[], acc.dev.acc_1G_rec

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

// ------------------ Local storage ------------------

static float displayAltitudeCm = 0.0f;   // what OSD shows (cm)
static float zeroedAltitudeCm  = 0.0f;   // relative altitude (cm), legacy interface

#ifdef USE_VARIO
static int16_t estimatedVario = 0;       // cm/s (from EKF.v, cosmetically filtered)
#endif

static pt2Filter_t altitudeLpf;          // cosmetic display LPF
static pt2Filter_t altitudeDerivativeLpf;// cosmetic vario LPF
static pt2Filter_t baroVarioLpf;         // baro-derived vario LPF (for altitude hold)

// altitude hold (BARO_MODE)
static bool altHoldActive = false;
static float altHoldTargetZ = 0.0f;      // meters
static float altHoldVIntegral = 0.0f;    // integral of v error
static bool altHoldWasInDeadband = true;

// EKF instance
static positionAltEKF_t zEkf;

// ------------------ Config ------------------

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altitudeSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 10);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source       = DEFAULT,
    .altitude_prefer_baro  = 100,
    .altitude_lpf          = 300,   // 3.00 Hz
    .altitude_d_lpf        = 100,   // 1.00 Hz
    // EKF defaults:
    .ekf_qv_centi          = 80,    // 0.80 m/s^2 accel noise std
    .ekf_qba_centi         = 2,     // 0.02 m/s^2/sqrt(s) accel bias RW std
    .ekf_qbb_centi         = 3,     // 0.03 m/sqrt(s) baro bias RW std
    .ekf_r_centi           = 25,    // 0.25 m baro measurement std
    .ekf_gate_sigma_x10    = 30,    // 3.0σ gate
    .ekf_enable_adapt_r    = 1,

    // Altitude hold defaults (BARO_MODE)
    .alt_hold_kz_x100      = 80,    // 0.80 1/s
    .alt_hold_kpv          = 200,   // 200 PWM per (m/s)
    .alt_hold_kiv          = 60,    // 60 PWM per (m/s*s)
    .alt_hold_vmax_cms     = 300,   // 3 m/s
    .alt_hold_vstick_slope_x1000 = 10, // 0.010 m/s per PWM (deadbanded)
    .alt_hold_thrust_zero_pwm = 1150,
    .alt_hold_hover_pwm = 1300,
    .alt_hold_i_limit_cms = 500
);

// ------------------ EKF helpers ------------------

static inline void ekfInit(positionAltEKF_t *ekf, float z0)
{
    ekf->z  = z0;
    ekf->v  = 0.0f;
    ekf->ba = 0.0f;
    ekf->bb = 0.0f;

    // initial covariance
    const float pz0  = 0.10f * 0.10f; // 10 cm std
    const float pv0  = 0.50f * 0.50f; // 0.5 m/s std
    const float pba0 = 1.00f * 1.00f; // 1 m/s^2 std
    const float pbb0 = 1.00f * 1.00f; // 1 m std baro bias

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = 0.0f;
        }
    }
    ekf->P[0][0] = pz0;
    ekf->P[1][1] = pv0;
    ekf->P[2][2] = pba0;
    ekf->P[3][3] = pbb0;

    // Pull from CLI-config (centi-scaling)
    const positionConfig_t *pcfg = positionConfig();
    const float qv   = (pcfg->ekf_qv_centi  / 100.0f);
    const float qba  = (pcfg->ekf_qba_centi / 100.0f);
    const float qbb  = (pcfg->ekf_qbb_centi / 100.0f);
    const float rstd = (pcfg->ekf_r_centi   / 100.0f);

    // noises (tunable)
    ekf->Qv    = qv  * qv;     // (m/s^2)^2
    ekf->Qba   = qba * qba;    // (m/s^2)^2 per sec
    ekf->Qbb   = qbb * qbb;    // (m^2) per sec
    ekf->R     = rstd * rstd;  // m^2
    ekf->R_eff = ekf->R;

    ekf->aWorldZ = 0.0f;
    ekf->innovZ  = 0.0f;
    ekf->r33     = 1.0f;
    ekf->gateRejected = 0;
}

static inline void ekfPredict(positionAltEKF_t *ekf, float aWorldZ, float dt)
{
    const float ba = ekf->ba;

    // state prediction (constant acceleration with accel bias)
    const float a = (aWorldZ - ba);
    ekf->z += ekf->v * dt + 0.5f * a * dt * dt;
    ekf->v += a * dt;
    // ba, bb are random-walk in covariance only

    // F matrix for x=[z v ba bb]
    const float dt2 = dt * dt;
    const float F[4][4] = {
        { 1.0f, dt,  -0.5f * dt2, 0.0f },
        { 0.0f, 1.0f, -dt,         0.0f },
        { 0.0f, 0.0f,  1.0f,       0.0f },
        { 0.0f, 0.0f,  0.0f,       1.0f },
    };

    float FP[4][4] = { { 0 } };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += F[i][k] * ekf->P[k][j];
            }
            FP[i][j] = s;
        }
    }

    float Pn[4][4] = { { 0 } };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += FP[i][k] * F[j][k]; // F^T
            }
            Pn[i][j] = s;
        }
    }

    // Discrete process noise:
    // accel noise drives z and v
    const float sa2 = ekf->Qv;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    Pn[0][0] += 0.25f * dt4 * sa2;
    Pn[0][1] += 0.5f  * dt3 * sa2;
    Pn[1][0] += 0.5f  * dt3 * sa2;
    Pn[1][1] += dt2          * sa2;

    // bias random walks
    Pn[2][2] += ekf->Qba * dt;
    Pn[3][3] += ekf->Qbb * dt;

    // write back (symmetrize)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
        }
    }

    ekf->aWorldZ = aWorldZ;
}

static inline void ekfUpdateZ(positionAltEKF_t *ekf, float zMeas, float R)
{
    // Measurement: zMeas = z + bb + noise
    const float P00 = ekf->P[0][0], P03 = ekf->P[0][3];
    const float P10 = ekf->P[1][0], P13 = ekf->P[1][3];
    const float P20 = ekf->P[2][0], P23 = ekf->P[2][3];
    const float P30 = ekf->P[3][0], P33 = ekf->P[3][3];

    const float S = P00 + P03 + P30 + P33 + R; // HPH^T + R, with H=[1 0 0 1]
    const float invS = 1.0f / MAX(1e-9f, S);

    const float Kz  = (P00 + P03) * invS;
    const float Kv  = (P10 + P13) * invS;
    const float Kba = (P20 + P23) * invS;
    const float Kbb = (P30 + P33) * invS;

    const float innov = zMeas - (ekf->z + ekf->bb);
    ekf->innovZ = innov;

    // state update
    ekf->z  += Kz  * innov;
    ekf->v  += Kv  * innov;
    ekf->ba += Kba * innov;
    ekf->bb += Kbb * innov;

    // Joseph covariance update: P = (I-KH)P(I-KH)' + KRK'
    // H = [1 0 0 1]
    const float K[4] = { Kz, Kv, Kba, Kbb };

    float A[4][4] = {
        { 1.0f - Kz, 0.0f, 0.0f, -Kz },
        { -Kv,       1.0f, 0.0f, -Kv },
        { -Kba,      0.0f, 1.0f, -Kba },
        { -Kbb,      0.0f, 0.0f, 1.0f - Kbb },
    };

    float AP[4][4] = { { 0 } };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += A[i][k] * ekf->P[k][j];
            }
            AP[i][j] = s;
        }
    }

    float Pn[4][4] = { { 0 } };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += AP[i][k] * A[j][k]; // A^T
            }
            Pn[i][j] = s + (K[i] * R * K[j]);
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
        }
    }
}

static inline void ekfUpdateV(positionAltEKF_t *ekf, float vMeas, float Rv)
{
    // Measurement: vMeas = v + noise, H=[0 1 0 0]
    const float S = ekf->P[1][1] + Rv;
    const float invS = 1.0f / MAX(1e-9f, S);

    const float Kz  = ekf->P[0][1] * invS;
    const float Kv  = ekf->P[1][1] * invS;
    const float Kba = ekf->P[2][1] * invS;
    const float Kbb = ekf->P[3][1] * invS;

    const float innov = vMeas - ekf->v;

    ekf->z  += Kz  * innov;
    ekf->v  += Kv  * innov;
    ekf->ba += Kba * innov;
    ekf->bb += Kbb * innov;

    const float K[4] = { Kz, Kv, Kba, Kbb };

    float A[4][4] = {
        { 1.0f, -Kz,  0.0f, 0.0f },
        { 0.0f, 1.0f - Kv, 0.0f, 0.0f },
        { 0.0f, -Kba, 1.0f, 0.0f },
        { 0.0f, -Kbb, 0.0f, 1.0f },
    };

    float AP[4][4] = { { 0 } };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += A[i][k] * ekf->P[k][j];
            }
            AP[i][j] = s;
        }
    }

    float Pn[4][4] = { { 0 } };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += AP[i][k] * A[j][k];
            }
            Pn[i][j] = s + (K[i] * Rv * K[j]);
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
        }
    }
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

    // baro-derived vario is only used as a slow-trim measurement; keep it conservative
    const float baroVarioCutoffHz = MIN(3.0f, MAX(0.3f, altitudeDerivativeCutoffHz));
    pt2FilterInit(&baroVarioLpf, pt2FilterGain(baroVarioCutoffHz, sampleTimeS));

    altHoldActive = false;
    altHoldTargetZ = 0.0f;
    altHoldVIntegral = 0.0f;
    mixerSetThrottleAltitudeCorrection(0);

    ekfInit(&zEkf, 0.0f);
}

#if defined(USE_BARO) || defined(USE_GPS)
void calculateEstimatedAltitude(void)
{
    static bool  wasArmed = false;
    static float baroAltOffsetCm = 0.0f;
    static float newBaroAltOffsetCm = 0.0f;
    static float prevZMeas_m = 0.0f;
    static bool  havePrevZMeas = false;

    // BARO altitude hold debug taps (DEBUG_BARO_ALTHOLD)
    float dbg_uP = 0.0f;      // PWM
    float dbg_uI = 0.0f;      // PWM
    float dbg_u = 0.0f;       // PWM
    float dbg_vP = 0.0f;      // m/s
    float dbg_vFF = 0.0f;     // m/s
    float dbg_vSet = 0.0f;    // m/s
    float dbg_vErr = 0.0f;    // m/s
    float dbg_r33 = 0.0f;     // unitless

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

        altHoldActive = false;
        altHoldVIntegral = 0.0f;
        altHoldWasInDeadband = true;
        mixerSetThrottleAltitudeCorrection(0);
        havePrevZMeas = false;

    } else {
        // armed
        if (!wasArmed) {
            baroAltOffsetCm = newBaroAltOffsetCm;
            wasArmed = true;
            float z0m = haveBaroAlt ? metersFromCm(baroAltCm - baroAltOffsetCm) : 0.0f;
            ekfInit(&zEkf, z0m);
            havePrevZMeas = false;
            altHoldActive = false;
            altHoldVIntegral = 0.0f;
            altHoldWasInDeadband = true;
            mixerSetThrottleAltitudeCorrection(0);
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
            float R = zEkf.R;
            if (positionConfig()->ekf_enable_adapt_r) {
                // Inflate baro variance when tilted or dynamically accelerating.
                // This helps reduce false altitude changes from airflow / pressure disturbances during motion.
                const float r33 = constrainf(fabsf(zEkf.r33), 0.2f, 1.0f);
                const float tiltScale = 1.0f / (r33 * r33); // variance scale
                const float accelScale = 1.0f + 0.20f * MIN(5.0f, fabsf(aWorldZ)); // mild inflation on strong vertical accel
                R *= MIN(25.0f, tiltScale * accelScale);
            }
            zEkf.R_eff = R;

            const float S = zEkf.P[0][0] + zEkf.P[0][3] + zEkf.P[3][0] + zEkf.P[3][3] + R;
            const float innov = zMeas_m - (zEkf.z + zEkf.bb);
            const float gateSigma = positionConfig()->ekf_gate_sigma_x10 / 10.0f;
            const float limit = gateSigma * sqrtf(MAX(1e-9f, S));
            zEkf.gateRejected = (fabsf(innov) > limit) ? 1 : 0;
            if (!zEkf.gateRejected) {
                ekfUpdateZ(&zEkf, zMeas_m, R);

                // Optional: use differentiated baro as a slow velocity trim when baro is trusted.
                if (havePrevZMeas) {
                    const float vBaroRaw = (zMeas_m - prevZMeas_m) / dt;
                    const float vBaro = pt2FilterApply(&baroVarioLpf, vBaroRaw);
                    if (fabsf(zEkf.r33) > 0.80f) {
                        float Rv = 0.50f * 0.50f; // (m/s)^2
                        if (positionConfig()->ekf_enable_adapt_r) {
                            const float r33 = constrainf(fabsf(zEkf.r33), 0.2f, 1.0f);
                            Rv *= MIN(10.0f, 1.0f / (r33 * r33));
                        }
                        ekfUpdateV(&zEkf, vBaro, Rv);
                    }
                }
            }
            zEkf.innovZ = innov; // keep for debug

            prevZMeas_m = zMeas_m;
            havePrevZMeas = true;
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

#ifdef USE_BARO
        // ---- 5b) Altitude hold (BARO_MODE) ----
        if (FLIGHT_MODE(BARO_MODE) && haveBaroAlt) {
            const positionConfig_t *pcfg = positionConfig();
            const float mid = rxConfig()->midrc;
            const float deadband = rcControlsConfig()->alt_hold_deadband;
            const float thr = rcCommand[THROTTLE];

            float stick = thr - mid;
            float stickAdj = 0.0f;
            if (fabsf(stick) > deadband) {
                stickAdj = (stick > 0.0f) ? (stick - deadband) : (stick + deadband);
            }
            const bool inDeadband = (stickAdj == 0.0f);

            if ((stickAdj != 0.0f) && !rcControlsConfig()->alt_hold_fast_change) {
                // Stick moved out of deadband: disable altitude hold until re-centered.
                altHoldActive = false;
                altHoldVIntegral = 0.0f;
                altHoldWasInDeadband = true;
                mixerSetThrottleAltitudeCorrection(0);
            } else {
                if (!altHoldActive) {
                    altHoldActive = true;
                    altHoldTargetZ = zEkf.z;
                    altHoldVIntegral = 0.0f;
                    altHoldWasInDeadband = true;
                    mixerSetThrottleAltitudeCorrection(0);
                }

                // When the throttle stick enters the deadband, re-latch the current altitude as the target.
                // This makes "stick centered" mean "hold current altitude now", rather than returning to the old target.
                if (inDeadband && !altHoldWasInDeadband) {
                    altHoldTargetZ = zEkf.z;
                    altHoldVIntegral = 0.0f;
                }
                altHoldWasInDeadband = inDeadband;

                const float m = constrainf(pcfg->alt_hold_vstick_slope_x1000 / 1000.0f, 0.001f, 0.020f);
                const float vStick = stickAdj * m;
                altHoldTargetZ += vStick * dt;

                const float kz = pcfg->alt_hold_kz_x100 / 100.0f;
                const float vmax = pcfg->alt_hold_vmax_cms / 100.0f;

                float vSet = kz * (altHoldTargetZ - zEkf.z) + vStick;
                vSet = constrainf(vSet, -vmax, vmax);

                const float vErr = vSet - zEkf.v;
                const float kp = pcfg->alt_hold_kpv;
                const float ki = pcfg->alt_hold_kiv;

                const float maxCorr = 500.0f;
                const float uP = kp * vErr;
                float uI = ki * altHoldVIntegral;
                float u = uP + uI;

                // Basic anti-windup: only integrate if not saturated or if integration would unwind.
                if ((fabsf(u) < maxCorr) || ((u > 0.0f) && (vErr < 0.0f)) || ((u < 0.0f) && (vErr > 0.0f))) {
                    altHoldVIntegral += vErr * dt;
                    const float iLimit = MAX(0.0f, pcfg->alt_hold_i_limit_cms) / 100.0f;
                    altHoldVIntegral = constrainf(altHoldVIntegral, -iLimit, iLimit);
                    uI = ki * altHoldVIntegral;
                    u = uP + uI;
                }

                u = constrainf(u, -maxCorr, maxCorr);
                mixerSetThrottleAltitudeCorrection(lrintf(u));

                // debug outputs (gain*term) for BARO altitude hold
                dbg_uP = uP;
                dbg_uI = uI;
                dbg_u = u;
                dbg_vP = kz * (altHoldTargetZ - zEkf.z);
                dbg_vFF = vStick;
                dbg_vSet = vSet;
                dbg_vErr = vErr;
                dbg_r33 = zEkf.r33;
            }
        } else {
            altHoldActive = false;
            altHoldVIntegral = 0.0f;
            mixerSetThrottleAltitudeCorrection(0);
        }
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

    // BARO altitude hold debug:
    // 0: uP (PWM), 1: uI (PWM), 2: u (PWM),
    // 3: vP (cm/s), 4: vFF (cm/s), 5: vSet (cm/s), 6: vErr (cm/s),
    // 7: r33*100 (cos tilt)
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 0, (int16_t)constrain(lrintf(dbg_uP), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 1, (int16_t)constrain(lrintf(dbg_uI), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 2, (int16_t)constrain(lrintf(dbg_u), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 3, (int16_t)constrain(lrintf(dbg_vP * 100.0f), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 4, (int16_t)constrain(lrintf(dbg_vFF * 100.0f), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 5, (int16_t)constrain(lrintf(dbg_vSet * 100.0f), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 6, (int16_t)constrain(lrintf(dbg_vErr * 100.0f), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_BARO_ALTHOLD, 7, (int16_t)constrain(lrintf(dbg_r33 * 100.0f), INT16_MIN, INT16_MAX));

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
