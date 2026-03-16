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

#ifdef USE_VARIO
static int16_t estimatedVario = 0;       // cm/s (from EKF.v, cosmetically filtered)
#endif

static pt2Filter_t altitudeLpf;          // cosmetic display LPF
static pt2Filter_t altitudeDerivativeLpf;// cosmetic vario LPF

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

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 12);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source       = DEFAULT,       // TODO: not yet used by Z_EKF; reserved for future GPS altitude fusion
    .altitude_prefer_baro  = 100,          // TODO: not yet used by Z_EKF; reserved for future GPS/baro blending
    .altitude_lpf          = 300,   // 3.00 Hz
    .altitude_d_lpf        = 100,   // 1.00 Hz
    // EKF defaults:
    .ekf_qv_centi          = 80,    // 0.80 m/s^2 accel noise std
    .ekf_qba_centi         = 2,     // 0.02 m/s^2/sqrt(s) accel bias RW std
    .ekf_qbb_centi         = 3,     // 0.03 m/sqrt(s) baro bias RW std
    .ekf_r_centi           = 25,    // 0.25 m baro measurement std
    .ekf_gate_sigma_x10    = 30,    // 3.0σ gate
    .ekf_enable_adapt_r    = 1,

    // EKF robustness defaults:
    .ekf_s_min_m2_x1000               = 40,   // 0.040 m^2 (sigma ~= 0.20 m)
    .ekf_reject_recovery_start_frames = 30,
    .ekf_recovery_r_scale_x10         = 500,  // 50.0x (initial R multiplier; decays exponentially)
    .ekf_recovery_decay_tc_frames     = 20,   // decay time constant: R_mult halves in ~0.7*20 = 14 frames
    .ekf_step_innov_thresh_cm         = 100,  // 1.00 m
    .ekf_step_rate_thresh_cms         = 100,  // 1.00 m/s
    .ekf_step_rate_filter_tau_ms      = 150,  // 0.15 s
    .ekf_step_streak_frames           = 5,
    .ekf_step_bb_alpha_x1000          = 100,  // 0.100
    .ekf_step_bb_max_adjust_cm        = 300,  // 3.00 m

    // Altitude hold defaults (BARO_MODE)
    .alt_hold_kz_x100      = 80,    // 0.80 1/s
    .alt_hold_kpv          = 200,   // 200 PWM per (m/s)
    .alt_hold_kiv          = 60,    // 60 PWM per (m/s*s)
    .alt_hold_vmax_cms     = 300,   // 3 m/s
    .alt_hold_vstick_slope_x1000 = 10, // 0.010 m/s per PWM (deadbanded)
    .alt_hold_vff_gain_x100 = 100,  // 1.00x velocity feedforward gain
    .alt_hold_thrust_zero_pwm = 1150,
    .alt_hold_hover_pwm = 1300,
    .alt_hold_i_limit_cms = 500
);

// ------------------ EKF helpers ------------------

// Robustness helpers for the altitude EKF:
//  - S_min prevents the Z gate from collapsing due to overconfidence / tiny R tuning (tunable via PG_POSITION).
//  - P floors prevent numerical collapse that can lead to gate deadlock.
static const float Z_EKF_PZ_MIN = (0.05f * 0.05f); // (m)^2
static const float Z_EKF_PV_MIN = (0.05f * 0.05f); // (m/s)^2
static const float Z_EKF_PBA_MIN = (0.10f * 0.10f); // (m/s^2)^2
static const float Z_EKF_PBB_MIN = (0.10f * 0.10f); // (m)^2

static inline void ekfCovarianceFloor(positionAltEKF_t *ekf)
{
    ekf->P[0][0] = MAX(ekf->P[0][0], Z_EKF_PZ_MIN);
    ekf->P[1][1] = MAX(ekf->P[1][1], Z_EKF_PV_MIN);
    ekf->P[2][2] = MAX(ekf->P[2][2], Z_EKF_PBA_MIN);
    ekf->P[3][3] = MAX(ekf->P[3][3], Z_EKF_PBB_MIN);
}

void positionUpdateAltEKFTunables(void)
{
    const positionConfig_t *pcfg = positionConfig();
    const float qv   = (pcfg->ekf_qv_centi  / 100.0f);
    const float qba  = (pcfg->ekf_qba_centi / 100.0f);
    const float qbb  = (pcfg->ekf_qbb_centi / 100.0f);
    const float rstd = (pcfg->ekf_r_centi   / 100.0f);

    zEkf.Qv    = qv  * qv;     // (m/s^2)^2
    zEkf.Qba   = qba * qba;    // (m/s^2)^2 per sec
    zEkf.Qbb   = qbb * qbb;    // (m^2) per sec
    zEkf.R     = rstd * rstd;  // m^2
    zEkf.R_eff = zEkf.R;

    zEkf.S_min = constrainf((positionConfig()->ekf_s_min_m2_x1000 / 1000.0f), 1e-6f, 10.0f);
}

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

    // noises (tunable)
    positionUpdateAltEKFTunables();

    ekf->aWorldZ = 0.0f;
    ekf->innovZ  = 0.0f;
    ekf->r33     = 1.0f;
    ekf->gateRejected = 0;
    ekf->rejectStreak = 0;
    ekf->recoveryRMult = 1.0f;
    ekf->gateForced = 0;
    ekf->gateS = 0.0f;
    ekf->gateLimit = 0.0f;
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

    ekfCovarianceFloor(ekf);

    ekf->aWorldZ = aWorldZ;
}

static inline void ekfUpdateZ(positionAltEKF_t *ekf, float zMeas, float R)
{
    // Measurement: zMeas = z + bb + noise
    const float P00 = ekf->P[0][0], P03 = ekf->P[0][3];
    const float P10 = ekf->P[1][0], P13 = ekf->P[1][3];
    const float P20 = ekf->P[2][0], P23 = ekf->P[2][3];
    const float P30 = ekf->P[3][0], P33 = ekf->P[3][3];

    const float Sraw = P00 + P03 + P30 + P33 + R; // HPH^T + R, with H=[1 0 0 1]
    const float S = MAX(ekf->S_min, Sraw);
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

    ekfCovarianceFloor(ekf);
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
    static bool  haveBaroAltOffsetInit = false;
    static float prevZMeas_m = 0.0f;
    static bool  havePrevZMeas = false;
    static int8_t baroStepSign = 0;
    static uint8_t baroStepStreak = 0;
    static float baroStepPrevAbsInnov = 0.0f;
    static float baroStepZRateFilt_mps = 0.0f;

    // BARO altitude hold debug taps (DEBUG_BARO_ALTHOLD)
    float dbg_uP = 0.0f;      // PWM
    float dbg_uI = 0.0f;      // PWM
    float dbg_u = 0.0f;       // PWM
    float dbg_vP = 0.0f;      // m/s (outer-loop altitude->velocity term; active in deadband only)
    float dbg_vFF = 0.0f;     // m/s (stick velocity feedforward after gain; non-deadband only)
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

    const float dtNom = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);
    float dt = dtNom;
    const timeDelta_t dtUs = getTaskDeltaTimeUs(TASK_ALTITUDE);
    if (dtUs > 0) {
        dt = dtUs * 1e-6f;
    }
    dt = constrainf(dt, 0.5f * dtNom, 2.0f * dtNom);

    // When debug_mode = Z_EKF (armed):
    //  0: aWorldZ*100 (centi m/s^2), 1: zMeas(cm), 2: z(cm), 3: v(cm/s),
    //  4: sqrt(R_eff)*100 (cm), 5: innov(cm), 6: limit(cm), 7: gate state
    DEBUG_SET(DEBUG_Z_EKF, 0, lrintf(aWorldZ * 100.0f));
    if (!ARMING_FLAG(ARMED)) {
        DEBUG_SET(DEBUG_Z_EKF, 0, 0);
        DEBUG_SET(DEBUG_Z_EKF, 1, 0);
        DEBUG_SET(DEBUG_Z_EKF, 2, 0);
        DEBUG_SET(DEBUG_Z_EKF, 3, 0);
        DEBUG_SET(DEBUG_Z_EKF, 4, 0);
        DEBUG_SET(DEBUG_Z_EKF, 5, 0);
        DEBUG_SET(DEBUG_Z_EKF, 6, 0);
        DEBUG_SET(DEBUG_Z_EKF, 7, 0);
    }

    // ---- 2) arm/disarm handling & zeroing ----
    if (!ARMING_FLAG(ARMED)) {
        // While disarmed the EKF state is meaningless (it will be re-initialized on arm).
        // Reset the EKF to the current baro reading each frame so it doesn't drift open-loop.
        // No predict step: it would be immediately overwritten by ekfInit.
        if (haveBaroAlt) {
            float z0 = metersFromCm(baroAltCm - newBaroAltOffsetCm);
            ekfInit(&zEkf, z0);
        } else {
            ekfInit(&zEkf, 0.0f);
        }

        if (wasArmed) {
            wasArmed = false;
            haveBaroAltOffsetInit = false;
        }

        if (haveBaroAlt) {
            if (!haveBaroAltOffsetInit) {
                newBaroAltOffsetCm = baroAltCm;
                haveBaroAltOffsetInit = true;
            } else {
                newBaroAltOffsetCm = 0.2f * baroAltCm + 0.8f * newBaroAltOffsetCm;
            }
        }
        // While disarmed, show altitude relative to the current (disarmed) baro zero estimate.
        // This avoids confusing "display" values before baroAltOffsetCm is latched on arming.
        displayAltitudeCm  = haveBaroAlt ? (baroAltCm - newBaroAltOffsetCm) : 0.0f;

        altHoldActive = false;
        altHoldVIntegral = 0.0f;
        altHoldWasInDeadband = true;
        mixerSetThrottleAltitudeCorrection(0);
        havePrevZMeas = false;
        baroStepSign = 0;
        baroStepStreak = 0;
        baroStepPrevAbsInnov = 0.0f;
        baroStepZRateFilt_mps = 0.0f;
        zEkf.rejectStreak = 0;
        zEkf.recoveryRMult = 1.0f;
        zEkf.gateForced = 0;

    } else {
        // armed
        ekfPredict(&zEkf, aWorldZ, dt);

        if (!wasArmed) {
            baroAltOffsetCm = newBaroAltOffsetCm;
            haveBaroAltOffsetInit = false;
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
            const positionConfig_t *pcfg = positionConfig();

            // Recovery parameters (frames at TASK_ALTITUDE_RATE_HZ)
            const uint16_t rejectRecoveryStartFrames = MAX((uint16_t)1, pcfg->ekf_reject_recovery_start_frames);
            const float    recoveryRScale = constrainf((pcfg->ekf_recovery_r_scale_x10 / 10.0f), 1.0f, 1000.0f);
            const uint8_t  recoveryDecayTc = (uint8_t)constrain(pcfg->ekf_recovery_decay_tc_frames, 1, 255);
            // Exponential decay factor per frame: exp(-1/tc).  Pre-compute once.
            const float    recoveryDecayAlpha = expf(-1.0f / (float)recoveryDecayTc);

            // Baro step handling (windows/pressure steps): if innovation is large and the baro is stepping fast,
            // adjust bb directly so the filter doesn't deadlock on gating.
            const float    stepInnovThresh_m = MAX(0.0f, pcfg->ekf_step_innov_thresh_cm / 100.0f);
            const float    stepRateThresh_mps = MAX(0.0f, pcfg->ekf_step_rate_thresh_cms / 100.0f);
            const uint16_t stepPersistRejectFrames = 10; // allow step handling even after the initial spike
            const uint8_t  stepStreakFrames = (uint8_t)constrain(pcfg->ekf_step_streak_frames, 1, 255);
            const float    stepBbAlpha = constrainf((pcfg->ekf_step_bb_alpha_x1000 / 1000.0f), 0.0f, 1.0f);
            const float    stepBbMaxAdjust_m = MAX(0.0f, pcfg->ekf_step_bb_max_adjust_cm / 100.0f);
            const float    stepPbbBoost = 0.50f; // (m)^2
            const float    stepInnovNotShrinkingEps_m = 0.05f;
            const float    stepRateFiltTau_s = MAX(0.0f, pcfg->ekf_step_rate_filter_tau_ms / 1000.0f);

            float R = zEkf.R;
            if (pcfg->ekf_enable_adapt_r) {
                // Inflate baro variance when tilted or dynamically accelerating.
                // This helps reduce false altitude changes from airflow / pressure disturbances during motion.
                const float r33 = constrainf(fabsf(zEkf.r33), 0.2f, 1.0f);
                const float tiltScale = 1.0f / (r33 * r33); // variance scale
                const float accelScale = 1.0f + 0.20f * MIN(5.0f, fabsf(aWorldZ)); // mild inflation on strong vertical accel
                R *= MIN(25.0f, tiltScale * accelScale);
            }

            const float zRateRaw_mps = havePrevZMeas ? ((zMeas_m - prevZMeas_m) / dt) : 0.0f;
            if (!havePrevZMeas) {
                baroStepZRateFilt_mps = 0.0f;
            } else {
                const float alpha = constrainf(dt / (stepRateFiltTau_s + dt), 0.0f, 1.0f);
                baroStepZRateFilt_mps += alpha * (zRateRaw_mps - baroStepZRateFilt_mps);
            }
            float innov = zMeas_m - (zEkf.z + zEkf.bb);
            float absInnov = fabsf(innov);
            const float gateSigma = pcfg->ekf_gate_sigma_x10 / 10.0f;

            // Nominal gate (pre-recovery): use an S floor to avoid permanent rejection deadlock.
            float S = zEkf.P[0][0] + zEkf.P[0][3] + zEkf.P[3][0] + zEkf.P[3][3] + R;
            S = MAX(zEkf.S_min, S);
            float limit = gateSigma * sqrtf(MAX(1e-9f, S));
            bool gateRejectedNominal = (fabsf(innov) > limit);

            const bool alreadyRecovering = (zEkf.recoveryRMult > 1.01f);
            if (!alreadyRecovering) {
                if (gateRejectedNominal) {
                    zEkf.rejectStreak = (zEkf.rejectStreak == UINT16_MAX) ? UINT16_MAX : (uint16_t)(zEkf.rejectStreak + 1);
                } else {
                    zEkf.rejectStreak = 0;
                }
            }

            // Baro step handling via bb when gating is rejecting.
            if (gateRejectedNominal && havePrevZMeas && (absInnov > stepInnovThresh_m) &&
                ((fabsf(baroStepZRateFilt_mps) > stepRateThresh_mps) || (zEkf.rejectStreak >= stepPersistRejectFrames))) {
                const bool innovNotShrinking = (baroStepPrevAbsInnov <= 0.0f) || (absInnov >= (baroStepPrevAbsInnov - stepInnovNotShrinkingEps_m));
                baroStepPrevAbsInnov = absInnov;

                if (!innovNotShrinking) {
                    baroStepStreak = 0;
                    baroStepSign = 0;
                }

                const int8_t sign = (innov >= 0.0f) ? 1 : -1;
                if (innovNotShrinking && (sign == baroStepSign)) {
                    baroStepStreak = (baroStepStreak == 255) ? 255 : (uint8_t)(baroStepStreak + 1);
                } else {
                    baroStepSign = sign;
                    baroStepStreak = innovNotShrinking ? 1 : 0;
                }

                if (baroStepStreak >= stepStreakFrames) {
                    const float bbAdjust = constrainf(stepBbAlpha * innov, -stepBbMaxAdjust_m, stepBbMaxAdjust_m);
                    zEkf.bb += bbAdjust;
                    zEkf.P[3][3] += stepPbbBoost;
                    ekfCovarianceFloor(&zEkf);
                    baroStepStreak = 0;

                    // Recompute innovation after bb adjustment.
                    innov = zMeas_m - (zEkf.z + zEkf.bb);
                    absInnov = fabsf(innov);
                    S = zEkf.P[0][0] + zEkf.P[0][3] + zEkf.P[3][0] + zEkf.P[3][3] + R;
                    S = MAX(zEkf.S_min, S);
                    limit = gateSigma * sqrtf(MAX(1e-9f, S));
                    gateRejectedNominal = (absInnov > limit);

                    // If the bb nudge brought us back into the gate, clear the reject streak to avoid
                    // immediately entering recovery due to stale reject history.
                    if (!gateRejectedNominal) {
                        zEkf.rejectStreak = 0;
                    }
                }
            } else {
                baroStepStreak = 0;
                baroStepPrevAbsInnov = 0.0f;
                baroStepSign = 0;
            }

            // Recovery: if gating has been rejecting for too long, kick the R multiplier
            // to recoveryRScale and let it decay exponentially toward 1.0 each frame.
            // While recoveryRMult > 1, updates are forced (gate bypassed) with inflated R,
            // giving the filter a smooth, decreasing leash back to normal confidence.
            if (gateRejectedNominal && (zEkf.rejectStreak >= rejectRecoveryStartFrames) && !alreadyRecovering) {
                zEkf.recoveryRMult = recoveryRScale;
                zEkf.rejectStreak = 0;
            }

            // Decay the recovery multiplier toward 1.0 every frame.
            if (zEkf.recoveryRMult > 1.01f) {
                // mult(k+1) = 1 + (mult(k) - 1) * alpha,  where alpha = exp(-1/tc)
                zEkf.recoveryRMult = 1.0f + (zEkf.recoveryRMult - 1.0f) * recoveryDecayAlpha;
                if (zEkf.recoveryRMult < 1.01f) {
                    zEkf.recoveryRMult = 1.0f;
                }
            }

            // If the nominal gate is healthy, snap recovery off immediately.
            if (!gateRejectedNominal) {
                zEkf.recoveryRMult = 1.0f;
            }

            const bool forceUpdate = (zEkf.recoveryRMult > 1.01f);
            zEkf.gateForced = forceUpdate ? 1 : 0;

            bool gateRejectedFinal = gateRejectedNominal;
            if (forceUpdate) {
                R *= zEkf.recoveryRMult;
                gateRejectedFinal = false;
            }

            // Store final gate values for debug (before the update modifies P).
            zEkf.R_eff = R;
            zEkf.innovZ = innov;
            zEkf.gateRejected = gateRejectedFinal ? 1 : 0;

            // Compute gate S and limit from pre-update P for consistent debug output.
            zEkf.gateS = zEkf.P[0][0] + zEkf.P[0][3] + zEkf.P[3][0] + zEkf.P[3][3] + R;
            zEkf.gateS = MAX(zEkf.S_min, zEkf.gateS);
            zEkf.gateLimit = gateSigma * sqrtf(MAX(1e-9f, zEkf.gateS));

            if (!gateRejectedFinal) {
                ekfUpdateZ(&zEkf, zMeas_m, R);
                // Note: baro-derived velocity (differentiated baro) is NOT fused here.
                // The Z update already corrects velocity via the cross-covariance (Kv * innov).
                // Fusing differentiated baro as a separate velocity measurement would double-count
                // the same baro information, making P artificially small (overconfident velocity).
                // This is critical for altitude hold: accurate velocity uncertainty drives proper
                // Kalman gain balance between accel prediction and baro correction.
            }

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
#ifdef USE_BARO_ALTHOLD
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
                // alt_hold_fast_change OFF (default): altitude hold is disabled entirely when
                // the throttle stick leaves the deadband, giving direct throttle control.
                // alt_hold_fast_change ON: altitude hold stays active and the target altitude
                // changes proportionally to stick deflection (velocity-command mode).
                altHoldActive = false;
                altHoldVIntegral = 0.0f;
                altHoldWasInDeadband = true;
                mixerSetThrottleAltitudeCorrection(0);
            } else {
                if (!altHoldActive) {
                    altHoldActive = true;
                    altHoldTargetZ = zEkf.z;
                    altHoldVIntegral = 0.0f;
                    altHoldWasInDeadband = inDeadband;
                    mixerSetThrottleAltitudeCorrection(0);
                }

                const float m = constrainf(pcfg->alt_hold_vstick_slope_x1000 / 1000.0f, 0.001f, 0.020f);
                const float vStick = stickAdj * m;
                const float vffGain = constrainf(pcfg->alt_hold_vff_gain_x100 / 100.0f, 0.0f, 2.0f);
                const float vFF = vStick * vffGain;

                const float kz = pcfg->alt_hold_kz_x100 / 100.0f;
                const float vmax = pcfg->alt_hold_vmax_cms / 100.0f;

                // Two-state behavior:
                //  - Out of deadband: velocity control only. Track vFF, keep z target aligned to current z.
                //  - In deadband: hold altitude. Freeze z target and use outer loop to generate velocity setpoint.
                if (inDeadband && !altHoldWasInDeadband) {
                    // entering hold
                    altHoldTargetZ = zEkf.z;
                    altHoldVIntegral = 0.0f;
                } else if (!inDeadband && altHoldWasInDeadband) {
                    // leaving hold
                    altHoldVIntegral = 0.0f;
                }

                if (!inDeadband) {
                    altHoldTargetZ = zEkf.z;
                }
                altHoldWasInDeadband = inDeadband;

                const float vP = inDeadband ? (kz * (altHoldTargetZ - zEkf.z)) : 0.0f;
                float vSet = vP + vFF;
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
                // Note: tilt compensation (1/cos(tilt)) for the throttle correction is applied
                // downstream in mixer.c, not here.  See mixerApplyThrottleAltitudeCorrection().
                mixerSetThrottleAltitudeCorrection(lrintf(u));

                // debug outputs (gain*term) for BARO altitude hold
                dbg_uP = uP;
                dbg_uI = uI;
                dbg_u = u;
                dbg_vP = vP;
                dbg_vFF = vFF;
                dbg_vSet = vSet;
                dbg_vErr = vErr;
                dbg_r33 = zEkf.r33;
            }
        } else {
            altHoldActive = false;
            altHoldVIntegral = 0.0f;
            mixerSetThrottleAltitudeCorrection(0);
        }
#endif // USE_BARO_ALTHOLD
#endif

    // ---- 6) debug channels (guarded) ----
    // Note: still inside the armed `else` branch, so zMeas_m (declared above) is in scope.
    // Debug is int16_t: scale for resolution
    DEBUG_SET(DEBUG_Z_EKF, 0, lrintf(aWorldZ * 100.0f));                    // centi m/s^2
    DEBUG_SET(DEBUG_Z_EKF, 1, lrintf(zMeas_m * 100.0f));                    // cm
    DEBUG_SET(DEBUG_Z_EKF, 2, lrintf(zEkf.z * 100.0f));                     // cm
    DEBUG_SET(DEBUG_Z_EKF, 3, lrintf(zEkf.v * 100.0f));                     // cm/s
    DEBUG_SET(DEBUG_Z_EKF, 4, lrintf(sqrtf(MAX(0.0f, zEkf.R_eff)) * 100.0f)); // cm
    DEBUG_SET(DEBUG_Z_EKF, 5, lrintf(zEkf.innovZ * 100.0f));                // cm
    DEBUG_SET(DEBUG_Z_EKF, 6, lrintf(zEkf.gateLimit * 100.0f));             // cm
    int16_t gateState = 0;
    if (zEkf.gateForced) {
        gateState = (int16_t)constrain((int32_t)zEkf.rejectStreak, 1, INT16_MAX);
        gateState = (int16_t)(-gateState);
    } else if (zEkf.gateRejected) {
        gateState = (int16_t)constrain((int32_t)zEkf.rejectStreak, 1, INT16_MAX);
    }
    DEBUG_SET(DEBUG_Z_EKF, 7, gateState);

    // BARO altitude hold debug (units shown after scaling below):
    // 0: uP (PWM), 1: uI (PWM), 2: u (PWM),
    // 3: vP (cm/s)  = kz * zErr      (deadband only; 0 out of deadband by design)
    // 4: vFF (cm/s) = stickFF gain'd (out of deadband only; 0 in deadband by design)
    // 5: vSet (cm/s), 6: vErr (cm/s),
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
