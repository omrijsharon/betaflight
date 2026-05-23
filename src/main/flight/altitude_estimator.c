#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "flight/altitude_estimator.h"
#include "pg/pg_ids.h"

#define ALTITUDE_ESTIMATOR_HISTORY_MAX 128
#define ALTITUDE_ESTIMATOR_DEFAULT_FLAGS (ALT_EST_CONFIG_DELAYED_FUSION | ALT_EST_CONFIG_ADAPTIVE_R | ALT_EST_CONFIG_BARO_STEP)

typedef struct altitudeState_s {
    float z;
    float v;
    float ba;
    float P[3][3];
} altitudeState_t;

typedef struct altitudeHistory_s {
    timeUs_t timeUs;
    float dt;
    float accelWorldZ;
    altitudeState_t state;
} altitudeHistory_t;

typedef struct altitudeEstimatorRuntime_s {
    altitudeState_t state;
    altitudeHistory_t history[ALTITUDE_ESTIMATOR_HISTORY_MAX];
    uint8_t historyCount;

    bool armed;
    bool hasDatum;
    float baroDatumM;
    float baroOffsetM;

    float accelNoise;
    float biasNoise;
    float biasLimit;
    float baroNoise;
    float innovVarianceFloor;
    float gateSigma;
    float heightRateCutoffHz;

    uint16_t rejectStreak;
    uint16_t recoveryFrames;
    float recoveryRMult;
    uint16_t stepStreak;
    float stepBaroRate;
    float previousBaroM;
    timeUs_t previousBaroTimeUs;
    bool havePreviousBaro;

    float posRateZ;
    float posRateV;
    float posRateA;
    bool posRateInitialized;

    timeUs_t lastTimeUs;
    altitudeEstimatorStatus_t status;
} altitudeEstimatorRuntime_t;

PG_REGISTER_WITH_RESET_TEMPLATE(altitudeEstimatorConfig_t, altitudeEstimatorConfig, PG_ALTITUDE_ESTIMATOR_CONFIG, 0);

PG_RESET_TEMPLATE(altitudeEstimatorConfig_t, altitudeEstimatorConfig,
    .alt_est_flags = ALTITUDE_ESTIMATOR_DEFAULT_FLAGS,
    .alt_est_accel_noise_cms2 = 35,
    .alt_est_accel_bias_noise_cms2 = 2,
    .alt_est_accel_bias_limit_cms2 = 200,
    .alt_est_baro_noise_cm = 150,
    .alt_est_baro_delay_ms = 60,
    .alt_est_innov_var_floor_cm2 = 400,
    .alt_est_history_ms = 300,
    .alt_est_gate_sigma_x10 = 50,
    .alt_est_recovery_start_frames = 30,
    .alt_est_recovery_r_scale_x10 = 500,
    .alt_est_recovery_decay_tc_frames = 20,
    .alt_est_step_innov_thresh_cm = 150,
    .alt_est_step_rate_thresh_cms = 200,
    .alt_est_step_rate_filter_tau_ms = 150,
    .alt_est_step_streak_frames = 5,
    .alt_est_step_offset_alpha_x1000 = 100,
    .alt_est_step_offset_limit_cm = 300,
    .alt_est_height_rate_lpf_hz_x100 = 200,
);

static altitudeEstimatorRuntime_t altEst;

static float metersFromCm(const float cm)
{
    return cm * 0.01f;
}

static float cmFromMeters(const float meters)
{
    return meters * 100.0f;
}

static uint8_t historyLimitFromConfig(void)
{
    const uint16_t historyMs = altitudeEstimatorConfig()->alt_est_history_ms;
    uint16_t samples = (uint16_t)((historyMs * 100u) / 1000u) + 4u;
    samples = constrain(samples, 8, ALTITUDE_ESTIMATOR_HISTORY_MAX);
    return (uint8_t)samples;
}

void altitudeEstimatorUpdateConfig(void)
{
    const altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfig();

    altEst.accelNoise = MAX(0.001f, cfg->alt_est_accel_noise_cms2 * 0.01f);
    altEst.biasNoise = MAX(0.0001f, cfg->alt_est_accel_bias_noise_cms2 * 0.01f);
    altEst.biasLimit = MAX(0.01f, cfg->alt_est_accel_bias_limit_cms2 * 0.01f);
    altEst.baroNoise = MAX(0.01f, cfg->alt_est_baro_noise_cm * 0.01f);
    altEst.innovVarianceFloor = MAX(0.0001f, cfg->alt_est_innov_var_floor_cm2 * 0.0001f);
    altEst.gateSigma = MAX(0.1f, cfg->alt_est_gate_sigma_x10 * 0.1f);
    altEst.heightRateCutoffHz = MAX(0.05f, cfg->alt_est_height_rate_lpf_hz_x100 * 0.01f);
}

static void resetState(const float zM)
{
    memset(&altEst.state, 0, sizeof(altEst.state));
    altEst.state.z = zM;
    altEst.state.P[0][0] = sq(MAX(0.25f, altEst.baroNoise));
    altEst.state.P[1][1] = sq(0.5f);
    altEst.state.P[2][2] = sq(0.25f);

    altEst.historyCount = 0;
    altEst.rejectStreak = 0;
    altEst.recoveryFrames = 0;
    altEst.recoveryRMult = 1.0f;
    altEst.stepStreak = 0;
    altEst.stepBaroRate = 0.0f;
    altEst.havePreviousBaro = false;
    altEst.posRateInitialized = false;
    altEst.status.flags |= ALT_EST_STATUS_RESET;
}

void altitudeEstimatorInit(void)
{
    memset(&altEst, 0, sizeof(altEst));
    altitudeEstimatorUpdateConfig();
    resetState(0.0f);
}

static void constrainState(altitudeState_t *state)
{
    if (state->ba > altEst.biasLimit) {
        state->ba = altEst.biasLimit;
        altEst.status.flags |= ALT_EST_STATUS_BIAS_LIMIT;
    } else if (state->ba < -altEst.biasLimit) {
        state->ba = -altEst.biasLimit;
        altEst.status.flags |= ALT_EST_STATUS_BIAS_LIMIT;
    }
}

static void predictState(altitudeState_t *state, const float accelWorldZ, const float dt)
{
    const float safeDt = constrainf(dt, 0.001f, 0.05f);
    const float accel = accelWorldZ - state->ba;
    const float halfDt2 = 0.5f * sq(safeDt);

    state->z += state->v * safeDt + accel * halfDt2;
    state->v += accel * safeDt;

    const float f00 = 1.0f;
    const float f01 = safeDt;
    const float f02 = -halfDt2;
    const float f10 = 0.0f;
    const float f11 = 1.0f;
    const float f12 = -safeDt;
    const float f20 = 0.0f;
    const float f21 = 0.0f;
    const float f22 = 1.0f;
    float fp[3][3];
    float newP[3][3];

    for (int col = 0; col < 3; col++) {
        fp[0][col] = f00 * state->P[0][col] + f01 * state->P[1][col] + f02 * state->P[2][col];
        fp[1][col] = f10 * state->P[0][col] + f11 * state->P[1][col] + f12 * state->P[2][col];
        fp[2][col] = f20 * state->P[0][col] + f21 * state->P[1][col] + f22 * state->P[2][col];
    }

    for (int row = 0; row < 3; row++) {
        newP[row][0] = fp[row][0] * f00 + fp[row][1] * f10 + fp[row][2] * f20;
        newP[row][1] = fp[row][0] * f01 + fp[row][1] * f11 + fp[row][2] * f21;
        newP[row][2] = fp[row][0] * f02 + fp[row][1] * f12 + fp[row][2] * f22;
    }

    const float accelVar = sq(altEst.accelNoise);
    newP[0][0] += sq(halfDt2) * accelVar;
    newP[0][1] += halfDt2 * safeDt * accelVar;
    newP[1][0] += halfDt2 * safeDt * accelVar;
    newP[1][1] += sq(safeDt) * accelVar;
    newP[2][2] += sq(altEst.biasNoise) * safeDt;

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            state->P[row][col] = 0.5f * (newP[row][col] + newP[col][row]);
        }
    }

    state->P[0][0] = MAX(state->P[0][0], sq(0.02f));
    state->P[1][1] = MAX(state->P[1][1], sq(0.02f));
    state->P[2][2] = MAX(state->P[2][2], sq(0.01f));

    constrainState(state);
}

static void pushHistory(const timeUs_t nowUs, const float dt, const float accelWorldZ)
{
    const uint8_t limit = historyLimitFromConfig();
    if (altEst.historyCount >= limit) {
        memmove(&altEst.history[0], &altEst.history[1], sizeof(altEst.history[0]) * (limit - 1));
        altEst.historyCount = limit - 1;
    }

    altitudeHistory_t *entry = &altEst.history[altEst.historyCount++];
    entry->timeUs = nowUs;
    entry->dt = dt;
    entry->accelWorldZ = accelWorldZ;
    entry->state = altEst.state;
}

static int findHistoryIndex(const timeUs_t targetUs)
{
    if (altEst.historyCount == 0) {
        return -1;
    }

    int bestIndex = 0;
    timeDelta_t bestDelta = ABS(cmpTimeUs(altEst.history[0].timeUs, targetUs));
    for (uint8_t i = 1; i < altEst.historyCount; i++) {
        const timeDelta_t delta = ABS(cmpTimeUs(altEst.history[i].timeUs, targetUs));
        if (delta < bestDelta) {
            bestDelta = delta;
            bestIndex = i;
        }
    }

    const timeDelta_t maxAgeUs = MAX(20000, altitudeEstimatorConfig()->alt_est_history_ms * 1000);
    if (bestDelta > maxAgeUs) {
        altEst.status.flags |= ALT_EST_STATUS_HISTORY_MISS;
    }

    return bestIndex;
}

static float baroRate(const float baroM, const timeUs_t baroTimeUs)
{
    float rate = 0.0f;
    float dt = 0.01f;
    if (altEst.havePreviousBaro) {
        dt = cmpTimeUs(baroTimeUs, altEst.previousBaroTimeUs) * 1e-6f;
        if (dt > 0.001f && dt < 1.0f) {
            rate = (baroM - altEst.previousBaroM) / dt;
        } else {
            dt = 0.01f;
        }
    }

    altEst.previousBaroM = baroM;
    altEst.previousBaroTimeUs = baroTimeUs;
    altEst.havePreviousBaro = true;

    const float tau = altitudeEstimatorConfig()->alt_est_step_rate_filter_tau_ms * 0.001f;
    if (tau <= 0.0f) {
        altEst.stepBaroRate = rate;
    } else {
        const float alpha = constrainf(dt / (tau + dt), 0.0f, 1.0f);
        altEst.stepBaroRate += alpha * (rate - altEst.stepBaroRate);
    }

    return altEst.stepBaroRate;
}

static bool fuseBaro(altitudeState_t *state, const float rawBaroM, const timeUs_t baroTimeUs)
{
    const altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfig();
    const float measuredZ = rawBaroM - altEst.baroDatumM - altEst.baroOffsetM;
    float innov = measuredZ - state->z;
    float rEff = sq(altEst.baroNoise);

    if (cfg->alt_est_flags & ALT_EST_CONFIG_ADAPTIVE_R) {
        rEff *= constrainf(altEst.recoveryRMult, 1.0f, cfg->alt_est_recovery_r_scale_x10 * 0.1f);
    }

    float s = MAX(state->P[0][0] + rEff, altEst.innovVarianceFloor);
    float gate = altEst.gateSigma * sqrtf(s);
    const float rate = baroRate(rawBaroM, baroTimeUs);

    altEst.status.innovationCm = cmFromMeters(innov);
    altEst.status.gateCm = cmFromMeters(gate);
    altEst.status.rEffCm2 = rEff * 10000.0f;
    altEst.status.sCm2 = s * 10000.0f;

    bool reject = fabsf(innov) > gate;
    bool forceFuse = false;

    if (reject) {
        altEst.rejectStreak++;
        altEst.status.flags |= ALT_EST_STATUS_BARO_REJECT;

        if ((cfg->alt_est_flags & ALT_EST_CONFIG_BARO_STEP) &&
            fabsf(cmFromMeters(innov)) > cfg->alt_est_step_innov_thresh_cm &&
            fabsf(cmFromMeters(rate)) > cfg->alt_est_step_rate_thresh_cms) {
            altEst.stepStreak++;
        } else {
            altEst.stepStreak = 0;
        }

        if (altEst.stepStreak >= cfg->alt_est_step_streak_frames) {
            const float limit = metersFromCm(cfg->alt_est_step_offset_limit_cm);
            const float alpha = constrainf(cfg->alt_est_step_offset_alpha_x1000 * 0.001f, 0.0f, 1.0f);
            const float adjustment = constrainf(alpha * innov, -limit, limit);
            altEst.baroOffsetM += adjustment;
            altEst.status.flags |= ALT_EST_STATUS_BARO_STEP;
            altEst.stepStreak = 0;

            innov = (rawBaroM - altEst.baroDatumM - altEst.baroOffsetM) - state->z;
            s = MAX(state->P[0][0] + rEff, altEst.innovVarianceFloor);
            gate = altEst.gateSigma * sqrtf(s);
            reject = fabsf(innov) > gate;
        }

        if (altEst.rejectStreak >= cfg->alt_est_recovery_start_frames) {
            altEst.recoveryFrames++;
            altEst.recoveryRMult = MAX(altEst.recoveryRMult, cfg->alt_est_recovery_r_scale_x10 * 0.1f);
            altEst.status.flags |= ALT_EST_STATUS_RECOVERY;
            forceFuse = true;
        }
    } else {
        altEst.rejectStreak = 0;
        altEst.stepStreak = 0;
    }

    if (reject && !forceFuse) {
        return false;
    }

    if (forceFuse) {
        rEff *= MAX(1.0f, altEst.recoveryRMult);
        s = MAX(state->P[0][0] + rEff, altEst.innovVarianceFloor);
        gate = altEst.gateSigma * sqrtf(s);
    }

    const float k0 = state->P[0][0] / s;
    const float k1 = state->P[1][0] / s;
    const float k2 = state->P[2][0] / s;

    state->z += k0 * innov;
    state->v += k1 * innov;
    state->ba += k2 * innov;

    const float p00 = state->P[0][0];
    const float p01 = state->P[0][1];
    const float p02 = state->P[0][2];

    state->P[0][0] -= k0 * p00;
    state->P[0][1] -= k0 * p01;
    state->P[0][2] -= k0 * p02;
    state->P[1][0] -= k1 * p00;
    state->P[1][1] -= k1 * p01;
    state->P[1][2] -= k1 * p02;
    state->P[2][0] -= k2 * p00;
    state->P[2][1] -= k2 * p01;
    state->P[2][2] -= k2 * p02;

    for (int row = 0; row < 3; row++) {
        for (int col = row + 1; col < 3; col++) {
            const float symmetric = 0.5f * (state->P[row][col] + state->P[col][row]);
            state->P[row][col] = symmetric;
            state->P[col][row] = symmetric;
        }
    }

    constrainState(state);

    altEst.status.flags |= ALT_EST_STATUS_BARO_FUSED;
    altEst.status.innovationCm = cmFromMeters(innov);
    altEst.status.gateCm = cmFromMeters(gate);
    altEst.status.rEffCm2 = rEff * 10000.0f;
    altEst.status.sCm2 = s * 10000.0f;

    return true;
}

static void replayFromHistory(const int index)
{
    if (index < 0 || index >= altEst.historyCount) {
        return;
    }

    for (uint8_t i = index + 1; i < altEst.historyCount; i++) {
        altEst.history[i].state = altEst.history[i - 1].state;
        predictState(&altEst.history[i].state, altEst.history[i].accelWorldZ, altEst.history[i].dt);
    }

    altEst.state = altEst.history[altEst.historyCount - 1].state;
}

static void updateRecoveryDecay(void)
{
    const altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfig();
    if (altEst.recoveryRMult <= 1.0f) {
        altEst.recoveryRMult = 1.0f;
        return;
    }

    const float tc = MAX(1.0f, cfg->alt_est_recovery_decay_tc_frames);
    altEst.recoveryRMult += (1.0f - altEst.recoveryRMult) / tc;
    if (altEst.recoveryRMult < 1.01f) {
        altEst.recoveryRMult = 1.0f;
    }
}

static void updatePositionRate(const float dt)
{
    if (!altEst.posRateInitialized) {
        altEst.posRateZ = altEst.state.z;
        altEst.posRateV = altEst.state.v;
        altEst.posRateA = 0.0f;
        altEst.posRateInitialized = true;
        return;
    }

    const float omega = 2.0f * M_PIf * altEst.heightRateCutoffHz;
    const float e = altEst.state.z - altEst.posRateZ;

    altEst.posRateZ += dt * altEst.posRateV;
    altEst.posRateV += dt * altEst.posRateA;
    altEst.posRateA += dt * ((omega * omega * omega) * e - 3.0f * omega * altEst.posRateA - 3.0f * omega * omega * altEst.posRateV);
}

void altitudeEstimatorUpdate(timeUs_t nowUs, bool armed, bool haveBaro, float baroAltitudeCm, timeUs_t baroTimeUs, bool haveAccel, float accelWorldZ)
{
    altitudeEstimatorUpdateConfig();
    altEst.status.flags = 0;
    altEst.status.timeUs = nowUs;
    altEst.status.baroAltitudeCm = haveBaro ? baroAltitudeCm : 0.0f;
    altEst.status.innovationCm = 0.0f;
    altEst.status.gateCm = 0.0f;
    altEst.status.rEffCm2 = 0.0f;
    altEst.status.sCm2 = 0.0f;
    altEst.status.baroAgeMs = 0.0f;

    if (armed) {
        altEst.status.flags |= ALT_EST_STATUS_ARMED;
    }
    if (haveBaro) {
        altEst.status.flags |= ALT_EST_STATUS_BARO_VALID;
    }

    const float accel = haveAccel ? accelWorldZ : 0.0f;
    float dt = 0.01f;
    if (altEst.lastTimeUs != 0) {
        dt = constrainf(cmpTimeUs(nowUs, altEst.lastTimeUs) * 1e-6f, 0.001f, 0.05f);
    }
    altEst.lastTimeUs = nowUs;

    if (!armed) {
        altEst.armed = false;
        if (haveBaro) {
            altEst.baroDatumM = metersFromCm(baroAltitudeCm);
            altEst.hasDatum = true;
            resetState(0.0f);
        }
        altEst.status.altitudeCm = 0.0f;
        altEst.status.velocityCms = 0.0f;
        altEst.status.positionRateCms = 0.0f;
        altEst.status.accelWorldZCms2 = cmFromMeters(accel);
        altEst.status.accelBiasCms2 = cmFromMeters(altEst.state.ba);
        altEst.status.configuredDelayMs = altitudeEstimatorConfig()->alt_est_baro_delay_ms;
        return;
    }

    if (!altEst.armed) {
        if (haveBaro || !altEst.hasDatum) {
            altEst.baroDatumM = haveBaro ? metersFromCm(baroAltitudeCm) : 0.0f;
            altEst.hasDatum = true;
        }
        altEst.baroOffsetM = 0.0f;
        resetState(0.0f);
        altEst.armed = true;
    }

    predictState(&altEst.state, accel, dt);
    pushHistory(nowUs, dt, accel);

    if (haveBaro && altEst.hasDatum) {
        const timeUs_t delayUs = altitudeEstimatorConfig()->alt_est_baro_delay_ms * 1000u;
        const timeUs_t targetUs = (altitudeEstimatorConfig()->alt_est_flags & ALT_EST_CONFIG_DELAYED_FUSION)
            ? baroTimeUs - delayUs
            : nowUs;
        const int historyIndex = findHistoryIndex(targetUs);
        if (historyIndex >= 0) {
            if (fuseBaro(&altEst.history[historyIndex].state, metersFromCm(baroAltitudeCm), baroTimeUs)) {
                replayFromHistory(historyIndex);
            }
        }
    }

    updateRecoveryDecay();
    updatePositionRate(dt);

    altEst.status.altitudeCm = cmFromMeters(altEst.state.z);
    altEst.status.velocityCms = cmFromMeters(altEst.state.v);
    altEst.status.positionRateCms = cmFromMeters(altEst.posRateV);
    altEst.status.accelWorldZCms2 = cmFromMeters(accel);
    altEst.status.accelBiasCms2 = cmFromMeters(altEst.state.ba);
    altEst.status.baroOffsetCm = cmFromMeters(altEst.baroOffsetM);
    altEst.status.baroAgeMs = haveBaro ? cmpTimeUs(nowUs, baroTimeUs) * 0.001f : 0.0f;
    altEst.status.configuredDelayMs = altitudeEstimatorConfig()->alt_est_baro_delay_ms;
    altEst.status.rejectStreak = altEst.rejectStreak;
    altEst.status.recoveryFrames = altEst.recoveryFrames;
    altEst.status.stepStreak = altEst.stepStreak;
}

const altitudeEstimatorStatus_t *altitudeEstimatorGetStatus(void)
{
    return &altEst.status;
}

int32_t altitudeEstimatorGetAltitudeCm(void)
{
    return lrintf(altEst.status.altitudeCm);
}

int16_t altitudeEstimatorGetVelocityCms(void)
{
    return (int16_t)constrain(lrintf(altEst.status.velocityCms), INT16_MIN, INT16_MAX);
}

float altitudeEstimatorGetAltitudeCmFloat(void)
{
    return altEst.status.altitudeCm;
}

float altitudeEstimatorGetVelocityCmsFloat(void)
{
    return altEst.status.velocityCms;
}

float altitudeEstimatorGetPositionRateCmsFloat(void)
{
    return altEst.status.positionRateCms;
}
