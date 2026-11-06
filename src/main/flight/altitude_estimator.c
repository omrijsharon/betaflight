#include <math.h>
#include <string.h>
#include "platform.h"
#include "common/maths.h"
#include "flight/altitude_estimator.h"
#include "pg/pg_ids.h"

// 250 ms at the 100 Hz altitude task needs 26 states plus an older anchor.
// Allow scheduling headroom without reserving the old one-second buffer.
#define ALTITUDE_ESTIMATOR_HISTORY_MAX 32
#define MAX_MEASUREMENT_AGE_US 250000

typedef struct {
    float z, v, ba;
    float P[3][3];
} altitudeState_t;
typedef struct {
    timeUs_t timeUs;
    float accel;
    altitudeState_t state;
} altitudeHistory_t;
typedef struct {
    altitudeState_t state;
    altitudeHistory_t history[ALTITUDE_ESTIMATOR_HISTORY_MAX];
    unsigned historyCount;
    bool armed, hasDatum, haveTime, haveBaroTime;
    timeUs_t lastTimeUs, lastBaroTimeUs;
    float datum, accelNoise, biasNoise, baroNoise, gateSigma;
    uint16_t rejectStreak;
    altitudeEstimatorStatus_t status;
} altitudeEstimatorRuntime_t;

PG_REGISTER_WITH_RESET_TEMPLATE(altitudeEstimatorConfig_t, altitudeEstimatorConfig, PG_ALTITUDE_ESTIMATOR_CONFIG, 1);

PG_RESET_TEMPLATE(altitudeEstimatorConfig_t, altitudeEstimatorConfig,
    .alt_est_flags = ALT_EST_CONFIG_DELAYED_FUSION,
    .alt_est_accel_noise_cms2 = 35,
    .alt_est_accel_bias_noise_cms2 = 2,
    .alt_est_accel_bias_limit_cms2 = 200,
    .alt_est_baro_noise_cm = 150,
    .alt_est_baro_delay_ms = 60,
    .alt_est_history_ms = 250,
    .alt_est_gate_sigma_x10 = 50,
);

static altitudeEstimatorRuntime_t altEst;

// Firmware uses -ffast-math: libc isfinite may be optimized to true.
// Inspect IEEE-754 exponent bits without floating-point comparisons instead.
static bool finiteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

void altitudeEstimatorUpdateConfig(void)
{
    const altitudeEstimatorConfig_t *c = altitudeEstimatorConfig();
    altEst.accelNoise = MAX(0.001f, c->alt_est_accel_noise_cms2 * 0.01f);
    altEst.biasNoise = MAX(0.0001f, c->alt_est_accel_bias_noise_cms2 * 0.01f);
    altEst.baroNoise = MAX(0.01f, c->alt_est_baro_noise_cm * 0.01f);
    altEst.gateSigma = MAX(0.1f, c->alt_est_gate_sigma_x10 * 0.1f);
}

static void resetState(void)
{
    memset(&altEst.state, 0, sizeof(altEst.state));
    // Same priors as sim.vertical.VerticalEstimator (metres, seconds).
    altEst.state.P[0][0] = 4.0f;
    altEst.state.P[1][1] = 1.0f;
    altEst.state.P[2][2] = 0.25f;
    altEst.historyCount = 0;
    altEst.rejectStreak = 0;
    altEst.status.flags |= ALT_EST_STATUS_RESET;
}

void altitudeEstimatorInit(void)
{
    memset(&altEst, 0, sizeof(altEst));
    altitudeEstimatorUpdateConfig();
    resetState();
}

static void predictState(altitudeState_t *s, float accel, float dt)
{
    const float h = 0.5f * dt * dt;
    s->z += s->v * dt + (accel - s->ba) * h;
    s->v += (accel - s->ba) * dt;
    const float F[3][3] = {{1, dt, -h}, {0, 1, -dt}, {0, 0, 1}};
    const float B[3] = {h, dt, 0};
    float fp[3][3] = {{0}}, p[3][3] = {{0}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) fp[i][j] += F[i][k] * s->P[k][j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) p[i][j] += fp[i][k] * F[j][k]; // F transpose
            p[i][j] += B[i] * B[j] * sq(altEst.accelNoise);
        }
    }
    p[2][2] += sq(altEst.biasNoise) * dt;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) s->P[i][j] = 0.5f * (p[i][j] + p[j][i]);
    }
}

static void pushHistory(timeUs_t nowUs, float accel)
{
    const timeDelta_t age = MIN(MAX_MEASUREMENT_AGE_US, (timeDelta_t)altitudeEstimatorConfig()->alt_est_history_ms * 1000);
    while (altEst.historyCount > 1 &&
        (altEst.historyCount == ALTITUDE_ESTIMATOR_HISTORY_MAX ||
         cmpTimeUs(nowUs, altEst.history[1].timeUs) > age)) {
        memmove(altEst.history, altEst.history + 1, (--altEst.historyCount) * sizeof(altEst.history[0]));
    }
    altEst.history[altEst.historyCount++] = (altitudeHistory_t){nowUs, accel, altEst.state};
}

static void rejectMeasurement(uint32_t reason)
{
    altEst.status.flags |= ALT_EST_STATUS_BARO_REJECT | reason;
    if (altEst.rejectStreak < UINT16_MAX) altEst.rejectStreak++;
}

static bool fuseBaro(altitudeState_t *s, float baroAltitudeCm)
{
    const float r = sq(altEst.baroNoise);
    const float innovation = baroAltitudeCm * 0.01f - altEst.datum - s->z;
    const float variance = s->P[0][0] + r;
    altEst.status.innovationCm = innovation * 100;
    altEst.status.gateCm = altEst.gateSigma * sqrtf(variance) * 100;
    altEst.status.rEffCm2 = r * 10000;
    altEst.status.sCm2 = variance * 10000;
    if (!finiteFloat(variance) || variance <= 0 || sq(innovation) > sq(altEst.gateSigma) * variance) {
        rejectMeasurement(0);
        return false;
    }
    float K[3], A[3][3], ap[3][3] = {{0}}, p[3][3] = {{0}};
    for (int i = 0; i < 3; i++) {
        K[i] = s->P[i][0] / variance;
        for (int j = 0; j < 3; j++) A[i][j] = (i == j ? 1.0f : 0.0f) - (j == 0 ? K[i] : 0.0f);
    }
    s->z += K[0] * innovation;
    s->v += K[1] * innovation;
    s->ba += K[2] * innovation;
    // Joseph form: (I-KH) P (I-KH)^T + K R K^T.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) ap[i][j] += A[i][k] * s->P[k][j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) p[i][j] += ap[i][k] * A[j][k];
            p[i][j] += K[i] * r * K[j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) s->P[i][j] = 0.5f * (p[i][j] + p[j][i]);
    }
    altEst.rejectStreak = 0;
    altEst.status.flags |= ALT_EST_STATUS_BARO_FUSED;
    return true;
}

static void correctBaro(float baroAltitudeCm, timeUs_t baroTimeUs, timeUs_t nowUs)
{
    // Measurement timestamps are effective sample times, not poll times.
    const timeDelta_t age = cmpTimeUs(nowUs, baroTimeUs);
    if (age < 0 || (altEst.haveBaroTime && cmpTimeUs(baroTimeUs, altEst.lastBaroTimeUs) <= 0)) {
        rejectMeasurement(ALT_EST_STATUS_BAD_TIME);
        return;
    }
    altEst.haveBaroTime = true;
    altEst.lastBaroTimeUs = baroTimeUs; // Consume gated/stale samples too.
    if (age > MAX_MEASUREMENT_AGE_US) {
        rejectMeasurement(ALT_EST_STATUS_STALE);
        return;
    }
    const timeUs_t target = (altitudeEstimatorConfig()->alt_est_flags & ALT_EST_CONFIG_DELAYED_FUSION) ? baroTimeUs : nowUs;
    if (!altEst.historyCount || cmpTimeUs(target, altEst.history[0].timeUs) < 0) {
        rejectMeasurement(ALT_EST_STATUS_HISTORY_MISS);
        return;
    }
    unsigned index = altEst.historyCount - 1;
    while (index && cmpTimeUs(altEst.history[index].timeUs, target) > 0) index--;
    altitudeState_t corrected = altEst.history[index].state;
    const bool fractional = cmpTimeUs(target, altEst.history[index].timeUs) > 0;
    if (fractional) {
        predictState(&corrected, altEst.history[index + 1].accel, cmpTimeUs(target, altEst.history[index].timeUs) * 1e-6f);
    }
    if (!fuseBaro(&corrected, baroAltitudeCm)) return;
    // For a fractional correction, retain the corrected anchor at the exact
    // measurement time; the consumed-time guard prevents rewinding before it.
    altEst.history[index].timeUs = target;
    altEst.history[index].state = corrected;
    timeUs_t previous = target;
    for (unsigned i = index + 1; i < altEst.historyCount; i++) {
        predictState(&corrected, altEst.history[i].accel, cmpTimeUs(altEst.history[i].timeUs, previous) * 1e-6f);
        altEst.history[i].state = corrected;
        previous = altEst.history[i].timeUs;
    }
    altEst.state = corrected;
}

void altitudeEstimatorUpdate(timeUs_t nowUs, bool armed, bool haveBaro, float baroAltitudeCm, timeUs_t baroTimeUs, bool haveAccel, float accelWorldZ)
{
    altitudeEstimatorUpdateConfig();
    altEst.status.flags = armed ? ALT_EST_STATUS_ARMED : 0;
    altEst.status.timeUs = nowUs;
    altEst.status.innovationCm = altEst.status.gateCm = 0;
    altEst.status.rEffCm2 = altEst.status.sCm2 = 0;
    altEst.status.configuredDelayMs = altitudeEstimatorConfig()->alt_est_baro_delay_ms;
    if (haveBaro && !finiteFloat(baroAltitudeCm)) {
        altEst.status.flags |= ALT_EST_STATUS_INVALID_INPUT;
        haveBaro = false;
    }
    if (haveBaro) {
        altEst.status.baroAltitudeCm = baroAltitudeCm;
        altEst.status.flags |= ALT_EST_STATUS_BARO_VALID;
    }
    const bool validAccel = haveAccel && finiteFloat(accelWorldZ);
    if (!validAccel) altEst.status.flags |= ALT_EST_STATUS_INVALID_INPUT;
    const float accel = validAccel ? accelWorldZ : 0;
    altEst.status.accelWorldZCms2 = accel * 100;
    const timeDelta_t elapsed = altEst.haveTime ? cmpTimeUs(nowUs, altEst.lastTimeUs) : 0;
    const bool validTime = altEst.haveTime && elapsed > 0 && elapsed <= 200000;
    if (altEst.haveTime && elapsed <= 0) {
        altEst.status.flags |= ALT_EST_STATUS_BAD_TIME;
        return; // Do not move the clock backwards or integrate duplicate calls.
    }
    altEst.haveTime = true;
    altEst.lastTimeUs = nowUs;
    if (!armed) {
        altEst.armed = false;
        if (haveBaro && cmpTimeUs(nowUs, baroTimeUs) >= 0 && cmpTimeUs(nowUs, baroTimeUs) <= MAX_MEASUREMENT_AGE_US &&
            (!altEst.haveBaroTime || cmpTimeUs(baroTimeUs, altEst.lastBaroTimeUs) > 0)) {
            altEst.datum = baroAltitudeCm * 0.01f;
            altEst.hasDatum = true;
            altEst.haveBaroTime = true;
            altEst.lastBaroTimeUs = baroTimeUs;
        }
        resetState();
    } else if (!altEst.armed) {
        // Retain the last disarmed datum; do not silently move the reference.
        resetState();
        altEst.armed = true;
        pushHistory(nowUs, accel);
    } else if (validTime && validAccel) {
        predictState(&altEst.state, accel, elapsed * 1e-6f);
        pushHistory(nowUs, accel);
        if (haveBaro && altEst.hasDatum) correctBaro(baroAltitudeCm, baroTimeUs, nowUs);
    } else {
        // A gap cannot be integrated using a fabricated/clamped time interval.
        altEst.status.flags |= validTime ? ALT_EST_STATUS_INVALID_INPUT : ALT_EST_STATUS_BAD_TIME;
        altEst.historyCount = 0;
        pushHistory(nowUs, accel);
    }
    if (!altEst.hasDatum) altEst.status.flags |= ALT_EST_STATUS_NO_DATUM;
    altEst.status.altitudeCm = altEst.state.z * 100;
    altEst.status.velocityCms = altEst.state.v * 100;
    altEst.status.accelBiasCms2 = altEst.state.ba * 100;
    altEst.status.baroAgeMs = altEst.haveBaroTime ? cmpTimeUs(nowUs, altEst.lastBaroTimeUs) * 0.001f : -1;
    if (altEst.status.baroAgeMs > MAX_MEASUREMENT_AGE_US * 0.001f) altEst.status.flags |= ALT_EST_STATUS_STALE;
    altEst.status.rejectStreak = altEst.rejectStreak;
    // Bias threshold is diagnostic, not an unmodelled state clamp.
    if (fabsf(altEst.status.accelBiasCms2) > altitudeEstimatorConfig()->alt_est_accel_bias_limit_cms2)
        altEst.status.flags |= ALT_EST_STATUS_BIAS_LIMIT;
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
