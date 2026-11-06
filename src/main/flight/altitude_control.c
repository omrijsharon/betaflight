#include "platform.h"
#ifdef USE_ALTITUDE_CONTROL
#include <math.h>
#include <string.h>
#include "common/maths.h"
#include "flight/altitude_control.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(altitudeControlConfig_t, altitudeControlConfig, PG_ALTITUDE_CONTROL, 0);
#define ALT_DEFAULT(n, lo, hi, d) .n = d,
PG_RESET_TEMPLATE(altitudeControlConfig_t, altitudeControlConfig, ALT_CONTROL_SETTINGS(ALT_DEFAULT));
#undef ALT_DEFAULT

static altitudeControlStatus_t s;
static bool switchReady, applied;
static float levelHover, integral, prevV, accelFiltered;
static struct { float jerk; uint32_t durationUs; } phases[3];
static unsigned phase;

const altitudeControlStatus_t *altitudeControlStatus(void) { return &s; }
bool altitudeControlActive(void) { return s.mode >= AC_WAIT_CENTER && s.mode <= AC_SPEED; }

// RX mode processing and the altitude task run at different rates. On the
// first mixer tick in ANGLE+ALTITUDE, use the last actual manual collective.
float altitudeControlCollective(float outgoing)
{
    if (!applied) {
        s.output = outgoing;
        levelHover = outgoing * sqrtf(s.tilt);
        applied = true;
        s.reason = AC_OK;
    }
    return s.output;
}

void altitudeControlRelease(uint8_t reason)
{
    s.mode = AC_LOCKED;
    s.reason = reason;
    switchReady = false;
}

// Symmetric jerk ramps with an optional constant-acceleration middle segment.
// The incoming acceleration is already bounded by the same planner limit.
static void capture(float z, float v, float aMax, float jerk)
{
    s.zRef = z;
    s.vRef = v;
    const float sign = v + s.aRef * fabsf(s.aRef) / (2 * jerk) >= 0 ? 1 : -1;
    const float b = sign * s.aRef, u = sign * v;
    const float peak = fminf(aMax, sqrtf(fmaxf(0, jerk * u + 0.5f * b * b)));
    phases[0].jerk = -sign * jerk;
    phases[0].durationUs = lrintf(1e6f * fmaxf(0, (b + peak) / jerk));
    phases[1].jerk = 0;
    phases[1].durationUs = peak > 0 ? lrintf(1e6f * fmaxf(0, (u + (b*b - 2*peak*peak)/(2*jerk)) / peak)) : 0;
    phases[2].jerk = sign * jerk;
    phases[2].durationUs = lrintf(1e6f * peak / jerk);
    phase = 0;
}

static void advanceStop(float dt)
{
    // Integer phase time avoids float subtraction drift during long slow stops.
    uint32_t remainingUs = lrintf(dt * 1e6f);
    while (phase < 3 && remainingUs > 0) {
        const float j = phases[phase].jerk;
        const uint32_t tickUs = MIN(remainingUs, phases[phase].durationUs);
        const float t = tickUs * 1e-6f;
        s.zRef += t * (s.vRef + t * (0.5f*s.aRef + t*j/6));
        s.vRef += t * (s.aRef + 0.5f*t*j);
        s.aRef += t*j;
        phases[phase].durationUs -= tickUs;
        remainingUs -= tickUs;
        if (!phases[phase].durationUs) { phase++; }
    }
    if (phase == 3) { s.vRef = s.aRef = 0; }
}

// A damaged estimate must not reach the mixer, including with -ffast-math.
static bool finiteValue(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

void altitudeControlStep(uint32_t now, bool armed, bool requested, bool allowed, bool healthy,
    int stick, float z, float v, float tilt, float manualCollective, float ceiling)
{
    const uint32_t elapsed = now - s.timeUs;
    healthy = healthy && finiteValue(z) && finiteValue(v) && finiteValue(tilt) &&
        finiteValue(manualCollective) && finiteValue(ceiling) && tilt > 0.5f && tilt <= 1.01f &&
        fabsf(v) < 50 && ceiling >= 0 && ceiling <= 1;
    s.timeUs = now;
    s.z = z;
    s.v = v;
    s.stick = stick;
    s.tilt = tilt;
    s.ceiling = ceiling;
    if (!altitudeControlActive() || !armed || !requested || !allowed || !healthy) {
        s.zRef = z;
        s.vCmd = s.vRef = v;
        s.p = s.i = s.d = s.aRef = 0;
        s.output = manualCollective;
    }
    if (!armed || !requested) {
        s.mode = AC_OFF;
        s.reason = AC_OK;
        switchReady = armed && !requested;
        return;
    }
    if (s.mode == AC_LOCKED) { return; }
    if (!allowed || !healthy || elapsed == 0 || elapsed > 50000) {
        altitudeControlRelease(!allowed ? AC_CONFLICT : !healthy ? AC_ESTIMATE : AC_TIMEOUT);
        return;
    }
    const altitudeControlConfig_t *cfg = altitudeControlConfig();
    const float aMax = 0.01f * cfg->accel_cms2, jerk = 0.01f * cfg->jerk_cms3;
    if (!altitudeControlActive()) {
        if (!switchReady || tilt < 0.85f || fabsf(v) > 0.8f || manualCollective < 0.1f ||
            manualCollective > 0.8f || manualCollective > ceiling) {
            altitudeControlRelease(!switchReady ? AC_SWITCH : AC_ENTRY);
            return;
        }
        switchReady = false;
        applied = false;
        s.mode = AC_WAIT_CENTER;
    }
    if (!applied) {
        s.reason = AC_WAIT_ANGLE;
        levelHover = manualCollective * sqrtf(tilt);
        prevV = s.vCmd = v;
        integral = accelFiltered = s.p = s.i = s.d = s.aRef = 0;
        capture(z, v, aMax, jerk);
        s.output = manualCollective;
        return;
    }
    const float dt = elapsed * 1e-6f, vmax = 0.01f * cfg->speed_cms;
    const float offset = fabsf((float)stick);
    if (offset <= 25 && s.mode != AC_HOLD) {
        if (s.mode == AC_SPEED) { capture(z, v, aMax, jerk); }
        s.mode = AC_HOLD;
    } else if (offset > 40 && s.mode == AC_HOLD) {
        s.mode = AC_SPEED;
    }

    if (s.mode == AC_SPEED) {
        const float u = constrainf((offset - 40) / 460, 0, 1);
        const float desired = copysignf(vmax * u*u * (3 - 2*u), stick);
        const float accel = constrainf(3 * (desired - s.vRef), -aMax, aMax);
        s.aRef += constrainf(accel - s.aRef, -jerk*dt, jerk*dt);
        s.vRef += s.aRef * dt;
        s.zRef = z; // no height feedback during stick speed control
        s.vCmd = s.vRef;
    } else {
        advanceStop(dt);
        const float bound = fmaxf(vmax, fabsf(s.vRef));
        s.vCmd = constrainf(s.vRef + 0.01f * (cfg->z_p*(s.zRef-z) + cfg->z_d*(s.vRef-v)), -bound, bound);
    }
    accelFiltered += ((v-prevV)/dt - accelFiltered) * dt / (0.12f + dt);
    prevV = v;
    const float error = s.vCmd - v;
    s.p = 0.01f * cfg->v_p * error;
    s.i = integral;
    s.d = -0.01f * cfg->v_d * accelFiltered;
    const float acceleration = s.aRef + s.p + s.i + s.d;
    const float gravity = 9.80665f;
    const float squared = levelHover*levelHover * (1 + acceleration/gravity) / tilt;
    s.output = sqrtf(constrainf(squared, 0, ceiling*ceiling));
    if (!((squared >= ceiling*ceiling && error > 0) || (squared <= 0 && error < 0))) {
        integral = constrainf(integral + 0.01f * cfg->v_i * error * dt, -4, 4);
    }
}
#endif
