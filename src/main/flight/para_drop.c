/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include "flight/para_drop.h"

#if defined(USE_BARO) && defined(USE_SERVOS)

#include <math.h>
#include <string.h>

#include "build/debug.h"
#include "drivers/pwm_output.h"
#include "pg/pg_ids.h"
#include "rx/rx.h"
#include "sensors/barometer.h"

PG_REGISTER_WITH_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig, PG_PARA_DROP_CONFIG, 0);

PG_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig,
    .altitudeMeters = 0,
    .servoChannel = 0,
    .holdTimeSec = 1,
);

typedef struct paraDropState_s {
    bool referenceLatched;
    bool triggered;
    bool holdActive;
    float referenceAltitudeMeters;
    timeUs_t holdStartTimeUs;
} paraDropState_t;

static paraDropState_t paraDropState;

enum {
    PARA_DROP_DEBUG_STATUS_BARO_READY        = (1 << 0),
    PARA_DROP_DEBUG_STATUS_REFERENCE_LATCHED = (1 << 1),
    PARA_DROP_DEBUG_STATUS_HOLD_ACTIVE       = (1 << 2),
    PARA_DROP_DEBUG_STATUS_TRIGGERED         = (1 << 3),
};

static bool paraDropHasValidServoChannel(void)
{
    return paraDropConfig()->servoChannel > 0 && paraDropConfig()->servoChannel <= MAX_SUPPORTED_SERVOS;
}

bool paraDropIsEnabled(void)
{
    return paraDropConfig()->altitudeMeters != 0 && paraDropHasValidServoChannel();
}

bool paraDropRequiresServoOutput(void)
{
    return paraDropIsEnabled();
}

void paraDropReset(void)
{
    memset(&paraDropState, 0, sizeof(paraDropState));
}

void paraDropInit(void)
{
    paraDropReset();
}

static uint8_t paraDropServoIndex(void)
{
    return paraDropConfig()->servoChannel - 1U;
}

static float paraDropCurrentAltitudeMeters(void)
{
    return baro.altitude / 100.0f;
}

static float paraDropThresholdMeters(void)
{
    return paraDropConfig()->altitudeMeters;
}

static timeUs_t paraDropHoldTimeUs(void)
{
    return (timeUs_t)paraDropConfig()->holdTimeSec * 1000000;
}

static int16_t paraDropClampToDebug16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }

    if (value < INT16_MIN) {
        return INT16_MIN;
    }

    return (int16_t)value;
}

static int16_t paraDropClampTimeMsToDebug16(timeDelta_t valueUs)
{
    return paraDropClampToDebug16(valueUs / 1000);
}

static bool paraDropShouldTrigger(float relativeAltitudeMeters)
{
    const float thresholdMeters = paraDropThresholdMeters();

    if (thresholdMeters >= 0.0f) {
        return relativeAltitudeMeters >= thresholdMeters;
    }

    return relativeAltitudeMeters <= thresholdMeters;
}

static void paraDropWriteOutput(uint16_t pwmValue)
{
    if (!paraDropHasValidServoChannel()) {
        return;
    }

    pwmWriteServo(paraDropServoIndex(), pwmValue);
}

static void paraDropUpdateDebug(timeUs_t currentTimeUs, uint16_t pwmValue, float relativeAltitudeMeters, float currentAltitudeMeters, bool baroReady)
{
    uint8_t status = 0;

    if (baroReady) {
        status |= PARA_DROP_DEBUG_STATUS_BARO_READY;
    }
    if (paraDropState.referenceLatched) {
        status |= PARA_DROP_DEBUG_STATUS_REFERENCE_LATCHED;
    }
    if (paraDropState.holdActive) {
        status |= PARA_DROP_DEBUG_STATUS_HOLD_ACTIVE;
    }
    if (paraDropState.triggered) {
        status |= PARA_DROP_DEBUG_STATUS_TRIGGERED;
    }

    DEBUG_SET(DEBUG_PARA_DROP, 0, pwmValue);
    DEBUG_SET(DEBUG_PARA_DROP, 1, paraDropClampToDebug16(lrintf(relativeAltitudeMeters)));
    DEBUG_SET(DEBUG_PARA_DROP, 2, paraDropClampToDebug16(lrintf(paraDropThresholdMeters())));
    DEBUG_SET(DEBUG_PARA_DROP, 3, paraDropState.holdActive ? paraDropClampTimeMsToDebug16(cmpTimeUs(currentTimeUs, paraDropState.holdStartTimeUs)) : 0);
    DEBUG_SET(DEBUG_PARA_DROP, 4, paraDropClampTimeMsToDebug16((timeDelta_t)paraDropHoldTimeUs()));
    DEBUG_SET(DEBUG_PARA_DROP, 5, status);
    DEBUG_SET(DEBUG_PARA_DROP, 6, paraDropClampToDebug16(lrintf(paraDropState.referenceAltitudeMeters)));
    DEBUG_SET(DEBUG_PARA_DROP, 7, paraDropClampToDebug16(lrintf(currentAltitudeMeters)));
}

void paraDropUpdate(timeUs_t currentTimeUs)
{
    float currentAltitudeMeters = 0.0f;
    float relativeAltitudeMeters = 0.0f;
    uint16_t outputPwm = PWM_RANGE_MIN;
    const bool baroReady = isBaroReady();

    if (!paraDropIsEnabled()) {
        paraDropUpdateDebug(currentTimeUs, outputPwm, relativeAltitudeMeters, currentAltitudeMeters, baroReady);
        return;
    }

    if (!baroReady) {
        paraDropWriteOutput(outputPwm);
        paraDropUpdateDebug(currentTimeUs, outputPwm, relativeAltitudeMeters, currentAltitudeMeters, baroReady);
        return;
    }

    currentAltitudeMeters = paraDropCurrentAltitudeMeters();

    if (!paraDropState.referenceLatched) {
        paraDropState.referenceAltitudeMeters = currentAltitudeMeters;
        paraDropState.referenceLatched = true;
    }

    relativeAltitudeMeters = currentAltitudeMeters - paraDropState.referenceAltitudeMeters;

    if (!paraDropState.triggered) {
        if (!paraDropShouldTrigger(relativeAltitudeMeters)) {
            paraDropState.holdActive = false;
        } else if (!paraDropState.holdActive) {
            paraDropState.holdActive = true;
            paraDropState.holdStartTimeUs = currentTimeUs;
        } else if (cmpTimeUs(currentTimeUs, paraDropState.holdStartTimeUs) >= (timeDelta_t)paraDropHoldTimeUs()) {
            paraDropState.triggered = true;
            paraDropState.holdActive = false;
        }
    }

    outputPwm = paraDropState.triggered ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    paraDropWriteOutput(outputPwm);
    paraDropUpdateDebug(currentTimeUs, outputPwm, relativeAltitudeMeters, currentAltitudeMeters, baroReady);
}

#else

#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig, PG_PARA_DROP_CONFIG, 0);

PG_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig,
    .altitudeMeters = 0,
    .servoChannel = 0,
    .holdTimeSec = 1,
);

void paraDropInit(void)
{
}

void paraDropReset(void)
{
}

void paraDropUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

bool paraDropIsEnabled(void)
{
    return false;
}

bool paraDropRequiresServoOutput(void)
{
    return false;
}

#endif
