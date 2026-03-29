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

#include "common/printf.h"
#include "build/debug.h"
#include "drivers/pinio.h"
#include "drivers/pwm_output.h"
#include "pg/pg_ids.h"
#include "rx/rx.h"
#include "sensors/barometer.h"

PG_REGISTER_WITH_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig, PG_PARA_DROP_CONFIG, 2);

PG_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig,
    .aslThresholdMeters = 0,
    .servoChannel = 0,
    .holdTimeMs = 25,
    .aboveHoldTimeSec = 1,
    .indicatorPinio = 0,
    .indicatorBlinkHz = 5,
    .activationKey = { 0 },
);

typedef struct paraDropState_s {
    enum {
        PARA_DROP_STATE_WAIT_ABOVE_THRESHOLD = 0,
        PARA_DROP_STATE_ARMED_WAIT_BELOW,
        PARA_DROP_STATE_TRIGGERED,
    } state;
    bool downwardHoldActive;
    timeUs_t downwardHoldStartTimeUs;
    bool aboveHoldActive;
    timeUs_t aboveHoldStartTimeUs;
} paraDropState_t;

static paraDropState_t paraDropState;

enum {
    PARA_DROP_DEBUG_STATUS_BARO_READY  = (1 << 0),
    PARA_DROP_DEBUG_STATUS_HOLD_ACTIVE = (1 << 1),
    PARA_DROP_DEBUG_STATUS_TRIGGERED   = (1 << 2),
    PARA_DROP_DEBUG_STATUS_LED_ENABLED = (1 << 3),
    PARA_DROP_DEBUG_STATUS_LED_ON      = (1 << 4),
    PARA_DROP_DEBUG_STATUS_ARMED       = (1 << 5),
    PARA_DROP_DEBUG_STATUS_ABOVE_HOLD_ACTIVE = (1 << 6),
    PARA_DROP_DEBUG_STATUS_ACTIVATION_VALID = (1 << 7),
};

#define PARA_DROP_ACTIVATION_PRODUCT_STRING "PARADROP_V1_4"
#define PARA_DROP_FNV1A64_OFFSET_BASIS UINT64_C(0xcbf29ce484222325)
#define PARA_DROP_FNV1A64_PRIME        UINT64_C(0x100000001b3)

static bool paraDropHasValidServoChannel(void)
{
    return paraDropConfig()->servoChannel > 0 && paraDropConfig()->servoChannel <= MAX_SUPPORTED_SERVOS;
}

static bool paraDropHasValidIndicatorPinio(void)
{
    return paraDropConfig()->indicatorPinio > 0 && paraDropConfig()->indicatorPinio <= PINIO_COUNT;
}

static uint64_t paraDropFnv1a64Update(uint64_t hash, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        hash ^= data[i];
        hash *= PARA_DROP_FNV1A64_PRIME;
    }

    return hash;
}

static char paraDropHexCharToLower(char c)
{
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 'a';
    }

    return c;
}

static bool paraDropActivationKeyMatchesExpected(void)
{
    char mcuIdString[25];
    char expectedActivationKey[PARA_DROP_ACTIVATION_KEY_LENGTH + 1];
    const char *storedActivationKey = paraDropConfig()->activationKey;
    uint64_t hash = PARA_DROP_FNV1A64_OFFSET_BASIS;

    if (strlen(storedActivationKey) != PARA_DROP_ACTIVATION_KEY_LENGTH) {
        return false;
    }

    tfp_sprintf(mcuIdString, "%08x%08x%08x", U_ID_0, U_ID_1, U_ID_2);

    hash = paraDropFnv1a64Update(hash, (const uint8_t *)PARA_DROP_ACTIVATION_PRODUCT_STRING, strlen(PARA_DROP_ACTIVATION_PRODUCT_STRING));
    hash = paraDropFnv1a64Update(hash, (const uint8_t *)mcuIdString, strlen(mcuIdString));

    for (uint8_t i = 0; i < PARA_DROP_ACTIVATION_KEY_LENGTH; i++) {
        const uint8_t nibbleShift = (PARA_DROP_ACTIVATION_KEY_LENGTH - 1U - i) * 4U;
        const uint8_t nibble = (hash >> nibbleShift) & 0x0FU;
        expectedActivationKey[i] = (nibble < 10U) ? ('0' + nibble) : ('a' + nibble - 10U);
    }
    expectedActivationKey[PARA_DROP_ACTIVATION_KEY_LENGTH] = '\0';

    for (uint8_t i = 0; i < PARA_DROP_ACTIVATION_KEY_LENGTH; i++) {
        if (paraDropHexCharToLower(storedActivationKey[i]) != expectedActivationKey[i]) {
            return false;
        }
    }

    return true;
}

static bool paraDropActivationIsValid(void)
{
    return paraDropActivationKeyMatchesExpected();
}

bool paraDropIsEnabled(void)
{
    return paraDropConfig()->aslThresholdMeters != 0
        && paraDropHasValidServoChannel()
        && paraDropActivationIsValid();
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

static float paraDropCurrentAltitudeAslMeters(void)
{
    return getBaroAltitudeAsl() / 100.0f;
}

static float paraDropCurrentAltitudeMeters(void)
{
    return getBaroAltitude() / 100.0f;
}

static float paraDropCurrentGroundAltitudeMeters(void)
{
    return getBaroGroundAltitude() / 100.0f;
}

static float paraDropThresholdMeters(void)
{
    return paraDropConfig()->aslThresholdMeters;
}

static timeUs_t paraDropHoldTimeUs(void)
{
    return (timeUs_t)paraDropConfig()->holdTimeMs * 1000;
}

static timeUs_t paraDropAboveHoldTimeUs(void)
{
    return (timeUs_t)paraDropConfig()->aboveHoldTimeSec * 1000000U;
}

static timeUs_t paraDropIndicatorBlinkPeriodUs(void)
{
    return 1000000U / paraDropConfig()->indicatorBlinkHz;
}

static bool paraDropIsTriggered(void)
{
    return paraDropState.state == PARA_DROP_STATE_TRIGGERED;
}

static bool paraDropIsArmedForCrossing(void)
{
    return paraDropState.state != PARA_DROP_STATE_WAIT_ABOVE_THRESHOLD;
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

static bool paraDropIsAtOrBelowThreshold(float currentAltitudeAslMeters)
{
    return currentAltitudeAslMeters <= paraDropThresholdMeters();
}

static timeDelta_t paraDropActiveHoldElapsedUs(timeUs_t currentTimeUs)
{
    if (paraDropState.aboveHoldActive) {
        return cmpTimeUs(currentTimeUs, paraDropState.aboveHoldStartTimeUs);
    }

    if (paraDropState.downwardHoldActive) {
        return cmpTimeUs(currentTimeUs, paraDropState.downwardHoldStartTimeUs);
    }

    return 0;
}

static timeUs_t paraDropActiveHoldTargetUs(void)
{
    if (paraDropState.aboveHoldActive) {
        return paraDropAboveHoldTimeUs();
    }

    if (paraDropState.downwardHoldActive) {
        return paraDropHoldTimeUs();
    }

    return 0;
}

static void paraDropWriteOutput(uint16_t pwmValue)
{
    if (!paraDropHasValidServoChannel()) {
        return;
    }

    pwmWriteServo(paraDropServoIndex(), pwmValue);
}

static bool paraDropUpdateIndicator(timeUs_t currentTimeUs, bool enabled)
{
    bool ledOn = false;

    if (!paraDropHasValidIndicatorPinio()) {
        return false;
    }

    if (enabled) {
        if (paraDropIsTriggered()) {
            ledOn = true;
        } else {
            const timeUs_t blinkPeriodUs = paraDropIndicatorBlinkPeriodUs();
            const timeUs_t effectiveBlinkPeriodUs = (paraDropState.state == PARA_DROP_STATE_WAIT_ABOVE_THRESHOLD) ? (blinkPeriodUs * 4U) : blinkPeriodUs;

            ledOn = (currentTimeUs % effectiveBlinkPeriodUs) < (effectiveBlinkPeriodUs / 2);
        }
    }

    pinioSet(paraDropConfig()->indicatorPinio - 1U, ledOn);
    return ledOn;
}

static void paraDropUpdateDebug(timeUs_t currentTimeUs, uint16_t pwmValue, float currentAltitudeAslMeters, float currentAltitudeMeters, float groundAltitudeMeters, bool baroReady, bool ledOn, bool activationValid)
{
    uint8_t status = 0;

    if (baroReady) {
        status |= PARA_DROP_DEBUG_STATUS_BARO_READY;
    }
    if (paraDropHasValidIndicatorPinio()) {
        status |= PARA_DROP_DEBUG_STATUS_LED_ENABLED;
    }
    if (ledOn) {
        status |= PARA_DROP_DEBUG_STATUS_LED_ON;
    }
    if (activationValid) {
        status |= PARA_DROP_DEBUG_STATUS_ACTIVATION_VALID;
        if (paraDropState.downwardHoldActive) {
            status |= PARA_DROP_DEBUG_STATUS_HOLD_ACTIVE;
        }
        if (paraDropIsTriggered()) {
            status |= PARA_DROP_DEBUG_STATUS_TRIGGERED;
        }
        if (paraDropIsArmedForCrossing()) {
            status |= PARA_DROP_DEBUG_STATUS_ARMED;
        }
        if (paraDropState.aboveHoldActive) {
            status |= PARA_DROP_DEBUG_STATUS_ABOVE_HOLD_ACTIVE;
        }
    }

    DEBUG_SET(DEBUG_PARA_DROP, 0, pwmValue);
    DEBUG_SET(DEBUG_PARA_DROP, 1, paraDropClampToDebug16(lrintf(currentAltitudeAslMeters)));
    DEBUG_SET(DEBUG_PARA_DROP, 2, paraDropClampToDebug16(lrintf(paraDropThresholdMeters())));
    DEBUG_SET(DEBUG_PARA_DROP, 3, paraDropClampTimeMsToDebug16(paraDropActiveHoldElapsedUs(currentTimeUs)));
    DEBUG_SET(DEBUG_PARA_DROP, 4, paraDropClampTimeMsToDebug16((timeDelta_t)paraDropActiveHoldTargetUs()));
    DEBUG_SET(DEBUG_PARA_DROP, 5, status);
    DEBUG_SET(DEBUG_PARA_DROP, 6, paraDropClampToDebug16(lrintf(currentAltitudeMeters)));
    DEBUG_SET(DEBUG_PARA_DROP, 7, paraDropClampToDebug16(lrintf(groundAltitudeMeters)));
}

void paraDropUpdate(timeUs_t currentTimeUs)
{
    float currentAltitudeAslMeters = 0.0f;
    float currentAltitudeMeters = 0.0f;
    float groundAltitudeMeters = 0.0f;
    uint16_t outputPwm = PWM_RANGE_MIN;
    const bool baroReady = isBaroReady();
    const bool baroCalibrated = baroIsCalibrated();
    const bool activationValid = paraDropActivationIsValid();
    bool ledOn = false;

    if (!paraDropIsEnabled()) {
        ledOn = paraDropUpdateIndicator(currentTimeUs, false);
        paraDropUpdateDebug(currentTimeUs, outputPwm, currentAltitudeAslMeters, currentAltitudeMeters, groundAltitudeMeters, baroReady && baroCalibrated, ledOn, activationValid);
        return;
    }

    if (!baroReady || !baroCalibrated) {
        paraDropWriteOutput(outputPwm);
        ledOn = paraDropUpdateIndicator(currentTimeUs, true);
        paraDropUpdateDebug(currentTimeUs, outputPwm, currentAltitudeAslMeters, currentAltitudeMeters, groundAltitudeMeters, baroReady && baroCalibrated, ledOn, activationValid);
        return;
    }

    currentAltitudeAslMeters = paraDropCurrentAltitudeAslMeters();
    currentAltitudeMeters = paraDropCurrentAltitudeMeters();
    groundAltitudeMeters = paraDropCurrentGroundAltitudeMeters();

    switch (paraDropState.state) {
    case PARA_DROP_STATE_WAIT_ABOVE_THRESHOLD:
        paraDropState.downwardHoldActive = false;
        if (currentAltitudeAslMeters > paraDropThresholdMeters()) {
            if (!paraDropState.aboveHoldActive) {
                paraDropState.aboveHoldActive = true;
                paraDropState.aboveHoldStartTimeUs = currentTimeUs;
            } else if (cmpTimeUs(currentTimeUs, paraDropState.aboveHoldStartTimeUs) >= (timeDelta_t)paraDropAboveHoldTimeUs()) {
                paraDropState.state = PARA_DROP_STATE_ARMED_WAIT_BELOW;
                paraDropState.aboveHoldActive = false;
            }
        } else {
            paraDropState.aboveHoldActive = false;
        }
        break;

    case PARA_DROP_STATE_ARMED_WAIT_BELOW:
        paraDropState.aboveHoldActive = false;
        if (!paraDropIsAtOrBelowThreshold(currentAltitudeAslMeters)) {
            paraDropState.downwardHoldActive = false;
        } else if (!paraDropState.downwardHoldActive) {
            paraDropState.downwardHoldActive = true;
            paraDropState.downwardHoldStartTimeUs = currentTimeUs;
        } else if (cmpTimeUs(currentTimeUs, paraDropState.downwardHoldStartTimeUs) >= (timeDelta_t)paraDropHoldTimeUs()) {
            paraDropState.state = PARA_DROP_STATE_TRIGGERED;
            paraDropState.downwardHoldActive = false;
        }
        break;

    case PARA_DROP_STATE_TRIGGERED:
        paraDropState.aboveHoldActive = false;
        paraDropState.downwardHoldActive = false;
        break;
    }

    outputPwm = paraDropIsTriggered() ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    paraDropWriteOutput(outputPwm);
    ledOn = paraDropUpdateIndicator(currentTimeUs, true);
    paraDropUpdateDebug(currentTimeUs, outputPwm, currentAltitudeAslMeters, currentAltitudeMeters, groundAltitudeMeters, baroReady && baroCalibrated, ledOn, activationValid);
}

#else

#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig, PG_PARA_DROP_CONFIG, 0);

PG_RESET_TEMPLATE(paraDropConfig_t, paraDropConfig,
    .aslThresholdMeters = 0,
    .servoChannel = 0,
    .holdTimeMs = 25,
    .aboveHoldTimeSec = 1,
    .indicatorPinio = 0,
    .indicatorBlinkHz = 5,
    .activationKey = { 0 },
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
