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

#include <string.h>

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"
#include "pg/pg_ids.h"
#include "sensors/camera_lock.h"

#ifdef USE_FINAL

PG_REGISTER_WITH_RESET_TEMPLATE(cameraLockConfig_t, cameraLockConfig, PG_CAMERA_LOCK_CONFIG, 0);

PG_RESET_TEMPLATE(cameraLockConfig_t, cameraLockConfig,
    .portOverride = SERIAL_PORT_NONE,
);

static cameraLockInfo_t cameraLockInfo;
static cameraLockRawState_t cameraLockRawState;
static timeUs_t cameraLockLastUpdateTimeUs;
static bool cameraLockHasEverUpdated;
static bool cameraLockInfoValid;
static bool cameraLockIsBound;
static bool cameraLockPollOutstanding;
static bool cameraLockShortTimeoutActive;
static mspDescriptor_t cameraLockBoundDescriptor;
static serialPortIdentifier_e cameraLockBoundPortIdentifier;
static mspVersion_e cameraLockBoundMspVersion;
static timeUs_t cameraLockPollPeriodUs;
static timeUs_t cameraLockLastPollSentUs;
static timeUs_t cameraLockLastReplyReceivedUs;
static timeUs_t cameraLockShortTimeoutUs;
static timeUs_t cameraLockLongTimeoutUs;

static uint16_t constrainToUint16(uint32_t value)
{
    if (value > UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)value;
}

static float decodeIntrinsicPx(uint32_t valueX1000)
{
    return valueX1000 / CAMERA_LOCK_INTRINSIC_SCALE;
}

static serialPortIdentifier_e cameraLockResolveBindPort(serialPortIdentifier_e discoveredPortIdentifier)
{
    const int8_t portOverride = cameraLockConfig()->portOverride;
    if (portOverride != SERIAL_PORT_NONE) {
        return (serialPortIdentifier_e)portOverride;
    }

    return discoveredPortIdentifier;
}

static void cameraLockResetLinkState(void)
{
    cameraLockInfoValid = false;
    cameraLockIsBound = false;
    cameraLockPollOutstanding = false;
    cameraLockShortTimeoutActive = false;
    cameraLockBoundDescriptor = -1;
    cameraLockBoundPortIdentifier = SERIAL_PORT_NONE;
    cameraLockBoundMspVersion = MSP_V1;
    cameraLockPollPeriodUs = 0;
    cameraLockLastPollSentUs = 0;
    cameraLockLastReplyReceivedUs = 0;
    cameraLockShortTimeoutUs = 0;
    cameraLockLongTimeoutUs = 0;
}

void cameraLockInit(void)
{
    cameraLockReset();
}

void cameraLockReset(void)
{
    memset(&cameraLockInfo, 0, sizeof(cameraLockInfo));
    memset(&cameraLockRawState, 0, sizeof(cameraLockRawState));
    cameraLockLastUpdateTimeUs = 0;
    cameraLockHasEverUpdated = false;
    cameraLockResetLinkState();
}

void cameraLockSetInfo(const cameraLockInfo_t *info)
{
    if (!info) {
        memset(&cameraLockInfo, 0, sizeof(cameraLockInfo));
        cameraLockInfoValid = false;
        return;
    }

    cameraLockInfo = *info;
}

bool cameraLockBindFromSource(const cameraLockInfo_t *info, mspDescriptor_t srcDesc, serialPortIdentifier_e portIdentifier, mspVersion_e mspVersion, timeUs_t currentTimeUs)
{
    const serialPortIdentifier_e bindPortIdentifier = cameraLockResolveBindPort(portIdentifier);

    if (!info || bindPortIdentifier == SERIAL_PORT_NONE) {
        return false;
    }

    cameraLockSetInfo(info);
    cameraLockInfoValid = true;
    cameraLockIsBound = true;
    cameraLockPollOutstanding = false;
    cameraLockShortTimeoutActive = false;
    cameraLockBoundDescriptor = srcDesc;
    cameraLockBoundPortIdentifier = bindPortIdentifier;
    cameraLockBoundMspVersion = mspVersion;
    cameraLockPollPeriodUs = info->lock_rate_hz ? (1000 * 1000) / info->lock_rate_hz : 0;
    cameraLockShortTimeoutUs = cameraLockPollPeriodUs * 2;
    cameraLockLongTimeoutUs = cameraLockPollPeriodUs * 10;
    cameraLockLastPollSentUs = (currentTimeUs > cameraLockPollPeriodUs) ? (currentTimeUs - cameraLockPollPeriodUs) : 0;
    cameraLockLastReplyReceivedUs = currentTimeUs;
    cameraLockClear(currentTimeUs);

    return true;
}

const cameraLockInfo_t *cameraLockGetInfo(void)
{
    return &cameraLockInfo;
}

float cameraLockGetFxPx(void)
{
    return decodeIntrinsicPx(cameraLockInfo.fx_px_x1000);
}

float cameraLockGetFyPx(void)
{
    return decodeIntrinsicPx(cameraLockInfo.fy_px_x1000);
}

float cameraLockGetCxPx(void)
{
    return decodeIntrinsicPx(cameraLockInfo.cx_px_x1000);
}

float cameraLockGetCyPx(void)
{
    return decodeIntrinsicPx(cameraLockInfo.cy_px_x1000);
}

void cameraLockSetRawState(const cameraLockRawState_t *state, timeUs_t currentTimeUs)
{
    if (!state) {
        return;
    }

    cameraLockRawState = *state;
    cameraLockLastUpdateTimeUs = currentTimeUs;
    cameraLockHasEverUpdated = true;
    cameraLockPollOutstanding = false;
    cameraLockShortTimeoutActive = false;
    cameraLockLastReplyReceivedUs = currentTimeUs;
}

void cameraLockClear(timeUs_t currentTimeUs)
{
    memset(&cameraLockRawState, 0, sizeof(cameraLockRawState));
    cameraLockLastUpdateTimeUs = currentTimeUs;
    cameraLockHasEverUpdated = true;
}

void cameraLockHandleReply(mspDescriptor_t srcDesc, const cameraLockRawState_t *state, timeUs_t currentTimeUs)
{
    if (!cameraLockIsBound || srcDesc != cameraLockBoundDescriptor || !state) {
        return;
    }

    cameraLockSetRawState(state, currentTimeUs);
}

void cameraLockService(timeUs_t currentTimeUs)
{
    if (!cameraLockInfoValid || !cameraLockIsBound || !cameraLockPollPeriodUs) {
        return;
    }

    const timeDelta_t sinceLastReplyUs = cmpTimeUs(currentTimeUs, cameraLockLastReplyReceivedUs);

    if (cameraLockLongTimeoutUs && sinceLastReplyUs >= (timeDelta_t)cameraLockLongTimeoutUs) {
        cameraLockClear(currentTimeUs);
        cameraLockResetLinkState();
        return;
    }

    if (cameraLockShortTimeoutUs && sinceLastReplyUs >= (timeDelta_t)cameraLockShortTimeoutUs) {
        cameraLockRawState.flags &= ~CAMERA_LOCK_FLAG_HEALTHY;
        cameraLockShortTimeoutActive = true;
        cameraLockPollOutstanding = false;
    }

    if (!cameraLockPollOutstanding && cmpTimeUs(currentTimeUs, cameraLockLastPollSentUs) >= (timeDelta_t)cameraLockPollPeriodUs) {
        if (mspSerialPush(cameraLockBoundPortIdentifier, MSP_CAMERA_GET_LOCK, NULL, 0, MSP_DIRECTION_REQUEST, cameraLockBoundMspVersion) > 0) {
            cameraLockLastPollSentUs = currentTimeUs;
            cameraLockPollOutstanding = true;
        }
    }
}

void cameraLockGetRawState(cameraLockRawState_t *state)
{
    if (!state) {
        return;
    }

    *state = cameraLockRawState;
}

uint16_t cameraLockGetAgeMs(timeUs_t currentTimeUs)
{
    if (!cameraLockHasEverUpdated) {
        return UINT16_MAX;
    }

    return constrainToUint16((uint32_t)(cmpTimeUs(currentTimeUs, cameraLockLastUpdateTimeUs) / 1000));
}

bool cameraLockHasDetection(void)
{
    return (cameraLockRawState.flags & CAMERA_LOCK_FLAG_DETECTED) != 0;
}

bool cameraLockIsHealthy(void)
{
    return (cameraLockRawState.flags & CAMERA_LOCK_FLAG_HEALTHY) != 0;
}

bool cameraLockIsFresh(timeUs_t currentTimeUs, uint16_t freshnessThresholdMs)
{
    if (!cameraLockHasEverUpdated || cameraLockShortTimeoutActive) {
        return false;
    }

    return cameraLockGetAgeMs(currentTimeUs) <= freshnessThresholdMs;
}

void cameraLockGetState(cameraLockState_t *state, timeUs_t currentTimeUs, uint16_t freshnessThresholdMs)
{
    if (!state) {
        return;
    }

    state->flags = cameraLockRawState.flags & (CAMERA_LOCK_FLAG_DETECTED | CAMERA_LOCK_FLAG_HEALTHY);
    state->x_px = cameraLockRawState.x_px;
    state->y_px = cameraLockRawState.y_px;
    state->age_ms = cameraLockGetAgeMs(currentTimeUs);

    if (cameraLockIsFresh(currentTimeUs, freshnessThresholdMs)) {
        state->flags |= CAMERA_LOCK_FLAG_FRESH;
    }
}

#endif
