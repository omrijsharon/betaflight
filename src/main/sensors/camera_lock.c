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

#include "sensors/camera_lock.h"

#ifdef USE_FINAL

static cameraLockInfo_t cameraLockInfo;
static cameraLockRawState_t cameraLockRawState;
static timeMs_t cameraLockLastUpdateTimeMs;
static bool cameraLockHasEverUpdated;

static uint16_t constrainToUint16(timeMs_t value)
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

void cameraLockInit(void)
{
    cameraLockReset();
}

void cameraLockReset(void)
{
    memset(&cameraLockInfo, 0, sizeof(cameraLockInfo));
    memset(&cameraLockRawState, 0, sizeof(cameraLockRawState));
    cameraLockLastUpdateTimeMs = 0;
    cameraLockHasEverUpdated = false;
}

void cameraLockSetInfo(const cameraLockInfo_t *info)
{
    if (!info) {
        memset(&cameraLockInfo, 0, sizeof(cameraLockInfo));
        return;
    }

    cameraLockInfo = *info;
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

void cameraLockSetRawState(const cameraLockRawState_t *state, timeMs_t currentTimeMs)
{
    if (!state) {
        return;
    }

    cameraLockRawState = *state;
    cameraLockLastUpdateTimeMs = currentTimeMs;
    cameraLockHasEverUpdated = true;
}

void cameraLockClear(timeMs_t currentTimeMs)
{
    memset(&cameraLockRawState, 0, sizeof(cameraLockRawState));
    cameraLockLastUpdateTimeMs = currentTimeMs;
    cameraLockHasEverUpdated = true;
}

void cameraLockGetRawState(cameraLockRawState_t *state)
{
    if (!state) {
        return;
    }

    *state = cameraLockRawState;
}

uint16_t cameraLockGetAgeMs(timeMs_t currentTimeMs)
{
    if (!cameraLockHasEverUpdated) {
        return UINT16_MAX;
    }

    return constrainToUint16((timeDelta_t)(currentTimeMs - cameraLockLastUpdateTimeMs));
}

bool cameraLockHasDetection(void)
{
    return (cameraLockRawState.flags & CAMERA_LOCK_FLAG_DETECTED) != 0;
}

bool cameraLockIsHealthy(void)
{
    return (cameraLockRawState.flags & CAMERA_LOCK_FLAG_HEALTHY) != 0;
}

bool cameraLockIsFresh(timeMs_t currentTimeMs, uint16_t freshnessThresholdMs)
{
    if (!cameraLockHasEverUpdated) {
        return false;
    }

    return cameraLockGetAgeMs(currentTimeMs) <= freshnessThresholdMs;
}

void cameraLockGetState(cameraLockState_t *state, timeMs_t currentTimeMs, uint16_t freshnessThresholdMs)
{
    if (!state) {
        return;
    }

    state->flags = cameraLockRawState.flags & (CAMERA_LOCK_FLAG_DETECTED | CAMERA_LOCK_FLAG_HEALTHY);
    state->x_px = cameraLockRawState.x_px;
    state->y_px = cameraLockRawState.y_px;
    state->age_ms = cameraLockGetAgeMs(currentTimeMs);

    if (cameraLockIsFresh(currentTimeMs, freshnessThresholdMs)) {
        state->flags |= CAMERA_LOCK_FLAG_FRESH;
    }
}

#endif
