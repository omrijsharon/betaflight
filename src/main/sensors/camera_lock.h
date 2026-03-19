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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "pg/pg.h"

#ifdef USE_FINAL

#define CAMERA_LOCK_INTRINSIC_SCALE          1000.0f
#define CAMERA_LOCK_DEFAULT_FRESHNESS_THRESHOLD_MS 100
#define CAMERA_LOCK_INFO_FLAG_VALID            (1U << 0)
#define CAMERA_LOCK_INFO_FLAG_CROPPED          (1U << 1)
#define CAMERA_LOCK_INFO_FLAG_INTRINSICS_VALID (1U << 2)
#define CAMERA_LOCK_INFO_FLAG_FOV_VALID        (1U << 3)
#define CAMERA_LOCK_INFO_FLAG_HEALTHY          (1U << 4)

#define CAMERA_LOCK_FLAG_DETECTED              (1U << 0)
#define CAMERA_LOCK_FLAG_HEALTHY               (1U << 1)
#define CAMERA_LOCK_FLAG_FRESH                 (1U << 2)

typedef enum {
    CAMERA_LOCK_ORIENTATION_0 = 0,
    CAMERA_LOCK_ORIENTATION_90 = 1,
    CAMERA_LOCK_ORIENTATION_180 = 2,
    CAMERA_LOCK_ORIENTATION_MINUS_90 = 3,
} cameraLockOrientation_e;

typedef struct cameraLockInfo_s {
    uint16_t width_px;
    uint16_t height_px;
    uint32_t fx_px_x1000;
    uint32_t fy_px_x1000;
    uint32_t cx_px_x1000;
    uint32_t cy_px_x1000;
    uint8_t hfov_deg;
    uint8_t vfov_deg;
    int8_t tilt_angle_deg;
    uint8_t orientation;
    uint16_t lock_rate_hz;
    uint8_t flags;
} cameraLockInfo_t;

typedef struct cameraLockRawState_s {
    uint8_t flags;
    uint16_t x_px;
    uint16_t y_px;
} cameraLockRawState_t;

typedef struct cameraLockState_s {
    uint8_t flags;
    uint16_t x_px;
    uint16_t y_px;
    uint16_t age_ms;
} cameraLockState_t;

typedef struct cameraLockConfig_s {
    int8_t portOverride;
} cameraLockConfig_t;

PG_DECLARE(cameraLockConfig_t, cameraLockConfig);

void cameraLockInit(void);
void cameraLockReset(void);

void cameraLockSetInfo(const cameraLockInfo_t *info);
bool cameraLockBindFromSource(const cameraLockInfo_t *info, mspDescriptor_t srcDesc, serialPortIdentifier_e portIdentifier, mspVersion_e mspVersion, timeUs_t currentTimeUs);
const cameraLockInfo_t *cameraLockGetInfo(void);
float cameraLockGetFxPx(void);
float cameraLockGetFyPx(void);
float cameraLockGetCxPx(void);
float cameraLockGetCyPx(void);

void cameraLockSetRawState(const cameraLockRawState_t *state, timeUs_t currentTimeUs);
void cameraLockClear(timeUs_t currentTimeUs);
void cameraLockService(timeUs_t currentTimeUs);
void cameraLockHandleReply(mspDescriptor_t srcDesc, const cameraLockRawState_t *state, timeUs_t currentTimeUs);

void cameraLockGetState(cameraLockState_t *state, timeUs_t currentTimeUs, uint16_t freshnessThresholdMs);
void cameraLockGetRawState(cameraLockRawState_t *state);

bool cameraLockHasDetection(void);
bool cameraLockIsHealthy(void);
bool cameraLockIsFresh(timeUs_t currentTimeUs, uint16_t freshnessThresholdMs);
uint16_t cameraLockGetAgeMs(timeUs_t currentTimeUs);

#endif
