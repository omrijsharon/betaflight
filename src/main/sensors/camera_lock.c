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

#include <math.h>
#include <string.h>

#include "common/maths.h"
#include "flight/imu.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"
#include "pg/pg_ids.h"
#include "sensors/camera_lock.h"

#ifdef USE_FINAL

PG_REGISTER_WITH_RESET_TEMPLATE(cameraLockConfig_t, cameraLockConfig, PG_CAMERA_LOCK_CONFIG, 0);

PG_RESET_TEMPLATE(cameraLockConfig_t, cameraLockConfig,
    .portOverride = SERIAL_PORT_NONE,
);

static seekerCamInfo_t cameraLockSeekerInfo;
static fpvCamInfo_t cameraLockFpvInfo;
static cameraLockRawState_t cameraLockRawState;
static timeUs_t cameraLockLastUpdateTimeUs;
static bool cameraLockHasEverUpdated;
static bool cameraLockSeekerInfoValid;
static bool cameraLockFpvInfoValid;
static bool cameraLockIsBound;
static bool cameraLockPollOutstanding;
static bool cameraLockShortTimeoutActive;
static bool cameraLockWaitingForFpvInfo;
static bool cameraLockShowNoFpvConfigWarning;
static bool cameraLockProjectionCacheValid;
static mspDescriptor_t cameraLockBoundDescriptor;
static serialPortIdentifier_e cameraLockBoundPortIdentifier;
static mspVersion_e cameraLockBoundMspVersion;
static timeUs_t cameraLockPollPeriodUs;
static timeUs_t cameraLockLastPollSentUs;
static timeUs_t cameraLockLastReplyReceivedUs;
static timeUs_t cameraLockShortTimeoutUs;
static timeUs_t cameraLockLongTimeoutUs;
static timeUs_t cameraLockWaitingForFpvInfoSinceUs;
static float cameraLockSeekerFxPx;
static float cameraLockSeekerFyPx;
static float cameraLockSeekerCxPx;
static float cameraLockSeekerCyPx;
static float cameraLockFpvFxPx;
static float cameraLockFpvFyPx;
static float cameraLockFpvCxPx;
static float cameraLockFpvCyPx;
static uint16_t cameraLockFpvWidthPx;
static uint16_t cameraLockFpvHeightPx;
static float cameraLockProjectionHomography[3][3];
static cameraLockCornerOverlay_t cameraLockCornerOverlay;

static bool cameraLockProjectionRequiredFromSeekerInfo(const seekerCamInfo_t *info);

static uint16_t constrainToUint16(uint32_t value)
{
    if (value > UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)value;
}

static serialPortIdentifier_e cameraLockResolveBindPort(serialPortIdentifier_e discoveredPortIdentifier)
{
    const int8_t portOverride = cameraLockConfig()->portOverride;
    if (portOverride != SERIAL_PORT_NONE) {
        return (serialPortIdentifier_e)portOverride;
    }

    return discoveredPortIdentifier;
}

static float cameraLockIntrinsicToFloat(uint32_t value_x1000)
{
    return value_x1000 / CAMERA_LOCK_INTRINSIC_SCALE;
}

static void cameraLockMatrixSetIdentity(float matrix[3][3])
{
    memset(matrix, 0, sizeof(float) * 9);
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;
}

static void cameraLockMatrixMultiply(float left[3][3], float right[3][3], float out[3][3])
{
    float result[3][3];

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            result[row][col] = 0.0f;
            for (int i = 0; i < 3; i++) {
                result[row][col] += left[row][i] * right[i][col];
            }
        }
    }

    memcpy(out, result, sizeof(result));
}

static void cameraLockMatrixTranspose(float in[3][3], float out[3][3])
{
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            out[row][col] = in[col][row];
        }
    }
}

static void cameraLockMatrixVectorMultiply(float matrix[3][3], float vector[3], float out[3])
{
    for (int row = 0; row < 3; row++) {
        out[row] = 0.0f;
        for (int col = 0; col < 3; col++) {
            out[row] += matrix[row][col] * vector[col];
        }
    }
}

static void cameraLockBuildAxisMap(float out[3][3])
{
    cameraLockMatrixSetIdentity(out);
    out[0][0] = 0.0f;
    out[0][1] = 0.0f;
    out[0][2] = 1.0f;
    out[1][0] = -1.0f;
    out[1][1] = 0.0f;
    out[1][2] = 0.0f;
    out[2][0] = 0.0f;
    out[2][1] = -1.0f;
    out[2][2] = 0.0f;
}

static void cameraLockBuildOrientationMatrix(uint8_t orientation, float out[3][3])
{
    cameraLockMatrixSetIdentity(out);

    switch (orientation) {
    case CAMERA_LOCK_ORIENTATION_90:
        out[1][1] = 0.0f;
        out[1][2] = -1.0f;
        out[2][1] = 1.0f;
        out[2][2] = 0.0f;
        break;
    case CAMERA_LOCK_ORIENTATION_180:
        out[1][1] = -1.0f;
        out[2][2] = -1.0f;
        break;
    case CAMERA_LOCK_ORIENTATION_MINUS_90:
        out[1][1] = 0.0f;
        out[1][2] = 1.0f;
        out[2][1] = -1.0f;
        out[2][2] = 0.0f;
        break;
    case CAMERA_LOCK_ORIENTATION_0:
    default:
        break;
    }
}

static void cameraLockBuildTiltMatrix(int8_t tiltAngleDeg, float out[3][3])
{
    const float tiltRadians = DEGREES_TO_RADIANS((float)tiltAngleDeg);
    const float cosTilt = cos_approx(tiltRadians);
    const float sinTilt = sin_approx(tiltRadians);

    cameraLockMatrixSetIdentity(out);
    out[0][0] = cosTilt;
    out[0][2] = sinTilt;
    out[2][0] = -sinTilt;
    out[2][2] = cosTilt;
}

static int32_t cameraLockRoundFloatToInt32(float value)
{
    if (value >= (float)INT32_MAX) {
        return INT32_MAX;
    }

    if (value <= (float)INT32_MIN) {
        return INT32_MIN;
    }

    return lrintf(value);
}

static bool cameraLockProjectPixelToDisplayTarget(uint16_t sourceX_px, uint16_t sourceY_px, cameraLockDisplayTarget_t *target)
{
    if (!target || !cameraLockSeekerInfoValid
        || (cameraLockSeekerInfo.flags & SEEKER_CAM_INFO_FLAG_VALID) == 0
        || cameraLockSeekerInfo.width_px <= 1 || cameraLockSeekerInfo.height_px <= 1) {
        return false;
    }

    memset(target, 0, sizeof(*target));
    target->sourceX_px = sourceX_px;
    target->sourceY_px = sourceY_px;

    if (cameraLockProjectionCacheValid) {
        float seekerPixel[3] = {
            (float)sourceX_px,
            (float)sourceY_px,
            1.0f
        };
        float projectedPoint[3];

        cameraLockMatrixVectorMultiply(cameraLockProjectionHomography, seekerPixel, projectedPoint);

        if (projectedPoint[2] <= 1.0e-6f) {
            return false;
        }

        const float invZ = 1.0f / projectedPoint[2];
        const float projectedX = projectedPoint[0] * invZ;
        const float projectedY = projectedPoint[1] * invZ;

        if (!isfinite(projectedX) || !isfinite(projectedY)) {
            return false;
        }

        target->projected = true;
        target->width_px = cameraLockFpvWidthPx;
        target->height_px = cameraLockFpvHeightPx;
        target->projectedX_px = cameraLockRoundFloatToInt32(projectedX);
        target->projectedY_px = cameraLockRoundFloatToInt32(projectedY);
        const int clampedX = constrain(target->projectedX_px, 0, cameraLockFpvWidthPx - 1);
        const int clampedY = constrain(target->projectedY_px, 0, cameraLockFpvHeightPx - 1);
        target->displayX_px = (uint16_t)clampedX;
        target->displayY_px = (uint16_t)clampedY;
        target->clamped = (clampedX != target->projectedX_px) || (clampedY != target->projectedY_px);
        return true;
    }

    if (cameraLockProjectionRequiredFromSeekerInfo(&cameraLockSeekerInfo)) {
        return false;
    }

    target->width_px = cameraLockSeekerInfo.width_px;
    target->height_px = cameraLockSeekerInfo.height_px;
    target->projectedX_px = sourceX_px;
    target->projectedY_px = sourceY_px;
    target->displayX_px = (uint16_t)constrain(sourceX_px, 0, cameraLockSeekerInfo.width_px - 1);
    target->displayY_px = (uint16_t)constrain(sourceY_px, 0, cameraLockSeekerInfo.height_px - 1);
    target->clamped = (target->displayX_px != sourceX_px) || (target->displayY_px != sourceY_px);
    return true;
}

static bool cameraLockProjectPixelToBodyRay(uint16_t sourceX_px, uint16_t sourceY_px, float bodyRay[3])
{
    if (!bodyRay || !cameraLockSeekerInfoValid
        || (cameraLockSeekerInfo.flags & (SEEKER_CAM_INFO_FLAG_VALID | SEEKER_CAM_INFO_FLAG_INTRINSICS_VALID))
            != (SEEKER_CAM_INFO_FLAG_VALID | SEEKER_CAM_INFO_FLAG_INTRINSICS_VALID)
        || cameraLockSeekerFxPx <= 0.0f || cameraLockSeekerFyPx <= 0.0f) {
        return false;
    }

    float axisMap[3][3];
    float seekerOrientation[3][3];
    float seekerTilt[3][3];
    float seekerBody[3][3];
    float seekerPixel[3] = {
        (float)sourceX_px,
        (float)sourceY_px,
        1.0f
    };
    float kSeekInv[3][3];
    float seekerRay[3];

    cameraLockBuildAxisMap(axisMap);
    cameraLockBuildOrientationMatrix(cameraLockSeekerInfo.orientation, seekerOrientation);
    cameraLockBuildTiltMatrix(cameraLockSeekerInfo.tilt_angle_deg, seekerTilt);

    cameraLockMatrixMultiply(seekerOrientation, axisMap, seekerBody);
    cameraLockMatrixMultiply(seekerTilt, seekerBody, seekerBody);

    cameraLockMatrixSetIdentity(kSeekInv);
    kSeekInv[0][0] = 1.0f / cameraLockSeekerFxPx;
    kSeekInv[0][2] = -cameraLockSeekerCxPx / cameraLockSeekerFxPx;
    kSeekInv[1][1] = 1.0f / cameraLockSeekerFyPx;
    kSeekInv[1][2] = -cameraLockSeekerCyPx / cameraLockSeekerFyPx;

    cameraLockMatrixVectorMultiply(kSeekInv, seekerPixel, seekerRay);
    cameraLockMatrixVectorMultiply(seekerBody, seekerRay, bodyRay);
    return true;
}

static int32_t cameraLockPolygonArea2(const cameraLockDisplayTarget_t corners[CAMERA_LOCK_CORNER_COUNT])
{
    int32_t area2 = 0;

    for (unsigned i = 0; i < CAMERA_LOCK_CORNER_COUNT; i++) {
        const unsigned next = (i + 1U) % CAMERA_LOCK_CORNER_COUNT;
        area2 += ((int32_t)corners[i].displayX_px * (int32_t)corners[next].displayY_px)
            - ((int32_t)corners[next].displayX_px * (int32_t)corners[i].displayY_px);
    }

    return area2;
}

static bool cameraLockCornerOverlayIsDegenerate(const cameraLockCornerOverlay_t *overlay)
{
    unsigned uniqueCount = 0;
    uint32_t packedPoints[CAMERA_LOCK_CORNER_COUNT];

    for (unsigned i = 0; i < CAMERA_LOCK_CORNER_COUNT; i++) {
        packedPoints[i] = ((uint32_t)overlay->corners[i].displayX_px << 16) | overlay->corners[i].displayY_px;
    }

    for (unsigned i = 0; i < CAMERA_LOCK_CORNER_COUNT; i++) {
        bool isNew = true;

        for (unsigned j = 0; j < i; j++) {
            if (packedPoints[i] == packedPoints[j]) {
                isNew = false;
                break;
            }
        }

        if (isNew) {
            uniqueCount++;
        }
    }

    if (uniqueCount < 3U) {
        return true;
    }

    return ABS(cameraLockPolygonArea2(overlay->corners)) < 2;
}

static void cameraLockRebuildCornerOverlay(void)
{
    memset(&cameraLockCornerOverlay, 0, sizeof(cameraLockCornerOverlay));

    if (!cameraLockSeekerInfoValid || (cameraLockSeekerInfo.flags & SEEKER_CAM_INFO_FLAG_VALID) == 0
        || cameraLockSeekerInfo.width_px <= 1 || cameraLockSeekerInfo.height_px <= 1) {
        return;
    }

    const uint16_t widthMax = cameraLockSeekerInfo.width_px - 1U;
    const uint16_t heightMax = cameraLockSeekerInfo.height_px - 1U;
    const uint16_t sourceCorners[CAMERA_LOCK_CORNER_COUNT][2] = {
        { 0U, 0U },
        { widthMax, 0U },
        { widthMax, heightMax },
        { 0U, heightMax },
    };

    for (unsigned i = 0; i < CAMERA_LOCK_CORNER_COUNT; i++) {
        if (!cameraLockProjectPixelToDisplayTarget(sourceCorners[i][0], sourceCorners[i][1], &cameraLockCornerOverlay.corners[i])) {
            memset(&cameraLockCornerOverlay, 0, sizeof(cameraLockCornerOverlay));
            return;
        }
    }

    if (cameraLockCornerOverlayIsDegenerate(&cameraLockCornerOverlay)) {
        memset(&cameraLockCornerOverlay, 0, sizeof(cameraLockCornerOverlay));
        return;
    }

    cameraLockCornerOverlay.valid = true;
}

static void cameraLockResetProjectionCache(void)
{
    cameraLockProjectionCacheValid = false;
    cameraLockSeekerFxPx = 0.0f;
    cameraLockSeekerFyPx = 0.0f;
    cameraLockSeekerCxPx = 0.0f;
    cameraLockSeekerCyPx = 0.0f;
    cameraLockFpvFxPx = 0.0f;
    cameraLockFpvFyPx = 0.0f;
    cameraLockFpvCxPx = 0.0f;
    cameraLockFpvCyPx = 0.0f;
    cameraLockFpvWidthPx = 0;
    cameraLockFpvHeightPx = 0;
    memset(cameraLockProjectionHomography, 0, sizeof(cameraLockProjectionHomography));
    memset(&cameraLockCornerOverlay, 0, sizeof(cameraLockCornerOverlay));
}

static void cameraLockRebuildProjectionCache(void)
{
    cameraLockResetProjectionCache();

    if (!cameraLockSeekerInfoValid || !cameraLockFpvInfoValid) {
        return;
    }

    if ((cameraLockSeekerInfo.flags & (SEEKER_CAM_INFO_FLAG_VALID | SEEKER_CAM_INFO_FLAG_INTRINSICS_VALID))
            != (SEEKER_CAM_INFO_FLAG_VALID | SEEKER_CAM_INFO_FLAG_INTRINSICS_VALID)
        || (cameraLockFpvInfo.flags & (FPV_CAM_INFO_FLAG_VALID | FPV_CAM_INFO_FLAG_INTRINSICS_VALID))
            != (FPV_CAM_INFO_FLAG_VALID | FPV_CAM_INFO_FLAG_INTRINSICS_VALID)) {
        return;
    }

    cameraLockSeekerFxPx = cameraLockIntrinsicToFloat(cameraLockSeekerInfo.intrinsics.fx_px_x1000);
    cameraLockSeekerFyPx = cameraLockIntrinsicToFloat(cameraLockSeekerInfo.intrinsics.fy_px_x1000);
    cameraLockSeekerCxPx = cameraLockIntrinsicToFloat(cameraLockSeekerInfo.intrinsics.cx_px_x1000);
    cameraLockSeekerCyPx = cameraLockIntrinsicToFloat(cameraLockSeekerInfo.intrinsics.cy_px_x1000);
    cameraLockFpvFxPx = cameraLockIntrinsicToFloat(cameraLockFpvInfo.intrinsics.fx_px_x1000);
    cameraLockFpvFyPx = cameraLockIntrinsicToFloat(cameraLockFpvInfo.intrinsics.fy_px_x1000);
    cameraLockFpvCxPx = cameraLockIntrinsicToFloat(cameraLockFpvInfo.intrinsics.cx_px_x1000);
    cameraLockFpvCyPx = cameraLockIntrinsicToFloat(cameraLockFpvInfo.intrinsics.cy_px_x1000);

    if (cameraLockSeekerFxPx <= 0.0f || cameraLockSeekerFyPx <= 0.0f
        || cameraLockFpvFxPx <= 0.0f || cameraLockFpvFyPx <= 0.0f) {
        return;
    }

    const uint32_t widthEstimate = (uint32_t)lrintf((2.0f * cameraLockFpvCxPx) + 1.0f);
    const uint32_t heightEstimate = (uint32_t)lrintf((2.0f * cameraLockFpvCyPx) + 1.0f);

    if (widthEstimate <= 1 || heightEstimate <= 1 || widthEstimate > UINT16_MAX || heightEstimate > UINT16_MAX) {
        return;
    }

    cameraLockFpvWidthPx = (uint16_t)widthEstimate;
    cameraLockFpvHeightPx = (uint16_t)heightEstimate;

    float axisMap[3][3];
    float seekerOrientation[3][3];
    float seekerTilt[3][3];
    float fpvTilt[3][3];
    float seekerBody[3][3];
    float fpvBody[3][3];
    float fpvBodyTranspose[3][3];
    float kFpv[3][3];
    float kSeekInv[3][3];
    float kFpvTimesR[3][3];

    cameraLockBuildAxisMap(axisMap);
    cameraLockBuildOrientationMatrix(cameraLockSeekerInfo.orientation, seekerOrientation);
    cameraLockBuildTiltMatrix(cameraLockSeekerInfo.tilt_angle_deg, seekerTilt);
    cameraLockBuildTiltMatrix(cameraLockFpvInfo.tilt_angle_deg, fpvTilt);

    cameraLockMatrixMultiply(seekerOrientation, axisMap, seekerBody);
    cameraLockMatrixMultiply(seekerTilt, seekerBody, seekerBody);
    cameraLockMatrixMultiply(fpvTilt, axisMap, fpvBody);
    cameraLockMatrixTranspose(fpvBody, fpvBodyTranspose);
    cameraLockMatrixMultiply(fpvBodyTranspose, seekerBody, kFpvTimesR);

    cameraLockMatrixSetIdentity(kFpv);
    kFpv[0][0] = cameraLockFpvFxPx;
    kFpv[0][2] = cameraLockFpvCxPx;
    kFpv[1][1] = cameraLockFpvFyPx;
    kFpv[1][2] = cameraLockFpvCyPx;

    cameraLockMatrixSetIdentity(kSeekInv);
    kSeekInv[0][0] = 1.0f / cameraLockSeekerFxPx;
    kSeekInv[0][2] = -cameraLockSeekerCxPx / cameraLockSeekerFxPx;
    kSeekInv[1][1] = 1.0f / cameraLockSeekerFyPx;
    kSeekInv[1][2] = -cameraLockSeekerCyPx / cameraLockSeekerFyPx;

    cameraLockMatrixMultiply(kFpv, kFpvTimesR, kFpvTimesR);
    cameraLockMatrixMultiply(kFpvTimesR, kSeekInv, cameraLockProjectionHomography);

    cameraLockProjectionCacheValid = true;
    cameraLockRebuildCornerOverlay();
}

static void cameraLockResetLinkState(void)
{
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

static void cameraLockResetConfigState(void)
{
    memset(&cameraLockSeekerInfo, 0, sizeof(cameraLockSeekerInfo));
    memset(&cameraLockFpvInfo, 0, sizeof(cameraLockFpvInfo));
    cameraLockSeekerInfoValid = false;
    cameraLockFpvInfoValid = false;
    cameraLockWaitingForFpvInfo = false;
    cameraLockShowNoFpvConfigWarning = false;
    cameraLockWaitingForFpvInfoSinceUs = 0;
    cameraLockResetProjectionCache();
}

static bool cameraLockProjectionRequiredFromSeekerInfo(const seekerCamInfo_t *info)
{
    return info && ((info->flags & SEEKER_CAM_INFO_FLAG_FPV_PROJECTION_REQUIRED) != 0);
}

static void cameraLockPreparePolling(timeUs_t currentTimeUs)
{
    cameraLockPollOutstanding = false;
    cameraLockShortTimeoutActive = false;
    cameraLockShowNoFpvConfigWarning = false;
    cameraLockWaitingForFpvInfo = false;
    cameraLockWaitingForFpvInfoSinceUs = 0;
    cameraLockLastPollSentUs = (currentTimeUs > cameraLockPollPeriodUs) ? (currentTimeUs - cameraLockPollPeriodUs) : 0;
    cameraLockLastReplyReceivedUs = currentTimeUs;
    cameraLockClear(currentTimeUs);
}

void cameraLockInit(void)
{
    cameraLockReset();
}

void cameraLockReset(void)
{
    memset(&cameraLockRawState, 0, sizeof(cameraLockRawState));
    cameraLockLastUpdateTimeUs = 0;
    cameraLockHasEverUpdated = false;
    cameraLockResetConfigState();
    cameraLockResetLinkState();
}

void cameraLockSetSeekerInfo(const seekerCamInfo_t *info)
{
    if (!info) {
        memset(&cameraLockSeekerInfo, 0, sizeof(cameraLockSeekerInfo));
        cameraLockSeekerInfoValid = false;
        cameraLockRebuildProjectionCache();
        return;
    }

    cameraLockSeekerInfo = *info;
    cameraLockSeekerInfoValid = true;
    cameraLockRebuildProjectionCache();
    cameraLockRebuildCornerOverlay();
}

bool cameraLockBindSeekerFromSource(const seekerCamInfo_t *info, mspDescriptor_t srcDesc, serialPortIdentifier_e portIdentifier, mspVersion_e mspVersion, timeUs_t currentTimeUs)
{
    const serialPortIdentifier_e bindPortIdentifier = cameraLockResolveBindPort(portIdentifier);

    if (!info || bindPortIdentifier == SERIAL_PORT_NONE) {
        return false;
    }

    cameraLockSetSeekerInfo(info);
    memset(&cameraLockFpvInfo, 0, sizeof(cameraLockFpvInfo));
    cameraLockFpvInfoValid = false;
    cameraLockWaitingForFpvInfo = false;
    cameraLockShowNoFpvConfigWarning = false;
    cameraLockWaitingForFpvInfoSinceUs = 0;
    cameraLockResetProjectionCache();
    cameraLockRebuildCornerOverlay();
    cameraLockResetLinkState();
    cameraLockIsBound = true;
    cameraLockBoundDescriptor = srcDesc;
    cameraLockBoundPortIdentifier = bindPortIdentifier;
    cameraLockBoundMspVersion = mspVersion;
    cameraLockPollPeriodUs = info->lock_rate_hz ? (1000 * 1000) / info->lock_rate_hz : 0;
    cameraLockShortTimeoutUs = cameraLockPollPeriodUs * 2;
    cameraLockLongTimeoutUs = cameraLockPollPeriodUs * 10;

    if (cameraLockProjectionRequiredFromSeekerInfo(info)) {
        cameraLockWaitingForFpvInfo = true;
        cameraLockShowNoFpvConfigWarning = false;
        cameraLockWaitingForFpvInfoSinceUs = currentTimeUs;
        cameraLockLastPollSentUs = 0;
        cameraLockLastReplyReceivedUs = 0;
        cameraLockClear(currentTimeUs);
    } else {
        cameraLockPreparePolling(currentTimeUs);
    }

    return true;
}

bool cameraLockSetFpvInfo(const fpvCamInfo_t *info, timeUs_t currentTimeUs)
{
    if (!info || !cameraLockSeekerInfoValid || !cameraLockIsBound) {
        return false;
    }

    cameraLockFpvInfo = *info;
    cameraLockFpvInfoValid = true;
    cameraLockRebuildProjectionCache();
    cameraLockRebuildCornerOverlay();

    if (cameraLockProjectionRequiredFromSeekerInfo(&cameraLockSeekerInfo) && cameraLockProjectionCacheValid) {
        cameraLockPreparePolling(currentTimeUs);
    }

    return cameraLockProjectionCacheValid;
}

const seekerCamInfo_t *cameraLockGetSeekerInfo(void)
{
    return &cameraLockSeekerInfo;
}

const fpvCamInfo_t *cameraLockGetFpvInfo(void)
{
    return &cameraLockFpvInfo;
}

bool cameraLockHasValidSeekerInfo(void)
{
    return cameraLockSeekerInfoValid;
}

bool cameraLockHasValidFpvInfo(void)
{
    return cameraLockProjectionCacheValid;
}

bool cameraLockIsFpvProjectionRequired(void)
{
    return cameraLockProjectionRequiredFromSeekerInfo(&cameraLockSeekerInfo);
}

bool cameraLockShouldShowNoFpvConfigWarning(void)
{
    return cameraLockShowNoFpvConfigWarning;
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
    if (!cameraLockSeekerInfoValid || !cameraLockIsBound || !cameraLockPollPeriodUs) {
        return;
    }

    if (cameraLockProjectionRequiredFromSeekerInfo(&cameraLockSeekerInfo) && !cameraLockProjectionCacheValid) {
        if (cameraLockWaitingForFpvInfo && cmpTimeUs(currentTimeUs, cameraLockWaitingForFpvInfoSinceUs) >= CAMERA_LOCK_REQUIRED_FPV_INFO_TIMEOUT_US) {
            cameraLockShowNoFpvConfigWarning = true;
        }
        return;
    }

    const timeDelta_t sinceLastReplyUs = cmpTimeUs(currentTimeUs, cameraLockLastReplyReceivedUs);

    if (cameraLockLongTimeoutUs && sinceLastReplyUs >= (timeDelta_t)cameraLockLongTimeoutUs) {
        cameraLockClear(currentTimeUs);
        cameraLockResetConfigState();
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

bool cameraLockGetDisplayTarget(const cameraLockState_t *state, cameraLockDisplayTarget_t *target)
{
    if (!state || !target) {
        return false;
    }

    return cameraLockProjectPixelToDisplayTarget(state->x_px, state->y_px, target);
}

bool cameraLockGetCornerOverlay(cameraLockCornerOverlay_t *overlay)
{
    if (!overlay || !cameraLockCornerOverlay.valid) {
        return false;
    }

    *overlay = cameraLockCornerOverlay;
    return true;
}

bool cameraLockGetRayDebug(const cameraLockState_t *state, cameraLockRayDebug_t *debugData)
{
    if (!state || !debugData) {
        return false;
    }

    float bodyRay[3];

    if (!cameraLockProjectPixelToBodyRay(state->x_px, state->y_px, bodyRay)) {
        return false;
    }

    debugData->bodyRay[0] = bodyRay[0];
    debugData->bodyRay[1] = bodyRay[1];
    debugData->bodyRay[2] = bodyRay[2];

    debugData->earthRay[0] = (rMat[0][0] * bodyRay[0]) + (rMat[0][1] * bodyRay[1]) + (rMat[0][2] * bodyRay[2]);
    debugData->earthRay[1] = (rMat[1][0] * bodyRay[0]) + (rMat[1][1] * bodyRay[1]) + (rMat[1][2] * bodyRay[2]);
    debugData->earthRay[2] = (rMat[2][0] * bodyRay[0]) + (rMat[2][1] * bodyRay[1]) + (rMat[2][2] * bodyRay[2]);

    debugData->headingEfDeg = RADIANS_TO_DEGREES(atan2_approx(debugData->earthRay[1], debugData->earthRay[0]));
    const float horizontalMagnitude = sqrtf((debugData->earthRay[0] * debugData->earthRay[0])
        + (debugData->earthRay[1] * debugData->earthRay[1]));
    debugData->elevationEfDeg = RADIANS_TO_DEGREES(atan2_approx(debugData->earthRay[2], horizontalMagnitude));
    return true;
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
