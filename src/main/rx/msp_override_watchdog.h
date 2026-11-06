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

#define MSP_OVERRIDE_REQUIRED_CHANNEL_MASK 0x000FU
#define MSP_OVERRIDE_WATCHDOG_TIMEOUT_US 100000U

typedef struct mspOverrideWatchdogState_s {
    timeUs_t lastFrameTimeUs;
    uint32_t expiryCount;
    bool modeWasActive;
    bool offObserved;
    bool frameObserved;
    bool permitted;
    bool timeoutLatched;
} mspOverrideWatchdogState_t;

void mspOverrideWatchdogInit(mspOverrideWatchdogState_t *state);
bool mspOverrideWatchdogUpdate(
    mspOverrideWatchdogState_t *state,
    timeUs_t currentTimeUs,
    bool overrideModeActive,
    uint32_t channelMask,
    bool validOverrideFrameReceived);
uint32_t mspOverrideWatchdogFrameAgeUs(
    const mspOverrideWatchdogState_t *state,
    timeUs_t currentTimeUs);
