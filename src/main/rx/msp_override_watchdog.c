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

#include "rx/msp_override_watchdog.h"

void mspOverrideWatchdogInit(mspOverrideWatchdogState_t *state)
{
    *state = (mspOverrideWatchdogState_t) { 0 };
}

bool mspOverrideWatchdogUpdate(
    mspOverrideWatchdogState_t *state,
    timeUs_t currentTimeUs,
    bool overrideModeActive,
    uint32_t channelMask,
    bool validOverrideFrameReceived,
    uint16_t timeoutMs)
{
    const bool wasPermitted = state->permitted;

    if (!overrideModeActive) {
        state->modeWasActive = false;
        state->offObserved = true;
        state->frameObserved = false;
        state->permitted = false;
        state->timeoutLatched = false;
        return wasPermitted;
    }

    if (!state->modeWasActive) {
        state->modeWasActive = true;
        state->frameObserved = false;
        state->permitted = false;
        state->timeoutLatched = !state->offObserved;
        state->offObserved = false;
    }

    if (channelMask != MSP_OVERRIDE_REQUIRED_CHANNEL_MASK) {
        state->permitted = false;
        state->timeoutLatched = true;
        return wasPermitted;
    }

    if (state->permitted
        && cmpTimeUs(currentTimeUs, state->lastFrameTimeUs) >= (timeDelta_t)(timeoutMs * 1000U)) {
        state->permitted = false;
        state->timeoutLatched = true;
        state->expiryCount++;
    }

    if (!state->timeoutLatched && validOverrideFrameReceived) {
        state->lastFrameTimeUs = currentTimeUs;
        state->frameObserved = true;
        state->permitted = true;
    }

    return wasPermitted != state->permitted;
}

uint32_t mspOverrideWatchdogFrameAgeUs(
    const mspOverrideWatchdogState_t *state,
    timeUs_t currentTimeUs)
{
    if (!state->frameObserved) {
        return UINT32_MAX;
    }

    const timeDelta_t ageUs = cmpTimeUs(currentTimeUs, state->lastFrameTimeUs);
    return ageUs > 0 ? (uint32_t)ageUs : 0;
}
