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

#if defined(USE_RX_MSP_OVERRIDE)

#include "rx/msp_override.h"
#include "rx/msp.h"
#include "rx/msp_override_watchdog.h"
#include "fc/rc_modes.h"
#include "common/maths.h"

static mspOverrideWatchdogState_t mspOverrideWatchdog;

uint16_t rxMspOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t rxSample = (rxRuntimeState->rcReadRawFn)(rxRuntimeState, chan);

    uint16_t overrideSample = constrainf(rxMspReadRawRC(rxRuntimeState, chan), rxConfig->rx_min_usec, rxConfig->rx_max_usec);

    bool override = (1 << chan) & rxConfig->msp_override_channels_mask;

    if (rxMspOverrideIsActive() && override) {
        return overrideSample;
    } else {
        return rxSample;
    }
}

bool rxMspOverrideWatchdogUpdate(timeUs_t currentTimeUs, bool validOverrideFrameReceived)
{
    return mspOverrideWatchdogUpdate(
        &mspOverrideWatchdog,
        currentTimeUs,
        IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE),
        rxConfig()->msp_override_channels_mask,
        validOverrideFrameReceived,
        rxConfig()->msp_override_watchdog_timeout_ms);
}

bool rxMspOverrideIsActive(void)
{
    return mspOverrideWatchdog.permitted;
}

bool rxMspOverrideWatchdogIsLatched(void)
{
    return mspOverrideWatchdog.timeoutLatched;
}

uint32_t rxMspOverrideWatchdogFrameAgeUs(timeUs_t currentTimeUs)
{
    return mspOverrideWatchdogFrameAgeUs(&mspOverrideWatchdog, currentTimeUs);
}

uint32_t rxMspOverrideWatchdogExpiryCount(void)
{
    return mspOverrideWatchdog.expiryCount;
}

bool isMspOverrideControllingSticks(void)
{
    if (!rxMspOverrideIsActive()) {
        return false;
    }

    return rxConfig()->msp_override_channels_mask == MSP_OVERRIDE_REQUIRED_CHANNEL_MASK;
}

#endif
