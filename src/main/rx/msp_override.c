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
#include "fc/rc_modes.h"
#include "common/maths.h"


uint16_t rxMspOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t rxSample = (rxRuntimeState->rcReadRawFn)(rxRuntimeState, chan);

    uint16_t overrideSample = constrainf(rxMspReadRawRC(rxRuntimeState, chan), rxConfig->rx_min_usec, rxConfig->rx_max_usec);

    bool override = (1 << chan) & rxConfig->msp_override_channels_mask;

    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override) {
        return overrideSample;
    } else {
        return rxSample;
    }
}


bool isMspOverrideControllingSticks(void)
{
    // Is the MSP override box active?
    if (!IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE)) {
        return false;
    }

    // For channels 0..3 (ROLL, PITCH, YAW, THROTTLE),
    // check if each one is included in msp_override_channels_mask.
    // i.e., if bits 0..3 of msp_override_channels_mask are set for each channel that we consider "controlled."
    // If you only require "any" of them to be overridden, adjust logic as desired.

    // Example: require *all* 4 sticks to be overridden
    const uint8_t stickMask = 0x0F;  // binary 1111, for channels 0..3
    uint8_t overrideMask = rxConfig()->msp_override_channels_mask & stickMask;

    // if overrideMask == 0x0F, that means channels 0..3 are all included
    // if partial override is enough for you, tweak this check.
    return (overrideMask == stickMask);
}

#endif

