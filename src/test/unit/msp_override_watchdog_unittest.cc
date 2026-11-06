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

#include <limits.h>

extern "C" {
#include "rx/msp_override_watchdog.h"
}

#include "gtest/gtest.h"

namespace {

constexpr uint32_t kMask = MSP_OVERRIDE_REQUIRED_CHANNEL_MASK;
constexpr uint16_t kTimeoutMs = 100;
constexpr timeUs_t kTimeoutUs = 100000;
constexpr timeUs_t kStartUs = 1000;

mspOverrideWatchdogState_t readyWatchdog()
{
    mspOverrideWatchdogState_t state;
    mspOverrideWatchdogInit(&state);
    EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, kStartUs, false, kMask, false, kTimeoutMs));
    return state;
}

TEST(MspOverrideWatchdogTest, RejectsHighAtBootUntilOffIsObserved)
{
    mspOverrideWatchdogState_t state;
    mspOverrideWatchdogInit(&state);

    EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, kStartUs, true, kMask, true, kTimeoutMs));
    EXPECT_FALSE(state.permitted);
    EXPECT_TRUE(state.timeoutLatched);

    EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, kStartUs + 1, false, kMask, false, kTimeoutMs));
    EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, kStartUs + 2, true, kMask, false, kTimeoutMs));
    EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, kStartUs + 3, true, kMask, true, kTimeoutMs));
    EXPECT_TRUE(state.permitted);
}

TEST(MspOverrideWatchdogTest, RequiresAFrameAfterTheRisingEdge)
{
    mspOverrideWatchdogState_t state = readyWatchdog();

    EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, kStartUs + 1, true, kMask, false, kTimeoutMs));
    EXPECT_FALSE(state.permitted);
    EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, kStartUs + 2, true, kMask, true, kTimeoutMs));
    EXPECT_TRUE(state.permitted);
}

TEST(MspOverrideWatchdogTest, ExpiresAtTheExactDeadline)
{
    mspOverrideWatchdogState_t state = readyWatchdog();
    const timeUs_t frameTimeUs = kStartUs + 10;

    EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, frameTimeUs, true, kMask, true, kTimeoutMs));
    EXPECT_FALSE(mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs - 1, true, kMask, false, kTimeoutMs));
    EXPECT_TRUE(state.permitted);

    EXPECT_TRUE(mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs, true, kMask, false, kTimeoutMs));
    EXPECT_FALSE(state.permitted);
    EXPECT_TRUE(state.timeoutLatched);
    EXPECT_EQ(1U, state.expiryCount);
}

TEST(MspOverrideWatchdogTest, UsesConfiguredMillisecondsAtTheDeadline)
{
    for (uint16_t timeoutMs : { 1, 75, 250, 1000 }) {
        auto state = readyWatchdog();
        const timeUs_t deadline = kStartUs + timeoutMs * 1000U;
        EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, kStartUs, true, kMask, true, timeoutMs));
        EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, deadline - 1, true, kMask, false, timeoutMs));
        EXPECT_TRUE(state.permitted);
        EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, deadline, true, kMask, true, timeoutMs));
        EXPECT_TRUE(state.timeoutLatched);
    }
}

TEST(MspOverrideWatchdogTest, IgnoresFramesAfterTimeoutUntilOffIsObserved)
{
    mspOverrideWatchdogState_t state = readyWatchdog();
    const timeUs_t frameTimeUs = kStartUs + 10;
    mspOverrideWatchdogUpdate(&state, frameTimeUs, true, kMask, true, kTimeoutMs);
    mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs, true, kMask, false, kTimeoutMs);

    EXPECT_FALSE(mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs + 1, true, kMask, true, kTimeoutMs));
    EXPECT_FALSE(state.permitted);

    mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs + 2, false, kMask, true, kTimeoutMs);
    EXPECT_FALSE(state.permitted);
    EXPECT_TRUE(mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs + 3, true, kMask, true, kTimeoutMs));
    EXPECT_TRUE(state.permitted);
}

TEST(MspOverrideWatchdogTest, LateFrameCannotRefreshAnExpiredHighSession)
{
    mspOverrideWatchdogState_t state = readyWatchdog();
    const timeUs_t frameTimeUs = kStartUs + 10;
    mspOverrideWatchdogUpdate(&state, frameTimeUs, true, kMask, true, kTimeoutMs);

    EXPECT_TRUE(mspOverrideWatchdogUpdate(
        &state, frameTimeUs + kTimeoutUs, true, kMask, true, kTimeoutMs));
    EXPECT_FALSE(state.permitted);
    EXPECT_TRUE(state.timeoutLatched);
    EXPECT_EQ(1U, state.expiryCount);
}

TEST(MspOverrideWatchdogTest, RequiresTheExactStickChannelMask)
{
    for (const uint32_t mask : { 0x0000U, 0x0007U, 0x001FU }) {
        mspOverrideWatchdogState_t state = readyWatchdog();
        EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, kStartUs + 1, true, mask, true, kTimeoutMs));
        EXPECT_FALSE(state.permitted);
        EXPECT_TRUE(state.timeoutLatched);
    }
}

TEST(MspOverrideWatchdogTest, HandlesMicrosWraparound)
{
    mspOverrideWatchdogState_t state = readyWatchdog();
    const timeUs_t frameTimeUs = UINT32_MAX - 50000U;

    EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, frameTimeUs, true, kMask, true, kTimeoutMs));
    EXPECT_FALSE(mspOverrideWatchdogUpdate(&state, 49998U, true, kMask, false, kTimeoutMs));
    EXPECT_TRUE(state.permitted);
    EXPECT_TRUE(mspOverrideWatchdogUpdate(&state, 49999U, true, kMask, false, kTimeoutMs));
    EXPECT_FALSE(state.permitted);
}

TEST(MspOverrideWatchdogTest, ReportsFrameAgeAndPreservesExpiryCount)
{
    mspOverrideWatchdogState_t state = readyWatchdog();
    EXPECT_EQ(UINT32_MAX, mspOverrideWatchdogFrameAgeUs(&state, kStartUs));

    mspOverrideWatchdogUpdate(&state, kStartUs + 10, true, kMask, true, kTimeoutMs);
    EXPECT_EQ(25U, mspOverrideWatchdogFrameAgeUs(&state, kStartUs + 35));
    mspOverrideWatchdogUpdate(
        &state, kStartUs + 10 + kTimeoutUs, true, kMask, false, kTimeoutMs);
    mspOverrideWatchdogUpdate(&state, kStartUs + 20 + kTimeoutUs, false, kMask, false, kTimeoutMs);
    EXPECT_EQ(1U, state.expiryCount);
}

} // namespace
