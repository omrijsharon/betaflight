#include <stdint.h>
#include <limits>

extern "C" {
    #include "platform.h"
    #include "flight/altitude_estimator.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static void resetEstimatorConfig(void)
{
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = ALT_EST_CONFIG_DELAYED_FUSION;
    cfg->alt_est_accel_noise_cms2 = 35;
    cfg->alt_est_accel_bias_noise_cms2 = 2;
    cfg->alt_est_accel_bias_limit_cms2 = 200;
    cfg->alt_est_baro_noise_cm = 150;
    cfg->alt_est_baro_delay_ms = 60;
    cfg->alt_est_history_ms = 250;
    cfg->alt_est_gate_sigma_x10 = 50;
    altitudeEstimatorInit();
}

static void armAtDatum(timeUs_t timeUs, float baroCm)
{
    altitudeEstimatorUpdate(timeUs, false, true, baroCm, timeUs, true, 0.0f);
    altitudeEstimatorUpdate(timeUs + 10000, true, true, baroCm, timeUs + 10000, true, 0.0f);
}

TEST(AltitudeEstimatorTest, PredictsVerticalVelocityFromAcceleration)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_flags = 0;
    armAtDatum(100000, 1000.0f);

    timeUs_t now = 120000;
    for (int i = 0; i < 100; i++) {
        now += 10000;
        altitudeEstimatorUpdate(now, true, false, 0.0f, now, true, 1.0f);
    }

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_NEAR(status->velocityCms, 100.0f, 8.0f);
    EXPECT_GT(status->altitudeCm, 40.0f);
}

TEST(AltitudeEstimatorTest, FusesBaroMeasurements)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_flags = 0;
    armAtDatum(100000, 1000.0f);

    timeUs_t now = 120000;
    for (int i = 0; i < 50; i++) {
        now += 10000;
        altitudeEstimatorUpdate(now, true, true, 1100.0f, now, true, 0.0f);
    }

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_FUSED);
    EXPECT_GT(status->altitudeCm, 10.0f);
}

TEST(AltitudeEstimatorTest, FreefallIntegratesMinusGravity)
{
    resetEstimatorConfig();
    armAtDatum(100000, 1000.0f);
    for (unsigned n = 1; n <= 100; n++)
        altitudeEstimatorUpdate(110000 + n * 10000, true, false, 0, 0, true, -9.80665f);
    EXPECT_NEAR(altitudeEstimatorGetVelocityCmsFloat(), -980.665f, 0.01f);
    EXPECT_NEAR(altitudeEstimatorGetAltitudeCmFloat(), -490.3325f, 0.01f);
}

TEST(AltitudeEstimatorTest, DuplicateStaleAndFutureMeasurementsAreRejected)
{
    resetEstimatorConfig();
    armAtDatum(100000, 1000.0f);
    altitudeEstimatorUpdate(120000, true, true, 1000, 120000, true, 0);
    altitudeEstimatorUpdate(130000, true, true, 1000, 120000, true, 0);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BAD_TIME);
    altitudeEstimatorUpdate(140000, true, true, 1000, 150000, true, 0);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BAD_TIME);
    for (unsigned t = 150000; t <= 500000; t += 10000)
        altitudeEstimatorUpdate(t, true, false, 0, 0, true, 0);
    altitudeEstimatorUpdate(510000, true, true, 1000, 160000, true, 0);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_STALE);
    EXPECT_FALSE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BARO_FUSED);
}

TEST(AltitudeEstimatorTest, TimeWrapDoesNotCreateIntegrationJump)
{
    resetEstimatorConfig();
    const timeUs_t t = (timeUs_t)0xffff0000u;
    armAtDatum(t, 1000);
    for (unsigned n = 1; n <= 100; n++)
        altitudeEstimatorUpdate(t + 10000 + n * 10000, true, false, 0, 0, true, 1);
    EXPECT_NEAR(altitudeEstimatorGetVelocityCmsFloat(), 100, 0.01);
}

TEST(AltitudeEstimatorTest, RejectsInvalidAccelerationAndDoesNotIntegrateTimeGap)
{
    resetEstimatorConfig();
    armAtDatum(100000, 1000);
    altitudeEstimatorUpdate(120000, true, false, 0, 0, false, 0);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_INVALID_INPUT);
    altitudeEstimatorUpdate(500000, true, false, 0, 0, true, 1);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BAD_TIME);
    EXPECT_FLOAT_EQ(altitudeEstimatorGetVelocityCmsFloat(), 0);
}

TEST(AltitudeEstimatorTest, RejectsLargeInnovation)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = 0;
    cfg->alt_est_baro_noise_cm = 20;
    cfg->alt_est_gate_sigma_x10 = 10;
    armAtDatum(100000, 1000.0f);

    altitudeEstimatorUpdate(130000, true, true, 5000.0f, 130000, true, 0.0f);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_REJECT);
    EXPECT_EQ(status->rejectStreak, 1);
}

TEST(AltitudeEstimatorTest, DelayedFusionUsesHistory)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_baro_delay_ms = 60;
    armAtDatum(100000, 1000.0f);

    timeUs_t now = 120000;
    for (int i = 0; i < 20; i++) {
        now += 10000;
        altitudeEstimatorUpdate(now, true, true, 1000.0f, now, true, 0.0f);
    }

    // Core accepts effective timestamps; driver delay is applied by position.c.
    // Start a new run so earlier current-time observations cannot make this
    // delayed sample out-of-order.
    resetEstimatorConfig();
    armAtDatum(100000, 1000.0f);
    for (timeUs_t t = 120000; t <= now; t += 10000)
        altitudeEstimatorUpdate(t, true, false, 0, 0, true, 0);
    altitudeEstimatorUpdate(now + 10000, true, true, 1020.0f, now - 50000, true, 0.0f);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_FUSED);
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_HISTORY_MISS);
}

TEST(AltitudeEstimatorTest, RejectionNeverForcesFusionOrMovesBarometerDatum)
{
    resetEstimatorConfig();
    armAtDatum(100000, 1000);
    for (unsigned n = 1; n <= 200; n++) {
        altitudeEstimatorUpdate(110000 + n * 10000, true, true, 1000000, 110000 + n * 10000, true, 0);
        const auto *status = altitudeEstimatorGetStatus();
        EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_REJECT);
        EXPECT_FALSE(status->flags & ALT_EST_STATUS_BARO_FUSED);
        EXPECT_FLOAT_EQ(status->altitudeCm, 0);
    }
}

TEST(AltitudeEstimatorTest, MeasurementOutsideStoredHistoryIsNotFusedAtWrongTime)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_history_ms = 20;
    armAtDatum(100000, 1000);
    for (unsigned t = 120000; t <= 300000; t += 10000)
        altitudeEstimatorUpdate(t, true, false, 0, 0, true, 0);
    altitudeEstimatorUpdate(310000, true, true, 1000, 200000, true, 0);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_HISTORY_MISS);
    EXPECT_FALSE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BARO_FUSED);
}

TEST(AltitudeEstimatorTest, FullHistoryWindowSurvivesRepeatedEviction)
{
    resetEstimatorConfig();
    armAtDatum(100000, 1000);
    for (unsigned n = 1; n <= 1000; n++) {
        const timeUs_t now = 110000 + n * 10000;
        const bool haveBaro = n > 30;
        // Alternate an exact oldest timestamp with a fractional one. Both
        // require the full window after many passes through buffer eviction.
        const timeUs_t captured = now - ((n & 1) ? 250000 : 245000);
        altitudeEstimatorUpdate(now, true, haveBaro, 1000, captured, true, 0);
        if (haveBaro) {
            EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BARO_FUSED);
            EXPECT_FALSE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_HISTORY_MISS);
            EXPECT_FLOAT_EQ(altitudeEstimatorGetAltitudeCmFloat(), 0);
        }
    }
}

TEST(AltitudeEstimatorTest, NonfiniteMeasurementsDoNotPoisonState)
{
    resetEstimatorConfig();
    armAtDatum(100000, 1000);
    altitudeEstimatorUpdate(120000, true, false, 0, 0, true, std::numeric_limits<float>::quiet_NaN());
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_INVALID_INPUT);
    EXPECT_FLOAT_EQ(altitudeEstimatorGetVelocityCmsFloat(), 0);
    altitudeEstimatorUpdate(130000, true, true, std::numeric_limits<float>::infinity(), 130000, true, 0);
    EXPECT_TRUE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_INVALID_INPUT);
    EXPECT_FALSE(altitudeEstimatorGetStatus()->flags & ALT_EST_STATUS_BARO_FUSED);
    EXPECT_FLOAT_EQ(altitudeEstimatorGetVelocityCmsFloat(), 0);
}
