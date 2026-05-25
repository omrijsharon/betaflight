#include <stdint.h>

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
    cfg->alt_est_flags = ALT_EST_CONFIG_DELAYED_FUSION | ALT_EST_CONFIG_ADAPTIVE_R | ALT_EST_CONFIG_BARO_STEP | ALT_EST_CONFIG_TILT_R | ALT_EST_CONFIG_ACCEL_R;
    cfg->alt_est_accel_noise_cms2 = 35;
    cfg->alt_est_accel_bias_noise_cms2 = 2;
    cfg->alt_est_accel_bias_limit_cms2 = 200;
    cfg->alt_est_baro_noise_cm = 150;
    cfg->alt_est_baro_delay_ms = 60;
    cfg->alt_est_innov_var_floor_cm2 = 400;
    cfg->alt_est_history_ms = 300;
    cfg->alt_est_gate_sigma_x10 = 50;
    cfg->alt_est_recovery_start_frames = 30;
    cfg->alt_est_recovery_r_scale_x10 = 500;
    cfg->alt_est_recovery_decay_tc_frames = 20;
    cfg->alt_est_step_innov_thresh_cm = 150;
    cfg->alt_est_step_rate_thresh_cms = 200;
    cfg->alt_est_step_rate_filter_tau_ms = 150;
    cfg->alt_est_step_streak_frames = 5;
    cfg->alt_est_step_offset_alpha_x1000 = 100;
    cfg->alt_est_step_offset_limit_cm = 300;
    cfg->alt_est_height_rate_lpf_hz_x100 = 200;
    cfg->alt_est_pz_floor_cm = 2;
    cfg->alt_est_pv_floor_cms = 2;
    cfg->alt_est_pba_floor_cms2 = 1;
    cfg->alt_est_tilt_r_start_deg = 45;
    cfg->alt_est_tilt_r_end_deg = 75;
    cfg->alt_est_tilt_r_scale_x10 = 50;
    cfg->alt_est_accel_r_start_cms2 = 300;
    cfg->alt_est_accel_r_end_cms2 = 800;
    cfg->alt_est_accel_r_scale_x10 = 50;
    altitudeEstimatorInit();
}

static void armAtDatum(timeUs_t timeUs, float baroCm)
{
    altitudeEstimatorUpdate(timeUs, false, true, baroCm, timeUs, true, 0.0f);
    altitudeEstimatorUpdate(timeUs + 10000, true, true, baroCm, timeUs + 10000, true, 0.0f);
}

static altitudeEstimatorMeasurement_t makeMeasurement(
    timeUs_t timeUs,
    altitudeEstimatorMeasurementSource_e source,
    altitudeEstimatorMeasurementType_e type,
    float valueCm,
    float varianceCm2 = 0.0f)
{
    altitudeEstimatorMeasurement_t measurement = {};
    measurement.timeUs = timeUs;
    measurement.source = source;
    measurement.type = type;
    measurement.valueCm = valueCm;
    measurement.varianceCm2 = varianceCm2;
    measurement.quality = UINT8_MAX;
    measurement.flags = ALT_EST_MEAS_FLAG_VALID;
    return measurement;
}

static altitudeEstimatorInput_t makeInput(bool haveAccel = true, float accelWorldZ = 0.0f, bool haveTilt = false, float cosTilt = 1.0f)
{
    altitudeEstimatorInput_t input = {};
    input.haveAccel = haveAccel;
    input.accelWorldZ = accelWorldZ;
    input.haveTilt = haveTilt;
    input.cosTilt = cosTilt;
    return input;
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

TEST(AltitudeEstimatorTest, FusesBaroThroughSourceAgnosticMeasurement)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_flags = 0;

    altitudeEstimatorMeasurement_t baro = makeMeasurement(100000, ALT_EST_MEAS_SOURCE_BAROMETER, ALT_EST_MEAS_TYPE_ALTITUDE, 1000.0f);
    altitudeEstimatorInput_t input = makeInput();
    altitudeEstimatorUpdateWithMeasurements(100000, false, &baro, 1, &input);
    baro.timeUs = 110000;
    altitudeEstimatorUpdateWithMeasurements(110000, true, &baro, 1, &input);

    timeUs_t now = 120000;
    for (int i = 0; i < 50; i++) {
        now += 10000;
        baro = makeMeasurement(now, ALT_EST_MEAS_SOURCE_BAROMETER, ALT_EST_MEAS_TYPE_ALTITUDE, 1100.0f);
        altitudeEstimatorUpdateWithMeasurements(now, true, &baro, 1, &input);
    }

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_VALID);
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_FUSED);
    EXPECT_GT(status->altitudeCm, 10.0f);
}

TEST(AltitudeEstimatorTest, AcceptsFutureMeasurementSourcesWithoutFusion)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_flags = 0;
    armAtDatum(100000, 1000.0f);

    altitudeEstimatorMeasurement_t measurements[2];
    measurements[0] = makeMeasurement(120000, ALT_EST_MEAS_SOURCE_GPS, ALT_EST_MEAS_TYPE_ALTITUDE, 1250.0f, 2500.0f);
    measurements[1] = makeMeasurement(120000, ALT_EST_MEAS_SOURCE_RANGEFINDER, ALT_EST_MEAS_TYPE_SURFACE_DISTANCE, 180.0f, 100.0f);

    altitudeEstimatorInput_t input = makeInput();
    altitudeEstimatorUpdateWithMeasurements(120000, true, measurements, 2, &input);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_GPS_ALT_VALID);
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_RANGEFINDER_VALID);
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_BARO_FUSED);
}

TEST(AltitudeEstimatorTest, ConfigurableCovarianceFloorConstrainsInnovationVariance)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = 0;
    cfg->alt_est_baro_noise_cm = 1;
    cfg->alt_est_innov_var_floor_cm2 = 1;
    cfg->alt_est_gate_sigma_x10 = 100;
    cfg->alt_est_pz_floor_cm = 100;
    cfg->alt_est_pv_floor_cms = 50;
    cfg->alt_est_pba_floor_cms2 = 10;
    altitudeEstimatorInit();
    armAtDatum(100000, 1000.0f);

    altitudeEstimatorUpdate(120000, true, true, 1000.0f, 120000, true, 0.0f);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_FUSED);
    EXPECT_GE(status->sCm2, 10000.0f);
}

TEST(AltitudeEstimatorTest, InflatesBaroRForTilt)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = ALT_EST_CONFIG_TILT_R;
    cfg->alt_est_baro_noise_cm = 100;
    cfg->alt_est_tilt_r_start_deg = 45;
    cfg->alt_est_tilt_r_end_deg = 75;
    cfg->alt_est_tilt_r_scale_x10 = 50;
    altitudeEstimatorInit();
    armAtDatum(100000, 1000.0f);

    altitudeEstimatorMeasurement_t baro = makeMeasurement(120000, ALT_EST_MEAS_SOURCE_BAROMETER, ALT_EST_MEAS_TYPE_ALTITUDE, 1000.0f);
    altitudeEstimatorInput_t input = makeInput(true, 0.0f, true, 0.0f);
    altitudeEstimatorUpdateWithMeasurements(120000, true, &baro, 1, &input);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_TILT_R_INFLATED);
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_ACCEL_R_INFLATED);
    EXPECT_GT(status->rEffCm2, 40000.0f);
}

TEST(AltitudeEstimatorTest, InflatesBaroRForHighVerticalAcceleration)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = ALT_EST_CONFIG_ACCEL_R;
    cfg->alt_est_baro_noise_cm = 100;
    cfg->alt_est_accel_r_start_cms2 = 300;
    cfg->alt_est_accel_r_end_cms2 = 800;
    cfg->alt_est_accel_r_scale_x10 = 50;
    altitudeEstimatorInit();
    armAtDatum(100000, 1000.0f);

    altitudeEstimatorMeasurement_t baro = makeMeasurement(120000, ALT_EST_MEAS_SOURCE_BAROMETER, ALT_EST_MEAS_TYPE_ALTITUDE, 1000.0f);
    altitudeEstimatorInput_t input = makeInput(true, 10.0f, false, 1.0f);
    altitudeEstimatorUpdateWithMeasurements(120000, true, &baro, 1, &input);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_TILT_R_INFLATED);
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_ACCEL_R_INFLATED);
    EXPECT_GT(status->rEffCm2, 40000.0f);
}

TEST(AltitudeEstimatorTest, AdaptiveRInflationCanBeDisabled)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = 0;
    cfg->alt_est_baro_noise_cm = 100;
    altitudeEstimatorInit();
    armAtDatum(100000, 1000.0f);

    altitudeEstimatorMeasurement_t baro = makeMeasurement(120000, ALT_EST_MEAS_SOURCE_BAROMETER, ALT_EST_MEAS_TYPE_ALTITUDE, 1000.0f);
    altitudeEstimatorInput_t input = makeInput(true, 10.0f, true, 0.0f);
    altitudeEstimatorUpdateWithMeasurements(120000, true, &baro, 1, &input);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_TILT_R_INFLATED);
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_ACCEL_R_INFLATED);
    EXPECT_NEAR(status->rEffCm2, 10000.0f, 1.0f);
}

TEST(AltitudeEstimatorTest, RejectsLargeInnovationBeforeRecovery)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = 0;
    cfg->alt_est_baro_noise_cm = 20;
    cfg->alt_est_gate_sigma_x10 = 10;
    cfg->alt_est_recovery_start_frames = 100;
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

    altitudeEstimatorUpdate(now + 10000, true, true, 1020.0f, now + 10000, true, 0.0f);

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_TRUE(status->flags & ALT_EST_STATUS_BARO_FUSED);
    EXPECT_FALSE(status->flags & ALT_EST_STATUS_HISTORY_MISS);
}

TEST(AltitudeEstimatorTest, BiasIsLimited)
{
    resetEstimatorConfig();
    altitudeEstimatorConfig_t *cfg = altitudeEstimatorConfigMutable();
    cfg->alt_est_flags = 0;
    cfg->alt_est_accel_bias_limit_cms2 = 5;
    cfg->alt_est_baro_noise_cm = 20;
    armAtDatum(100000, 1000.0f);

    timeUs_t now = 120000;
    for (int i = 0; i < 100; i++) {
        now += 10000;
        altitudeEstimatorUpdate(now, true, true, 1000.0f, now, true, 2.0f);
    }

    const altitudeEstimatorStatus_t *status = altitudeEstimatorGetStatus();
    EXPECT_LE(status->accelBiasCms2, 5.0f);
    EXPECT_GE(status->accelBiasCms2, -5.0f);
}

TEST(AltitudeEstimatorTest, PositionRateOutputFollowsVelocity)
{
    resetEstimatorConfig();
    altitudeEstimatorConfigMutable()->alt_est_flags = 0;
    armAtDatum(100000, 1000.0f);

    timeUs_t now = 120000;
    for (int i = 0; i < 80; i++) {
        now += 10000;
        altitudeEstimatorUpdate(now, true, false, 0.0f, now, true, 0.8f);
    }

    EXPECT_GT(altitudeEstimatorGetPositionRateCmsFloat(), 1.0f);
}
