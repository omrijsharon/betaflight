# Simulator vertical filter port — 2026-11-04

The `feature/altitude-estimator` filter now follows `betaflight-sim` commit 8ed71d7,
`sim/vertical.py` and `sim/delayed_estimator.py`. This is an estimator change,
not an altitude-control mode or a hardware flight validation.

## Model and timing

States are altitude (m, up positive), velocity (m/s) and acceleration bias (m/s²).
`position.c` rotates sampled proper acceleration using BF attitude and subtracts
9.80665 m/s². Freefall therefore predicts -g; level rest predicts zero.
The existing 100 Hz altitude task remains. Predictions use actual elapsed time;
zero/backward time and gaps above 200 ms are flagged, not silently clamped.
Missing/nonfinite acceleration invalidates prediction and clears replay history.

Covariance uses F P F-transpose + Q and Joseph-form measurement correction.
Priors match Python: standard deviations 2 m, 1 m/s, 0.5 m/s².
Acceleration noise is per-sample standard deviation; bias noise is random walk.
Noise/delay defaults remain the existing hardware assumptions (0.35 m/s²,
0.02 m/s²/sqrt(s), 1.5 m baro, 60 ms delay), not a claim of hardware tuning.
The parity test gives both implementations the SAME settings (baro 0.2 m,
bias random walk 0.02) and SAME 100 Hz/jittered inputs.

The barometer driver publishes a monotonically changing sample number and receipt
timestamp only after a valid calibrated pressure conversion. `position.c` consumes
each generation once, including equal-valued new readings. Receipt time minus
configured sensor delay supplies the effective timestamp; the filter does not
subtract that delay again. This is an estimated sample time, not a claim that
hardware conversion/filter delay has been measured. A sensor faster than the
altitude task can have intermediate samples skipped; samples are never duplicated.

Delayed correction uses the last state before the observation, predicts any
fractional interval, corrects there, then replays acceleration to the present.
Duplicate/out-of-order, future, >250 ms old, out-of-history and innovation-gated
samples are rejected. Consumed timestamps include gated/stale measurements.
History is bounded to 32 entries and min(configured history, 250 ms), retaining
one older anchor. At 100 Hz this covers the supported window. Raising prediction
rate requires reviewing history capacity and MCU CPU/RAM cost.

The launch datum is acquired while disarmed; disarmed state/velocity remain zero.
Arming retains that datum and creates a fresh replay anchor. Measurements before
that anchor are rejected. If no datum was acquired, NO_DATUM reports that condition;
a height reference is not silently invented while armed.

## Configuration and protocol cleanup

Removed the old four-state EKF type, all 16 `ekf_*` settings, the old update alias
and MSP commands 231/232. The filter has only eight active settings:

| CLI name | Default | Meaning |
| --- | ---: | --- |
| `alt_est_flags` | 1 | Bit 0: delayed fusion enabled; 0 disables it |
| `alt_est_accel_noise_cms2` | 35 | Acceleration sample noise standard deviation |
| `alt_est_accel_bias_noise_cms2` | 2 | Acceleration bias random-walk standard deviation |
| `alt_est_accel_bias_limit_cms2` | 200 | Diagnostic threshold; does not clamp state |
| `alt_est_baro_noise_cm` | 150 | Barometer altitude noise standard deviation |
| `alt_est_baro_delay_ms` | 60 | Receipt-to-effective-sample delay, range 0..250 ms |
| `alt_est_history_ms` | 250 | Replay history age, range 20..250 ms |
| `alt_est_gate_sigma_x10` | 50 | Innovation gate in tenths of standard deviations |

History previously defaulted to 300 ms but was already capped internally at
250 ms. The new default and CLI/MSP limits match that cap. The buffer is reduced from 128 to 32 entries: the 100 Hz task requires 26
states for the full window, plus an older anchor. This saves 5,376 bytes of
state storage on the target. Faster prediction rates must revisit capacity.

Removed 20 inactive `alt_est_*` controls, adaptive-R/step/recovery flags and
counters, covariance floor settings, redundant position-rate and constant
barometer-offset outputs. Blackbox headers and both frame encoders omit those
channels. Unused generic GPS/rangefinder/quality/tilt input structures are gone;
the core accepts barometer height/time and world-Z acceleration directly.
OSD/vario filters and `ALTITUDE_ESTIMATOR` debug channels remain.

Parameter-group versions are altitude-estimator **1** and position **13**.
Loading older saved data resets these two groups to current defaults without
migration. This includes position display/alt-hold settings in that group.
Other groups retain their normal loading behavior.

MSP2 command IDs remain 0x3010/0x3011/0x3012, with payload version **3**.
Config is exactly **16 bytes**: version u8, seven little-endian u16 fields in
the table order, gate u8. Status is exactly **53 bytes**: version u8, timeUs u32,
flags u32, ten signed i32 values (altitude, velocity, raw barometer altitude,
acceleration, acceleration bias, innovation, gate, R, S, barometer age), then
configured delay u16 and rejection streak u16. Units are cm, seconds and ms as
named in `msp_client.py`; R/S are cm squared.
Wrong version or length is rejected before config writes. Pi client, fake client,
plot defaults and analysis labels match the new schema. Old clients must be
updated; no old protocol or config decoder remains.

Status bits are 0 armed, 1 baro valid, 2 fused, 3 rejected, 5 reset, 7 history miss,
8 bias limit, 13 bad time, 14 stale, 15 invalid input, 16 no datum.
`baroAgeMs` follows the last consumed effective timestamp, grows between readings,
and is -1 before any consumed measurement.

## Verification

From this repo, in Linux/WSL:

```
make -C src/test test_altitude_estimator_unittest CC=gcc CXX=g++ OBJECT_DIR=/tmp/bf-altitude-tests USE_COVERAGE=
python3 -B src/test/altitude_estimator_parity.py --sim-repo ../betaflight-sim
python3 -B src/test/altitude_estimator_protocol.py
python -m pytest althold/tests -q
# From althold/tuning_app: npm test
```

The parity script builds the actual firmware C source with warnings as errors and `-ffast-math` in
a temporary directory. It compares all three states, all nine covariance entries,
and fusion decisions with the Python filter for 30-second deterministic cases:
freefall, bias learning, delayed noisy barometer, dropout/outlier/stale/duplicate
readings, and irregular prediction/fractional measurement timing. It checks
covariance symmetry and positive-semidefinite principal minors. The sensor adapter
is compiled too (only hardware/display dependencies stubbed) to verify freshness,
calibration, equal new readings, receipt/delay semantics and gravity convention.

Observed parity errors with barometer fusion: <0.00002 m altitude and
<0.00002 m/s velocity. Extreme 30-second uncorrected freefall spans ~4.4 km;
float-versus-double discrepancy is 0.047 m / 0.00391 m/s, with relative covariance
tolerance 1e-4. This does not establish real sensor accuracy or flight performance.

Validation on 2026-11-04: 45 unit tests passed (12 estimator, 17 IMU,
16 barometer driver), including a repeat with `-O2 -ffast-math`. The parity
and adapter checks above passed with `-ffast-math` and `-Werror` too.
The MSP check compiles actual handler bodies and the real stream buffer against
the Pi client, checking defaults, signed fields, round trips, clamping and all
truncated/config-version failures without partial writes. All 11 Python tooling
tests and all 4 TypeScript/backend tests passed.
NaN/infinity guards inspect float exponent bits because the production compiler
can optimize away ordinary `isfinite` calls under fast-math.

Before the compatibility cleanup, the complete JHEF405PRO / STM32F405 build
passed with the bundled
ARM GCC 10.3.1 toolchain (`make CONFIG=JHEF405PRO`). Final FLASH1 use was
501975 bytes; RAM 94712 bytes; CCM 15148 bytes. This build includes the pre-existing
local MSP/RX edits; they were not changed by this estimator task.

## Blitz F435 cleanup build

The final `IFLIGHT_BLITZ_F435` build passed using the user's requested cloud-build
flags, release name 4.5.3 and bundled ARM GCC 10.3.1. The source base remains
4.5.2, so the generated filename still contains 4.5.2.

| Region | Before cleanup | After cleanup | Saved |
| --- | ---: | ---: | ---: |
| FLASH1 | 422,440 B | 417,808 B | 4,632 B |
| RAM | 99,744 B | 94,184 B | 5,560 B |

The separate 524-byte FLASH region is unchanged. Estimator runtime storage is
1,936 bytes (was 7,324); estimator config/reset template is 16 bytes (was 54).
Binary symbols and strings confirm the compact structures and absence of old
EKF settings. The full-window eviction test exercises 245/250 ms observations
at 100 Hz for 1,000 predictions; the five simulator comparisons also pass after
the buffer reduction.

Artifact: `obj/betaflight_4.5.2_AT32F435G_IFLIGHT_BLITZ_F435.hex`.
SHA-256: `78f8f2007ea2f16171ff66f072da3fb4f41f99ef2f8f8a280261881a14ac302a`.
Build log: `build/iflight-blitz-f435-cleanup-final-20261104.log` (ignored).
This build includes the existing local MSP/RX changes. The RX watchdog is maintained separately from this estimator branch.

No flight controller was flashed or connected. Live-flight tuning, pressure delay,
vibration/attitude errors and CPU budget on the intended board remain to validate.

## Magnetometer-enabled Blitz F435 rebuild (2026-11-04)

At the user's request, rebuilt the same cleaned-up firmware with `-DUSE_MAG`
appended to the original EXTRA_FLAGS (including CLOUD_BUILD and release 4.5.3).
No source defaults or estimator settings changed. This fork's common target
configuration includes the supported compass drivers when USE_MAG is enabled.
The board config selects I2C1 for the magnetometer.

The build passed; ELF inspection confirms compass configuration and
`mag_hardware`, the cleaned estimator and release 4.5.3. All Intel HEX record
lengths/checksums and the EOF record were verified.

Artifact: `obj/betaflight_4.5.2_AT32F435G_IFLIGHT_BLITZ_F435_MAG.hex`.
SHA-256: `b47cd5a138494b017076c96cef8d3feae67c0daf38e9501dee2568aa9857461a`.
The normal generated HEX filename also contains this new build; the earlier
cleanup-only checksum above refers to the superseded artifact.
Build log: `build/iflight-blitz-f435-mag-20261104.log` (ignored).
Final FLASH1: 425,664 bytes (41.90%); RAM: 94,712 bytes (48.17%).
Magnetometer support adds 7,856 bytes FLASH1 and 528 bytes RAM compared with
the cleanup-only build. The separate 524-byte FLASH region is unchanged.
The agent did not flash the FC or test physical magnetometer detection.
