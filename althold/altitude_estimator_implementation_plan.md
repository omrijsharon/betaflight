# Altitude Estimator Implementation Plan

Status reviewed: 2026-05-24.

Legend:

- `[x]` means implemented, documented, or verified for the scope stated on that line.
- `[ ]` means still open.
- A checked firmware implementation item does not mean flight-tuned or flight-validated unless the validation item is also checked.

This plan turns the altitude-estimator research notes into a staged Betaflight implementation path. Scope is altitude and vertical velocity estimation only. Altitude-control loops will be planned separately after the estimator/logging loop is working.

## [x] Source Documents

- [x] `althold/ardupilot_ekf3_altitude_estimation.md`
- [x] `althold/betaflight_altitude_estimator_proposal.md`

## [x] Implementation Goal

Implement a robust vertical estimator for Betaflight using:

- [x] Barometer height as the long-term altitude reference.
- [x] Calculated attitude and accelerometer data for short-term vertical acceleration and velocity.
- [x] No baro-derived velocity fusion.
- [x] Delayed baro fusion with a history/replay path.
- [x] Explicit estimator health, reset, gate, and source-status outputs.
- [x] Blackbox/MSP/Pi logging designed for tuning and analysis.

The estimator must provide:

- [x] Altitude, positive up.
- [x] Vertical velocity, positive up.
- [x] Position-consistent altitude-rate output.
- [x] Estimator status flags.
- [x] Reset/step/recovery event visibility in estimator status and logs.

## [x] Decisions Already Made

- [x] Do not port ArduPilot EKF3 as a full 24-state EKF.
- [x] Use a dedicated 1D vertical estimator.
- [x] Start from a 3-state EKF: `z`, `v`, and accelerometer bias `ba`.
- [x] Keep barometer offset/datum handling outside the EKF state.
- [x] Do not fuse differentiated baro as an independent velocity measurement.
- [x] Use positive-up values in Betaflight-facing APIs.
- [x] New altitude-control mode naming is `ALTITUDE_MODE` / `BOXALTITUDE`.
- [x] Old `BARO_MODE` / `BOXBARO` remain only as temporary aliases.
- [x] Build logs and tune tooling before trying to tune estimator gates in flight.

## [x] Resolved Coding Decisions

- [x] First estimator implementation includes delayed fusion with history/replay.
- [x] Persistent estimator parameter prefix is `alt_est_*`.
- [x] MSP2 command IDs are defined:
  - [x] `MSP2_BETAFLIGHT_ALT_EST_CONFIG = 0x3010`
  - [x] `MSP2_BETAFLIGHT_SET_ALT_EST_CONFIG = 0x3011`
  - [x] `MSP2_BETAFLIGHT_ALT_EST_STATUS = 0x3012`
- [x] Blackbox reuses altitude/baro logging integration and adds dedicated estimator fields.
- [x] Existing local `ekf_*` / `alt_hold_*` settings can remain temporarily, but the new estimator uses `alt_est_*`.
- [x] Estimator config lives in a new PG: `PG_ALTITUDE_ESTIMATOR_CONFIG = 559`.

## [x] Phase 1: Current-Code Reconciliation

Read the current local implementation and map all call sites before moving code:

- [x] `src/main/flight/position.c`
- [x] `src/main/flight/position.h`
- [x] `src/main/fc/tasks.c`
- [x] `src/main/sensors/barometer.c`
- [x] `src/main/fc/core.c`
- [x] `src/main/fc/runtime_config.h`
- [x] `src/main/fc/rc_modes.h`
- [x] `src/main/msp/msp.c`
- [x] `src/main/msp/msp_protocol.h`
- [x] `src/main/msp/msp_protocol_v2_betaflight.h`
- [x] `src/main/blackbox/blackbox.c`
- [x] `src/main/blackbox/blackbox_fielddefs.h`
- [x] `src/main/cli/settings.c`
- [x] `src/main/fc/parameter_names.h`

Expected output:

- [x] List of estimator inputs currently available.
- [x] List of altitude outputs currently consumed.
- [x] List of old `USE_BARO_ALTHOLD` paths to remove or isolate from estimator work.
- [x] Exact build flags that gate baro, vario, altitude, and old althold behavior.

## [ ] Phase 2: Estimator Module Boundary

Create a dedicated estimator module:

- [x] `src/main/flight/altitude_estimator.h`
- [x] `src/main/flight/altitude_estimator.c`
- [x] Include `flight/altitude_estimator.c` in `mk/source.mk`.
- [x] Add estimator source to relevant unit-test source lists.

Keep `position.c` as temporary glue only:

- [x] `calculateEstimatedAltitude()` feeds baro and IMU-derived vertical acceleration to the new estimator.
- [x] `getEstimatedAltitudeCm()` returns the new estimator output through the existing wrapper path.
- [x] `getAltitude()` returns the new estimator output.
- [x] `getEstimatedVario()` returns the new estimator vertical velocity path when `USE_VARIO` is enabled.
- [x] Old altitude controller output is disabled by forcing throttle altitude correction to zero.

Core public/runtime types:

- [x] Config struct for persistent tunables.
- [x] Runtime state for `z`, `v`, `ba`, covariance, output predictor, datum, and flags.
- [x] Source-agnostic timestamped measurement struct for future sensors.
- [x] Status/log struct that snapshots values needed by MSP and Blackbox.

## [ ] Phase 3: Estimator Config

Add estimator tunables with explicit units and conservative defaults:

- [x] Accelerometer process noise.
- [x] Accelerometer bias process noise.
- [x] Accelerometer bias limit.
- [x] Baro height noise.
- [x] Baro delay.
- [x] Height gate sigma.
- [x] Minimum innovation variance.
- [x] Recovery timeout/start threshold.
- [x] Recovery `R` inflation.
- [x] Baro step detector thresholds.
- [x] Baro offset step limits.
- [x] Height-rate complementary filter frequency.
- [x] Covariance floors as explicit configurable parameters.
- [x] Optional `R` inflation controls for tilt.
- [x] Optional `R` inflation controls for high vertical acceleration.
- [ ] Optional `R` inflation controls for takeoff/landing.
- [ ] Optional `R` inflation controls for vibration.

Use ArduPilot defaults as starting references:

- [x] Accelerometer process noise near `0.35 m/s^2`.
- [x] Accel bias process noise near `0.02 m/s^3`.
- [x] Baro noise initially `0.75 m` to `2.0 m`; current default is `1.50 m`.
- [x] Baro delay near `60 ms`.
- [x] Height gate initially `5 sigma`.
- [x] Height-rate filter near `2 Hz`.

## [ ] Phase 4: IMU Prediction

Implement prediction from vertical acceleration:

- [x] Convert calibrated accelerometer data to `m/s^2`.
- [x] Rotate body acceleration into world frame using the current attitude matrix.
- [x] Subtract gravity.
- [x] Subtract estimated vertical accelerometer bias.
- [x] Integrate altitude and velocity with real `dt`.
- [x] Clamp `dt` to survive scheduler jitter.
- [ ] Detect accelerometer clipping and set status flags.
- [ ] Detect invalid attitude and set status flags.

Initial EKF state:

- [x] `x = [ z, v, ba ]`

Initial transition:

- [x] `z = z + v*dt + 0.5*(a_world_z - ba)*dt^2`
- [x] `v = v + (a_world_z - ba)*dt`
- [x] `ba = ba`

Near-term improvement:

- [ ] Accumulate accelerometer delta-v between altitude task executions if Betaflight exposes the necessary data cleanly.
- [ ] Feed the estimator delta-v plus delta-time instead of one instantaneous acceleration sample.

## [ ] Phase 5: Barometer Measurement Plumbing

Baro must enter as timestamped height, not as "latest global altitude":

- [ ] Capture the actual pressure/altitude conversion timestamp from the barometer path.
- [x] Pass a baro timestamp into the estimator API. Current implementation uses altitude task time as the baro timestamp.
- [x] Define armed/home datum behavior.
- [x] Convert to positive-up altitude above datum inside the estimator datum/offset model.
- [x] Attach configured variance.
- [ ] Attach source quality beyond simple baro-present status.
- [x] Include measurement age in status/log outputs.

Rules:

- [x] Fuse baro position only.
- [x] Do not fuse baro derivative as velocity.
- [x] Keep baro datum/offset outside the EKF state.
- [x] Reset datum when disarmed or on first arm.
- [x] Do not reset datum in flight unless a logged source-reset/step event occurs.

## [x] Phase 6: Baro Fusion

Implement scalar baro height fusion:

- [x] Innovation: `innov = z_baro - z_pred`.
- [x] Innovation variance: `S = HPH' + R`.
- [x] Apply a floor to `S`.
- [x] Gate using `innov^2 / S`.
- [x] Update all three states through covariance cross-terms.
- [x] Update covariance with a numerically safe scalar form.
- [x] Constrain accelerometer bias after update.
- [x] Log accept/reject reason and effective `R`.

Delayed fusion:

- [x] Store predicted state/covariance/input history in a ring buffer.
- [x] Subtract configured baro delay from baro measurement timestamp when delayed fusion is enabled.
- [x] Find the closest delayed state.
- [x] Fuse there.
- [x] Replay buffered IMU increments to current time.

## [ ] Phase 7: Output Predictor And Position Rate

Expose two vertical-rate outputs:

- [x] `velocity`: EKF velocity state.
- [x] `positionRate`: position-consistent altitude derivative.

Implement the third-order baro-inertial complementary filter described in the proposal:

- [x] Input: published altitude and vertical acceleration.
- [x] Output: position-consistent altitude rate.
- [x] Default crossover: about `2 Hz`.

Use `positionRate` for:

- [ ] High-vibration fallback.
- [ ] Reset recovery.
- [ ] Later controller logic that needs continuity after estimator correction steps.

## [ ] Phase 8: Robustness And Reset Policy

Minimum required behavior:

- [x] Covariance floor behavior in estimator math.
- [x] Innovation variance floor to prevent gate lockout.
- [x] Persistent reject detection.
- [x] Recovery mode with temporary `R` inflation.
- [x] Explicit reset events.
- [x] Explicit baro step/offset events.
- [x] No silent altitude jumps from baro step handling; offset is logged.

Disturbance handling:

- [x] Inflate baro `R` for high tilt.
- [x] Inflate baro `R` during high vertical acceleration.
- [ ] Mark accelerometer clipping/high vibration.
- [ ] Add takeoff/landing ground-effect handling once controller/motor state is available.

Baro step handling:

- [x] Detect sustained large innovation with baro-rate evidence.
- [x] Adjust baro datum/offset, not EKF altitude directly.
- [x] Limit per-event offset change.
- [x] Log current offset and event reason/flag.

## [ ] Phase 9: CLI, MSP2, And Blackbox Contract

Implement observability before serious flight tuning.

CLI:

- [x] Add estimator parameters with `alt_est_*` names.
- [x] Use explicit scaled units in names where possible.
- [x] Add Blackbox parameter-name constants for all tuning parameters.

MSP:

- [x] Keep `MSP_ALTITUDE` as the simple altitude/vario output.
- [x] Define new MSP2 payload for estimator config.
- [x] Define new MSP2 payload for estimator config writes.
- [x] Define new MSP2 payload for estimator status.
- [x] Include payload version bytes.
- [x] Append-field compatible layout is established for v1 payloads.
- [x] Refresh runtime tunables after config writes.
- [ ] Validate MSP2 config/status handlers against a real FC.

Blackbox:

- [x] Add dedicated estimator fields, not only `debug[0..7]`.
- [x] Log estimated altitude.
- [x] Log velocity.
- [x] Log position-rate.
- [x] Log vertical acceleration.
- [x] Log accelerometer bias.
- [x] Log baro altitude.
- [x] Log innovation.
- [x] Log `S`.
- [x] Log gate.
- [x] Log effective `R`.
- [x] Log flags.
- [x] Log baro offset.
- [x] Log sample age.
- [x] Log configured delay through MSP status and parameter headers.
- [ ] Log tune profile ID if/when tune profiles exist.
- [x] Print estimator tuning params in Blackbox headers.
- [x] Add event visibility for reset, recovery, and step handling through status flags.
- [ ] Validate Blackbox output from real FC logs.

## [x] Phase 10: Pi Logger, Log Transfer, And Codex Analysis

Important workflow decision:

- [x] Do not build the tuning-analysis loop around an OpenAI API key inside `desktop-agent` or the Pi webapp.
- [x] Pi and desktop tooling collect, transfer, archive, and open logs.
- [x] Codex in this local repository analyzes downloaded logs and proposes tuning parameter changes.
- [x] Analysis scripts are local and have no OpenAI API use.
- [x] Any non-firmware implementation lives under root `althold/`.
- [x] External repos are references only.

Non-firmware workspace layout:

- [x] `althold/pi_logger/` contains the Pi Zero Flask/logger app and MSP2 recorder code.
- [x] `althold/desktop_tools/` contains local Windows helper code.
- [x] `althold/analysis/` contains Codex-run parser/metrics helpers.
- [x] `althold/logs/` exists and real logs are git-ignored by default.
- [x] Example configs live under the relevant `althold/` subdirectories.

Reuse/adapt previous collection pipeline:

- [x] Reviewed `desktop-agent` as reference material.
- [x] Copied/adapted the useful workflow shape into this repo instead of depending on `desktop-agent`.
- [x] Replaced old `Z_EKF` concepts with `ALTITUDE` / `alt_est` naming in the new local tooling.
- [x] Replaced old MSP v1 assumptions with MSP2 client calls.
- [x] Replaced `debug[0..7]` JSONL assumptions with dedicated estimator status fields.
- [x] Kept session metadata and download endpoints.
- [x] Kept log bundles self-describing with manifest/config snapshots.
- [x] Removed any hard dependency on `OPENAI_API_KEY`.
- [x] Avoided `CalibrationAnalysisWindow` as the normal analysis path.
- [x] Added local helper behavior for dry-run paths, command generation, archive creation, and latest-bundle selection.

Codex analysis loop:

- [x] Parser accepts the local Pi session bundle format.
- [x] Metrics include innovation stats, normalized innovation, reject/recovery/step counts, velocity plausibility, bias convergence, baro delay/correlation, and altitude drift.
- [x] Output includes `summary.json` and `summary.csv`.
- [x] Optional plots are supported when `matplotlib` is available.
- [ ] Analyze a real flight bundle.
- [ ] Use the analysis to recommend estimator parameter changes from real data.
- [ ] Apply candidate parameters through the Pi webapp/MSP2 path against a real FC.

## [ ] Phase 10b: Pi Field-Readiness Setup

Field network policy:

- [x] Preferred field mode: both the laptop and Pi connect to the smartphone hotspot.
- [x] In preferred field mode, the laptop stays online and reaches the Pi through Tailscale.
- [x] Pi boot/network fallback order is hotspot, home Wi-Fi, then Pi AP.
- [x] Tailscale is brought up automatically only when the Pi is connected to the smartphone hotspot.
- [x] Tailscale is brought down or left down on home Wi-Fi and AP fallback.
- [x] Pi logger/webapp works over phone hotspot + Tailscale.
- [x] Pi logger/webapp works over home Wi-Fi without automatic Tailscale.
- [ ] Pi logger/webapp works over Pi AP fallback without internet.

Tailscale setup:

- [x] Document official Raspberry Pi/Linux install docs:
  - [x] `https://tailscale.com/docs/install/linux`
  - [x] `https://tailscale.com/download/linux/rpi`
- [x] Install on Raspberry Pi OS with `curl -fsSL https://tailscale.com/install.sh | sh`.
- [x] Authenticate the Pi manually.
- [x] Verify assigned tailnet address and peer status.
- [x] Verify Windows laptop Tailscale connectivity to Pi.
- [x] Bring Tailscale down after manual home-Wi-Fi test.
- [ ] Consider disabling key expiry for the Pi in the Tailscale admin console.

When the user authorizes Pi access:

- [x] SSH into the Pi from the development machine.
- [x] Install or verify Python dependencies for `althold/pi_logger/`.
- [x] Install Tailscale on the Pi.
- [x] Manage runtime Tailscale state through the network-mode helper/service.
- [x] Deploy/copy the current Pi logger app from this repository to the Pi.
- [ ] Configure UART device, baud rate, user permissions, and serial service conflicts for real FC MSP2.
- [x] Configure Pi Wi-Fi behavior: smartphone hotspot first, home Wi-Fi second, Pi AP fallback last.
- [x] Configure a network-mode helper/service that detects the active network and runs `tailscale up` only on the smartphone hotspot.
- [x] Configure log directories and retention location.
- [x] Add a systemd service for the logger/webapp.
- [x] Add a systemd service for the field network helper.
- [x] Verify the webapp is reachable over Tailscale when laptop is on home Wi-Fi and Pi is on the smartphone hotspot.
- [ ] Verify the webapp is reachable over the Pi AP.
- [ ] Verify MSP/MSP2 communication with the FC.
- [x] Perform a fake-MSP dry-run recording session without flying.
- [ ] Pull a test log back to the local computer and run the Codex analysis parser on that downloaded bundle.
- [x] Document the Pi setup commands in `althold/pi_logger/README.md`.

Secret/config hygiene:

- [x] Add `althold/pi_logger/field_network.example.json` as a non-secret template.
- [x] Add local ignored `althold/pi_logger/field_network.json` for real SSIDs/passwords.
- [x] Add `.gitignore` rules for Wi-Fi/password credential JSON files.
- [x] Copy the real network config to `/etc/althold/field_network.json` on the Pi with `600` permissions.
- [x] Keep real Wi-Fi passwords out of git.

## [ ] Phase 11: Bench And Replay Validation

Before flight:

- [ ] Stationary bench log: velocity near zero, bias bounded, altitude does not walk rapidly.
- [ ] Hand lift/drop: signs of altitude and velocity correct.
- [ ] Pressure disturbance near baro: no permanent gate lockout.
- [ ] Synthetic baro step replay: offset handling does not create a velocity spike from recorded/replayed data.
- [x] Synthetic estimator unit tests cover baro fusion, delayed replay, gate/recovery, step offset, bias limit, and position-rate output.
- [ ] Timing test: baro sample age and configured delay are visible in real logs.
- [x] Parser test: analysis tool recovers field schema and parameters from generated logs.

Replay metrics:

- [x] Innovation mean/std/P95/P99 metric support.
- [x] Normalized innovation metric support.
- [x] Reject percentage metric support.
- [x] Longest reject segment metric support.
- [x] Recovery count metric support.
- [x] Reset/step event count metric support.
- [x] Velocity plausibility metric support.
- [x] Bias convergence and bounds metric support.
- [x] Correlation/delay metric support.
- [ ] Run these metrics on a real bench log.
- [ ] Run these metrics on a real hover/flight log.

## [ ] Phase 12: Estimator-Only Flight Validation

Fly with altitude-control mode disabled first:

- [ ] Hover.
- [ ] Slow climb/descent.
- [ ] Throttle punch.
- [ ] Forward flight with tilt.
- [ ] Takeoff/landing ground-effect checks.

Acceptance before controller work:

- [ ] Altitude sign and velocity sign are correct.
- [ ] Velocity magnitude is plausible.
- [ ] No realistic baro disturbance causes permanent gate lockout.
- [ ] No large altitude jump from ordinary propwash.
- [ ] Bias remains bounded.
- [ ] Baro delay is measurable in logs.
- [ ] Every reset/recovery/step event is visible in logs.
- [ ] Tuning recommendations can point to logged evidence.

## [ ] First Code Milestone: Estimator-Only Firmware

The first useful firmware milestone is estimator-only:

- [x] New `altitude_estimator` module exists.
- [x] New `altitude_estimator` module is included in firmware source lists.
- [x] Current wrappers return outputs from the new estimator.
- [x] Old althold controller path is not used for new behavior.
- [x] Baro + IMU estimator runs in code while altitude-control behavior is disabled.
- [x] Blackbox/MSP status exposes data for bench and hover analysis.
- [x] Pi/local tooling can record and parse estimator-session data in fake-MSP mode.
- [ ] Firmware is flashed to the FC.
- [ ] Pi UART is connected to the FC.
- [ ] Pi logger records real MSP2 estimator status from the FC.
- [ ] One estimator-only bench log is recorded.
- [ ] One estimator-only hover/flight log is recorded.
- [ ] The first real log is analyzed and used to propose tuning changes.

After this milestone:

- [ ] Start tuning estimator parameters from logs.
- [ ] Do not start cascaded altitude controllers until estimator-only logs are stable.
