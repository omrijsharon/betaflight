# Altitude Estimator Implementation Plan

This plan turns the altitude-estimator research notes into a staged Betaflight implementation path. Scope is altitude and vertical velocity estimation only. Altitude-control loops will be planned separately after the estimator/logging loop is working.

Source documents:

- `althold/ardupilot_ekf3_altitude_estimation.md`
- `althold/betaflight_altitude_estimator_proposal.md`

## Implementation Goal

Implement a robust vertical estimator for Betaflight using:

- barometer height as the long-term altitude reference.
- calculated attitude and accelerometer data for short-term vertical acceleration and velocity.
- no baro-derived velocity fusion.
- delayed baro fusion, or a staged path that records enough timing data to tune delay before enabling full rewind/replay.
- explicit estimator health, reset, gate, and source-status outputs.
- Blackbox/MSP/Pi logging designed for tuning and analysis.

The estimator must provide:

- altitude, positive up.
- vertical velocity, positive up.
- position-consistent altitude rate output.
- estimator status flags.
- reset/step/recovery event visibility.

## Decisions Already Made

- Do not port ArduPilot EKF3 as a full 24-state EKF.
- Use a dedicated 1D vertical estimator.
- Start from a 3-state EKF: `z`, `v`, and accelerometer bias `ba`.
- Keep barometer offset/datum handling outside the EKF state.
- Do not fuse differentiated baro as an independent velocity measurement.
- Use positive-up values in Betaflight-facing APIs.
- New altitude-control mode naming should be `ALTITUDE_MODE` / `BOXALTITUDE`; old `BARO_MODE` / `BOXBARO` can only be temporary aliases.
- Build logs and tune tooling before trying to tune estimator gates in flight.

## Open Decisions Before Coding

- Whether the first merge should include full delayed fusion, or a current-time EKF with timestamp logging and delayed-fusion scaffolding. Preferred answer: build the history/replay structure from the start if resource usage stays reasonable.
- Exact persistent parameter names and scaling. Preferred prefix: `alt_est_*`.
- Exact MSP2 command IDs and payload layouts.
- Exact Blackbox field names and whether to reuse the existing altitude field-select bit or add a new field-select bit.
- Whether to keep the existing local `ekf_*` CLI/MSP v1 parameters as temporary migration names.
- Where to place estimator PG config: temporary `positionConfig_t` fields or a new altitude estimator PG.

## Phase 1: Current-Code Reconciliation

Read the current local implementation and map all call sites before moving code:

- `src/main/flight/position.c`
- `src/main/flight/position.h`
- `src/main/fc/tasks.c`
- `src/main/sensors/barometer.c`
- `src/main/fc/core.c`
- `src/main/fc/runtime_config.h`
- `src/main/fc/rc_modes.h`
- `src/main/msp/msp.c`
- `src/main/msp/msp_protocol.h`
- `src/main/msp/msp_protocol_v2_betaflight.h`
- `src/main/blackbox/blackbox.c`
- `src/main/blackbox/blackbox_fielddefs.h`
- `src/main/cli/settings.c`
- `src/main/fc/parameter_names.h`

Expected output:

- list of estimator inputs currently available.
- list of altitude outputs currently consumed.
- list of old `USE_BARO_ALTHOLD` paths to remove or isolate from estimator work.
- exact build flags that gate baro, vario, altitude, and old althold behavior.

## Phase 2: Estimator Module Boundary

Create a dedicated estimator module:

- `src/main/flight/altitude_estimator.h`
- `src/main/flight/altitude_estimator.c`

Keep `position.c` as temporary glue only. It should call the new module and expose existing wrappers until callers are migrated:

- `getEstimatedAltitudeCm()`
- `getAltitude()`
- `getEstimatedVario()`

Core public types:

- config struct for persistent tunables.
- runtime state struct for `z`, `v`, `ba`, covariance, output predictor, datum, and flags.
- timestamped measurement struct with source, type, value, variance, quality, and timestamp.
- status/log struct that snapshots all values needed by MSP and Blackbox.

## Phase 3: Estimator Config

Add estimator tunables with explicit units and conservative defaults:

- accelerometer process noise.
- accelerometer bias process noise.
- accelerometer bias limit.
- baro height noise.
- baro delay.
- height gate sigma.
- minimum innovation variance.
- covariance floors.
- recovery timeout.
- recovery `R` inflation.
- baro step detector thresholds.
- baro offset step limits.
- height-rate complementary filter frequency.
- optional `R` inflation controls for tilt, high vertical acceleration, takeoff/landing, and vibration.

Use ArduPilot defaults as starting references:

- accelerometer process noise near `0.35 m/s^2`.
- accel bias process noise near `0.02 m/s^3`.
- baro noise initially `0.75 m` to `2.0 m` for small propwash-heavy quads.
- baro delay near `60 ms`.
- height gate initially `5 sigma` for early flight testing, then tighten.
- height-rate filter near `2 Hz`.

## Phase 4: IMU Prediction

Implement prediction from vertical acceleration:

- convert calibrated accelerometer data to m/s^2.
- rotate body acceleration into world frame using the current attitude matrix/quaternion.
- subtract gravity.
- subtract estimated vertical accelerometer bias.
- integrate altitude and velocity with real `dt`.
- clamp `dt` to survive scheduler jitter.
- detect accelerometer clipping or invalid attitude and set status flags.

Initial EKF state:

```text
x = [ z, v, ba ]
```

Initial transition:

```text
z  = z + v*dt + 0.5*(a_world_z - ba)*dt^2
v  = v + (a_world_z - ba)*dt
ba = ba
```

Near-term improvement:

- accumulate accelerometer delta-v between altitude task executions if Betaflight exposes the necessary data cleanly.
- feed the estimator delta-v plus delta-time instead of one instantaneous acceleration sample.

## Phase 5: Barometer Measurement Plumbing

Baro must enter as timestamped height, not as "latest global altitude":

- capture pressure/altitude conversion timestamp.
- define armed/home datum.
- convert to positive-up altitude above datum.
- attach variance and quality.
- include measurement age in status/log outputs.

Rules:

- fuse baro position only.
- do not fuse baro derivative as velocity.
- keep baro datum/offset outside the EKF state.
- reset datum when disarmed or on first arm.
- do not reset datum in flight unless a logged source-reset event occurs.

## Phase 6: Baro Fusion

Implement scalar baro height fusion:

- innovation: `innov = z_baro - z_pred`.
- innovation variance: `S = HPH' + R`.
- apply a floor to `S`.
- gate using `innov^2 / S`.
- update all three states through covariance cross-terms.
- update covariance with a numerically safe scalar form.
- constrain accelerometer bias after update.
- log accept/reject reason and effective `R`.

If delayed fusion is implemented immediately:

- store predicted state/covariance/input history in a ring buffer.
- subtract configured baro delay from baro measurement timestamp.
- find the closest delayed state.
- fuse there.
- replay buffered IMU increments to current time.

If delayed fusion is staged:

- keep a clear interface for delayed fusion.
- log sample timestamp, estimator timestamp, configured delay, measured age, and current-time innovation.
- do not pretend the current-time estimator is final.

## Phase 7: Output Predictor And Position Rate

Expose two vertical-rate outputs:

- `velocity`: EKF velocity state.
- `positionRate`: position-consistent altitude derivative.

Implement the third-order baro-inertial complementary filter described in the proposal:

- input: published altitude and vertical acceleration.
- output: position-consistent altitude rate.
- default crossover: about `2 Hz`.

Use `positionRate` for:

- high-vibration fallback.
- reset recovery.
- later controller logic that needs continuity after estimator correction steps.

## Phase 8: Robustness And Reset Policy

Minimum required behavior:

- covariance floors to prevent overconfidence.
- innovation variance floor to prevent gate lockout.
- persistent reject detection.
- recovery mode with temporary `R` inflation.
- explicit reset events.
- explicit baro step/offset events.
- no silent altitude jumps.

Disturbance handling:

- inflate baro `R` for high tilt.
- inflate baro `R` during high vertical acceleration.
- mark accelerometer clipping/high vibration.
- add takeoff/landing ground-effect handling once controller/motor state is available.

Baro step handling:

- detect sustained large innovation with baro rate evidence.
- adjust baro datum/offset, not EKF altitude directly.
- limit per-event offset change.
- log current offset and event reason.

## Phase 9: CLI, MSP2, And Blackbox Contract

Implement observability before serious flight tuning.

CLI:

- add estimator parameters with `alt_est_*` names where possible.
- use explicit scaled units in names.
- add Blackbox parameter-name constants for all tuning parameters.

MSP:

- keep `MSP_ALTITUDE` as the simple altitude/vario output.
- define new MSP2 payloads for estimator config and status.
- include payload version bytes.
- append fields only at the end.
- refresh runtime tunables after config writes.

Blackbox:

- add dedicated estimator fields, not only `debug[0..7]`.
- log estimated altitude, velocity, position-rate, vertical acceleration, accelerometer bias, baro altitude, innovation, `S`, gate, effective `R`, flags, baro offset, sample age, delay, and tune profile ID.
- print all estimator tuning params in headers.
- add event visibility for reset, recovery, and step handling.

## Phase 10: Pi Logger, Log Transfer, And Codex Analysis

Important workflow decision:

- Do not build the tuning-analysis loop around an OpenAI API key inside `desktop-agent` or the Pi webapp.
- The Pi and desktop tooling are responsible for collecting, transferring, archiving, and opening logs.
- Codex in this local repository will analyze downloaded logs and propose tuning parameter changes.
- Analysis scripts can be generated and run locally by Codex against the downloaded files.
- Any work that is not Betaflight firmware implementation must live under the root `althold/` directory in this repository.
- External repos are references only. Copy/adapt useful pieces into `althold/` instead of making the workflow depend on editing `desktop-agent` or `ArUco_Chaser`.

Suggested non-firmware workspace layout:

- `althold/pi_logger/` for the Pi Zero Flask/logger app and MSP2 recorder code.
- `althold/desktop_tools/` for the local Windows helper that connects to the Pi AP, starts/stops the logger, pulls logs, archives runs, and shows the latest bundle path.
- `althold/analysis/` for Codex-run parsers, plotting scripts, metrics, and reusable tuning-analysis helpers.
- `althold/logs/` for downloaded or sample log bundles if we decide to keep short test fixtures in-repo. Real flight logs may be large and should be git-ignored if stored here.
- `althold/config_examples/` for example Pi/logger/desktop config files without secrets.

Reuse the previous collection/transfer pipeline where possible:

- desktop orchestration from `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_ui.py`.
- Wi-Fi/SSH/SCP helpers from `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_ops.py`.
- config structure from `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_config.py`, but remove or bypass API-key/model settings for this workflow.
- Pi Flask recorder pattern from `C:/Users/tamipinhasi/Documents/repos/ArUco_Chaser/tests/z_ekf_calibration_webapp/app.py`.

Required adaptations:

- rename old `Z_EKF` concepts to `ALTITUDE` / `alt_est`.
- replace old MSP v1 config calls with new MSP2 calls.
- replace `debug[0..7]` JSONL assumptions with dedicated estimator status fields.
- keep session metadata and download endpoints.
- keep log bundles self-describing with firmware version, tune profile, field schema, and parameter snapshot.
- remove any hard dependency on `OPENAI_API_KEY` for the calibration workflow.
- do not launch `CalibrationAnalysisWindow` as the normal analysis path if it requires API credentials.
- add a simple "open logs folder / copy path for Codex" workflow, or write a run manifest that points Codex at the latest downloaded bundle.
- place the adapted Pi app, desktop helper, analysis scripts, schemas, and example configs under `althold/`.

Codex analysis loop:

1. User downloads/pulls the flight bundle from the Pi to the local computer.
2. User asks Codex to analyze the latest run or a specific log directory.
3. Codex reads the local log bundle, writes/updates local parsing scripts if needed, runs the analysis, and summarizes metrics.
4. Codex proposes new estimator parameter values with rationale and expected effect.
5. User applies candidate parameters through the Pi webapp/MSP2 path.
6. Repeat with the next flight.

## Phase 10b: Pi Field-Readiness Setup

After firmware, MSP2, logging, and local tooling are ready, the Pi Zero 2 W should be prepared over SSH for field experiments.

Field network policy:

- Preferred field mode: both the laptop and Pi connect to the smartphone hotspot.
- In preferred field mode, the laptop stays online and reaches the Pi through Tailscale.
- Pi boot/network fallback order:
  1. Try to connect to the smartphone hotspot.
  2. If the hotspot is unavailable, try to connect to home Wi-Fi.
  3. If home Wi-Fi is also unavailable, start the Pi's own access point for offline local access.
- Tailscale should be brought up only when the Pi is connected to the smartphone hotspot.
- If the Pi is on home Wi-Fi or its own AP fallback, the connection manager should bring Tailscale down or leave it down.
- The Pi logger/webapp must still work in all three modes:
  - phone hotspot + Tailscale.
  - home Wi-Fi without automatic Tailscale.
  - Pi AP fallback without internet.

Tailscale install and verification reference:

- Official Raspberry Pi/Linux install docs:
  - `https://tailscale.com/docs/install/linux`
  - `https://tailscale.com/download/linux/rpi`
- Install on Raspberry Pi OS:

```sh
curl -fsSL https://tailscale.com/install.sh | sh
```

- First authentication, performed manually during setup:

```sh
sudo tailscale up
```

- Verify assigned tailnet address and peer status:

```sh
tailscale ip
tailscale status
```

- After first authentication, the field network-mode service should control runtime state:

```sh
sudo tailscale up     # only when connected to the smartphone hotspot
sudo tailscale down   # on home Wi-Fi or Pi AP fallback
```

- Consider disabling key expiry for the Pi in the Tailscale admin console after setup, because the Pi is a field device and re-authentication at the field would be disruptive.

When the user authorizes Pi access:

- SSH into the Pi from the development machine.
- install or verify Python dependencies for the `althold/pi_logger/` app.
- install Tailscale on the Pi, but manage when it is brought up through our network-mode script.
- deploy/copy the current Pi logger app from this repository to the Pi.
- configure UART device, baud rate, user permissions, and serial service conflicts.
- configure the Pi Wi-Fi behavior needed at the field: smartphone hotspot first, home Wi-Fi second, Pi AP fallback last.
- configure a small network-mode script/service that detects the active SSID/network and runs `tailscale up` only on the smartphone hotspot, otherwise `tailscale down`.
- configure log directories and retention policy.
- add a systemd service or explicit launch script for the logger/webapp.
- verify the webapp is reachable over the Pi AP.
- verify the webapp is reachable over Tailscale when both laptop and Pi are on the smartphone hotspot.
- verify MSP/MSP2 communication with the FC.
- perform a dry-run recording session without flying.
- pull the test log back to the local computer and run the Codex analysis parser.
- document the exact Pi setup commands in `althold/pi_logger/` so the setup is reproducible.

## Phase 11: Bench And Replay Validation

Before flight:

- stationary bench log: velocity near zero, bias bounded, altitude does not walk rapidly.
- hand lift/drop: signs of altitude and velocity correct.
- pressure disturbance near baro: no permanent gate lockout.
- synthetic baro step replay: offset handling does not create a velocity spike.
- timing test: baro sample age and configured delay are visible.
- parser test: analysis tool recovers field schema and parameters from logs.

Replay metrics:

- innovation mean/std/P95/P99.
- normalized innovation squared.
- reject percentage.
- longest reject segment.
- recovery count.
- reset/step event count.
- velocity plausibility against low-pass differentiated altitude.
- bias convergence and bounds.
- correlation/delay between baro motion and estimator output.

## Phase 12: Estimator-Only Flight Validation

Fly with altitude-control mode disabled first:

- hover.
- slow climb/descent.
- throttle punch.
- forward flight with tilt.
- takeoff/landing ground-effect checks.

Acceptance before controller work:

- altitude sign and velocity sign are correct.
- velocity magnitude is plausible.
- no realistic baro disturbance causes permanent gate lockout.
- no large altitude jump from ordinary propwash.
- bias remains bounded.
- baro delay is measurable in logs.
- every reset/recovery/step event is visible in logs.
- tuning recommendations can point to logged evidence.

## First Code Milestone

The first useful firmware milestone should be estimator-only:

- new `altitude_estimator` module compiles.
- current wrappers return outputs from the new estimator.
- old althold controller path is not used for new behavior.
- baro + IMU estimator runs while altitude-control mode is disabled.
- Blackbox/MSP status exposes enough data for bench and hover analysis.
- Pi/local tooling can record and parse one estimator-only flight.

After this milestone, start tuning estimator parameters from logs. Do not start cascaded altitude controllers until this milestone is stable.
