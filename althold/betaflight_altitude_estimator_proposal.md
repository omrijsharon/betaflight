# Betaflight Baro + IMU Altitude Estimator Proposal

This document proposes how to implement robust altitude position and velocity estimation in Betaflight using only the barometer, calculated attitude, and accelerometer. The design keeps future sensor fusion in mind, but controllers are intentionally out of scope here.

## Current Betaflight Context

This checkout contains your experimental vertical estimator/control implementation. Treat it as a useful map of existing hooks and lessons learned, not as the production baseline. The gate/recovery tuning problems are a signal that the next design needs cleaner estimator boundaries, stronger observability, and logging that is built for analysis from the start.

- `src/main/flight/position.h`
  - defines `positionAltEKF_t`.
  - current state is `[z, v, ba, bb]`.
  - `z` is altitude in meters, positive up.
  - `v` is vertical velocity in m/s, positive up.
  - `ba` is vertical accelerometer bias.
  - `bb` is barometer altitude bias/offset.
- `src/main/flight/position.c`
  - predicts with world-Z acceleration from `acc.accADC[]` and `rMat`.
  - fuses baro height with an innovation gate.
  - has persistent-reject recovery and baro step handling.
  - exports altitude and vario.
  - contains `USE_BARO_ALTHOLD` hooks; these are local context only and should be replaced, not kept as a parallel legacy control path.
- `src/main/sensors/barometer.c`
  - converts pressure to altitude using `pressureToAltitude()`.
  - calibrates ground level.
- `src/main/fc/tasks.c`
  - runs `TASK_ALTITUDE` at `TASK_ALTITUDE_RATE_HZ`, currently 100 Hz.

Existing integration points that the replacement must account for:

- `src/main/fc/runtime_config.h`
  - `BARO_MODE` is the old flight mode bit.
  - `BOXBARO` maps the old user-facing mode box to `BARO_MODE`.
- `src/main/fc/core.c`
  - enables/disables `BARO_MODE` from the BARO mode switch today. The replacement should introduce the new altitude-control mode and route any temporary BARO alias into it, not into the old althold behavior.
- `src/main/msp/msp_protocol.h`
  - `MSP_ALTITUDE` is the legacy altitude/vario output.
  - this checkout already has local custom config commands `MSP_Z_EKF_CONFIG`, `MSP_SET_Z_EKF_CONFIG`, `MSP_BARO_ALTHOLD_CONFIG`, and `MSP_SET_BARO_ALTHOLD_CONFIG`.
- `src/main/msp/msp.c`
  - implements those MSP read/write handlers and uses `sbufBytesRemaining()` for appended fields.
- `src/main/cli/settings.c`
  - exposes estimator settings under `ekf_*` and controller settings under `alt_hold_*`.
  - exposes RC behavior through `alt_hold_deadband` and `alt_hold_fast_change`.
- `src/main/blackbox/blackbox.c`
  - already logs `baroAlt`, `debug[0..7]`, and several position/althold parameter headers.
- `src/main/fc/parameter_names.h`
  - provides the stable parameter-name strings used by Blackbox headers.

## Recommended Architecture

Do not port ArduPilot EKF3 as a full 24-state EKF. Betaflight already has an attitude solution and does not need EKF3's horizontal position, magnetic field, wind, GPS, beacon, and lane-switching machinery for altitude hold.

Instead, implement a dedicated vertical estimator:

- prediction from IMU vertical acceleration.
- correction from barometer altitude.
- delayed measurement handling.
- clean future sensor interface for rangefinder, GPS altitude, GPS vertical velocity, optical-flow height aid, or external navigation.
- public outputs for altitude, vertical velocity, estimator health, and reset events.

Suggested module boundary:

- `src/main/flight/altitude_estimator.h`
- `src/main/flight/altitude_estimator.c`
- keep `position.c` only as temporary migration glue until callers are moved to the new estimator/controller APIs.

## Replacement And Naming Policy

The old althold feature should be completely replaced. Do not maintain a second legacy althold controller behind the same mode switch, and do not keep old behavior as a fallback unless a specific flight-test safety switch is intentionally added for development.

Historical names can still be reused when they are useful integration points:

- New flight mode name should be `ALTITUDE_MODE`.
- New mode box name should be `BOXALTITUDE`.
- User-facing mode name should be `ALTITUDE` or `ALTITUDE CONTROL`.
- Old `BARO_MODE` and `BOXBARO` may exist only as temporary aliases or migration mappings, and must route to the new altitude-control implementation.
- `MSP_ALTITUDE` can remain the simple altitude/vario output.
- `baroAlt` can remain the raw/datum-relative barometer Blackbox field.
- CLI/MSP parameter names may be kept only when the meaning is still the same. If the new controller changes the meaning, use a new name and retire the old one.

Use clearer internal names for the new implementation:

- `althold` is the feature family and documentation directory.
- `altitude_estimator` is the estimator module.
- future controller modules should use names such as `altitude_controller`.

For new protocol work, prefer versioned MSP2 commands instead of consuming more legacy MSP v1 IDs. The Pi logger/webapp should target the new MSP2 payloads. Old local v1 althold-config commands should either be removed or treated as temporary migration commands; they should not define the new architecture.

## Coordinate Convention

Use Betaflight-facing positive-up altitude and velocity because existing APIs and OSD behavior are already positive-up:

- `z`: altitude above the armed/home datum, meters, positive up.
- `v`: vertical velocity, m/s, positive up.
- `a`: vertical acceleration, m/s^2, positive up.

Isolate any sign conversions at sensor adapters. This avoids NED sign errors leaking into controllers and UI.

## Core State

For the first robust baro + IMU estimator, prefer a 3-state baseline:

```text
x = [ z, v, ba ]

z  = altitude, m, positive up
v  = vertical velocity, m/s, positive up
ba = vertical accelerometer bias, m/s^2
```

Prediction:

```text
a_world = R_body_to_world * acc_body - gravity
a = a_world_z - ba

z = z + v*dt + 0.5*a*dt^2
v = v + a*dt
ba = ba
```

Covariance transition:

```text
F = [ 1, dt, -0.5*dt^2
      0,  1, -dt
      0,  0,  1 ]
```

Measurement:

```text
z_baro = z + noise
H = [1, 0, 0]
```

Reason for not making baro bias a normal state at first: with only one absolute height source, true altitude and barometer bias are not independently observable over long periods. A freely learned `bb` state can steal real altitude changes or hide accelerometer bias. ArduPilot also does not keep a baro-bias state in its EKF3 state vector.

Keep baro offset handling outside the EKF:

- armed datum offset.
- baro step offset for sudden pressure jumps.
- optional slow ground/disarmed calibration offset.

If we keep the current 4-state `[z, v, ba, bb]` for iteration speed, constrain `bb` heavily:

- very small random walk during normal flight.
- bounded magnitude and bounded per-event step adjustment.
- explicit debug/log fields showing how much altitude is coming from `z` vs `bb`.
- disable or slow `bb` learning during sustained climb/descent unless a step detector has fired.

## IMU Input

Use calibrated acceleration in m/s^2 and the current attitude matrix/quaternion:

```text
a_world_z = dot(world_z_row, acc_body_mss) - 9.80665
```

Requirements:

- use accurate `dt`, not a fixed task period.
- clamp `dt` to a sane range to survive scheduler jitter.
- reject or down-weight prediction during accelerometer clipping.
- track vibration metrics and high vertical acceleration periods.
- constrain `ba` to a reasonable bound, for example +/- 1.5 to 2.5 m/s^2 initially.

Best implementation detail from ArduPilot: integrate delta velocity, not just instantaneous acceleration samples. If Betaflight can accumulate accelerometer data between altitude-task executions, feed the estimator an averaged delta velocity and delta time. This reduces aliasing and is closer to ArduPilot's `readIMUData()` and `UpdateStrapdownEquationsNED()` model.

## Barometer Input

Baro should enter the estimator as a timestamped height measurement, not as "latest global altitude".

Recommended measurement struct:

```c
typedef enum {
    ALT_MEAS_POS,
    ALT_MEAS_VEL
} altitudeMeasurementType_e;

typedef enum {
    ALT_SRC_BARO,
    ALT_SRC_RANGEFINDER,
    ALT_SRC_GPS,
    ALT_SRC_EXTERNAL_NAV
} altitudeSource_e;

typedef struct altitudeMeasurement_s {
    altitudeMeasurementType_e type;
    altitudeSource_e source;
    timeUs_t timeUs;
    float value;
    float variance;
    uint8_t quality;
} altitudeMeasurement_t;
```

For baro:

- `type = ALT_MEAS_POS`.
- `value = altitude above armed datum`, meters, positive up.
- `variance = R_baro`, adaptively inflated as needed.
- no vertical velocity measurement from differentiated baro.

Do not fuse barometer derivative as independent vertical velocity. It double-counts the same barometer data and makes the filter overconfident. Baro-derived vario can still be displayed or logged as a diagnostic, but estimator velocity should come from IMU prediction corrected by height innovations.

## Delayed Fusion

This is the biggest missing piece to copy from ArduPilot conceptually.

Barometer conversion and filtering add delay. ArduPilot timestamps baro as "last baro update minus configured height delay minus half the EKF update period", stores it in a buffer, and fuses it against the delayed EKF state.

For Betaflight:

1. Store estimator states in a ring buffer:
   - timestamp.
   - state vector.
   - covariance.
   - accumulated IMU delta velocity since previous state.
2. Timestamp each baro altitude sample at the actual pressure conversion completion time.
3. Subtract a configurable baro delay, initially 40 to 80 ms.
4. Fuse the baro measurement against the buffered state closest to that timestamp.
5. Propagate the corrected delayed state forward to now using buffered IMU increments.

Simpler first step if full rewind/replay is too much:

- keep the current-time EKF, but add an output observer/complementary predictor and tune larger baro `R` to tolerate delay.
- still record measurement timestamps and delay estimates so logs can validate the real delay.

Production goal should be delayed fusion or rewind/replay.

## Output Predictor And Height-Rate Output

Expose two vertical rates, matching ArduPilot's distinction:

- `velocity`: EKF velocity state, best estimate of actual vertical speed.
- `positionRate`: derivative of the published altitude estimate, kinematically consistent with `z`.

Use a third-order baro-inertial complementary filter for `positionRate`, based on ArduPilot's `vertCompFiltState`:

```text
omega = 2*pi*height_rate_filter_hz
pos_err = z_output - comp.pos

comp.acc += pos_err * omega^3 * dt
comp.vel += a_world_z * dt + (comp.acc + 3*omega^2*pos_err) * dt
comp.pos += (comp.vel + 3*omega*pos_err) * dt
```

Initial frequency: 1.5 to 3.0 Hz. ArduPilot defaults to 2.0 Hz.

Use `positionRate` for:

- high vibration fallback.
- immediately after position resets.
- controller modes that must avoid a velocity/position discontinuity.

## Robustness Rules

Minimum robust behavior:

- gate baro innovations using `innov^2 / S`.
- enforce a floor on innovation variance `S` to prevent permanent gate lockout.
- enforce covariance floors to avoid overconfidence.
- if baro innovations reject for too long, enter recovery with inflated `R`, or reset height if the vehicle is disarmed/on ground.
- identify pressure steps separately from ordinary outliers.
- never allow recovery logic to silently create a large altitude jump without logging a reset/offset event.

Recommended baro disturbance handling:

- inflate baro `R` when tilt is high.
- inflate baro `R` during strong vertical acceleration.
- inflate or dead-zone baro during takeoff/landing ground effect.
- add a motor-spool/propwash flag later when throttle and motor RPM data are available.

Recommended step handling:

- detect a pressure step when baro innovation is large, baro height changes rapidly, and innovation does not shrink for several samples.
- apply the step to a baro datum/offset, not directly to altitude state.
- limit per-event offset adjustment.
- log the event and current offset.

## Future Sensor Fusion

The estimator should allow additional measurements without restructuring:

| Future source | Measurement type | Notes |
| --- | --- | --- |
| Rangefinder | position | Height above ground, only valid in range and at low tilt. Needs terrain/datum handling. |
| GPS altitude | position | Noisy and delayed. Useful as a long-term baro drift reference, not primary for small quads. |
| GPS vertical velocity | velocity | If valid, can directly correct `v`. Keep source-frame and delay checks. |
| External navigation | position and velocity | Needs explicit frame, origin, timestamp, and reset handling. |

Each source needs:

- source enum.
- variance.
- delay.
- innovation gate.
- health timeout.
- reset policy.

## Initial Tuning Targets

Use ArduPilot defaults as a starting reference, then tune with Betaflight logs:

- accelerometer process noise: start around `0.35 m/s^2` RMS.
- accelerometer bias process noise: start around `0.02 m/s^3`.
- baro height noise: do not start as low as the sensor datasheet. For propwash-heavy small quads, start around `0.75 m` to `2.0 m` RMS and rely on adaptive `R`.
- height gate: 3 sigma for tight testing, 5 sigma for early flight testing.
- baro delay: start at 60 ms.
- height-rate complementary filter: 2 Hz.

The current local defaults in `position.c` use `0.25 m` baro RMS, which may be too optimistic in real propwash. It can work on a bench and still be fragile in flight.

## Observability And Reset Policy

Only baro + IMU means:

- short-term velocity comes from accelerometer integration.
- long-term altitude comes from baro.
- accelerometer bias is observable through the pattern of baro height innovations over time.
- baro drift is not distinguishable from real altitude drift without another reference.

Therefore:

- estimate accelerometer bias carefully.
- treat baro datum as the altitude reference.
- handle baro drift/steps as source offset management, not as proof the vehicle moved.
- reset altitude datum when disarmed or on first arm, not in midair unless a clear source reset event occurs.

## MSP, CLI, And Blackbox Integration

Estimator tuning will not work if the only available output is "altitude looks bad". The first implementation should expose enough configuration and logs to explain why the estimate is bad.

### CLI

How CLI settings are added in this checkout:

- Add persistent fields to `positionConfig_t` in `src/main/flight/position.h`, or move them into a new altitude estimator PG when the estimator is split out.
- Add/reset defaults in the PG reset template, currently near the existing `PG_RESET_TEMPLATE(positionConfig_t, positionConfig, ...)` block in `src/main/flight/position.c`.
- Add CLI entries in `src/main/cli/settings.c` with explicit bounds and scaling.
- Add matching constants in `src/main/fc/parameter_names.h` for every parameter that must appear in Blackbox headers.
- For throttle-stick behavior, either reuse `rcControlsConfig_t` fields only if their meaning remains identical, or add new fields for the new altitude-control semantics. `alt_hold_deadband` and `alt_hold_fast_change` show where this kind of RC-mode behavior currently lives, but they should not force the new behavior to match the old feature.

CLI naming rules for this feature:

- Reuse `alt_hold_*` only for parameters whose semantics are still accurate in the replacement feature.
- Prefer clearer new names for new estimator and controller parameters, for example `alt_est_*` for estimator settings and `alt_ctl_*` or `althold_*` for controller settings.
- Keep existing `ekf_*` names only as temporary migration names while the local experimental code depends on them.
- Keep units in names where possible: `_cm`, `_cms`, `_ms`, `_m2_x1000`, `_x100`, `_x1000`.
- Separate estimator parameters from controller parameters. Estimator tuning should not require changing throttle/controller gains.

### MSP

Existing MSP altitude hooks:

- `MSP_ALTITUDE` in `src/main/msp/msp_protocol.h` returns `getEstimatedAltitudeCm()` and `getEstimatedVario()` from `src/main/msp/msp.c`.
- `MSP_Z_EKF_CONFIG` and `MSP_SET_Z_EKF_CONFIG` already read/write local estimator parameters.
- `MSP_BARO_ALTHOLD_CONFIG` and `MSP_SET_BARO_ALTHOLD_CONFIG` already read/write local althold/controller parameters.
- The current SET handlers append optional fields and decode them with `sbufBytesRemaining()`, which is a useful migration/extension pattern.

Recommended MSP plan:

- Keep `MSP_ALTITUDE` stable as a simple altitude/vario output.
- Do not use the current local v1 althold config commands as the design contract for the replacement. They can be deleted, renamed, or left as temporary migration commands while the Pi/webapp moves to MSP2.
- Add new MSP2 commands in `src/main/msp/msp_protocol_v2_betaflight.h` for the Pi logger/webapp and future configurator work.
- Implement read commands in the output command switch in `src/main/msp/msp.c`.
- Implement write commands in the input command switch in `src/main/msp/msp.c`.
- Put a payload version byte at the start of every new MSP2 payload.
- Append fields only at the end, and make the receiver tolerate shorter payloads.
- After a config write, call the estimator/controller update function so runtime tunables are refreshed immediately.

MSP2 command groups that will be useful:

- estimator config get/set.
- controller config get/set, later when controllers are designed.
- estimator status snapshot: estimated altitude, velocity, position-rate output, accelerometer bias, baro altitude, innovation, innovation variance, effective `R`, gate limit, baro offset, sample age, delay, and flags.
- live logging control: start, stop, set logging profile/rate, and insert event marks.
- tune profile control: read active profile, write candidate profile, apply candidate, and save.

The Pi should not need to parse CLI text in normal operation. CLI remains useful for manual debugging, but the Pi logger/webapp should use MSP/MSP2 for structured reads and writes.

### Blackbox

Existing Blackbox hooks:

- `src/main/blackbox/blackbox.c` defines and writes `baroAlt` when `USE_BARO` is enabled.
- `debug[0..7]` is available for quick development logging.
- `src/main/blackbox/blackbox_fielddefs.h` defines field conditions and field-select bits; the field-select enum is limited to 32 bits.
- `src/main/cli/settings.c` exposes `blackbox_disable_alt`, `blackbox_disable_debug`, and other field-disable CLI settings.
- `src/main/blackbox/blackbox.c` prints position/althold parameter headers using names from `src/main/fc/parameter_names.h`.

Recommended Blackbox plan:

- Use `debug[0..7]` only for early bring-up and short experiments. It is too small and mode-dependent to be the main estimator tuning interface.
- Add dedicated estimator fields for production tuning logs.
- Reuse the existing altitude field-select bit if the fields are clearly altitude-related; add a new field-select bit only if the extra fields need an independent enable/disable switch. If a new bit is added, update `blackbox_fielddefs.h`, `blackbox_disable_*` CLI settings, and stay within the 32-bit mask limit.
- Add fields in `blackboxMainFields`.
- Add storage in `blackboxMainState_t`.
- Populate the state where `blackboxCurrent` is filled.
- Encode the fields in I-frame and P-frame writing paths.
- Print all estimator/controller tuning parameters in the log header so analysis tools know exactly which tune produced the flight.

Dedicated estimator fields to log:

- raw or datum-relative baro altitude.
- estimated altitude.
- estimated vertical velocity.
- position-consistent height-rate output.
- world-Z acceleration or vertical delta-v used by the estimator.
- accelerometer bias estimate.
- baro datum/offset.
- baro innovation.
- innovation variance `S`.
- gate threshold.
- effective baro measurement variance `R`.
- estimator flags: baro accepted, gated, forced recovery, step detected, reset, stale sample, accel clipping, high vibration.
- baro sample age and configured/effective delay.
- active tune profile or tuning iteration ID.

Controller work later should also log throttle stick, desired vertical velocity, desired altitude, acceleration setpoint, throttle correction, integrator state, hover throttle estimate, saturation flags, and mode transitions. Some of this already exists in RC/setpoint/motor fields, but controller-specific internal state will need new fields.

When the new controller is implemented, `ALTITUDE_MODE` should route only to the new cascaded altitude-control path. Any temporary `BARO_MODE` alias should also route there. The old `USE_BARO_ALTHOLD` control code should be removed or compiled out during that migration, not left active as another behavior sharing the same mode.

## Pi Logger And Tuning Workflow

The Pi Zero 2 W should be treated as the flight-side data and workflow bridge, not as part of the estimator loop. The flight controller must keep estimating and controlling altitude by itself.

Pi responsibilities:

- talk to the FC over UART using MSP/MSP2.
- start/stop a logging session from a local webapp.
- record a structured MSP telemetry stream, a Blackbox serial stream if enabled, or both.
- capture the full tune/config snapshot before each flight.
- tag each run with profile ID, date/time, battery, prop setup, airframe notes, and free-form pilot notes.
- expose the log bundle over Wi-Fi/AP immediately after landing.
- apply candidate tuning parameters back to the FC through MSP/MSP2.
- keep old runs and candidate profiles so changes can be compared.

Existing code to reuse or copy from:

- `C:/Users/tamipinhasi/Documents/repos/desktop-agent/ui/automated_calibration/README.md`
  - documents an existing Pi AP -> webapp -> logs -> internet -> LLM/Python analysis workflow.
  - for this project, reuse the Pi AP/webapp/log-transfer pieces, but do not use the API-key-based LLM analysis path.
  - entry point: `C:/Users/tamipinhasi/Documents/repos/desktop-agent/scripts/automated_calibration_ui.py`.
- `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_ui.py`
  - PySide6 desktop runner.
  - connects the Windows laptop to the Pi AP.
  - starts a remote webapp on the Pi over SSH.
  - pulls logs with SCP after flight.
  - archives previous local logs.
  - deletes remote logs after a successful pull.
  - switches the laptop back to an internet Wi-Fi/hotspot.
  - currently opens an API-key-based analysis window; for this project, replace that with "open logs folder / show latest bundle path for Codex".
- `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_ops.py`
  - reusable operations: `connect_wifi_windows()`, `list_wifi_ssids_windows()`, `ssh_run()`, `ssh_popen()`, `scp_pull_dir()`, `remote_dir_listing()`, `archive_dir()`, `ensure_empty_dir()`, `wait_for_tcp()`, and `has_internet()`.
- `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_config.py`
  - reusable config model for `pi_ap`, `hotspot`, `pi_ssh`, `remote`, and `local`.
  - the existing `analysis` model/prompt builder is reference only; do not require an OpenAI API key for this workflow.
- `C:/Users/tamipinhasi/Documents/repos/desktop-agent/src/desktop_agent/automated_calibration_analysis_ui.py`
  - existing API-key-based analysis chat window.
  - loads selected log files or the configured logs directory.
  - uses `python_sandbox` and saved analysis tools.
  - has a config dialog for prompt, debug mapping, Betaflight snippets, and control params.
  - reference only unless it is rewritten to hand off local log paths to Codex without requiring API credentials.
- `C:/Users/tamipinhasi/Documents/repos/desktop-agent/ui/automated_calibration/analysis_tools/analyze_z_ekf_jsonl/20260213T101935Z/analyze_z_ekf_jsonl.py`
  - existing analysis tool for Betaflight `DEBUG_Z_EKF` JSONL logs.
  - computes innovation statistics, rejection percentage, reject segments, innovation histograms, and innovation-vs-tilt bins.
  - useful as a starting point, but it is tied to the old `debug[0..7]` mapping and should be rewritten around dedicated altitude-estimator log fields.

The Pi-side Flask logger is referenced by the `desktop-agent` config rather than living inside `desktop-agent`:

- `C:/Users/tamipinhasi/Documents/repos/ArUco_Chaser/tests/z_ekf_calibration_webapp/app.py`
  - starts a Flask webapp on `0.0.0.0:5000`.
  - uses `utils.yamspy.MSPy` over `FC_PORT`/`FC_BAUD` (`/dev/ttyS0`, `500000` by default).
  - starts a background `ZEkfRecorder`.
  - polls armed state.
  - opens a new JSONL session on arm and closes it on disarm.
  - records `fast_read_debug()` samples as `{"t_ns": ..., "dbg": [...]}`.
  - writes per-session JSON metadata.
  - exposes `/api/z_ekf/status`, `/api/z_ekf/config` GET/POST, `/api/z_ekf/sessions`, `/api/z_ekf/sessions/<id>`, `/api/z_ekf/sessions/<id>/download`, and `/healthz`.
- `C:/Users/tamipinhasi/Documents/repos/ArUco_Chaser/tests/z_ekf_calibration_webapp/templates/index.html`
  - existing browser UI for fetching/applying Z_EKF config and viewing recorded sessions.
- `C:/Users/tamipinhasi/Documents/repos/ArUco_Chaser/tests/z_ekf_calibration_webapp/PLAN.md`
  - documents the completed MSPy, Flask, route, UI, and validation work for the old Z_EKF logger.

Reuse guidance:

- Reuse the desktop orchestration flow almost directly.
- Reuse config and file-selection ideas, but not the API-key-based analysis flow.
- Reuse the idea of a Pi Flask app with a background recorder, session metadata, JSONL logs, and download endpoints.
- Replace old `Z_EKF` names, old MSP v1 IDs, and `debug[0..7]` assumptions with new `ALTITUDE`/`alt_est`/MSP2 names and dedicated estimator log fields.
- Keep `run_config.json` private. Use `run_config.example.json` as the template and do not copy Wi-Fi passwords into this repository.
- Analysis and tuning recommendations should happen in this Codex session on local downloaded files, using local scripts/tools that Codex creates or updates as needed.

Recommended tuning iteration:

1. Select a tune profile on the Pi webapp.
2. Start logging.
3. Fly with altitude-control mode disabled first, so estimator quality can be measured before controller behavior is added.
4. Land and stop logging.
5. Download the bundle from the Pi over Wi-Fi/AP.
6. Parse Blackbox plus MSP sidecar metadata on the local computer.
7. Compute estimator metrics: innovation distribution, normalized innovation squared, reject percentage, recovery count, delay estimate, velocity plausibility, hover drift, acceleration residual, bias convergence, and step/reset events.
8. Ask Codex to analyze the local downloaded bundle and recommend parameter changes with a reason and expected effect.
9. Push a candidate tune to the Pi/FC.
10. Repeat until the estimator is stable enough for controller work.

Design rule: every tuning recommendation must be traceable to logged evidence. If the logs cannot show why a gate fired, why velocity drifted, or why a reset happened, add logging before changing the estimator again.

## Betaflight Implementation Path

1. Define the replacement boundary: introduce `ALTITUDE_MODE`/`BOXALTITUDE`; any temporary `BARO_MODE`/`BOXBARO` alias must route to the new implementation, while old `USE_BARO_ALTHOLD` behavior is removed rather than preserved.
2. Extract the current estimator from `position.c` into a standalone altitude estimator module.
3. Keep `getEstimatedAltitudeCm()`, `getAltitude()`, and `getEstimatedVario()` as temporary API adapters until callers move to the new module.
4. Add timestamped baro measurement plumbing.
5. Add estimator status/state structs that can feed MSP, Blackbox, and debug modes from one source.
6. Add or clean up CLI settings for estimator parameters, with explicit units/scales and Blackbox header names. Reuse old names only when semantics match.
7. Add MSP2 estimator config/status/log-control commands for the Pi logger/webapp, and avoid making the current v1 althold commands the long-term interface.
8. Add dedicated Blackbox estimator fields and header values needed for analysis.
9. Build the Pi-side logging bundle format and local parser around those fields.
10. Add delay compensation.
11. Add the position-consistent height-rate complementary output.
12. Validate the estimator and log-analysis loop before revisiting controllers.

## Validation Plan

Bench tests:

- stationary for several minutes: velocity should stay near zero, accel bias should settle, altitude should not walk quickly.
- hand lift/drop: altitude and velocity signs must be correct.
- pressure disturbance near baro: estimator should gate/recover without permanent lockout.
- artificial baro step in replay: offset/step logic should handle it without a violent velocity spike.

Log replay tests:

- replay old hover logs.
- replay aggressive throttle punch-outs.
- replay high-vibration logs.
- compare baro raw altitude, estimated altitude, estimated velocity, and innovation.
- verify the parser can recover the exact estimator parameters from Blackbox headers and MSP sidecar metadata.
- verify analysis can identify gate rejects, recovery periods, baro steps, resets, and stale/delayed samples.

Pi logger workflow tests:

- start/stop logging from the Pi webapp.
- verify every flight bundle contains log data, config snapshot, tune profile ID, and notes.
- verify a candidate tune can be written through MSP/MSP2 without using CLI text parsing.
- verify log files can be downloaded immediately after landing over the Pi Wi-Fi/AP.

Flight tests:

- hover with BARO mode disabled, estimator only.
- slow climbs/descents.
- throttle punches.
- forward flight with tilt.
- land/takeoff ground-effect checks.

Acceptance criteria before controller work:

- velocity estimate has correct sign and plausible magnitude.
- no gate deadlock after realistic baro disturbances.
- no large altitude jump from ordinary propwash.
- accel bias remains bounded.
- baro delay estimate is visible in logs and approximately tuned.
- estimator reset/step events are explicit in debug/log output.
- a tuning recommendation can be traced back to specific logged metrics, not only subjective flight feel.

## Main Differences From ArduPilot

What to copy:

- delayed baro fusion.
- baro innovation gating.
- height timeout/recovery behavior.
- position-consistent vertical-rate output.
- source abstraction.

What to simplify:

- use a 1D vertical estimator, not EKF3's full 24-state system.
- do not implement EKF lanes/affinity yet.
- do not add a freely learned baro-bias state unless another altitude source exists or the bias is tightly constrained as step/datum management.

What to be stricter about:

- log every estimator reset or baro offset step.
- separate estimator outputs from display filters.
- keep controllers from directly reading raw baro.
- replace old althold behavior instead of maintaining it as a hidden fallback.
- make MSP/CLI/Blackbox migration part of the design, not an afterthought.
- design the Pi logging workflow before trying to tune gates in flight.
