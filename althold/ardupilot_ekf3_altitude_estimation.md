# ArduPilot EKF3 Altitude And Vertical Velocity Estimation

This document summarizes how ArduPilot estimates altitude and vertical velocity for Copter using EKF3, with emphasis on what we need to re-implement a barometer + IMU vertical estimator in Betaflight.

Primary source is the local ArduPilot checkout at `C:\Users\tamipinhasi\Documents\repos\ardupilot`. Public ArduPilot docs are used only for high-level confirmation:

- EKF overview: https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html
- EKF source selection: https://ardupilot.org/plane/docs/common-ekf-sources.html
- EKF3 affinity / lanes: https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html

## Key Conclusions

ArduPilot Copter uses EKF3 as the normal attitude and position estimator. EKF3 can run multiple "cores" or "lanes", usually one per IMU, and the vehicle uses the primary lane's output.

The EKF3 vertical position and velocity are part of a larger 24-state navigation EKF. The relevant states are:

- `stateStruct.velocity.z`: local NED vertical velocity, positive down, in m/s.
- `stateStruct.position.z`: local NED vertical position, positive down, in m.
- `stateStruct.accel_bias`: body-frame delta-velocity bias states, including the vertical accelerometer bias contribution.

Barometer altitude is a positive-up height measurement. EKF3 converts it to a positive-down observation by setting `velPosObs[5] = -hgtMea`.

When only barometer + IMU are used for vertical estimation, ArduPilot does not fuse a baro-derived vertical velocity. Vertical velocity comes from IMU integration and is corrected indirectly when baro height innovations update the velocity state through the covariance cross-terms.

ArduPilot exposes two vertical-rate concepts:

- EKF vertical velocity: `get_velocity_NED().z` / `get_velocity_D()`.
- A vertical-position derivative from a third-order complementary filter: `getPosDownDerivative()`. This is intentionally different from EKF velocity and is used when a controller needs a rate that is kinematically consistent with the reported vertical position, especially under high vibration.

## Source Configuration

EKF source selection is configured through `EK3_SRCx_*` parameters in `AP_NavEKF_Source`.

Relevant files:

- `../ardupilot/libraries/AP_NavEKF/AP_NavEKF_Source.h`
- `../ardupilot/libraries/AP_NavEKF/AP_NavEKF_Source.cpp`

Important details:

- `SourceZ` values: `NONE=0`, `BARO=1`, `RANGEFINDER=2`, `GPS=3`, `BEACON=4`, `EXTNAV=6`.
- Default source set 1 uses `EK3_SRC1_POSZ = BARO`.
- Default source set 1 uses `EK3_SRC1_VELZ = GPS`.
- `EK3_SRC_OPTIONS` bit 0 can fuse all configured velocity sources.

For a baro + IMU-only implementation, the ArduPilot-equivalent vertical setup is:

- position Z source: barometer.
- velocity Z source: none.
- no separate baro vertical-velocity measurement.

## Barometer Height Production

Relevant files:

- `../ardupilot/libraries/AP_Baro/AP_Baro.cpp`
- `../ardupilot/libraries/AP_Baro/AP_Baro_atmosphere.cpp`

Flow:

1. Driver backends update pressure and temperature.
2. `AP_Baro::update()` calculates corrected pressure per sensor.
3. For air barometers, altitude is computed from ground pressure and corrected pressure with `get_altitude_difference()`.
4. Wind and thrust pressure corrections can be applied before altitude conversion when those features are enabled.
5. `baro.get_altitude(selected_baro)` is later read by EKF3.

Important functions:

- `AP_Baro::update()`: updates sensor altitude and the climb-rate filter.
- `AP_Baro::get_altitude_difference_simple()`: simple atmosphere conversion from pressure ratio to altitude difference.
- `AP_Baro::get_altitude_difference()`: wrapper that may use the extended atmosphere model.

## EKF3 State Prediction

Relevant files:

- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.h`
- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`

State layout is declared in `NavEKF3_core::state_elements`:

- quaternion: local NED to body.
- velocity: local NED, m/s.
- position: local NED, m.
- gyro bias.
- accelerometer delta-velocity bias.
- magnetic field states.
- wind states.

Prediction flow:

1. `NavEKF3::UpdateFilter()` calls each active core.
2. `NavEKF3_core::UpdateFilter()` calls `readIMUData()`.
3. `readIMUData()` reads gyro delta angle and accelerometer delta velocity, handles IMU switching, learns inactive sensor biases, downsamples IMU data into a FIFO, and extracts the delayed IMU sample used for EKF fusion.
4. `UpdateStrapdownEquationsNED()` corrects IMU delta velocity for learned bias, rotates body-frame delta velocity into NED, adds gravity to the down axis, integrates velocity, then integrates position.
5. `CovariancePrediction()` grows the covariance using gyro noise, accelerometer noise, gyro bias process noise, and accelerometer bias process noise.

Critical point: EKF3 predicts vertical velocity by integrating corrected vertical acceleration. Baro height later corrects it statistically.

## Delayed Baro Fusion

Relevant files:

- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`

ArduPilot fuses delayed measurements against a delayed EKF state, not against the latest output state.

Baro input path:

1. `readBaroData()` reads `baro.get_altitude(selected_baro)`.
2. It timestamps the measurement as:
   - baro last update time
   - minus `EK3_HGT_DELAY`
   - minus half of the local EKF update period.
3. It clamps that timestamp so it does not precede the oldest buffered IMU state.
4. It pushes the sample into `storedBaro`.
5. `selectHeightForFusion()` recalls `baroDataDelayed` from `storedBaro` at `imuDataDelayed.time_ms`.

This delay compensation is one of the main implementation details worth copying into Betaflight.

## Height Source Selection

Relevant file:

- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`

Function:

- `NavEKF3_core::selectHeightForFusion()`

Behavior:

- Reads rangefinder and barometer samples.
- Selects active height source from `EK3_SRCx_POSZ`.
- Supports barometer, rangefinder, GPS, beacon, external navigation, and no-height modes.
- Falls back to barometer if GPS, rangefinder, beacon, or external-navigation height is lost.
- Maintains `baroHgtOffset` when not using baro as the active source so EKF3 can switch back to baro without a large step.
- Can fuse a synthetic constant zero height when no height source is configured.

For baro-only Betaflight work, the important branch is:

- active height source is BARO.
- `hgtMea = baroDataDelayed.hgt - baroHgtOffset`.
- `posDownObsNoise = sq(EK3_ALT_M_NSE)`.
- observation is `velPosObs[5] = -hgtMea`.

## Height Fusion And Vertical Velocity Correction

Relevant file:

- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`

Function:

- `NavEKF3_core::FuseVelPosNED()`

Fusion behavior:

1. EKF3 performs sequential scalar fusion for velocity, horizontal position, and height.
2. Height innovation is:
   - `innovVelPos[5] = stateStruct.position.z - velPosObs[5]`
3. Innovation variance is:
   - `P[9][9] + R_OBS_DATA_CHECKS[5]`
4. The consistency gate uses `EK3_HGT_I_GATE`.
5. If the gate passes, or height fusion has timed out, or bad IMU data is detected, height is fused.
6. If height fusion has timed out, `ResetHeight()` resets vertical position to the latest height measurement.
7. In the scalar update, `stateIndex = 9` for height. Kalman gains are calculated from `P[i][9]`, so the height innovation can update:
   - position down.
   - velocity down through `P[6][9]`.
   - accelerometer bias through `P[13..15][9]`.

This is how barometer measurements correct altitude velocity without differentiating the barometer.

Ground-effect handling:

- When takeoff or touchdown is expected and baro is active, EKF3 inflates baro observation variance with `gndEffectBaroScaler`.
- During scalar fusion, it applies a dead zone to baro height innovations using `EK3_GND_EFF_DZ`.

## Vertical Velocity Measurements

Relevant files:

- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`

EKF3 can fuse vertical velocity from GPS or external navigation if configured:

- `readGpsData()` sets `useGpsVertVel` when GPS reports vertical velocity and `EK3_SRCx_VELZ` allows GPS.
- `SelectVelPosFusion()` sets `fuseVelVertData` and fills `velPosObs[2]` for GPS vertical velocity.
- External navigation can also provide vertical velocity when enabled.

For our baro + IMU-only target, this path should remain absent. Do not create a fake vertical velocity measurement by differentiating baro height and fusing it as independent data; it double-counts the baro measurement and makes the filter overconfident.

## Output Predictor And Position-Consistent Height Rate

Relevant files:

- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
- `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp`

Function:

- `NavEKF3_core::calcOutputStates()`
- `NavEKF3_core::getPosDownDerivative()`

EKF3 runs on a delayed fusion horizon. `calcOutputStates()` propagates the output state to the current time horizon using latest IMU data and applies a complementary output observer so control loops do not see sharp EKF correction steps.

For vertical rate, EKF3 also runs a third-order complementary filter:

- input: output vertical position and current vertical acceleration.
- state: `vertCompFiltState.pos`, `.vel`, `.acc`.
- parameter: `EK3_HRT_FILT`, crossover frequency in Hz.
- output: `getPosDownDerivative()` returns `vertCompFiltState.vel + velOffsetNED.z`.

The source comments cite:

- Widnall and Sinha, "Optimizing the Gains of the Baro-Inertial Vertical Channel".
- Khosravian, Trumpf, Mahony, Hamel, "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements".

Implementation detail worth copying: expose both the EKF velocity state and a position-consistent vertical-rate output. Use the position-consistent rate for control fallback under high vibration or after estimator resets.

## AHRS And Copter Consumption

Relevant files:

- `../ardupilot/libraries/AP_AHRS/AP_AHRS.cpp`
- `../ardupilot/libraries/AC_AttitudeControl/AC_PosControl.cpp`
- `../ardupilot/ArduCopter/inertia.cpp`
- `../ardupilot/ArduCopter/mode_althold.cpp`

Flow:

1. `AP_AHRS::_get_velocity_NED()` returns EKF3 velocity.
2. `AP_AHRS::get_velocity_D()` normally returns `get_velocity_NED().z`.
3. If `high_vibes` is true, `get_velocity_D()` falls back to `get_vert_pos_rate_D()`.
4. `AP_AHRS::get_vert_pos_rate_D()` returns `EKF3.getPosDownDerivative()`.
5. `AC_PosControl::update_estimates()` reads vertical position from AHRS and velocity from AHRS; under high vibration it uses the vertical-position derivative fallback.
6. `Copter::read_inertia()` calls `pos_control->update_estimates(vibration_check.high_vibes)`.
7. `ModeAltHold::run()` converts pilot throttle stick to desired climb rate and runs the vertical controller.

The estimator/control boundary is clean: Copter control code does not directly read baro. It consumes AHRS position and velocity outputs.

## Important Parameters

ArduPilot parameter names below are useful references, even if Betaflight should use its own names and scaling.

| ArduPilot parameter | Meaning |
| --- | --- |
| `EK3_SRCx_POSZ` | Primary vertical position source. Baro is value `1`. |
| `EK3_SRCx_VELZ` | Vertical velocity source. GPS is value `3`; none is value `0`. |
| `EK3_ALT_M_NSE` | Height measurement RMS noise. Used as baro height observation noise when baro is active. |
| `EK3_HGT_DELAY` | Height measurement delay relative to inertial measurements. |
| `EK3_HGT_I_GATE` | Height innovation gate size in percent of sigma. |
| `EK3_ACC_P_NSE` | Accelerometer process noise. |
| `EK3_ABIAS_P_NSE` | Accelerometer bias process noise. |
| `EK3_ACC_BIAS_LIM` | Accelerometer bias limit. |
| `EK3_TAU_OUTPUT` | Output complementary filter time constant. |
| `EK3_HRT_FILT` | Height-rate complementary filter crossover frequency. |
| `EK3_GND_EFF_DZ` | Dead zone for baro ground-effect height spikes. |

For Copter defaults in this ArduPilot checkout:

- `ALT_M_NSE_DEFAULT = 2.0f`
- `ACC_P_NSE_DEFAULT = 0.35f`
- `ABIAS_P_NSE_DEFAULT = 0.02f`
- `HGT_I_GATE_DEFAULT = 500`
- `EK3_HGT_DELAY = 60 ms`
- `EK3_HRT_FILT = 2.0 Hz`
- `EK3_TAU_OUTPUT = 25 centiseconds`

## File And Function Map

| Area | File | Functions / symbols |
| --- | --- | --- |
| EKF frontend | `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3.cpp` | `NavEKF3::UpdateFilter`, `NavEKF3::getPosD`, `NavEKF3::getPosDownDerivative`, `NavEKF3::resetHeightDatum` |
| EKF state layout | `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.h` | `state_elements`, `baro_elements`, `imu_elements`, `vertCompFiltState` |
| EKF main update | `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` | `NavEKF3_core::UpdateFilter`, `UpdateStrapdownEquationsNED`, `CovariancePrediction`, `calcOutputStates` |
| IMU and baro reads | `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` | `readIMUData`, `readBaroData`, `calcFiltBaroOffset`, `correctEkfOriginHeight` |
| Height and velocity fusion | `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp` | `SelectVelPosFusion`, `selectHeightForFusion`, `FuseVelPosNED`, `ResetHeight`, `ResetPositionD`, `ResetVelocity` |
| EKF outputs | `../ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp` | `getVelNED`, `getPosD`, `getPosDownDerivative`, `getAccelBias` |
| Source selection | `../ardupilot/libraries/AP_NavEKF/AP_NavEKF_Source.h/.cpp` | `SourceZ`, `getPosZSource`, `useVelZSource`, `haveVelZSource` |
| Barometer altitude | `../ardupilot/libraries/AP_Baro/AP_Baro.cpp` | `AP_Baro::update`, `get_altitude` |
| Pressure model | `../ardupilot/libraries/AP_Baro/AP_Baro_atmosphere.cpp` | `get_altitude_difference`, `get_altitude_difference_simple` |
| AHRS outputs | `../ardupilot/libraries/AP_AHRS/AP_AHRS.cpp` | `_get_velocity_NED`, `get_velocity_D`, `get_vert_pos_rate_D`, `get_relative_position_D_origin` |
| Copter estimate consumption | `../ardupilot/libraries/AC_AttitudeControl/AC_PosControl.cpp` | `AC_PosControl::update_estimates` |
| Copter scheduler path | `../ardupilot/ArduCopter/inertia.cpp` | `Copter::read_inertia` |
| Copter AltHold mode | `../ardupilot/ArduCopter/mode_althold.cpp` | `ModeAltHold::init`, `ModeAltHold::run` |

## What To Re-Implement, Not Copy Blindly

Worth copying conceptually:

- delayed baro fusion against a delayed inertial state.
- IMU delta-velocity integration with gravity correction.
- baro height innovation gating.
- timeout/reset path when height fusion is stale.
- output predictor/current-time estimate separate from delayed EKF fusion.
- separate position-consistent vertical-rate output.
- source abstraction that can later accept GPS, rangefinder, external nav, etc.

Not necessary to copy for Betaflight's first vertical estimator:

- full 24-state EKF.
- magnetometer, wind, airspeed, beacon, optical flow, and horizontal navigation states.
- multi-lane EKF3 affinity machinery.

Important difference:

ArduPilot does not estimate a dedicated barometer bias state in the EKF state vector. It handles baro datum reset, GPS-origin correction, and switching offsets outside the core state. If Betaflight uses a baro-bias state with only baro + IMU, that state is weakly observable and must be constrained carefully.
