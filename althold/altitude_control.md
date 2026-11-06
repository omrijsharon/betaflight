# Firmware altitude control

Branch: altitude-control, based on feature/altitude-estimator (d22953df9).
Experimental implementation for manual takeoff, hover engagement, and manual landing.
The estimator remains unchanged. The controller uses its current unsmoothed height
and speed, with the existing barometer delay compensation.

## Pilot operation and mode admission

Build with USE_ALTITUDE_CONTROL in addition to the existing Blitz F435 flags,
including USE_MAG. No USE_BARO_ALTHOLD flag is needed; those old settings are not
used. In this build, Configurator exposes ALTITUDE (BOXALTITUDE, permanent ID 3)
when barometer and accelerometer are detected.

Use switch arming. Take off manually, establish a near-level steady hover with the
ALTITUDE switch OFF, then select it. Admission requires fresh fused pressure,
fresh IMU, calibrated sensors, a valid datum, a healthy RX, tilt cosine >=0.85,
absolute vertical speed <=0.8 m/s, and collective between 0.10 and 0.80 within the
configured throttle ceiling. These are admission checks, NOT airborne detection:
do not engage on the ground. An ON switch at arming cannot engage automatically.

The altitude task runs at 100 Hz. On admission it requests ANGLE automatically,
suppresses HORIZON, and waits for the mixer to observe ANGLE+ALTITUDE. It then
seeds output from the preceding actual manual collective. In this sense the
transfer has no deliberate throttle step. It does not remove the attitude change
from enabling ANGLE. LEVEL hover thrust is inferred from that collective and tilt;
incorrect trim, throttle transients or an accelerating entry can bias it.

Your initial hover stick position is ignored as a vertical request. Until you
move the stick within 100 PWM of midrc (inclusive), it brakes/holds. After that,
moving beyond 115 PWM from center requests speed; returning within 100 captures
a stop once. Between 100 and 115 on return it requests zero speed while waiting
for center capture; an existing HOLD remains latched throughout that range.
At midrc=1500, capture is 1400..1600; HOLD exits below 1385 or above 1615.
These thresholds are compile-time constants, not CLI/MSP settings. The speed
mapping is rescaled over the remaining 385 PWM so full stick retains the configured
maximum speed. This adds no parameter storage or runtime state.
Input is rcData, centered at configured midrc, not throttle-curve output.
It assumes conventional approximately 1000..2000 endpoints.

Switching OFF returns manual throttle authority immediately; physical stick position
then matters again. No automatic takeoff, touchdown detector, landing or disarm is
implemented. Switch back to manual before landing. Pilot roll/pitch/yaw remain
available; ALTITUDE does not hold horizontal position.

Failsafe/RX loss, GPS Rescue, launch mode, crash-flip, 3D, fixed-wing and stick arming
are incompatible. An estimator fault, >50 ms controller scheduling gap, >50 ms IMU
age, >500 ms without accepted pressure fusion, or >60-degree tilt removes ownership.
Single barometer rejections do not immediately disable it. Invalid/nonfinite input
and implausible >50 m/s vertical speed are rejected even under fast-math.
A fault latches: cycle the ALTITUDE switch OFF then ON to request another admission.
Existing failsafe/Rescue and disarm behavior take precedence; sensor fault alone
returns control to the pilot. This fallback can cause a throttle change.

## Control law and memory

Pilot speed request uses a signed smoothstep outside the deadband. A bounded
acceleration governor and jerk limit shape the speed reference.

Centering plans a three-phase constant-jerk stopping trajectory from measured
height/speed and current commanded acceleration: jerk into braking, constant
braking if needed, jerk back to zero acceleration. No allocation, history buffer,
matrix, iterative optimizer, or second estimator is added. Phase times are integer
microseconds to avoid accumulated float-duration error on long slow stops.
The fixed state and three phase slots use single-precision math.

While stopping or holding:
v_cmd = v_ref + (z_p/100)*(z_ref-z) + (z_d/100)*(v_ref-v).
The speed bound accommodates the reference's incoming speed.
During stick motion: v_cmd = v_ref and there is no height correction.

a_req = a_ref + (v_p/100)*(v_cmd-v) + I - (v_d/100)*filtered_dv_dt.
The derivative uses a 0.12-second first-order filter. I is bounded to +/-4 m/s^2.
Integration is inhibited when output saturation and speed error push the same way.

collective = hover_collective * sqrt(max(0, 1+a_req/g) / cos_tilt).
It is limited by zero and the current throttle ceiling. This is an approximate
quadratic thrust model, NOT identification of the real motor/prop/battery response.
The speed integral corrects small hover-model errors. Existing motor mixing,
thrust linearization, sag compensation and motor limits remain downstream.
Control bypasses manual throttle curve/boost and avoids applying tilt correction
twice. It respects the configured throttle-limit maximum. Full-down stick is
descent, so MOTOR_STOP, auto-disarm and low-stick PID reset must not trigger while
the controller owns throttle. GPS Rescue still owns throttle when active.

## CLI and defaults

The outer loop is PD, not PID: no second height integral.
The speed loop has P/I/D; D defaults to zero. Gains are fixed-point x100.

    set alt_ctrl_z_p = 100
    set alt_ctrl_z_d = 40
    set alt_ctrl_v_p = 300
    set alt_ctrl_v_i = 100
    set alt_ctrl_v_d = 0
    set alt_ctrl_speed_cms = 100
    set alt_ctrl_accel_cms2 = 100
    set alt_ctrl_jerk_cms3 = 400
    save

Defaults are provisional: height P=1/s, D=0.4; speed P=3/s, I=1/s^2, D=0;
speed +/-1 m/s, acceleration +/-1 m/s^2, jerk 4 m/s^3.
Height P range 0..500, height D 0..200, speed P/I 0..1200, speed D 0..200
(all x100); speed 10..500 cm/s, acceleration 10..300 cm/s^2,
jerk 20..1000 cm/s^3. CLI and MSP share one firmware settings definition.
Changing controller settings does not change estimator settings.

## MSP2, persistence and flight logging

API remains MSP 1.46. These are fork-specific MSP2 commands:

- 0x3013 GET config: version 1 byte, then eight LE uint16 values in CLI order above.
- 0x3014 SET config: same exact 17-byte payload. Firmware rejects armed writes,
  wrong version/length and any out-of-range field without partially updating.
  Read back after SET. Changes apply to the next engagement without reboot.
- 0x3015 GET status: 57 bytes, Python struct <BIBBh12f.
  Version, FC micros timestamp, mode, reason, signed stick offset; then twelve LE
  IEEE754 floats: z, v, z_ref, v_ref, a_ref, v_cmd, P, I, D, collective, ceiling, tilt.
  Height is meters, speed m/s, accelerations m/s^2, collective/ceiling normalized,
  tilt a cosine. Timestamp wraps every approximately 71.6 minutes.
- MSP_EEPROM_WRITE (250) persists ALL current FC settings, disarmed only.
  SET alone is RAM-only. No reboot is necessary; no automatic reboot is issued.

Modes: 0 OFF, 1 WAIT_CENTER, 2 HOLD (including braking), 3 SPEED, 4 LOCKED.
Reasons: 0 OK, 1 SWITCH, 2 CONFLICT, 3 ESTIMATE, 4 ENTRY, 5 TIMEOUT, 6 WAIT_ANGLE.
WAIT_ANGLE means admitted but awaiting the first ANGLE+ALTITUDE mixer tick.
When inactive, controller telemetry is diagnostic; it is not an applied actuator
measurement. Collective while active is requested before downstream motor mixing.

The real field service source is in sibling ArUco_Chaser:
Final_v3/tools/bf_altitude_logger. It now detects controller support, polls estimator
and controller with separate timestamps at requested 50 Hz, and snapshots both
configurations before each flight. Sync/export retains the flight index and puts
controller settings into the matching flight_NNNN.params.json. Status includes
the P/I/D terms, references, output ceiling and control state for tuning.

The native bf_altitude_control_config utility uses the same exclusive UART locks
and disarmed guards as bf_altitude_config. Stop the logger before tuning; restart it
afterward. On Pi (500000 baud, existing /dev/serial0 setup):

    sudo systemctl stop bf-altitude-logger
    sudo ./bin/bf_altitude_control_config get
    sudo ./bin/bf_altitude_control_config set --backup before-control.cfg --param alt_ctrl_v_p=300 --save
    sudo systemctl start bf-altitude-logger

Use a new backup filename per change. The tuner also supports restore and defaults.
See that tool's PARAMETERS.md for exact syntax. Do not open two UART clients.
The Python client in this firmware repo also provides get/set control config and
get control status, using the same checked wire codec, for integrations.
Neither source update installs itself on the Pi or flashes the FC.

## Verification and remaining work

Host tests compile actual C source with -O2 -ffast-math -Werror:
src/test/altitude_control_closed_loop.py and altitude_control_adapter.py.
They cover 200 stopping trajectories against the simulator's planner, arming and
mode latches, off-center engagement, faults/timeouts/nonfinite inputs, sensor
freshness, accepted-pressure timeout, configuration roundtrips and all truncated
writes, armed rejection, saturation and motor-lag vertical closed-loop cases.
Climb/descent cases use actual C Kalman fusion, 60 ms pressure delay, pressure noise,
IMU bias and a tilt change. Both finished about 0.020 m from the reference with
approximately zero vertical speed in that model. Estimator parity/protocol and
11 existing Python tooling tests passed unchanged.

The native Pi logger/tuners passed host CTest and UART PTY tests. The recorder was
also exercised through the real exporter/downloader with two different per-flight
gain snapshots. It still needs a native Pi rebuild/install, FC flash, disarmed
protocol check and measured polling cadence before use.

This is NOT a full attitude/motor SITL test or a physical flight validation.
The actual mixer integration compiles in the Blitz firmware, but attitude effects,
mixer saturation, thrust-model error, CPU task budget, and fault transitions on
hardware remain to be measured. First flight objective is engagement/hold/manual
exit from steady hover with modest limits. Flight testing is not done.


## Blitz F435 build footprint (2026-11-06)

Built with ARM GCC 10.3.1 and the user's cloud-build flags, USE_MAG and
USE_ALTITUDE_CONTROL. Compared against the exact parent d22953df9 with the same
toolchain, target config, release label and magnetometer flags:

| Region | Parent | Altitude control | Added |
| --- | ---: | ---: | ---: |
| FLASH1 | 425,724 B | 429,820 B | 4,096 B |
| RAM | 94,720 B | 94,864 B | 144 B |

The separate 524-byte FLASH region is unchanged. FLASH1 is 42.31% of 992 KiB;
RAM is 48.25% of 192 KiB. No heap allocations or additional estimator history.
These are linker region sizes, not measured worst-case stack or task timing.

HEX is local/ignored: obj/IFLIGHT_BLITZ_F435_ALTITUDE_CONTROL_MAG.hex.
It has not been flashed. The source base remains 4.5.2 with release label 4.5.3.

The native Pi changes are in a separate, already dirty checkout and were not
committed with firmware. A reviewed source-only backup is:
C:/Users/tamipinhasi/Documents/BetaflightSourceBackups/bf-altitude-control-pi-source-20261106.tar.gz
SHA-256: 33deafb294bbc27cc467bde822828abbced1797d8ac562505f97adc3d4dad945.
It includes tests and build dependencies, excludes field configs/logs and binaries,
and passed an independent unpack/build/test run. No device was changed.
