"""Exercise production RX mode selection, controller and mixer fallback on Linux/WSL.

No hardware or simulator process is launched. Compile both feature-enabled and
feature-disabled mode selection; do not substitute a copy of the mode policy.
"""
import ctypes as C
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
ANGLE, HORIZON, ALTITUDE = 1, 2, 8  # runtime_config.h flight mode flags
OFF, WAIT_CENTER, HOLD, SPEED, LOCKED = range(5)

WRAPPER = r'''
#include "platform.h"
#include "flight/altitude_control.c"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "sensors/sensors.h"
uint8_t armingFlags;
uint16_t flightModeFlags;
static uint32_t requests;
static bool haveAcc, fail, rxHealthy;
bool IS_RC_MODE_ACTIVE(boxId_e box) { return (requests & (1u << box)) != 0; }
bool failsafeIsActive(void) { return fail; }
bool failsafeIsReceivingRxData(void) { return rxHealthy; }
bool sensors(uint32_t mask) { return haveAcc && (mask & SENSOR_ACC); }
uint16_t enableFlightMode(flightModeFlags_e mask) { return flightModeFlags |= mask; }
uint16_t disableFlightMode(flightModeFlags_e mask) { return flightModeFlags &= ~mask; }
float mixerAltitudeCeiling(void) { return .8f; }
void reset(void) {
    requests = armingFlags = flightModeFlags = 0;
    haveAcc = rxHealthy = true;
    fail = false;
#ifdef USE_ALTITUDE_CONTROL
    altitudeControlConfig_System = pgResetTemplate_altitudeControlConfig;
    s = (altitudeControlStatus_t){0};
    switchReady = applied = false;
#endif
}
unsigned updateModes(int aux, int angle, int horizon, int acc, int failsafe) {
    requests = (aux ? 1u << BOXALTITUDE : 0) | (angle ? 1u << BOXANGLE : 0) |
        (horizon ? 1u << BOXHORIZON : 0);
    haveAcc = acc;
    fail = failsafe;
MODE_SELECTION
    return flightModeFlags;
}
#ifdef USE_ALTITUDE_CONTROL
void step(unsigned now, int armed, int allowed, int healthy, int stick, float velocity, float tilt) {
    armingFlags = armed ? ARMED : 0;
    altitudeControlStep(now, armed, IS_RC_MODE_ACTIVE(BOXALTITUDE), allowed, healthy,
        stick, 2, velocity, tilt, .37f, .8f);
}
int controlMode(void) { return altitudeControlStatus()->mode; }
int controlReason(void) { return altitudeControlStatus()->reason; }
#endif
float mix(unsigned currentTimeUs, float mixerThrottle, float manual, int rx) {
    rxHealthy = rx;
    float throttle = manual;
    const bool launchControlActive = false;
    (void)currentTimeUs; (void)mixerThrottle; (void)launchControlActive;
#ifdef USE_ALTITUDE_CONTROL
MIXER
#endif
    return throttle;
}
'''


def check(lib, enabled):
    if enabled:
        lib.step.argtypes = [C.c_uint] + [C.c_int]*4 + [C.c_float]*2
    lib.mix.argtypes = [C.c_uint, C.c_float, C.c_float, C.c_int]
    lib.mix.restype = C.c_float
    lib.reset()

    def modes(aux=1, angle=0, horizon=0, acc=1, failsafe=0):
        return lib.updateModes(aux, angle, horizon, acc, failsafe)

    # ALTITUDE alone must level even before the 100 Hz controller has run.
    assert modes() == (ANGLE if enabled else 0), "ALTITUDE selection lost ANGLE before admission"
    assert modes(horizon=1) == (ANGLE if enabled else HORIZON)
    assert modes(acc=0) == 0  # preserve the existing accelerometer requirement
    assert modes(aux=0, angle=1) == ANGLE
    assert modes(aux=0, horizon=1) == HORIZON
    assert modes(aux=0, horizon=1, failsafe=1) == ANGLE
    assert modes(aux=0) == 0
    if not enabled:
        assert abs(lib.mix(10000, .37, .21, 1)-.21) < 1e-6
        print("PASS: feature-disabled ANGLE/HORIZON/failsafe and manual throttle unchanged")
        return

    now = 100000

    def tick(aux=1, armed=1, allowed=1, healthy=1, stick=0, velocity=0., tilt=1., delta=10000):
        nonlocal now
        now += delta
        modes(aux=aux)
        lib.step(now, armed, allowed, healthy, stick, velocity, tilt)
        flags = modes(aux=aux)
        output = lib.mix(now, .37, .21, 1)
        return lib.controlMode(), flags, output

    def prepare():
        lib.reset()
        tick(aux=0, armed=0)
        tick(aux=0)

    def engage():
        prepare()
        state, flags, output = tick(stick=-150)
        assert state == WAIT_CENTER and flags == ANGLE | ALTITUDE
        assert abs(output-.37) < 1e-6
        assert tick()[0:2] == (HOLD, ANGLE | ALTITUDE)
        assert tick(stick=500)[0:2] == (SPEED, ANGLE | ALTITUDE)

    # Reproduce the earlier flight: excessive speed rejects admission while
    # the pilot's switch remains ALTITUDE. No automatic throttle retry.
    prepare()
    state, flags, output = tick(velocity=1.295)
    assert state == LOCKED and lib.controlReason() == 4  # AC_ENTRY
    assert flags == ANGLE and abs(output-.21) < 1e-6
    assert tick()[0:2] == (LOCKED, ANGLE)
    assert modes(horizon=1) == ANGLE
    print("PASS: rejected engagement and latched lockout retain ANGLE, suppress HORIZON, use manual throttle")

    # No automatic engagement when armed with ALTITUDE already selected.
    lib.reset()
    assert tick(armed=0)[0:2] == (OFF, ANGLE)
    assert tick()[0:2] == (LOCKED, ANGLE)

    for name, failure in (
        ("60-degree tilt cutoff", {"tilt": .5}),
        ("estimator/stale sensor fault", {"healthy": 0}),
        ("mode conflict", {"allowed": 0}),
        ("controller timeout", {"delta": 51000}),
    ):
        engage()
        state, flags, output = tick(**failure)
        assert state == LOCKED and flags == ANGLE, (name, state, flags)
        assert abs(output-.21) < 1e-6, name
        assert tick()[0:2] == (LOCKED, ANGLE), name
        assert tick(aux=0)[0:2] == (OFF, 0)
        assert tick()[0:2] == (WAIT_CENTER, ANGLE | ALTITUDE)
        print(f"PASS: {name} releases throttle but preserves ANGLE until switch OFF")

    # Mixer runs faster than mode selection; test its immediate release paths
    # before the next RX update clears ALTITUDE_MODE.
    for name, age, rx, failsafe in (("stale task", 51000, 1, 0), ("RX loss", 1000, 0, 0),
                                   ("failsafe", 1000, 1, 1)):
        engage()
        modes(failsafe=failsafe)
        assert abs(lib.mix(now+age, .37, .21, rx)-.21) < 1e-6
        assert lib.controlMode() == LOCKED
        assert modes(failsafe=failsafe) == ANGLE, name
    engage()
    assert modes(aux=0, angle=1) == ANGLE
    assert abs(lib.mix(now+1000, .37, .21, 1)-.21) < 1e-6
    assert lib.controlMode() == LOCKED
    assert modes(aux=0, horizon=1) == HORIZON
    assert modes(aux=0) == 0
    print("PASS: mixer watchdog/RX/failsafe release retains ANGLE; switch OFF restores selected attitude mode")


def main():
    core = (ROOT/"src/main/fc/core.c").read_text()
    start = core.index("#ifdef USE_ALTITUDE_CONTROL\n    const bool altitudeModeRequested")
    mode_selection = core[start:core.index("\n#ifdef USE_GPS_RESCUE", start)]
    mixer = (ROOT/"src/main/flight/mixer.c").read_text()
    mixer = re.search(r"#ifdef USE_ALTITUDE_CONTROL\n(    if \(altitudeControlActive\(\)\).*?)\n#endif", mixer, re.S).group(1)
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        (tmp/"modes.c").write_text(WRAPPER.replace("MODE_SELECTION", mode_selection).replace("MIXER", mixer))
        for enabled in (True, False):
            output = tmp/f"modes_{enabled}.so"
            subprocess.run(["gcc", "-shared", "-fPIC", "-O2", "-ffast-math", "-Wall", "-Wextra", "-Werror",
                "-DUNIT_TEST", *(["-DUSE_ALTITUDE_CONTROL"] if enabled else []),
                "-I"+str(ROOT/"src/test/unit"), "-I"+str(ROOT/"src/main"),
                str(tmp/"modes.c"), "-lm", "-o", str(output)], check=True)
            check(C.CDLL(str(output)), enabled)


if __name__ == "__main__":
    main()
