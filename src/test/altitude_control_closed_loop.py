"""Run the production C estimator, controller and MSP handlers against a vertical plant.
Linux/WSL, GCC. No hardware, UART, or simulator backend needed.
"""
import ctypes as C
import math
import random
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from althold.pi_logger.control import DEFAULTS, GET, SET, STATUS, decode_config, decode_status, encode_config

WRAPPER = r'''
#include "flight/altitude_estimator.c"
#include "flight/altitude_control.c"
#include "common/streambuf.h"
#include "msp/msp_protocol_v2_betaflight.h"
#define MSP_RESULT_ERROR -1
static bool armed;
#define ARMING_FLAG(x) armed
static bool rxHealthy = true, auxOn = true, fail;
static unsigned modes;
#define ANGLE_MODE 1
#define ALTITUDE_MODE 2
#define GPS_RESCUE_MODE 4
#define BOXALTITUDE 3
#define FLIGHT_MODE(mask) (modes & (mask))
#define IS_RC_MODE_ACTIVE(box) auxOn
bool failsafeIsReceivingRxData(void) { return rxHealthy; }
bool failsafeIsActive(void) { return fail; }
float mixerAltitudeCeiling(void) { return .8f; }
float route(unsigned currentTimeUs, float mixerThrottle, int mode, int aux, int rx, int fs) {
    modes = mode; auxOn = aux; rxHealthy = rx; fail = fs;
    float throttle = .2f;
    const bool launchControlActive = false;
MIXER
    return throttle;
}
void reset(void) {
    altitudeControlConfig_System = pgResetTemplate_altitudeControlConfig;
    altitudeEstimatorConfig_System = pgResetTemplate_altitudeEstimatorConfig;
    altitudeEstimatorInit();
    s = (altitudeControlStatus_t){0};
    switchReady = false;
}
void step(unsigned now, int arm, int aux, int allow, int good, int stick,
    float z, float v, float tilt, float manual, float cap) {
    armed = arm;
    altitudeControlStep(now, arm, aux, allow, good, stick, z, v, tilt, manual, cap);
}
void plan(float z, float v, float a, float limit, float jerk) {
    s.aRef = a;
    capture(z, v, limit, jerk);
}
void advance(float dt, float *out) {
    advanceStop(dt);
    out[0] = s.zRef; out[1] = s.vRef; out[2] = s.aRef;
}
void estimate(unsigned now, int arm, int baro, float height, float accel, float *out) {
    altitudeEstimatorUpdate(now, arm, baro, height*100, now-60000, true, accel);
    out[0] = altEst.status.altitudeCm*.01f;
    out[1] = altEst.status.velocityCms*.01f;
}
int command(int cmd, unsigned char *data, int size, unsigned char *result) {
    sbuf_t in, out;
    sbuf_t *src = sbufInit(&in, data, data + size);
    sbuf_t *dst = sbufInit(&out, result, result + 128);
    switch (cmd) {
CASES
    default: return -1;
    }
    return sbufPtr(dst) - result;
}
'''


def run(lib):
    lib.step.argtypes = [C.c_uint] + [C.c_int]*5 + [C.c_float]*5
    lib.altitudeControlActive.restype = C.c_bool
    lib.altitudeControlCollective.argtypes = [C.c_float]
    lib.altitudeControlCollective.restype = C.c_float
    lib.command.argtypes = [C.c_int, C.c_char_p, C.c_int, C.c_void_p]
    lib.estimate.argtypes = [C.c_uint, C.c_int, C.c_int, C.c_float, C.c_float, C.POINTER(C.c_float)]

    def cmd(index, data=b""):
        out = C.create_string_buffer(128)
        n = lib.command(index, data, len(data), out)
        return out.raw[:n] if n >= 0 else None

    sys.path.insert(0, str(ROOT.parent / "betaflight-sim"))
    from sim.trajectory import StopTrajectory
    lib.plan.argtypes = [C.c_float]*5
    lib.advance.argtypes = [C.c_float, C.POINTER(C.c_float)]
    rng = random.Random(42)
    result = (C.c_float*3)()
    for _ in range(200):
        limit, jerk = rng.uniform(.1, 3), rng.uniform(.2, 10)
        z, v, a = rng.uniform(-20, 20), rng.uniform(-5, 5), rng.uniform(-limit, limit)
        reference = StopTrajectory(z, v, a, limit, jerk)
        lib.plan(z, v, a, limit, jerk)
        for _ in range(int(reference.duration/.01)+3):
            expected = reference.advance(.01)
            lib.advance(.01, result)
            assert max(abs(x-y) for x,y in zip(expected, result)) < .002, (expected, list(result))
    print("PASS: 200 randomized stopping trajectories versus simulator")
    lib.reset()
    cfg = cmd(GET)
    assert decode_config(cfg) == DEFAULTS
    trial = encode_config(dict(DEFAULTS, z_p=125, jerk_cms3=600))
    assert cmd(SET, trial) == b"" and cmd(GET) == trial
    for bad in [trial[:n] for n in range(len(trial))] + [trial+b"x", b"\x05"+trial[1:], trial[:-2]+b"\xff\xff"]:
        assert cmd(SET, bad) is None and cmd(GET) == trial
    assert cmd(SET, cfg) == b""
    now = 100000

    def tick(arm=1, aux=1, allow=1, good=1, stick=-150, z=2, v=0, tilt=1, manual=.37, cap=1, delta=10000):
        nonlocal now
        now += delta
        lib.step(now, arm, aux, allow, good, stick, z, v, tilt, manual, cap)
        if lib.altitudeControlActive():
            lib.altitudeControlCollective(manual)
        return decode_status(cmd(STATUS))

    tick(arm=0)
    assert tick()["mode"] == 4  # already switched on when arming
    tick(aux=0)
    assert tick(tilt=.7)["mode"] == 4
    tick(aux=0)
    first = tick()
    assert first["mode"] == 1 and abs(first["collective"]-.37) < 1e-6
    assert cmd(SET, trial) is None and cmd(GET) == cfg
    assert all(tick()["mode"] == 1 for _ in range(50))
    assert tick(stick=0)["mode"] == 2
    for sign in (-1, 1):
        tick(aux=0)
        assert tick(stick=sign*101)["mode"] == 1
        assert tick(stick=sign*101)["mode"] == 1
        assert tick(stick=sign*100)["mode"] == 2  # inclusive center capture
        target = tick(stick=sign*100)["height_ref_m"]
        for offset in (74, 92, 100, 101, 115, 100):
            state = tick(stick=sign*offset, z=3)
            assert state["mode"] == 2 and state["height_ref_m"] == target
        assert tick(stick=sign*116)["mode"] == 3
        for _ in range(400):
            state = tick(stick=sign*308)
        u = (308-115)/(500-115)
        assert abs(state["speed_cmd_mps"]-sign*u*u*(3-2*u)) < 1e-4
        for _ in range(400):
            state = tick(stick=sign*500)
        assert abs(state["speed_cmd_mps"]-sign) < 1e-4
        for offset in (115, 101):
            assert tick(stick=sign*offset)["mode"] == 3
        assert tick(stick=sign*100)["mode"] == 2
    print("PASS: signed ±100 capture, ±115 exit hysteresis, no recapture, rescaled speed mapping and endpoints")
    for key in ("good", "allow", "delta", "z"):
        assert tick(**{key: 55000 if key == "delta" else float("nan") if key == "z" else 0})["mode"] == 4
        assert tick()["mode"] == 4
        tick(aux=0)
        assert tick()["mode"] == 1
    assert tick(arm=0)["mode"] == 0
    print("PASS: switch/arming, hover seed, center latch, hysteresis, fault lockout, timeout and MSP/CLI gain contract")

    for direction in (-1, 1):
        lib.reset()
        z = v = accel = 0.
        hover = motor = .37
        history = [z]*7
        state = (C.c_float*2)()
        dt = .01
        for k in range(2400):
            arm = k >= 50
            aux = k >= 100
            stick = (-150 if k < 200 else direction*500 if 300 <= k < 800 else 0)
            # 20 Hz pressure, six samples latency, deterministic pressure noise and IMU bias.
            lib.estimate(now+10000, arm, k%5 == 0, history[0]+.025*math.sin(k*.23), accel+.025, state)
            tilt = .88 if k >= 1100 else 1.
            s = tick(arm=arm, aux=aux, stick=stick, z=state[0], v=state[1], tilt=tilt, manual=hover)
            demand = s["collective"] if s["mode"] in (1, 2, 3) else hover
            motor += (demand-motor)*dt/(.035+dt)
            accel = 9.80665*((motor/hover)**2*tilt-1)
            z += v*dt + .5*accel*dt*dt
            v += accel*dt
            history = history[1:]+[z]
            assert 0 <= demand <= 1
            if k == 799:
                assert abs(v-direction) < .12, (v, direction)
                released = z
        assert s["mode"] == 2 and abs(v) < .06 and abs(z-s["height_ref_m"]) < .12, (z, v, s)
        assert .25 < direction*(s["height_ref_m"] - released) < 1.2
        print(f"PASS: {'climb' if direction>0 else 'descent'}, brake, tilted hold. Final height error {z-s['height_ref_m']:+.3f} m, speed {v:+.3f} m/s")

    # Anti-windup with sustained unachievable demand and fixed measured velocity.
    tick(aux=0)
    tick()
    tick(stick=0)
    for _ in range(500):
        s = tick(stick=500, cap=.37)
    assert abs(s["collective"]-.37)<1e-5 and abs(s["i_mps2"])<1e-4
    print("PASS: throttle ceiling and saturation anti-windup")
    lib.route.argtypes = [C.c_uint, C.c_float] + [C.c_int]*4
    lib.route.restype = C.c_float
    lib.reset()
    tick(arm=0, aux=0)
    tick(aux=0)
    # Actual production mixer block waits for ANGLE+ALTITUDE then seeds the latest collective.
    now += 10000
    lib.step(now, 1, 1, 1, 1, -150, 2, 0, 1, .37, .8)
    assert abs(lib.route(now, .45, 0, 1, 1, 0)-.2) < 1e-6
    assert abs(lib.route(now, .45, 3, 1, 1, 0)-.45) < 1e-6
    for case in ((1,1,1,1000), (0,1,0,1000), (1,0,0,1000), (1,1,0,51000)):
        aux, rx, fs, age = case
        assert abs(lib.route(now+age, .45, 3, aux, rx, fs)-.2)<1e-6
        assert not lib.altitudeControlActive()
        tick(aux=0); tick()
    print("PASS: production mixer waits for ANGLE, exact handoff, immediate OFF/RX/failsafe/stale release")


def main():
    source = (ROOT/"src/main/msp/msp.c").read_text()
    blocks = re.findall(r"#ifdef USE_ALTITUDE_CONTROL\n(    case MSP2_.*?)\n#endif", source, re.S)
    assert len(blocks) == 2
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        mixer = (ROOT/"src/main/flight/mixer.c").read_text()
        mixer = re.search(r"#ifdef USE_ALTITUDE_CONTROL\n(    if \(altitudeControlActive\(\)\).*?)\n#endif", mixer, re.S).group(1)
        (tmp/"check.c").write_text(WRAPPER.replace("CASES", "\n".join(blocks)).replace("MIXER", mixer))
        subprocess.run(["gcc", "-shared", "-fPIC", "-O2", "-ffast-math", "-Wall", "-Wextra", "-Werror",
                        "-DUNIT_TEST", "-DUSE_ALTITUDE_CONTROL", "-I"+str(ROOT/"src/test/unit"),
                        "-I"+str(ROOT/"src/main"), str(tmp/"check.c"), str(ROOT/"src/main/common/streambuf.c"),
                        "-lm", "-o", str(tmp/"check.so")], check=True)
        run(C.CDLL(str(tmp/"check.so")))


if __name__ == "__main__":
    main()
