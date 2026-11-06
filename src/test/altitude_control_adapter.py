"""Compile production position task + estimator + controller; test sensor/mode admission."""
import ctypes as C
from pathlib import Path
import subprocess
import tempfile
from altitude_estimator_parity import ADAPTER, ROOT

SOURCE = ADAPTER.replace('#include "flight/position.c"', '#include "flight/altitude_control.c"\n#include "flight/position.c"') + r"""
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
rxConfig_t rxConfig_System;
uint16_t flightModeFlags;
static bool requested, conflict;
bool IS_RC_MODE_ACTIVE(boxId_e box) { return box == BOXALTITUDE && requested; }
bool failsafeIsActive(void) { return conflict; }
bool failsafeIsReceivingRxData(void) { return !conflict; }
bool isLaunchControlActive(void) { return false; }
bool isFlipOverAfterCrashActive(void) { return false; }
bool isFixedWing(void) { return false; }
bool isUsingSticksForArming(void) { return false; }
bool featureIsEnabled(const uint32_t mask) { (void)mask; return false; }
float mixerGetThrottle(void) { return 0.37f; }
float mixerAltitudeCeiling(void) { return 1.0f; }
int poll(unsigned now, int arm, int aux, int failure, int baroFresh, int accFresh) {
    requested = aux; conflict = failure;
    rxConfig_System.midrc = 1500;
    rcData[THROTTLE] = 1370;
    if (accFresh) acc.sampleTimeUs = now;
    float result[3];
    adapter_poll(now, now, baro.sampleNumber + baroFresh, 1, arm, 1, result);
    if (altitudeControlActive()) altitudeControlCollective(.37f);
    return altitudeControlStatus()->mode;
}
void resetControl(void) {
    adapter_reset();
    altitudeControlConfig_System = pgResetTemplate_altitudeControlConfig;
}
"""

def main():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d)
        (p/"adapter.c").write_text(SOURCE)
        subprocess.run(["gcc", "-shared", "-fPIC", "-O2", "-ffast-math", "-Wall", "-Wextra", "-Werror",
            "-DUNIT_TEST", "-DUSE_ALTITUDE_CONTROL", "-I"+str(ROOT/"src/test/unit"), "-I"+str(ROOT/"src/main"),
            str(p/"adapter.c"), "-lm", "-o", str(p/"adapter.so")], check=True)
        lib = C.CDLL(str(p/"adapter.so"))
        lib.resetControl()
        now = 100000
        def tick(arm=1, aux=1, failure=0, baro=1, accel=1):
            nonlocal now
            now += 10000
            return lib.poll(now, arm, aux, failure, baro, accel)
        tick(arm=0, aux=0)
        tick(aux=0)
        for _ in range(12): tick(aux=0)
        assert tick() == 1
        for _ in range(20): assert tick(baro=0) == 1  # tolerate transient rejection/dropout
        for _ in range(40): tick(baro=0)
        assert tick(baro=0) == 4
        for _ in range(15): assert tick() == 4  # no automatic re-engagement
        tick(aux=0)
        assert tick() == 1
        for _ in range(7): tick(accel=0)
        assert tick(accel=0) == 4
        tick(aux=0)
        assert tick() == 1
        assert tick(failure=1) == 4
        assert tick(arm=0) == 0
        print("PASS: actual 100Hz adapter admission, 500ms fusion watchdog, stale IMU, failsafe, rearm latch")

if __name__ == "__main__":
    main()
