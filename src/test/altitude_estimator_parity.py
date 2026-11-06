#!/usr/bin/env python3
"""Compile the real firmware filter and compare it to the simulator on identical inputs.
Run on Linux/WSL: python3 src/test/altitude_estimator_parity.py --sim-repo ../betaflight-sim
No firmware/hardware process is launched. Build products live in a temporary directory.
"""
import argparse
import ctypes as C
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = r"""
#include "flight/altitude_estimator.c"
void parity_reset(void) {
    altitudeEstimatorConfig_System = pgResetTemplate_altitudeEstimatorConfig;
    altitudeEstimatorConfig_System.alt_est_flags = ALT_EST_CONFIG_DELAYED_FUSION;
    altitudeEstimatorConfig_System.alt_est_baro_noise_cm = 20;
    altitudeEstimatorInit();
    altitudeEstimatorUpdate(0, false, true, 0, 0, true, 0);
    altitudeEstimatorUpdate(10000, true, false, 0, 0, true, 0);
}
unsigned parity_step(unsigned now, float accel, int have, float height, unsigned captured, float *out) {
    altitudeEstimatorUpdate(now, true, have, height * 100, captured, true, accel);
    out[0] = altEst.state.z; out[1] = altEst.state.v; out[2] = altEst.state.ba;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) out[3+i*3+j] = altEst.state.P[i][j];
    return altEst.status.flags;
}
"""

ADAPTER = r"""
#include "flight/altitude_estimator.c"
#include "flight/position.c"
baro_t baro;
acc_t acc;
float rMat[3][3];
uint8_t armingFlags, debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
static timeUs_t testTime;
static bool calibrated;
timeUs_t micros(void) { return testTime; }
bool sensors(uint32_t mask) { return (mask & (SENSOR_BARO | SENSOR_ACC)) != 0; }
bool baroIsCalibrated(void) { return calibrated; }
bool accIsCalibrationComplete(void) { return true; }
float getBaroAltitude(void) { return baro.altitude; }
void mixerSetThrottleAltitudeCorrection(int correction) { (void)correction; }
// Display filtering is outside the estimator/adapter being tested.
float pt2FilterGain(float cutoff, float dt) { (void)cutoff; (void)dt; return 1; }
void pt2FilterInit(pt2Filter_t *filter, float gain) { (void)filter; (void)gain; }
float pt2FilterApply(pt2Filter_t *filter, float value) { (void)filter; return value; }
void adapter_reset(void) {
    memset(&baro, 0, sizeof(baro)); memset(&acc, 0, sizeof(acc));
    armingFlags = 0; calibrated = false;
    altitudeEstimatorConfig_System = pgResetTemplate_altitudeEstimatorConfig;
    altitudeEstimatorConfig_System.alt_est_baro_delay_ms = 60;
    positionConfig_System = pgResetTemplate_positionConfig;
    acc.dev.acc_1G_rec = 1; acc.isAccelUpdatedAtLeastOnce = true;
    rMat[2][2] = 1;
    positionInit();
}
unsigned adapter_poll(unsigned now, unsigned receipt, unsigned sequence, int ready, int armed, float specificG, float *out) {
    testTime = now; baro.sampleTimeUs = receipt; baro.sampleNumber = sequence;
    baro.altitude = 1000; calibrated = ready;
    armingFlags = armed ? ARMED : 0; acc.accADC[Z] = specificG;
    calculateEstimatedAltitude();
    out[0] = altEst.status.accelWorldZCms2;
    out[1] = altEst.status.baroAgeMs;
    out[2] = altEst.status.velocityCms;
    return altEst.status.flags;
}
"""

def check_adapter(tmp):
    (tmp/'adapter.c').write_text(ADAPTER)
    subprocess.run(['gcc', '-shared', '-fPIC', '-O2', '-ffast-math', '-std=gnu99', '-Wall', '-Wextra', '-Werror',
                    '-DUNIT_TEST', '-I'+str(ROOT/'src/test/unit'), '-I'+str(ROOT/'src/main'),
                    str(tmp/'adapter.c'), '-lm', '-o', str(tmp/'adapter.so')], check=True)
    lib = C.CDLL(str(tmp/'adapter.so'))
    lib.adapter_poll.argtypes = [C.c_uint, C.c_uint, C.c_uint, C.c_int, C.c_int, C.c_float, C.POINTER(C.c_float)]
    lib.adapter_poll.restype = C.c_uint
    out = (C.c_float*3)()
    lib.adapter_reset()
    flags = lib.adapter_poll(100000, 100000, 1, 0, 0, 1, out)
    assert not flags & 2, 'Uncalibrated barometer accepted'
    flags = lib.adapter_poll(110000, 110000, 2, 1, 0, 1, out)
    assert flags & 2 and abs(out[0]) < .001
    lib.adapter_poll(120000, 110000, 2, 1, 1, 1, out)
    for now in range(130000, 210000, 10000):
        flags = lib.adapter_poll(now, 110000, 2, 1, 1, 1, out)
        assert not flags & 4, 'Stale pressure was fused again'
    flags = lib.adapter_poll(210000, 200000, 3, 1, 1, 1, out)
    assert flags & 4 and abs(out[1]-70) < .001, (flags, list(out))
    flags = lib.adapter_poll(220000, 210000, 4, 1, 1, 1, out)
    assert flags & 4, 'Fresh equal-height measurement must be fused'
    flags = lib.adapter_poll(230000, 210000, 4, 1, 1, 0, out)
    assert not flags & 4 and abs(out[0]+980.665) < .01, (flags, list(out))
    assert out[2] < -9, 'Zero specific force must predict downward motion'
    print('sensor adapter PASS: calibration, freshness, identical values, receipt-minus-delay timing, freefall')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-repo', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.sim_repo.resolve()))
    from sim.vertical import EstimatorConfig
    from sim.delayed_estimator import DelayedVerticalEstimator
    with tempfile.TemporaryDirectory(prefix='bf-altitude-parity-') as tmp:
        tmp = Path(tmp)
        (tmp/'wrapper.c').write_text(WRAPPER)
        subprocess.run(['gcc', '-shared', '-fPIC', '-O2', '-ffast-math', '-std=gnu99', '-Wall', '-Wextra', '-Werror',
                        '-DUNIT_TEST', '-I'+str(ROOT/'src/test/unit'), '-I'+str(ROOT/'src/main'),
                        str(tmp/'wrapper.c'), '-lm', '-o', str(tmp/'filter.so')], check=True)
        check_adapter(tmp)
        lib = C.CDLL(str(tmp/'filter.so'))
        lib.parity_step.argtypes = [C.c_uint, C.c_float, C.c_int, C.c_float, C.c_uint, C.POINTER(C.c_float)]
        lib.parity_step.restype = C.c_uint
        lib.parity_reset()
        invalid_out = (C.c_float*12)()
        flags = lib.parity_step(20000, float('nan'), 0, 0, 0, invalid_out)
        assert flags & (1 << 15) and all(math.isfinite(x) for x in invalid_out)
        flags = lib.parity_step(30000, 0, 1, float('inf'), 30000, invalid_out)
        assert flags & (1 << 15) and not flags & 4 and all(math.isfinite(x) for x in invalid_out)
        print('fast-math nonfinite guards PASS')
        for scenario in ['freefall', 'bias', 'delayed_noise', 'dropout_outlier', 'jitter_fractional']:
            lib.parity_reset()
            ref = DelayedVerticalEstimator(EstimatorConfig(baro_noise_std_m=.2, bias_random_walk_std=.02))
            ref.reset(timestamp=.01)
            rng = random.Random(384)
            us = 10000
            z = v = 0.
            pending = []
            errors = [0.] * 12
            accepted = rejected = 0
            out = (C.c_float*12)()
            last_sample = None
            for n in range(1, 3001):
                dtus = rng.choice([8000, 10000, 12000]) if scenario == 'jitter_fractional' else 10000
                dt = dtus * 1e-6
                us += dtus
                t = us * 1e-6
                a = -9.80665 if scenario == 'freefall' else (.7*math.sin(t*1.1) if scenario != 'bias' else 0.)
                z += v*dt + .5*a*dt*dt
                v += a*dt
                accel = C.c_float(a + (.18 if scenario != 'freefall' else 0) + (rng.gauss(0,.03) if 'noise' in scenario else 0)).value
                ref.predict(accel, dt)
                if scenario != 'freefall' and n % 5 == 0 and not (scenario == 'dropout_outlier' and 800 < n < 1200):
                    capture = us - (3000 if scenario == 'jitter_fractional' else 0)
                    height = z - v*(us-capture)*1e-6 + rng.gauss(0,.06)
                    if scenario == 'dropout_outlier' and n == 500: height += 100
                    delay = 33000 if scenario in ['delayed_noise', 'jitter_fractional'] else 0
                    pending.append((us+delay, capture, C.c_float(height).value))
                sample = None
                if pending and pending[0][0] <= us:
                    _, capture, height = pending.pop(0)
                    sample = (capture, height)
                    last_sample = sample
                # Exercise consumed timestamp guards as well as gating.
                if scenario == 'dropout_outlier' and n in (701, 740):
                    sample = last_sample if n == 701 else (us-300000, z)
                if sample:
                    capture, height = sample
                    height = C.c_float(height).value
                    ok = ref.correct_baro(height, capture * 1e-6)
                    accepted += ok
                    rejected += not ok
                flags = lib.parity_step(us, accel, bool(sample), height if sample else 0, capture if sample else 0, out)
                if sample:
                    assert bool(flags & 4) == ok, (scenario, n, flags, ref.last_baro_status)
                expected = list(ref.state) + [x for row in ref.covariance for x in row]
                for i, (actual, want) in enumerate(zip(out, expected)):
                    assert math.isfinite(actual), (scenario, n, i)
                    errors[i] = max(errors[i], abs(actual-want))
                    assert abs(actual-want) < .003 + (3e-5 if i < 3 else 1e-4)*abs(want), (scenario, n, i, actual, want)
                # PSD principal minors, including the determinant. Symmetry alone is insufficient.
                p = [list(out[3+i*3:6+i*3]) for i in range(3)]
                assert all(p[i][i] >= -1e-7 for i in range(3))
                for i in range(3):
                    for j in range(i):
                        assert abs(p[i][j]-p[j][i]) < 1e-6
                        assert p[i][i]*p[j][j]-p[i][j]**2 >= -1e-5
                det = p[0][0]*(p[1][1]*p[2][2]-p[1][2]**2)-p[0][1]*(p[0][1]*p[2][2]-p[1][2]*p[0][2])+p[0][2]*(p[0][1]*p[1][2]-p[1][1]*p[0][2])
                assert det >= -1e-4, (scenario, n, det)
            if scenario == 'bias':
                assert abs(out[2]-.18) < .025, out[2]
                assert abs(out[1]) < .1, out[1]
            print(scenario, 'PASS', 'max z/v/bias errors', [round(e,7) for e in errors[:3]],
                  'max covariance error', round(max(errors[3:]),7), 'fused/rejected', accepted, rejected)

if __name__ == '__main__':
    main()
