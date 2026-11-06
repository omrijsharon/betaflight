#!/usr/bin/env python3
"""Check actual MSP handler bodies against the Pi client (Linux/WSL, GCC).

Extract only the three estimator switch cases so unrelated MSP/hardware services
need no mocks. Compile those unchanged bodies with the real streambuf and filter.
"""
import ctypes as C
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from althold.pi_logger.msp_client import (  # noqa: E402
    DEFAULT_CONFIG, encode_config, parse_config, parse_status,
    MSP2_BETAFLIGHT_ALT_EST_CONFIG as GET_CONFIG,
    MSP2_BETAFLIGHT_SET_ALT_EST_CONFIG as SET_CONFIG,
    MSP2_BETAFLIGHT_ALT_EST_STATUS as GET_STATUS,
)


def main():
    source = (ROOT / 'src/main/msp/msp.c').read_text()
    cases = []
    for name in ('ALT_EST_CONFIG', 'SET_ALT_EST_CONFIG', 'ALT_EST_STATUS'):
        match = re.search(r'    case MSP2_BETAFLIGHT_' + name + r':.*?        break;', source, re.S)
        assert match, name
        cases.append(match.group())
    wrapper = '''
#include "flight/altitude_estimator.c"
#include "common/streambuf.h"
#include "msp/msp_protocol_v2_betaflight.h"
#define MSP_RESULT_ERROR -1
void reset(void) {
    altitudeEstimatorConfig_System = pgResetTemplate_altitudeEstimatorConfig;
    altitudeEstimatorInit();
    altitudeEstimatorUpdate(0, false, true, 1000, 0, true, 0);
    altitudeEstimatorUpdate(10000, true, false, 0, 0, true, 0);
    altitudeEstimatorUpdate(20000, true, false, 0, 0, true, -9.80665f);
}
int command(int cmd, unsigned char *input, int length, unsigned char *output) {
    sbuf_t in, out;
    sbuf_t *src = sbufInit(&in, input, input + length);
    sbuf_t *dst = sbufInit(&out, output, output + 256);
    switch (cmd) {
''' + '\n'.join(cases) + '''
    default: return MSP_RESULT_ERROR;
    }
    return sbufPtr(dst) - output;
}
'''
    with tempfile.TemporaryDirectory(prefix='bf-altitude-protocol-') as directory:
        tmp = Path(directory)
        (tmp / 'protocol.c').write_text(wrapper)
        subprocess.run([
            'gcc', '-shared', '-fPIC', '-O2', '-ffast-math', '-Wall', '-Wextra', '-Werror',
            '-DUNIT_TEST', '-I' + str(ROOT / 'src/test/unit'), '-I' + str(ROOT / 'src/main'),
            str(tmp / 'protocol.c'), str(ROOT / 'src/main/common/streambuf.c'),
            '-lm', '-o', str(tmp / 'protocol.so'),
        ], check=True)
        lib = C.CDLL(str(tmp / 'protocol.so'))
        lib.command.argtypes = [C.c_int, C.c_char_p, C.c_int, C.c_void_p]
        lib.command.restype = C.c_int
        lib.reset()

        def command(cmd, payload=b''):
            out = C.create_string_buffer(256)
            size = lib.command(cmd, payload, len(payload), out)
            return None if size < 0 else out.raw[:size]

        original = command(GET_CONFIG)
        assert len(original) == 16 and parse_config(original) == DEFAULT_CONFIG
        candidate = dict(DEFAULT_CONFIG, flags=0, baro_delay_ms=75, history_ms=200,
                         baro_noise_cm=130, accel_noise_cms2=42, gate_sigma_x10=45)
        assert command(SET_CONFIG, encode_config(candidate)) == b''
        assert parse_config(command(GET_CONFIG)) == candidate
        saved = command(GET_CONFIG)
        for invalid in [saved[:i] for i in range(len(saved))] + [saved + b'\0', b'\x02' + saved[1:]]:
            assert command(SET_CONFIG, invalid) is None
            assert command(GET_CONFIG) == saved, 'Invalid write partially changed configuration'
        extreme = dict(DEFAULT_CONFIG, flags=65535, history_ms=65535, baro_delay_ms=65535)
        assert command(SET_CONFIG, encode_config(extreme)) == b''
        clamped = parse_config(command(GET_CONFIG))
        assert clamped['flags'] == 1 and clamped['history_ms'] == clamped['baro_delay_ms'] == 250
        assert command(231) is None and command(232, b'\0' * 28) is None
        payload = command(GET_STATUS)
        status = parse_status(payload)
        assert len(payload) == 53 and status['timestamp_us'] == 20000
        assert status['velocity_cms'] == -10 and status['accel_world_z_cms2'] == -981
        assert status['baro_altitude_cm'] == 1000 and status['baro_age_ms'] == 20
        assert status['configured_delay_ms'] == 60 and status['reject_streak'] == 0
        print('MSP v3 PASS: firmware/client defaults, config round-trip, limits, malformed/version rejection, signed status')


if __name__ == '__main__':
    main()
