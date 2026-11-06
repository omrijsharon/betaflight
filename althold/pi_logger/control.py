"""Version 1 altitude control MSP2 codec for Pi consumers and firmware contract tests."""
import struct

GET, SET, STATUS = 0x3013, 0x3014, 0x3015
FIELDS = ("z_p", "z_d", "v_p", "v_i", "v_d", "speed_cms", "accel_cms2", "jerk_cms3")
DEFAULTS = dict(zip(FIELDS, (100, 40, 300, 100, 0, 100, 100, 400)))
RANGES = ((0, 500), (0, 200), (0, 1200), (0, 1200), (0, 200), (10, 500), (10, 300), (20, 1000))
CONFIG = struct.Struct("<B8H")
SAMPLE = struct.Struct("<BIBBh12f")
CHANNELS = ("version", "fc_us", "mode", "reason", "stick_offset", "height_m", "speed_mps",
            "height_ref_m", "speed_ref_mps", "accel_ref_mps2", "speed_cmd_mps",
            "p_mps2", "i_mps2", "d_mps2", "collective", "ceiling", "cos_tilt")


def decode_config(raw):
    version, *values = CONFIG.unpack(raw)
    if version != 1:
        raise ValueError("unsupported altitude controller")
    return dict(zip(FIELDS, values))


def encode_config(values):
    if values.keys() != DEFAULTS.keys() or any(type(values[n]) is not int or not lo <= values[n] <= hi
                                               for n, (lo, hi) in zip(FIELDS, RANGES)):
        raise ValueError("unknown, missing or out-of-range control setting")
    return CONFIG.pack(1, *(values[n] for n in FIELDS))


def decode_status(raw):
    data = dict(zip(CHANNELS, SAMPLE.unpack(raw)))
    if data["version"] != 1:
        raise ValueError("unsupported control sample")
    return data
