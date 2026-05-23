from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field
from typing import Any

MSP2_BETAFLIGHT_ALT_EST_CONFIG = 0x3010
MSP2_BETAFLIGHT_SET_ALT_EST_CONFIG = 0x3011
MSP2_BETAFLIGHT_ALT_EST_STATUS = 0x3012

ALT_EST_VERSION = 1

CONFIG_FIELDS = [
    "flags",
    "accel_noise_cms2",
    "accel_bias_noise_cms2",
    "accel_bias_limit_cms2",
    "baro_noise_cm",
    "baro_delay_ms",
    "innov_var_floor_cm2",
    "history_ms",
    "gate_sigma_x10",
    "recovery_start_frames",
    "recovery_r_scale_x10",
    "recovery_decay_tc_frames",
    "step_innov_thresh_cm",
    "step_rate_thresh_cms",
    "step_rate_filter_tau_ms",
    "step_streak_frames",
    "step_offset_alpha_x1000",
    "step_offset_limit_cm",
    "height_rate_lpf_hz_x100",
]

DEFAULT_CONFIG = {
    "version": ALT_EST_VERSION,
    "flags": 0x0007,
    "accel_noise_cms2": 35,
    "accel_bias_noise_cms2": 2,
    "accel_bias_limit_cms2": 200,
    "baro_noise_cm": 150,
    "baro_delay_ms": 60,
    "innov_var_floor_cm2": 400,
    "history_ms": 300,
    "gate_sigma_x10": 50,
    "recovery_start_frames": 30,
    "recovery_r_scale_x10": 500,
    "recovery_decay_tc_frames": 20,
    "step_innov_thresh_cm": 150,
    "step_rate_thresh_cms": 200,
    "step_rate_filter_tau_ms": 150,
    "step_streak_frames": 5,
    "step_offset_alpha_x1000": 100,
    "step_offset_limit_cm": 300,
    "height_rate_lpf_hz_x100": 200,
}

STATUS_FIELDS = [
    "timestamp_us",
    "flags",
    "altitude_cm",
    "velocity_cms",
    "position_rate_cms",
    "baro_altitude_cm",
    "accel_world_z_cms2",
    "accel_bias_cms2",
    "innovation_cm",
    "gate_cm",
    "r_eff_cm2",
    "s_cm2",
    "baro_offset_cm",
    "baro_age_ms",
    "configured_delay_ms",
    "reject_streak",
    "recovery_frames",
    "step_streak",
]


def _crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _read_exact(port: Any, size: int) -> bytes:
    data = port.read(size)
    if len(data) != size:
        raise TimeoutError(f"short MSP read: expected {size}, got {len(data)}")
    return data


def _read_i32(raw: int) -> int:
    return struct.unpack("<i", struct.pack("<I", raw))[0]


def parse_config(payload: bytes) -> dict[str, int]:
    if not payload or payload[0] != ALT_EST_VERSION:
        raise ValueError("unsupported altitude estimator config payload")

    offset = 1
    out: dict[str, int] = {"version": payload[0]}
    widths = [2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2]
    for name, width in zip(CONFIG_FIELDS, widths):
        if width == 1:
            out[name] = payload[offset]
        else:
            out[name] = struct.unpack_from("<H", payload, offset)[0]
        offset += width
    return out


def encode_config(config: dict[str, int]) -> bytes:
    merged = dict(DEFAULT_CONFIG)
    merged.update(config)
    parts = [struct.pack("<B", ALT_EST_VERSION)]
    widths = [2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2]
    for name, width in zip(CONFIG_FIELDS, widths):
        value = int(merged[name])
        parts.append(struct.pack("<B" if width == 1 else "<H", value))
    return b"".join(parts)


def parse_status(payload: bytes) -> dict[str, int]:
    if not payload or payload[0] != ALT_EST_VERSION:
        raise ValueError("unsupported altitude estimator status payload")

    version, timestamp_us, flags = struct.unpack_from("<BII", payload, 0)
    offset = struct.calcsize("<BII")
    signed_values = [_read_i32(v) for v in struct.unpack_from("<12I", payload, offset)]
    offset += struct.calcsize("<12I")
    delay_ms, reject_streak, recovery_frames, step_streak = struct.unpack_from("<4H", payload, offset)

    values = [timestamp_us, flags, *signed_values, delay_ms, reject_streak, recovery_frames, step_streak]
    out = {"version": version}
    out.update(dict(zip(STATUS_FIELDS, values)))
    return out


@dataclass
class SerialMspClient:
    port: str
    baudrate: int = 115200
    timeout_s: float = 0.4
    _serial: Any = field(default=None, init=False, repr=False)

    def open(self) -> None:
        if self._serial is None:
            import serial

            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def request(self, command: int, payload: bytes = b"") -> bytes:
        self.open()
        assert self._serial is not None
        body = struct.pack("<BHH", 0, command, len(payload)) + payload
        frame = b"$X<" + body + bytes([_crc8_dvb_s2(body)])
        self._serial.write(frame)
        self._serial.flush()
        return self._read_response(command)

    def _read_response(self, expected_command: int) -> bytes:
        assert self._serial is not None
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if _read_exact(self._serial, 1) != b"$":
                continue
            if _read_exact(self._serial, 1) != b"X":
                continue
            direction = _read_exact(self._serial, 1)
            if direction not in (b">", b"!"):
                continue
            header = _read_exact(self._serial, 5)
            flags, command, size = struct.unpack("<BHH", header)
            payload = _read_exact(self._serial, size)
            crc = _read_exact(self._serial, 1)[0]
            if crc != _crc8_dvb_s2(header + payload):
                raise ValueError("bad MSP2 checksum")
            if direction == b"!":
                raise RuntimeError(f"MSP command {command:#x} returned an error")
            if command != expected_command:
                continue
            _ = flags
            return payload
        raise TimeoutError(f"no MSP response for command {expected_command:#x}")

    def get_alt_est_config(self) -> dict[str, int]:
        return parse_config(self.request(MSP2_BETAFLIGHT_ALT_EST_CONFIG))

    def set_alt_est_config(self, config: dict[str, int]) -> dict[str, int]:
        self.request(MSP2_BETAFLIGHT_SET_ALT_EST_CONFIG, encode_config(config))
        return self.get_alt_est_config()

    def get_alt_est_status(self) -> dict[str, int]:
        return parse_status(self.request(MSP2_BETAFLIGHT_ALT_EST_STATUS))


class FakeMspClient:
    def __init__(self) -> None:
        self.config = dict(DEFAULT_CONFIG)
        self.started = time.monotonic()
        self.bias = 0.0

    def close(self) -> None:
        return None

    def get_alt_est_config(self) -> dict[str, int]:
        return dict(self.config)

    def set_alt_est_config(self, config: dict[str, int]) -> dict[str, int]:
        self.config.update({key: int(value) for key, value in config.items() if key in CONFIG_FIELDS})
        return self.get_alt_est_config()

    def get_alt_est_status(self) -> dict[str, int]:
        t = time.monotonic() - self.started
        altitude = 60.0 * math.sin(t * 0.35)
        velocity = 21.0 * math.cos(t * 0.35)
        innovation = 8.0 * math.sin(t * 1.7)
        self.bias += 0.002 * math.sin(t * 0.2)
        return {
            "version": ALT_EST_VERSION,
            "timestamp_us": int(t * 1_000_000) & 0xFFFFFFFF,
            "flags": 0x0007,
            "altitude_cm": int(altitude),
            "velocity_cms": int(velocity),
            "position_rate_cms": int(velocity * 0.9),
            "baro_altitude_cm": int(altitude + innovation),
            "accel_world_z_cms2": int(-7.0 * math.sin(t * 0.7)),
            "accel_bias_cms2": int(self.bias),
            "innovation_cm": int(innovation),
            "gate_cm": 250,
            "r_eff_cm2": int(self.config["baro_noise_cm"] ** 2),
            "s_cm2": int(self.config["baro_noise_cm"] ** 2 + 400),
            "baro_offset_cm": 0,
            "baro_age_ms": 0,
            "configured_delay_ms": self.config["baro_delay_ms"],
            "reject_streak": 0,
            "recovery_frames": 0,
            "step_streak": 0,
        }
