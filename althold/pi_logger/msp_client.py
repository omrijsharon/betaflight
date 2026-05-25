from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any

MSP2_BETAFLIGHT_ALT_EST_CONFIG = 0x3010
MSP2_BETAFLIGHT_SET_ALT_EST_CONFIG = 0x3011
MSP2_BETAFLIGHT_ALT_EST_STATUS = 0x3012
MSP_REBOOT = 68
MSP_STATUS = 101
MSP_BOXIDS = 119
MSP_EEPROM_WRITE = 250

ALT_EST_VERSION = 2
ARM_BOX_PERMANENT_ID = 0

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
    "pz_floor_cm",
    "pv_floor_cms",
    "pba_floor_cms2",
    "tilt_r_start_deg",
    "tilt_r_end_deg",
    "tilt_r_scale_x10",
    "accel_r_start_cms2",
    "accel_r_end_cms2",
    "accel_r_scale_x10",
]

CONFIG_WIDTHS = [
    2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2,
    2, 2, 2, 1, 1, 2, 2, 2, 2,
]

DEFAULT_CONFIG = {
    "version": ALT_EST_VERSION,
    "flags": 0x001F,
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
    "pz_floor_cm": 2,
    "pv_floor_cms": 2,
    "pba_floor_cms2": 1,
    "tilt_r_start_deg": 45,
    "tilt_r_end_deg": 75,
    "tilt_r_scale_x10": 50,
    "accel_r_start_cms2": 300,
    "accel_r_end_cms2": 800,
    "accel_r_scale_x10": 50,
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


def _crc_msp1(command: int, payload: bytes) -> int:
    crc = len(payload) ^ command
    for value in payload:
        crc ^= value
    return crc & 0xFF


def _read_exact(port: Any, size: int) -> bytes:
    data = port.read(size)
    if len(data) != size:
        raise TimeoutError(f"short MSP read: expected {size}, got {len(data)}")
    return data


def _read_i32(raw: int) -> int:
    return struct.unpack("<i", struct.pack("<I", raw))[0]


def _flag_bit(first_word: int, extra: bytes, index: int) -> bool:
    if index < 0:
        return False
    if index < 32:
        return bool(first_word & (1 << index))
    extra_index = index - 32
    byte_index = extra_index // 8
    bit_index = extra_index % 8
    if byte_index >= len(extra):
        return False
    return bool(extra[byte_index] & (1 << bit_index))


def parse_config(payload: bytes) -> dict[str, int]:
    if not payload or payload[0] != ALT_EST_VERSION:
        raise ValueError("unsupported altitude estimator config payload")

    offset = 1
    out: dict[str, int] = {"version": payload[0]}
    for name, width in zip(CONFIG_FIELDS, CONFIG_WIDTHS):
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
    for name, width in zip(CONFIG_FIELDS, CONFIG_WIDTHS):
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


def parse_msp_status(payload: bytes, arm_box_index: int = 0) -> dict[str, Any]:
    if len(payload) < 16:
        raise ValueError("short MSP_STATUS payload")

    cycle_time_us, i2c_errors, sensors, mode_flags = struct.unpack_from("<HHHI", payload, 0)
    current_profile = payload[10]
    system_load = struct.unpack_from("<H", payload, 11)[0]
    gyro_cycle_time = struct.unpack_from("<H", payload, 13)[0]
    extra_len = payload[15] & 0x0F
    offset = 16
    extra_flags = payload[offset: offset + extra_len]
    offset += extra_len

    arming_disable_count = payload[offset] if len(payload) > offset else 0
    offset += 1
    arming_disable_flags = struct.unpack_from("<I", payload, offset)[0] if len(payload) >= offset + 4 else 0
    offset += 4
    config_state_flags = payload[offset] if len(payload) > offset else 0

    return {
        "cycle_time_us": cycle_time_us,
        "i2c_errors": i2c_errors,
        "sensors": sensors,
        "mode_flags": mode_flags,
        "current_profile": current_profile,
        "system_load": system_load,
        "gyro_cycle_time": gyro_cycle_time,
        "extra_mode_flags_bytes": list(extra_flags),
        "arm_box_index": arm_box_index,
        "armed": _flag_bit(mode_flags, extra_flags, arm_box_index),
        "arming_disable_count": arming_disable_count,
        "arming_disable_flags": arming_disable_flags,
        "reboot_required": bool(config_state_flags & 0x01),
    }


@dataclass
class SerialMspClient:
    port: str
    baudrate: int = 115200
    timeout_s: float = 0.4
    _serial: Any = field(default=None, init=False, repr=False)
    _arm_box_index: int | None = field(default=None, init=False, repr=False)
    _io_lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def open(self) -> None:
        if self._serial is None:
            import serial

            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def request(self, command: int, payload: bytes = b"") -> bytes:
        with self._io_lock:
            self.open()
            assert self._serial is not None
            body = struct.pack("<BHH", 0, command, len(payload)) + payload
            frame = b"$X<" + body + bytes([_crc8_dvb_s2(body)])
            self._serial.write(frame)
            self._serial.flush()
            return self._read_response(command)

    def request_msp1(self, command: int, payload: bytes = b"") -> bytes:
        if command < 0 or command > 255:
            raise ValueError("MSP1 command must fit in one byte")
        if len(payload) > 255:
            raise ValueError("MSP1 payload too large")
        with self._io_lock:
            self.open()
            assert self._serial is not None
            frame = b"$M<" + bytes([len(payload), command]) + payload + bytes([_crc_msp1(command, payload)])
            self._serial.write(frame)
            self._serial.flush()
            return self._read_response_msp1(command)

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
                continue
            if direction == b"!":
                raise RuntimeError(f"MSP command {command:#x} returned an error")
            if command != expected_command:
                continue
            _ = flags
            return payload
        raise TimeoutError(f"no MSP response for command {expected_command:#x}")

    def _read_response_msp1(self, expected_command: int) -> bytes:
        assert self._serial is not None
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if _read_exact(self._serial, 1) != b"$":
                continue
            if _read_exact(self._serial, 1) != b"M":
                continue
            direction = _read_exact(self._serial, 1)
            if direction not in (b">", b"!"):
                continue
            size = _read_exact(self._serial, 1)[0]
            response_command = _read_exact(self._serial, 1)[0]
            payload = _read_exact(self._serial, size)
            crc = _read_exact(self._serial, 1)[0]
            if crc != _crc_msp1(response_command, payload):
                continue
            if direction == b"!":
                raise RuntimeError(f"MSP command {response_command:#x} returned an error")
            if response_command != expected_command:
                continue
            return payload
        raise TimeoutError(f"no MSP response for command {expected_command:#x}")

    def get_alt_est_config(self) -> dict[str, int]:
        return parse_config(self.request(MSP2_BETAFLIGHT_ALT_EST_CONFIG))

    def set_alt_est_config(self, config: dict[str, int]) -> dict[str, int]:
        self.request(MSP2_BETAFLIGHT_SET_ALT_EST_CONFIG, encode_config(config))
        return self.get_alt_est_config()

    def get_alt_est_status(self) -> dict[str, int]:
        return parse_status(self.request(MSP2_BETAFLIGHT_ALT_EST_STATUS))

    def get_arm_box_index(self) -> int:
        if self._arm_box_index is not None:
            return self._arm_box_index
        try:
            for page in range(4):
                box_ids = list(self.request_msp1(MSP_BOXIDS, bytes([page])))
                if ARM_BOX_PERMANENT_ID in box_ids:
                    self._arm_box_index = page * 32 + box_ids.index(ARM_BOX_PERMANENT_ID)
                    return self._arm_box_index
                if len(box_ids) < 32:
                    break
        except Exception:
            pass
        self._arm_box_index = 0
        return self._arm_box_index

    def get_fc_status(self) -> dict[str, Any]:
        payload = self.request_msp1(MSP_STATUS)
        return parse_msp_status(payload, self.get_arm_box_index())

    def save_config(self) -> dict[str, Any]:
        self.request_msp1(MSP_EEPROM_WRITE)
        return {"saved": True}

    def reboot(self) -> dict[str, Any]:
        try:
            payload = self.request_msp1(MSP_REBOOT, bytes([0]))
            reboot_mode = payload[0] if payload else 0
        finally:
            self.close()
        return {"rebooting": True, "mode": reboot_mode}


class FakeMspClient:
    def __init__(self) -> None:
        self.config = dict(DEFAULT_CONFIG)
        self.started = time.monotonic()
        self.bias = 0.0
        self.armed = False
        self.reboot_required = False
        self.saved_at: float | None = None

    def close(self) -> None:
        return None

    def get_alt_est_config(self) -> dict[str, int]:
        return dict(self.config)

    def set_alt_est_config(self, config: dict[str, int]) -> dict[str, int]:
        self.config.update({key: int(value) for key, value in config.items() if key in CONFIG_FIELDS})
        self.reboot_required = False
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

    def set_armed(self, armed: bool) -> None:
        self.armed = armed

    def get_fc_status(self) -> dict[str, Any]:
        return {
            "cycle_time_us": 125,
            "i2c_errors": 0,
            "sensors": 0,
            "mode_flags": 1 if self.armed else 0,
            "current_profile": 0,
            "system_load": 100,
            "gyro_cycle_time": 0,
            "extra_mode_flags_bytes": [],
            "arm_box_index": 0,
            "armed": self.armed,
            "arming_disable_count": 0,
            "arming_disable_flags": 0,
            "reboot_required": self.reboot_required,
        }

    def save_config(self) -> dict[str, Any]:
        if self.armed:
            raise RuntimeError("cannot save config while armed")
        self.saved_at = time.monotonic()
        return {"saved": True}

    def reboot(self) -> dict[str, Any]:
        if self.armed:
            raise RuntimeError("cannot reboot while armed")
        self.started = time.monotonic()
        return {"rebooting": True, "mode": 0}
