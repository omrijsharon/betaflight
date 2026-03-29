#!/usr/bin/env python3
"""Generate a para-drop activation key from a Betaflight MCU ID."""

from __future__ import annotations

import re
import sys


PRODUCT_STRING = "PARADROP_V1_4"
FNV1A64_OFFSET_BASIS = 0xCBF29CE484222325
FNV1A64_PRIME = 0x100000001B3
MCU_ID_RE = re.compile(r"^[0-9a-fA-F]{24}$")


def fnv1a64(data: bytes) -> int:
    value = FNV1A64_OFFSET_BASIS
    for byte in data:
        value ^= byte
        value = (value * FNV1A64_PRIME) & 0xFFFFFFFFFFFFFFFF
    return value


def normalize_mcu_id(value: str) -> str:
    value = value.strip()
    if value.lower().startswith("mcu_id "):
        value = value.split(None, 1)[1]

    if not MCU_ID_RE.fullmatch(value):
        raise ValueError("mcu_id must be exactly 24 hex characters")

    return value.lower()


def activation_key_for_mcu_id(mcu_id: str) -> str:
    normalized = normalize_mcu_id(mcu_id)
    payload = (PRODUCT_STRING + normalized).encode("ascii")
    return f"{fnv1a64(payload):016x}"


def main() -> int:
    raw_mcu_id = sys.argv[1] if len(sys.argv) > 1 else input("Paste mcu_id: ")

    try:
        key = activation_key_for_mcu_id(raw_mcu_id)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(key)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
