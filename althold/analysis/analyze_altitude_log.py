from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any


REJECT_FLAG = 1 << 3
RECOVERY_FLAG = 1 << 4
STEP_FLAG = 1 << 6


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    if not path.exists():
        return rows
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def session_dir(path: Path) -> Path:
    if path.is_file():
        return path.parent
    return path


def values(records: list[dict[str, Any]], name: str) -> list[float]:
    out = []
    for record in records:
        status = record.get("status", {})
        if name in status:
            out.append(float(status[name]))
    return out


def count_flags(records: list[dict[str, Any]], flag: int) -> int:
    return sum(1 for record in records if int(record.get("status", {}).get("flags", 0)) & flag)


def stat_block(data: list[float]) -> dict[str, float | int | None]:
    if not data:
        return {"count": 0, "min": None, "max": None, "mean": None, "std": None, "rms": None}
    mean = statistics.fmean(data)
    rms = math.sqrt(statistics.fmean([x * x for x in data]))
    return {
        "count": len(data),
        "min": min(data),
        "max": max(data),
        "mean": mean,
        "std": statistics.pstdev(data) if len(data) > 1 else 0.0,
        "rms": rms,
    }


def normalized_innovation(records: list[dict[str, Any]]) -> list[float]:
    out = []
    for record in records:
        status = record.get("status", {})
        s = float(status.get("s_cm2", 0.0))
        if s > 0.0:
            out.append(float(status.get("innovation_cm", 0.0)) / math.sqrt(s))
    return out


def estimate_baro_delay(records: list[dict[str, Any]]) -> dict[str, Any]:
    accel = values(records, "accel_world_z_cms2")
    velocity = values(records, "velocity_cms")
    if len(accel) < 20 or len(velocity) < 20:
        return {"best_lag_samples": None, "correlation": None}

    n = min(len(accel), len(velocity))
    accel = accel[:n]
    velocity = velocity[:n]
    best = {"best_lag_samples": 0, "correlation": 0.0}
    for lag in range(-20, 21):
        if lag < 0:
            a = accel[-lag:]
            v = velocity[: len(a)]
        elif lag > 0:
            a = accel[: n - lag]
            v = velocity[lag:]
        else:
            a = accel
            v = velocity
        if len(a) < 10:
            continue
        ma = statistics.fmean(a)
        mv = statistics.fmean(v)
        num = sum((x - ma) * (y - mv) for x, y in zip(a, v))
        den_a = math.sqrt(sum((x - ma) ** 2 for x in a))
        den_v = math.sqrt(sum((y - mv) ** 2 for y in v))
        corr = num / (den_a * den_v) if den_a > 0.0 and den_v > 0.0 else 0.0
        if abs(corr) > abs(float(best["correlation"])):
            best = {"best_lag_samples": lag, "correlation": corr}
    return best


def analyze(path: Path) -> dict[str, Any]:
    root = session_dir(path)
    records = load_jsonl(root / "telemetry.jsonl")
    events = load_jsonl(root / "events.jsonl")
    alt = values(records, "altitude_cm")
    vel = values(records, "velocity_cms")
    bias = values(records, "accel_bias_cms2")
    innov = values(records, "innovation_cm")

    summary: dict[str, Any] = {
        "session": root.name,
        "sample_count": len(records),
        "event_count": len(events),
        "innovation": stat_block(innov),
        "normalized_innovation": stat_block(normalized_innovation(records)),
        "velocity_cms": stat_block(vel),
        "accel_bias_cms2": stat_block(bias),
        "reject_count": count_flags(records, REJECT_FLAG),
        "recovery_count": count_flags(records, RECOVERY_FLAG),
        "step_count": count_flags(records, STEP_FLAG),
        "baro_delay_correlation": estimate_baro_delay(records),
    }
    if alt:
        summary["altitude_drift_cm"] = alt[-1] - alt[0]
        summary["altitude_range_cm"] = max(alt) - min(alt)
    else:
        summary["altitude_drift_cm"] = None
        summary["altitude_range_cm"] = None
    return summary


def write_outputs(summary: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "summary.json", "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
        fh.write("\n")

    flat: dict[str, Any] = {}

    def flatten(prefix: str, value: Any) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                flatten(f"{prefix}.{key}" if prefix else key, child)
        else:
            flat[prefix] = value

    flatten("", summary)
    with open(output_dir / "summary.csv", "w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["metric", "value"])
        for key in sorted(flat):
            writer.writerow([key, flat[key]])


def maybe_plot(path: Path, output_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return
    root = session_dir(path)
    records = load_jsonl(root / "telemetry.jsonl")
    if not records:
        return
    t = [idx for idx, _ in enumerate(records)]
    output_dir.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(10, 6))
    plt.plot(t, values(records, "altitude_cm"), label="altitude_cm")
    plt.plot(t, values(records, "baro_altitude_cm"), label="baro_altitude_cm", alpha=0.65)
    plt.plot(t, values(records, "velocity_cms"), label="velocity_cms", alpha=0.8)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_dir / "altitude_summary.png")
    plt.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze an altitude logger session")
    parser.add_argument("session", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args()

    root = session_dir(args.session)
    output_dir = args.output_dir or (root / "analysis")
    summary = analyze(root)
    write_outputs(summary, output_dir)
    if args.plots:
        maybe_plot(root, output_dir)
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
