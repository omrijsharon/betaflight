from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from althold.analysis.analyze_altitude_log import analyze
from althold.desktop_tools.field_helper import archive_session, latest_bundle, main as desktop_main
from althold.pi_logger.msp_client import DEFAULT_CONFIG, encode_config, parse_config


def test_config_payload_round_trip() -> None:
    payload = encode_config(DEFAULT_CONFIG)
    assert parse_config(payload)["baro_delay_ms"] == 60


def test_pi_logger_endpoints_and_session(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    from althold.pi_logger.app import create_app

    app = create_app(config_override={"mode": "fake", "log_root": str(tmp_path), "poll_hz": 20})
    client = app.test_client()

    assert client.get("/healthz").json["ok"] is True
    assert "altitude_cm" in client.get("/api/altitude/status").json

    start = client.post("/api/altitude/logging/start").json
    assert start["active"] is True
    time.sleep(0.12)
    stop = client.post("/api/altitude/logging/stop").json
    assert stop["active"] is False

    session_id = start["session_id"]
    session_dir = tmp_path / session_id
    assert (session_dir / "manifest.json").exists()
    assert (session_dir / "telemetry.jsonl").exists()
    assert (session_dir / "events.jsonl").exists()
    assert client.get(f"/api/altitude/sessions/{session_id}/download").status_code == 200


def test_analysis_outputs_summary(tmp_path: Path) -> None:
    session = tmp_path / "session"
    session.mkdir()
    rows = []
    for idx in range(20):
        rows.append(
            {
                "host_time_ns": idx,
                "status": {
                    "flags": 0 if idx < 10 else 8,
                    "altitude_cm": idx,
                    "baro_altitude_cm": idx + 1,
                    "velocity_cms": 10,
                    "accel_world_z_cms2": 1,
                    "accel_bias_cms2": 0,
                    "innovation_cm": 2,
                    "s_cm2": 4,
                },
            }
        )
    (session / "telemetry.jsonl").write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
    (session / "events.jsonl").write_text("", encoding="utf-8")

    summary = analyze(session)
    assert summary["sample_count"] == 20
    assert summary["reject_count"] == 10
    assert summary["normalized_innovation"]["mean"] == 1.0


def test_desktop_helper_dry_run_and_archive(tmp_path: Path, capsys) -> None:
    local = tmp_path / "downloaded"
    session = local / "session1"
    session.mkdir(parents=True)
    (session / "manifest.json").write_text("{}", encoding="utf-8")
    assert latest_bundle(local) == session
    archive = archive_session(session, tmp_path / "archives")
    assert archive.suffix == ".zip"

    assert desktop_main(["commands"]) == 0
    assert "ssh" in capsys.readouterr().out
