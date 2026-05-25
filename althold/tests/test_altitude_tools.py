from __future__ import annotations

import json
import subprocess
import struct
import time
from pathlib import Path

import pytest

from althold.analysis.analyze_altitude_log import analyze
from althold.desktop_tools.field_helper import archive_session, latest_bundle, main as desktop_main
from althold.pi_logger import field_network_manager
from althold.pi_logger.msp_client import DEFAULT_CONFIG, FakeMspClient, encode_config, parse_config, parse_msp_status


def test_config_payload_round_trip() -> None:
    payload = encode_config(DEFAULT_CONFIG)
    parsed = parse_config(payload)
    assert parsed["version"] == 2
    assert parsed["baro_delay_ms"] == 60
    assert parsed["tilt_r_scale_x10"] == 50
    assert parsed["accel_r_end_cms2"] == 800


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
    sessions = client.get("/api/altitude/sessions").json
    current = next(session for session in sessions if session["session_id"] == session_id)
    assert current["duration_s"] >= 0.1
    assert client.get(f"/api/altitude/sessions/{session_id}/download").status_code == 200
    assert client.delete(f"/api/altitude/sessions/{session_id}").json["deleted"] is True
    assert not session_dir.exists()


def test_msp_status_arm_state_parsing() -> None:
    base = struct.pack("<HHHIBHHB", 125, 0, 0, 1, 0, 100, 0, 0)
    tail = struct.pack("<BI", 0, 0) + bytes([0])
    assert parse_msp_status(base + tail, 0)["armed"] is True

    extended = struct.pack("<HHHIBHHB", 125, 0, 0, 0, 0, 100, 0, 1) + bytes([0b00001000]) + tail
    assert parse_msp_status(extended, 35)["armed"] is True


def test_pi_logger_fc_status_recommendations_and_save_guards(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    from althold.pi_logger.app import create_app

    app = create_app(config_override={
        "mode": "fake",
        "log_root": str(tmp_path / "logs"),
        "recommendation_path": str(tmp_path / "pending.json"),
        "auto_recording": False,
    })
    client = app.test_client()
    msp = app.config["ALTHOLD_MSP_CLIENT"]
    assert isinstance(msp, FakeMspClient)

    assert client.get("/api/fc/status").json["status"]["armed"] is False
    assert client.post("/api/altitude/config/save").status_code == 200
    msp.set_armed(True)
    assert client.post("/api/altitude/config/save").status_code == 409

    recommendation = {
        "schema": "althold-recommendation-v1",
        "session_id": "s1",
        "created_utc": "2026-05-25T00:00:00Z",
        "source_thread_id": None,
        "parameters": [],
    }
    assert client.post("/api/recommendations/pending", json=recommendation).json["exists"] is True
    assert client.get("/api/recommendations/pending").json["recommendation"]["session_id"] == "s1"
    assert client.post("/api/recommendations/pending/consume").json["exists"] is True
    assert client.get("/api/recommendations/pending").json["exists"] is False


def test_arm_watcher_auto_starts_and_stops_recording(tmp_path: Path) -> None:
    pytest.importorskip("flask")
    from althold.pi_logger.app import ArmStateWatcher, Recorder

    msp = FakeMspClient()
    recorder = Recorder(msp, tmp_path, poll_hz=20)
    watcher = ArmStateWatcher(msp, recorder, poll_hz=20)
    watcher.start()

    msp.set_armed(True)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not recorder.active():
        time.sleep(0.02)
    assert recorder.active() is True
    session_id = recorder.session_id

    msp.set_armed(False)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and recorder.active():
        time.sleep(0.02)
    assert recorder.active() is False
    assert session_id is not None
    assert (tmp_path / session_id / "events.jsonl").exists()


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
    assert summary["status_flag_counts"]["baro_reject"] == 10
    assert summary["baro_altitude_cm"]["mean"] == 10.5
    assert summary["baro_altitude_range_cm"] == 19.0
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


def test_field_network_template_writes_logger_config(tmp_path: Path) -> None:
    config_path = Path(__file__).parents[1] / "pi_logger" / "field_network.example.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    assert config["tailscale"]["auto_up_policy"] == "hotspot_only"
    assert config["logger"]["mode"] == "fake"

    out = tmp_path / "logger.json"
    field_network_manager.write_logger_config(config, out)
    written = json.loads(out.read_text(encoding="utf-8"))
    assert written["serial_port"] == "/dev/serial0"
    assert written["log_root"] == "/opt/althold/logs/pi_sessions"


def test_tailscale_up_uses_project_policy_flags(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[list[str]] = []

    monkeypatch.setattr(field_network_manager.shutil, "which", lambda name: "/usr/bin/tailscale")

    def fake_run(
        command: list[str],
        timeout: int = 30,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(field_network_manager, "run", fake_run)
    ok = field_network_manager.tailscale_up(
        {"tailscale": {"hostname": "omrijsharon-pi", "accept_routes": False, "ssh": False}}
    )

    assert ok is True
    assert calls == [["tailscale", "up", "--hostname=omrijsharon-pi", "--accept-routes=false"]]


def test_field_network_status_parses_tailscale_backend_state(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(field_network_manager, "tailscale_available", lambda: True)
    monkeypatch.setattr(field_network_manager, "active_ssid", lambda: "home")
    monkeypatch.setattr(field_network_manager, "active_connection", lambda iface="wlan0": "althold-home")

    def fake_run(
        command: list[str],
        timeout: int = 30,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(command, 0, json.dumps({"BackendState": "Stopped"}), "")

    monkeypatch.setattr(field_network_manager, "run", fake_run)
    current = field_network_manager.status({"wifi": {"interface": "wlan0"}})

    assert current["tailscale"] == "down"
