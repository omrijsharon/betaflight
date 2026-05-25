from __future__ import annotations

import argparse
import json
import os
import shutil
import tempfile
import threading
import time
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from flask import Flask, Response, jsonify, request, send_file

from .msp_client import FakeMspClient, SerialMspClient


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def session_duration_s(session_dir: Path) -> float | None:
    event_duration = duration_from_events(session_dir / "events.jsonl")
    if event_duration is not None:
        return event_duration
    return duration_from_telemetry(session_dir / "telemetry.jsonl")


def duration_from_events(path: Path) -> float | None:
    if not path.exists():
        return None
    start_ns: int | None = None
    end_ns: int | None = None
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            host_time_ns = record.get("host_time_ns")
            if not isinstance(host_time_ns, int):
                continue
            if record.get("event") == "start" and start_ns is None:
                start_ns = host_time_ns
            if record.get("event") in {"stop", "stop_error", "disarmed"}:
                end_ns = host_time_ns
    if start_ns is None or end_ns is None or end_ns < start_ns:
        return None
    return (end_ns - start_ns) / 1_000_000_000.0


def duration_from_telemetry(path: Path) -> float | None:
    if not path.exists():
        return None
    first_ns: int | None = None
    last_ns: int | None = None
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            host_time_ns = record.get("host_time_ns")
            if not isinstance(host_time_ns, int):
                continue
            if first_ns is None:
                first_ns = host_time_ns
            last_ns = host_time_ns
    if first_ns is None or last_ns is None or last_ns < first_ns:
        return None
    return (last_ns - first_ns) / 1_000_000_000.0


def load_config(path: str | os.PathLike[str] | None) -> dict[str, Any]:
    defaults = {
        "mode": "fake",
        "serial_port": "/dev/ttyAMA0",
        "baudrate": 115200,
        "poll_hz": 50,
        "arm_poll_hz": 5,
        "auto_recording": True,
        "log_root": "althold/logs/pi_sessions",
        "recommendation_path": "althold/logs/pending_recommendation.json",
        "site_host": "0.0.0.0",
        "site_port": 8080,
    }
    if path and Path(path).exists():
        with open(path, "r", encoding="utf-8") as fh:
            defaults.update(json.load(fh))
    return defaults


def make_msp_client(config: dict[str, Any]) -> Any:
    if config.get("mode") == "fake":
        return FakeMspClient()
    return SerialMspClient(
        port=str(config["serial_port"]),
        baudrate=int(config.get("baudrate", 115200)),
        timeout_s=float(config.get("serial_timeout_s", 0.4)),
    )


class Recorder:
    def __init__(self, msp_client: Any, log_root: Path, poll_hz: float) -> None:
        self.msp_client = msp_client
        self.log_root = log_root
        self.poll_hz = poll_hz
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.session_dir: Path | None = None
        self.session_id: str | None = None

    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def status(self) -> dict[str, Any]:
        return {
            "active": self.active(),
            "session_id": self.session_id,
            "session_dir": str(self.session_dir) if self.session_dir else None,
            "poll_hz": self.poll_hz,
        }

    def start(self, reason: str = "manual") -> dict[str, Any]:
        with self._lock:
            if self.active():
                assert self.session_id is not None
                return {"active": True, "session_id": self.session_id}

            self.log_root.mkdir(parents=True, exist_ok=True)
            self.session_id = f"altitude_{utc_stamp()}"
            self.session_dir = self.log_root / self.session_id
            self.session_dir.mkdir(parents=True, exist_ok=False)

            config_start = self.msp_client.get_alt_est_config()
            manifest = {
                "schema": "betaflight-altitude-session-v1",
                "session_id": self.session_id,
                "created_utc": utc_stamp(),
                "poll_hz": self.poll_hz,
                "files": {
                    "telemetry": "telemetry.jsonl",
                    "events": "events.jsonl",
                    "config_start": "config_start.json",
                    "config_end": "config_end.json",
                },
            }
            self._write_json("manifest.json", manifest)
            self._write_json("config_start.json", config_start)
            self.write_event("start", {"poll_hz": self.poll_hz, "reason": reason})

            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="altitude-recorder", daemon=True)
            self._thread.start()
            return {"active": True, "session_id": self.session_id}

    def stop(self, reason: str = "manual") -> dict[str, Any]:
        with self._lock:
            session_id = self.session_id
            if not self.active():
                return {"active": False, "session_id": session_id}
            self._stop.set()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        with self._lock:
            if self.session_dir is not None:
                try:
                    self._write_json("config_end.json", self.msp_client.get_alt_est_config())
                    self.write_event("stop", {"reason": reason})
                except Exception as exc:  # noqa: BLE001 - event logging must not hide stop state.
                    self.write_event("stop_error", {"error": str(exc), "reason": reason})
            self._thread = None
            return {"active": False, "session_id": session_id}

    def _run(self) -> None:
        period = 1.0 / max(1.0, self.poll_hz)
        assert self.session_dir is not None
        telemetry_path = self.session_dir / "telemetry.jsonl"
        with open(telemetry_path, "a", encoding="utf-8") as fh:
            while not self._stop.is_set():
                started = time.monotonic()
                try:
                    status = self.msp_client.get_alt_est_status()
                    record = {"host_time_ns": time.time_ns(), "status": status}
                    fh.write(json.dumps(record, separators=(",", ":")) + "\n")
                    fh.flush()
                except Exception as exc:  # noqa: BLE001 - keep session alive and log the fault.
                    self.write_event("poll_error", {"error": str(exc)})
                elapsed = time.monotonic() - started
                self._stop.wait(max(0.0, period - elapsed))

    def _write_json(self, name: str, data: Any) -> None:
        assert self.session_dir is not None
        with open(self.session_dir / name, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2, sort_keys=True)
            fh.write("\n")

    def write_event(self, event: str, data: dict[str, Any]) -> None:
        if self.session_dir is None:
            return
        record = {"host_time_ns": time.time_ns(), "event": event, "data": data}
        with open(self.session_dir / "events.jsonl", "a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":")) + "\n")


class ArmStateWatcher:
    def __init__(self, msp_client: Any, recorder: Recorder, poll_hz: float) -> None:
        self.msp_client = msp_client
        self.recorder = recorder
        self.poll_hz = max(0.5, poll_hz)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_armed: bool | None = None
        self._last_status: dict[str, Any] | None = None
        self._last_error: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="altitude-arm-state-watcher", daemon=True)
        self._thread.start()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "running": self._thread is not None and self._thread.is_alive(),
                "last_armed": self._last_armed,
                "last_status": self._last_status,
                "last_error": self._last_error,
                "poll_hz": self.poll_hz,
            }

    def _run(self) -> None:
        period = 1.0 / self.poll_hz
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                status = self.msp_client.get_fc_status()
                armed = bool(status.get("armed"))
                with self._lock:
                    previous = self._last_armed
                    self._last_armed = armed
                    self._last_status = status
                    self._last_error = None
                if previous is not None and not previous and armed:
                    self.recorder.start("armed")
                    self.recorder.write_event("armed", {"fc_status": status})
                elif previous is not None and previous and not armed:
                    self.recorder.write_event("disarmed", {"fc_status": status})
                    self.recorder.stop("disarmed")
            except Exception as exc:  # noqa: BLE001 - keep watcher alive.
                with self._lock:
                    self._last_error = str(exc)
            elapsed = time.monotonic() - started
            self._stop.wait(max(0.0, period - elapsed))


class RecommendationStore:
    def __init__(self, path: Path) -> None:
        self.path = path

    def get(self) -> dict[str, Any]:
        if not self.path.exists():
            return {"exists": False, "recommendation": None}
        return {"exists": True, "recommendation": json.loads(self.path.read_text(encoding="utf-8"))}

    def put(self, recommendation: dict[str, Any]) -> dict[str, Any]:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(recommendation, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        tmp.replace(self.path)
        return self.get()

    def consume(self) -> dict[str, Any]:
        current = self.get()
        if self.path.exists():
            self.path.unlink()
        return current


def list_sessions(log_root: Path) -> list[dict[str, Any]]:
    if not log_root.exists():
        return []
    sessions = []
    for path in sorted(log_root.iterdir(), reverse=True):
        manifest_path = path / "manifest.json"
        if not manifest_path.exists():
            continue
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            manifest = {"session_id": path.name}
        manifest["path"] = str(path)
        duration_s = session_duration_s(path)
        if duration_s is not None:
            manifest["duration_s"] = round(duration_s, 3)
        sessions.append(manifest)
    return sessions


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Altitude Logger</title>
  <style>
    body { margin: 0; font-family: system-ui, sans-serif; background: #f6f7f9; color: #1d2430; }
    main { max-width: 980px; margin: 0 auto; padding: 24px; }
    section { margin: 0 0 16px; padding: 16px; background: #fff; border: 1px solid #d9dee7; border-radius: 8px; }
    button { height: 36px; padding: 0 14px; border: 1px solid #b9c2d0; border-radius: 6px; background: #fff; cursor: pointer; }
    button.primary { background: #244f9e; color: #fff; border-color: #244f9e; }
    pre { overflow: auto; background: #101723; color: #dbe7ff; padding: 12px; border-radius: 6px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
  </style>
</head>
<body>
<main>
  <h1>Altitude Logger</h1>
  <section>
    <div class="row">
      <button class="primary" onclick="startLog()">Start</button>
      <button onclick="stopLog()">Stop</button>
      <button onclick="refresh()">Refresh</button>
    </div>
    <p id="state"></p>
  </section>
  <section>
    <h2>Status</h2>
    <pre id="status">{}</pre>
  </section>
  <section>
    <h2>Sessions</h2>
    <pre id="sessions">[]</pre>
  </section>
</main>
<script>
async function api(path, options) {
  const res = await fetch(path, options || {});
  if (!res.ok) throw new Error(await res.text());
  return await res.json();
}
async function refresh() {
  document.getElementById('status').textContent = JSON.stringify(await api('/api/altitude/status'), null, 2);
  document.getElementById('sessions').textContent = JSON.stringify(await api('/api/altitude/sessions'), null, 2);
}
async function startLog() {
  document.getElementById('state').textContent = JSON.stringify(await api('/api/altitude/logging/start', {method: 'POST'}));
  await refresh();
}
async function stopLog() {
  document.getElementById('state').textContent = JSON.stringify(await api('/api/altitude/logging/stop', {method: 'POST'}));
  await refresh();
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
"""


def create_app(config_path: str | os.PathLike[str] | None = None, config_override: dict[str, Any] | None = None) -> Flask:
    config = load_config(config_path)
    if config_override:
        config.update(config_override)
    log_root = Path(config["log_root"]).resolve()
    recommendation_path = Path(config["recommendation_path"]).resolve()
    client = make_msp_client(config)
    recorder = Recorder(client, log_root, float(config.get("poll_hz", 50)))
    watcher = ArmStateWatcher(client, recorder, float(config.get("arm_poll_hz", 5)))
    recommendations = RecommendationStore(recommendation_path)
    if config.get("auto_recording", True):
        watcher.start()

    app = Flask(__name__)
    app.config["ALTHOLD_CONFIG"] = config
    app.config["ALTHOLD_MSP_CLIENT"] = client
    app.config["ALTHOLD_RECORDER"] = recorder
    app.config["ALTHOLD_ARM_WATCHER"] = watcher
    app.config["ALTHOLD_RECOMMENDATIONS"] = recommendations

    @app.get("/")
    def index() -> Response:
        return Response(INDEX_HTML, mimetype="text/html")

    @app.get("/healthz")
    def healthz() -> Any:
        watcher_status = watcher.status()
        return jsonify({
            "ok": True,
            "mode": config.get("mode"),
            "logging": recorder.active(),
            "recording": recorder.status(),
            "fc": watcher_status.get("last_status"),
            "fc_error": watcher_status.get("last_error"),
            "auto_recording": bool(config.get("auto_recording", True)),
        })

    @app.get("/api/fc/status")
    def fc_status() -> Any:
        watcher_status = watcher.status()
        try:
            status = client.get_fc_status()
            return jsonify({"ok": True, "status": status, "watcher": watcher_status})
        except Exception as exc:  # noqa: BLE001 - expose FC connection state to cockpit.
            return jsonify({"ok": False, "error": str(exc), "watcher": watcher_status}), 503

    @app.get("/api/recording/status")
    def recording_status() -> Any:
        return jsonify({"recording": recorder.status(), "watcher": watcher.status()})

    @app.get("/api/altitude/status")
    def altitude_status() -> Any:
        return jsonify(client.get_alt_est_status())

    @app.route("/api/altitude/config", methods=["GET", "POST"])
    def altitude_config() -> Any:
        if request.method == "GET":
            return jsonify(client.get_alt_est_config())
        payload = request.get_json(force=True, silent=False)
        updated = client.set_alt_est_config(payload)
        if recorder.session_dir is not None:
            recorder.write_event("config_update", {"config": updated})
        return jsonify(updated)

    @app.post("/api/altitude/config/save")
    def altitude_config_save() -> Any:
        status = client.get_fc_status()
        if status.get("armed"):
            return jsonify({"error": "cannot save EEPROM while armed", "fc": status}), 409
        return jsonify(client.save_config())

    @app.post("/api/altitude/fc/reboot")
    def altitude_fc_reboot() -> Any:
        status = client.get_fc_status()
        if status.get("armed"):
            return jsonify({"error": "cannot reboot while armed", "fc": status}), 409
        return jsonify(client.reboot())

    @app.post("/api/altitude/logging/start")
    def logging_start() -> Any:
        return jsonify(recorder.start("manual"))

    @app.post("/api/altitude/logging/stop")
    def logging_stop() -> Any:
        return jsonify(recorder.stop("manual"))

    @app.route("/api/altitude/sessions", methods=["GET", "POST"])
    def sessions() -> Any:
        if request.method == "POST":
            return jsonify(recorder.start())
        return jsonify(list_sessions(log_root))

    @app.get("/api/altitude/sessions/<session_id>")
    def session_manifest(session_id: str) -> Any:
        path = log_root / session_id / "manifest.json"
        if not path.exists():
            return jsonify({"error": "session not found"}), 404
        return jsonify(json.loads(path.read_text(encoding="utf-8")))

    @app.post("/api/altitude/sessions/<session_id>/stop")
    def session_stop(session_id: str) -> Any:
        if recorder.session_id != session_id:
            return jsonify({"error": "session is not active"}), 409
        return jsonify(recorder.stop("manual"))

    @app.get("/api/altitude/sessions/<session_id>/download")
    def session_download(session_id: str) -> Any:
        session_dir = log_root / session_id
        if not session_dir.exists():
            return jsonify({"error": "session not found"}), 404
        temp_dir = Path(tempfile.mkdtemp(prefix="althold_zip_"))
        zip_path = temp_dir / f"{session_id}.zip"
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for file_path in sorted(session_dir.iterdir()):
                if file_path.is_file():
                    archive.write(file_path, arcname=f"{session_id}/{file_path.name}")
        response = send_file(zip_path, as_attachment=True, download_name=f"{session_id}.zip")
        response.call_on_close(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        return response

    @app.delete("/api/altitude/sessions/<session_id>")
    def session_delete(session_id: str) -> Any:
        if recorder.session_id == session_id and recorder.active():
            return jsonify({"error": "cannot delete active session"}), 409
        session_dir = log_root / session_id
        if not session_dir.exists():
            return jsonify({"error": "session not found"}), 404
        shutil.rmtree(session_dir)
        return jsonify({"deleted": True, "session_id": session_id})

    @app.route("/api/recommendations/pending", methods=["GET", "POST"])
    def recommendations_pending() -> Any:
        if request.method == "GET":
            return jsonify(recommendations.get())
        payload = request.get_json(force=True, silent=False)
        if not isinstance(payload, dict):
            return jsonify({"error": "recommendation must be a JSON object"}), 400
        return jsonify(recommendations.put(payload))

    @app.post("/api/recommendations/pending/consume")
    def recommendations_pending_consume() -> Any:
        return jsonify(recommendations.consume())

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Altitude estimator Pi logger webapp")
    parser.add_argument("--config", default=os.environ.get("ALTHOLD_LOGGER_CONFIG"))
    parser.add_argument("--fake", action="store_true", help="force fake MSP mode")
    args = parser.parse_args()
    override = {"mode": "fake"} if args.fake else None
    app = create_app(args.config, override)
    cfg = app.config["ALTHOLD_CONFIG"]
    app.run(host=cfg.get("site_host", "0.0.0.0"), port=int(cfg.get("site_port", 8080)))


if __name__ == "__main__":
    main()
