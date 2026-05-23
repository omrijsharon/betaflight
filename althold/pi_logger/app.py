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


def load_config(path: str | os.PathLike[str] | None) -> dict[str, Any]:
    defaults = {
        "mode": "fake",
        "serial_port": "/dev/ttyAMA0",
        "baudrate": 115200,
        "poll_hz": 50,
        "log_root": "althold/logs/pi_sessions",
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

    def start(self) -> dict[str, Any]:
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
            self._write_event("start", {"poll_hz": self.poll_hz})

            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="altitude-recorder", daemon=True)
            self._thread.start()
            return {"active": True, "session_id": self.session_id}

    def stop(self) -> dict[str, Any]:
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
                    self._write_event("stop", {})
                except Exception as exc:  # noqa: BLE001 - event logging must not hide stop state.
                    self._write_event("stop_error", {"error": str(exc)})
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
                    self._write_event("poll_error", {"error": str(exc)})
                elapsed = time.monotonic() - started
                self._stop.wait(max(0.0, period - elapsed))

    def _write_json(self, name: str, data: Any) -> None:
        assert self.session_dir is not None
        with open(self.session_dir / name, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2, sort_keys=True)
            fh.write("\n")

    def _write_event(self, event: str, data: dict[str, Any]) -> None:
        assert self.session_dir is not None
        record = {"host_time_ns": time.time_ns(), "event": event, "data": data}
        with open(self.session_dir / "events.jsonl", "a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, separators=(",", ":")) + "\n")


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
    client = make_msp_client(config)
    recorder = Recorder(client, log_root, float(config.get("poll_hz", 50)))

    app = Flask(__name__)
    app.config["ALTHOLD_CONFIG"] = config
    app.config["ALTHOLD_RECORDER"] = recorder

    @app.get("/")
    def index() -> Response:
        return Response(INDEX_HTML, mimetype="text/html")

    @app.get("/healthz")
    def healthz() -> Any:
        return jsonify({"ok": True, "mode": config.get("mode"), "logging": recorder.active()})

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
            recorder._write_event("config_update", {"config": updated})
        return jsonify(updated)

    @app.post("/api/altitude/logging/start")
    def logging_start() -> Any:
        return jsonify(recorder.start())

    @app.post("/api/altitude/logging/stop")
    def logging_stop() -> Any:
        return jsonify(recorder.stop())

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
        return jsonify(recorder.stop())

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
