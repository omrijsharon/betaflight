from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_CONFIG = {
    "pi_host": "althold-pi",
    "pi_user": "pi",
    "pi_app_dir": "/opt/althold",
    "remote_log_dir": "/opt/althold/logs/pi_sessions",
    "local_log_dir": "althold/logs/downloaded",
    "wifi_hotspot_profile": "phone-hotspot",
    "wifi_home_profile": "home-wifi",
}


def load_config(path: str | None) -> dict[str, Any]:
    config = dict(DEFAULT_CONFIG)
    if path and Path(path).exists():
        with open(path, "r", encoding="utf-8") as fh:
            config.update(json.load(fh))
    return config


def run_or_print(command: list[str], execute: bool) -> int:
    print(" ".join(command))
    if not execute:
        return 0
    return subprocess.call(command)


def ssh_target(config: dict[str, Any]) -> str:
    return f"{config['pi_user']}@{config['pi_host']}"


def latest_bundle(local_log_dir: Path) -> Path | None:
    candidates = []
    for pattern in ("*.zip", "*/manifest.json"):
        for path in local_log_dir.glob(pattern):
            candidates.append(path.parent if path.name == "manifest.json" else path)
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def archive_session(source: Path, archive_dir: Path) -> Path:
    archive_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if source.is_file() and source.suffix == ".zip":
        dest = archive_dir / f"{source.stem}_{stamp}.zip"
        shutil.copy2(source, dest)
        return dest

    dest = archive_dir / f"{source.name}_{stamp}.zip"
    with zipfile.ZipFile(dest, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for file_path in sorted(source.rglob("*")):
            if file_path.is_file():
                archive.write(file_path, arcname=str(file_path.relative_to(source.parent)))
    return dest


def cmd_commands(config: dict[str, Any]) -> int:
    target = ssh_target(config)
    print("Connect laptop to hotspot:")
    print(f"  netsh wlan connect name=\"{config['wifi_hotspot_profile']}\"")
    print("Start remote app later over SSH:")
    print(f"  ssh {target} 'cd {config['pi_app_dir']} && ./venv/bin/python -m althold.pi_logger.app'")
    print("Pull logs later over SCP:")
    print(f"  scp -r {target}:{config['remote_log_dir']} {config['local_log_dir']}")
    return 0


def cmd_wifi(config: dict[str, Any], execute: bool, profile: str) -> int:
    return run_or_print(["netsh", "wlan", "connect", f"name={profile}"], execute)


def cmd_start(config: dict[str, Any], execute: bool) -> int:
    target = ssh_target(config)
    command = f"cd {config['pi_app_dir']} && ./venv/bin/python -m althold.pi_logger.app"
    return run_or_print(["ssh", target, command], execute)


def cmd_pull(config: dict[str, Any], execute: bool) -> int:
    target = ssh_target(config)
    local_dir = Path(config["local_log_dir"])
    local_dir.mkdir(parents=True, exist_ok=True)
    return run_or_print(["scp", "-r", f"{target}:{config['remote_log_dir']}", str(local_dir)], execute)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Local helper for altitude field logging")
    parser.add_argument("--config", default=None)
    parser.add_argument("--execute", action="store_true", help="run generated commands instead of printing them")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("commands")
    wifi = sub.add_parser("connect-wifi")
    wifi.add_argument("--profile", default=None)
    sub.add_parser("start-webapp")
    sub.add_parser("pull-logs")
    sub.add_parser("latest")
    archive = sub.add_parser("archive")
    archive.add_argument("source", nargs="?")
    archive.add_argument("--archive-dir", default="althold/logs/archives")

    args = parser.parse_args(argv)
    config = load_config(args.config)

    if args.command == "commands":
        return cmd_commands(config)
    if args.command == "connect-wifi":
        return cmd_wifi(config, args.execute, args.profile or config["wifi_hotspot_profile"])
    if args.command == "start-webapp":
        return cmd_start(config, args.execute)
    if args.command == "pull-logs":
        return cmd_pull(config, args.execute)
    if args.command == "latest":
        latest = latest_bundle(Path(config["local_log_dir"]))
        print(latest if latest else "no local bundles found")
        return 0 if latest else 1
    if args.command == "archive":
        source = Path(args.source) if args.source else latest_bundle(Path(config["local_log_dir"]))
        if not source:
            print("no source bundle found", file=sys.stderr)
            return 1
        print(archive_session(source, Path(args.archive_dir)))
        return 0

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
