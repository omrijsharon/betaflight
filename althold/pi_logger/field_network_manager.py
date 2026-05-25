from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

DEFAULT_CONFIG_PATH = Path("/etc/althold/field_network.json")
DEFAULT_LOGGER_CONFIG_PATH = Path("/etc/althold/logger.json")
HOTSPOT_PROFILE = "althold-hotspot"
HOME_PROFILE = "althold-home"
AP_PROFILE = "althold-ap"


class CommandError(RuntimeError):
    pass


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def run(command: list[str], timeout: int = 30, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, text=True, capture_output=True, timeout=timeout)
    if check and result.returncode != 0:
        raise CommandError(f"{' '.join(command)} failed: {result.stderr.strip() or result.stdout.strip()}")
    return result


def nmcli(*args: str, timeout: int = 30, check: bool = True) -> subprocess.CompletedProcess[str]:
    return run(["nmcli", *args], timeout=timeout, check=check)


def connection_exists(name: str) -> bool:
    result = nmcli("-t", "-f", "NAME", "connection", "show", check=False)
    return name in result.stdout.splitlines()


def connection_type(name: str) -> str | None:
    result = nmcli("-g", "connection.type", "connection", "show", name, check=False)
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def delete_if_wrong_type(name: str, expected_type: str) -> None:
    current = connection_type(name)
    if current and current != expected_type:
        nmcli("connection", "delete", name, check=False)


def ensure_wifi_client_profile(profile: str, iface: str, network: dict[str, Any]) -> None:
    ssid = str(network["ssid"])
    password = str(network["password"])
    priority = str(int(network.get("priority", 0)))
    delete_if_wrong_type(profile, "802-11-wireless")
    if not connection_exists(profile):
        nmcli("connection", "add", "type", "wifi", "ifname", iface, "con-name", profile, "ssid", ssid)
    nmcli(
        "connection",
        "modify",
        profile,
        "connection.autoconnect",
        "yes",
        "connection.autoconnect-priority",
        priority,
        "802-11-wireless.ssid",
        ssid,
        "802-11-wireless.mode",
        "infrastructure",
        "wifi-sec.key-mgmt",
        "wpa-psk",
        "wifi-sec.psk",
        password,
        "ipv4.method",
        "auto",
        "ipv6.method",
        "auto",
    )


def ensure_ap_profile(iface: str, ap: dict[str, Any]) -> None:
    ssid = str(ap["ssid"])
    password = str(ap["password"])
    if len(password) < 8:
        raise ValueError("fallback_ap.password must be at least 8 characters for WPA-PSK")
    ip_address = str(ap.get("ip_address", "10.42.0.1/24"))
    channel = str(int(ap.get("channel", 6)))
    delete_if_wrong_type(AP_PROFILE, "802-11-wireless")
    if not connection_exists(AP_PROFILE):
        nmcli("connection", "add", "type", "wifi", "ifname", iface, "con-name", AP_PROFILE, "ssid", ssid)
    nmcli(
        "connection",
        "modify",
        AP_PROFILE,
        "connection.autoconnect",
        "no",
        "802-11-wireless.ssid",
        ssid,
        "802-11-wireless.mode",
        "ap",
        "802-11-wireless.band",
        "bg",
        "802-11-wireless.channel",
        channel,
        "wifi-sec.key-mgmt",
        "wpa-psk",
        "wifi-sec.psk",
        password,
        "ipv4.method",
        "shared",
        "ipv4.addresses",
        ip_address,
        "ipv6.method",
        "ignore",
    )


def configure_profiles(config: dict[str, Any]) -> None:
    wifi = config["wifi"]
    iface = str(wifi.get("interface", "wlan0"))
    run(["raspi-config", "nonint", "do_wifi_country", str(wifi.get("country", "IL"))], check=False)
    ensure_wifi_client_profile(HOTSPOT_PROFILE, iface, wifi["hotspot"])
    ensure_wifi_client_profile(HOME_PROFILE, iface, wifi["home"])
    ensure_ap_profile(iface, wifi["fallback_ap"])


def active_ssid() -> str | None:
    result = nmcli("-t", "-f", "ACTIVE,SSID", "device", "wifi", check=False)
    for line in result.stdout.splitlines():
        active, _, ssid = line.partition(":")
        if active == "yes":
            return ssid
    return None


def active_connection(iface: str = "wlan0") -> str | None:
    result = nmcli("-t", "-f", "NAME,DEVICE", "connection", "show", "--active", check=False)
    for line in result.stdout.splitlines():
        name, _, device = line.partition(":")
        if device == iface:
            return name
    return None


def bring_up(profile: str, iface: str, timeout: int = 45) -> bool:
    result = nmcli("connection", "up", profile, "ifname", iface, timeout=timeout, check=False)
    return result.returncode == 0


def tailscale_available() -> bool:
    return shutil.which("tailscale") is not None


def tailscale_up(config: dict[str, Any]) -> bool:
    if not tailscale_available():
        return False
    ts = config.get("tailscale", {})
    accept_routes = "true" if ts.get("accept_routes", False) else "false"
    command = [
        "tailscale",
        "up",
        f"--hostname={ts.get('hostname', 'omrijsharon-pi')}",
        f"--accept-routes={accept_routes}",
    ]
    if ts.get("ssh", False):
        command.append("--ssh")
    return run(command, timeout=90, check=False).returncode == 0


def tailscale_down() -> bool:
    if not tailscale_available():
        return False
    return run(["tailscale", "down"], timeout=30, check=False).returncode == 0


def select_network(config: dict[str, Any]) -> str:
    wifi = config["wifi"]
    iface = str(wifi.get("interface", "wlan0"))
    nmcli("device", "wifi", "rescan", "ifname", iface, timeout=20, check=False)

    if bring_up(HOTSPOT_PROFILE, iface):
        if config.get("tailscale", {}).get("auto_up_policy", "hotspot_only") == "hotspot_only":
            tailscale_up(config)
        return "hotspot"

    if bring_up(HOME_PROFILE, iface):
        tailscale_down()
        return "home"

    tailscale_down()
    if bring_up(AP_PROFILE, iface):
        return "fallback_ap"

    raise CommandError("failed to connect hotspot/home and failed to start fallback AP")


def write_logger_config(config: dict[str, Any], path: Path = DEFAULT_LOGGER_CONFIG_PATH) -> None:
    logger = config.get("logger", {})
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(logger, fh, indent=2, sort_keys=True)
        fh.write("\n")
    os.replace(tmp, path)
    os.chmod(path, 0o644)


def status(config: dict[str, Any]) -> dict[str, Any]:
    iface = str(config.get("wifi", {}).get("interface", "wlan0"))
    ts_status = "missing"
    if tailscale_available():
        result = run(["tailscale", "status", "--json"], timeout=15, check=False)
        ts_status = "down"
        if result.returncode == 0:
            try:
                backend_state = json.loads(result.stdout).get("BackendState", "Unknown")
            except json.JSONDecodeError:
                backend_state = "Unknown"
            if backend_state == "Running":
                ts_status = "up"
            elif backend_state == "NeedsLogin":
                ts_status = "needs_login"
            elif backend_state in {"Stopped", "NoState"}:
                ts_status = "down"
            else:
                ts_status = str(backend_state).lower()
    return {
        "active_ssid": active_ssid(),
        "active_connection": active_connection(iface),
        "tailscale": ts_status,
        "auto_up_policy": config.get("tailscale", {}).get("auto_up_policy", "hotspot_only"),
    }


def daemon(config: dict[str, Any], interval_s: int) -> None:
    configure_profiles(config)
    write_logger_config(config)
    while True:
        try:
            mode = select_network(config)
            print(json.dumps({"mode": mode, **status(config)}), flush=True)
        except Exception as exc:  # noqa: BLE001 - daemon must keep retrying in the field.
            print(json.dumps({"error": str(exc)}), flush=True)
        time.sleep(interval_s)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Altitude logger NetworkManager/Tailscale helper")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("configure")
    sub.add_parser("run-once")
    daemon_parser = sub.add_parser("daemon")
    daemon_parser.add_argument("--interval-s", type=int, default=60)
    sub.add_parser("write-logger-config")
    sub.add_parser("status")
    sub.add_parser("tailscale-home-test")
    sub.add_parser("tailscale-down")

    args = parser.parse_args(argv)
    config = load_config(args.config)

    if args.command == "configure":
        configure_profiles(config)
        write_logger_config(config)
        return 0
    if args.command == "run-once":
        configure_profiles(config)
        write_logger_config(config)
        print(select_network(config))
        return 0
    if args.command == "daemon":
        daemon(config, args.interval_s)
        return 0
    if args.command == "write-logger-config":
        write_logger_config(config)
        return 0
    if args.command == "status":
        print(json.dumps(status(config), indent=2, sort_keys=True))
        return 0
    if args.command == "tailscale-home-test":
        if not config.get("tailscale", {}).get("allow_manual_home_wifi_test", False):
            print("manual home Wi-Fi Tailscale test is disabled", file=sys.stderr)
            return 2
        return 0 if tailscale_up(config) else 1
    if args.command == "tailscale-down":
        return 0 if tailscale_down() else 1

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
