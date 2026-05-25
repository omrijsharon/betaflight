# Altitude Pi Logger

This is the Pi Zero 2 W logger/webapp for altitude-estimator field work. It talks directly to Betaflight MSP2 over UART with `pyserial`; it does not use `yamspy`, `ArUco_Chaser`, or any OpenAI API key.

Run locally in fake-MSP mode:

```bash
python -m althold.pi_logger.app --fake
```

Session layout:

- `manifest.json`
- `telemetry.jsonl`
- `events.jsonl`
- `config_start.json`
- `config_end.json`

## Field Config

The repo contains templates only. Real Wi-Fi credentials must live on the Pi and should not be committed:

- Template: `althold/pi_logger/field_network.example.json`
- Pi path: `/etc/althold/field_network.json`
- Pi permissions: `sudo chmod 600 /etc/althold/field_network.json`
- Logger config path: `/etc/althold/logger.json`

The field network helper uses NetworkManager. It creates three profiles:

- `althold-hotspot`: phone hotspot client, tried first.
- `althold-home`: home Wi-Fi client, tried second.
- `althold-ap`: fallback Pi AP using `ipv4.method shared`.

Tailscale policy is `hotspot_only`: the helper runs `tailscale up` only after connecting to the phone hotspot. Home Wi-Fi Tailscale is an explicit manual test command, and AP mode forces Tailscale down.

## Pi Deployment From Windows

Copy the code:

```powershell
scp -r .\althold omrijsharon@omrijsharon.local:/home/omrijsharon/althold-upload
```

Install on the Pi:

```bash
sudo mkdir -p /opt/althold /etc/althold /opt/althold/logs/pi_sessions
sudo rm -rf /opt/althold/althold
sudo cp -a /home/omrijsharon/althold-upload /opt/althold/althold
sudo chown -R root:root /opt/althold/althold
sudo chmod -R a+rX /opt/althold/althold
sudo chown -R omrijsharon:omrijsharon /opt/althold/logs
sudo python3 -m venv /opt/althold/venv
sudo /opt/althold/venv/bin/pip install -r /opt/althold/althold/pi_logger/requirements.txt
sudo cp /opt/althold/althold/pi_logger/logger.example.json /etc/althold/logger.json
sudo cp /opt/althold/althold/pi_logger/field_network.example.json /etc/althold/field_network.json
sudo chmod 600 /etc/althold/field_network.json
sudo install -m 0644 /opt/althold/althold/pi_logger/templates/althold-pi-logger.service /etc/systemd/system/althold-pi-logger.service
sudo install -m 0644 /opt/althold/althold/pi_logger/templates/althold-field-network.service /etc/systemd/system/althold-field-network.service
sudo systemctl daemon-reload
sudo systemctl enable --now althold-pi-logger.service
```

Do not enable `althold-field-network.service` until `/etc/althold/field_network.json` contains real SSIDs/passwords. Starting it with placeholder values can switch the Pi into fallback AP mode and drop the current SSH session.

## Useful Pi Commands

```bash
python3 -m althold.pi_logger.field_network_manager --config /etc/althold/field_network.json configure
python3 -m althold.pi_logger.field_network_manager --config /etc/althold/field_network.json status
python3 -m althold.pi_logger.field_network_manager --config /etc/althold/field_network.json tailscale-home-test
python3 -m althold.pi_logger.field_network_manager --config /etc/althold/field_network.json tailscale-down
```

Install Tailscale on the Pi:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --hostname omrijsharon-pi --accept-routes=false
```

## Verification

Fake-mode logger checks:

```bash
curl -fsS http://127.0.0.1:8080/healthz
curl -fsS http://127.0.0.1:8080/api/altitude/status
```

Network checks after real credentials are installed:

- Home Wi-Fi/LAN: open `http://omrijsharon.local:8080`.
- AP fallback: connect to the configured AP and open `http://10.42.0.1:8080`.
- Hotspot/Tailscale: use `tailscale status`, `tailscale ping omrijsharon-pi`, and `http://<tailscale-ip>:8080/healthz`.

The logger remains in fake-MSP mode until the FC is flashed and UART/MSP2 is physically connected.
