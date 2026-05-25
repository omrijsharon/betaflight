# Altitude Tuning Cockpit

Laptop-hosted tuning cockpit for the Betaflight altitude estimator.

## Local Setup

```powershell
cd althold\tuning_app
npm.cmd install
npm.cmd run build
npm.cmd start
```

Default local URL:

```text
http://127.0.0.1:8790/operator
```

The public tunnel URL is:

```text
https://altitude-ekf-tuning.flying-agents.com/operator
```

## Manual Field Startup

Set secrets in the current PowerShell session or pass them as parameters:

```powershell
$env:ALTHOLD_TUNING_SESSION_SECRET = "long-random-session-secret"
$env:ALTHOLD_TUNING_OPERATOR_SECRET = "separate-operator-secret"
powershell -ExecutionPolicy Bypass -File .\althold\tuning_app\scripts\start-althold-tuning-stack.ps1
```

The launcher starts:

- Codex app-server on `ws://127.0.0.1:4500`
- tuning backend on `localhost:8790`
- Cloudflare tunnel `althold-tuning`

The Pi backend remains private on LAN/Tailscale.
