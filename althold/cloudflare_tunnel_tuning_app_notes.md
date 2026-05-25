# Cloudflare Tunnel And Auth Notes For The Altitude Tuning App

Source reviewed: `C:\Users\tamipinhasi\Documents\repos\vscode-copilot-bridge`

Reviewed files:

- `docs/CLOUDFLARE_TUNNEL_SETUP.md`
- `docs/REMOTE_ACCESS_SETUP.md`
- `scripts/start-codex-remote-stack.ps1`
- `src/relay/server.ts`
- `src/relay/state.ts`
- `relay-client/operator.html`
- `README.md`

## Goal

Replicate the `vscode-copilot-bridge` remote-access pattern for the altitude tuning app so a phone in the field can open a public HTTPS URL, authenticate, and orchestrate the local laptop backend while the laptop talks privately to the Pi over Tailscale.

## Current Deployment Decisions

- Requested public hostname: `altitude_ekf_tuning.flying-agents.com`
- Recommended public hostname: `altitude-ekf-tuning.flying-agents.com`
- Reason: underscores are not suitable for a public HTTPS web hostname; use hyphens for DNS, TLS, and browser compatibility
- Cloudflare tunnel: create a new tunnel named `althold-tuning`
- Created tunnel ID: `8fd2f913-1f34-46c7-8fde-4ca9f57cebd3`
- Local tuning backend port: `8790`
- Pi hostname: keep the current Pi name; use `omrijsharon.local` where mDNS/LAN works and the Pi Tailscale name/IP where Tailscale is required
- Startup mode: manual launcher script, not a Windows service
- Public auth model: app auth only, same as the bridge repo, using app-level operator secret plus pairing/session cookies
- Cloudflare Access: not used for this implementation

The intended topology is:

```text
phone browser
  -> https://altitude-ekf-tuning.flying-agents.com
  -> Cloudflare Tunnel
  -> laptop tuning backend on localhost:8790
  -> Codex app-server on 127.0.0.1
  -> Pi over Tailscale
  -> FC over MSP2 UART
```

Important rule from the bridge repo: expose only the relay/tuning backend through Cloudflare. Do not expose the raw Codex app-server, the Pi backend, or the FC-facing MSP interface directly to the internet.

## Cloudflare Tunnel Pattern

The bridge setup uses a named Cloudflare tunnel and routes a DNS hostname to the local relay:

```powershell
cloudflared tunnel login
cloudflared tunnel create codex-remote-relay
cloudflared tunnel route dns codex-remote-relay codex.YOUR_DOMAIN
```

For this project, use the same pattern with altitude-specific names:

```powershell
cloudflared tunnel login
cloudflared tunnel create althold-tuning
cloudflared tunnel route dns althold-tuning altitude-ekf-tuning.flying-agents.com
```

The launcher should start the tunnel with:

```powershell
cloudflared tunnel run --url "http://localhost:$TuningPort" "$TunnelName"
```

The hostname used in the app must match the actual public origin. Pairing links, cookies, and WebSocket origin checks depend on the configured public base URL.

## Launcher Pattern

The bridge repo uses `scripts/start-codex-remote-stack.ps1` to start three local pieces in separate PowerShell windows:

- Codex app-server
- relay server
- Cloudflare tunnel

The future altitude tuning app should use a similar launcher, likely:

```text
althold/tuning_app/scripts/start-althold-tuning-stack.ps1
```

Recommended launcher parameters:

- `TunnelName`, default `althold-tuning`
- `PublicBaseUrl`, `https://altitude-ekf-tuning.flying-agents.com`
- `TuningPort`, default `8790`
- `CodexAppServerUrl`, default `ws://127.0.0.1:4500`
- `PiBaseUrl`, for example `http://omrijsharon-pi:8080` or the Pi Tailscale IP
- `SessionSecret`
- `OperatorSecret`
- `ForceRestart`

Behaviors to copy:

- Require `PublicBaseUrl`.
- Require a long `SessionSecret`.
- Require `OperatorSecret` whenever the public base URL is not localhost.
- Detect an existing healthy process on the required port and reuse it unless `ForceRestart` is set.
- Start long-running processes in separate PowerShell windows.
- Use encoded PowerShell commands for robust quoting.
- Check health endpoints after startup.
- Detect `cloudflared.exe`, including the common Windows path:

```text
C:\Program Files (x86)\cloudflared\cloudflared.exe
```

The bridge launcher resolves the Codex executable from the VS Code extension path. The tuning app can copy that logic if it needs to start `codex app-server` automatically.

## Runtime Configuration

The bridge uses environment variables such as:

- `CODEX_RELAY_HOST`
- `CODEX_RELAY_PORT`
- `CODEX_RELAY_BASE_URL`
- `CODEX_APP_SERVER_URL`
- `CODEX_RELAY_SESSION_SECRET`
- `CODEX_RELAY_OPERATOR_SECRET`
- `CODEX_RELAY_PAIRING_TTL_MS`
- `CODEX_RELAY_SESSION_TTL_MS`
- `CODEX_RELAY_SESSION_REQ_PER_MIN`
- `CODEX_RELAY_PAIRING_REQ_PER_HOUR`

For the altitude tuning app, use project-specific names:

- `ALTHOLD_TUNING_HOST`
- `ALTHOLD_TUNING_PORT`
- `ALTHOLD_TUNING_BASE_URL`
- `ALTHOLD_CODEX_APP_SERVER_URL`
- `ALTHOLD_PI_BASE_URL`
- `ALTHOLD_TUNING_SESSION_SECRET`
- `ALTHOLD_TUNING_OPERATOR_SECRET`
- `ALTHOLD_TUNING_PAIRING_TTL_MS`
- `ALTHOLD_TUNING_SESSION_TTL_MS`
- `ALTHOLD_TUNING_SESSION_REQ_PER_MIN`
- `ALTHOLD_TUNING_PAIRING_REQ_PER_HOUR`

Do not commit real domains, tunnel IDs, session secrets, operator secrets, or Wi-Fi credentials. Keep real deployment config in ignored local files or environment variables.

## Auth And Pairing Model To Reuse

The bridge has two layers of browser access:

1. Operator access
2. Paired field-device session access

Operator access:

- `/operator` serves an operator page.
- If `OperatorSecret` is configured, the operator must log in with it.
- Login sets an operator cookie.
- If `OperatorSecret` is not configured, operator endpoints are local-only.
- Public deployment without an operator secret is rejected at startup.

Pairing flow:

- Operator opens the operator page.
- Operator creates a one-time pairing token.
- Backend returns a pair URL and QR SVG.
- Phone opens or scans the pair URL.
- `/pair?token=...` consumes the token once.
- Backend creates a session and sets a signed session cookie.
- Phone is redirected to the main client UI.

Session model:

- Session IDs are signed with `sha256(sessionId:secret)`.
- Session cookies use `sessionId.signature`.
- Sessions include metadata such as IP, user agent, operating system, device label, creation time, last seen time, and expiration.
- Operator can revoke one session or all sessions.

Cookie attributes copied from the bridge:

- `HttpOnly`
- `Path=/`
- `SameSite=Lax`
- `Secure` only when the public base URL is HTTPS

Request protections to copy:

- Require same-origin `Origin` or `Referer` for POST endpoints.
- Allow local requests for local-only development.
- Reject unauthenticated WebSocket connections.
- Rate-limit pairing creation.
- Rate-limit session API/WebSocket activity.
- Periodically clean expired pairings and sessions.

## Tuning App Endpoint Shape

Generic remote-access endpoints to copy/adapt:

- `GET /`
- `GET /operator`
- `POST /api/operator/login`
- `POST /api/operator/logout`
- `POST /api/pairing/start`
- `GET /pair?token=...`
- `GET /api/session`
- `POST /api/logout`
- `POST /api/session/revoke-current`
- `GET /api/operator/state`
- `POST /api/operator/session/revoke`
- `POST /api/operator/session/revoke-all`
- `GET /ws`

Altitude-specific endpoints to add behind the same auth layer:

- `GET /api/pi/status`
- `GET /api/fc/status`
- `GET /api/altitude/status`
- `GET /api/altitude/config`
- `POST /api/altitude/config`
- `POST /api/altitude/config/apply`
- `GET /api/sessions`
- `GET /api/sessions/<id>`
- `GET /api/sessions/<id>/download`
- `POST /api/sessions/<id>/download-to-laptop`
- `POST /api/sessions/<id>/analyze`
- `GET /api/recommendations/<id>`

The laptop backend should own downloads, analysis, Codex interaction, and final parameter recommendations. The Pi backend should own FC-facing MSP reads/writes and logger control.

## Security Rules For This Project

- Never expose the Pi backend directly through Cloudflare.
- Never expose the raw Codex app-server through Cloudflare.
- The public Cloudflare hostname should terminate at the laptop tuning backend only.
- The laptop backend talks to the Pi over Tailscale.
- The Pi talks to the FC over UART/MSP2.
- No OpenAI API key is used by the Pi.
- Real secrets remain outside git.
- Config writes to the FC should be blocked while armed unless a future explicit override is added.
- The phone UI should show whether the system is using home Wi-Fi, hotspot, AP fallback, Tailscale, and FC MSP connectivity before allowing field actions.

Cloudflare Access is not part of the selected implementation. The public hostname is protected by the app's operator-secret login, short-lived pairing tokens, signed session cookies, same-origin checks, and session revocation.

## Verification Checklist

Local-only:

- Start tuning backend on localhost.
- Confirm `/health` reports backend status.
- Confirm operator login works on localhost.
- Confirm pairing creates a URL and QR.
- Confirm paired browser gets a valid session.
- Confirm unauthenticated WebSocket requests are rejected.

Cloudflare tunnel:

- Confirm DNS points to the named tunnel.
- Open `https://altitude.YOUR_DOMAIN/operator`.
- Confirm operator login works over HTTPS.
- Confirm pairing URL uses the public HTTPS origin.
- Pair the phone using the QR/link.
- Confirm the phone UI connects over WebSocket.

Backend integrations:

- Confirm laptop can reach Pi over Tailscale.
- Confirm laptop backend can query Pi logger status.
- Confirm Pi can report FC MSP status when the FC is connected later.
- Confirm log sessions can be downloaded from Pi to laptop.
- Confirm analysis runs on the laptop.
- Confirm recommendations are shown in the phone UI.
- Confirm FC config writes are blocked while armed and allowed while disarmed.

## Implementation Recommendation

Build the tuning app in three steps:

1. Local laptop tuning backend with auth, operator login, pairing, sessions, and a local-only UI.
2. Cloudflare tunnel launcher copied from the bridge pattern and adapted to `ALTHOLD_TUNING_*` environment variables.
3. Pi/Tailscale/Codex integrations behind the authenticated laptop backend.

This allows the phone workflow to work early through Cloudflare, even before the Codex app-server integration is complete.
