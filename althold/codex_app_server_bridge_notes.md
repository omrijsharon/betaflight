# Codex App-Server Notes From `vscode-copilot-bridge`

Reviewed: 2026-05-24

Source repository:

- `C:\Users\tamipinhasi\Documents\repos\vscode-copilot-bridge`

Purpose of this note:

- Capture the app-server knowledge from `vscode-copilot-bridge`.
- Keep it available inside the Betaflight `althold/` work area.
- Decide how this should influence the altitude-tuning webapp architecture.

## High-Level Architecture

The `vscode-copilot-bridge` repo implements a **Codex remote relay** around `codex app-server`.

The important architecture is:

```text
browser UI
  -> authenticated relay HTTP/WebSocket API
  -> local codex app-server on 127.0.0.1
  -> Codex thread storage/history
```

The key rule repeated across the repo is:

- Do not expose raw `codex app-server` directly.
- Keep `codex app-server` bound to localhost.
- Expose only a controlled relay layer when remote/browser access is needed.
- Treat Codex `threadId` as the source of truth for conversation history.

For our altitude work, this means:

- The Pi logger should stay a data-collection appliance.
- Codex app-server should run on the laptop/local computer, not on the Pi.
- A future altitude tuning UI can talk to a local relay/app-server on the laptop to ask Codex to analyze downloaded logs.
- We still do not need an OpenAI API key in the Pi logger or analysis scripts.

## Main Files In The Bridge Repo

- `README.md`
  - Project overview, runtime config, local URLs, current status.
- `codex-app-server_plan.md`
  - Implementation plan and design decisions.
- `src/relay/appServerClient.ts`
  - Persistent WebSocket JSON-RPC client for `codex app-server`.
- `src/relay/server.ts`
  - Main authenticated relay server.
- `src/relay/config.ts`
  - Environment-based relay/app-server configuration.
- `src/relay/state.ts`
  - Pairing token and signed session-cookie stores.
- `src/relay/types.ts`
  - Relay protocol types.
- `relay-client/index.html`
  - Phone-first remote chat UI.
- `relay-client/app.js`
  - Browser WebSocket client for the relay.
- `relay-client/operator.html`
  - Operator/pairing/session-management UI.
- `examples/codex-app-server-chat.html`
  - Direct app-server reference page.
- `examples/app-server-thread-list.mjs`
  - Minimal JSON-RPC thread listing/reading example.
- `examples/app-server-thread-probe.mjs`
  - Minimal JSON-RPC thread resume + turn-start example.
- `scripts/start-codex-relay.ps1`
  - Starts local app-server plus relay.
- `scripts/start-codex-remote-stack.ps1`
  - Starts app-server, relay, and a Cloudflare tunnel.

## Starting App-Server

The bridge scripts locate the Codex executable under the VS Code extension directory:

```text
%USERPROFILE%\.vscode\extensions\openai.chatgpt-*-win32-x64\bin\windows-x86_64\codex.exe
```

Then they start app-server like this:

```powershell
codex.exe app-server --listen ws://127.0.0.1:4500
```

The relay defaults to:

```text
CODEX_APP_SERVER_URL = ws://127.0.0.1:4500
CODEX_RELAY_HOST = 127.0.0.1
CODEX_RELAY_PORT = 8787
CODEX_RELAY_BASE_URL = http://127.0.0.1:8787
```

The repo also has a remote stack script that can expose the relay through Cloudflare Tunnel, but its own docs repeatedly say to expose only the relay and never raw app-server.

## App-Server JSON-RPC Protocol Pattern

The direct examples and `AppServerClient` use WebSocket JSON-RPC 2.0.

Every app-server client must first call:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "initialize",
  "params": {
    "clientInfo": {
      "name": "client-name",
      "title": "Client Title",
      "version": "0.1.0"
    },
    "capabilities": {
      "experimentalApi": true
    }
  }
}
```

Then it sends the initialized notification:

```json
{
  "jsonrpc": "2.0",
  "method": "notifications/initialized",
  "params": {}
}
```

The bridge repo uses these app-server methods:

- `thread/list`
  - Lists known Codex threads.
- `thread/read`
  - Reads a thread.
  - Supports `includeTurns: true` when transcript content is needed.
- `thread/resume`
  - Resumes an existing thread before continuing it.
- `thread/start`
  - Starts a new thread.
  - Can include runtime settings such as `cwd`, `model`, `approvalPolicy`, `effort`, `collaborationMode`, `serviceTier`, and `sandboxPolicy`.
- `turn/start`
  - Sends a user turn to a thread.
  - Uses `threadId` and an `input` array.
- `collaborationMode/list`
  - Optional helper used by the relay to populate UI options.

The minimal text turn shape used by the examples is:

```json
{
  "threadId": "THREAD_ID",
  "input": [
    {
      "type": "text",
      "text": "User prompt text"
    }
  ]
}
```

The relay also supports richer input conversion:

- Browser file uploads are staged by the relay.
- Text-like attachments are turned into extra `{ "type": "text" }` inputs.
- Local images are turned into `{ "type": "localImage", "path": "..." }`.
- Remote image URLs are passed as `{ "type": "image", "url": "..." }`.

For altitude logs, the simplest first integration should send a text prompt with paths to local downloaded bundles and summaries. If we need attachment upload later, we can copy the bridge repo's staging pattern.

## Streaming Notifications

The app-server emits notifications without JSON-RPC ids.

The bridge handles these notification methods:

- `turn/started`
  - Start a streamed assistant response in the UI.
- `item/agentMessage/delta`
  - Append streamed assistant text.
  - Delta can be found as `params.delta`, `params.textDelta`, or `params.text`.
- `turn/plan/updated`
  - Stream/update a plan/checklist.
- `item/plan/delta`
  - Alternate plan streaming event.
- `item/completed`
  - Direct app-server reference page uses this as a fallback for completed message content.
- `turn/completed`
  - Finish the current assistant response.
- `turn/failed`
  - Mark the streamed response as failed.
- `turn/error`
  - Mark the streamed response as failed.
- `thread/updated`
  - Direct app-server reference uses this to update the current `threadId`.

The relay maps app-server notifications into its own browser-facing events:

- `session`
- `threads`
- `threadLoaded`
- `assistantDelta`
- `assistantMessage`
- `planUpdated`
- `turnStarted`
- `turnCompleted`
- `error`

## Relay Behavior

The relay is implemented in `src/relay/server.ts`.

It serves:

- `/`
  - Remote chat client.
- `/app.js`
  - Remote client JavaScript.
- `/operator`
  - Operator UI for pairing/session management.
- `/health`
  - Relay health plus app-server connectivity status.
- `/api/operator/auth-state`
  - Operator auth status.
- `/api/operator/login`
  - Operator login when `CODEX_RELAY_OPERATOR_SECRET` is configured.
- `/api/pairing/start`
  - Creates a short-lived pairing token and QR SVG.
- `/pair?token=...`
  - Consumes a pairing token and creates a signed remote session cookie.
- `/api/session`
  - Current browser session status.
- `/api/options`
  - Model/runtime option metadata for the UI.
- `/api/attachments/upload`
  - Stages text/image attachments for a session.
- `/api/thread/image`
  - Serves images referenced by thread output after validating path access.
- `/api/logout`
  - Logs out current session.
- `/api/session/revoke-current`
  - Revokes current session.
- `/api/operator/state`
  - Operator summary of sessions/events/alerts.
- `/api/operator/session`
  - Operator view of one session.
- `/api/operator/pairings`
  - Pairing state.
- `/api/operator/session/revoke`
  - Revoke one session.
- `/api/operator/session/revoke-all`
  - Revoke every session.
- `/ws`
  - Browser WebSocket for relay client messages.

The browser WebSocket messages accepted by the relay include:

- `refreshThreads`
- `loadThread`
- `startThread`
- `sendPrompt`

The relay validates session cookies before accepting WebSocket connections.

## Security Model

Important patterns from the bridge repo:

- `codex app-server` stays on `127.0.0.1`.
- Public/remote browser access goes through the relay only.
- Pairing tokens are short-lived and one-time use.
- Remote sessions are signed cookies.
- Sessions expire and can be revoked.
- Operator APIs are local-only unless an explicit operator secret is configured.
- If `CODEX_RELAY_BASE_URL` is not localhost and no operator secret is set, startup fails.
- Same-origin checks are used for POST endpoints.
- Rate limiting exists for pairing and session/WebSocket activity.
- Logs avoid storing sensitive conversation content by default.
- Real secrets, tunnel names, local diagnostics, and deployment-specific notes should not be committed.

For altitude tuning, this matters because a Codex-enabled tuning UI could potentially trigger actions and read logs. It should be laptop-local by default, and any remote access should use a relay/session model instead of exposing Codex app-server or local files directly.

## Runtime Controls Used By Thread Start/Turn Start

The bridge exposes these controls in its UI and forwards them best-effort to app-server:

- `cwd`
- `model`
- `approvalPolicy`
- `effort`
- `collaborationMode`
- `serviceTier`
- `sandboxPolicy`

For the altitude-tuning workflow, the important field is `cwd`: a new Codex thread should be started with `cwd` set to this Betaflight repository root so Codex can read `althold/logs/...`, analysis scripts, and firmware files.

Suggested altitude-tuning thread defaults:

```text
cwd = C:\Users\tamipinhasi\Documents\repos\betaflight
approvalPolicy = on-request or never, depending on desired automation
sandboxPolicy = workspaceWrite or dangerFullAccess, depending on whether local commands are needed
model = user-selected
```

## How This Applies To The Altitude Webapp

Current altitude tooling:

```text
Pi Zero
  -> Flask logger webapp
  -> MSP2 status/config over UART
  -> session bundle download

Laptop
  -> local repo
  -> althold/analysis scripts
  -> Codex in this repo reads logs and recommends tuning
```

Recommended future Codex app-server architecture:

```text
Pi Zero logger webapp
  -> collect logs and expose downloads only

Laptop tuning webapp / relay
  -> downloads or reads local log bundles
  -> runs/links analysis scripts
  -> starts/resumes a Codex thread through app-server
  -> asks Codex to inspect summary + raw files
  -> displays streamed tuning recommendation
```

Do not run Codex app-server on the Pi. The Pi should not need Codex credentials, an OpenAI API key, or access to local repo internals.

The first useful integration should be simple:

1. Keep the Pi logger as-is.
2. Pull/download a session bundle to `althold/logs/downloaded`.
3. Run `althold/analysis/analyze_altitude_log.py`.
4. Start or resume a local Codex app-server thread with `cwd` set to the Betaflight repo.
5. Send a prompt that includes:
   - path to the session bundle
   - path to `summary.json`
   - path to `summary.csv`
   - current `alt_est_*` config values
   - requested output format for tuning recommendations
6. Stream the Codex response into the tuning UI.

This keeps Codex analysis local to the laptop and avoids pushing large logs or secrets through the Pi.

## Practical Reuse Candidates

Useful pieces to copy/adapt later:

- `src/relay/appServerClient.ts`
  - JSON-RPC WebSocket client with initialize, notifications, pending calls, retry, timeout.
- `examples/app-server-thread-list.mjs`
  - Minimal thread list/read example.
- `examples/app-server-thread-probe.mjs`
  - Minimal resume + turn/start example.
- `relay-client/app.js`
  - Browser event handling for streamed assistant replies.
- `src/relay/server.ts`
  - Relay patterns for authenticated browser WebSocket, session cookies, pairing, and app-server event forwarding.
- `scripts/start-codex-relay.ps1`
  - Local launcher pattern for starting app-server and relay.

For our altitude project, the first implementation should probably be smaller than the bridge relay:

- a local-only laptop service under `althold/`
- direct `ws://127.0.0.1:4500` app-server client
- no phone pairing at first
- no public tunnel
- no Pi-side Codex integration

Add the relay/pairing/security model only if we later want to control Codex analysis from a phone or another remote browser.

## Open Questions For Our Webapp

- Should the tuning UI be a laptop-local app under `althold/desktop_tools`, or a new `althold/tuning_app`?
- Should Codex app-server be optional, with manual "open bundle for Codex" remaining as fallback?
- Should the app create a new Codex thread per flight session, or keep one long tuning thread per vehicle?
- Should `turn/start` prompts include raw JSONL snippets, summary files only, or file paths for Codex to inspect from the local workspace?
- Should parameter application stay manual at first, or should the UI later call Pi/FC MSP2 config writes after user approval?

## Initial Recommendation

Do not change the Pi logger into a Codex app-server client.

Instead, build a laptop-local tuning app in a later phase:

- backend: local Python or Node service under `althold/`
- frontend: local browser UI
- inputs: downloaded Pi session bundles and analysis outputs
- Codex integration: local `codex app-server` WebSocket client
- output: streamed tuning recommendation plus proposed `alt_est_*` changes
- parameter writes: manual approval first, then optional MSP2 write through the Pi/FC path

This matches the bridge repo's strongest design lesson: app-server is powerful, but it should be kept local and wrapped by a task-specific, controlled UI rather than exposed directly.
