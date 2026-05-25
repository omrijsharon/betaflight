# Altitude EKF Tuning Agent Instructions

You are helping tune Betaflight's altitude estimator for robust altitude velocity and position estimation from IMU and barometer data.

## Goal

Analyze downloaded flight-log sessions, identify estimator problems, and recommend small parameter changes that improve altitude and vertical-velocity estimation.

## Project Paths

- Logs are stored under `althold/logs/`.
- Each session should contain `manifest.json`, `telemetry.jsonl`, `events.jsonl`, `config_start.json`, and `config_end.json` when available.
- Multi-session analysis runs are stored under `althold/logs/analysis_runs/<run_id>/`.
- Analysis tools live under `althold/analysis/`.
- The Pi logger and MSP client live under `althold/pi_logger/`.
- The tuning cockpit lives under `althold/tuning_app/`.

## Expected Workflow

1. Inspect the session bundle and existing analysis outputs.
2. Run or improve local analysis tools when metrics are missing or weak.
3. Compare the telemetry, events, and config snapshots.
4. Recommend only changes that are supported by the logged data.
5. Keep recommendations small and easy to reverse.

## Output Requirement

At the end of each analysis, create or update `recommendation.json` in the requested output directory. For a single session this is the session directory. For multi-session analysis this is the analysis run directory under `althold/logs/analysis_runs/`.

```json
{
  "schema": "althold-recommendation-v1",
  "session_id": "SESSION_ID",
  "created_utc": "2026-05-25T00:00:00.000Z",
  "source_thread_id": "CODEX_THREAD_ID_OR_NULL",
  "source_sessions": ["SESSION_ID"],
  "parameters": [
    {
      "name": "baro_noise_cm",
      "old_value": 150,
      "new_value": 180,
      "unit": "cm",
      "reason": "Brief reason tied to observed metrics.",
      "confidence": "medium"
    }
  ]
}
```

Use `confidence` values `low`, `medium`, or `high`.

## Tooling Policy

You may create or improve analysis scripts under `althold/analysis/`.
You may update this `AGENTS.md` file when the tuning workflow learns a better repeatable procedure.
Do not require an OpenAI API key in Pi or analysis tooling.
