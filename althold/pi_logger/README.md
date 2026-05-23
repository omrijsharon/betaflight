# Altitude Pi Logger

This is local code for the Pi Zero 2 W logger/webapp. It talks directly to Betaflight MSP2 over UART with `pyserial`; it does not use `yamspy`, `ArUco_Chaser`, or any OpenAI API key.

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

Real Pi deployment, UART validation, Wi-Fi fallback behavior, Tailscale behavior, and field testing are intentionally deferred.
