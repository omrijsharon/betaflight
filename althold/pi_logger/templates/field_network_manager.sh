#!/usr/bin/env bash
set -euo pipefail

CONFIG="${ALTHOLD_FIELD_NETWORK_CONFIG:-/etc/althold/field_network.json}"
COMMAND="${1:-run-once}"

# Compatibility wrapper. The real implementation is the Python
# NetworkManager helper so config parsing and Tailscale policy stay in one place.
exec /usr/bin/python3 -m althold.pi_logger.field_network_manager --config "$CONFIG" "$COMMAND"
