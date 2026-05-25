# Tailscale Notes

For Raspberry Pi OS, the standard install path is:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --hostname omrijsharon-pi --accept-routes=false
```

For this project, `tailscale up` is normally managed by `field_network_manager.py` and only runs automatically after the Pi has connected to the smartphone hotspot. Home Wi-Fi is manual test-only:

```bash
sudo python3 -m althold.pi_logger.field_network_manager --config /etc/althold/field_network.json tailscale-home-test
sudo python3 -m althold.pi_logger.field_network_manager --config /etc/althold/field_network.json tailscale-down
```

Fallback AP mode leaves Tailscale down.
