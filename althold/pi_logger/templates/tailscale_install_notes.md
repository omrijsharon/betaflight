# Tailscale Notes

For Raspberry Pi OS, the standard install path is:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

For this project, `tailscale up` should be managed by the field network script and only run after the Pi has connected to the smartphone hotspot. Home Wi-Fi and fallback AP modes should leave Tailscale down unless we intentionally change that policy.
