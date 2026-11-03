# AGENTS.md - Pi NTP Display

## Quick Start

```bash
# On Pi (DietPi)
cd ~/Pi_NTP_Display
source venv/bin/activate
python3 Pi_NTP_Display.py
```

Web UI at `http://<pi-ip>:8080/`, API at `/api/stats`.

## Architecture

- **Single file**: `Pi_NTP_Display.py` — LCD loop (main thread) + web server (daemon thread) + stats collector (daemon thread)
- **Shared state**: `shared_stats` dict + `offset_history` deque (thread-safe via `history_lock`)
- **No root required**: `gpiozero`/`lgpio` (via `/dev/gpiomem`), `psutil`, `gpsdclient` (gpsd socket)

## Dependencies

| Type | Packages |
|------|----------|
| System (apt) | `gpsd gpsd-clients chrony python3-gpiozero python3-lgpio python3-psutil python3-pip` |
| Python (venv) | `gpsdclient`, `pynmea2` (see `requirements.txt`) |

User must be in `gpio` and `dialout` groups.

## Key Behaviors

- **Chrony parsing**: Regex by field labels (not indices) — handles variable spacing in `chronyc -n tracking`
- **GPS satellites**: Shows "used/total" (e.g., `6/18`) from SKY report `satellites` list; fix mode from TPV `mode`
- **Stats intervals**: chrony 2s, GPS 5s, network/memory 10s
- **Web server**: `ThreadingTCPServer` on 8080, ETag caching for static HTML

## Common Tasks

```bash
# Update & run
cd ~/Pi_NTP_Display && git pull && source venv/bin/activate && python3 Pi_NTP_Display.py

# Systemd (production)
sudo systemctl restart pi-ntp-display
sudo journalctl -u pi-ntp-display -f

# Debug GPS
gpspipe -w -n 5

# Debug chrony
chronyc tracking
chronyc sources
```

## Gotchas

- **Externally-managed-environment**: Always use venv (`python3 -m venv venv --system-site-packages`)
- **GPS device**: Check `/etc/default/gpsd` has correct `DEVICES="/dev/ttyACM0 /dev/pps0"`
- **PPS**: Requires `dtoverlay=pps-gpio,gpiopin=18` in `/boot/firmware/config.txt`
- **Git permissions**: If `git pull` fails, `sudo chown -R dietpi:dietpi .git`