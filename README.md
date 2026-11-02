# Pi NTP Display

A GPS-disciplined Raspberry Pi NTP server with a 20x4 LCD clock/stats display and a live web status page.

![Web status page](docs/web-ui.png)

## Overview

This is a rebuild of my GPS disciplined Raspberry Pi NTP server, using DietPi as a lightweight minimal OS ideally suited for a Pi 1 Model B.

The Pi uses a USB GPS dongle, modified to extract the PPS signal (fed into the Pi on GPIO18) and ultimately fed to gpsd/chrony for high accuracy.

To make it more useful I added a 20x4 LCD display to use it as a clock and show some stats on rotation. The display shows the time + date between each stats page, cycling through two pages of network stats, memory usage, two pages of GPS stats from `gpsd`, and five pages of chrony stats from `chronyc`. The same stats are also served as a web page.

LCD code is based on a simplified version of [Matt Hawkins' 20x4 LCD code](https://www.raspberrypi-spy.co.uk/2012/08/20x4-lcd-module-control-using-python/); I've just removed the backlight toggling.

**Version 1.4+ runs without root privileges** — uses `gpiozero` + `lgpio` (via `/dev/gpiomem`) for GPIO, `psutil` for system stats, and `gpsd-py3` for GPS data over the gpsd socket.

## Hardware

- Raspberry Pi (tested on a Pi 1 Model B)
- USB GPS receiver with its PPS output wired to GPIO18
- Displaytech 204A 20x4 character LCD (HD44780-compatible), driven in 4-bit mode

The full LCD pin-to-GPIO wiring table is in the header comment of [Pi_NTP_Display.py](Pi_NTP_Display.py).

## Requirements

### System packages (install once)
```bash
sudo apt-get update && sudo apt-get install -y gpsd gpsd-clients chrony python3-pip
```

Configure `gpsd` for your GPS receiver (e.g., `/dev/ttyAMA0` or `/dev/ttyUSB0`) and `chrony` to use gpsd/PPS as its reference.

### Python packages (no root required)
```bash
pip3 install -r requirements.txt
```

Or manually:
```bash
pip3 install gpiozero lgpio psutil gpsd-py3 pynmea2
```

### User permissions (no sudo needed)
Add your user to the required groups:
```bash
sudo usermod -a -G gpio,dialout $USER
# Log out and back in for groups to take effect
```
- `gpio` group: access to `/dev/gpiomem` for LCD control
- `dialout` group: access to GPS serial device (used by gpsd, not directly by this script)

## Running

**No sudo required!**
```bash
python3 Pi_NTP_Display.py
```

The LCD starts cycling through its pages and the web page is available at `http://<pi-ip>:8080/`.

### Run as a systemd service (recommended for production)

Create `/etc/systemd/system/pi-ntp-display.service`:
```ini
[Unit]
Description=Pi NTP Display
After=network.target gpsd.service chrony.service
Requires=gpsd.service chrony.service

[Service]
Type=simple
User=dietpi
Group=gpio
SupplementaryGroups=dialout
ExecStart=/usr/bin/python3 /home/dietpi/Pi_NTP_Display/Pi_NTP_Display.py
Restart=on-failure
RestartSec=5
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=read-only
ReadWritePaths=/home/dietpi/Pi_NTP_Display

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pi-ntp-display
```

## Web status page

The page is a single self-contained HTML file served by the script, with no external fonts, scripts or CDNs, so it works on a network with no internet access.

- **Live clock** to the millisecond, running on the Pi's time rather than the viewing device's.
- **Sync status**: LOCKED (chrony synced, 3D GPS fix, offset under 1 ms), DEGRADED (synced but no 3D fix or offset over 1 ms), NO SYNC (chrony not synchronised) or LINK LOST (the page can't reach the Pi).
- **Last offset and RMS offset**, auto-scaled to ns / µs / ms.
- **Offset history chart** of the last 150 chrony samples, kept on the Pi so it survives a page refresh.
- **Chrony, GPS and System panels**. Hover (or tap on a phone) any chrony label for a plain-English explanation of what it means.
- **Device vs server** in the footer: how far the clock of the device you're viewing from is from the Pi.

The page polls every 2 seconds and updates in place without reloading. Stats are collected by a background thread on independent intervals (chrony: 2s, GPS: 5s, network/memory: 10s) so the web API always returns fresh data instantly without blocking.

### JSON API

`GET /api/stats` returns the current stats as JSON, handy for scripts or other dashboards:

```json
{
  "last_updated": "2026-09-26 14:02:11",
  "memory":  { "total": "...", "used": "...", "free": "..." },
  "network": { "ip": "...", "state": "...", "mac": "...", "rx": "...", "tx": "...", ... },
  "gps":     { "fix": "3D Fix", "satellites": "09", "latitude": "...", "longitude": "...", "latitude_decimal": 51.5074, "longitude_decimal": -0.1278, ... },
  "chrony":  { "last_offset": "+0.000000213 s", "rms_offset": "...", "leap_status": "Normal", ... },
  "offset_history": [[1790000000000, 2.13e-07], ...],
  "server_time_ms": 1790000000123
}
```

Values are passed through as reported by the underlying tools (memory in KiB, network counters in bytes). `offset_history` entries are `[epoch milliseconds, offset in seconds]`. GPS includes both DMS (`latitude`/`longitude`) and decimal degrees (`latitude_decimal`/`longitude_decimal`) for easy copy/paste into mapping tools.

See [WALKTHROUGH.md](WALKTHROUGH.md) for how the code is structured.

## Changelog

**v1.4**
- **Performance**: Background stats collector thread (no more blocking subprocess calls in main loop); removed `shell=True` from all subprocess calls; web server uses `ThreadingTCPServer` for concurrent requests; static HTML served with ETag/conditional requests (304 Not Modified).
- **GPS**: Added `latitude_decimal` and `longitude_decimal` to JSON API and web page (6 decimal places).
- **Caching**: Static HTML page cached by browsers (ETag + `Cache-Control: public, max-age=60`).

**v1.3**
- Redesigned web page as a live "instrument panel" that updates in place instead of reloading every 5 seconds.
- Added `/api/stats` JSON endpoint, offset history chart, sync status indicator and millisecond clock.
- Web page now shows all chrony fields plus GPS dilution of precision, with explanatory tooltips on the chrony stats.
- Leap status now shows the full text (e.g. "Not synchronised"), and a scheduled leap second is no longer treated as loss of sync.

**v1.2**
- Added live time and date display to the web page.

**v1.1**
- Added web page for display of stats.

**v1.0**
- Initial rebuild on DietPi with LCD clock and rotating stats.

I'm sure there are more efficient ways of doing this, but overall I'm pleased with the result.
