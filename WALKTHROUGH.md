# Pi NTP Display: Code Walkthrough

How `Pi_NTP_Display.py` is put together, current as of v1.3.

## Overview

The script does two jobs at once:

1. **LCD loop** (main thread): cycles a 20x4 LCD through a clock page and pages of network, memory, GPS and chrony stats.
2. **Web server** (daemon thread): serves a status page on port `8080` plus a JSON endpoint the page polls.

Both read from the same shared data, so the LCD and the web page always show the same values.

## Data collection

Each stats area has a `get_*` function that runs a system command, parses its output, stores the result in `shared_stats`, and returns strings formatted for the LCD:

| Function | Source | `shared_stats` key |
|---|---|---|
| `get_network_data()` | `ip add show dev eth0`, `ip -s link show eth0` | `network` |
| `get_mem_data()` | `free` | `memory` |
| `get_gps_data()` | `gpspipe -r -x 3` (GPGGA and GPGSA sentences, parsed with `pynmea2`) | `gps` |
| `get_chrony_data()` | `chronyc tracking` | `chrony` |

`displayTime()` also writes `shared_stats["last_updated"]` every second while the clock page is showing.

Stats are only refreshed when the LCD reaches the matching `display*` page; there is no separate polling loop. A full LCD cycle takes around two minutes with the default timings in `main()`.

Parsing relies on fixed word positions in each command's output (e.g. `chronyResult[29]` for the last offset). If an OS or chrony update changes the output format, the affected fields will be wrong or missing and an error is printed to the console.

### Offset history

`get_chrony_data()` also appends `[epoch_ms, last_offset_seconds]` to `offset_history`, a `deque(maxlen=150)`. Because the web thread reads it while the LCD thread writes it, both sides hold `history_lock`; the web handler copies it to a list under the lock before serialising.

## Web server

Built on the standard library only (`http.server`, `socketserver`, `json`, `threading`). `run_web_server()` starts a `TCPServer` with `StatsHandler`, launched from `main()` in a daemon thread so it never blocks the LCD loop.

`StatsHandler` routes:

| Path | Response |
|---|---|
| `/` | The page (`WEB_PAGE`, a static HTML string) |
| `/api/stats` | `shared_stats` plus `offset_history` and `server_time_ms`, as JSON |
| anything else | 404 |

Responses are sent with `Cache-Control: no-store`. Per-request access logging is switched off (`log_message` is overridden) because the page polls every 2 seconds.

## The web page

`WEB_PAGE` is a self-contained HTML document with inline CSS and JavaScript: no external fonts, libraries or CDNs, so it works on an isolated network. All values are inserted with `textContent`, never as HTML.

**Polling:** `poll()` fetches `/api/stats` every 2 seconds and calls `render()`, which updates the page in place. If requests fail for more than three poll intervals, the status pill shows LINK LOST.

**Clock:** the browser measures the difference between its own clock and the Pi's using `server_time_ms` and the request round-trip time (assuming the response was generated halfway through the round trip). It keeps the estimate from the fastest round trip, refreshed at least once a minute. `tick()` runs on `requestAnimationFrame` and draws `Date.now() + clockOffset`, so the clock shows the Pi's time to the millisecond. The same offset is shown in the footer as "This device vs server".

**Status pill** (evaluated in `render()`):
- **NO SYNC**: chrony leap status is missing or "Not synchronised"
- **DEGRADED**: synced, but no 3D GPS fix or `|last offset| > 1 ms`
- **LOCKED**: everything else

**Offset chart:** `drawChart()` renders `offset_history` as an inline SVG polyline, scaled symmetrically around a dashed zero line to the largest absolute offset in the window.

**Units:** `fmtSec()` auto-scales seconds to ns / µs / ms / s; `fmtBytes()` formats memory (reported by `free` in KiB) and network counters.

**Tooltips:** chrony labels carry a `data-tip` attribute and `tabindex="0"`. CSS shows the text in a `::after` popover on `:hover` or `:focus`, so they work with a mouse, a keyboard, or a tap on a touch screen. Labels on the right-hand side use `.tip-right` so their popover opens leftwards.

## Testing without a Pi

The web side can be exercised on any machine by stubbing the hardware modules and filling `shared_stats` with sample values:

```python
import sys, types
sys.modules["pynmea2"] = types.ModuleType("pynmea2")
sys.modules["RPi"] = types.ModuleType("RPi")
sys.modules["RPi.GPIO"] = types.ModuleType("RPi.GPIO")

import Pi_NTP_Display as p
p.shared_stats["chrony"] = {"last_offset": "+0.000000213 s", "rms_offset": "0.000000641 s", "leap_status": "Normal"}
p.run_web_server()   # then open http://localhost:8080/
```
