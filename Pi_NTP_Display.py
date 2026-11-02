#!/usr/bin/env python3
#--------------------------------------------------------------------------------
#
# Pi NTP Server Stats + Clock Display
# LCD Driver with text justification
#
# Displaytech 204A 20x4 LCD
# https://uk.rs-online.com/web/p/lcd-monochrome-displays/5326818
#
# LCD code is a simplified version of Matt Hawkins work:
# https://www.raspberrypi-spy.co.uk/2012/08/20x4-lcd-module-control-using-python/
#
#---------------------------------------------------------------------------------
#
# The wiring for the LCD is as follows:
# 1 : GND [RPi Pin 6]
# 2 : 5V [RPi Pin 4]
# 3 : Contrast (0-5V) [10K pot between RPi Pins 4,6 wiper to LCD]
# 4 : RS (Register Select) [RPi Pin 26]
# 5 : R/W (Read Write) [0V via link on LCD]
# 6 : Enable or Strobe [RPi Pin 24]
# 7 : Data Bit 0 [N.C.]
# 8 : Data Bit 1 [N.C.]
# 9 : Data Bit 2 [N.C.]
# 10: Data Bit 3 [N.C.]
# 11: Data Bit 4 [RPi Pin 22]
# 12: Data Bit 5 [RPi Pin 18]
# 13: Data Bit 6 [RPi Pin 16]
# 14: Data Bit 7 [RPi Pin 15]
# 15: LCD Backlight +5V**
# 16: LCD Backlight GND
#
# The wiring for the GPS is as follows:
# Main NMEA via USB (/dev/ttyAMA0)
# PPS signal for accurate timing GPIO Pin 18
#
# REQUIREMENTS (no root needed if user in gpio,dialout groups):
#   pip3 install gpiozero lgpio psutil pynmea2 gpsd-py3
#
# GPIO: uses gpiozero + lgpio (via /dev/gpiomem) - needs 'gpio' group
# GPS:  uses gpsd-py3 connecting to gpsd socket - needs 'dialout' group for serial device
# Net/Mem: uses psutil (pure Python, no root)
# Chrony: uses chronyc over Unix socket (chronyc -n tracking) - no root needed
#
#---------------------------------------------------------------------------------

# import
import pynmea2
from gpiozero import DigitalOutputDevice
import time
from datetime import datetime
import threading
import http.server
import socketserver
import json
from collections import deque
import hashlib
import psutil
import socket
import re
from gpsdclient import GPSDClient

# Shared dictionary to store statistics for the web page
shared_stats = {
    "last_updated": "",
    "memory": {},
    "network": {},
    "gps": {},
    "chrony": {}
}

# Recent chrony last-offset samples as [epoch_ms, seconds], for the web page's history chart
offset_history = deque(maxlen=150)
history_lock = threading.Lock()

# Thread-safe cache for stats with timestamps
stats_cache = {}
stats_cache_lock = threading.Lock()
CACHE_TTL = 2.0  # seconds

# Define GPIO to LCD mapping (BCM numbering)
LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 22

# Define some device constants
LCD_WIDTH = 20    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# Global LCD pin objects (initialized in main)
lcd_rs = None
lcd_e = None
lcd_d4 = None
lcd_d5 = None
lcd_d6 = None
lcd_d7 = None

# Web page served at '/'. It is static: the browser pulls live values from /api/stats
WEB_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pi NTP Status</title>
<style>
  :root {
    --bg: #0a0c0f; --panel: #111419; --line: #1e232b; --text: #e4e8ee; --muted: #6f7885;
    --accent: #3ddc97; --warn: #f2b33d; --bad: #ff5f5f;
    --mono: ui-monospace, "SF Mono", "JetBrains Mono", "Cascadia Mono", Menlo, Consolas, "DejaVu Sans Mono", monospace;
    --sans: system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
  }
  * { box-sizing: border-box; }
  html, body { margin: 0; background: var(--bg); color: var(--text); font-family: var(--sans); }
  body { padding: 20px 16px 40px; max-width: 1100px; margin: 0 auto; }
  .num { font-family: var(--mono); font-variant-numeric: tabular-nums; }
  header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; }
  .brand { font-family: var(--mono); font-size: 13px; letter-spacing: .18em; color: var(--muted); text-transform: uppercase; }
  .brand b { color: var(--text); font-weight: 600; }
  .pill { display: inline-flex; align-items: center; gap: 8px; padding: 6px 12px; border-radius: 999px;
          border: 1px solid var(--line); font-family: var(--mono); font-size: 12px; letter-spacing: .12em; }
  .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); }
  .pill.ok .dot { background: var(--accent); box-shadow: 0 0 10px var(--accent); animation: pulse 2s infinite; }
  .pill.warn .dot { background: var(--warn); box-shadow: 0 0 10px var(--warn); }
  .pill.bad .dot { background: var(--bad); box-shadow: 0 0 10px var(--bad); }
  .pill.ok { color: var(--accent); } .pill.warn { color: var(--warn); } .pill.bad { color: var(--bad); }
  @keyframes pulse { 50% { opacity: .35; } }

  .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 18px 20px; }
  .label { font-size: 11px; letter-spacing: .14em; text-transform: uppercase; color: var(--muted); }
  h2 { margin: 0 0 14px; font-size: 11px; font-weight: 600; letter-spacing: .16em; text-transform: uppercase; color: var(--muted); }

  .hero { display: grid; grid-template-columns: 1fr auto; gap: 24px; align-items: end; margin-bottom: 16px; }
  .clock { font-family: var(--mono); font-variant-numeric: tabular-nums; font-size: clamp(44px, 11vw, 104px);
           line-height: 1; letter-spacing: -.02em; font-weight: 500; }
  .clock .ms { color: var(--muted); font-size: .42em; margin-left: .1em; }
  .sub { margin-top: 10px; font-family: var(--mono); font-size: 14px; color: var(--muted); }
  .sub span + span::before { content: "·"; margin: 0 10px; color: var(--line); }
  .offset { text-align: right; }
  .offset .big { font-family: var(--mono); font-variant-numeric: tabular-nums; font-size: clamp(28px, 6vw, 44px); color: var(--accent); }
  .offset .small { font-family: var(--mono); font-size: 13px; color: var(--muted); margin-top: 4px; }

  .spark { margin-bottom: 16px; }
  .spark-head { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 10px; }
  .spark-head .num { font-size: 12px; color: var(--muted); }
  svg.chart { width: 100%; height: 110px; display: block; }
  .zero { stroke: var(--line); stroke-dasharray: 3 4; }
  .trace { fill: none; stroke: var(--accent); stroke-width: 1.5; vector-effect: non-scaling-stroke; }
  .area { fill: var(--accent); opacity: .08; }
  .empty { fill: var(--muted); font-family: var(--mono); font-size: 12px; }

  .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px; }
  .row { display: flex; justify-content: space-between; gap: 12px; padding: 7px 0; border-top: 1px solid var(--line); font-size: 13px; }
  .row:first-of-type { border-top: 0; }
  .row .k { color: var(--muted); }
  .row .v { font-family: var(--mono); font-variant-numeric: tabular-nums; text-align: right; overflow-wrap: anywhere; }
  .stat { display: flex; align-items: baseline; gap: 10px; margin-bottom: 12px; }
  .stat .n { font-family: var(--mono); font-size: 40px; line-height: 1; }
  .badge { font-family: var(--mono); font-size: 11px; letter-spacing: .1em; padding: 3px 8px; border-radius: 4px;
           border: 1px solid currentColor; color: var(--muted); }
  .badge.ok { color: var(--accent); } .badge.warn { color: var(--warn); } .badge.bad { color: var(--bad); }
  .bar { height: 6px; background: var(--line); border-radius: 3px; overflow: hidden; margin: 6px 0 14px; }
  .bar i { display: block; height: 100%; width: 0; background: var(--accent); transition: width .6s ease; }

  /* Explanatory tooltips: hover on desktop, tap (focus) on touch screens */
  [data-tip] { position: relative; cursor: help; text-decoration: underline dotted var(--muted); text-underline-offset: 3px; outline: none; }
  [data-tip]::after { content: attr(data-tip); position: absolute; left: 0; top: calc(100% + 8px); z-index: 10;
    width: max-content; max-width: min(280px, 80vw); padding: 10px 12px; border-radius: 6px;
    background: #1a1f27; border: 1px solid #2c333d; box-shadow: 0 8px 24px rgba(0,0,0,.5);
    color: var(--text); font-family: var(--sans); font-size: 12px; line-height: 1.45; letter-spacing: normal;
    text-transform: none; text-align: left; font-weight: 400; white-space: normal;
    opacity: 0; visibility: hidden; transform: translateY(-4px); transition: opacity .15s, transform .15s, visibility .15s; }
  [data-tip]:hover::after, [data-tip]:focus::after { opacity: 1; visibility: visible; transform: none; }
  [data-tip]:focus-visible { outline: 1px solid var(--accent); outline-offset: 2px; }
  .tip-right::after { left: auto; right: 0; }
  .offset .label[data-tip] { display: inline-block; }

  footer { margin-top: 20px; display: flex; flex-wrap: wrap; justify-content: space-between; gap: 8px;
           font-family: var(--mono); font-size: 12px; color: var(--muted); }

  @media (max-width: 860px) { .grid { grid-template-columns: 1fr 1fr; } }
  @media (max-width: 600px) {
    .tip-right::after { left: 0; right: auto; }
    .grid { grid-template-columns: 1fr; }
    .hero { grid-template-columns: 1fr; }
    .offset { text-align: left; }
  }
</style>
</head>
<body>
  <header>
    <div class="brand"><b>Pi</b>&nbsp;/&nbsp;NTP&nbsp;Server</div>
    <div id="status" class="pill"><span class="dot"></span><span id="status-text">CONNECTING</span></div>
  </header>

  <section class="hero">
    <div>
      <div class="clock" id="clock">--:--:--<span class="ms">.---</span></div>
      <div class="sub"><span id="date">----</span><span id="utc">UTC --:--:--</span></div>
    </div>
    <div class="offset">
      <div class="label tip-right" tabindex="0" data-tip="How far the system clock was from true time at the last update, just before chrony corrected it. Positive means the clock was fast. Closer to zero is better.">Last offset</div>
      <div class="big" id="offset">--</div>
      <div class="small"><span class="tip-right" tabindex="0" data-tip="Root-mean-square of recent offsets: a long-term measure of how accurately the clock is being held. Lower is better.">RMS</span> <span id="rms">--</span></div>
    </div>
  </section>

  <section class="panel spark">
    <div class="spark-head">
      <h2 style="margin:0">Offset history</h2>
      <span class="num" id="spark-range"></span>
    </div>
    <svg class="chart" id="chart" viewBox="0 0 600 110" preserveAspectRatio="none"></svg>
  </section>

  <section class="grid">
    <div class="panel">
      <h2>Chrony</h2>
      <div class="row"><span class="k" tabindex="0" data-tip="How far the system clock currently is from chrony's best estimate of true time. Chrony slews the clock gradually rather than jumping it, so this shrinks over time. "Fast" means the clock is ahead.">System time</span><span class="v" id="c-system">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="How fast or slow the Pi's crystal would run without correction, in parts per million. Chrony compensates for this continuously. 1 ppm is about 86 ms per day.">Frequency</span><span class="v" id="c-freq">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="The difference between the frequency the reference source suggests and the one chrony is using. Near zero means chrony's frequency estimate is accurate.">Residual freq</span><span class="v" id="c-resfreq">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="The estimated error bound on the frequency value. Smaller means chrony is more confident in its frequency estimate.">Skew</span><span class="v" id="c-skew">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="Total network round-trip delay to the stratum-1 reference at the top of the chain. With the GPS/PPS source attached directly, this should be close to zero.">Root delay</span><span class="v" id="c-rdelay">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="Accumulated error estimate back to the stratum-1 reference. The worst-case clock error is roughly root dispersion + half the root delay.">Root dispersion</span><span class="v" id="c-rdisp">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="Time between the last two clock updates from the reference source.">Update interval</span><span class="v" id="c-interval">--</span></div>
      <div class="row"><span class="k" tabindex="0" data-tip="Normal when synchronised. Shows Insert/Delete second when a leap second is scheduled, or Not synchronised when chrony has no usable source.">Leap status</span><span class="v" id="c-leap">--</span></div>
    </div>

    <div class="panel">
      <h2>GPS</h2>
      <div class="stat"><span class="n" id="g-sats">--</span><span class="label">satellites</span>
        <span class="badge" id="g-fix">NO DATA</span></div>
      <div class="row"><span class="k">Latitude</span><span class="v" id="g-lat">--</span></div>
      <div class="row"><span class="k">Longitude</span><span class="v" id="g-lon">--</span></div>
      <div class="row"><span class="k">Lat (decimal)</span><span class="v" id="g-lat-dec">--</span></div>
      <div class="row"><span class="k">Lon (decimal)</span><span class="v" id="g-lon-dec">--</span></div>
      <div class="row"><span class="k">Altitude</span><span class="v" id="g-alt">--</span></div>
      <div class="row"><span class="k">PDOP / HDOP / VDOP</span><span class="v" id="g-dop">--</span></div>
    </div>

    <div class="panel">
      <h2>System</h2>
      <div class="row" style="border:0;padding-bottom:0"><span class="k">Memory</span><span class="v" id="m-used">--</span></div>
      <div class="bar"><i id="m-bar"></i></div>
      <div class="row"><span class="k">IP</span><span class="v" id="n-ip">--</span></div>
      <div class="row"><span class="k">Link</span><span class="v" id="n-state">--</span></div>
      <div class="row"><span class="k">MAC</span><span class="v" id="n-mac">--</span></div>
      <div class="row"><span class="k">RX / TX</span><span class="v" id="n-rxtx">--</span></div>
    </div>
  </section>

  <footer>
    <span id="updated">Stats: --</span>
    <span id="drift">This device vs server: --</span>
  </footer>

<script>
  var POLL_MS = 2000;
  var clockOffset = 0;   // server time minus browser time, in ms
  var bestRtt = Infinity;
  var lastOk = 0;

  function $(id) { return document.getElementById(id); }
  function set(id, v) { $(id).textContent = (v === undefined || v === null || v === "") ? "--" : v; }
  function pad(n, w) { n = String(n); while (n.length < (w || 2)) n = "0" + n; return n; }

  // Format a value in seconds with an SI unit suited to its size
  function fmtSec(s, signed) {
    if (s === null || isNaN(s)) return "--";
    var a = Math.abs(s), v, u;
    if (a === 0) { v = 0; u = "s"; }
    else if (a < 1e-6) { v = s * 1e9; u = "ns"; }
    else if (a < 1e-3) { v = s * 1e6; u = "µs"; }
    else if (a < 1) { v = s * 1e3; u = "ms"; }
    else { v = s; u = "s"; }
    var t = Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2);
    return (signed && s > 0 ? "+" : "") + t + " " + u;
  }
  function fmtBytes(b) {
    b = Number(b); if (isNaN(b)) return "--";
    var u = ["B", "KB", "MB", "GB", "TB"], i = 0;
    while (b >= 1024 && i < u.length - 1) { b /= 1024; i++; }
    return b.toFixed(i ? 1 : 0) + " " + u[i];
  }

  function tick() {
    var now = new Date(Date.now() + clockOffset);
    $("clock").innerHTML = pad(now.getHours()) + ":" + pad(now.getMinutes()) + ":" + pad(now.getSeconds()) +
      '<span class="ms">.' + pad(now.getMilliseconds(), 3) + "</span>";
    $("date").textContent = now.toLocaleDateString(undefined, { weekday: "short", year: "numeric", month: "short", day: "numeric" });
    $("utc").textContent = "UTC " + pad(now.getUTCHours()) + ":" + pad(now.getUTCMinutes()) + ":" + pad(now.getUTCSeconds());
    requestAnimationFrame(tick);
  }

  function setStatus(level, text) {
    $("status").className = "pill " + level;
    $("status-text").textContent = text;
  }

  function drawChart(hist) {
    var svg = $("chart"), W = 600, H = 110, mid = H / 2;
    if (!hist || hist.length < 2) {
      svg.innerHTML = '<line class="zero" x1="0" y1="' + mid + '" x2="' + W + '" y2="' + mid + '"/>' +
        '<text class="empty" x="8" y="' + (mid - 8) + '">collecting samples...</text>';
      set("spark-range", "");
      return;
    }
    var t0 = hist[0][0], t1 = hist[hist.length - 1][0], span = Math.max(t1 - t0, 1);
    var peak = 0;
    hist.forEach(function (p) { peak = Math.max(peak, Math.abs(p[1])); });
    peak = peak || 1e-9;
    var pts = hist.map(function (p) {
      return ((p[0] - t0) / span * W).toFixed(1) + "," + (mid - p[1] / peak * (mid - 6)).toFixed(1);
    });
    svg.innerHTML =
      '<line class="zero" x1="0" y1="' + mid + '" x2="' + W + '" y2="' + mid + '"/>' +
      '<polygon class="area" points="0,' + mid + " " + pts.join(" ") + " " + W + "," + mid + '"/>' +
      '<polyline class="trace" points="' + pts.join(" ") + '"/>';
    var mins = Math.round(span / 60000);
    set("spark-range", "\u00b1" + fmtSec(peak) + " \u00b7 last " + (mins < 1 ? "<1" : mins) + " min");
  }

  function render(d) {
    var c = d.chrony || {}, g = d.gps || {}, m = d.memory || {}, n = d.network || {};

    var off = parseFloat(c.last_offset), rms = parseFloat(c.rms_offset);
    set("offset", fmtSec(off, true));
    set("rms", fmtSec(rms));

    var sys = parseFloat(c.system_time);
    set("c-system", isNaN(sys) ? c.system_time : fmtSec(sys) + " " + (c.system_time.split(" ").pop() || ""));
    set("c-freq", c.frequency);
    set("c-resfreq", c.residual_frequency);
    set("c-skew", c.skew);
    set("c-rdelay", fmtSec(parseFloat(c.root_delay)));
    set("c-rdisp", fmtSec(parseFloat(c.root_dispersion)));
    set("c-interval", c.update_interval);
    var leap = { Not: "Not synchronised", Insert: "Insert second", Delete: "Delete second" }[c.leap_status] || c.leap_status;
    set("c-leap", leap);

    set("g-sats", g.satellites ? parseInt(g.satellites, 10) : "--");
    var fix = g.fix || "NO DATA";
    $("g-fix").textContent = fix.toUpperCase();
    $("g-fix").className = "badge " + (fix === "3D Fix" ? "ok" : fix === "2D Fix" ? "warn" : "bad");
    set("g-lat", g.latitude);
    set("g-lon", g.longitude);
    set("g-lat-dec", g.latitude_decimal !== undefined && g.latitude_decimal !== null ? g.latitude_decimal.toFixed(6) : "--");
    set("g-lon-dec", g.longitude_decimal !== undefined && g.longitude_decimal !== null ? g.longitude_decimal.toFixed(6) : "--");
    set("g-alt", g.altitude);
    set("g-dop", g.pdop ? [g.pdop, g.hdop, g.vdop].join(" / ") : null);

    var total = parseInt(m.total, 10), used = parseInt(m.used, 10);
    if (total && !isNaN(used)) {
      // psutil reports bytes
      set("m-used", fmtBytes(used) + " / " + fmtBytes(total));
      $("m-bar").style.width = (used / total * 100).toFixed(1) + "%";
    }
    set("n-ip", n.ip);
    set("n-state", n.state);
    set("n-mac", n.mac ? n.mac.replace(/(..)(?!$)/g, "$1:") : null);
    set("n-rxtx", n.rx ? fmtBytes(n.rx) + " / " + fmtBytes(n.tx) : null);

    drawChart(d.offset_history);
    set("updated", "Stats: " + (d.last_updated || "--"));

    // Overall health: chrony must be synchronised; GPS fix and small offset make it fully locked
    if (!c.leap_status || c.leap_status === "Not") setStatus("bad", "NO SYNC");
    else if (fix !== "3D Fix" || Math.abs(off) > 1e-3) setStatus("warn", "DEGRADED");
    else setStatus("ok", "LOCKED");
  }

  function poll() {
    var sent = Date.now();
    fetch("/api/stats", { cache: "no-store" })
      .then(function (r) { if (!r.ok) throw new Error(r.status); return r.json(); })
      .then(function (d) {
        var recv = Date.now(), rtt = recv - sent;
        // Keep the estimate from the fastest round trip (least network jitter), refreshed occasionally
        if (rtt <= bestRtt || recv - lastOk > 60000) {
          bestRtt = rtt;
          clockOffset = d.server_time_ms - (sent + recv) / 2;
          set("drift", "This device vs server: " + (clockOffset >= 0 ? "-" : "+") + Math.abs(Math.round(clockOffset)) + " ms (\u00b1" + Math.ceil(rtt / 2) + ")");
        }
        lastOk = recv;
        render(d);
      })
      .catch(function () {
        if (Date.now() - lastOk > POLL_MS * 3) setStatus("bad", "LINK LOST");
      })
      .then(function () { setTimeout(poll, POLL_MS); });
  }

  drawChart([]);
  requestAnimationFrame(tick);
  poll();
</script>
</body>
</html>
"""

# Pre-computed ETag for the static HTML page
WEB_PAGE_ETAG = hashlib.md5(WEB_PAGE.encode()).hexdigest()

class StatsHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            # Check If-None-Match header for conditional request
            if_none_match = self.headers.get('If-None-Match')
            if if_none_match == WEB_PAGE_ETAG:
                self.send_response(304)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(WEB_PAGE.encode())))
            self.send_header('ETag', WEB_PAGE_ETAG)
            self.send_header('Cache-Control', 'public, max-age=60')
            self.end_headers()
            self.wfile.write(WEB_PAGE.encode())
        elif self.path == '/api/stats':
            # Serve fresh data from shared_stats (updated by background collector)
            with history_lock:
                history = list(offset_history)
            payload = dict(shared_stats, offset_history=history, server_time_ms=int(time.time() * 1000))
            body = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # The page polls every 2s; don't flood the console with access logs
        pass

def run_web_server():
    PORT = 8080
    Handler = StatsHandler
    # Allow address reuse to avoid 'Address already in use' errors
    socketserver.TCPServer.allow_reuse_address = True
    # Use ThreadingTCPServer to handle multiple concurrent requests
    class ThreadingHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        allow_reuse_address = True
        daemon_threads = True

    with ThreadingHTTPServer(("", PORT), Handler) as httpd:
        print(f"Serving at port {PORT}")
        httpd.serve_forever()


# =============================================================================
# LCD Functions using gpiozero (no root required with gpio group)
# =============================================================================

def lcd_toggle_enable():
    # Toggle enable
    time.sleep(E_DELAY)
    lcd_e.on()
    time.sleep(E_PULSE)
    lcd_e.off()
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = data
    # mode = True  for character
    #        False for command
    lcd_rs.value = mode

    # High bits
    lcd_d4.off()
    lcd_d5.off()
    lcd_d6.off()
    lcd_d7.off()
    if bits & 0x10 == 0x10:
        lcd_d4.on()
    if bits & 0x20 == 0x20:
        lcd_d5.on()
    if bits & 0x40 == 0x40:
        lcd_d6.on()
    if bits & 0x80 == 0x80:
        lcd_d7.on()

    # Toggle 'Enable' pin
    lcd_toggle_enable()

    # Low bits
    lcd_d4.off()
    lcd_d5.off()
    lcd_d6.off()
    lcd_d7.off()
    if bits & 0x01 == 0x01:
        lcd_d4.on()
    if bits & 0x02 == 0x02:
        lcd_d5.on()
    if bits & 0x04 == 0x04:
        lcd_d6.on()
    if bits & 0x08 == 0x08:
        lcd_d7.on()

    # Toggle 'Enable' pin
    lcd_toggle_enable()

def lcd_init():
    # Initialise display
    lcd_byte(0x33, LCD_CMD) # 110011 Initialise
    lcd_byte(0x32, LCD_CMD) # 110010 Initialise
    lcd_byte(0x06, LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C, LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28, LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01, LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

def blank_display():
    # Blank display
    lcd_byte(0x01, LCD_CMD)

def lcd_string(message, line, style):
    # Send string to display
    # style=1 Left justified
    # style=2 Centred
    # style=3 Right justified

    if style == 1:
        message = message.ljust(LCD_WIDTH, " ")
    elif style == 2:
        message = message.center(LCD_WIDTH, " ")
    elif style == 3:
        message = message.rjust(LCD_WIDTH, " ")

    lcd_byte(line, LCD_CMD)

    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)


# =============================================================================
# Data Collection Functions (no subprocess, no root)
# =============================================================================

def get_mem_data():
    try:
        mem = psutil.virtual_memory()
        # psutil returns bytes
        shared_stats["memory"] = {
            "total": str(mem.total),
            "used": str(mem.used),
            "free": str(mem.available)
        }
        return True
    except Exception as e:
        print(f"Error getting memory stats: {e}")
        return False

def get_network_data():
    try:
        # Get interface info for eth0 (or first non-loopback)
        addrs = psutil.net_if_addrs()
        stats = psutil.net_if_stats()
        io_counters = psutil.net_io_counters(pernic=True)

        # Find the primary interface (prefer eth0, then first non-loopback up interface)
        target_iface = None
        for iface in ['eth0', 'enp0s3', 'enx*', 'wlan0']:
            if iface in addrs:
                target_iface = iface
                break
        if not target_iface:
            for iface, iface_addrs in addrs.items():
                if iface != 'lo' and iface in stats and stats[iface].isup:
                    target_iface = iface
                    break

        if not target_iface:
            return False

        iface_addrs = addrs[target_iface]
        iface_stats = stats.get(target_iface)
        iface_io = io_counters.get(target_iface, None)

        ip_val = "N/A"
        mac_val = "N/A"
        state_val = "DOWN"
        mtu_val = "N/A"

        for addr in iface_addrs:
            if addr.family == socket.AF_INET:
                ip_val = addr.address
            elif addr.family == socket.AF_PACKET:
                mac_val = addr.address.replace(':', '')

        if iface_stats:
            state_val = "UP" if iface_stats.isup else "DOWN"
            mtu_val = str(iface_stats.mtu)

        rx_val = str(iface_io.bytes_recv) if iface_io else "0"
        tx_val = str(iface_io.bytes_sent) if iface_io else "0"
        port_val = target_iface

        shared_stats["network"] = {
            "port": port_val,
            "state": state_val,
            "ip": ip_val,
            "mac": mac_val,
            "mtu": mtu_val,
            "rx": rx_val,
            "tx": tx_val
        }
        return True
    except Exception as e:
        print(f"Network error: {e}")
        return False

def format_gps_coord(val, dir, is_lat, degree_symbol=chr(223)):
    # Determine the degree part length based on lat/lon
    # Lat: DDMM.MMMM -> 2 digits for degrees
    # Lon: DDDMM.MMMM -> 3 digits for degrees
    deg_len = 2 if is_lat else 3

    try:
        val_str = str(val).strip()
        if not val_str:
            return "N/A"

        degrees = val_str[:deg_len]
        minutes = val_str[deg_len:]

        # degree_symbol defaults to chr(223), the HD44780 ROM code for the
        # degree glyph on the physical LCD. The web page passes the real
        # Unicode degree sign (U+00B0) instead, since chr(223) is 'ß' in
        # Unicode text and renders as garbage in a browser.
        return f"{degrees}{degree_symbol}{minutes}'{dir}"
    except Exception as e:
        print(f"Error formatting: {e}")
        return f"{val}{dir}"

def nmea_to_decimal(val, dir, is_lat):
    """Convert NMEA DDMM.MMMM format to decimal degrees."""
    try:
        val_str = str(val).strip()
        if not val_str:
            return None
        deg_len = 2 if is_lat else 3
        degrees = float(val_str[:deg_len])
        minutes = float(val_str[deg_len:])
        decimal = degrees + minutes / 60.0
        if dir in ('S', 'W'):
            decimal = -decimal
        return decimal
    except Exception as e:
        print(f"Error converting to decimal: {e}")
        return None

def get_gps_data():
    try:
        # Connect to gpsd (default localhost:2947)
        client = GPSDClient(host="127.0.0.1", port=2947, timeout=5)

        # Get TPV (position) and SKY (satellites/DOP) reports
        tpv_report = None
        sky_report = None
        start_time = time.time()
        max_wait = 10  # seconds to wait for both reports

        for report in client.dict_stream():
            # dict_stream returns dicts, check keys not attributes
            if report.get('class') == 'TPV' and report.get('lat') is not None:
                tpv_report = report
            elif report.get('class') == 'SKY':
                sky_report = report
            if tpv_report and sky_report:
                break
            # Timeout safety
            if time.time() - start_time > max_wait:
                break

        if not tpv_report:
            raise ValueError("No TPV report from gpsd")

        # Extract GPS data
        sat_val = str(sky_report.get('nSat', 0)) if sky_report else "0"
        lat_val = format_gps_coord(tpv_report['lat'], 'N' if tpv_report['lat'] >= 0 else 'S', True)
        lon_val = format_gps_coord(tpv_report['lon'], 'E' if tpv_report['lon'] >= 0 else 'W', False)
        lat_web = format_gps_coord(tpv_report['lat'], 'N' if tpv_report['lat'] >= 0 else 'S', True, degree_symbol="°")
        lon_web = format_gps_coord(tpv_report['lon'], 'E' if tpv_report['lon'] >= 0 else 'W', False, degree_symbol="°")
        lat_decimal = tpv_report['lat']
        lon_decimal = tpv_report['lon']
        alt_val = f"{tpv_report['alt']:.1f}m" if tpv_report.get('alt') is not None else "N/A"

        # Fix mode: 1=No fix, 2=2D, 3=3D
        fix_mode = sky_report.get('mode', 1) if sky_report else 1
        pdop_val = f"{sky_report.get('pdop', 0):.1f}" if sky_report else "0"
        hdop_val = f"{sky_report.get('hdop', 0):.1f}" if sky_report else "0"
        vdop_val = f"{sky_report.get('vdop', 0):.1f}" if sky_report else "0"

        readable_fix = ("No fix", "2D Fix", "3D Fix")
        fix_text = readable_fix[fix_mode] if fix_mode < 3 else "3D Fix"

        shared_stats["gps"] = {
            "satellites": sat_val,
            "latitude": lat_web,
            "longitude": lon_web,
            "latitude_decimal": lat_decimal,
            "longitude_decimal": lon_decimal,
            "altitude": alt_val,
            "fix": fix_text,
            "pdop": pdop_val,
            "hdop": hdop_val,
            "vdop": vdop_val
        }
        return True

    except Exception as e:
        print(f"GPS error: {e}")
        return False

def get_chrony_data():
    try:
        # Use chronyc over Unix socket (no root needed)
        import subprocess
        output = subprocess.check_output(
            ["chronyc", "-n", "tracking"],
            text=True,
            stderr=subprocess.DEVNULL
        )

        # DEBUG: print raw output to diagnose parsing issues
        print(f"DEBUG chrony output:\n{output}")

        # Robust parsing using regex for each field
        # chronyc tracking output format:
        # Reference ID    : 47505300 (GPS)
        # Stratum         : 1
        # Ref time (UTC)  : Mon Nov 02 21:59:48 2026
        # System time     : 0.000000001 seconds slow of NTP time
        # Last offset     : +0.000007034 seconds
        # RMS offset      : 0.000097306 seconds
        # Frequency       : 12.116 ppm fast
        # Residual freq   : +0.011 ppm
        # Skew            : 0.269 ppm
        # Root delay      : 0.000000001 seconds
        # Root dispersion : 0.000030802 seconds
        # Update interval : 16.0 seconds
        # Leap status     : Normal

        import re

        def extract(key, pattern=r'\s*:\s*(.+)$'):
            m = re.search(rf'^{re.escape(key)}{pattern}', output, re.MULTILINE)
            result = m.group(1).strip() if m else None
            print(f"DEBUG extract '{key}': {result}")
            return result

        ref_id = extract("Reference ID")
        stratum = extract("Stratum")
        ref_time = extract(r"Ref time \(UTC\)")
        system_time = extract("System time")
        last_offset = extract("Last offset")
        rms_offset = extract("RMS offset")
        frequency = extract("Frequency")
        res_freq = extract("Residual freq")
        skew = extract("Skew")
        root_delay = extract("Root delay")
        root_dispersion = extract("Root dispersion")
        update_interval = extract("Update interval")
        leap_status = extract("Leap status")

        # Validate required fields
        if not all([system_time, last_offset, rms_offset, leap_status]):
            raise ValueError("Missing required chrony fields")

        # last_offset is like "+0.000007034 seconds" - extract numeric value for history
        offset_match = re.search(r'([+-]?\d+\.?\d*)', last_offset)
        offset_seconds = float(offset_match.group(1)) if offset_match else 0.0

        with history_lock:
            offset_history.append([int(time.time() * 1000), offset_seconds])

        # Format values to match original format for display compatibility
        system_time_val = f"{system_time}"
        last_offset_val = f"{last_offset}"
        rms_offset_val = f"{rms_offset}"
        freq_val = f"{frequency}"
        res_freq_val = f"{res_freq}"
        skew_val = f"{skew}"
        root_dly_val = f"{root_delay} sec"
        root_disp_val = f"{root_dispersion} sec"
        up_int_val = f"{update_interval} sec"
        lp_stat_val = f"{leap_status}"

        shared_stats["chrony"] = {
            "system_time": system_time_val,
            "last_offset": last_offset_val,
            "rms_offset": rms_offset_val,
            "frequency": freq_val,
            "residual_frequency": res_freq_val,
            "skew": skew_val,
            "root_delay": root_dly_val,
            "root_dispersion": root_disp_val,
            "update_interval": up_int_val,
            "leap_status": lp_stat_val
        }
        return True

    except Exception as e:
        print(f"Chrony error: {e}")
        return False


# Background collector thread - runs independently to keep stats fresh
def stats_collector():
    """Background thread that periodically collects all stats and caches them."""
    # Initial collection intervals (seconds)
    intervals = {
        "memory": 10.0,
        "network": 10.0,
        "gps": 5.0,
        "chrony": 2.0,
    }
    last_run = {k: 0.0 for k in intervals}

    while True:
        now = time.time()

        # Memory
        if now - last_run["memory"] >= intervals["memory"]:
            try:
                get_mem_data()
                last_run["memory"] = now
            except Exception as e:
                print(f"Memory collector error: {e}")

        # Network
        if now - last_run["network"] >= intervals["network"]:
            try:
                get_network_data()
                last_run["network"] = now
            except Exception as e:
                print(f"Network collector error: {e}")

        # GPS
        if now - last_run["gps"] >= intervals["gps"]:
            try:
                get_gps_data()
                last_run["gps"] = now
            except Exception as e:
                print(f"GPS collector error: {e}")

        # Chrony
        if now - last_run["chrony"] >= intervals["chrony"]:
            try:
                get_chrony_data()
                last_run["chrony"] = now
            except Exception as e:
                print(f"Chrony collector error: {e}")

        # Update last_updated timestamp
        shared_stats["last_updated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Sleep a bit before checking again
        time.sleep(0.5)


# =============================================================================
# Display Functions (read from shared_stats)
# =============================================================================

def displayTime(cycles):
    blank_display()

    # Initialise cycle count
    x = 1

    # Print page title
    lcd_string("Date and Time", LCD_LINE_1, 2)
    lcd_string("--------------------", LCD_LINE_2, 2)

    # Initialise time and date strings
    timestr = ""
    datestr = ""

    while x < cycles:
        # Get current date and time
        now = datetime.now()
        shared_stats["last_updated"] = now.strftime("%Y-%m-%d %H:%M:%S")

        # Update only if changed
        if datestr != now.strftime("%b %d, %Y"):
            datestr = now.strftime("%b %d, %Y")
            print(datestr)
            lcd_string(datestr, LCD_LINE_3, 2)

        if timestr != now.strftime("%H:%M:%S"):
            timestr = now.strftime("%H:%M:%S")
            print(timestr)
            lcd_string(timestr, LCD_LINE_4, 2)

        x += 1
        time.sleep(1)

def displayMemData(delay):
    # Read from cached stats (updated by background collector)
    mem = shared_stats.get("memory", {})
    memTotal = mem.get("total")
    memUsed = mem.get("used")
    memFree = mem.get("free")

    if memTotal:
        blank_display()
        print("Memory Stats")
        lcd_string("Memory Stats", LCD_LINE_1, 2)
        print(f"Total: {memTotal}")
        lcd_string(f"Total: {memTotal}", LCD_LINE_2, 1)
        print(f"Used: {memUsed}")
        lcd_string(f"Used: {memUsed}", LCD_LINE_3, 1)
        print(f"Free: {memFree}")
        lcd_string(f"Free: {memFree}", LCD_LINE_4, 1)
    else:
        print("Error getting memory stats")
        blank_display()
        lcd_string("********************", LCD_LINE_1, 2)
        lcd_string("*   Memory stats   *", LCD_LINE_2, 2)
        lcd_string("*      error       *", LCD_LINE_3, 2)
        lcd_string("********************", LCD_LINE_4, 2)

    time.sleep(delay)

def displayNetworkData(page, delay):
    # Read from cached stats (updated by background collector)
    net = shared_stats.get("network", {})
    port_val = net.get("port")
    state_val = net.get("state")
    ip_val = net.get("ip")
    mac_val = net.get("mac")
    mtu_val = net.get("mtu")
    rx_val = net.get("rx")
    tx_val = net.get("tx")

    if port_val:
        port = f"Port: {port_val}"
        state = f"Network status: {state_val}"
        ip = f"IP: {ip_val}"
        mac = f"MAC: {mac_val}"
        mtu = f"MTU: {mtu_val}"
        rx = f"RX: {rx_val}"
        tx = f"TX: {tx_val}"

        # Print Data
        if page == 1:
            # Page 1
            blank_display()
            print(port)
            lcd_string(port, LCD_LINE_1, 1)
            print(state)
            lcd_string(state, LCD_LINE_2, 1)
            print(ip)
            lcd_string(ip, LCD_LINE_3, 1)
            print(mac)
            lcd_string(mac, LCD_LINE_4, 1)

        elif page == 2:
            # Page 2
            blank_display()
            print(mtu)
            lcd_string(mtu, LCD_LINE_1, 1)
            print(rx)
            lcd_string(rx, LCD_LINE_2, 1)
            print(tx)
            lcd_string(tx, LCD_LINE_3, 1)

    else:
        print("Network error")
        blank_display()
        lcd_string("********************", LCD_LINE_1, 2)
        lcd_string("*  Network error   *", LCD_LINE_2, 2)
        lcd_string("*                  *", LCD_LINE_3, 2)
        lcd_string("********************", LCD_LINE_4, 2)

    time.sleep(delay)

def displayGPSData(page, delay):
    # Read from cached stats (updated by background collector)
    gps = shared_stats.get("gps", {})
    sat_val = gps.get("satellites")
    lat_val = gps.get("latitude")
    lon_val = gps.get("longitude")
    alt_val = gps.get("altitude")
    fix_text = gps.get("fix")
    pdop_val = gps.get("pdop")
    hdop_val = gps.get("hdop")
    vdop_val = gps.get("vdop")

    if sat_val:
        satellites = f"Satellites: {sat_val}"
        latitude = f"Lat: {lat_val}"
        longitude = f"Lon: {lon_val}"
        altitude = f"Alt: {alt_val}"
        fix = f"Fix: {fix_text}"
        pdop = f"PDOP: {pdop_val}"
        hdop = f"HDOP: {hdop_val}"
        vdop = f"VDOP: {vdop_val}"

        if page == 1:
            # Page 1
            blank_display()
            print(satellites)
            print(latitude)
            print(longitude)
            print(altitude)
            lcd_string(satellites, LCD_LINE_3, 1)
            lcd_string(latitude, LCD_LINE_1, 1)
            lcd_string(longitude, LCD_LINE_2, 1)
            lcd_string(altitude, LCD_LINE_4, 1)

        elif page == 2:
            # Page 2
            blank_display()
            print(fix)
            print(pdop)
            print(hdop)
            print(vdop)
            lcd_string(fix, LCD_LINE_1, 1)
            lcd_string(pdop, LCD_LINE_2, 1)
            lcd_string(hdop, LCD_LINE_3, 1)
            lcd_string(vdop, LCD_LINE_4, 1)

        time.sleep(delay)

    else:
        print("gpsd error or no gps data")
        blank_display()
        lcd_string("********************", LCD_LINE_1, 2)
        lcd_string("*   gpsd error     *", LCD_LINE_2, 2)
        lcd_string("*                  *", LCD_LINE_3, 2)
        lcd_string("********************", LCD_LINE_4, 2)
        time.sleep(delay)

def displayChronyStats(page, delay):
    blank_display()

    # Read from cached stats (updated by background collector)
    chrony = shared_stats.get("chrony", {})
    systemTime = chrony.get("system_time")
    lastOffset = chrony.get("last_offset")
    rmsOffset = chrony.get("rms_offset")
    freq = chrony.get("frequency")
    resFreq = chrony.get("residual_frequency")
    skew = chrony.get("skew")
    rootDly = chrony.get("root_delay")
    rootDisp = chrony.get("root_dispersion")
    upInt = chrony.get("update_interval")
    lpStat = chrony.get("leap_status")

    if systemTime:
        # Output stats to console and LCD
        if page == 1:
            # Page 1
            blank_display()
            print("System Time:")
            print(systemTime)
            print("Last Offset:")
            print(lastOffset)
            lcd_string("System Time: ", LCD_LINE_1, 1)
            lcd_string(systemTime, LCD_LINE_2, 3)
            lcd_string("Last offset: ", LCD_LINE_3, 1)
            lcd_string(lastOffset, LCD_LINE_4, 3)

        elif page == 2:
            # Page 2
            blank_display()
            print("RMS offset: ")
            print(rmsOffset)
            print("Frequency: ")
            print(freq)
            lcd_string("RMS offset: ", LCD_LINE_1, 1)
            lcd_string(rmsOffset, LCD_LINE_2, 3)
            lcd_string("Frequency: ", LCD_LINE_3, 1)
            lcd_string(freq, LCD_LINE_4, 3)

        elif page == 3:
            # Page 3
            blank_display()
            print("Residual frequency: ")
            print(resFreq)
            print("Skew: ")
            print(skew)
            lcd_string("Residual frequency: ", LCD_LINE_1, 1)
            lcd_string(resFreq, LCD_LINE_2, 3)
            lcd_string("Skew: ", LCD_LINE_3, 1)
            lcd_string(skew, LCD_LINE_4, 3)

        elif page == 4:
            # Page 4
            blank_display()
            print("Root delay: ")
            print(rootDly)
            print("Root dispersion: ")
            print(rootDisp)
            lcd_string("Root delay: ", LCD_LINE_1, 1)
            lcd_string(rootDly, LCD_LINE_2, 3)
            lcd_string("Root dispersion: ", LCD_LINE_3, 1)
            lcd_string(rootDisp, LCD_LINE_4, 3)

        elif page == 5:
            # Page 5
            blank_display()
            print("Update Interval:")
            print(upInt)
            print("Leap Status: ")
            print(lpStat)
            lcd_string("Update Interval:", LCD_LINE_1, 1)
            lcd_string(upInt, LCD_LINE_2, 3)
            lcd_string("Leap Year Status: ", LCD_LINE_3, 1)
            lcd_string(lpStat, LCD_LINE_4, 3)

        time.sleep(delay)

    else:
        print("chronyc error or no data")
        blank_display()
        lcd_string("********************", LCD_LINE_1, 2)
        lcd_string("*  chronyc error   *", LCD_LINE_2, 2)
        lcd_string("*                  *", LCD_LINE_3, 2)
        lcd_string("********************", LCD_LINE_4, 2)
        time.sleep(delay)


def main():
    global lcd_rs, lcd_e, lcd_d4, lcd_d5, lcd_d6, lcd_d7

    # Initialize LCD pins using gpiozero (uses lgpio /dev/gpiomem, no root needed)
    print("Initializing LCD GPIO...")
    lcd_rs = DigitalOutputDevice(LCD_RS)
    lcd_e = DigitalOutputDevice(LCD_E)
    lcd_d4 = DigitalOutputDevice(LCD_D4)
    lcd_d5 = DigitalOutputDevice(LCD_D5)
    lcd_d6 = DigitalOutputDevice(LCD_D6)
    lcd_d7 = DigitalOutputDevice(LCD_D7)

    # Start Web Server in a separate thread
    try:
        web_thread = threading.Thread(target=run_web_server, daemon=True)
        web_thread.start()
        print("Web server started on port 8080")
    except Exception as e:
        print(f"Failed to start web server: {e}")

    # Start background stats collector thread
    try:
        collector_thread = threading.Thread(target=stats_collector, daemon=True)
        collector_thread.start()
        print("Stats collector started")
    except Exception as e:
        print(f"Failed to start stats collector: {e}")

    # Initialise display
    lcd_init()

    # Startup message
    lcd_string("GPS Disciplined", LCD_LINE_1, 2)
    lcd_string("NTP Time Server", LCD_LINE_2, 2)
    lcd_string("v1.4-no-root", LCD_LINE_3, 2)
    lcd_string("No root required", LCD_LINE_4, 2)
    time.sleep(2)

    # Time to display each stats page (seconds)
    stats_delay = 3
    # Time to display time page (seconds)
    time_cycles = 10
    # Number of GPS data pages
    gpsPages = 2
    # Number of chrony stats pages
    chronyPages = 5
    # Number of pages of network stats
    netPages = 2

    while True:
        # Display time and network stats alternately
        i = 1
        while i <= netPages:
            displayTime(time_cycles)
            displayNetworkData(i, stats_delay)
            if i == netPages:
                break
            i += 1

        # Display time and memory stats
        displayTime(time_cycles)
        displayMemData(stats_delay)

        # Display time and GPS data alternately
        i = 1
        while i <= gpsPages:
            displayTime(time_cycles)
            displayGPSData(i, stats_delay)
            if i == gpsPages:
                break
            i += 1

        # Display time and chrony stats alternately
        i = 1
        while i <= chronyPages:
            displayTime(time_cycles)
            displayChronyStats(i, stats_delay)
            if i == chronyPages:
                break
            i += 1

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        blank_display()
        lcd_string("********************", LCD_LINE_1, 2)
        lcd_string("*  System Stopped  *", LCD_LINE_2, 2)
        lcd_string("*     Goodbye!     *", LCD_LINE_3, 2)
        lcd_string("********************", LCD_LINE_4, 2)

        # Cleanup GPIO
        try:
            if lcd_rs: lcd_rs.close()
            if lcd_e: lcd_e.close()
            if lcd_d4: lcd_d4.close()
            if lcd_d5: lcd_d5.close()
            if lcd_d6: lcd_d6.close()
            if lcd_d7: lcd_d7.close()
        except:
            pass