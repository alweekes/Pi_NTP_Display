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

# The wiring for the GPS is as follows:
# Main NMEA via USB (/dev/ttyAMA0)
# PPS signal for accurate timing GPIO Pin 18

#import
import pynmea2
import RPi.GPIO as GPIO
import time
from datetime import datetime
import subprocess
import threading
import http.server
import socketserver
import json
from collections import deque

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

# Define GPIO to LCD mapping
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
      <div class="row"><span class="k" tabindex="0" data-tip="How far the system clock currently is from chrony's best estimate of true time. Chrony slews the clock gradually rather than jumping it, so this shrinks over time. “Fast” means the clock is ahead.">System time</span><span class="v" id="c-system">--</span></div>
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
        '<text class="empty" x="8" y="' + (mid - 8) + '">collecting samples…</text>';
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
    set("spark-range", "±" + fmtSec(peak) + " · last " + (mins < 1 ? "<1" : mins) + " min");
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
    set("g-alt", g.altitude);
    set("g-dop", g.pdop ? [g.pdop, g.hdop, g.vdop].join(" / ") : null);

    var total = parseInt(m.total, 10), used = parseInt(m.used, 10);
    if (total && !isNaN(used)) {
      // `free` reports KiB
      set("m-used", fmtBytes(used * 1024) + " / " + fmtBytes(total * 1024));
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
          set("drift", "This device vs server: " + (clockOffset >= 0 ? "-" : "+") + Math.abs(Math.round(clockOffset)) + " ms (±" + Math.ceil(rtt / 2) + ")");
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

class StatsHandler(http.server.BaseHTTPRequestHandler):
  def do_GET(self):
    if self.path == '/':
      self.send_body(WEB_PAGE.encode(), 'text/html; charset=utf-8')
    elif self.path == '/api/stats':
      with history_lock:
        history = list(offset_history)
      payload = dict(shared_stats, offset_history=history, server_time_ms=int(time.time() * 1000))
      self.send_body(json.dumps(payload).encode(), 'application/json')
    else:
      self.send_error(404)

  def send_body(self, body, content_type):
    self.send_response(200)
    self.send_header('Content-type', content_type)
    self.send_header('Content-Length', str(len(body)))
    self.send_header('Cache-Control', 'no-store')
    self.end_headers()
    self.wfile.write(body)

  def log_message(self, format, *args):
    # The page polls every 2s; don't flood the console with access logs
    pass

def run_web_server():
  PORT = 8080
  Handler = StatsHandler
  # Allow address reuse to avoid 'Address already in use' errors
  socketserver.TCPServer.allow_reuse_address = True
  with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Serving at port {PORT}")
    httpd.serve_forever()

def main():
  # Main program block
  
  # Start Web Server in a separate thread
  try:
    web_thread = threading.Thread(target=run_web_server, daemon=True)
    web_thread.start()
    print("Web server started on port 8080")
  except Exception as e:
    print(f"Failed to start web server: {e}")

  GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
  GPIO.setup(LCD_E, GPIO.OUT)  # E
  GPIO.setup(LCD_RS, GPIO.OUT) # RS
  GPIO.setup(LCD_D4, GPIO.OUT) # DB4
  GPIO.setup(LCD_D5, GPIO.OUT) # DB5
  GPIO.setup(LCD_D6, GPIO.OUT) # DB6
  GPIO.setup(LCD_D7, GPIO.OUT) # DB7

  # Initialise display
  lcd_init()

  # Startup message
  lcd_string("GPS Disciplined",LCD_LINE_1,2)
  lcd_string("NTP Time Server",LCD_LINE_2,2)
  lcd_string("v1.0 28/12/23",LCD_LINE_3,2)
  lcd_string("Andrew L. Weekes",LCD_LINE_4,2)
  time.sleep(2) # 2 second delay

  # Time to display each stats page (seconds)
  stats_delay = 3
  # Time to display time page (seconds)
  time_cycles = 10
  # Number of GPS data chronyPages
  gpsPages = 2
  # Number of chronyStats chronyPages
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

def displayTime(cycles):
  
  blank_display()
  
  # Initialise cycl count
  x=1

  # Print page title
  lcd_string("Date and Time",LCD_LINE_1,2)
  lcd_string("--------------------",LCD_LINE_2,2)

  #Initialise time and date stringa
  timestr = ""
  datestr = ""

  while x < cycles:
    #Get current date and time
    now = datetime.now()
    shared_stats["last_updated"] = now.strftime("%Y-%m-%d %H:%M:%S")

    #Update only if changed
    if datestr != now.strftime("%b %d, %Y"):
      datestr = now.strftime("%b %d, %Y")
      print(datestr)
      lcd_string(datestr,LCD_LINE_3,2)

    if timestr != now.strftime("%H:%M:%S"):
      timestr = now.strftime("%H:%M:%S")
      print(timestr)
      lcd_string(timestr,LCD_LINE_4,2)

    x += 1
    time.sleep(1)

def get_mem_data():
  try:
    mem = subprocess.check_output("free", shell=True, text=True)
    memData = mem.split()
    
    memTotal = memData[8]
    memUsed = memData[9]
    memFree = memData[10]
    
    shared_stats["memory"] = {
      "total": memTotal,
      "used": memUsed,
      "free": memFree
    }
    
    return ("Total: " + memTotal), ("Used: " + memUsed), ("Free: " + memFree)
  except Exception as e:
    print(f"Error getting memory stats: {e}")
    return None, None, None

def displayMemData(delay):

  #Get memory useage stats
  memTotal, memUsed, memFree = get_mem_data()
  
  if memTotal:
    blank_display()
    print ("Memory Stats")
    lcd_string("Memory Stats",LCD_LINE_1,2)
    print (memTotal)
    lcd_string(memTotal,LCD_LINE_2,1)
    print (memUsed)
    lcd_string(memUsed,LCD_LINE_3,1)
    print (memFree)
    lcd_string(memFree,LCD_LINE_4,1)
  else:
    print("Error getting memory stats")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*   Memory stats   *",LCD_LINE_2,2)
    lcd_string("*      error       *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)

  time.sleep(delay)

def get_network_data():
  try:
    #Get address and port data
    addr = subprocess.check_output("ip add show dev eth0", shell=True, text=True)
    #Get link stats
    link = subprocess.check_output("ip -s link show eth0", shell=True, text=True)
    
    #Split data into list elements
    addrData = addr.split()
    linkData = link.split()

    port_val = addrData[1]
    state_val = addrData[8]
    # Handle cases where IP might be missing or different index
    # The original code used hardcoded indices which is risky, keeping similar logic but being careful
    try:
       ip_val = addrData[18].split("/")[0]
    except IndexError:
       ip_val = "N/A"
       
    mac_val = addrData[14].replace(":","")
    
    mtu_val = linkData[4]
    rx_val = linkData[26]
    tx_val = linkData[39]

    shared_stats["network"] = {
      "port": port_val,
      "state": state_val,
      "ip": ip_val,
      "mac": mac_val,
      "mtu": mtu_val,
      "rx": rx_val,
      "tx": tx_val
    }

    port = ("Port: " + port_val)
    state = ("Network status: " + state_val)
    ip = ("IP: " + ip_val)
    mac = ("MAC: " + mac_val)

    mtu = ("MTU: " + mtu_val)
    rx = ("RX: " + rx_val)
    tx = ("TX: " + tx_val)
    
    return port, state, ip, mac, mtu, rx, tx
  except Exception as e:
    print(f"Network error: {e}")
    return None

def displayNetworkData(page, delay):
  
  #Get ip address
  data = get_network_data()
  
  if data:
    port, state, ip, mac, mtu, rx, tx = data
    
    #Print Data
    if page ==1:
      #Page 1
      blank_display()
      print (port)
      lcd_string(port,LCD_LINE_1,1)
      print (state)
      lcd_string(state,LCD_LINE_2,1)
      print (ip)
      lcd_string(ip,LCD_LINE_3,1)
      print (mac)
      lcd_string(mac,LCD_LINE_4,1)

    elif page == 2:
      #Page 2
      blank_display()
      print (mtu)
      lcd_string(mtu,LCD_LINE_1,1)
      print (rx)
      lcd_string(rx,LCD_LINE_2,1)
      print (tx)
      lcd_string(tx,LCD_LINE_3,1)

  else:
    print("Network error")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*  Network error   *",LCD_LINE_2,2)
    lcd_string("*                  *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)
    
  time.sleep(delay)

def format_gps_coord(val, dir, is_lat, degree_symbol=chr(223)):
  # Determine the degree part length based on lat/lon
  # Lat: DDMM.MMMM -> 2 digits for degrees
  # Lon: DDDMM.MMMM -> 3 digits for degrees
  deg_len = 2 if is_lat else 3

  try:
    # Check if val is a string or float, handle accordingly.
    # pynmea2 usually provides these as strings or floats depending on the exact property accessed.
    # The user code was using `msg.lat` (string) + `msg.lat_dir` (string).
    # We need to parse the raw string "DDMM.MMMM"

    # Ensure it's a string for slicing
    val_str = str(val).strip()
    if not val_str: return "N/A"

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

def get_gps_data():
  try:
    output = subprocess.check_output("gpspipe -r -x 3", shell=True, text=True)
    
    #Split output into list elements
    gpsPipe = output.split()
    
    # Get $GPGGA messages for location data
    pattern = "$GPGGA"
    gpgga = [x for x in gpsPipe if x.startswith(pattern)]
    
    if not gpgga:
       raise ValueError("No GPGGA message found")

    # pynmea2 can't process list, so pick first available item in list
    msg = pynmea2.parse(gpgga[0])

    sat_val = msg.num_sats
    lat_val = format_gps_coord(msg.lat, msg.lat_dir, True)
    lon_val = format_gps_coord(msg.lon, msg.lon_dir, False)
    # Web page uses the real Unicode degree sign; the LCD uses its own ROM glyph (chr(223))
    lat_web = format_gps_coord(msg.lat, msg.lat_dir, True, degree_symbol="°")
    lon_web = format_gps_coord(msg.lon, msg.lon_dir, False, degree_symbol="°")
    alt_val = str(msg.altitude) + msg.altitude_units

    # Get $GPGSA message for fix and dilution of precision data
    pattern = "$GPGSA"
    gpgsa = [x for x in gpsPipe if x.startswith(pattern)]
    
    if not gpgsa:
      raise ValueError("No GPGSA message found")

    # pynmea2 can't process list, so pick first available item in list
    dop = pynmea2.parse(gpgsa[0])

    fix_mode = dop.mode_fix_type
    pdop_val = dop.pdop
    hdop_val = dop.hdop
    vdop_val = dop.vdop
    
    readable_fix = ("No fix", "2D Fix", "3D Fix")
    fix_text = readable_fix[int(fix_mode) -1]

    shared_stats["gps"] = {
      "satellites": sat_val,
      "latitude": lat_web,
      "longitude": lon_web,
      "altitude": alt_val,
      "fix": fix_text,
      "pdop": pdop_val,
      "hdop": hdop_val,
      "vdop": vdop_val
    }

    satellites = ("Satellites: " + sat_val)
    latitude = ("Lat: " + lat_val)
    longitude = ("Lon: " + lon_val)
    altitude = ("Alt: " + alt_val)
    
    fix = ("Fix: " + fix_text)
    pdop = ("PDOP: " + pdop_val)
    hdop = ("HDOP: " + hdop_val)
    vdop = ("VDOP: " + vdop_val)
    
    return satellites, latitude, longitude, altitude, fix, pdop, hdop, vdop
    
  except Exception as e:
    print(f"GPS error: {e}")
    return None

def displayGPSData(page, delay):
  
  data = get_gps_data()

  if data:
    satellites, latitude, longitude, altitude, fix, pdop, hdop, vdop = data

    if page == 1:
      #Page 1
      blank_display()
      print(satellites)
      print(latitude)
      print(longitude)
      print(altitude)
      lcd_string(satellites,LCD_LINE_3,1)
      lcd_string(latitude,LCD_LINE_1,1)
      lcd_string(longitude,LCD_LINE_2,1)
      lcd_string(altitude,LCD_LINE_4,1)

    elif page == 2:
      #Page 2
      blank_display()
      print(fix)
      print(pdop)
      print(hdop)
      print(vdop)
      lcd_string(fix,LCD_LINE_1,1)
      lcd_string(pdop,LCD_LINE_2,1)
      lcd_string(hdop,LCD_LINE_3,1)
      lcd_string(vdop,LCD_LINE_4,1)

    time.sleep(delay)
  
  else:
    print("gpspipe error or no gps data")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*  gpspipe error   *",LCD_LINE_2,2)
    lcd_string("*                  *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)
    time.sleep(delay)
 
def get_chrony_data():
  try:
    output = subprocess.check_output("chronyc tracking", shell=True, text=True)
    #Split chronyc output into element list
    chronyResult = output.split()
    
    systemTime_val = (chronyResult[20] + " " + "s " + chronyResult[22])
    lastOffset_val = (chronyResult[29] + " s")
    rmsOffset_val = (chronyResult[34] + " s")
    freq_val = (chronyResult[38] + " " + chronyResult[39] + " " + chronyResult[40])
    resFreq_val = (chronyResult[44] + " " + chronyResult[45])
    skew_val = (chronyResult[48] + " " + chronyResult[49])
    rootDly_val = (chronyResult[53] + " sec")
    rootDisp_val = (chronyResult[58] + " sec")
    upInt_val = (chronyResult[63] + " sec")
    lpStat_val = (chronyResult[68])

    with history_lock:
      offset_history.append([int(time.time() * 1000), float(chronyResult[29])])

    shared_stats["chrony"] = {
      "system_time": systemTime_val,
      "last_offset": lastOffset_val,
      "rms_offset": rmsOffset_val,
      "frequency": freq_val,
      "residual_frequency": resFreq_val,
      "skew": skew_val,
      "root_delay": rootDly_val,
      "root_dispersion": rootDisp_val,
      "update_interval": upInt_val,
      "leap_status": lpStat_val
    }

    return systemTime_val, lastOffset_val, rmsOffset_val, freq_val, resFreq_val, skew_val, rootDly_val, rootDisp_val, upInt_val, lpStat_val

  except Exception as e:
    print(f"Chrony error: {e}")
    return None

def displayChronyStats(page, delay):
  
  blank_display()

  data = get_chrony_data()
  if data:
    systemTime, lastOffset, rmsOffset, freq, resFreq, skew, rootDly, rootDisp, upInt, lpStat = data

    #Output stats to console and LCD
    if page == 1:
      #Page 1
      blank_display()
      print("System Time:")
      print(systemTime)
      print ("Last Offset:")
      print(lastOffset)
      lcd_string("System Time: ",LCD_LINE_1,1)
      lcd_string(systemTime,LCD_LINE_2,3)
      lcd_string("Last offset: ",LCD_LINE_3,1)
      lcd_string(lastOffset,LCD_LINE_4,3)

    elif page == 2:
      #Page 2
      blank_display()
      print("RMS offset: ")
      print(rmsOffset)
      print("Frequency: ")
      print(freq)
      lcd_string("RMS offset: ",LCD_LINE_1,1)
      lcd_string(rmsOffset, LCD_LINE_2,3)
      lcd_string("Frequency: ",LCD_LINE_3,1)
      lcd_string(freq,LCD_LINE_4,3)

    elif page == 3:
      #Page 3
      blank_display()
      print("Residual frequency: ")
      print(resFreq)
      print("Skew: ")
      print(skew)
      lcd_string("Residual frequency: ",LCD_LINE_1,1)
      lcd_string(resFreq,LCD_LINE_2,3)
      lcd_string("Skew: ",LCD_LINE_3,1)
      lcd_string(skew,LCD_LINE_4,3)

    elif page == 4:
      #Page 4
      blank_display()
      print("Root delay: ")
      print(rootDly)
      print("Root dispersion: ")
      print(rootDisp)
      lcd_string("Root delay: ",LCD_LINE_1,1)
      lcd_string(rootDly,LCD_LINE_2,3)
      lcd_string("Root dispersion: ",LCD_LINE_3,1)
      lcd_string(rootDisp,LCD_LINE_4,3)

    elif page == 5:
      #Page 5
      blank_display()
      print("Update Interval:")
      print(upInt)
      print("Leap Status: ")
      print(lpStat)
      lcd_string("Update Interval:",LCD_LINE_1,1)
      lcd_string(upInt,LCD_LINE_2,3)
      lcd_string("Leap Year Status: ",LCD_LINE_3,1)
      lcd_string(lpStat,LCD_LINE_4,3)

    time.sleep(delay)
  
  else:
    print("chronyc error or no data")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*  chronyc error   *",LCD_LINE_2,2)
    lcd_string("*                  *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)
    time.sleep(delay)

def blank_display():
  # Blank display
  lcd_byte(0x01, LCD_CMD)

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line,style):
  # Send string to display
  # style=1 Left justified
  # style=2 Centred
  # style=3 Right justified

  if style==1:
    message = message.ljust(LCD_WIDTH," ")
  elif style==2:
    message = message.center(LCD_WIDTH," ")
  elif style==3:
    message = message.rjust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*  System Stopped  *",LCD_LINE_2,2)
    lcd_string("*     Goodbye!     *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)

    GPIO.cleanup()
