#!/usr/bin/python
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
import os
import pynmea2
import RPi.GPIO as GPIO
import time
from datetime import datetime
import subprocess
from gps import *
import re
import threading
import http.server
import socketserver
import json
import logging

# Shared dictionary to store statistics for the web page
shared_stats = {
    "last_updated": "",
    "memory": {},
    "network": {},
    "gps": {},
    "chrony": {}
}

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

class StatsHandler(http.server.BaseHTTPRequestHandler):
  def do_GET(self):
    if self.path == '/':
      self.send_response(200)
      self.send_header('Content-type', 'text/html')
      self.end_headers()
      
      # Simple HTML Template
      html = """
<!DOCTYPE html>
<html>
<head>
    <title>Pi NTP Status</title>
    <meta http-equiv="refresh" content="5">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: monospace; background: #222; color: #0f0; padding: 20px; max-width: 800px; margin: auto; }
        .box { border: 1px solid #444; padding: 15px; margin-bottom: 20px; border-radius: 5px; background: #111; }
        h1 { text-align: center; color: #fff; }
        h2 { margin-top: 0; color: #ddd; border-bottom: 1px solid #444; padding-bottom: 5px; }
        .item { margin: 5px 0; display: flex; justify-content: space-between; }
        .label { color: #888; }
        .value { color: #0f0; font-weight: bold; }
        .error { color: #f00; }
    </style>
</head>
<body>
    <h1>Pi NTP Server Status</h1>
    <div style="text-align:center; color:#888; margin-bottom: 20px;">Last Updated: %s</div>
""" % shared_stats["last_updated"]

      # Memory Section
      mem = shared_stats.get("memory", {})
      html += """
    <div class="box">
        <h2>Memory</h2>
        <div class="item"><span class="label">Total</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Used</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Free</span><span class="value">%s</span></div>
    </div>
""" % (mem.get("total", "N/A"), mem.get("used", "N/A"), mem.get("free", "N/A"))

      # Network Section
      net = shared_stats.get("network", {})
      html += """
    <div class="box">
        <h2>Network</h2>
        <div class="item"><span class="label">IP</span><span class="value">%s</span></div>
        <div class="item"><span class="label">State</span><span class="value">%s</span></div>
        <div class="item"><span class="label">MAC</span><span class="value">%s</span></div>
        <div class="item"><span class="label">RX</span><span class="value">%s</span></div>
        <div class="item"><span class="label">TX</span><span class="value">%s</span></div>
    </div>
""" % (net.get("ip", "N/A"), net.get("state", "N/A"), net.get("mac", "N/A"), net.get("rx", "N/A"), net.get("tx", "N/A"))

      # GPS Section
      gps = shared_stats.get("gps", {})
      html += """
    <div class="box">
        <h2>GPS</h2>
        <div class="item"><span class="label">Fix</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Satellites</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Lat</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Lon</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Alt</span><span class="value">%s</span></div>
    </div>
""" % (gps.get("fix", "N/A"), gps.get("satellites", "N/A"), gps.get("latitude", "N/A"), gps.get("longitude", "N/A"), gps.get("altitude", "N/A"))

      # Chrony Section
      chrony = shared_stats.get("chrony", {})
      html += """
    <div class="box">
        <h2>Chrony</h2>
        <div class="item"><span class="label">System Time</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Last Offset</span><span class="value">%s</span></div>
        <div class="item"><span class="label">RMS Offset</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Frequency</span><span class="value">%s</span></div>
        <div class="item"><span class="label">Root Delay</span><span class="value">%s</span></div>
    </div>
</body>
</html>
""" % (chrony.get("system_time", "N/A"), chrony.get("last_offset", "N/A"), chrony.get("rms_offset", "N/A"), chrony.get("frequency", "N/A"), chrony.get("root_delay", "N/A"))

      self.wfile.write(html.encode())

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
       ip_val = addrData[18].strip("/24)")
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
    lat_val = msg.lat + msg.lat_dir
    lon_val = msg.lon + msg.lon_dir
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
      "latitude": lat_val,
      "longitude": lon_val,
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
    print("chronyc error or no gps data")
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
