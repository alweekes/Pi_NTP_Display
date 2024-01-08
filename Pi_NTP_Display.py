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

def main():
  # Main program block
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

def displayMemData(page, delay):

  #Get memory useage stats
  try:
    mem = subprocess.check_output("free", shell=True, text=True)
    
    memData = mem.split()
    
    for (i, item) in enumerate (memData, start=1):
      print (i, item)

  except Exception as e:
    print(e)
    print("Error getting memory stats")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*   Memory stats   *",LCD_LINE_2,2)
    lcd_string("*      error       *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2

def displayNetworkData(page, delay):
  
  #Get ip address
  try:
    #Get address and port data
    addr = subprocess.check_output("ip add show dev eth0", shell=True, text=True)
    
    #Get link stats
    link = subprocess.check_output("ip -s link show eth0", shell=True, text=True)
        
    #Split data into list elements
    addrData = addr.split()
    linkData = link.split()

    #Get address data from list items
    port = ("Port: " + addrData[1])
    state = ("Network status: " + addrData[8])
    ip = ("IP: " + addrData[18].strip("/24)"))
    mac = ("MAC: " + addrData[14].replace(":",""))

    #Get link stats from list items
    mtu = ("MTU: " + linkData[4])
    rx = ("RX: " + linkData[26])
    tx = ("TX: " + linkData[39])

    
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

  except Exception as e:
    print(e)
    print("Network error")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*  Network error   *",LCD_LINE_2,2)
    lcd_string("*                  *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)
    
  time.sleep(delay)
  
  
def displayGPSData(page, delay):

  # Get output of gpspipe, output is native GPSD JSON sentences + NMEA messages
  # -r = raw NMEA senteces, -x causes gpspipe to close after no. seconds
  try:
    output = subprocess.check_output("gpspipe -r -x 3", shell=True, text=True)
    
    #Split output into list elements
    gpsPipe = output.split()
    
    # Get $GPGGA messages for location data
    pattern = "$GPGGA"
    gpgga = [x for x in gpsPipe if x.startswith(pattern)]
    
    # pynmea2 can't process list, so pick first available item in list
    msg = pynmea2.parse(gpgga[0])

    #Get data from $GPGGA message (number of satellites, lat, long, height)
    satellites = ("Satellites: " + msg.num_sats)
    latitude = ("Lat: " + msg.lat + msg.lat_dir)
    longitude = ("Lon: " + msg.lon + msg.lon_dir)
    altitude = ("Alt: " + str(msg.altitude) + msg.altitude_units)

    # Get $GPGSA message for fix and dilution of precision data
    pattern = "$GPGSA"
    gpgsa = [x for x in gpsPipe if x.startswith(pattern)]
    
    # pynmea2 can't process list, so pick first available item in list
    dop = pynmea2.parse(gpgsa[0])

    #Get data from $GPGSA message for fix status and DOP
    readable_fix = ("No fix", "2D Fix", "3D Fix")
    fix = ("Fix: " + readable_fix[int(dop.mode_fix_type) -1])
    pdop = ("PDOP: " + dop.pdop)
    hdop = ("HDOP: " + dop.hdop)
    vdop = ("VDOP: " + dop.vdop)

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
 
  except Exception as e:
    print(e)
    print("gpspipe error or no gps data")
    blank_display()
    lcd_string("********************",LCD_LINE_1,2)
    lcd_string("*  gpspipe error   *",LCD_LINE_2,2)
    lcd_string("*                  *",LCD_LINE_3,2)
    lcd_string("********************",LCD_LINE_4,2)
    time.sleep(delay)


def displayChronyStats(page, delay):
  
  blank_display()

  try:
    output = subprocess.check_output("chronyc tracking", shell=True, text=True)
    
    #Split chronyc output into element list
    chronyResult = output.split()
    
    systemTime = (chronyResult[20] + " " + "s " + chronyResult[22])
    lastOffset = (chronyResult[29] + " s")
    
    rmsOffset = (chronyResult[34] + " s")
    freq = (chronyResult[38] + " " + chronyResult[39] + " " + chronyResult[40])
    
    resFreq = (chronyResult[44] + " " + chronyResult[45])
    skew = (chronyResult[48] + " " + chronyResult[49])

    rootDly = (chronyResult[53] + " sec")
    rootDisp = (chronyResult[58] + " sec")

    upInt = (chronyResult[63] + " sec")
    lpStat = (chronyResult[68])

    


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

  
  except Exception as e:
    print(e)
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
