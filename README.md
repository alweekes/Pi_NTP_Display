v1.1
Added web page for display of stats.
v1.0
Rebuild of my GPS disciplned Raspberry Pi NTP server using DietPi as a lightweight minimal OS ideally suited for a Pi 1 Model B.
Pi uses a USB GPS dongle,  modified to extract the PPS signal (fed into the pi on GPIO18) and ultimately fed to GPSD/chrony for high accuracy.
To make it more useful I added a 20x4 LCD display to use it as a clock and display some stats on rotation.
LCD code is based on a simplified version of this code: https://www.raspberrypi-spy.co.uk/2012/08/20x4-lcd-module-control-using-python/, I've just removed the backlight toggling.
The display cycles through Time + Date and two pages of GPS stats, from gpspipe and 5 pages of chrony stats, gathered from chronyc
I'm sure there are more efficient ways of doing this, but overall I'm pleased with the result.
