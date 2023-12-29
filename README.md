Rebuild of my GPS disciplned Raspberry Pi NTP server using DietPi as a lightweight minimal OS ideally suited for a Pi 1 Model B.
Pi uses a USB GPS dongle,  modified to extract the PPS signal and feed to chrony for high accuracy.
Thought it would be useful to add an LCD display to use it as a clock and display some stats on rotation.
Display cycles through Time / Date and two pages of GPS stats, from gpspipe and 5 pages of chrony stats, gathered from chronyc
I'm sure there are more efficient ways of doing this, but overall I'm pleased with the result.
