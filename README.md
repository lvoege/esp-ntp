
*** NOTE I don't really have an idea of what I'm doing here ***

This is a barebones stratum 1 NTP server using an ESP32-C3 microcontroller, a
ublox neo-6 GPS receiver and a W5500-mini ethernet adapter using nothing but
the standard ESP IDF environment. The code boots the ethernet interface, waits
for a GPS fix, gets the date and time from it and starts listening for the PPS
signal from then on.

If the PPS signal comes in more than one-ish seconds after the last one the
signal is considered lost. So too if the main loop detects it's been more than
one-ish seconds since the last PPS. When the signal is lost this listens to
GPS again until that gives a good signal.

The main loop then waits for a packet or a timeout. If a timeout it checks if
it still has signal and, if not, tries to reacquire. If it's a packet it fills
the relevant bits in as quick as it can and sends it back as the reply.

That's all there is. Settings as to what GPIO pin is connected to what are
near the top of the code as is the hostname it should be telling the dhcp
server. Adapt that, fill in a random unicast mac address, build it, flash it
and you have a stratum 1 ntp server.

*** NOTE ***

You're probably better off with something that can run a real OS with a real
ntpd implementation. This is just an experiment. That said, both chrony and
ntpsec appear to find this an acceptable ticker. A Raspberry Pi 4 running
ntpsec with the same GPS on the same network has ntpsec prefer this over the
pi, though chrony picks the pi.
