
*** NOTE I don't really have an idea of what I'm doing here ***

This is a barebones stratum 1 NTP server using an ESP32-C3 microcontroller, a
ublox neo-6 GPS receiver and a W5500-mini ethernet adapter using nothing but
the standard ESP IDF environment. The code boots the ethernet interface, waits
for a GPS fix, gets the date and time from it and starts listening for the PPS
signal from then (so it doesn't look at the GPS signal any more). on the PPS
pulse it increments its notion of GPS time by one second and saves the current
microseconds since boot.

the main task then opens a udp socket on port 123 and does a blocking read.
when it gets a packet it quickly fills in the bits and sends it back out then
does it again, in an infinite loop. no threading or delegating requests to
tasks or anything of the sort. get packet, process packet, reply, etc.

that's all there is. toggles as to what GPIO pin is connected to what are near
the top of the code as is the hostname it should be telling the dhcp server.
adapt that, fill in a random unicast mac address, build it, flash it and you
have a stratum 1 ntp server.

*** NOTE ***

You're probably better off with something that can run a real OS with a real
ntpd implementation. This is just an experiment.
