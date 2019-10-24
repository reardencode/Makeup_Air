This really should be several separate Arduino libraries, and then a small
project, but I haven't split it up.

Included:
* Simple library for interacting with a Sensirion SDP8xx differential pressure sensor.
* PID library loosely based on https://github.com/br3ttb/Arduino-PID-Library/
* Library for interacting with ESP32 ADC for (fairly) accurate voltage readings
* Simulation code for testing
* OTA update
* Actual control loop

Missing:
* secrets.h for OTA password hash, and WiFi SSID/PSK
