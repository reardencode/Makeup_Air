#ifndef __SDP_H
#define __SDP_H 1

#include <Arduino.h> // TODO: Used for Serial in sdp.cpp - report error better?
#include <Wire.h>

#define SDP_CRC_INIT 0xFF
#define SDP_CRC_POLY 0x31
#define SDP_ADDRESS 0x25
#define SDP_START_COMMAND 0x3615 
#define SDP_READ_SIZE 6 // 2 bytes pressure, 1 byte crc, 2 bytes temperature, 1 byte crc
#define SDP_SCALE 240.0
#define SDP_TEMP_SCALE 200.0

void sdp_setup();
bool sdp_read(double *pdiff, double *temp);

#endif
