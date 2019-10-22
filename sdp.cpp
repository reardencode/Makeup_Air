#include "sdp.h"

void send_sdp_command(uint16_t command) {
  Wire.beginTransmission(SDP_ADDRESS);
  Wire.write((command >> 8) & 0xFF);
  Wire.write(command & 0xFF);
  Wire.endTransmission();
}

bool sdp_checked_read(double *result, double scale) {
  uint8_t crc = 0xFF;
  uint8_t data[2];
  int16_t raw_result;

  for (uint8_t i = 0; i < 2; i++) {
    data[i] = Wire.read();
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  if (crc == Wire.read()) {
    raw_result = (data[0] << 8) + data[1];
    *result = raw_result / scale;
    return true;
  } else {
    Serial.println("CRC mismatch reading from SDP, value not modified");
    return false;
  }
}

void sdp_setup() {
  Wire.begin();
  send_sdp_command(SDP_START_COMMAND);
}

bool sdp_read(double *pdiff, double *temp) {
  Wire.requestFrom(SDP_ADDRESS, SDP_READ_SIZE);
  bool rc = true;
  rc &= sdp_checked_read(pdiff, SDP_SCALE);
  rc &= sdp_checked_read(temp, SDP_TEMP_SCALE);
  return rc;
}
