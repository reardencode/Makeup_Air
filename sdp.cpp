#include "sdp.h"

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define ENDIAN_AWARE_LOOP(var, base, bytes) for (int var = (base); var < (bytes); var++)
#else // LITTLE_ENDIAN
#define ENDIAN_AWARE_LOOP(var, base, bytes) for (int var = (bytes) - 1; var >= (base); var--)
#endif

SDP::SDP(uint8_t address, TwoWire& i2c)
  : _address(address), _i2c(i2c), _scale({0})
{}

bool SDP::_send_command(SensorData command) {
  _i2c.beginTransmission(_address);
  // The sensor seems to accept the command bytes in either order, but let's do it right
  ENDIAN_AWARE_LOOP(i, 0, 2) {
    _i2c.write(command.bytes[i]);
  }
  return _i2c.endTransmission() == I2C_ERROR_ACK;
}

bool SDP::_read_value(SensorData& result, char* field_name) {
  uint8_t crc = SDP_CRC_INIT;

  ENDIAN_AWARE_LOOP(i, 0, 2) {
    int rc = _i2c.read();
    if (rc < 0) {
      snprintf(_last_error, SDP_ERROR_BUF_SIZE, "Failed to read %d byte of %s", 2-i, field_name);
      return false;
    }
    result.bytes[i] = rc & 0xFF;
    crc ^= rc & 0xFF;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ SDP_CRC_POLY;
      else
        crc <<= 1;
    }
  }
  if (crc != _i2c.read()) {
    snprintf(_last_error, SDP_ERROR_BUF_SIZE, "CRC check failed for %s", field_name);
    return false;
  }
  return true;
}

bool SDP::begin_mass_flow(bool averaged) {
  return _send_command(averaged ? SDP_BEGIN_MASS_FLOW_AVERAGED : SDP_BEGIN_MASS_FLOW);
}

bool SDP::begin_pdiff(bool averaged) {
  return _send_command(averaged ? SDP_BEGIN_PDIFF_AVERAGED : SDP_BEGIN_PDIFF);
}

bool SDP::trigger_mass_flow(bool averaged) {
  return _send_command(averaged ? SDP_TRIGGER_MASS_FLOW_STRETCH : SDP_TRIGGER_MASS_FLOW);
}

bool SDP::trigger_pdiff(bool averaged) {
  return _send_command(averaged ? SDP_TRIGGER_PDIFF_STRETCH : SDP_TRIGGER_PDIFF);
}

bool SDP::read(double *pdiff, double *temp) {
  uint8_t read_size;
  if (_scale.value == 0) {
    read_size = SDP_VALUE_SIZE * 3;
  } else if (temp != NULL) {
    read_size = SDP_VALUE_SIZE * 2;
  } else if (pdiff != NULL) {
    read_size = SDP_VALUE_SIZE;
  } else {
    return true;
  }
  uint8_t bytes_read = _i2c.requestFrom(_address, read_size);
  if (bytes_read != read_size) {
    snprintf(_last_error, SDP_ERROR_BUF_SIZE, "Expected %d bytes, read %d", read_size, bytes_read);
    return false;
  }
  SensorData scaled_pdiff;
  SensorData scaled_temp;
  bool rc = true;
  rc &= _read_value(scaled_pdiff, "Differential Pressure");
  if (temp != NULL || _scale.value == 0) {
    rc &= _read_value(scaled_temp, "Temperature");
  }
  if (_scale.value == 0) {
    rc &= _read_value(_scale, "Scale");
  }
  if (rc) {
    *pdiff = scaled_pdiff.value / ((double)_scale.value);
    if (temp != NULL) {
      *temp = scaled_temp.value / SDP8XX_TEMP_SCALE;
    }
  }
  return rc;
}

char* SDP::get_last_error() {
  return _last_error;
}
