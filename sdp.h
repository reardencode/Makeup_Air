#ifndef __SDP_H
#define __SDP_H 1

#include <Wire.h>

#define SDP_CRC_INIT 0xFF
#define SDP_CRC_POLY 0x31

#define SDP8X0_I2C_ADDRESS 0x25
#define SDP8X1_I2C_ADDRESS 0x26

#define SDP_BEGIN_MASS_FLOW_AVERAGED ((SensorData){0x3603})
#define SDP_BEGIN_MASS_FLOW ((SensorData){0x3608})
#define SDP_BEGIN_PDIFF_AVERAGED ((SensorData){0x3615})
#define SDP_BEGIN_PDIFF ((SensorData){0x361E})
#define SDP_TRIGGER_MASS_FLOW ((SensorData){0x3624})
#define SDP_TRIGGER_MASS_FLOW_STRETCH ((SensorData){0x3726})
#define SDP_TRIGGER_PDIFF ((SensorData){0x362F})
#define SDP_TRIGGER_PDIFF_STRETCH ((SensorData){0x372D})

#define SDP_VALUE_SIZE 3 // msb, lsb, crc
#define SDP8XX_TEMP_SCALE 200.0

#define SDP_ERROR_BUF_SIZE 256

union SensorData {
  int16_t value;
  uint8_t bytes[2];
};

class SDP {
  SDP() = delete;

  public:

  SDP(uint8_t, TwoWire&);

  bool begin_mass_flow(bool);
  bool begin_pdiff(bool);

  bool trigger_mass_flow(bool);
  bool trigger_pdiff(bool);

  bool read(double *, double* = NULL);

  char* get_last_error();

  /* TODO:
  reset();
  sleep();
  wake();
  */

  private:

  bool _send_command(SensorData);
  bool _read_value(SensorData&, char*);
  TwoWire& _i2c;
  uint8_t _address;
  SensorData _scale;
  char _last_error[SDP_ERROR_BUF_SIZE];
};

#endif
