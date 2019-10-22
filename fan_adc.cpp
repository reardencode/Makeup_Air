#include "fan_adc.h"

esp_adc_cal_characteristics_t *fan_adc_cal = new esp_adc_cal_characteristics_t;

uint8_t fan_pin;
uint8_t fan_reading_i = 0;
int16_t fan_readings[FAN_READING_N] = {};

void fan_adc_setup(uint8_t pin) {
  fan_pin = pin;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, fan_adc_cal);
  for (int i = 0; i < FAN_READING_N; i++) {
    fan_readings[i] = -1;
  }
  analogSetPinAttenuation(fan_pin, FAN_IN_ATTENUATION);
}

double fan_adc_read() {
  fan_readings[fan_reading_i] = analogRead(fan_pin);
  fan_reading_i = (fan_reading_i + 1) % FAN_READING_N;
  uint32_t fan_volts_sum = 0;
  int i = 0;
  for (; i < FAN_READING_N; i++) {
    if (fan_readings[i] < 0) {
      break; // Not all readings initialized, use average of initialized readings.
    }
    fan_volts_sum += fan_readings[i];
  }
  return esp_adc_cal_raw_to_voltage(round((double)fan_volts_sum / i), fan_adc_cal) / FAN_VOLTS_DIVIDER;
}
