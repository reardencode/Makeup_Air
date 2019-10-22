#ifndef __FAN_ADC_H
#define __FAN_ADC_H 1

#include <esp32-hal.h>
#include <esp_adc_cal.h>

#define FAN_READING_N 1 // Turns out that with a capacitor over the input pin, it's pretty smooth
#define FAN_VOLTS_DIVIDER (1000*1.5/(1.5+10)) // mV -> V, Resistive Voltage divider from 10V
#define FAN_IN_ATTENUATION ADC_2_5db

void fan_adc_setup(uint8_t pin);
double fan_adc_read();

#endif
