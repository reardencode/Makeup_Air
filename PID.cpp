/**********************************************************************************************
 * PID Library
 * by Brandon Black <freedom@reardencode.com>
 * 
 * Based loosely on Arduino PID Library
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "Arduino.h"

#include "PID.h"

// TODO: Think about these min/max vars, do they make sense?
PID::PID(double kp, double ki, double kd,
         double *output, double setpoint,
         double out_min, double out_max,
         double err_min, double err_max)
  :PID::PID(kp, 0, ki, kd, output, setpoint, out_min, out_max, err_min, err_max)
{
}

PID::PID(double kpe, double kpm, double ki, double kd,
         double *output, double setpoint,
         double out_min, double out_max,
         double err_min, double err_max)
{
  _output = output;
  PID::set_setpoint(setpoint);
  PID::set_tunings(kpe, kpm, ki, kd);
  if (out_min >= out_max) { // Revert to the range of the Arduino DAC
    out_min = 0;
    out_max = UINT8_MAX;
  }
  PID::set_output_limits(out_min, out_max);
  if (err_min >= err_max) { // Revert to full control authority of the Arduino DAC
    err_min = -UINT8_MAX;
    err_max = UINT8_MAX;
  }
  PID::set_error_limits(err_min, err_max);

  _initialized = false;
}

// Call compute at _regular_ intervals with the current input value to update output
void PID::compute(double input) {
  unsigned long now = millis();
  double last_error = _error;
  _error = _setpoint - input;

  if (_initialized) {
    double seconds = (now - _last_time) / 1000.0;

    // KI * Area (_last_time, last_error) (now, _error)
    _error_sum += _ki * seconds * (last_error + _error) / 2;
    _error_sum = min(_err_max, max(_error_sum, _err_min));

    // KD * Slope (_last_time, last_error), (now, _error)
    _de_dt = _kd * (_error - last_error) / seconds;

    // KPe for Proportional on Error, KPm for Proportional on Measurement
    *_output = _kpe * _error + _kpm * input + _error_sum + _de_dt;
    *_output = min(_out_max, max(*_output, _out_min));
  } else {
    // Assume that the system was operating at steady state, so the output would be entirely I
    _error_sum = *_output;
    _initialized = true;
  }

  _last_time = now;
}

void PID::set_setpoint(double setpoint) {
  _setpoint = setpoint;
}

void PID::set_tunings(double kp, double ki, double kd) {
  PID::set_tunings(kp, _kpm, ki, kd);
}

void PID::set_tunings(double kpe, double kpm, double ki, double kd) {
  _kpe = kpe;
  _kpm = kpm;
  if (ki == 0) {
    _error_sum = 0;
  }
  _ki = ki;
  _kd = kd;
}

void PID::set_output_limits(double out_min, double out_max) {
  if (out_min >= out_max) return;
  _out_min = out_min;
  _out_max = out_max;
}

void PID::set_error_limits(double err_min, double err_max) {
  if (err_min >= err_max) return;
  _err_min = err_min;
  _err_max = err_max;
}

void PID::manual() {
  _initialized = false;
}

double PID::get_setpoint() { return _setpoint; }
double PID::get_kp() { return _kpe; }
double PID::get_kpe() { return _kpe; }
double PID::get_kpm() { return _kpm; }
double PID::get_ki() { return _ki; }
double PID::get_kd() { return _kd; }
double PID::get_out_min() { return _out_min; }
double PID::get_out_max() { return _out_max; }
double PID::get_err_min() { return _err_min; }
double PID::get_err_max() { return _err_max; }

double PID::get_error() { return _error; }
double PID::get_error_sum() { return _error_sum; }
double PID::get_de_dt() { return _de_dt; }
