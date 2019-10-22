#ifndef __PID_H
#define __PID_H 1

class PID {

  public:

  PID(double, double, double,
      double*, double,
      double = 0, double = UINT8_MAX,
      double = -UINT8_MAX, double = UINT8_MAX);
  PID(double, double, double, double,
      double*, double,
      double = 0, double = UINT8_MAX,
      double = -UINT8_MAX, double = UINT8_MAX);

  void compute(double);
  void manual();
  
  void set_setpoint(double);
  double get_setpoint();

  void set_tunings(double, double, double);
  void set_tunings(double, double, double, double);
  double get_kp();
  double get_kpe();
  double get_kpm();
  double get_ki();
  double get_kd();

  void set_output_limits(double, double);
  double get_out_min();
  double get_out_max();

  void set_error_limits(double, double);
  double get_err_min();
  double get_err_max();

  double get_error();
  double get_error_sum();
  double get_de_dt();

  private:

  double* _output;
  double _setpoint;

  double _kpe;
  double _kpm;
  double _ki;
  double _kd;

  double _out_min, _out_max;
  double _err_min, _err_max;
  
  double _error_sum;
  double _error;
  double _de_dt;

  unsigned long _last_time;

  bool _initialized;
};

#endif
