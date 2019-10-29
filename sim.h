#ifndef __SIM_H
#define __SIM_H 1

class Simulator {
  public:

  Simulator();

  void sdp_read(double*, double*);
  void toggle();
  bool is_active();
  int set_hood(uint8_t);
  uint8_t set_hood_delay(uint8_t);
  uint8_t set_fan_delay(uint8_t);
  uint8_t set_sdp_delay(uint8_t);
  int get_hood_cfm();
  int get_fan_cfm();

  private:
  uint8_t sdp_delay;
  int* sdp_pa;
  uint8_t sdp_pa_i;

  int vortex_cfm();
  uint8_t fan_delay;
  int* fan_cfm;
  uint8_t fan_cfm_i;
  bool fan_on;

  int typhoon_cfm();
  uint8_t hood_delay;
  int* hood_cfm;
  uint8_t hood_cfm_i;
  uint8_t hood;

  bool active;
  unsigned long last_millis;
};
#endif
