#include <algorithm>
#include <cstdint>
#include <cmath>

#include "sim.h"
#include "sdp.h"
#include "fan_adc.h"

#define FAN_BOOST_DELAY 10
#define DEFAULT_FAN_DELAY (FAN_BOOST_DELAY + 10)
#define DEFAULT_HOOD_DELAY 25
#define DEFAULT_SDP_DELAY 50

#define SIM_HOUSE_VOLUME 40000
#define ATM_PA 101325
#define VORTEX_MIN_CFM 30
#define FAN_BOOST_CFM 150
#define VORTEX_MAX_CFM 1032
#define VORTEX_CFM_RANGE (VORTEX_MAX_CFM - VORTEX_MIN_CFM)
#define FAN_ON_VOLTS 2.55 // Based on real world testing
#define FAN_OFF_VOLTS 2.05 // Based on real world testing
#define FAN_MAX_VOLTS 10
#define FAN_VOLTS_RANGE (FAN_MAX_VOLTS - FAN_ON_VOLTS)
#define SIM_SDP_SCALE 240.0

const int typhoon_cfm_table[] = {0, -50, -150, -300, -450, -650, -850};

void fill_arr(int arr[], uint8_t* arr_i, int delay, int phase1_delay, int start, int mid, int end) {
  for (int i = 1; i < phase1_delay; i++) {
    arr[(*arr_i + i) % delay] = start + (mid - start) * i / phase1_delay;
  }
  for (int i = phase1_delay; i <= delay; i++) {
    arr[(*arr_i + i) % delay] = mid + (end - mid) * (i - phase1_delay) / (delay - phase1_delay);
  }
}

void fill_arr(int arr[], uint8_t* arr_i, int delay, int start, int end) {
  fill_arr(arr, arr_i, delay, 1, -1, start, end);
}

int Simulator::typhoon_cfm() {
  hood_cfm_i = (hood_cfm_i + 1) % hood_delay;
  int delayed_hood_cfm = hood_cfm[hood_cfm_i];
  hood_cfm[hood_cfm_i] = typhoon_cfm_table[hood];
  return delayed_hood_cfm;
}

int calc_vortex_cfm(double fan_volts) {
  if (fan_volts > FAN_OFF_VOLTS && fan_volts < FAN_ON_VOLTS) {
    return VORTEX_MIN_CFM;
  }
  fan_volts = fmin(fan_volts, 10.0); // Controller stops at 10
  return VORTEX_MIN_CFM + (fan_volts - FAN_ON_VOLTS) * VORTEX_CFM_RANGE / FAN_VOLTS_RANGE;
}

int Simulator::vortex_cfm() {
  double fan_volts = fan_adc_read();
  fan_cfm_i = (fan_cfm_i + 1) % fan_delay;
  int delayed_fan_cfm = fan_cfm[fan_cfm_i];

  if (fan_on && fan_volts <= FAN_OFF_VOLTS) {
    fan_on = false;
    fill_arr(fan_cfm, &fan_cfm_i, fan_delay, delayed_fan_cfm, 0);
  } else if (!fan_on && fan_volts >= FAN_ON_VOLTS) {
    fan_on = true;
    fill_arr(fan_cfm, &fan_cfm_i,
             fan_delay, FAN_BOOST_DELAY, delayed_fan_cfm, FAN_BOOST_CFM, calc_vortex_cfm(fan_volts));
  } else if (fan_on) {
    fan_cfm[fan_cfm_i] = calc_vortex_cfm(fan_volts);
  } else {
    fan_cfm[fan_cfm_i] = 0;
  }

  return delayed_fan_cfm;
}

double exfiltration_cfm(double sdp) {
  // Tried to estimate using an exponential model for exfiltration vs.
  // pressure, but online calculators and papers tell me that it's a square
  // root relationship between pressure and volume flow rate for a given
  // equivalent leakage area
  // But the sqrt seems like it might only be correct with a single large hole?
  //return -282.843 * copysign(sqrt(abs(sdp)), sdp);
  return -0.268 * copysign(exp(.05 * abs(sdp)) - 1, sdp) * SIM_HOUSE_VOLUME / 60;
  //return -0.86 * copysign(exp(.03 * abs(sdp)) - 1, sdp) * SIM_HOUSE_VOLUME / 60;
}

void Simulator::sdp_read(double *sdp_out, double *sdp_temp_out) {
  unsigned long now = millis();
  *sdp_temp_out = 25; // 25C is standard simulation temp, right?!

  int recent_sdp = sdp_pa[sdp_pa_i];
  sdp_pa_i = (sdp_pa_i + 1) % sdp_delay;
  *sdp_out = sdp_pa[sdp_pa_i] / SIM_SDP_SCALE;

  double cfm_change = vortex_cfm() + typhoon_cfm() + exfiltration_cfm(recent_sdp / SIM_SDP_SCALE);
  double pa_change_per_minute = (ATM_PA + recent_sdp / SIM_SDP_SCALE) * (cfm_change / SIM_HOUSE_VOLUME);
  int new_sdp = recent_sdp + round((pa_change_per_minute * SIM_SDP_SCALE / (60 * 1000)) * (now - last_millis));
  sdp_pa[sdp_pa_i] = std::max(INT16_MIN, std::min(INT16_MAX, new_sdp));
  last_millis = now;
}

uint8_t set_delay(int** arr, uint8_t* arr_i, uint8_t *delay, uint8_t new_delay) {
  int cur_arr = (*arr)[*arr_i];
  int final_arr = (*arr)[(*arr_i + *delay) % *delay];
  int* new_arr = (int*)calloc(new_delay, sizeof(int));
  if (new_arr == NULL) return *delay;
  *arr_i = 0;
  fill_arr(new_arr, arr_i, new_delay, cur_arr, final_arr);
  *delay = new_delay;
  *arr = new_arr;
  return *delay;
}

uint8_t Simulator::set_sdp_delay(uint8_t delay) {
  return set_delay(&sdp_pa, &sdp_pa_i, &sdp_delay, delay);
}

uint8_t Simulator::set_fan_delay(uint8_t delay) {
  return set_delay(&fan_cfm, &fan_cfm_i, &fan_delay, delay);
}

uint8_t Simulator::set_hood_delay(uint8_t delay) {
  return set_delay(&hood_cfm, &hood_cfm_i, &hood_delay, delay);
}

void Simulator::toggle() {
  if (!active) {
    if (NULL == (hood_cfm = (int*)calloc(hood_delay, sizeof(int)))) goto cleanup;
    if (NULL == (fan_cfm = (int*)calloc(fan_delay, sizeof(int)))) goto cleanup;
    if (NULL == (sdp_pa = (int*)calloc(sdp_delay, sizeof(int)))) goto cleanup;
    last_millis = millis();
    hood = 0;
    fan_on = false;
    active = true;
    return;
  }
cleanup:
  active = false;
  free(sdp_pa);
  sdp_pa = NULL;
  free(hood_cfm);
  hood_cfm = NULL;
  free(fan_cfm);
  fan_cfm = NULL;
}

bool Simulator::is_active() {
  return active;
}

int Simulator::set_hood(uint8_t val) {
  if (val <= 6 && val != hood) {
    int cur_cfm = hood_cfm[hood_cfm_i];
    int new_cfm = typhoon_cfm_table[val];
    int cfm_change = new_cfm - cur_cfm;
    fill_arr(hood_cfm, &hood_cfm_i, hood_delay, cur_cfm, new_cfm);
    hood = val;
  }
  return typhoon_cfm_table[hood];
}

int Simulator::get_hood_cfm() {
  return hood_cfm[hood_cfm_i];
}

int Simulator::get_fan_cfm() {
  return fan_cfm[fan_cfm_i];
}

Simulator::Simulator() :
  fan_delay(DEFAULT_FAN_DELAY), fan_cfm_i(0),
  hood_delay(DEFAULT_HOOD_DELAY), hood_cfm_i(0),
  sdp_delay(DEFAULT_SDP_DELAY), sdp_pa_i(0),
  active(false)
{}
