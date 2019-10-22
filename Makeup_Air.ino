#include "secrets.h"
#include "OTA.h"
#include "fan_adc.h"
#include "sdp.h"
#include "PID.h"

#include <WiFi.h>
#include <AsyncUDP.h>
#include <cfloat>

#define SIMULATABLE 1
#ifdef SIMULATABLE

#include <cmath>

#define SIM_HOUSE_VOLUME 40000
#define ATM_PA 101325

const int sim_hood_cfm[] = {0, -50, -100, -200, -300, -450, -650, -850};
bool simulate = false;
unsigned long last_sim_millis;
bool sim_fan_on;
int sim_hood;

double sim_vortex_cfm(double sim_fan_volts) {
  if (sim_fan_on && sim_fan_volts <= 2) {
    sim_fan_on = false;
  } else if (!sim_fan_on && sim_fan_volts >= 2.5) {
    sim_fan_on = true;
  }
  if (!sim_fan_on) {
    return 0;
  }
  return sim_fan_volts * 100;
}

double sim_exfiltration_cfm(double sim_sdp) {
  return -0.86 * copysign(exp(.03 * abs(sim_sdp)), sim_sdp) * SIM_HOUSE_VOLUME / 60;
}

void sim_sdp(double *sdp, double *sdp_temp) {
  unsigned long now = millis();
  *sdp_temp = 25; // 25C is standard simulation temp, right?!
//  Serial.printf("Simulation Vortex CFM: %f, Hood CFM: %f, Exfiltration CFM: %f\n",
//                sim_vortex_cfm(fan_adc_read()), sim_hood_cfm[sim_hood], sim_exfiltration_cfm(*sdp));
  // Use a fresh fan voltage every simulation
  double cfm_change = sim_vortex_cfm(fan_adc_read()) + sim_hood_cfm[sim_hood] + sim_exfiltration_cfm(*sdp);
  double pa_change_per_minute = (ATM_PA + *sdp) * (cfm_change / SIM_HOUSE_VOLUME);
  *sdp = *sdp + (pa_change_per_minute / (60 * 1000)) * (now - last_sim_millis);
  last_sim_millis = now;
//  Serial.printf("Simulation CFM Change: %f, Sim Pa Change: %f\n", cfm_change, pa_change_per_minute);
//  delay(1000);
}

void sim_toggle() {
  simulate = !simulate;
  if (simulate) {
    last_sim_millis = millis();
    sim_hood = 0;
    sim_fan_on = false;
  }
}

#endif

#define LOOP_TICKS (100 / portTICK_PERIOD_MS) // .1 second
TickType_t last_loop_time;

#define LOCK_TICKS (1000 / portTICK_PERIOD_MS) // 1 second
SemaphoreHandle_t lock = NULL;

TickType_t last_debug_time;
#define DEBUG_TICKS (100 / portTICK_PERIOD_MS) // .1 second
#define MAX_DEBUG_LEN 255
char debug_buf[MAX_DEBUG_LEN];

#define UDP_PORT 1982
#define MAX_UDP_DATA 128
char udp_data[MAX_UDP_DATA];
AsyncUDP udp;

#define HEAT_ON_FAN_THRESHOLD 64 // ~25% fan speed, not too much unheated air for furnace
// This needs adjusting depending on temp source, and calibration currently saying that
// if it's below 15C at the differential pressure sensor, activate inbound air heating
#define HEAT_ON_TEMP_THRESHOLD 15
int heat_override = -1;
bool heat_on = false;

#define SW1 18
#define SW2 10

#define HEAT_SW SW1
#define FAN_SW 4
#define FAN_OUT 25
#define FAN_IN 37

int fan_override = -2;
bool fan_on = false;
double fan_volts = 0;

double sdp = 0;
double sdp_temp = 0;

// Target +2 Pascal indoor pressure
// This will determine how often the fan runs to bring in fresh air
#define PID_SETPOINT 2
#define PID_MIN -DBL_MIN
#define PID_MAX 255
#define PID_ERR_MIN -PID_MAX
#define PID_ERR_MAX PID_MAX * 2
#define PID_KP 1
#define PID_KI 0.1
#define PID_KD 0.1
double pid_out = PID_MIN;
PID fan_pid(PID_KP, PID_KI, PID_KD, &pid_out, PID_SETPOINT, PID_MIN, PID_MAX);

void setup() {
  Serial.begin(500000);
  while (!Serial); // Wait for Serial port to connect

  Serial.println("Initializing SDP..."); // Do this before WiFi to give it warmup time (20ms min)
  sdp_setup();

  Serial.println("Initializing WiFi...");
  WiFi.disconnect();
  WiFi.onEvent(wifi_event_handler);
  WiFi.begin(WIFI_SSID, WPA2_PSK);

  Serial.println("Initializing UDP...");
  if (udp.listen(UDP_PORT)) {
    udp.onPacket(udp_ui);
    Serial.println("UDP listening");
  } else {
    Serial.println("UDP failed");
  }

  Serial.println("Initializing pins...");
  pinMode(FAN_OUT, OUTPUT);
  pinMode(FAN_SW, OUTPUT);
  pinMode(SW1, OUTPUT);
  pinMode(SW2, OUTPUT);

  Serial.println("Initializing fan ADC...");
  fan_adc_setup(FAN_IN);

  Serial.println("Initializing OTA...");
  ota_setup();

  Serial.println("Initializing task timer...");
  last_loop_time = xTaskGetTickCount();

  Serial.println("Initializing mutex...");
  lock = xSemaphoreCreateMutex();

  Serial.println("Initializing debug format task...");
  if (lock == NULL) {
    Serial.println("Failed to create mutex");
    for (;;) {
      ota_loop(); // Do the OTA loop for potential recovery
      delay(1000);
    }
  }
  xTaskCreate(debug_task_fn, "DebugTask", 10000, NULL, 1, NULL);

  Serial.println("Initialized.");
}

void udp_ui(AsyncUDPPacket packet) {
  Serial.print("UDP Packet From: ");
  Serial.print(packet.remoteIP());
  Serial.print(":");
  Serial.print(packet.remotePort());
  Serial.print(", Length: ");
  Serial.print(packet.length());

  if (packet.length() + 1 > MAX_UDP_DATA) {
    packet.println("Data too big");
    Serial.println(", Data too big");
    return;
  }
  memcpy(udp_data, packet.data(), packet.length());
  udp_data[packet.length()] = 0;
  Serial.print(", Data: ");
  Serial.println(udp_data);
  if (!xSemaphoreTake(lock, LOCK_TICKS)) {
    Serial.println("UDP UI failed to get lock!?");
    return;
  }
  if (packet.length() > 4 && strncmp("fan:", udp_data, 4) == 0) {
    fan_override = atoi(udp_data + 4);
    if (fan_override >= -1) {
      fan_pid.manual();
    }
    packet.printf("Overriding fan to %d\n", fan_override);
  } else if (packet.length() > 5 && strncmp("heat:", udp_data, 5) == 0) {
    heat_override = atoi(udp_data + 5);
    packet.printf("Overriding heat to %d\n", heat_override);
  } else if (packet.length() > 5 && strncmp("pid", udp_data, 3) == 0)  {
    double value = atof(udp_data + 5);
    switch (udp_data[3]) {
      case 'p':
        fan_pid.set_tunings(value, fan_pid.get_ki(), fan_pid.get_kd());
        packet.printf("Set PID proportional constant to %f\n", value);
        break;
      case 'i':
        fan_pid.set_tunings(fan_pid.get_kp(), value, fan_pid.get_kd());
        packet.printf("Set PID integral constant to %f\n", value);
        break;
      case 'd':
        fan_pid.set_tunings(fan_pid.get_kp(), fan_pid.get_ki(), value);
        packet.printf("Set PID derivative constant to %f\n", value);
        break;
      default:
        packet.printf("Invalid PID parameter: %c\n", udp_data[3]);
    }
  } else if (packet.length() > 7 && strncmp("target:", udp_data, 7) == 0) {
    fan_pid.set_setpoint(atof(udp_data + 7));
    packet.printf("Set target differential pressure to %fPa\n", fan_pid.get_setpoint());
  } else if (packet.length() >= 6 && strncmp("status", udp_data, 6) == 0) {
    packet.print(debug_buf);
#ifdef SIMULATABLE
  } else if (packet.length() >= 8 && strncmp("simulate", udp_data, 8) == 0) {
    sim_toggle();
    packet.printf("Simulation: %d\n", simulate);
  } else if (packet.length() > 5 && strncmp("hood:", udp_data, 5) == 0) {
    sim_hood = atoi(udp_data + 5);
    packet.printf("Simulating hood to %d\n", sim_hood);
#endif
  } else {
    packet.printf("No matching command of length %d\n", packet.length());
  }
  xSemaphoreGive(lock);
}

void set_heat() {
  if (heat_override >= 0) {
    heat_on = heat_override > 0;
  } else {
    heat_on = sdp_temp < HEAT_ON_TEMP_THRESHOLD;
  }
  if (pid_out >= HEAT_ON_FAN_THRESHOLD && fan_on && heat_on && !digitalRead(HEAT_SW)) {
    Serial.println("Activating heater");
    digitalWrite(HEAT_SW, HIGH);
  } else if (!fan_on && digitalRead(HEAT_SW)) {
    Serial.println("Deactivating heater");
    digitalWrite(HEAT_SW, LOW);
  }
}

void set_fan() {
  if (fan_override >= -1) {
    pid_out = fan_override;
    fan_on = fan_override >= 0;
  } else {
    fan_pid.compute(sdp);
    fan_on = pid_out >= 0;
  }
  dacWrite(FAN_OUT, max((int)round(pid_out), 0));

  if (fan_on && !digitalRead(FAN_SW)) {
    Serial.println("Activating fan");
    digitalWrite(FAN_SW, HIGH);
  } else if (!fan_on && digitalRead(FAN_SW)) {
    Serial.println("Deactivating fan");
    digitalWrite(FAN_SW, LOW);
  }
}

void debug_task_fn(void *parameters) {
  last_debug_time = xTaskGetTickCount();
  for (;;) {
    Serial.print(WiFi.localIP());
    Serial.print(": ");
    if (xSemaphoreTake(lock, LOCK_TICKS)) {
      fan_volts = fan_adc_read();
      snprintf(debug_buf, MAX_DEBUG_LEN,
               "Target: %4.2fPa, SDP: %7.2fPa, %4.1fC, PID: %3.1f,%3.1f,%3.1f -> %5.1f, Fan(%d): %5.2fV, Heat(%d)\n",
               fan_pid.get_setpoint(), sdp, sdp_temp,
               fan_pid.get_kp(), fan_pid.get_ki(), fan_pid.get_kd(),
               pid_out, fan_on, fan_volts, heat_on);
      Serial.print(debug_buf);
      xSemaphoreGive(lock);
    } else {
      Serial.println("Debug task failed to get lock!?");
    }

    vTaskDelayUntil(&last_debug_time, DEBUG_TICKS);
  }
}

void loop() {
  if (xSemaphoreTake(lock, LOCK_TICKS)) {
#ifdef SIMULATABLE
    if (simulate) {
      sim_sdp(&sdp, &sdp_temp);
    } else
#endif
    sdp_read(&sdp, &sdp_temp);
    set_fan();
    set_heat();
    xSemaphoreGive(lock);
  } else {
    Serial.println("Loop failed to get lock!?");
  }

  ota_loop();
  vTaskDelayUntil(&last_loop_time, LOOP_TICKS);
}

void wifi_event_handler(system_event_id_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      WiFi.reconnect();
      break;
  }
}
