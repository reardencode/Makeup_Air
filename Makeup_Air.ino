#include <cfloat>
#include <algorithm>

#include <WiFi.h>
#include <AsyncUDP.h>

#include "secrets.h"
#include "OTA.h"
#include "fan_adc.h"
#include "sdp.h"
#include "PID.h"

#define SIMULATABLE 1
#ifdef SIMULATABLE
#include "sim.h"
Simulator sim;
#endif

TickType_t first_tick;

#define LOOP_TICKS (25 / portTICK_PERIOD_MS) // .1 second

#define LOCK_TICKS (1000 / portTICK_PERIOD_MS) // 1 second
SemaphoreHandle_t lock = NULL;

#define DEBUG_TICKS (100 / portTICK_PERIOD_MS) // .1 second
#define MAX_DEBUG_LEN 256
char debug_buf[MAX_DEBUG_LEN];

#define OTA_TICKS (1000 / portTICK_PERIOD_MS) // 1 second

#define UDP_PORT 1982
#define MAX_UDP_DATA 128
char udp_data[MAX_UDP_DATA];
AsyncUDP udp;

// These thresholds should turn the heat on before too much unheated air would
// potentially reduce the furnace intake air temp below design minimum (55f),
// and turn the heat off before the fan could possibly go off.
#define HEAT_ON_FAN_THRESHOLD 64 // ~25% voltage, ~20% airflow?, ~200CFM?
#define HEAT_OFF_FAN_THRESHOLD 16 // ~12.5% voltage, min airflow, ~30CFM?
// This needs adjusting depending on temp source, and calibration.
// If below 15C at differential pressure sensor, activate inbound air heating
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

SDP sdp810(SDP8X0_I2C_ADDRESS, Wire);
double sdp = 0;
double sdp_temp = 0;

#define PID_SETPOINT 0.2 // This determines how often the fan runs to bring in fresh air (range ~0-5)
#define PID_MIN -DBL_MIN // Negative value drops the output to significantly <2V to ensure fan off
#define PID_MAX 255 // 8-pit PWM
#define PID_ERR_MIN -PID_MAX // Allow sufficient negative windup to provide occasional pulses of fresh air
#define PID_ERR_MAX PID_MAX // Prevent excessive windup past the control range
// Tuning values tuned with simulation. See sim.{h,cpp} for more simulation details.
// Takes about 1 minute to get within 1Pa of target, before oscillating in that range.
#define PID_KP 2.5
#define PID_KI 0.2
#define PID_KD 0.4
double pid_out = PID_MIN;
PID fan_pid(PID_KP, PID_KI, PID_KD, &pid_out, PID_SETPOINT, PID_MIN, PID_MAX, PID_ERR_MIN, PID_ERR_MAX);

void setup() {
  Serial.begin(500000);
  while (!Serial); // Wait for Serial port to connect

  Serial.println("Initializing task timer...");
  first_tick = xTaskGetTickCount();

  Serial.println("Initializing WiFi...");
  WiFi.disconnect();
  WiFi.onEvent(wifi_event_handler);
  WiFi.begin(WIFI_SSID, WPA2_PSK);

  Serial.println("Initializing OTA task..."); // Do this early to give some chance of recovery
  xTaskCreate(ota_task_fn, "OTA", 8192, NULL, 2, NULL);

  Serial.println("Initializing I2C...");
  Wire.begin(-1, -1, 400000L); // Default pins, 400kHz

  Serial.println("Initializing SDP...");
  while (!sdp810.begin_pdiff(true)) { // Do relatively early so it can warm up
    Serial.println("Failed to begin averaged differential pressure reading");
    delay(10000);
  }

  Serial.println("Initializing pins...");
  pinMode(FAN_OUT, OUTPUT);
  pinMode(FAN_SW, OUTPUT);
  pinMode(SW1, OUTPUT);
  pinMode(SW2, OUTPUT);

  Serial.println("Initializing fan ADC...");
  fan_adc_setup(FAN_IN);

  Serial.println("Initializing mutex...");
  lock = xSemaphoreCreateMutex();
  if (lock == NULL) {
    Serial.println("Failed to create mutex");
    for (;;) {
      delay(10000);
    }
  }

  Serial.println("Initializing UDP...");
  if (udp.listen(UDP_PORT)) {
    udp.onPacket(udp_ui); // Requires lock
    Serial.println("UDP listening");
  } else {
    Serial.println("UDP failed");
  }

  Serial.println("Initializing debug format task...");
  xTaskCreate(debug_task_fn, "Debug", 8192, NULL, 1, NULL); // Requires lock

  Serial.println("Initializing main task...");
  xTaskCreate(main_task_fn, "Main", 8192, NULL, 0, NULL); // Requires lock

  Serial.println("Initialized.");
}

#define UDP_UI_WHITESPACE " \f\n\r\t\v"
#define UDP_UI_COMMAND_SEP ":=" UDP_UI_WHITESPACE

void udp_ui(AsyncUDPPacket packet) {
  Serial.printf("UDP Packet From: %s:%d, Length: %d\n",
                packet.remoteIP().toString().c_str(), packet.remotePort(), packet.length());

  if (packet.length() + 1 > MAX_UDP_DATA) {
    packet.println("Data too big");
    Serial.println("Data too big");
    return;
  }
  memcpy(udp_data, packet.data(), packet.length());
  udp_data[packet.length()] = 0;

  char* command = &udp_data[strspn(udp_data, UDP_UI_WHITESPACE)];
  char* command_end = &command[strcspn(command, UDP_UI_COMMAND_SEP)];
  char *value_str = &command_end[strspn(command_end, UDP_UI_COMMAND_SEP)];
  *command_end = 0;
  value_str[strcspn(value_str, UDP_UI_WHITESPACE)] = 0;
  double value = atof(value_str);
  Serial.printf("UDP Command: %s, Value: %f\n", command, value);

  if (!xSemaphoreTake(lock, LOCK_TICKS)) {
    Serial.println("UDP UI failed to get lock!?");
    return;
  }
  if (strcmp("fan", command) == 0) {
    fan_override = value;
    if (fan_override >= -1) {
      fan_pid.manual();
    }
    packet.printf("Overriding fan to %d\n", fan_override);
  } else if (strcmp("heat", command) == 0) {
    heat_override = value;
    packet.printf("Overriding heat to %d\n", heat_override);
  } else if (strncmp("pid", command, 3) == 0)  {
    switch (command[3]) {
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
        packet.printf("Invalid PID parameter: %c\n", command[3]);
    }
  } else if (strcmp("target", command) == 0) {
    fan_pid.set_setpoint(value);
    packet.printf("Set target differential pressure to %fPa\n", fan_pid.get_setpoint());
  } else if (strcmp("status", command) == 0) {
    packet.print(debug_buf);
#ifdef SIMULATABLE
  } else if (strcmp("simulate", command) == 0) {
    sim.toggle();
    packet.printf("Simulation: %d\n", sim.is_active());
  } else if (strcmp("hood", command) == 0) {
    packet.printf("Simulating hood to %dCFM\n", sim.set_hood(std::max(0, (int)value)));
  } else if (strcmp("sdp_delay", command) == 0) {
    packet.printf("Set sdp delay %d loops\n", sim.set_sdp_delay(std::max(0, (int)value)));
  } else if (strcmp("fan_delay", command) == 0) {
    packet.printf("Set fan delay %d loops\n", sim.set_fan_delay(std::max(0, (int)value)));
  } else if (strcmp("hood_delay", command) == 0) {
    packet.printf("Set hood delay %d loops\n", sim.set_hood_delay(std::max(0, (int)value)));
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
  } else if (pid_out <= HEAT_OFF_FAN_THRESHOLD && digitalRead(HEAT_SW)) {
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

void ota_task_fn(void* parameters) {
  TickType_t last_loop_time = first_tick;
  ota_setup();
  for (;;) {
    vTaskDelayUntil(&last_loop_time, OTA_TICKS);
    ota_loop();
  }
}

void debug_task_fn(void* parameters) {
  TickType_t last_loop_time = first_tick;
  for (;;) {
    vTaskDelayUntil(&last_loop_time, DEBUG_TICKS);
    Serial.print(WiFi.localIP());
    Serial.print(": ");
    if (xSemaphoreTake(lock, LOCK_TICKS)) {
      fan_volts = fan_adc_read();
      snprintf(debug_buf, MAX_DEBUG_LEN,
               "Target: %.2fPa, SDP: %7.2fPa, %.1fC, PID: %.1f,%.1f|%.1f,%.1f -> %.1f, Fan(%d): %5.2fV, Heat(%d)\n",
               fan_pid.get_setpoint(), sdp, sdp_temp,
               fan_pid.get_kp(), fan_pid.get_ki(), fan_pid.get_error_sum(), fan_pid.get_kd(),
               pid_out, fan_on, fan_volts, heat_on);
      Serial.print(debug_buf);
      xSemaphoreGive(lock);
    } else {
      Serial.println("Debug task failed to get lock!?");
    }
  }
}

void main_task_fn(void* parameters) {
  TickType_t last_loop_time = first_tick;
  for (;;) {
    vTaskDelayUntil(&last_loop_time, LOOP_TICKS);
    if (xSemaphoreTake(lock, LOCK_TICKS)) {
#ifdef SIMULATABLE
      if (sim.is_active()) {
        sim.sdp_read(&sdp, &sdp_temp);
      } else
#endif
      if (!sdp810.read(&sdp, &sdp_temp)) {
        Serial.println(sdp810.get_last_error());
      }
      set_fan();
      set_heat();
      xSemaphoreGive(lock);
    } else {
      Serial.println("Loop failed to get lock!?");
    }
  }
}

void loop() {
  vTaskDelete(NULL);
}

void wifi_event_handler(system_event_id_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.printf("WiFi connected! IP address: %s\n", WiFi.localIP().toString().c_str());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      WiFi.reconnect();
      break;
  }
}
