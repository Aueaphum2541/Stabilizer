#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "motionbit.h"

#define PWM_FREQ 50
#define ANALOG_PIN 32
#define SECOND 0.001

// ------------ WiFi & MQTT Config ------------
const char* ssid = "EZEKIEL";
const char* password = "late1978";
const char* mqtt_server = "broker.emqx.io";

const char* NAME = "stabilizer"; // เปลี่ยนชื่ออุปกรณ์
const char* pair_topic = "stabilizer/server/pair";
const char* dev_topic = "stabilizer/server/device";
char data_topic[100];
char cmd_topic[100];
char start_topic[100];

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastSent = 0;

// ------------ Map pressure (mmHg) to PWM (0–4095) ------------
int mapPressureToPWM(int pressure_mmHg) {
  const float max_mmHg = 100.0;
  int pwm = (int)((pressure_mmHg / max_mmHg) * 4095.0);
  pwm = constrain(pwm, 0, 4095);
  return pwm;
}

// ------------ Motor Control ------------
void motorControl(int* pwm_value) {
  MotionBit.Motor.run(M4, ROTATE_F, *pwm_value);
  delay(10);
  MotionBit.Motor.run(M2, ROTATE_F, *pwm_value / 2);
  delay(10 / SECOND);
  MotionBit.Motor.brake(M2);
  MotionBit.Motor.brake(M4);

  MotionBit.Motor.run(M3, ROTATE_F, *pwm_value * 25);
  delay(10);
  MotionBit.Motor.run(M2, ROTATE_F, *pwm_value / 2);
  delay(13 / SECOND);
  MotionBit.Motor.brake(M2);
  MotionBit.Motor.brake(M3);

  String json = String("{\"start\":") + true + "}";
  client.publish(start_topic, json.c_str());
  Serial.println(start_topic);
  Serial.println("[DATA] START COMMAND: " + json);
}

// ------------ Get Minimum Pressure per Mode ------------
int getMinimumPressure(const String& mode) {
  if (mode == "Cervical Flexion") return 20;
  else if (mode == "Thoracic Extension") return 30;
  else if (mode == "Thoracic Side-Shift to Right" || mode == "Thoracic Rotation to Right") return 40;
  else if (mode == "Lumbar Flexion") return 40;
  else if (mode == "Lumbar Extension") return 30;
  else if (mode == "Lumbar Side-Shift to Right" || mode == "Lumbar Rotation to Right") return 40;
  else if (mode == "Custom Mode") return 30;
  else return -1;
}

// ------------ Run Mode ------------
void runMode(const String& mode, int pressure_mmHg) {
  if (pressure_mmHg <= 0) {
    Serial.println("[ERROR] Invalid pressure");
    return;
  }

  int pwm_value = mapPressureToPWM(pressure_mmHg);
  Serial.printf("[MODE] %s | Pressure: %d mmHg → PWM: %d\n", mode.c_str(), pressure_mmHg, pwm_value);
  
  motorControl(&pwm_value);
}

// ------------ MQTT Callback ------------
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String msg = String((char*)payload);

  if (String(topic) == pair_topic && msg.indexOf("Pair Request") >= 0) {
    String response = String("{\"device\":\"") + NAME + "\"}";
    client.publish(dev_topic, response.c_str());
    Serial.println("[PAIR] Responded to Pair Request");
  }
  else if (String(topic) == cmd_topic) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("[ERROR] JSON parse failed: ");
      Serial.println(error.c_str());
      return;
    }

    String mode = doc["mode"] | "";
    int pressure = doc["pressure"] | -1;

    if (pressure <= 0) {
      pressure = getMinimumPressure(mode);
      Serial.printf("[AUTO] Using default pressure = %d mmHg\n", pressure);
    }

    runMode(mode, pressure);
  }
}

// ------------ WiFi Setup ------------
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected");
}

// ------------ MQTT Setup ------------
void setup_mqtt() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(pair_topic);
  client.subscribe(cmd_topic);
}

// ------------ MQTT Reconnect ------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("[MQTT] Connecting...");
    if (client.connect(NAME)) {
      Serial.println("connected");
      client.subscribe(pair_topic);
      client.subscribe(cmd_topic);
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

// ------------ Setup ------------
void setup() {
  Serial.begin(115200);
  MotionBit.begin(PWM_FREQ);

  snprintf(data_topic, sizeof(data_topic), "stabilizer/server/data/%s", NAME);
  snprintf(cmd_topic, sizeof(cmd_topic), "stabilizer/server/cmd/%s", NAME);
  snprintf(start_topic, sizeof(start_topic), "stabilizer/server/start/%s", NAME);

  setup_wifi();
  setup_mqtt();
}

// ------------ Main Loop ------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (millis() - lastSent > 1000) {
    uint16_t pressure = analogRead(ANALOG_PIN);
    String json = String("{\"pressure\":") + pressure + "}";
    client.publish(data_topic, json.c_str());
    Serial.println("[DATA] Published: " + json);
    lastSent = millis();


    // static bool state = false;

    // if (state) {
    //   MotionBit.Motor.run(M4, ROTATE_F, 200); // stretch
    //   delay(10);
    //   MotionBit.Motor.run(M2, ROTATE_F, 100); // stretch

    //   Serial.println("M2, M4");
    //   delay(2000);
    //   MotionBit.Motor.brake(M2);
    //   MotionBit.Motor.brake(M4);
    //   state = false;
    //   delay(4000);
    // }
    // else {
    //   MotionBit.Motor.run(M3, ROTATE_F, 200); // release
    //   delay(10);
    //   MotionBit.Motor.run(M2, ROTATE_F, 100); // stretch

    //   Serial.println("M2, M3");
    //   delay(2000);
    //   MotionBit.Motor.brake(M2);
    //   MotionBit.Motor.brake(M3);
    //   state = true;
    // }
  }
}
 