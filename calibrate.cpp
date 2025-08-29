#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>   // ArduinoJson v7 API
#include <Preferences.h>
#include "motionbit.h"
#include <math.h>

// ================== User Config ==================
#define PWM_FREQ 50

// ใช้ ADC1 (32~39)
const int ANALOG_PINS[] = {32, 33};
constexpr int NUM_CH = sizeof(ANALOG_PINS)/sizeof(ANALOG_PINS[0]);

// ADC settings (ESP32)
#define ADC_BIT_WIDTH 12          // 0..4095
#define ADC_ATTN      ADC_11db    // ~0..3.3V

// Guard & smoothing
const int   MIN_VALID_RAW = 1;      // raw <= 0 => ไม่มีสัญญาณ -> mmHg=0
const float MAX_STEP_MMHG = 8.0f;   // จำกัดความชันต่อเฟรม
const int   STABLE_TRIES  = 3;
const int   STABLE_N      = 15;
const int   STABLE_MAD_TH = 18;

// mmHg bounds
const float MMHG_MIN = 0.0f;
const float MMHG_MAX = 100.0f;

// ------------ WiFi & MQTT Config ------------
const char* ssid        = "EZEKIEL";
const char* password    = "late1978";
const char* mqtt_server = "broker.emqx.io";

const char* NAME       = "stabilizer";
const char* pair_topic = "stabilizer/server/pair";
const char* dev_topic  = "stabilizer/server/device";
char data_topic[100];
char cmd_topic[100];
char start_topic[100];

WiFiClient espClient;
PubSubClient client(espClient);
Preferences prefs;

unsigned long lastSent = 0;

// ============== Linear fallback calib (gain/offset) ==============
struct Calib { float gain; float offset; };
Calib cal[NUM_CH];

void saveCalToNVS(){
  prefs.begin("stabilizer", false);
  for(int ch=0; ch<NUM_CH; ch++){
    char k1[16], k2[16];
    snprintf(k1,sizeof(k1),"gain%d",ch);
    snprintf(k2,sizeof(k2),"off%d",ch);
    prefs.putFloat(k1, cal[ch].gain);
    prefs.putFloat(k2, cal[ch].offset);
  }
  prefs.end();
}
void loadCalFromNVS(){
  prefs.begin("stabilizer", true);
  for(int ch=0; ch<NUM_CH; ch++){
    char k1[16], k2[16];
    snprintf(k1,sizeof(k1),"gain%d",ch);
    snprintf(k2,sizeof(k2),"off%d",ch);
    cal[ch].gain   = prefs.getFloat(k1, 100.0f/4095.0f); // default linear 0..4095 -> 0..100 mmHg
    cal[ch].offset = prefs.getFloat(k2, 0.0f);
  }
  prefs.end();
}

// ============== LUT mapping (piecewise-linear) ===================
#define MAX_LUT_POINTS 32
struct Lut {
  bool     enabled = true;
  int      n = 0;
  uint16_t x[MAX_LUT_POINTS]; // ADC
  float    y[MAX_LUT_POINTS]; // mmHg
};
Lut lut[NUM_CH];

// ============== Two-point (zero + ref) calib ====================
struct Cal2 {
  bool     active = false;
  int      ch     = 0;
  uint16_t raw0   = 0;   // raw @ 0 mmHg
  bool     haveZero = false;
} cal2;

// ============== State (for smoothing) ============================
float last_mmHg[NUM_CH] = {0};
bool  has_last[NUM_CH]  = {false};

// ============== Small utils =====================================
static void isort_u16(uint16_t *a, int n){
  for(int i=1;i<n;i++){ uint16_t k=a[i]; int j=i-1; while(j>=0 && a[j]>k){a[j+1]=a[j]; j--;} a[j+1]=k; }
}
static uint16_t median_u16(uint16_t *a, int n){
  isort_u16(a, n); return a[n/2];
}
uint16_t readStableRaw(int pin){
  uint16_t samp[STABLE_N], dev[STABLE_N];
  for(int t=0;t<STABLE_TRIES;t++){
    for(int i=0;i<STABLE_N;i++){ samp[i]=analogRead(pin); delayMicroseconds(250); }
    uint16_t med=median_u16(samp,STABLE_N);
    for(int i=0;i<STABLE_N;i++){ dev[i]=(uint16_t)abs((int)samp[i]-(int)med); }
    uint16_t mad=median_u16(dev,STABLE_N);
    if (mad<=STABLE_MAD_TH) return med;
    delay(2);
  }
  return median_u16(samp,STABLE_N);
}

// ============== LUT helpers (ArduinoJson v7 safe) ===============
float lutInterp(int ch, uint16_t raw){
  const Lut& L = lut[ch];
  if (!L.enabled || L.n < 2) return NAN;

  if (raw <= L.x[0]) {
    float dx = (float)(L.x[1]-L.x[0]); if (dx < 1) return L.y[0];
    float m  = (L.y[1]-L.y[0]) / dx;
    return L.y[0] + m * (float)((int)raw - (int)L.x[0]);
  }
  if (raw >= L.x[L.n-1]) {
    float dx = (float)(L.x[L.n-1]-L.x[L.n-2]); if (dx < 1) return L.y[L.n-1];
    float m  = (L.y[L.n-1]-L.y[L.n-2]) / dx;
    return L.y[L.n-1] + m * (float)((int)raw - (int)L.x[L.n-1]);
  }
  for (int i=0; i<L.n-1; i++){
    if (raw >= L.x[i] && raw <= L.x[i+1]){
      float dx = (float)(L.x[i+1]-L.x[i]); if (dx < 1) return L.y[i];
      float m = (L.y[i+1]-L.y[i])/dx;
      return L.y[i] + m * (float)((int)raw - (int)L.x[i]);
    }
  }
  return L.y[L.n-1];
}

bool setLutFromJson(int ch, JsonArrayConst pts){
  if (ch<0 || ch>=NUM_CH) return false;
  int n = min((int)pts.size(), MAX_LUT_POINTS);
  if (n < 2) return false;

  struct Pair { uint16_t x; float y; } tmp[MAX_LUT_POINTS];
  int i = 0;
  for (JsonObjectConst o : pts) {
    if (i>=n) break;
    tmp[i].x = (uint16_t)(o["x"] | 0);
    tmp[i].y = (float)(o["y"] | 0.0);
    i++;
  }
  n = i; if (n < 2) return false;

  // sort by x
  for(int k=1;k<n;k++){
    Pair key=tmp[k]; int j=k-1; while(j>=0 && tmp[j].x>key.x){ tmp[j+1]=tmp[j]; j--; } tmp[j+1]=key;
  }
  lut[ch].n = n;
  for (int k=0;k<n;k++){ lut[ch].x[k]=tmp[k].x; lut[ch].y[k]=tmp[k].y; }
  lut[ch].enabled = true;
  return true;
}

bool shiftLutX(int ch, int dx){
  if (ch<0 || ch>=NUM_CH) return false;
  if (lut[ch].n < 1) return false;
  for (int i=0;i<lut[ch].n;i++){
    int nx = (int)lut[ch].x[i] + dx;
    if (nx < 0) nx = 0;
    if (nx > 4095) nx = 4095;
    lut[ch].x[i] = (uint16_t)nx;
  }
  return true;
}

// ============== mmHg mapping ====================================
float adcToMmHg(int ch, uint16_t raw){
  if (raw <= MIN_VALID_RAW) {   // raw=0 -> 0 mmHg ทันที
    last_mmHg[ch] = 0.0f;
    has_last[ch]  = false;
    return 0.0f;
  }

  float v = lutInterp(ch, raw);
  if (isnan(v)) {
    v = cal[ch].gain*(float)raw + cal[ch].offset; // fallback linear
  }
  if (v < MMHG_MIN) v = MMHG_MIN;
  if (v > MMHG_MAX) v = MMHG_MAX;

  if (has_last[ch]) {
    float dv = v - last_mmHg[ch];
    if (dv >  MAX_STEP_MMHG) v = last_mmHg[ch] + MAX_STEP_MMHG;
    if (dv < -MAX_STEP_MMHG) v = last_mmHg[ch] - MAX_STEP_MMHG;
  }
  last_mmHg[ch] = v; has_last[ch] = true;
  return v;
}

// ============== Motor control (ลอจิกเดิม) ======================
int mapPressureToPWM(int pressure_mmHg) {
  int pwm = (int)((pressure_mmHg / 100.0f) * 4095.0f);
  return constrain(pwm, 0, 4095);
}
void motorControl(int* pwm_value) {
  MotionBit.Motor.run(M4, ROTATE_F, *pwm_value);
  delay(10);
  MotionBit.Motor.run(M2, ROTATE_F, *pwm_value / 2);
  delay(10);
  MotionBit.Motor.brake(M2);
  MotionBit.Motor.brake(M4);

  MotionBit.Motor.run(M3, ROTATE_F, *pwm_value * 25);
  delay(10);
  MotionBit.Motor.run(M2, ROTATE_F, *pwm_value / 2);
  delay(13);
  MotionBit.Motor.brake(M2);
  MotionBit.Motor.brake(M3);

  String json = String("{\"start\":") + true + "}";
  client.publish(start_topic, json.c_str());
  Serial.println(start_topic);
  Serial.println("[DATA] START COMMAND: " + json);
}

// ============== Modes (min pressure) ============================
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
void runMode(const String& mode, int pressure_mmHg) {
  if (pressure_mmHg <= 0) { Serial.println("[ERROR] Invalid pressure"); return; }
  int pwm_value = mapPressureToPWM(pressure_mmHg);
  Serial.printf("[MODE] %s | Pressure: %d mmHg → PWM: %d\n", mode.c_str(), pressure_mmHg, pwm_value);
  motorControl(&pwm_value);
}

// ============== WiFi / MQTT ====================================
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n[WiFi] Connected");
}

void publishLut(){
  JsonDocument doc;
  JsonArray arr = doc["lut"].to<JsonArray>();
  for (int ch=0; ch<NUM_CH; ch++){
    JsonObject o = arr.add<JsonObject>();
    o["ch"]=ch; o["en"]=lut[ch].enabled;
    JsonArray p = o["pts"].to<JsonArray>();
    for (int i=0;i<lut[ch].n;i++){
      JsonObject q = p.add<JsonObject>();
      q["x"]=lut[ch].x[i]; q["y"]=lut[ch].y[i];
    }
  }
  char buf[1024]; size_t n = serializeJson(doc, buf, sizeof(buf));
  client.publish(data_topic, buf, n);
  Serial.printf("[LUT] %s\n", buf);
}

void publishCal(){
  JsonDocument doc;
  JsonArray ca = doc["cal"].to<JsonArray>();
  for(int ch=0; ch<NUM_CH; ch++){
    JsonObject o = ca.add<JsonObject>();
    o["ch"]=ch; o["gain"]=cal[ch].gain; o["offset"]=cal[ch].offset;
  }
  char buf[512]; size_t n = serializeJson(doc, buf, sizeof(buf));
  client.publish(data_topic, buf, n);
  Serial.printf("[CAL] %s\n", buf);
}

void publishTelemetry(){
  JsonDocument doc;
  JsonArray arr = doc["ch"].to<JsonArray>();

  for (int ch=0; ch<NUM_CH; ch++){
    uint16_t raw = readStableRaw(ANALOG_PINS[ch]);
    float mm = adcToMmHg(ch, raw);
    JsonObject o = arr.add<JsonObject>();
    o["ch"]   = ch;
    o["raw"]  = raw;
    o["mmHg"] = mm;
  }

  // แนบ LUT + คาลิเบรต linear
  JsonArray la = doc["lut"].to<JsonArray>();
  for (int ch=0; ch<NUM_CH; ch++){
    JsonObject o = la.add<JsonObject>();
    o["ch"]=ch; o["en"]=lut[ch].enabled;
    JsonArray p = o["pts"].to<JsonArray>();
    for (int i=0;i<lut[ch].n;i++){
      JsonObject q = p.add<JsonObject>();
      q["x"]=lut[ch].x[i]; q["y"]=lut[ch].y[i];
    }
  }
  JsonArray ca = doc["cal"].to<JsonArray>();
  for (int ch=0; ch<NUM_CH; ch++){
    JsonObject o = ca.add<JsonObject>();
    o["ch"]=ch; o["gain"]=cal[ch].gain; o["offset"]=cal[ch].offset;
  }

  char buf[1536]; size_t n=serializeJson(doc, buf, sizeof(buf));
  client.publish(data_topic, buf, n);
  Serial.printf("[DATA] %s\n", buf);
}

void handleCmdJson(const JsonDocument& doc){
  // ---------- LUT commands ----------
  String cmd = doc["cmd"] | "";
  if (cmd == "lut_enable"){
    int ch = doc["ch"] | 0; bool en = doc["en"] | true;
    if (ch>=0 && ch<NUM_CH){ lut[ch].enabled = en; }
    publishLut(); return;
  }
  if (cmd == "lut_set"){
    int ch = doc["ch"] | 0;
    if (doc["pts"].is<JsonArrayConst>()){
      JsonArrayConst pts = doc["pts"].as<JsonArrayConst>();
      if (setLutFromJson(ch, pts)) publishLut();
      else Serial.println("[LUT] set failed (need >=2 points)");
    }
    return;
  }
  if (cmd == "lut_clear"){
    int ch = doc["ch"] | 0;
    if (ch>=0 && ch<NUM_CH){ lut[ch].n=0; lut[ch].enabled=false; }
    publishLut(); return;
  }
  if (cmd == "lut_shift"){
    int ch = doc["ch"] | 0; int dx = doc["dx"] | 0; // shift x ทั้งชุด
    if (shiftLutX(ch, dx)) publishLut();
    else Serial.println("[LUT] shift failed");
    return;
  }
  if (cmd == "lut_print"){ publishLut(); return; }

  // ---------- Two-point linear calib ----------
  if (cmd == "cal2_start"){ 
    int ch = doc["ch"] | 0; 
    cal2.active=true; cal2.ch=ch; delay(200);
    cal2.raw0 = readStableRaw(ANALOG_PINS[ch]); cal2.haveZero=true;
    Serial.printf("[CAL2] START ch=%d raw_zero=%u\n", ch, cal2.raw0);
    return; 
  }
  if (cmd == "cal2_ref")  { 
    float ref = doc["ref"] | 40.0f; 
    if (!cal2.active || !cal2.haveZero) { Serial.println("[CAL2] Not started"); return; }
    int ch=cal2.ch; delay(200);
    uint16_t raw_ref = readStableRaw(ANALOG_PINS[ch]);
    float denom = (float)raw_ref - (float)cal2.raw0;
    if (fabs(denom) < 1.0f) { Serial.println("[CAL2] invalid points"); return; }
    cal[ch].gain   = ref / denom;
    cal[ch].offset = -cal[ch].gain * (float)cal2.raw0;
    saveCalToNVS();
    Serial.printf("[CAL2] COMMIT ch=%d raw0=%u raw_ref=%u ref=%.1f => gain=%.6f offset=%.2f\n",
                  ch, cal2.raw0, raw_ref, ref, cal[ch].gain, cal[ch].offset);
    cal2.active=false; cal2.haveZero=false;
    publishCal(); 
    return;
  }
  if (cmd == "cal_print") { publishCal(); return; }

  // ---------- Legacy run ----------
  String mode = doc["mode"] | "";
  int pressure = doc["pressure"] | -1;
  if (pressure <= 0) { pressure = getMinimumPressure(mode); Serial.printf("[AUTO] Using default pressure = %d mmHg\n", pressure); }
  runMode(mode, pressure);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg; msg.reserve(length+1);
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];

  if (t == pair_topic && msg.indexOf("Pair Request") >= 0) {
    String response = String("{\"device\":\"") + NAME + "\"}";
    client.publish(dev_topic, response.c_str());
    Serial.println("[PAIR] Responded to Pair Request");
    return;
  }

  if (t == cmd_topic) {
    JsonDocument doc;                 // v7
    DeserializationError err = deserializeJson(doc, msg);
    if (err) { Serial.print("[ERROR] JSON parse failed: "); Serial.println(err.c_str()); return; }
    handleCmdJson(doc);
  }
}

void setup_mqtt() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(pair_topic);
  client.subscribe(cmd_topic);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("[MQTT] Connecting...");
    if (client.connect(NAME)) {
      Serial.println("connected");
      client.subscribe(pair_topic);
      client.subscribe(cmd_topic);
      publishLut();
      publishCal();
    } else {
      Serial.print("failed, rc="); Serial.print(client.state()); delay(2000);
    }
  }
}

// ================== Setup / Loop ==================
void setup() {
  Serial.begin(115200);
  MotionBit.begin(PWM_FREQ);

  // ADC
  analogSetWidth(ADC_BIT_WIDTH);
  for (int i=0;i<NUM_CH;i++) analogSetPinAttenuation(ANALOG_PINS[i], ADC_ATTN);

  // linear fallback defaults + load saved
  for (int ch=0; ch<NUM_CH; ch++){ cal[ch].gain = 100.0f/4095.0f; cal[ch].offset = 0.0f; }
  loadCalFromNVS();

  // Default LUT for ch0 (เริ่มต้น: 0→0, 200→3, ..., 2200→40, 2400→50)
  {
    int ch=0; lut[ch].enabled=true;
    uint16_t defx[] = { 0, 220, 440, 660, 880, 1100, 1320, 1540, 1760, 1980, 2200, 2800, 3400 };
    float    defy[] = { 0,   3,   6,   9,   12,   15,   18,   21,   24,   27,   30,   40,   50 };
    lut[ch].n = sizeof(defx)/sizeof(defx[0]);
    for (int i=0;i<lut[ch].n;i++){ lut[ch].x[i]=defx[i]; lut[ch].y[i]=defy[i]; }
  }
  // ch1: ปิด LUT ไว้ก่อน
  lut[1].enabled = false; lut[1].n = 0;

  // topics
  snprintf(data_topic, sizeof(data_topic), "stabilizer/server/data/%s", NAME);
  snprintf(cmd_topic,  sizeof(cmd_topic),  "stabilizer/server/cmd/%s",  NAME);
  snprintf(start_topic,sizeof(start_topic),"stabilizer/server/start/%s", NAME);

  WiFi.mode(WIFI_STA);
  setup_wifi();
  setup_mqtt();

  publishLut();
  publishCal();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (millis() - lastSent > 1000) {
    publishTelemetry();
    lastSent = millis();
  }
}
