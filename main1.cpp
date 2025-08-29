// ===== main.cpp (No Wi-Fi — Serial Only) =====
#include <Arduino.h>
#include "motionbit.h"   // ใช้เฉพาะถ้าจะสั่งมอเตอร์; ไม่ใช้ก็ลบ include ได้

// ---------- Pins / ADC ----------
#define ANALOG_PIN       32
#define SAMPLES_ZERO     120
#define READ_INTERVAL_MS 1000

// ---------- Calibration & Threshold ----------
#define ADC_AT_REF       2000
#define MMHG_AT_REF      40.0f
#define THRESHOLD_MMHG   40.0f

static float adc_zero = 0.0f;

// ---------- (ถ้าจะทดลองมอเตอร์ ให้เปิดใช้) ----------
#define ENABLE_MOTOR 0
#if ENABLE_MOTOR
  #define PWM_FREQ 50
  void driveByPressure(float mmHg) {
    uint8_t speed = (uint8_t) constrain((mmHg / 100.0f) * 100.0f, 0.0f, 100.0f);
    MotionBit.Motor.run(M3, ROTATE_F, speed);
    delay(50);
    MotionBit.Motor.brake(M3);
  }
#endif

// ---------- Utils ----------
float adcToMmHg(int adc) {
  float eff = (float)adc - adc_zero;
  if (eff < 0) eff = 0;
  return eff * (MMHG_AT_REF / (float)ADC_AT_REF);   // 0.02 mmHg ต่อ 1 count
}

void calibrateZero(uint16_t samples = SAMPLES_ZERO) {
  uint32_t sum = 0;
  delay(50);
  for (uint16_t i = 0; i < samples; ++i) {
    sum += analogRead(ANALOG_PIN);
    delay(2);
  }
  adc_zero = (float)sum / samples;
  Serial.printf("[CAL] Zero set: adc_zero = %.1f (counts)\n", adc_zero);
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  #if ENABLE_MOTOR
    MotionBit.begin(PWM_FREQ);
  #endif

  calibrateZero();
  Serial.println("[READY] Press 'z' + Enter to recalibrate zero.");
}

void loop() {
  // กด z เพื่อคาลิเบรตใหม่
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'z' || c == 'Z') calibrateZero();
  }

  static uint32_t last = 0;
  if (millis() - last >= READ_INTERVAL_MS) {
    last = millis();

    int   adc   = analogRead(ANALOG_PIN);
    float mmHg  = adcToMmHg(adc);
    int   digi  = (mmHg >= THRESHOLD_MMHG) ? 1 : 0;

    Serial.printf("[DATA] ADC=%d | mmHg=%.2f | digital=%d | zero=%.1f\n",
                  adc, mmHg, digi, adc_zero);

    #if ENABLE_MOTOR
      driveByPressure(mmHg);
    #endif
  }
}
