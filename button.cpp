#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "motionbit.h"

// ================= PIN MAP =================
#define BTN_A 36   // Button A (P9)
#define BTN_B 4    // Button B (P8)

#define NEOPIXEL_PIN   13
#define NEOPIXEL_COUNT 25

// ================= NeoPixel =================
Adafruit_NeoPixel pixels(
  NEOPIXEL_COUNT,
  NEOPIXEL_PIN,
  NEO_GRB + NEO_KHZ800
);

// ================= Timing =================
const unsigned long MOTOR_DURATION_MS = 5000;   // 5 วินาที

// ================= State =================
bool motorRunning = false;
unsigned long motorStart = 0;

int prevA;
int prevB;

// ================= Pixel Helpers =================
void pixelOff() {
  pixels.clear();
  pixels.show();
}

void pixelRed() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
}

void pixelBlue() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
}

// ================= Motor Actions =================
// ปล่อยลมออก (Button A)
void startExhaust() {
  pixelRed();
  MotionBit.Motor.run(M2, ROTATE_B, 1200);
  MotionBit.Motor.run(M3, ROTATE_B, 1200);
  motorRunning = true;
  motorStart = millis();
  Serial.println("[ACTION] EXHAUST (RED)");
}

// ดูดลมเข้า (Button B)
void startInflate() {
  pixelBlue();
  MotionBit.Motor.run(M2, ROTATE_B, 1200);
  MotionBit.Motor.run(M4, ROTATE_F, 1200);
  motorRunning = true;
  motorStart = millis();
  Serial.println("[ACTION] INFLATE (BLUE)");
}

void stopAll() {
  MotionBit.Motor.brake(M2);
  MotionBit.Motor.brake(M3);
  MotionBit.Motor.brake(M4);
  pixelOff();
  motorRunning = false;
  Serial.println("[ACTION] STOP");
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  Serial.println("SYSTEM READY");

  // GPIO36 / GPIO4 ใช้ pull-up ภายนอกบนบอร์ด
  pinMode(BTN_A, INPUT);
  pinMode(BTN_B, INPUT);

  pixels.begin();
  pixelOff();

  MotionBit.begin(50);   // PWM 50 Hz

  // สำคัญมาก: อ่านสถานะปุ่มจริงตอนบูต
  prevA = digitalRead(BTN_A);
  prevB = digitalRead(BTN_B);
}

// ================= Loop =================
void loop() {

  int nowA = digitalRead(BTN_A);
  int nowB = digitalRead(BTN_B);

  // -------- Button A (LOW -> HIGH) --------
  if (!motorRunning && prevA == LOW && nowA == HIGH) {
    startExhaust();
  }

  // -------- Button B (LOW -> HIGH) --------
  if (!motorRunning && prevB == LOW && nowB == HIGH) {
    startInflate();
  }

  // -------- Stop after 5 seconds --------
  if (motorRunning && millis() - motorStart >= MOTOR_DURATION_MS) {
    stopAll();
  }

  prevA = nowA;
  prevB = nowB;
}
