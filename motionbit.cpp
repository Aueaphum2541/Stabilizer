#include <Arduino.h>
#include <esp_log.h>
#include "motionbit.h"

#define TAG     "MotionBit"

// PCA9685 constants
#define PCA9685_I2C_ADDR        0x40
#define REG_ADD_MODE1           0x00
#define REG_ADD_LED0_ON_L       0x06
#define REG_ADD_ALL_LED_ON_L    0xFA
#define REG_ADD_ALL_LED_OFF_L   0xFC
#define REG_ADD_PRESCALE        0xFE

// WS2812B constants
#define RGB_PIN                 27
#define RGB_NUMPIXELS           2

// static functions
static void i2c_write_8bit(uint8_t reg, uint8_t value);
static uint8_t i2c_read_8bit(uint8_t reg);
static void i2c_write_16bit(uint8_t reg, uint16_t value);
static uint16_t i2c_read_16bit(uint8_t reg);
static void i2c_write_32bit(uint8_t reg, uint16_t low_16bit, uint16_t hi_16bit);

// global variables
MotionBitInterface MotionBit;

// MotionBitInterface constructor
MotionBitInterface::MotionBitInterface() : PCA9685(*this), Motor(*this), NeoPixel(*this) {
    ESP_LOGI(TAG, "MotionBitInterface constructor");
    Wire.begin(22,21);
}

//  MotionBitInterface destructor
MotionBitInterface::~MotionBitInterface() {
    ESP_LOGI(TAG, "MotionBitInterface destructor");
}

// Initialize MotionBit instance
void MotionBitInterface::begin(uint32_t pwm_freq) {
    ESP_LOGI(TAG, "MotionBitInterface begin");
    this->PCA9685.begin(pwm_freq);
}

// PCA9685Interface constructor
MotionBitInterface::PCA9685Interface::PCA9685Interface(MotionBitInterface &MotionBit) : MotionBitRef(MotionBit) {
    ESP_LOGI(TAG, "PCA9685Interface constructor");
}

// PCA9685Interface destructor
MotionBitInterface::PCA9685Interface::~PCA9685Interface() {
    ESP_LOGI(TAG, "PCA9685Interface destructor");
}

// Initialize PCA9685
void MotionBitInterface::PCA9685Interface::begin(uint32_t pwm_freq) {
    if (i2c_read_8bit(REG_ADD_PRESCALE) != 30) {
        ESP_LOGI(TAG, "PCA9685 ready");
        return;
    }

    ESP_LOGI(TAG, "Configure PCA9685");
    // Enable Register Auto-Increment and go to sleep.
    i2c_write_8bit(REG_ADD_MODE1, 0x30);

    // Set the prescale.
    uint32_t prescale = (25000000 / (pwm_freq * 4096)) - 1;
    i2c_write_8bit(REG_ADD_PRESCALE, prescale);

    // Turn off all outputs.
    this->close_all();

    i2c_write_8bit(REG_ADD_MODE1, 0x20);    // Wake up.
    delay(10);                              // Wait for oscillator to stabilize.
    i2c_write_8bit(REG_ADD_MODE1, 0xA0);    // Restart.
}

// Set the PWM frequency for the PCA9685
void MotionBitInterface::PCA9685Interface::set_pwm(uint8_t ch, uint16_t Ton) {
    if (ch > 15) {
        ESP_LOGE(TAG, "Invalid channel");
        return;
    }
    uint16_t regAdd = REG_ADD_LED0_ON_L + (ch*4);
    if (Ton <= 0) {
        i2c_write_32bit(regAdd, 0x0000, 0x1000);
    } else if (Ton >= 4095) {
        i2c_write_32bit(regAdd, 0x1000, 0x0000);
    } else {
        i2c_write_32bit(regAdd, 0x0000, Ton);
    }
}

// close all channels
void MotionBitInterface::PCA9685Interface::close_all() {
    i2c_write_32bit(REG_ADD_ALL_LED_ON_L, 0x0000, 0x1000);
}

// MotionBitInterface MotorInterface constructor
MotionBitInterface::MotorInterface::MotorInterface(MotionBitInterface &MotionBit) : MotionBitRef(MotionBit) {
    ESP_LOGI(TAG, "MotionBitInterface::MotorInterface constructor");
}

// MotionBitInterface MotorInterface destructor
MotionBitInterface::MotorInterface::~MotorInterface() {
    ESP_LOGI(TAG, "MotionBitInterface::MotorInterface destructor");
}

// set motor speed
void MotionBitInterface::MotorInterface::run(motionbit_motor_ch_t ch, motionbit_motor_dir_t dir, uint8_t speed) {
    speed = constrain(speed, 0, 100);
    uint16_t Ton = map(speed, 0, 100, 0, 4095);

    //this->MotionBitRef.PCA9685.begin(PWM_FREQ);
    if (dir == ROTATE_F) {
        if (ch == ALL_CH) {
            this->MotionBitRef.PCA9685.set_pwm(M1, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M1+1, 0);
            this->MotionBitRef.PCA9685.set_pwm(M2, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M2+1, 0);
            this->MotionBitRef.PCA9685.set_pwm(M3, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M3+1, 0);
            this->MotionBitRef.PCA9685.set_pwm(M4, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M4+1, 0);
        } else {
            this->MotionBitRef.PCA9685.set_pwm(ch, Ton);
            this->MotionBitRef.PCA9685.set_pwm(ch+1, 0);
        }
    } else {
        if (ch == ALL_CH) {
            this->MotionBitRef.PCA9685.set_pwm(M1, 0);
            this->MotionBitRef.PCA9685.set_pwm(M1+1, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M2, 0);
            this->MotionBitRef.PCA9685.set_pwm(M2+1, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M3, 0);
            this->MotionBitRef.PCA9685.set_pwm(M3+1, Ton);
            this->MotionBitRef.PCA9685.set_pwm(M4, 0);
            this->MotionBitRef.PCA9685.set_pwm(M4+1, Ton);
        } else {
            this->MotionBitRef.PCA9685.set_pwm(ch, 0);
            this->MotionBitRef.PCA9685.set_pwm(ch+1, Ton);
        }
    }
}

void MotionBitInterface::MotorInterface::brake(motionbit_motor_ch_t ch) {
    if (ch == ALL_CH) {
        this->MotionBitRef.PCA9685.set_pwm(M1, 0);
        this->MotionBitRef.PCA9685.set_pwm(M1+1, 0);
        this->MotionBitRef.PCA9685.set_pwm(M2, 0);
        this->MotionBitRef.PCA9685.set_pwm(M2+1, 0);
        this->MotionBitRef.PCA9685.set_pwm(M3, 0);
        this->MotionBitRef.PCA9685.set_pwm(M3+1, 0);
        this->MotionBitRef.PCA9685.set_pwm(M4, 0);
        this->MotionBitRef.PCA9685.set_pwm(M4+1, 0);
    } else {
        this->MotionBitRef.PCA9685.set_pwm(ch, 0);
        this->MotionBitRef.PCA9685.set_pwm(ch+1, 0);
    }
}

// MotionBitInterface NeoPixelInterface constructor
MotionBitInterface::NeoPixelInterface::NeoPixelInterface(MotionBitInterface &MotionBit) : MotionBitRef(MotionBit) {
    ESP_LOGI(TAG, "MotionBitInterface::NeoPixelInterface constructor");
    
}

// MotionBitInterface NeoPixelInterface destructor
MotionBitInterface::NeoPixelInterface::~NeoPixelInterface() {
    ESP_LOGI(TAG, "MotionBitInterface::NeoPixelInterface destructor");
}

// I2C utility functions: write one byte to a register
static void i2c_write_8bit(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// I2C utility functions: read one byte from a register
static uint8_t i2c_read_8bit(uint8_t reg) {
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCA9685_I2C_ADDR, 1);
  return Wire.read();
}

// I2C utility functions: write one half word to a register
static void i2c_write_16bit(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value & 0xFF);
  Wire.write(value >> 8);
  Wire.endTransmission();
}

// I2C utility functions: read one half word from a register
static uint16_t i2c_read_16bit(uint8_t reg) {
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCA9685_I2C_ADDR, 2);
  uint16_t value = Wire.read() | (Wire.read() << 8);
  return value;
}

// I2C utility functions: write one word to a register
static void i2c_write_32bit(uint8_t reg, uint16_t low_16bit, uint16_t hi_16bit) {
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(reg);
  Wire.write(low_16bit & 0xFF);
  Wire.write(low_16bit >> 8);
  Wire.write(hi_16bit & 0xFF);
  Wire.write(hi_16bit >> 8);
  Wire.endTransmission();
}