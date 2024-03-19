#include <Arduino.h>
#include <Wire.h>
#include <BlueDisplay.h>

// Constants for the PWM and analog pins
const int pwmPinCooling = 25;
const int pwmPinHeating = 26;
const int pwmPinZenerControl = 27; // PWM pin for Zener diode voltage control
const int voltageSensorPin = 34;
const int zenerVoltageSensorPin = 35;
const int adt75Address = 0x48; // ADT75 I2C address

// Variables for PWM signal
const int freq = 5000;
const int pwmChannelCooling = 0;
const int pwmChannelHeating = 1;
const int pwmChannelZenerControl = 2; // PWM channel for Zener control
const int resolution = 8;

// Temperature range and steps
const float tempStart = -75.0;
const float tempEnd = -20.0;
const int numSteps = 16;
const float tempStep = (tempEnd - tempStart) / (numSteps - 1);
const float errorRange = 0.5; // Acceptable error range in degrees Celsius

// Bluetooth Display object
BDClass myDisplay;

// Function prototypes
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
float readTemperature();
void regulateTemperature(float targetTemp);
void initializeSystem();
void testZenerDiodeAtTemperature(float targetTemp);

void setup() {
  initializeSystem();
}

void loop() {
  for (int i = 0; i < numSteps; ++i) {
    float targetTemp = fmap(i, 0, numSteps - 1, tempStart, tempEnd);
    testZenerDiodeAtTemperature(targetTemp);
  }
}

void initializeSystem() {
  Serial.begin(115200);
  myDisplay.begin();
  myDisplay.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, 0, 0);
  delay(500); // Wait for display size detection
  Wire.begin();
  ledcSetup(pwmChannelCooling, freq, resolution);
  ledcAttachPin(pwmPinCooling, pwmChannelCooling);
  ledcSetup(pwmChannelHeating, freq, resolution);
  ledcAttachPin(pwmPinHeating, pwmChannelHeating);
  ledcSetup(pwmChannelZenerControl, freq, resolution);
  ledcAttachPin(pwmPinZenerControl, pwmChannelZenerControl);
  myDisplay.clearDisplay(COLOR_WHITE);
  myDisplay.setTitle("Zener Diode Tester");
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readTemperature() {
  // ... read temperature from ADT75 ...
}

void regulateTemperature(float targetTemp) {
  // ... regulate temperature ...
}

void testZenerDiodeAtTemperature(float targetTemp) {
  regulateTemperature(targetTemp);
  delay(1000); // Wait for temperature to stabilize
  if (abs(readTemperature() - targetTemp) <= errorRange) {
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
      ledcWrite(pwmChannelZenerControl, dutyCycle); // Control Zener diode voltage
      delay(1000); // Wait for voltage to stabilize
      int voltageSensorValue = analogRead(voltageSensorPin);
      int zenerVoltageSensorValue = analogRead(zenerVoltageSensorPin);
      float converterVoltage = voltageSensorValue * (3.3 / 4095.0);
      float zenerDiodeVoltage = zenerVoltageSensorValue * (3.3 / 4095.0);
      int displayX = fmap(converterVoltage, 0, 3.3, 0, myDisplay.getDisplayWidth());
      int displayY = fmap(zenerDiodeVoltage, 0, 3.3, myDisplay.getDisplayHeight(), 0);
      myDisplay.plot(displayX, displayY, COLOR_RED);
    }
    ledcWrite(pwmChannelZenerControl, 0); // Reset Zener control PWM
  }
}
