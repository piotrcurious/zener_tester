#include <Arduino.h>
#include <Wire.h>
#include "BlueDisplay.hpp"
#include <PID_v1.h>

/*
 * Integrated Zener Diode Tester with Improved BlueDisplay Handling
 *
 * Features:
 * - PID-controlled Heating and Cooling
 * - ADT75 12-bit I2C Temperature Sensor support
 * - Automated Temperature Sweep (10°C steps)
 * - Compound Plotting on BlueDisplay: Each temperature step uses a unique color.
 * - Structured CSV Output: Optimized for gnuplot and external analysis.
 */

// --- BlueDisplay Settings ---
#define AMOUNT_OF_REQUESTS_PROCESSED 32
#define LABEL_FONT_SIZE 24

// --- Hardware Pins ---
const int PIN_PWM_COOLING = 25;
const int PIN_PWM_HEATING = 26;
const int PIN_PWM_ZENER_CTRL = 27;
const int PIN_ANA_CONV_VOLT = 34;
const int PIN_ANA_ZENER_VOLT = 35;

// --- Constants ---
const int ADT75_ADDR = 0x48;
const int PWM_FREQ = 5000;
const int PWM_RES = 10;
const int PWM_MAX = (1 << PWM_RES) - 1;

const int CH_COOLING = 0;
const int CH_HEATING = 1;
const int CH_ZENER = 2;

// --- PID Settings ---
double setpoint, input, output;
double Kp = 10.0, Ki = 0.5, Kd = 0.1;
PID tempPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Sweep Parameters ---
float tempStart = 10.0;
float tempEnd = 60.0;
float tempStepSize = 10.0;
int zenerSteps = 256;
unsigned long stabilizeDelayMs = 2000;
unsigned long stepDelayMs = 50;

// --- State ---
float currentTargetTemp = 0;
bool testFinished = false;

// --- BlueDisplay Callbacks ---

void initDisplay(void) {
  uint16_t displayWidth = BlueDisplay1.getMaxDisplayWidth();
  uint16_t displayHeight = BlueDisplay1.getMaxDisplayHeight();
  BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE, displayWidth, displayHeight);
}

void drawGui(void) {
  BlueDisplay1.clearDisplay(COLOR_WHITE);
  BlueDisplay1.drawGrid(20, 20, COLOR_LIGHTGRAY);

  if (testFinished) {
    BlueDisplay1.drawText(10, 10, "TEST FINISHED", LABEL_FONT_SIZE * 2, COLOR_RED, COLOR_WHITE);
  } else if (currentTargetTemp > 0) {
    char buf[32];
    sprintf(buf, "Current Step: %.1f C", currentTargetTemp);
    BlueDisplay1.drawText(10, 10, buf, LABEL_FONT_SIZE, COLOR_BLACK, COLOR_WHITE);
  } else {
    BlueDisplay1.drawText(10, 10, "Zener Diode Tester", LABEL_FONT_SIZE, COLOR_BLUE, COLOR_WHITE);
  }
}

void yieldToDisplay() {
  for (uint8_t i = 0; i < AMOUNT_OF_REQUESTS_PROCESSED; i++) {
    checkAndHandleEvents();
  }
}

// --- Hardware Functions ---

float readTemperature() {
  Wire.beginTransmission(ADT75_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -999.0;

  Wire.requestFrom(ADT75_ADDR, 2);
  if (Wire.available() == 2) {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    raw >>= 4; // 12-bit
    if (raw & 0x800) raw |= 0xF000; // Sign extend
    return raw * 0.0625;
  }
  return -999.0;
}

void setPeltier(double val) {
  if (val > 0) {
    ledcWrite(CH_HEATING, (uint32_t)val);
    ledcWrite(CH_COOLING, 0);
  } else if (val < 0) {
    ledcWrite(CH_HEATING, 0);
    ledcWrite(CH_COOLING, (uint32_t)(-val));
  } else {
    ledcWrite(CH_HEATING, 0);
    ledcWrite(CH_COOLING, 0);
  }
}

uint16_t getTempColor(float t) {
  uint8_t r = (uint8_t)map((long)(t * 10), (long)(tempStart * 10), (long)(tempEnd * 10), 0, 31);
  uint8_t b = (uint8_t)map((long)(t * 10), (long)(tempStart * 10), (long)(tempEnd * 10), 31, 0);
  return (r << 11) | b;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ledcSetup(CH_COOLING, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_COOLING, CH_COOLING);
  ledcSetup(CH_HEATING, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_HEATING, CH_HEATING);
  ledcSetup(CH_ZENER, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_ZENER_CTRL, CH_ZENER);

  tempPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  tempPID.SetMode(AUTOMATIC);

  BlueDisplay1.initCommunication(&initDisplay, &drawGui);

  Serial.println("# TargetTemp,ActualTemp,Duty,ConvVolt,ZenerVolt");
}

void performZenerSweep(float targetTemp, float currentTemp) {
  int displayWidth = BlueDisplay1.getDisplayWidth();
  int displayHeight = BlueDisplay1.getDisplayHeight();
  uint16_t plotColor = getTempColor(targetTemp);

  currentTargetTemp = targetTemp;

  // Update status on display
  char buf[32];
  sprintf(buf, "Current Step: %.1f C", targetTemp);
  BlueDisplay1.drawText(10, 10, buf, LABEL_FONT_SIZE, COLOR_BLACK, COLOR_WHITE);

  for (int d = 0; d <= PWM_MAX; d += (PWM_MAX / zenerSteps + 1)) {
    ledcWrite(CH_ZENER, d);
    delay(stepDelayMs);

    uint32_t mvConv = 0;
    uint32_t mvZener = 0;
    for (int i = 0; i < 10; i++) {
      mvConv += analogReadMilliVolts(PIN_ANA_CONV_VOLT);
      mvZener += analogReadMilliVolts(PIN_ANA_ZENER_VOLT);
      delay(1);
    }
    float vConv = (mvConv / 10.0) / 1000.0 * 11.0;
    float vZener = (mvZener / 10.0) / 1000.0 * 11.0;

    Serial.print(targetTemp); Serial.print(",");
    Serial.print(currentTemp); Serial.print(",");
    Serial.print(d); Serial.print(",");
    Serial.print(vConv); Serial.print(",");
    Serial.println(vZener);

    int px = (int)((vConv / 40.0) * displayWidth);
    int py = displayHeight - (int)((vZener / 40.0) * displayHeight);
    BlueDisplay1.plot(px, py, plotColor);

    yieldToDisplay();
  }
  ledcWrite(CH_ZENER, 0);
  Serial.println("");
}

void loop() {
  for (float t = tempStart; t <= tempEnd; t += tempStepSize) {
    setpoint = t;
    currentTargetTemp = t;

    unsigned long startWait = millis();
    while (millis() - startWait < 60000) {
      input = readTemperature();
      if (input == -999.0) break;

      tempPID.Compute();
      setPeltier(output);

      yieldToDisplay();

      if (abs(input - setpoint) < 0.2) {
        delay(stabilizeDelayMs);
        break;
      }
      delay(100);
    }

    performZenerSweep(setpoint, readTemperature());
  }

  Serial.println("# ALL SWEEPS COMPLETE");
  testFinished = true;
  setPeltier(0);

  while(1) {
    BlueDisplay1.drawText(10, 10, "TEST FINISHED", LABEL_FONT_SIZE * 2, COLOR_RED, COLOR_WHITE);
    yieldToDisplay();
    delay(1000);
  }
}
