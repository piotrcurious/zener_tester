#include <Arduino.h>
#include <Wire.h>
#include "BlueDisplay.hpp"
#include <PID_v1.h>

/*
 * Integrated Zener Diode Tester with Accurate Post-Test Inspection Tool
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

// --- Sweep Parameters ---
const float tempStart = 10.0;
const float tempEnd = 60.0;
const float tempStepSize = 10.0;
const int MAX_TEMPS = 6;
const int MAX_STEPS = 64;
unsigned long stabilizeDelayMs = 2000;
unsigned long stepDelayMs = 50;

// --- Data Structures ---
struct DataPoint {
  float vConv;
  float vZener;
};

DataPoint sweepData[MAX_TEMPS][MAX_STEPS];

// --- State ---
double setpoint, input, output;
double Kp = 10.0, Ki = 0.5, Kd = 0.1;
PID tempPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

float currentTargetTemp = 0;
bool testFinished = false;

// Inspection State
int selectedTempIdx = 0;
int lastSelectedTempIdx = -1;
int touchedX = -1;
int lastTouchedX = -1;
uint16_t dispW, dispH, plotW;

// --- BlueDisplay Callbacks ---

void handleTouch(struct TouchEvent *const aTouchPtr) {
  if (!testFinished) return;
  int x = aTouchPtr->TouchPosition.PositionX;
  int y = aTouchPtr->TouchPosition.PositionY;

  if (x > plotW) {
    selectedTempIdx = map(y, 0, dispH, 0, MAX_TEMPS - 1);
    selectedTempIdx = constrain(selectedTempIdx, 0, MAX_TEMPS - 1);
  } else {
    touchedX = x;
  }
}

void initDisplay(void) {
  dispW = BlueDisplay1.getMaxDisplayWidth();
  dispH = BlueDisplay1.getMaxDisplayHeight();
  plotW = (dispW * 9) / 10;
  BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE, dispW, dispH);
}

uint16_t getTempColor(float t) {
  uint8_t r = (uint8_t)map((long)(t * 10), (long)(tempStart * 10), (long)(tempEnd * 10), 0, 31);
  uint8_t b = (uint8_t)map((long)(t * 10), (long)(tempStart * 10), (long)(tempEnd * 10), 31, 0);
  return (r << 11) | b;
}

void drawInspectionTool(bool forceRedraw) {
  if (forceRedraw || selectedTempIdx != lastSelectedTempIdx || touchedX != lastTouchedX) {
    BlueDisplay1.clearDisplay(COLOR16_WHITE);

    // Grid (40V range)
    for (int i = 0; i <= 40; i += 10) {
      int py = dispH - (int)((i / 40.0) * dispH);
      BlueDisplay1.drawLine(0, py, plotW, py, COLOR16_LIGHTGRAY);
      int px = (int)((i / 40.0) * plotW);
      BlueDisplay1.drawLine(px, 0, px, dispH, COLOR16_LIGHTGRAY);
    }

    // Curves
    for (int t = 0; t < MAX_TEMPS; t++) {
      uint16_t color = getTempColor(tempStart + t * tempStepSize);
      int prevX = -1, prevY = -1;
      for (int s = 0; s < MAX_STEPS; s++) {
        int px = (int)((sweepData[t][s].vConv / 40.0) * plotW);
        int py = dispH - (int)((sweepData[t][s].vZener / 40.0) * dispH);
        if (prevX != -1) BlueDisplay1.drawLine(prevX, prevY, px, py, color);
        prevX = px; prevY = py;
      }
    }

    // Slider
    BlueDisplay1.fillRect(plotW, 0, dispW - 1, dispH - 1, COLOR16_LIGHTGRAY);
    int handleY = map(selectedTempIdx, 0, MAX_TEMPS - 1, 20, dispH - 20);
    BlueDisplay1.fillRect(plotW + 5, handleY - 15, dispW - 5, handleY + 15, COLOR16_RED);

    char buf[64];
    sprintf(buf, "T:%.0fC", tempStart + selectedTempIdx * tempStepSize);
    BlueDisplay1.drawText(plotW - 100, 10, buf, LABEL_FONT_SIZE, COLOR16_BLACK, COLOR16_WHITE);

    // Crosshairs
    if (touchedX >= 0 && touchedX <= plotW) {
      BlueDisplay1.drawLine(touchedX, 0, touchedX, dispH, COLOR16_GRAY);

      float vConvTarget = (touchedX / (float)plotW) * 40.0;
      // Nearest neighbor search in stored data for the selected curve
      int bestIdx = 0;
      float minDist = 1000.0;
      for (int s = 0; s < MAX_STEPS; s++) {
        float d = abs(sweepData[selectedTempIdx][s].vConv - vConvTarget);
        if (d < minDist) { minDist = d; bestIdx = s; }
      }

      float vZ = sweepData[selectedTempIdx][bestIdx].vZener;
      float vC = sweepData[selectedTempIdx][bestIdx].vConv;
      int py = dispH - (int)((vZ / 40.0) * dispH);
      int pxActual = (int)((vC / 40.0) * plotW);

      BlueDisplay1.drawLine(0, py, plotW, py, getTempColor(tempStart + selectedTempIdx * tempStepSize));

      sprintf(buf, "Vin:%.2fV Vz:%.2fV", vC, vZ);
      BlueDisplay1.drawText(10, dispH - 30, buf, LABEL_FONT_SIZE, COLOR16_BLUE, COLOR16_WHITE);
    }
    lastSelectedTempIdx = selectedTempIdx;
    lastTouchedX = touchedX;
  }
}

void drawGui(void) {
  if (testFinished) {
    drawInspectionTool(true);
  } else {
    BlueDisplay1.clearDisplay(COLOR16_WHITE);
    if (currentTargetTemp > 0) {
      char buf[32];
      sprintf(buf, "Testing: %.1f C", currentTargetTemp);
      BlueDisplay1.drawText(10, 10, buf, LABEL_FONT_SIZE, COLOR16_BLACK, COLOR16_WHITE);
    } else {
      BlueDisplay1.drawText(10, 10, "Zener Diode Tester", LABEL_FONT_SIZE, COLOR16_BLUE, COLOR16_WHITE);
    }
  }
}

void yieldToDisplay() {
  for (uint8_t i = 0; i < AMOUNT_OF_REQUESTS_PROCESSED; i++) {
    checkAndHandleEvents();
  }
}

// --- Hardware ---

float readTemperature() {
  Wire.beginTransmission(ADT75_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -999.0;
  Wire.requestFrom(ADT75_ADDR, 2);
  if (Wire.available() == 2) {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    raw >>= 4;
    if (raw & 0x800) raw |= 0xF000;
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
  registerTouchDownCallback(&handleTouch);
  registerTouchMoveCallback(&handleTouch);
}

void performZenerSweep(float targetTemp, float currentTemp, int tempIdx) {
  uint16_t plotColor = getTempColor(targetTemp);
  currentTargetTemp = targetTemp;
  int stepNum = 0;
  int prevX = -1, prevY = -1;
  for (int d = 0; d <= PWM_MAX; d += (PWM_MAX / MAX_STEPS + 1)) {
    if (stepNum >= MAX_STEPS) break;
    ledcWrite(CH_ZENER, d);
    delay(stepDelayMs);
    uint32_t mvConv = 0, mvZener = 0;
    for (int i = 0; i < 10; i++) {
      mvConv += analogReadMilliVolts(PIN_ANA_CONV_VOLT);
      mvZener += analogReadMilliVolts(PIN_ANA_ZENER_VOLT);
      delay(1);
    }
    float vConv = (mvConv / 10.0) / 1000.0 * 11.0;
    float vZener = (mvZener / 10.0) / 1000.0 * 11.0;
    sweepData[tempIdx][stepNum] = {vConv, vZener};
    Serial.printf("%.1f,%.2f,%d,%.3f,%.3f\n", targetTemp, currentTemp, d, vConv, vZener);
    int px = (int)((vConv / 40.0) * plotW);
    int py = dispH - (int)((vZener / 40.0) * dispH);
    if (prevX != -1) BlueDisplay1.drawLine(prevX, prevY, px, py, plotColor);
    prevX = px; prevY = py;
    yieldToDisplay();
    stepNum++;
  }
  Serial.println("");
}

void loop() {
  int tIdx = 0;
  for (float t = tempStart; t <= tempEnd && tIdx < MAX_TEMPS; t += tempStepSize, tIdx++) {
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
    performZenerSweep(setpoint, readTemperature(), tIdx);
  }
  testFinished = true;
  setPeltier(0);
  while(1) {
    yieldToDisplay();
    drawInspectionTool(false);
    delay(50);
  }
}
