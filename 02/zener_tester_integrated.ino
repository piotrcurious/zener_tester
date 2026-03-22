#include <Arduino.h>
#include <Wire.h>
#include <BlueDisplay.h>
#include <PID_v1.h>

/*
 * Integrated Zener Diode Tester with Temperature Control
 *
 * Features:
 * - PID-controlled Heating and Cooling
 * - ADT75 12-bit I2C Temperature Sensor support
 * - Automated Temperature Sweep (10°C steps)
 * - Clear Plot per Temperature Step on BlueDisplay
 * - Automated Zener Diode V-I characteristic sweep
 * - Serial CSV output with step markers
 */

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
float tempStepSize = 10.0; // 10 degree steps as requested
int zenerSteps = 256;
unsigned long stabilizeDelayMs = 2000;
unsigned long stepDelayMs = 50;

// --- State ---
BDClass myDisplay;

// --- Functions ---

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

  myDisplay.begin();
  myDisplay.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE, 0, 0);
  delay(1000);
  myDisplay.clearDisplay(COLOR_WHITE);
  myDisplay.setTitle("Integrated Zener Tester");

  Serial.println("--- START OF TEST ---");
  Serial.println("TargetTemp,ActualTemp,Duty,ConvVolt,ZenerVolt");
}

void performZenerSweep(float targetTemp, float currentTemp) {
  int displayWidth = myDisplay.getDisplayWidth();
  int displayHeight = myDisplay.getDisplayHeight();

  // Clear plot for each temperature step as requested
  myDisplay.clearDisplay(COLOR_WHITE);
  char buf[32];
  sprintf(buf, "Temp Step: %.1f C", targetTemp);
  myDisplay.drawText(10, 10, buf, 1, COLOR_BLACK, COLOR_WHITE);
  myDisplay.drawGrid(20, 20, COLOR_LIGHTGRAY);

  Serial.print("# New Sweep at "); Serial.print(targetTemp); Serial.println(" C");

  for (int d = 0; d <= PWM_MAX; d += (PWM_MAX / zenerSteps + 1)) {
    ledcWrite(CH_ZENER, d);
    delay(stepDelayMs);

    int rawConv = analogRead(PIN_ANA_CONV_VOLT);
    int rawZener = analogRead(PIN_ANA_ZENER_VOLT);

    float vConv = rawConv * (3.3 / 4095.0) * 11.0;
    float vZener = rawZener * (3.3 / 4095.0) * 11.0;

    Serial.print(targetTemp); Serial.print(",");
    Serial.print(currentTemp); Serial.print(",");
    Serial.print(d); Serial.print(",");
    Serial.print(vConv); Serial.print(",");
    Serial.println(vZener);

    // Plotting: X=Supply, Y=Zener
    int px = map(rawConv, 0, 4095, 0, displayWidth);
    int py = map(4095 - rawZener, 0, 4095, 0, displayHeight);
    myDisplay.plot(px, py, COLOR_RED);
  }
  ledcWrite(CH_ZENER, 0);
}

void loop() {
  for (float t = tempStart; t <= tempEnd; t += tempStepSize) {
    setpoint = t;
    Serial.print("Stabilizing at "); Serial.print(setpoint); Serial.println(" C...");

    unsigned long startWait = millis();
    while (millis() - startWait < 60000) { // Max 60s for large steps
      input = readTemperature();
      if (input == -999.0) break;

      tempPID.Compute();
      setPeltier(output);

      if (abs(input - setpoint) < 0.2) {
        delay(stabilizeDelayMs);
        break;
      }
      delay(100);
    }

    performZenerSweep(setpoint, readTemperature());
  }

  Serial.println("--- ALL SWEEPS COMPLETE ---");
  setPeltier(0);
  while(1) {
    myDisplay.drawText(10, 10, "TEST FINISHED", 2, COLOR_RED, COLOR_WHITE);
    delay(1000);
  }
}
