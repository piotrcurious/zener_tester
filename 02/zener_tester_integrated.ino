#include <Arduino.h>
#include <Wire.h>
#include <BlueDisplay.h>
#include <PID_v1.h>

/*
 * Integrated Zener Diode Tester with Temperature Control
 *
 * Features:
 * - PID-controlled Heating and Cooling (using Peltier or separate elements)
 * - ADT75 12-bit I2C Temperature Sensor support
 * - Automated Temperature Sweep
 * - Automated Zener Diode V-I characteristic sweep via PWM-controlled Step-up
 * - BlueDisplay real-time plotting
 * - Serial CSV output for external analysis
 */

// --- Hardware Pins ---
const int PIN_PWM_COOLING = 25;
const int PIN_PWM_HEATING = 26;
const int PIN_PWM_ZENER_CTRL = 27;
const int PIN_ANA_CONV_VOLT = 34; // Voltage after step-up converter
const int PIN_ANA_ZENER_VOLT = 35; // Voltage drop across Zener diode

// --- Constants ---
const int ADT75_ADDR = 0x48;
const int PWM_FREQ = 5000;
const int PWM_RES = 10; // Increased to 10-bit for better control
const int PWM_MAX = (1 << PWM_RES) - 1;

const int CH_COOLING = 0;
const int CH_HEATING = 1;
const int CH_ZENER = 2;

// --- PID Settings ---
double setpoint, input, output;
double Kp = 10.0, Ki = 0.5, Kd = 0.1; // Initial tuning, may need refinement
PID tempPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Sweep Parameters ---
float tempStart = 10.0;
float tempEnd = 60.0;
int tempSteps = 10;
int zenerSteps = 256;
unsigned long stabilizeDelayMs = 2000;
unsigned long stepDelayMs = 100;

// --- State ---
BDClass myDisplay;
bool isRunning = false;

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
  // val is from -PWM_MAX (Full Cooling) to +PWM_MAX (Full Heating)
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

  // PWM Setup
  ledcSetup(CH_COOLING, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_COOLING, CH_COOLING);
  ledcSetup(CH_HEATING, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_HEATING, CH_HEATING);
  ledcSetup(CH_ZENER, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_ZENER_CTRL, CH_ZENER);

  // PID Setup
  tempPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  tempPID.SetMode(AUTOMATIC);

  // Display Setup
  myDisplay.begin();
  myDisplay.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE, 0, 0);
  delay(1000);
  myDisplay.clearDisplay(COLOR_WHITE);
  myDisplay.setTitle("Integrated Zener Tester");

  Serial.println("Temp,Duty,ConvVolt,ZenerVolt");
}

void performZenerSweep(float currentTemp) {
  int displayWidth = myDisplay.getDisplayWidth();
  int displayHeight = myDisplay.getDisplayHeight();

  for (int d = 0; d <= PWM_MAX; d += (PWM_MAX / zenerSteps + 1)) {
    ledcWrite(CH_ZENER, d);
    delay(stepDelayMs);

    int rawConv = analogRead(PIN_ANA_CONV_VOLT);
    int rawZener = analogRead(PIN_ANA_ZENER_VOLT);

    float vConv = rawConv * (3.3 / 4095.0) * 11.0; // Assuming 1/11 voltage divider for higher voltages
    float vZener = rawZener * (3.3 / 4095.0) * 11.0;

    Serial.print(currentTemp); Serial.print(",");
    Serial.print(d); Serial.print(",");
    Serial.print(vConv); Serial.print(",");
    Serial.println(vZener);

    // Plotting
    int px = map(rawConv, 0, 4095, 0, displayWidth);
    int py = map(4095 - rawZener, 0, 4095, 0, displayHeight);
    uint16_t color = (uint16_t)map((int)(currentTemp * 10), (int)(tempStart * 10), (int)(tempEnd * 10), 0, 0xF800);
    myDisplay.plot(px, py, color);
  }
  ledcWrite(CH_ZENER, 0);
}

void loop() {
  // Simple state machine or sequential sweep
  for (int i = 0; i < tempSteps; i++) {
    setpoint = tempStart + i * (tempEnd - tempStart) / (tempSteps - 1);
    Serial.print("Target Temp: "); Serial.println(setpoint);

    unsigned long startWait = millis();
    while (millis() - startWait < 30000) { // Max 30s to reach temp
      input = readTemperature();
      if (input == -999.0) break;

      tempPID.Compute();
      setPeltier(output);

      if (abs(input - setpoint) < 0.5) {
        delay(stabilizeDelayMs); // Let it settle
        break;
      }
      delay(100);
    }

    performZenerSweep(readTemperature());
  }

  Serial.println("Sweep Complete.");
  setPeltier(0);
  while(1) delay(1000); // Wait for reset
}
