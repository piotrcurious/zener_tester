#include <Arduino.h>
#include <Wire.h>
#include <BlueDisplay.h>
#include <PID_v1.h>

// Constants for the PWM and analog pins
const int pwmPinCooling = 25;
const int pwmPinHeating = 26;
const int pwmPinZenerControl = 27;
// ... other constants ...

// PID parameters
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Hysteresis overlap
const float hysteresis = 0.5;

// Task handle for the temperature regulation task
TaskHandle_t TempRegulationTaskHandle;

// Function prototypes
void TempRegulationTask(void *pvParameters);
// ... other function prototypes ...

void setup() {
  // ... setup code ...
  xTaskCreate(TempRegulationTask, "TempRegulation", 2048, NULL, 1, &TempRegulationTaskHandle);
}

void loop() {
  // Main loop code
}

void TempRegulationTask(void *pvParameters) {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // Limits for the PID output

  for (;;) {
    Input = readTemperature();
    myPID.Compute();

    // Apply hysteresis logic to prevent both heating and cooling from running at the same time
    if (Output > 0 && Input < Setpoint - hysteresis) {
      // Heating
      ledcWrite(pwmChannelHeating, Output);
      ledcWrite(pwmChannelCooling, 0);
    } else if (Output < 0 && Input > Setpoint + hysteresis) {
      // Cooling
      ledcWrite(pwmChannelCooling, -Output);
      ledcWrite(pwmChannelHeating, 0);
    } else {
      // Neither heating nor cooling
      ledcWrite(pwmChannelCooling, 0);
      ledcWrite(pwmChannelHeating, 0);
    }

    // Delay for a bit before re-running the loop
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ... other function implementations ...
