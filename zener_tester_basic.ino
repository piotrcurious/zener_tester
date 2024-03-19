#include <Arduino.h>

// Constants for the PWM and analog pins
const int pwmPin = 25; // PWM pin connected to the step-up converter
const int voltageSensorPin = 34; // Analog pin to measure voltage after converter
const int zenerVoltageSensorPin = 35; // Analog pin to measure Zener diode voltage drop

// Variables to store sensor values
int voltageSensorValue = 0;
int zenerVoltageSensorValue = 0;

// Variables for PWM signal
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure PWM parameters
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(pwmPin, pwmChannel);

  // Print CSV header
  Serial.println("Converter Voltage,Zener Diode Voltage");
}

void loop() {
  // Increase the voltage using PWM and read the sensor values
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(pwmChannel, dutyCycle); // Write PWM signal
    delay(1000); // Wait for the voltage to stabilize

    // Read the analog values
    voltageSensorValue = analogRead(voltageSensorPin);
    zenerVoltageSensorValue = analogRead(zenerVoltageSensorPin);

    // Convert the analog values to voltages
    float converterVoltage = voltageSensorValue * (3.3 / 4095.0);
    float zenerDiodeVoltage = zenerVoltageSensorValue * (3.3 / 4095.0);

    // Output the voltages in CSV format
    Serial.print(converterVoltage);
    Serial.print(",");
    Serial.println(zenerDiodeVoltage);
  }

  // Reset the PWM signal
  ledcWrite(pwmChannel, 0);

  // Add a delay before the next measurement cycle
  delay(5000);
}
