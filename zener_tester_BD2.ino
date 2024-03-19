#include <Arduino.h>
#include <BlueDisplay.h>

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

// Bluetooth Display object
BDClass myDisplay;

// Function to map float values
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize Bluetooth Display
  myDisplay.begin(); // Starts the Bluetooth service

  // Configure PWM parameters
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(pwmPin, pwmChannel);

  // Print CSV header
  Serial.println("Converter Voltage,Zener Diode Voltage");

  // Detect and set the display size
  myDisplay.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, 0, 0);

  // Setup the display layout after a short delay
  delay(500); // Wait for display size detection
  myDisplay.clearDisplay(COLOR_WHITE);
  myDisplay.setTitle("Zener Diode Tester");
  myDisplay.drawGrid(10, 10, COLOR_BLUE);
}

void loop() {
  // Get the display width and height
  int displayWidth = myDisplay.getDisplayWidth();
  int displayHeight = myDisplay.getDisplayHeight();

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

    // Scale the voltages to the display resolution
    int displayX = fmap(converterVoltage, 0, 3.3, 0, displayWidth);
    int displayY = fmap(zenerDiodeVoltage, 0, 3.3, displayHeight, 0); // Inverted Y for display coordinates

    // Plot the values on the display
    myDisplay.plot(displayX, displayY, COLOR_RED);
  }

  // Reset the PWM signal
  ledcWrite(pwmChannel, 0);

  // Add a delay before the next measurement cycle
  delay(5000);
}
