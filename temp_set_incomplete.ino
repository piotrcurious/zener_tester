#include <Arduino.h>
#include <BlueDisplay.h>
#include <Wire.h>

// Constants for the PWM and analog pins
const int pwmPinCooling = 25; // PWM pin connected to the cooling system
const int pwmPinHeating = 26; // PWM pin connected to the heating system
const int voltageSensorPin = 34; // Analog pin to measure voltage after converter
const int zenerVoltageSensorPin = 35; // Analog pin to measure Zener diode voltage drop
const int adt75Address = 0x48; // ADT75 I2C address

// Variables for PWM signal
const int freq = 5000;
const int pwmChannelCooling = 0;
const int pwmChannelHeating = 1;
const int resolution = 8;

// Bluetooth Display object
BDClass myDisplay;

// Function to map float values
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to read temperature from ADT75
float readTemperature() {
  Wire.beginTransmission(adt75Address);
  Wire.write(0x00); // Point to the temperature register
  Wire.endTransmission();
  Wire.requestFrom(adt75Address, 2);
  
  if (Wire.available() == 2) {
    byte msb = Wire.read();
    byte lsb = Wire.read();
    int temp = ((msb << 8) | lsb) >> 4; // 12-bit resolution
    if (temp > 2047) temp -= 4096; // Sign extend negative numbers
    return temp * 0.0625; // Convert to Celsius
  }
  return -999; // Error value
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize Bluetooth Display
  myDisplay.begin(); // Starts the Bluetooth service
  myDisplay.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_TOUCH_BASIC_DISABLE, 0, 0);
  delay(500); // Wait for display size detection

  // Initialize I2C communication
  Wire.begin();

  // Configure PWM parameters
  ledcSetup(pwmChannelCooling, freq, resolution);
  ledcAttachPin(pwmPinCooling, pwmChannelCooling);
  ledcSetup(pwmChannelHeating, freq, resolution);
  ledcAttachPin(pwmPinHeating, pwmChannelHeating);

  // Setup the display layout
  myDisplay.clearDisplay(COLOR_WHITE);
  myDisplay.setTitle("Zener Diode Tester");
}

void loop() {
  // Get the display width and height
  int displayWidth = myDisplay.getDisplayWidth();
  int displayHeight = myDisplay.getDisplayHeight();

  // Read the current temperature
  float currentTemp = readTemperature();

  // Control cooling or heating based on the current temperature
  if (currentTemp < -20) {
    ledcWrite(pwmChannelHeating, 255); // Turn on heating
    ledcWrite(pwmChannelCooling, 0);   // Turn off cooling
  } else if (currentTemp > -75) {
    ledcWrite(pwmChannelCooling, 255); // Turn on cooling
    ledcWrite(pwmChannelHeating, 0);   // Turn off heating
  }

  // Increase the voltage using PWM and read the sensor values
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(pwmChannelCooling, dutyCycle); // Write PWM signal for cooling
    ledcWrite(pwmChannelHeating, dutyCycle); // Write PWM signal for heating
    delay(1000); // Wait for the voltage to stabilize

    // Read the analog values
    int voltageSensorValue = analogRead(voltageSensorPin);
    int zenerVoltageSensorValue = analogRead(zenerVoltageSensorPin);

    // Convert the analog values to voltages
    float converterVoltage = voltageSensorValue * (3.3 / 4095.0);
    float zenerDiodeVoltage = zenerVoltageSensorValue * (3.3 / 4095.0);

    // Scale the voltages to the display resolution
    int displayX = fmap(converterVoltage, 0, 3.3, 0, displayWidth);
    int displayY = fmap(zenerDiodeVoltage, 0, 3.3, displayHeight, 0); // Inverted Y for display coordinates

    // Plot the values on the display
    myDisplay.plot(displayX, displayY, COLOR_RED);
  }

  // Reset the PWM signals
  ledcWrite(pwmChannelCooling, 0);
  ledcWrite(pwmChannelHeating, 0);

  // Add a delay before the next measurement cycle
  delay(5000);
}
