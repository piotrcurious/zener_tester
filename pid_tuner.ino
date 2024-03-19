#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Define the connections
const int PIN_THERM = A0; // Thermistor or temperature sensor pin
const int PIN_HEATING = 5; // Heating element control pin
const int PIN_COOLING = 6; // Cooling element control pin

// Define the PID variables for heating
double Setpoint, Input, OutputHeating;
double Kp_Heating = 2.0, Ki_Heating = 5.0, Kd_Heating = 1.0;
PID myPID_Heating(&Input, &OutputHeating, &Setpoint, Kp_Heating, Ki_Heating, Kd_Heating, DIRECT);

// Define the PID variables for cooling
double OutputCooling;
double Kp_Cooling = 2.0, Ki_Cooling = 5.0, Kd_Cooling = 1.0;
PID myPID_Cooling(&Input, &OutputCooling, &Setpoint, Kp_Cooling, Ki_Cooling, Kd_Cooling, DIRECT);

// Specify the links and initial tuning parameters
double aTuneStep = 50, aTuneNoise = 5, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

// Autotune objects for heating and cooling
PID_ATune aTuneHeating(&Input, &OutputHeating);
PID_ATune aTuneCooling(&Input, &OutputCooling);

void setup() {
  // Initialize communication
  Serial.begin(9600);

  // Start with PID in manual mode
  myPID_Heating.SetMode(MANUAL);
  myPID_Cooling.SetMode(MANUAL);

  // Set the output to the desired starting frequency.
  OutputHeating = aTuneStartValue;
  OutputCooling = aTuneStartValue;
  analogWrite(PIN_HEATING, OutputHeating);
  analogWrite(PIN_COOLING, OutputCooling);

  // Setup the PID autotuner for heating
  aTuneHeating.SetNoiseBand(aTuneNoise);
  aTuneHeating.SetOutputStep(aTuneStep);
  aTuneHeating.SetLookbackSec((int)aTuneLookBack);
  aTuneHeating.SetControlType(1); // Set to PI control

  // Start the PID autotuning process for heating
  bool tuning = true;
  aTuneHeating.Cancel(); // Ensure no previous tuning is running
  while(tuning) {
    Input = analogRead(PIN_THERM);
    byte val = aTuneHeating.Runtime();
    if (val != 0) {
      tuning = false;
    }
  }

  // Store the tuning parameters for heating
  Kp_Heating = aTuneHeating.GetKp();
  Ki_Heating = aTuneHeating.GetKi();
  Kd_Heating = aTuneHeating.GetKd();

  // Setup the PID autotuner for cooling
  aTuneCooling.SetNoiseBand(aTuneNoise);
  aTuneCooling.SetOutputStep(aTuneStep);
  aTuneCooling.SetLookbackSec((int)aTuneLookBack);
  aTuneCooling.SetControlType(1); // Set to PI control

  // Start the PID autotuning process for cooling
  tuning = true;
  aTuneCooling.Cancel(); // Ensure no previous tuning is running
  while(tuning) {
    Input = analogRead(PIN_THERM);
    byte val = aTuneCooling.Runtime();
    if (val != 0) {
      tuning = false;
    }
  }

  // Store the tuning parameters for cooling
  Kp_Cooling = aTuneCooling.GetKp();
  Ki_Cooling = aTuneCooling.GetKi();
  Kd_Cooling = aTuneCooling.GetKd();

  // Apply the tuning parameters to the PID controllers
  myPID_Heating.SetTunings(Kp_Heating, Ki_Heating, Kd_Heating);
  myPID_Cooling.SetTunings(Kp_Cooling, Ki_Cooling, Kd_Cooling);

  // Switch PID controllers to automatic mode
  myPID_Heating.SetMode(AUTOMATIC);
  myPID_Cooling.SetMode(AUTOMATIC);
}

void loop() {
  // Main loop code
  // Here you would implement the logic to switch between heating and cooling based on the current temperature
}
