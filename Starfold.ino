#include "SensorDataProvider.h"
#include "FlightHardwareProvider.h"
#include "Fusion.h"
#include "Control.h"
#include "Plotter.h"

SensorDataProvider dataProvider;
FlightHardwareProvider hardwareProvider;
Fusion fusion;
Control control = Control(hardwareProvider);
               
// Base note settings
float baseDistance = 5.0;   // cm
float baseFrequency = 220;  // Hz (A4)
float octaveStep = 50.0;    // cm per octave

// Filtering
float filteredDistance = baseDistance;
float alpha = 0.3; // Smoothing factor (0.0 - 1.0, lower = smoother)

// Function: Convert distance to frequency
float distanceToFrequency(float distance) {
    float distanceDiff = distance - baseDistance;
    float octaves = distanceDiff / octaveStep;
    float frequency = baseFrequency * pow(2, octaves);
    return frequency;
}

void setup() {
  Serial.begin(250000);
  dataProvider.initialize();
  hardwareProvider.initialize();
}

void loop() { 
  unsigned long usStart = micros();
  Plotter::setLimits(-180, 180);

  SensorData sensorData = dataProvider.getData();
  FusionData fusionData = fusion.getData(sensorData);
  
  control.update(fusionData);
  Plotter::plot(fusionData.orientation.x);
  Plotter::endPlot();

  hardwareProvider.lightLed(sensorData.gyroscope.x > 1 ? 255 : 0, sensorData.gyroscope.y > 1 ? 255 : 0, sensorData.gyroscope.z > 1 ? 255 : 0);

  if (sensorData.distance < 100)
    hardwareProvider.toneBuzzer(distanceToFrequency(sensorData.distance));
  else
    hardwareProvider.stopBuzzer();
  
  hardwareProvider.deployLegs(sensorData.distance < 100);

  unsigned long usEnd = micros();
  delayMicroseconds(20000 - (usEnd - usStart));
}