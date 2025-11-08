#include "SensorDataProvider.h"
#include "FlightHardwareProvider.h"
#include "Fusion.h"
#include "Control.h"
#include "Plotter.h"

SensorDataProvider dataProvider;
FlightHardwareProvider hardwareProvider;
Control control = Control(hardwareProvider);
Fusion fusion;

void setup() {
  Serial.begin(250000);
  dataProvider.initialize();
  hardwareProvider.initialize();
}

void loop() { 
  unsigned long usStart = micros();
  Plotter::setLimits(-360, 360);

  SensorData sensorData = dataProvider.getData();
  FusionData fusionData = fusion.getData(sensorData);
  
  control.update(fusionData);
  Plotter::plot(fusionData.orientation.x);
  Plotter::plot(fusionData.orientation.y);
  Plotter::plot(fusionData.orientation.z);
  Plotter::endPlot();

  unsigned long usEnd = micros();
  delayMicroseconds(20000 - (usEnd - usStart));
}