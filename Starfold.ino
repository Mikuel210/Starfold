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
  Plotter::setLimits(-180, 180);

  SensorData sensorData = dataProvider.getData();
  FusionData fusionData = fusion.getData(sensorData);
  
  control.update(fusionData);
  /*Plotter::plot(fusionData.orientation.x); // at -90 both ways are less negative
  Plotter::plot(fusionData.orientation.y); // Y goes up Z goes down or vice versa for X rotation
  Plotter::plot(fusionData.orientation.z);*/
  Plotter::endPlot();

  unsigned long usEnd = micros();
  delayMicroseconds(20000 - (usEnd - usStart));
}