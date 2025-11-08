#include "SensorDataProvider.h"
#include "FlightHardwareProvider.h"
#include "Fusion.h"
#include "Control.h"
#include "Plotter.h"

// Flight profile
#define HOVER_ALTITUDE 100
#define LANDING_ALTITUDE 20 
#define HOVER_SECONDS 10
#define THROTTLE_UP_SECONDS 1

// Instances
SensorDataProvider dataProvider;
FlightHardwareProvider hardwareProvider;
Control control = Control(hardwareProvider);
Fusion fusion;

// State machine
enum State {
  IDLE,
  STARTUP,
  FLIGHT,
  ABORT,
  LANDED
};

enum State currentState = IDLE;

// Flight
unsigned long flightStartMillis;

void setup() {
  Serial.begin(250000);
  dataProvider.initialize();
  hardwareProvider.initialize();
}

void loop() { 
  unsigned long usStart = micros();

  switch (currentState) {
    case IDLE:
      hardwareProvider.lightLed(0, 0, 255);
      delay(3000);
      currentState = STARTUP;

      break;

    case STARTUP:
      hardwareProvider.lightLed(0, 255, 0);
      delay(3000);

      flightStartMillis = millis();
      currentState = FLIGHT;

      break;

    case FLIGHT: {
      hardwareProvider.lightLed(150, 0, 150);
      unsigned long elapsed = millis() - flightStartMillis;

      // Set target altitude
      if (elapsed < THROTTLE_UP_SECONDS * 1000) control.targetAltitude(0);
      else if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000) control.targetAltitude(LANDING_ALTITUDE);
      else control.targetAltitude(HOVER_ALTITUDE);

      // Update control system
      SensorData sensorData = dataProvider.getData();
      FusionData fusionData = fusion.getData(sensorData);
      control.update(fusionData);

      // Check for landing
      if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000 && fusionData.altitude <= LANDING_ALTITUDE + 1)
        currentState = LANDED;
      
      // Debug
      Plotter::setLimits(-360, 360);

      Plotter::plot(fusionData.orientation.x);
      Plotter::plot(fusionData.orientation.y);
      Plotter::plot(fusionData.orientation.z);

      if (elapsed < THROTTLE_UP_SECONDS * 1000) Plotter::plot(0);
      else if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000) Plotter::plot(LANDING_ALTITUDE);
      else Plotter::plot(HOVER_ALTITUDE);

      Plotter::endPlot();

      break;
    }

    case ABORT:
      hardwareProvider.lightLed(255, 0, 0);
      control.shutdown();

      break;

    case LANDED:
      hardwareProvider.lightLed(150, 150, 0);
      control.shutdown();

      break;
  }

  unsigned long usEnd = micros();
  long usDelay = 20000 - (usEnd - usStart);
  if (usDelay > 0) delayMicroseconds(usDelay);
}