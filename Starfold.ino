#include "SensorDataProvider.h"
#include "FlightHardwareProvider.h"
#include "Fusion.h"
#include "Control.h"
#include "Plotter.h"
#include "BMS.h"

// Flight profile
#define HOVER_ALTITUDE 100
#define LANDING_ALTITUDE 20
#define HOVER_SECONDS 10
#define THROTTLE_UP_SECONDS 1

#define ABORT_THRESHOLD 45
#define STARTUP_SECONDS 20

// Instances
SensorDataProvider dataProvider;
FlightHardwareProvider hardwareProvider;

// State machine
enum State {
    IDLE,
    STARTUP,
    FLIGHT,
    ABORT,
    LANDED
};

enum State currentState = IDLE;
unsigned long startupStartMillis;
unsigned long flightStartMillis;

// Entry point
void setup() {
    Serial.begin(115200);
    dataProvider.initialize();
    hardwareProvider.initialize();
    Control::initialize(dataProvider, hardwareProvider);
    BMS::initialize();
}

void loop() {
    unsigned long usStart = micros();

    switch (currentState) {
        case IDLE:
            hardwareProvider.lightLed(0, 0, 255);

            // Reset state
            hardwareProvider.deployLegs(false);
            Control::shutdown();
            delay(3000);

            startupStartMillis = millis();
            currentState = STARTUP;
            break;

        case STARTUP: {
            hardwareProvider.lightLed(0, 150, 150);
            unsigned long elapsed = millis() - startupStartMillis;
            long timeLeft = STARTUP_SECONDS * 1000 - elapsed;

            dataProvider.calibrate();
            hardwareProvider.updateWiggle(timeLeft);
            hardwareProvider.updateCountdown(timeLeft);

            // Stabilize angle
            SensorData sensorData = dataProvider.getData();
            FusionData fusionData = Fusion::getData(sensorData);

            if (timeLeft <= 0) {
                hardwareProvider.writeBuzzer(LOW);
                flightStartMillis = millis();
                currentState = FLIGHT;
            }

            break;
        }

        case FLIGHT: {
            hardwareProvider.lightLed(150, 0, 150);
            unsigned long elapsed = millis() - flightStartMillis;

            // Set altitude and deploy legs
            if (elapsed < THROTTLE_UP_SECONDS * 1000) {
                Control::targetAltitude(0);
            } else if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000) {
                Control::targetAltitude(LANDING_ALTITUDE);
                hardwareProvider.deployLegs();
            } else {
                Control::targetAltitude(HOVER_ALTITUDE);
            }

            // Update control system
            SensorData sensorData = dataProvider.getData();
            FusionData fusionData = Fusion::getData(sensorData);
            Control::update(fusionData);

            // Check for landing
            if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000 && fusionData.altitude <= LANDING_ALTITUDE + 1)
                currentState = LANDED;

            // Check abort
            if (abs(fusionData.orientation.x) > ABORT_THRESHOLD || abs(fusionData.orientation.y) > ABORT_THRESHOLD)
                currentState = ABORT;

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
            Control::shutdown();
            break;

        case LANDED:
            hardwareProvider.lightLed(0, 255, 0);
            Control::shutdown();
            break;
    }

    unsigned long usEnd = micros();
    long usDelay = 20000 - (usEnd - usStart);
    if (usDelay > 0) delayMicroseconds(usDelay);
}
