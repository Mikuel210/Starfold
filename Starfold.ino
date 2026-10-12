#include "src/Globals.h"
#include "src/communication/Starhub.h"
#include "src/fusion/Fusion.h"
#include "src/control/Control.h"
#include "src/Plotter.h"
#include "src/State.h"
#include "src/BMS.h"

// Instances
#ifdef SIMULATION
    #include "src/sensors/SimulationDataProvider.h"
    #include "src/hardware/SimulationHardwareProvider.h"
    SimulationDataProvider dataProvider;
    SimulationHardwareProvider hardwareProvider;
#else
    #include "src/sensors/SensorDataProvider.h"
    #include "src/hardware/FlightHardwareProvider.h"
    SensorDataProvider dataProvider;
    FlightHardwareProvider hardwareProvider;
#endif

// State machine
enum State currentState = IDLE;
unsigned long startupStartMillis;
unsigned long flightStartMillis;
double dt_s;

// Entry point
void setup() {
    Serial.begin(115200);
    Starhub::initialize();

    dataProvider.initialize();
    hardwareProvider.initialize();
    Control::initialize(dataProvider, hardwareProvider);
    BMS::initialize();
}

void loop() {
    unsigned long usStart = micros();
    SensorData sensorData = dataProvider.getData();
    FusionData fusionData = Fusion::getData(sensorData, dt_s);

    switch (currentState) {
        case IDLE:
            hardwareProvider.lightLed(0, 0, 255);

            // Reset state
            hardwareProvider.deployLegs(false);
            Control::shutdown();

            if (Starhub::abortRequested) {
                currentState = ABORT;
                break;
            }

            if (Starhub::startupRequested) {
                dataProvider.calibrate();
                startupStartMillis = millis();
                currentState = STARTUP;
            }

            break;

        case STARTUP: {
            hardwareProvider.lightLed(0, 150, 150);
            unsigned long elapsed = millis() - startupStartMillis;
            long timeLeft = STARTUP_SECONDS * 1000 - elapsed;

            hardwareProvider.updateWiggle(timeLeft);
            hardwareProvider.updateCountdown(timeLeft);

            if (Starhub::abortRequested) {
                currentState = ABORT;
                break;
            }

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
                Control::targetAltitude_mm(0);
            } else if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000) {
                Control::targetAltitude_mm(LANDING_ALTITUDE_MM);
                hardwareProvider.deployLegs();
            } else {
                Control::targetAltitude_mm(HOVER_ALTITUDE_MM);
            }

            // Update control system
            Control::update(fusionData);

            // Check for landing
            if (elapsed > (THROTTLE_UP_SECONDS + HOVER_SECONDS) * 1000 && fusionData.altitude_mm <= LANDING_ALTITUDE_MM + 1)
                currentState = LANDED;

            // Check abort
            if (abs(fusionData.orientation_deg.x) > ABORT_THRESHOLD_DEG
                || abs(fusionData.orientation_deg.y) > ABORT_THRESHOLD_DEG
                || Starhub::abortRequested)
                currentState = ABORT;

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

    // Starhub communication
    Starhub::update();
    Starhub::notifyPacket({currentState, sensorData, fusionData});

    // Delay to run at HZ
    unsigned long usEnd = micros();
    long usDelay = 1'000'000.0 / HZ - (usEnd - usStart);

    if (usDelay <= 0) {
        dt_s = (usEnd - usStart) / 1'000'000.0;
        return;
    }

    dt_s = (usEnd - usStart + usDelay) / 1'000'000.0;
    delayMicroseconds(usDelay);
}
