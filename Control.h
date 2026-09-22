#pragma once

#include "IDataProvider.h"
#include "IHardwareProvider.h"
#include "FusionData.h"
#include "PID.h"
#include <algorithm>
#include <Arduino.h>

// TVC
#define KP 1
#define KI 0.25
#define KD 2

// Altitude
#define AKP 1
#define AKI 0.25
#define AKD 50

#define MIN_THROTTLE 20.0
#define MAX_THROTTLE 90.0

class Control {
    public:
        static IDataProvider* dataProvider;
        static IHardwareProvider* hardwareProvider;

        static void initialize(IDataProvider& _dataProvider, IHardwareProvider _hardwareProvider) {
            dataProvider = &_dataProvider;
            hardwareProvider = &_hardwareProvider;
        }

        static void targetAltitude(double altitude) {
            altitudePID.setpoint = altitude;
        }

        static void shutdown() {
            hardwareProvider->throttleMotors(0);
            hardwareProvider->applyCorrection(Vector3(0, 0, 0));
        }

        static void update(FusionData data) {
            // TVC
            Vector3 correction;
            correction.x = xPID.getCorrection(data.orientation.x);
            correction.y = yPID.getCorrection(data.orientation.y);
            correction.z = getRollRate(data.orientation.z) * KD;
            hardwareProvider->applyCorrection(correction);

            // Throttle
            double altitudeCorrection = altitudePID.getCorrection(data.altitude);
            double throttlePercentage = dmap(altitudeCorrection, -100, 100, MIN_THROTTLE, MAX_THROTTLE);
            throttlePercentage = std::clamp(throttlePercentage, MIN_THROTTLE, MAX_THROTTLE);
            hardwareProvider->throttleMotors(throttlePercentage);
        }

    private:
        static double previousRoll, previousTime;
        static PID xPID, yPID, altitudePID;

        static double getRollRate(double roll) {
            unsigned long currentTime = micros();

            // Get rate with 360 to 0 warping
            double rate = fmodf(roll - previousRoll + 540.0, 360.0);
            if (rate < 0) rate += 360.0;
            rate -= 180.0;

            rate /= previousTime - currentTime;
            rate *= 100000; // Micros to seconds

            previousRoll = roll;
            previousTime = currentTime;

            return rate;
        }

        static double dmap(double x, double in_min, double in_max, double out_min, double out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
};
