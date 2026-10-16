#pragma once
#ifndef STARFOLD_CONTROL_H
#define STARFOLD_CONTROL_H
#include "../Globals.h"
#include "../sensors/IDataProvider.h"
#include "../hardware/IHardwareProvider.h"
#include "../fusion/FusionData.h"
#include "../Plotter.h"
#include "ControlData.h"
#include "PID.h"
#include <algorithm>
#include <Arduino.h>

class Control {
    public:
        static IDataProvider* dataProvider;
        static IHardwareProvider* hardwareProvider;

        static void initialize(IDataProvider& _dataProvider, IHardwareProvider& _hardwareProvider) {
            dataProvider = &_dataProvider;
            hardwareProvider = &_hardwareProvider;
        }

        static void targetAltitude_mm(double altitude) {
            altitudePID.setpoint = altitude;
        }

        static ControlData getData(FusionData data) {
            ControlData controlData;

            // TVC
            PIDData xData = xPID.getData(data.orientation_deg.x);
            PIDData yData = xPID.getData(data.orientation_deg.y);
            double zRate = getRollRate(data.orientation_deg.z) * RKD;

            controlData.xPid_deg = xData;
            controlData.yPid_deg = yData;
            controlData.correction_deg = { xData.r, yData.r, zRate };

            // Throttle
            PIDData altitudeData = altitudePID.getData(data.altitude_mm);
            double throttlePercentage = dmap(altitudeData.r, -1000, 1000, MIN_THROTTLE, MAX_THROTTLE);
            throttlePercentage = std::clamp(throttlePercentage, MIN_THROTTLE, MAX_THROTTLE);

            controlData.altitudePid_mm = altitudeData;
            controlData.throttle_01 = throttlePercentage / 100.0;
            return controlData;
        }

        static ControlData shutdown(FusionData data) {
            hardwareProvider->throttleMotors(0);
            hardwareProvider->applyCorrection(Vector3(0, 0, 0));
            return getData(data);
        }

        static ControlData update(FusionData data) {
            ControlData controlData = getData(data);
            hardwareProvider->applyCorrection(controlData.correction_deg);
            hardwareProvider->throttleMotors(controlData.throttle_01);
            return controlData;
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
            rate /= (currentTime - previousTime) / 10'000.0;

            previousRoll = roll;
            previousTime = currentTime;
            return rate;
        }

        static double dmap(double x, double in_min, double in_max, double out_min, double out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
};

#endif
