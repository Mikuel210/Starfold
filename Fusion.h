#pragma once
#include "Vector3.h"
#include "SensorData.h"
#include "FusionData.h"
#include "AltitudeKF.h"
#include "Plotter.h"
#include <SensorFusion.h>

#define LIDAR_ALPHA 0.2
#define VOLTAGE_THRESHOLD_V 3.4
#define G 9.807

class Fusion {
    public:
        static FusionData getData(SensorData sensorData, double dt_s) {
            FusionData fusionData;
            deltat_s = fusion.deltatUpdate();

            fusion.MahonyUpdate(
                sensorData.gyro_radps.x, sensorData.gyro_radps.y, sensorData.gyro_radps.z,
                sensorData.accel_mps2.x, sensorData.accel_mps2.y, sensorData.accel_mps2.z,
                sensorData.magnetometer.x, sensorData.magnetometer.y, sensorData.magnetometer.z, deltat_s
            );

            // Z up/roll
            fusionData.orientation_deg.x = fusion.getPitch();
            fusionData.orientation_deg.y = fusion.getRoll();
            fusionData.orientation_deg.z = fusion.getYaw();

            // Transform to world space
            double x_rad = fusionData.orientation_deg.x / RAD_TO_DEG;
            double y_rad = fusionData.orientation_deg.y / RAD_TO_DEG;
            double correctedDistance_mm = sensorData.distance_cm * 10.0 * cos(x_rad) * cos(y_rad);

            // TODO: Transform to world space
            double rocketAccelZ_mmps2 = (sensorData.accel_mps2.z - G) * 1000;

            // Altitude Kalman Filter
            kf.predict(rocketAccelZ_mmps2, dt_s);
            if (sensorData.lidarAvaliable) kf.update(correctedDistance_mm);

            fusionData.altitude_mm = kf.s.position;
            previousAltitude_mm = fusionData.altitude_mm;

            // BMS
            fusionData.underVoltage = getVoltageData(sensorData).underVoltage;

            // Debug
            Plotter::setLimits(-360, 360);
            Plotter::plot("ax", sensorData.accel_mps2.x);
            Plotter::plot("ay", sensorData.accel_mps2.y);
            Plotter::plot("az", sensorData.accel_mps2.z);
            Plotter::plot("x", fusionData.orientation_deg.x);
            Plotter::plot("y", fusionData.orientation_deg.y);
            Plotter::plot("z", fusionData.orientation_deg.z);
            Plotter::endPlot();

            return fusionData;
        }

        static FusionData getVoltageData(SensorData sensorData) {
            FusionData fusionData;

            fusionData.underVoltage = sensorData.voltage1_v < VOLTAGE_THRESHOLD_V
                || sensorData.voltage2_v < VOLTAGE_THRESHOLD_V;

            return fusionData;
        }

    private:
        static double deltat_s, previousAltitude_mm;
        static AltitudeKF kf;
        static SF fusion;
};
