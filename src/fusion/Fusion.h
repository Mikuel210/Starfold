#pragma once
#include "src/sensors/SensorData.h"
#include "src/fusion/FusionData.h"
#include "src/fusion/AltitudeKF.h"
#include "src/Vector3.h"
#include "src/Plotter.h"
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
            if (correctedDistance_mm < 30) correctedDistance_mm = 0;

            // Altitude Kalman Filter
            double rocketAccelZ_mmps2 = getWorldAccelZ_mmps2(sensorData);
            kf.predict(rocketAccelZ_mmps2, dt_s);
            if (sensorData.lidarAvaliable) kf.update(correctedDistance_mm);

            fusionData.altitude_mm = kf.s.position;
            previousAltitude_mm = fusionData.altitude_mm;

            // BMS
            fusionData.underVoltage = getVoltageData(sensorData).underVoltage;

            // Debug
            Plotter::plot("rd", sensorData.distance_cm * 10.0);
            Plotter::plot("d", correctedDistance_mm);
            Plotter::plot("accel", rocketAccelZ_mmps2);
            Plotter::plot("alt", fusionData.altitude_mm);
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

        static double getWorldAccelZ_mmps2(SensorData sensorData) {
            float* q = fusion.getQuat();
            float w = q[0], x = q[1], y = q[2], z = q[3];

            double ax = sensorData.accel_mps2.x;
            double ay = sensorData.accel_mps2.y;
            double az = sensorData.accel_mps2.z;

            // Third row of the body -> world rotation matrix
            double worldAccelZ_mps2 =
                2.0 * (x*z - w*y) * ax +
                2.0 * (y*z + w*x) * ay +
                (1.0 - 2.0 * (x*x + y*y)) * az;

            // Remove gravity in world space
            return (worldAccelZ_mps2 - G) * 1000 + 5;
        }
};
