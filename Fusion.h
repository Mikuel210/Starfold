#pragma once
#include "Vector3.h"
#include "SensorData.h"
#include "FusionData.h"
#include "AltitudeKF.h"
#include "Plotter.h"
#include <SensorFusion.h>

#define LIDAR_ALPHA 0.2f
#define VOLTAGE_THRESHOLD_V 3.4

class Fusion {
    public:
        static FusionData getData(SensorData sensorData, double dt) {
            FusionData fusionData;
            deltat_s = fusion.deltat_sUpdate();

            fusion.MahonyUpdate(
                sensorData.gyro_radps.x, sensorData.gyro_radps.y, sensorData.gyro_radps.z,
                sensorData.accel_mps2.x, sensorData.accel_mps2.y, sensorData.accel_mps2.z,
                sensorData.magnetometer.x, sensorData.magnetometer.y, sensorData.magnetometer.z, deltat_s
            );

            // Z up
            fusionData.orientation_deg.x = fusion.getPitch();
            fusionData.orientation_deg.y = fusion.getRoll();
            fusionData.orientation_deg.z = fusion.getYaw();

            // Make 0 = upright
            if (fusionData.orientation_deg.y > 0) fusionData.orientation_deg.y -= 180;
            else fusionData.orientation_deg.y += 180;

            // Altitude Kalman Filter
            kf.predict(sensorData.accel_mps2.y * 1000, dt);
            kf.update(sensorData.distance_cm * 10);

            fusionData.altitude_mm = kf.s.position;
            previousAltitude_mm = fusionData.altitude_mm;

            // BMS
            fusionData.underVoltage = sensorData.voltage1_v < VOLTAGE_THRESHOLD_V || sensorData.voltage2_v < VOLTAGE_THRESHOLD_V;
            return fusionData;
        }

    private:
        static double deltat_s, previousAltitude_mm;
        static AltitudeKF kf;
        static SF fusion;
};
