#pragma once
#include "Vector3.h"
#include "SensorData.h"
#include "FusionData.h"
#include "Plotter.h"
#include <SensorFusion.h>

#define LIDAR_ALPHA 0.2f

class Fusion {
  public:
    FusionData getData(SensorData sensorData) {
      FusionData fusionData;
      deltat = fusion.deltatUpdate();

      fusion.MadgwickUpdate(
        sensorData.gyroscope.x, sensorData.gyroscope.y, sensorData.gyroscope.z, 
        sensorData.acceleration.x, sensorData.acceleration.y, sensorData.acceleration.z, 
        sensorData.magnetometer.x, sensorData.magnetometer.y, sensorData.magnetometer.z, deltat
      );

      fusionData.orientation.x = fusion.getPitch();
      fusionData.orientation.y = fusion.getYaw();
      fusionData.orientation.z = fusion.getRoll(); // Z up

      Plotter::plot();

      // TODO: Fuse altitude with accelerometer, LiDAR and angle
      fusionData.altitude = previousAltitude + LIDAR_ALPHA * (sensorData.distance - previousAltitude);
      previousAltitude = fusionData.altitude;

      return fusionData;
    }

  private:
    float deltat, previousAltitude;
    SF fusion;
};