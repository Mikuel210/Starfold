#pragma once
#include "Vector3.h"
#include "SensorData.h"
#include "FusionData.h"
#include <SensorFusion.h>

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

      // TODO: Fuse altitude
      fusionData.altitude = sensorData.distance;
      return fusionData;
    }

  private:
    float deltat;
    SF fusion;
};