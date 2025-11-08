#pragma once
#include "IDataProvider.h"
#include "mpu9250.h"
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

#define CALIBRATION_SAMPLES 100

class SensorDataProvider : public IDataProvider {
  public:
    void initialize() override {
      // Initialize IMU and LiDAR
      Wire.begin();
      Wire.setClock(400000);
      imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

      if (!imu.Begin()) { Serial.println("Error initializing communication with IMU"); }
      if (!imu.ConfigSrd(19)) { Serial.println("Error configuring IMU SRD"); }

      // Calibration
      Serial.println("Calibrating IMU");

      for (int i = 0; i < CALIBRATION_SAMPLES; i++)
      {
        if (imu.Read())
        {
          imu.new_imu_data();
          imu.new_mag_data();

          gyroscopeOffset.x += imu.gyro_x_radps();
          gyroscopeOffset.y += imu.gyro_y_radps();
          gyroscopeOffset.z += imu.gyro_z_radps();
        }

        delay(50);
      }

      gyroscopeOffset.x /= CALIBRATION_SAMPLES;
      gyroscopeOffset.y /= CALIBRATION_SAMPLES;
      gyroscopeOffset.z /= CALIBRATION_SAMPLES;
    }

    SensorData getData() override {
      SensorData data;

      if (imu.Read()) {
        /*imu.new_imu_data();
        imu.new_mag_data();*/

        data.acceleration = readAccelerometer();
        data.gyroscope = readGyroscope();
        data.magnetometer = readMagnetometer();
      }
      
      if (tfI2C.getData(tfDistance, tfAddress))
        data.distance = tfDistance;

      else tfI2C.printStatus();

      return data;
    }

  private:
    // Orientation
    bfs::Mpu9250 imu;
    Vector3 gyroscopeOffset; // Consider accelerometer offset as well

    Vector3 readAccelerometer() { 
      Vector3 acceleration;
      acceleration.x = imu.accel_x_mps2();
      acceleration.y = imu.accel_y_mps2();
      acceleration.z = imu.accel_z_mps2();

      return acceleration;
    }

    Vector3 readGyroscope() { 
      Vector3 gyroscope;
      gyroscope.x = imu.gyro_x_radps() - gyroscopeOffset.x;
      gyroscope.y = imu.gyro_y_radps() - gyroscopeOffset.y;
      gyroscope.z = imu.gyro_z_radps() - gyroscopeOffset.z;

      return gyroscope;
    }

    Vector3 readMagnetometer() { 
      Vector3 magnetometer;
      magnetometer.x = imu.mag_x_ut();
      magnetometer.y = imu.mag_y_ut();
      magnetometer.z = imu.mag_z_ut();

      return magnetometer;
    }
    
    // Distance
    TFLI2C tfI2C;
    int16_t tfDistance;
    int16_t tfAddress = TFL_DEF_ADR;
};
