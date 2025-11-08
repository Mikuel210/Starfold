#pragma once
#include "IDataProvider.h"
#include "Plotter.h"
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <FastIMU.h>

#define IMU_ADDRESS 0x68
#define G -9.807f
#define DEG2RAD 0.01745329251f

class SensorDataProvider : public IDataProvider {
  public:
    void initialize() override {
      // Initialize IMU and LiDAR
      Wire.begin();
      Wire.setClock(400000);

      IMU.setIMUGeometry(7);
      int error = IMU.init({ 0 }, IMU_ADDRESS);

      if (error != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(error);
        while (true) {}
      }
    }

    void calibrate() override {
      // TODO: Update using FastIMU example https://github.com/LiquidCGS/FastIMU/blob/main/examples/Calibrated_sensor_output/Calibrated_sensor_output.ino
    }

    SensorData getData() override {
      SensorData data;
      IMU.update();

      data.acceleration = readAccelerometer();
      data.gyroscope = readGyroscope();
      data.magnetometer = readMagnetometer();
      
      if (tfI2C.getData(tfDistance, tfAddress))
        data.distance = tfDistance;

      else tfI2C.printStatus();

      return data;
    }

  private:
    // Orientation
    MPU9250 IMU;
    AccelData accelData;
    GyroData gyroData;
    MagData magData;

    Vector3 readAccelerometer() { 
      Vector3 acceleration;
      IMU.getAccel(&accelData);
      acceleration.x = accelData.accelX * G;
      acceleration.y = accelData.accelY * G;
      acceleration.z = accelData.accelZ * G;

      return acceleration;
    }

    Vector3 readGyroscope() { 
      Vector3 gyroscope;
      IMU.getGyro(&gyroData);
      gyroscope.x = gyroData.gyroX * DEG2RAD;
      gyroscope.y = gyroData.gyroY * DEG2RAD;
      gyroscope.z = gyroData.gyroZ * DEG2RAD;

      return gyroscope;
    }

    Vector3 readMagnetometer() { 
      Vector3 magnetometer;
      IMU.getMag(&magData);
      magnetometer.x = magData.magX;
      magnetometer.y = magData.magY;
      magnetometer.z = magData.magZ;

      return magnetometer;
    }
    
    // Distance
    TFLI2C tfI2C;
    int16_t tfDistance;
    int16_t tfAddress = TFL_DEF_ADR;
};
