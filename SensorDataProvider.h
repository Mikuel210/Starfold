#pragma once
#include "IDataProvider.h"
#include "Plotter.h"
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <FastIMU.h>

#define IMU_ADDRESS 0x68
#define BMS_1 36
#define BMS_2 39

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

            data.accel_mps2 = readAccelerometer();
            data.gyro_radps = readGyroscope();
            data.magnetometer = readMagnetometer();

            if (tfI2C.getData(tfDistance_cm, tfAddress)) {
                data.lidarAvaliable = true;
                data.distance_cm = tfDistance_cm;
            } else {
                data.lidarAvaliable = false;
                #ifdef DEBUG
                tfI2C.printStatus();
                #endif
            }

            data.voltage1_v = analogRead(BMS_1) / 4023.0 * 3.3 * 2.0;
            data.voltage2_v = analogRead(BMS_2) / 4023.0 * 3.3 * 2.0;
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
            gyroscope.x = gyroData.gyroX / RAD_TO_DEG;
            gyroscope.y = gyroData.gyroY / RAD_TO_DEG;
            gyroscope.z = gyroData.gyroZ / RAD_TO_DEG;

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
        int16_t tfDistance_cm;
        int16_t tfAddress = TFL_DEF_ADR;
};
