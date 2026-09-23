#pragma once
#include "Vector3.h"

struct SensorData {
    Vector3 accel_mps2;
    Vector3 gyro_radps;
    Vector3 magnetometer;
    double distance_cm;

    double voltage1_v;
    double voltage2_v;
};
