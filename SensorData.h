#pragma once
#include "Vector3.h"

struct SensorData {
    Vector3 acceleration;
    Vector3 gyroscope;
    Vector3 magnetometer;
    double distance;

    double voltage1;
    double voltage2;
};
