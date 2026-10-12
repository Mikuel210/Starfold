#pragma once
#include "src/Vector3.h"

struct FusionData {
    Vector3 orientation_deg;
    double altitude_mm;
    bool underVoltage;
};
