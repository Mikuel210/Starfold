#pragma once
#ifndef STARFOLD_FUSIONDATA_H
#define STARFOLD_FUSIONDATA_H
#include "../Vector3.h"

struct FusionData {
    Vector3 orientation_deg;
    double altitude_mm;
    bool underVoltage;
};

#endif
