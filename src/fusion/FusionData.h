#pragma once
#ifndef STARFOLD_FUSIONDATA_H
#define STARFOLD_FUSIONDATA_H
#include "../fusion/AltitudeKF.h"
#include "../Vector3.h"

struct FusionData {
    Vector3 orientation_deg;
    double altitude_mm;
    bool underVoltage;
    double correctedDistance_mm;
    double rocketAccelZ_mmps2;
    KFState altitudeKfState;
};

#endif
