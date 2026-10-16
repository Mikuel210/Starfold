#pragma once
#ifndef STARFOLD_HARDWAREDATA_H
#define STARFOLD_HARDWAREDATA_H
#include "../Vector3.h"

struct HardwareData {
    double tvc_deg[4];
    double leg_deg;
    int throttle_us;
};

#endif
