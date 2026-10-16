#pragma once
#ifndef STARFOLD_CONTROLDATA_H
#define STARFOLD_CONTROLDATA_H
#include "PID.h"
#include "../Vector3.h"

struct ControlData {
    Vector3 correction_deg;
    double throttle_01;
    PIDData xPid_deg;
    PIDData yPid_deg;
    PIDData altitudePid_mm;
};

#endif
