#pragma once
#include "Fusion.h"

double Fusion::deltat_s = 0;
double Fusion::previousAltitude_mm = 0;
AltitudeKF Fusion::kf = AltitudeKF(10, 10, KFState()); // TODO test noise
SF Fusion::fusion = SF();
