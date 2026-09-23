#pragma once
#include "Fusion.h"

double Fusion::deltat = 0;
double Fusion::previousAltitude = 0;
AltitudeKF Fusion::kf = AltitudeKF(1, 1, KFState());
SF Fusion::fusion = SF();
