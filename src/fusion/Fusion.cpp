#pragma once
#include "src/fusion/Fusion.h"

double Fusion::deltat_s = 0;
double Fusion::previousAltitude_mm = 0;
AltitudeKF Fusion::kf = AltitudeKF(25, 250, KFState());
SF Fusion::fusion = SF();
