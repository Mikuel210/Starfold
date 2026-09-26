#pragma once
#include "Control.h"

IDataProvider* Control::dataProvider = nullptr;
IHardwareProvider* Control::hardwareProvider = nullptr;
double Control::previousRoll = 0;
double Control::previousTime = 0;
PID Control::xPID = PID(KP, KI, KD);
PID Control::yPID = PID(KP, KI, KD);
PID Control::altitudePID = PID(AKP, AKI, AKD, 0, 0.1);
