#pragma once
#ifndef STARFOLD_DATAPACKET_H
#define STARFOLD_DATAPACKET_H
#include "../sensors/SensorData.h"
#include "../hardware/HardwareData.h"
#include "../fusion/FusionData.h"
#include "../control/ControlData.h"
#include "../State.h"

struct DataPacket {
    State state;
    SensorData sensorData;
    HardwareData hardwareData;
    FusionData fusionData;
    ControlData controlData;
};

#endif
