#pragma once
#ifndef STARFOLD_DATAPACKET_H
#define STARFOLD_DATAPACKET_H
#include "src/State.h"
#include "src/SensorData.h"
#include "src/FusionData.h"

struct DataPacket {
    State state;
    SensorData sensorData;
    FusionData fusionData;
};

#endif
