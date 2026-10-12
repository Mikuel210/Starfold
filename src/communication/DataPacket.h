#pragma once
#ifndef STARFOLD_DATAPACKET_H
#define STARFOLD_DATAPACKET_H
#include "../sensors/SensorData.h"
#include "../fusion/FusionData.h"
#include "../State.h"

struct DataPacket {
    State state;
    SensorData sensorData;
    FusionData fusionData;
};

#endif
