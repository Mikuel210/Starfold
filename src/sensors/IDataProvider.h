#pragma once
#ifndef STARFOLD_IDATAPROVIDER_H
#define STARFOLD_IDATAPROVIDER_H
#include "SensorData.h"

class IDataProvider {
    public:
        virtual ~IDataProvider() {}
        virtual void initialize() {}

        virtual void calibrate() {}
        virtual SensorData getData() = 0;
};

#endif
