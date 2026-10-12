#pragma once
#include "src/sensors/IDataProvider.h"

class SimulationDataProvider : public IDataProvider {
    public:
        void initialize() override {
            // Serial becomes communication not debugging
            // -> Remove #define DEBUG on plotter
        }

        void calibrate() override {}

        SensorData getData() override {
            SensorData data;
            return data;
        }
};
