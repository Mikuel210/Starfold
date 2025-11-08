#pragma once
#include "SensorData.h"

class IDataProvider {
  public:
    virtual ~IDataProvider() {}
    virtual void initialize() {}
    
    virtual void calibrate() {}
    virtual SensorData getData() = 0;
};
