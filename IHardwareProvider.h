#pragma once
#include "Vector3.h"

class IHardwareProvider {
  public:
    virtual ~IHardwareProvider() {}
    virtual void initialize() {}

    virtual void applyCorrection(Vector3 correction) {}
    virtual void throttleMotors(float throttlePercentage) {}
    virtual void deployLegs(bool deploy = true) {}
    
    virtual void lightLed(int r, int g, int b) {}
    virtual void toneBuzzer(float frequency) {}
    virtual void stopBuzzer() {}
};
