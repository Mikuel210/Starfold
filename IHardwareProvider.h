#pragma once

class IHardwareProvider {
  public:
    virtual ~IHardwareProvider() {}
    virtual void initialize() {}

    virtual void deployLegs(bool deploy = true) {}
    virtual void lightLed(int r, int g, int b) {}
    virtual void toneBuzzer(float frequency) {}
    virtual void stopBuzzer() {}
};
