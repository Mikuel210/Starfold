#pragma once
#include "Vector3.h"
#include <Arduino.h>

class IHardwareProvider {
    public:
        virtual ~IHardwareProvider() {}
        virtual void initialize() {}

        virtual void applyCorrection(Vector3 correction) {}
        virtual void throttleMotors(double throttlePercentage) {}
        virtual void deployLegs(bool deploy = true) {}

        virtual void updateWiggle(unsigned long timeLeftMillis) {}
        virtual void updateCountdown(unsigned long timeLeftMillis) {}

        virtual void lightLed(int r, int g, int b) {}
        virtual void writeBuzzer(uint8_t value) {}
};
