#pragma once
#include "Vector3.h"
#include <Arduino.h>

class IHardwareProvider {
    public:
        virtual ~IHardwareProvider() {}
        virtual void initialize() {}

        virtual void applyCorrection(Vector3 correction_deg) {}
        virtual void throttleMotors(double throttlePercentage) {}
        virtual void deployLegs(bool deploy = true) {}

        virtual void updateWiggle(unsigned long timeLeft_ms) {}
        virtual void updateCountdown(unsigned long timeLeft_ms) {}

        virtual void lightLed(int r, int g, int b) {}
        virtual void writeBuzzer(uint8_t value) {}
};
