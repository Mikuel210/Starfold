#pragma once
#include "IHardwareProvider.h"

class FlightHardwareProvider : public IHardwareProvider {
  public:
    void initialize() override {
      // connection already established by simulation data
      // maybe should check if it's established already on both
    }

    void applyCorrection(Vector3 correction) override {
      /*
      XPlus.write(std::clamp(correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      XMinus.write(std::clamp(-correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      YPlus.write(std::clamp(correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      YMinus.write(std::clamp(-correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      */
    }

    void throttleMotors(float throttlePercentage) override {
      
    }

    void deployLegs(bool deploy = true) override {
      
    }

    
    void updateWiggle(unsigned long timeLeftMillis) override {
    }

    void updateCountdown(unsigned long timeLeftMillis) override {
      
    }


    void lightLed(int r, int g, int b) override {
      
    }

    void toneBuzzer(float frequency) override {
      
    }

    void stopBuzzer() override {
      
    }

  private:
    void wiggleStep(int angle) {
    
    }
};