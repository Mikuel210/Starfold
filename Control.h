#pragma once

#include "IHardwareProvider.h"
#include "FusionData.h"
#include "PID.h"
#include "Plotter.h"
#include <algorithm>

// TVC
#define KP 1
#define KI 0.25f
#define KD 2

// Altitude
#define AKP 1
#define AKI 0.25f
#define AKD 50

#define MIN_THROTTLE 50
#define MAX_THROTTLE 230

class Control {
  public:
    Control(IHardwareProvider& hardwareProvider_) : hardwareProvider(hardwareProvider_) {}

    void update(FusionData data) {
      Vector3 correction;
      correction.x = xPID.getCorrection(data.orientation.x);
      correction.y = yPID.getCorrection(data.orientation.y);
      correction.z = getRollRate(data.orientation.z) * KD;

      hardwareProvider.applyCorrection(correction);

      float altitudeCorrection = altitudePID.getCorrection(data.altitude);
      int throttle = map(altitudeCorrection, -100, 100, MIN_THROTTLE, MAX_THROTTLE);
      throttle = std::clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);
      hardwareProvider.throttleMotors(throttle);
    }

  private:
    IHardwareProvider& hardwareProvider;
    float previousRoll, previousTime;

    PID xPID = PID(KP, KI, KD);
    PID yPID = PID(KP, KI, KD);
    PID altitudePID = PID(AKP, AKI, AKD, 5, 0.1f);

    float getRollRate(float roll) {
      unsigned long currentTime = micros();

      // Get rate with 360 to 0 warping
      float rate = fmodf(roll - previousRoll + 540.0f, 360.0f);
      if (rate < 0) rate += 360.0f;
      rate -= 180.0f;

      rate /= previousTime - currentTime;
      rate *= 100000; // Micros to seconds

      Serial.print("   "); Plotter::plot(rate); Serial.print("   ");

      previousRoll = roll;
      previousTime = currentTime;

      return rate;
    }
};