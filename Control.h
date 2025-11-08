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
      correction.z = zPID.getCorrection(data.orientation.z);

      hardwareProvider.applyCorrection(correction);

      float altitudeCorrection = altitudePID.getCorrection(data.altitude);
      int throttle = map(altitudeCorrection, -100, 100, MIN_THROTTLE, MAX_THROTTLE);
      throttle = std::clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);

      Serial.print("    "); Plotter::plot(data.altitude);
      Serial.print("    "); Plotter::plot(altitudeCorrection);
      Serial.print("    "); Plotter::plot(throttle); Serial.print("    ");
      hardwareProvider.throttleMotors(throttle);
    }

  private:
    IHardwareProvider& hardwareProvider;

    PID xPID = PID(KP, KI, KD, 0, 0.5f);
    PID yPID = PID(KP, KI, KD, 0, 0.5f);
    PID zPID = PID(0, 0, KD, 0, 0.5f);
    PID altitudePID = PID(AKP, AKI, AKD, 5, 0.1f);
};