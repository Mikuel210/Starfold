#pragma once

#include "IHardwareProvider.h"
#include "FusionData.h"
#include "PID.h"

#define KP 1
#define KI 0.25f
#define KD 2

class Control {
  public:
    Control(IHardwareProvider& hardwareProvider_) : hardwareProvider(hardwareProvider_) {}

    void update(FusionData data) {
      Vector3 correction;
      correction.x = xPID.getCorrection(data.orientation.x);
      correction.y = yPID.getCorrection(data.orientation.y);
      correction.z = zPID.getCorrection(data.orientation.z);

      hardwareProvider.applyCorrection(correction);
    }

  private:
    IHardwareProvider& hardwareProvider;

    PID xPID = PID(KP, KI, KD);
    PID yPID = PID(KP, KI, KD);
    PID zPID = PID(0, 0, KD);
};