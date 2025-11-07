#pragma once

#include "IHardwareProvider.h"
#include "FusionData.h"
#include "PID.h"
#include "Plotter.h"

class Control {
  public:
    Control(IHardwareProvider& hardwareProvider_) : hardwareProvider(hardwareProvider_) {}

    void update(FusionData data) {
      Plotter::plot(xPID.getCorrection(data.orientation.x));
    }

  private:
    IHardwareProvider& hardwareProvider;

    PID xPID = PID(1, 1, 1);
    PID yPID = PID(1, 1, 1);
    PID zPID = PID(1, 1, 1);
};