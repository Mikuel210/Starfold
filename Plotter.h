#pragma once

class Plotter {
  public:
    static void plot(float number) {
      Serial.print(number);
      Serial.print(", ");
    }

    static void setLimits(float lowerEnd, float upperEnd) {
      plot(lowerEnd);
      plot(upperEnd);
    }

    static void endPlot() {
      Serial.println();
    }
};