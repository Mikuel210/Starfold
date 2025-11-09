#pragma once
// #define DEBUG

class Debug {
  public:
    static void plot(float number) {
      #ifdef DEBUG
      Serial.print(number);
      Serial.print(", ");
      #endif
    }

    static void setLimits(float lowerEnd, float upperEnd) {
      #ifdef DEBUG
      plot(lowerEnd);
      plot(upperEnd);
      #endif
    }

    static void endPlot() {
      #ifdef DEBUG
      Serial.println();
      #endif
    }
};