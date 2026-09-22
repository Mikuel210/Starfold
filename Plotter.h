#pragma once
#undef DEBUG

class Plotter {
    public:
        static void plot(double number) {
            #ifdef DEBUG
            Serial.print(number);
            Serial.print(", ");
            #endif
        }

        static void setLimits(double lowerEnd, double upperEnd) {
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
