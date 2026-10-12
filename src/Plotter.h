#pragma once
#ifndef STARFOLD_PLOTTER_H
#define STARFOLD_PLOTTER_H
#include "Globals.h"
#include <Arduino.h>

class Plotter {
    public:
        static void plot(String name, double number) {
            #ifdef DEBUG
            Serial.print(name);
            Serial.print(":");
            Serial.print(number);
            Serial.print(", ");
            #endif
        }

        static void setLimits(double lowerEnd, double upperEnd) {
            #ifdef DEBUG
            plot("LOWER_LIMIT", lowerEnd);
            plot("UPPER_LIMIT", upperEnd);
            #endif
        }

        static void endPlot() {
            #ifdef DEBUG
            Serial.println();
            #endif
        }
};

#endif
