#pragma once
#ifndef STARFOLD_PID_H
#define STARFOLD_PID_H
#include "../Globals.h"
#include <Arduino.h>

class PID {
    public:
        double Kp, Ki, Kd, setpoint, alpha;

        PID(double Kp_, double Ki_, double Kd_, double setpoint_ = 0, double alpha_ = 1)
            : Kp(Kp_), Ki(Ki_), Kd(Kd_), setpoint(setpoint_), alpha(alpha_) {}

        double getCorrection(double input) {
            unsigned long currentMicros = micros();
            unsigned long elapsedMicros = currentMicros - previousMicros;

            // Get error and rate
            double error = setpoint - input;
            double errorRate = (error - lastError) / elapsedMicros * 100'000.0;

            // Get integral
            errorIntegral += error * elapsedMicros / 1'000'000.0 * INTEGRAL_RATE;

            if (abs(errorIntegral) > INTEGRAL_LIMIT)
                errorIntegral = (errorIntegral > 0 ? 1 : -1) * INTEGRAL_LIMIT;

            if ((error * errorIntegral) < 0)
                errorIntegral = 0;

            // Get output
            double output = Kp * error + Ki * errorIntegral + Kd * errorRate;
            output = previousOutput + alpha * (output - previousOutput);
            previousOutput = output;

            lastError = error;
            previousMicros = currentMicros;

            return output;
        }

    private:
        double lastError, errorIntegral, previousMicros, previousOutput;
};

#endif
