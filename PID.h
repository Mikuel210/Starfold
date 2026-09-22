#pragma once
#include <Arduino.h>

#define INTEGRAL_LIMIT 10.0f
#define INTEGRAL_RATE 1.0f

class PID {
    public:
        double Kp, Ki, Kd, setpoint, alpha;

        PID(double Kp_, double Ki_, double Kd_, double setpoint_ = 0, double alpha_ = 1)
            : Kp(Kp_), Ki(Ki_), Kd(Kd_), setpoint(setpoint_), alpha(alpha_) {}

        double getCorrection(double input) {
            unsigned long currentTime = micros();
            unsigned long elapsedTime = currentTime - previousTime;

            double error = setpoint - input;
            errorIntegral += error * elapsedTime / 1000000 * INTEGRAL_RATE;
            double errorRate = (error - lastError) / elapsedTime * 100000;

            if (abs(errorIntegral) > INTEGRAL_LIMIT)
                errorIntegral = (errorIntegral > 0 ? 1 : -1) * INTEGRAL_LIMIT;

            if ((error * errorIntegral) < 0)
                errorIntegral = 0;

            double output = Kp * error + Ki * errorIntegral + Kd * errorRate;
            output = previousOutput + alpha * (output - previousOutput);
            previousOutput = output;

            lastError = error;
            previousTime = currentTime;

            return output;
        }

    private:
        double lastError, errorIntegral, previousTime, previousOutput;
};
