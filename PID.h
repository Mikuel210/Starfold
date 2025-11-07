#pragma once

class PID {
  public:
    float Kp, Ki, Kd, setpoint;
    PID(float Kp_, float Ki_, float Kd_, float setpoint_ = 0) : Kp(Kp_), Ki(Ki_), Kd(Kd_), setpoint(setpoint_) {}

    float getCorrection(float input) {
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - previousTime;
        
        float error = setpoint - input;
        errorIntegral += error * elapsedTime;
        float errorRate = (error - lastError) / elapsedTime;
  
        lastError = error;
        previousTime = currentTime;
 
        return Kp * error + Ki * errorIntegral + Kd * errorRate;
    }

  private:
    float lastError, errorIntegral, previousTime;
};