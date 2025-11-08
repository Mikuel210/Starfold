#pragma onces

#define INTEGRAL_LIMIT 10.0f
#define INTEGRAL_RATE 1.0f

class PID {
  public:
    float Kp, Ki, Kd, setpoint, alpha;

    PID(float Kp_, float Ki_, float Kd_, float setpoint_ = 0, float alpha_ = 1) 
      : Kp(Kp_), Ki(Ki_), Kd(Kd_), setpoint(setpoint_), alpha(alpha_) {}

    float getCorrection(float input) {
      unsigned long currentTime = micros();
      unsigned long elapsedTime = currentTime - previousTime;
      
      float error = setpoint - input;
      errorIntegral += error * elapsedTime / 1000000 * INTEGRAL_RATE;
      float errorRate = (error - lastError) / elapsedTime * 100000;

      if (fabs(errorIntegral) > INTEGRAL_LIMIT)
        errorIntegral = (errorIntegral > 0 ? 1 : -1) * INTEGRAL_LIMIT;

      if ((error * errorIntegral) < 0)
        errorIntegral = 0;

      float output = Kp * error + Ki * errorIntegral + Kd * errorRate;
      output = previousOutput + alpha * (output - previousOutput);
      previousOutput = output;

      lastError = error;
      previousTime = currentTime;

      return output;
    }

  private:
    float lastError, errorIntegral, previousTime, previousOutput;
};