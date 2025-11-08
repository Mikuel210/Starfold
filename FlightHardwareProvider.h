#pragma once
#include "IHardwareProvider.h"
#include "Plotter.h"
#include <ESP32Servo.h>
#include <algorithm>

#define LED_R 2
#define LED_G 19
#define LED_B 23
#define BUZZER 27

#define TVC_LIMIT 45.0f
#define X_PLUS 26
#define X_MINUS 25
#define Y_PLUS 33
#define Y_MINUS 32

#define ESC 18
#define LEG 4

class FlightHardwareProvider : public IHardwareProvider {
  public:
    void initialize() override {
      pinMode(LED_R, OUTPUT);
      pinMode(LED_G, OUTPUT);
      pinMode(LED_B, OUTPUT);
      pinMode(BUZZER, OUTPUT);

      XPlus.attach(X_PLUS);
      XMinus.attach(X_MINUS);
      YPlus.attach(Y_PLUS);
      YMinus.attach(Y_MINUS);

      Esc.attach(ESC);
      Leg.attach(LEG);
    }

    void applyCorrection(Vector3 correction) {
      XPlus.write(std::clamp(correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      XMinus.write(std::clamp(-correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      YPlus.write(std::clamp(correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      YMinus.write(std::clamp(-correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
    }

    void throttleMotors(float throttlePercentage) {
      int microseconds = (int)(throttlePercentage * 10 + 1000);
      microseconds = std::clamp(microseconds, 1000, 2000);

      // Esc.writeMicroseconds(microseconds);
      Leg.write(map(microseconds, 1000, 2000, 0, 180));
    }

    void deployLegs(bool deploy = true) override {
      Leg.write(deploy ? 0 : 90);
    }

    void lightLed(int r, int g, int b) override {
      analogWrite(LED_R, r);
      analogWrite(LED_G, g);
      analogWrite(LED_B, b);
    }

    void toneBuzzer(float frequency) override {
      tone(BUZZER, frequency);
    }

    void stopBuzzer() override {
      noTone(BUZZER);
    }

  private:
    Servo XPlus;
    Servo XMinus;
    Servo YPlus;
    Servo YMinus;
    
    Servo Esc;
    Servo Leg;
};