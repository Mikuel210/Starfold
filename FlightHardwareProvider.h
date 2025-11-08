#pragma once
#include "IHardwareProvider.h"
#include "Plotter.h"
#include <ESP32Servo.h>

#define LED_R 2
#define LED_G 19
#define LED_B 23
#define BUZZER 27

#define X_PLUS 26
#define X_MINUS 25
#define Y_PLUS 33
#define Y_MINUS 32
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
      Leg.attach(LEG);
    }

    void applyCorrection(Vector3 correction) {
      int angle = 0; // Vertical
      XPlus.write(correction.x + correction.z + angle);
      XMinus.write(-correction.x + correction.z + angle);
      YPlus.write(correction.y + correction.z + angle);
      YMinus.write(-correction.y + correction.z + angle);
    }

    void throttleMotors(int throttle) {
      // TODO
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
    Servo Leg;
};