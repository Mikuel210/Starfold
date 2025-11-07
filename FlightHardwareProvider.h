#pragma once
#include "IHardwareProvider.h"
#include <ESP32Servo.h>

#define LED_R 2
#define LED_G 19
#define LED_B 23
#define BUZZER 27
#define LEG 4

class FlightHardwareProvider : public IHardwareProvider {
  public:
    void initialize() override {
      pinMode(LED_R, OUTPUT);
      pinMode(LED_G, OUTPUT);
      pinMode(LED_B, OUTPUT);
      pinMode(BUZZER, OUTPUT);

      Leg.attach(LEG);
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
    Servo Leg;
};