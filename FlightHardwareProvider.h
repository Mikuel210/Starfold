#pragma once
#include "IHardwareProvider.h"
#include "Plotter.h"
#include <ESP32Servo.h>
#include <algorithm>

// Actuators
#define TVC_LIMIT 45.0f
#define X_PLUS 26
#define X_MINUS 25
#define Y_PLUS 33
#define Y_MINUS 32

#define ESC 18
#define LEG 4

// State indication
#define COUNTDOWN_FREQUENCY 220
#define COUNTDOWN_BEEP_SECONDS 0.5f

#define LED_R 2
#define LED_G 19
#define LED_B 23
#define BUZZER 27

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

    void applyCorrection(Vector3 correction) override {
      XPlus.write(std::clamp(correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      XMinus.write(std::clamp(-correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      YPlus.write(std::clamp(correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
      YMinus.write(std::clamp(-correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
    }

    void throttleMotors(float throttlePercentage) override {
      int microseconds = (int)(throttlePercentage * 10 + 1000);
      microseconds = std::clamp(microseconds, 1000, 2000);

      // Esc.writeMicroseconds(microseconds);
      Leg.write(map(microseconds, 1000, 2000, 0, 180));
    }

    void deployLegs(bool deploy = true) override {
      Leg.write(deploy ? 0 : 90);
    }

    
    void updateWiggle(unsigned long timeLeftMillis) override {
      // TODO
      // bambulab sound <3
    }

    void updateCountdown(unsigned long timeLeftMillis) override {
      if (timeLeftMillis > 10'000) return; // Start at T-10 i thinkmaybe

      if (timeLeftMillis % 1000 < COUNTDOWN_BEEP_SECONDS * 1000.0f)
        toneBuzzer(COUNTDOWN_FREQUENCY);
      else
        stopBuzzer();
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