#pragma once
#ifndef STARFOLD_FLIGHTHARDWAREPROVIDER_H
#define STARFOLD_FLIGHTHARDWAREPROVIDER_H
#include "../Globals.h"
#include "../Plotter.h"
#include "IHardwareProvider.h"
#include <ESP32Servo.h>
#include <algorithm>

// Actuators
#define TVC_LIMIT 45.0
#define X_PLUS 26
#define X_MINUS 25
#define Y_PLUS 33
#define Y_MINUS 32
#define ESC 18
#define LEG 4

// State indication
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

            #ifndef UNPOWERED_TVC
            XPlus.attach(X_PLUS);
            XMinus.attach(X_MINUS);
            YPlus.attach(Y_PLUS);
            YMinus.attach(Y_MINUS);
            #endif

            #ifndef UNPOWERED_ESC
            Leg.attach(LEG);
            #endif

            Esc.attach(ESC);
        }

        void applyCorrection(Vector3 correction) override {
            #ifndef UNPOWERED_TVC
            XPlus.write(std::clamp(-correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
            XMinus.write(std::clamp(correction.x + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
            YPlus.write(std::clamp(-correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
            YMinus.write(std::clamp(correction.y + correction.z + 90, 90 - TVC_LIMIT, 90 + TVC_LIMIT));
            #endif
        }

        void throttleMotors(double throttlePercentage) override {
            #ifndef UNPOWERED_ESC
            int microseconds = (int)(throttlePercentage * 10 + 1000);
            microseconds = std::clamp(microseconds, 1000, 2000);

            Esc.writeMicroseconds(microseconds);
            #else
            Esc.writeMicroseconds(1000);
            #endif
        }

        void deployLegs(bool deploy = true) override {
            #ifndef UNPOWERED_ESC
            Leg.write(deploy ? 0 : 90);
            #endif
        }

        void updateWiggle(unsigned long timeLeftMillis) override {
            #ifndef UNPOWERED_TVC
            unsigned long wiggleStart = WIGGLE_T_MINUS * 1000;
            if (timeLeftMillis > wiggleStart) return;

            // Wiggle sequence
            if      (timeLeftMillis < wiggleStart - WIGGLE_STEP * 11'000 - WIGGLE_INTERVAL * 1000) wiggleStep(0);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 10'000 - WIGGLE_INTERVAL * 1000) wiggleStep(-WIGGLE_ANGLE);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 9000   - WIGGLE_INTERVAL * 1000) wiggleStep(0);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 8000   - WIGGLE_INTERVAL * 1000) wiggleStep(WIGGLE_ANGLE);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 7000   - WIGGLE_INTERVAL * 1000) wiggleStep(0);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 6000   - WIGGLE_INTERVAL * 1000) wiggleStep(-WIGGLE_ANGLE);

            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 6000) wiggleStep(0);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 5000) wiggleStep(WIGGLE_ANGLE);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 4000) wiggleStep(0);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 3000) wiggleStep(-WIGGLE_ANGLE);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 2000) wiggleStep(0);
            else if (timeLeftMillis < wiggleStart - WIGGLE_STEP * 1000) wiggleStep(WIGGLE_ANGLE);
            #endif
        }

        void updateCountdown(unsigned long timeLeftMillis) override {
            if (timeLeftMillis > COUNDOWN_SECONDS * 1000) return;

            if (timeLeftMillis % 1000 > COUNTDOWN_BEEP_SECONDS * 1000.0f)
                writeBuzzer(HIGH);
            else
                writeBuzzer(LOW);
        }

        void lightLed(int r, int g, int b) override {
            analogWrite(LED_R, r);
            analogWrite(LED_G, g);
            analogWrite(LED_B, b);
        }

        void writeBuzzer(uint8_t value) override {
            digitalWrite(BUZZER, value);
        }

    private:
        Servo XPlus;
        Servo XMinus;
        Servo YPlus;
        Servo YMinus;

        Servo Esc;
        Servo Leg;

        void wiggleStep(int angle) {
            XPlus.write(90 + angle);
            XMinus.write(90 + angle);
            YPlus.write(90 + angle);
            YMinus.write(90 + angle);
        }
};

#endif
