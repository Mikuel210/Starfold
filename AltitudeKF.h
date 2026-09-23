#pragma once

struct KFState {
    double position, velocity;

    // Prediction uncertainty
    double P11; // Position variance
    double P22; // Velocity variance
    double P12; // Covariance

    // Delta P because of accelerometer noise
    double Q11; // Position
    double Q22; // Velocity
    double Q12; // Covariance
};

class AltitudeKF {
    public:
        KFState s;

        AltitudeKF(double _lidarNoise, double _accelNoise, KFState _s)
            : lidarNoise(_lidarNoise), accelNoise(_accelNoise), s(_s) {}

        void predict(double accelY, double dt) {
            // Predict position and velocity
            s.position = s.position + dt*s.velocity + 0.5*dt*dt*accelY;
            s.velocity = s.velocity + dt*accelY;

            // Predict Q (delta P)
            s.Q11 = dt*dt*dt*dt *0.25 *accelNoise;
            s.Q12 = dt*dt*dt    *0.5  *accelNoise;
            s.Q22 = dt*dt             *accelNoise;

            // Predict of error variance
            s.P11 = s.P11 + 2*dt*s.P12 + dt*dt*s.P22 + s.Q11;
            s.P12 = s.P12 + dt*s.P22;
            s.P22 = s.P22 + s.Q22;
        }

        void update(double z) {
            // Computate Kalman gain
            double S  = s.P11 + lidarNoise;
            double K1 = s.P11 / S;
            double K2 = s.P12 / S;
            double y  = z - s.position;

            // Compute state
            s.position += K1 * y;
            s.velocity += K2 * y;

            // Compute uncertainty
            s.P22 -= K2 * s.P12;
            s.P11 = s.P11 * (1 - K1);
            s.P12 = s.P12 * (1 - K1);
        }

    private:
        double lidarNoise;
        double accelNoise;
};
