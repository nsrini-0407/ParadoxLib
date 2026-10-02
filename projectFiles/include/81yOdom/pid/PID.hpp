#pragma once
#include <algorithm>
#include <cmath>
#include "81yOdom/utils/math.hpp"

struct PIDConfig {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    // Integral only accumulates while |error| < windupRange. Far from target
    // P is already saturating the motors and any I built up there is pure
    // overshoot; near target I is what closes the last fraction of an inch.
    // 0 = always integrate.
    double windupRange = 0.0;

    // Hard clamp on the accumulated integral term (in output units).
    double integralLimit = 50.0;

    // Zero the integral when error changes sign - the robot has crossed the
    // target, so whatever I had accumulated is now pushing the wrong way.
    bool signFlipReset = true;

    // Low-pass on the derivative, 0..1. Odometry updates in discrete 10ms
    // steps, so raw d(error)/dt is a staircase and kD amplifies every step
    // edge. 0.5-0.8 removes the buzz without much lag. 0 = raw derivative.
    double dFilter = 0.6;
};

class PID {
    public:
        explicit PID(PIDConfig config) : config(config) {}

        double update(double error, double dt) {
            if (dt <= 0.0) dt = 0.01;

            if (config.signFlipReset && sign(error) != sign(prevError) && !firstUpdate) {
                integral = 0.0;
            }
            if (config.windupRange <= 0.0 || std::fabs(error) < config.windupRange) {
                integral += error * dt;
            }
            integral = clamp(integral, -config.integralLimit, config.integralLimit);

            double rawDerivative = 0.0;
            if (!firstUpdate) rawDerivative = (error - prevError) / dt;
            derivative = firstUpdate ? 0.0 : lowPass(derivative, rawDerivative, config.dFilter);

            prevError = error;
            firstUpdate = false;

            return (config.kP * error) + (config.kI * integral) + (config.kD * derivative);
        }

        double Update(double error, double dt) { return update(error, dt); }

        void reset() {
            integral = 0.0;
            prevError = 0.0;
            derivative = 0.0;
            firstUpdate = true;
        }

        void setConfig(PIDConfig newConfig) { config = newConfig; }
        const PIDConfig& getConfig() const { return config; }

    private: 
        PIDConfig config;
        double integral = 0.0; 
        double prevError = 0.0;
        double derivative = 0.0;
        bool firstUpdate = true;
};
