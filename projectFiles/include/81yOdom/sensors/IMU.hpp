#pragma once
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"
#include "81yOdom/utils/math.hpp"
#include <cmath>
#include <cstdint>

// V5 Inertial Sensor wrapper.
//
// Built on get_rotation(), which is continuous (no 0/360 wrap) and clockwise
// positive natively - exactly the library's heading convention, so no sign
// flip is needed. Working in unbounded rotation means the odometry can take a
// simple difference for dTheta and a drift scalar can be applied cleanly.
class IMU {
    public: 
        explicit IMU(int port) : imu(port) {}

        // Blocking calibration with retry. 
        bool calibrate(int maxAttempts = 3) {
            for (int attempt = 1; attempt <= maxAttempts; attempt++) {
                if (imu.reset(true) == PROS_ERR) {
                    pros::delay(250);
                    continue;
                }
                // reset(true) returns when the sensor says it is done; give it
                // a beat and confirm it actually produces numbers.
                pros::delay(50);
                if (!imu.is_calibrating() && std::isfinite(imu.get_rotation())) {
                    imu.tare_rotation();
                    offset = 0.0;
                    calibrated = true;
                    return true;
                }
                pros::delay(250);
            }
            calibrated = false;
            return false;
        }

        bool isCalibrated() const { return calibrated; }
        bool isCalibrating() const { return imu.is_calibrating(); }

        // Continuous heading in degrees, CW positive, unbounded. Returns a
        // non-finite value if the sensor is faulted - callers guard with
        // std::isfinite().
        double getRotation() {
            const double raw = imu.get_rotation();
            if (!std::isfinite(raw) || raw == PROS_ERR_F) {
                faulted = true;
                return NAN;
            }
            faulted = false;
            return raw * scalar + offset;
        }

        // Heading normalized to (-180, 180]. Non-finite if faulted.
        double getHeading() { return normalizeAngle(getRotation()); }

        // Re-zero so that getRotation() reads `degrees` right now.
        void setRotation(double degrees) {
            const double raw = imu.get_rotation();
            if (!std::isfinite(raw) || raw == PROS_ERR_F) return;
            offset = degrees - raw * scalar;
        }
        void setHeading(double degrees) { setRotation(degrees); }

        // Drift correction: actual rotation / reported rotation. Measure with
        // Chassis::measureImuScalar(). 1.0 = trust the sensor as-is.
        void   setScalar(double s) { if (s > 0.5 && s < 2.0) scalar = s; }
        double getScalar() const { return scalar; }

        bool isFaulted() const { return faulted; }

        // Raw device access for anything not wrapped here.
        pros::IMU& raw() { return imu; }

    private: 
        pros::IMU imu; 
        double offset = 0.0;
        double scalar = 1.0;
        bool calibrated = false;
        bool faulted = false;
};
