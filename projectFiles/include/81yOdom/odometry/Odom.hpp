#pragma once
#include "81yOdom/sensors/TrackingWheel.hpp"
#include "81yOdom/sensors/IMU.hpp"
#include "81yOdom/odometry/Pose.hpp"
#include "pros/rtos.hpp"
#include <cstdint>


struct OdomSensors {
    TrackingWheel* vertical1   = nullptr;
    TrackingWheel* vertical2   = nullptr;
    TrackingWheel* horizontal1 = nullptr;
    TrackingWheel* horizontal2 = nullptr;
    IMU*           imu         = nullptr;
};

class Odom {
    public: 
        explicit Odom(OdomSensors sensors) : s(sensors) {}
        void setPose(Pose p);

        void setPose(double x, double y, double theta) { setPose(Pose{x, y, theta}); }

        Pose getPose() const;

        void reset();

        void update();

        // Velocity estimates from the last few cycles, low-passed.
        double getLinearVelocity()  const;   // inches / second, signed along robot forward
        double getAngularVelocity() const;   // degrees / second, CW positive

        // Diagnostics
        bool isHeadingFromImu() const { return headingFromImu; }
        bool anyWheelFaulted() const;

        const OdomSensors& sensors() const { return s; }

    private: 
        OdomSensors s;
        Pose pose;
        double prevRotation = 0.0;       // last IMU rotation used (unbounded deg)
        bool   havePrevRotation = false;
        uint32_t lastUpdateMs = 0;
        double linVel = 0.0, angVel = 0.0;
        bool headingFromImu = true;
        mutable pros::Mutex mutex;
};

class ScopedLock {
    public:
        explicit ScopedLock(pros::Mutex& m) : m(m), held(m.take(TIMEOUT_MAX)) {}
        ~ScopedLock() { if (held) m.give(); }
        ScopedLock(const ScopedLock&) = delete;
        ScopedLock& operator=(const ScopedLock&) = delete;
    private:
        pros::Mutex& m;
        bool held;
};
