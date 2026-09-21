#pragma once
#include "81yOdom/sensors/TrackingWheel.hpp"
#include "81yOdom/sensors/IMU.hpp"
#include "81yOdom/odometry/Pose.hpp"
#include "pros/rtos.hpp"
#include <cstdint>

// Which sensors the odometry has to work with. Any of these may be null;
// the integrator uses whatever is present:
//
//   heading   IMU if present and healthy, else derived from two vertical
//             wheels, else held constant.
//   forward   average of the healthy vertical wheels (each corrected for its
//             own lateral offset). With none, forward motion is not tracked.
//   lateral   average of the healthy horizontal wheels (each corrected for its
//             own forward offset). With none, lateral motion is assumed zero -
//             fine for a tank drive that is not being shoved sideways.
//
// Minimum useful setup: one vertical wheel + IMU. Best: two vertical, one
// horizontal, IMU. Zero tracking wheels: build the verticals from the drive
// MotorGroups (see TrackingWheel's MotorGroup constructor).
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

        // Teleport the tracked pose. Also re-zeroes the IMU so update() does
        // not overwrite theta with the old heading one cycle later.
        void setPose(Pose p);
        void setPose(double x, double y, double theta) { setPose(Pose{x, y, theta}); }

        // Thread-safe snapshot. Called from motions, the GUI, anywhere.
        Pose getPose() const;

        // Re-baseline every sensor without moving the pose. Call once after the
        // IMU is calibrated and before the first update().
        void reset();

        // Run one integration cycle. Call from a dedicated task every 10ms;
        // Chassis::calibrate() starts that task for you.
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

// Never-throwing RAII lock for pros::Mutex. pros::Mutex::lock() throws
// std::system_error on failure, and an exception escaping a task calls
// std::terminate() and takes the whole program down. take()/give() do not.
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
