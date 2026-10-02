#pragma once
#include "pros/rotation.hpp"
#include "pros/motor_group.hpp"
#include "pros/error.h"
#include "81yOdom/utils/math.hpp"
#include <cmath>
#include <vector>

// A wheel whose rotation is measured to track ground travel. Either a dedicated
// unpowered tracking wheel on a V5 Rotation sensor, or - as a fallback when you
// have none - the drive motors' own encoders.

class TrackingWheel {
public:
    // Dedicated tracking wheel on a Rotation sensor.
    TrackingWheel(pros::Rotation* sensor, double wheelDiameter, double offset,
                  double gearRatio = 1.0, bool reversed = false)
        : rotation(sensor), motors(nullptr),
          circumference(wheelDiameter * PI), gearRatio(gearRatio),
          reversed(reversed), offset(offset) {}

    // Drive-encoder fallback. Pass the left or right MotorGroup; offset is
    // +/- half the track width. Accuracy is limited by wheel slip, but it
    // means odometry works on a robot with zero tracking wheels.
    TrackingWheel(pros::MotorGroup* group, double wheelDiameter, double offset,
                  double gearRatio = 1.0)
        : rotation(nullptr), motors(group),
          circumference(wheelDiameter * PI), gearRatio(gearRatio),
          reversed(false), offset(offset) {}

    // Configure the device. Called from Chassis::calibrate(), NOT the
    // constructor: TrackingWheels are globals, and device calls made from a
    // global constructor can run before PROS has enumerated the ports, in
    // which case they are silently dropped. Whether they land varies boot to
    // boot - so the data rate and direction flag were randomly not applied.
    //
    // Direction is owned by the `reversed` argument alone. The sensor's own
    // reversed flag lives in the device and survives program restarts, so it
    // is forced off here; otherwise a flag left over from an earlier program
    // stacks with `reversed` and flips this wheel's sign on some runs only.
    void init() {
        if (rotation) {
            rotation->set_data_rate(5);   // fastest the sensor supports
            rotation->set_reversed(false);
        }
        if (motors) motors->set_encoder_units_all(pros::MotorUnits::degrees);
    }

    // Total distance travelled since reset, in inches. NaN if the sensor is
    // unreadable (disconnected port) - callers must isfinite() this.
    double getDistance() {
        const double deg = readDegrees();
        if (!std::isfinite(deg)) return deg;
        const double d = (deg / 360.0) * circumference * gearRatio;
        return reversed ? -d : d;
    }

    // Call exactly once per odometry cycle. Returns inches travelled since the
    // previous call and re-baselines in the same read.
    //
    // ONE sensor read on purpose. Reading the port twice (once for the delta,
    // once to re-baseline) drops any motion between the two reads, and leaves
    // the wheels sampled at different instants from each other and from the
    // IMU - which breaks the offset*dTheta cancellation the odometry depends on.
    //
    // Returns 0 and sets faulted=true on an unreadable sensor so a single bad
    // sample cannot inject a spike into the pose.
    double update() {
        const double now = getDistance();
        if (!std::isfinite(now)) {
            faulted = true;
            return 0.0;
        }
        if (faulted) {              // recovering: re-baseline, no delta
            faulted = false;
            lastDistance = now;
            return 0.0;
        }
        const double delta = now - lastDistance;
        lastDistance = now;
        return delta;
    }

    // Re-baseline without consuming a delta. Use when seeding the odom loop.
    void recordPosition() {
        const double now = getDistance();
        if (std::isfinite(now)) lastDistance = now;
    }

    // Re-baseline so the next update() measures from here. Deliberately does
    // NOT zero the sensor: Rotation::reset_position() is applied by the device
    // a packet or two later, so a read straight after it still returns the
    // old count. That stale value became the baseline, and when the zero
    // landed the whole old count (whatever the last program left on the
    // sensor) was integrated as one cycle of travel. Deltas never need the
    // hardware zeroed.
    void reset() {
        faulted = false;
        recordPosition();
    }

    double getOffset() const { return offset; }
    void   setOffset(double o) { offset = o; }
    bool   isFaulted() const { return faulted; }
    double getWheelDiameter() const { return circumference / PI; }
    void   setWheelDiameter(double d) { circumference = d * PI; }

private:
    // Raw shaft angle in degrees, or NaN if unreadable.
    double readDegrees() {
        if (rotation) {
            const std::int32_t centi = rotation->get_position();
            if (centi == PROS_ERR) return NAN;
            return centi / 100.0;
        }
        if (motors) {
            const std::vector<double> pos = motors->get_position_all();
            double sum = 0.0; int n = 0;
            for (double p : pos) {
                if (std::isfinite(p) && p != PROS_ERR_F) { sum += p; n++; }
            }
            return n ? sum / n : NAN;
        }
        return NAN;
    }

    pros::Rotation* rotation;
    pros::MotorGroup* motors;
    double circumference;
    double gearRatio;
    bool reversed;
    double offset;
    double lastDistance = 0.0;
    bool faulted = false;
};
