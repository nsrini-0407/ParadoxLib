#include "81yOdom/chassis/Chassis.hpp"
#include "81yOdom/utils/ExitCondition.hpp"
#include "81yOdom/utils/math.hpp"
#include <cmath>

//  Shared helpers 

namespace {

// Wall-clock dt for this iteration, seconds. Never asserts a period.
double stepDt(uint32_t& lastMs) {
    const uint32_t now = pros::millis();
    double dt = (now - lastMs) / 1000.0;
    lastMs = now;
    return (dt <= 0.0 || dt > 0.25) ? Chassis::CONTROL_PERIOD_MS / 1000.0 : dt;
}

// Signed curvature (1/inch) of the arc from `pose`, tangent to its heading,
// through (ox, oy). Positive = the point is to the robot's right = CW turn.
//
// With heading unit vector h = (sin θ, cos θ) and the vector v to the point,
// the perpendicular offset of the point from the heading line is the 2D cross
// product h × v, and the arc curvature is 2 * offset / |v|².
double curvatureTo(const Pose& pose, double ox, double oy) {
    const double th = toRad(pose.theta);
    const double vx = ox - pose.x, vy = oy - pose.y;
    const double d2 = vx*vx + vy*vy;
    if (d2 < 1e-9) return 0.0;
    const double cross = vx * std::cos(th) - vy * std::sin(th);
    return 2.0 * cross / d2;
}

} // namespace


void Chassis::turnToAngle(double theta, double timeoutMs, TurnToAngleOptions o) {
    requestMotionStart(o.background, [=, this] {
        // Per-motion overrides; unset fields fall back to the chassis config.
        // Reloaded every motion, so an override never leaks into the next one.
        const ControllerSettings ang = o.turnTune.applyTo(angularSettings);
        angularPID.setConfig(ang.toPID());

        ExitCondition small(ang.smallError, ang.smallErrorTimeout);
        ExitCondition large(ang.largeError, ang.largeErrorTimeout);

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        double prevOut = 0.0;
        double lastTheta = odom.getPose().theta;
        bool earlyExit = false;

        while (!cancelled() && pros::millis() - start < (uint32_t)timeoutMs) {
            const double dt = stepDt(lastMs);
            const Pose pose = odom.getPose();

            distTravelled.store(distTravelled.load() + std::fabs(normalizeAngle(pose.theta - lastTheta)));
            lastTheta = pose.theta;

            const double error = pose.headingError(theta);

            if (small.update(error) || large.update(error)) break;
            if (o.handoffRange > 0 && std::fabs(error) < o.handoffRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, error, ang.largeError)) break;

            double out = angularPID.update(error, dt);
            out = clamp(out, -o.speedCap, o.speedCap);
            out = slew(out, prevOut, ang.slew, dt);

            // Floor to beat static friction - but only while there is real
            // error left. Applied inside the settle band it flips sign every
            // time error crosses zero and the robot buzzes instead of stopping.
            if (std::fabs(error) > ang.smallError && std::fabs(out) < o.speedFloor) {
                out = o.speedFloor * sign(error);
            }
            prevOut = out;

            tank(out, -out);
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && o.speedFloor > 0);
    });
}

//  swingToAngle
//
// Same control loop as turnToAngle, but only one side drives. The locked
// side is put on HOLD for the duration - on plain brake it gets dragged along
// and the pivot point wanders.

void Chassis::swingToAngle(double theta, Pivot pivotSide, double timeoutMs, SwingToAngleOptions o) {
    requestMotionStart(o.background, [=, this] {
        const ControllerSettings ang = o.turnTune.applyTo(angularSettings);
        angularPID.setConfig(ang.toPID());

        ExitCondition small(ang.smallError, ang.smallErrorTimeout);
        ExitCondition large(ang.largeError, ang.largeErrorTimeout);

        pros::MotorGroup* locked = pivotSide == Pivot::LEFT ? dt.leftMotors  : dt.rightMotors;
        pros::MotorGroup* driven = pivotSide == Pivot::LEFT ? dt.rightMotors : dt.leftMotors;
        // CW-positive: a CW turn needs left > right, so a driven left side
        // takes +out and a driven right side takes -out.
        const double drivenSign = pivotSide == Pivot::LEFT ? -1.0 : 1.0;

        pros::MotorBrake lockedPrevMode = pros::MotorBrake::brake;
        if (locked) {
            lockedPrevMode = locked->get_brake_mode();
            locked->set_brake_mode_all(pros::MotorBrake::hold);
            locked->brake();
        }

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        double prevOut = 0.0;
        double lastTheta = odom.getPose().theta;
        // A forced direction only matters while the target is more than 180
        // deg away that way. Once shortest-path agrees with it, switch to
        // shortest-path for good so an overshoot is corrected, not treated as
        // another 360 deg to go.
        bool forced = o.spin != Spin::SHORTEST;
        bool earlyExit = false;

        while (!cancelled() && pros::millis() - start < (uint32_t)timeoutMs) {
            const double dt = stepDt(lastMs);
            const Pose pose = odom.getPose();

            distTravelled.store(distTravelled.load() + std::fabs(normalizeAngle(pose.theta - lastTheta)));
            lastTheta = pose.theta;

            double error = pose.headingError(theta);
            if (forced) {
                const bool wantCW = o.spin == Spin::CW;
                if (error == 0.0 || (error > 0.0) == wantCW) {
                    forced = false;
                } else {
                    error += wantCW ? 360.0 : -360.0;
                }
            }

            if (small.update(error) || large.update(error)) break;
            if (o.handoffRange > 0 && std::fabs(error) < o.handoffRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, error, ang.largeError)) break;

            double out = angularPID.update(error, dt);
            out = clamp(out, -o.speedCap, o.speedCap);
            out = slew(out, prevOut, ang.slew, dt);

            // Same static-friction floor as turnToAngle, same reason for
            // keeping it out of the settle band.
            if (std::fabs(error) > ang.smallError && std::fabs(out) < o.speedFloor) {
                out = o.speedFloor * sign(error);
            }
            prevOut = out;

            if (driven) driven->move_voltage((int)(clamp(out * drivenSign, -100.0, 100.0) * 120.0));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && o.speedFloor > 0);
        if (locked) locked->set_brake_mode_all(lockedPrevMode);
    });
}

//  driveToPoint

void Chassis::driveToPoint(double x, double y, double timeoutMs, DriveToPointOptions o) {
    requestMotionStart(o.background, [=, this] {
        const ControllerSettings lat = o.driveTune.applyTo(lateralSettings);
        const ControllerSettings ang = o.turnTune.applyTo(angularSettings);
        lateralPID.setConfig(lat.toPID());
        angularPID.setConfig(ang.toPID());

        ExitCondition small(lat.smallError, lat.smallErrorTimeout);
        ExitCondition large(lat.largeError, lat.largeErrorTimeout);

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        Pose lastPose = odom.getPose();
        double prevLateral = 0.0, prevAngular = 0.0;
        double maxSpeed = o.speedCap;
        double closeBearing = 0.0;
        bool close = false, earlyExit = false;

        while (!cancelled() && pros::millis() - start < (uint32_t)timeoutMs) {
            const double dt = stepDt(lastMs);
            const Pose pose = odom.getPose();

            distTravelled.store(distTravelled.load() + pose.distanceTo(lastPose));
            lastPose = pose;

            const double d = pose.distanceTo(x, y);

            double targetHeading = pose.angleTo(x, y);
            if (o.reverse) targetHeading = normalizeAngle(targetHeading + 180.0);

            if (!close && d < CLOSE_RANGE_IN) {
                close = true;
                closeBearing = targetHeading;
                maxSpeed = std::max(std::fabs(prevLateral), 30.0);
            }

            const double angularError = pose.headingError(close ? closeBearing : targetHeading);

            // Distance to the target projected onto the robot's forward axis.
            // Signed: once the robot overshoots, the target is behind it, this
            // goes negative and the PID backs up. (d * cos(angularError) is
            // always positive past the target because closeBearing is frozen,
            // so the robot would accelerate away.) Also covers reverse - a
            // target behind the robot projects negative on its own.
            const double lateralError = d * std::cos(toRad(normalizeAngle(pose.angleTo(x, y) - pose.theta)));

            if (close && d < lat.largeError * 1.5) {
                if (small.update(lateralError) || large.update(lateralError)) break;
            }
            if (o.handoffRange > 0 && d < o.handoffRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, d, lat.largeError)) break;

            double lateralRaw = lateralPID.update(lateralError, dt);
            double angularOut = angularPID.update(angularError, dt);

            lateralRaw = clamp(lateralRaw, -maxSpeed, maxSpeed);
            angularOut = clamp(angularOut, -maxSpeed, maxSpeed);

            if (!close) lateralRaw = o.reverse ? std::min(lateralRaw, 0.0) : std::max(lateralRaw, 0.0);

            if ((!close || o.handoffRange > 0) && std::fabs(lateralRaw) < o.speedFloor) {
                lateralRaw = o.speedFloor * (o.reverse ? -1.0 : 1.0);
            }

            const double lateralOut = slew(lateralRaw, prevLateral, lat.slew, dt);
            angularOut = slew(angularOut, prevAngular, ang.slew, dt);
            prevLateral = lateralOut;
            prevAngular = angularOut;

            applyOutput(lateralOut, angularOut, maxSpeed, turnCapFor(lateralOut, lateralRaw, close ? CLOSE_TURN_CAP : maxSpeed));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && o.speedFloor > 0);
    });
}

//  driveToPose (boomerang) 
//
// Drive toward a "carrot" placed `carrot * distance` behind the target along the
// target heading. As the robot closes in, the carrot slides onto the target and
// the approach direction converges on the requested final heading, so the
// robot arrives already pointing the right way.

void Chassis::driveToPose(double x, double y, double theta, double timeoutMs, DriveToPoseOptions o) {
    requestMotionStart(o.background, [=, this] {
        const ControllerSettings lat = o.driveTune.applyTo(lateralSettings);
        const ControllerSettings ang = o.turnTune.applyTo(angularSettings);
        lateralPID.setConfig(lat.toPID());
        angularPID.setConfig(ang.toPID());

        ExitCondition small(lat.smallError, lat.smallErrorTimeout);
        ExitCondition large(lat.largeError, lat.largeErrorTimeout);

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        Pose lastPose = odom.getPose();
        double prevLateral = 0.0, prevAngular = 0.0;
        double maxSpeed = o.speedCap;
        bool close = false, earlyExit = false;
        // Reversed approaches: the robot arrives travelling backward, so its
        // direction of travel is heading+180. Flip BOTH the target heading (so
        // the carrot lands on the correct side) and the robot's own heading (so
        // the angular error still drives the robot's FRONT to `theta`). Flipping
        // only one of them leaves the final heading 180 off.
        const double travelTheta = o.reverse ? normalizeAngle(theta + 180.0) : theta;
        const double tRad = toRad(travelTheta);

        while (!cancelled() && pros::millis() - start < (uint32_t)timeoutMs) {
            const double dt = stepDt(lastMs);
            const Pose pose = odom.getPose();

            distTravelled.store(distTravelled.load() + pose.distanceTo(lastPose));
            lastPose = pose;

            const double d = pose.distanceTo(x, y);

            if (!close && d < CLOSE_RANGE_IN) {
                close = true;
                maxSpeed = std::max(std::fabs(prevLateral), 30.0);
            }

            // Carrot: `carrot * d` behind the target along the direction of
            // travel; heading unit vector in this convention is (sin θ, cos θ).
            // Once close it snaps onto the target. Without the snap the lateral
            // error - which is measured to the carrot - would under-report the
            // real distance by (1 - carrot), and the exit conditions would fire
            // 2-3 inches short of the target.
            double cx = x - d * o.carrot * std::sin(tRad);
            double cy = y - d * o.carrot * std::cos(tRad);
            if (close) { cx = x; cy = y; }

            const double robotTravelHeading = o.reverse ? normalizeAngle(pose.theta + 180.0) : pose.theta;
            const double angularError = close
                ? normalizeAngle(travelTheta - robotTravelHeading)          // settle onto final heading
                : normalizeAngle(pose.angleTo(cx, cy) - robotTravelHeading); // steer at the carrot

            // Distance to the carrot projected onto the robot's forward axis.
            // Signed: a carrot behind the robot projects negative and the PID
            // drives backward on its own - no per-direction negation needed,
            // and overshoot past the target is corrected instead of chased.
            const double lateralError = pose.distanceTo(cx, cy)
                                      * std::cos(toRad(normalizeAngle(pose.angleTo(cx, cy) - pose.theta)));

            if (close && (small.update(lateralError) || large.update(lateralError))) break;
            if (o.handoffRange > 0 && d < o.handoffRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, d, lat.largeError)) break;

            double lateralRaw = lateralPID.update(lateralError, dt);
            double angularOut = angularPID.update(angularError, dt);

            lateralRaw = clamp(lateralRaw, -maxSpeed, maxSpeed);
            angularOut = clamp(angularOut, -maxSpeed, maxSpeed);

            // Cornering limit: the tighter the arc to the carrot, the slower we
            // may go before the drive slides sideways. Empirical, LemLib-style.
            if (!close) {
                const double curvature = curvatureTo(pose, cx, cy);
                if (std::fabs(curvature) > 1e-6) {
                    const double radius = 1.0 / std::fabs(curvature);
                    const double maxSlip = std::sqrt(o.cornerGrip * radius * 9.8);
                    lateralRaw = clamp(lateralRaw, -maxSlip, maxSlip);
                }
                lateralRaw = o.reverse ? std::min(lateralRaw, 0.0) : std::max(lateralRaw, 0.0);
            }

            if ((!close || o.handoffRange > 0) && std::fabs(lateralRaw) < o.speedFloor) {
                lateralRaw = o.speedFloor * (o.reverse ? -1.0 : 1.0);
            }

            const double lateralOut = slew(lateralRaw, prevLateral, lat.slew, dt);
            angularOut = slew(angularOut, prevAngular, ang.slew, dt);
            prevLateral = lateralOut;
            prevAngular = angularOut;

            // Heading is a goal here, not a side-effect, so never strangle the
            // turn term: the cap only bites during drive ramp-up (see
            // turnCapFor); otherwise the desaturation shares power fairly.
            applyOutput(lateralOut, angularOut, maxSpeed, turnCapFor(lateralOut, lateralRaw, maxSpeed));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && o.speedFloor > 0);
    });
}

//  driveDistance 
//
// Drive a signed distance along a fixed heading, holding that heading.
// Progress is the displacement PROJECTED onto the heading - never a raw
// distance, which is unsigned and reads backing-up as progress.

void Chassis::driveDistance(double distance, double timeoutMs, DriveDistanceOptions o) {
    requestMotionStart(o.background, [=, this] {
        const ControllerSettings lat = o.driveTune.applyTo(lateralSettings);
        const ControllerSettings ang = o.turnTune.applyTo(angularSettings);
        lateralPID.setConfig(lat.toPID());
        angularPID.setConfig(ang.toPID());

        ExitCondition small(lat.smallError, lat.smallErrorTimeout);
        ExitCondition large(lat.largeError, lat.largeErrorTimeout);

        const Pose startPose = odom.getPose();
        const double heading = std::isfinite(o.holdAngle) ? o.holdAngle : startPose.theta;
        const double hr = toRad(heading);

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        Pose lastPose = startPose;
        double prevLateral = 0.0, prevAngular = 0.0;
        bool earlyExit = false;

        while (!cancelled() && pros::millis() - start < (uint32_t)timeoutMs) {
            const double dt = stepDt(lastMs);
            const Pose pose = odom.getPose();

            distTravelled.store(distTravelled.load() + pose.distanceTo(lastPose));
            lastPose = pose;

            const double traveled = (pose.x - startPose.x) * std::sin(hr)
                                  + (pose.y - startPose.y) * std::cos(hr);
            const double error = distance - traveled;
            const double angularError = pose.headingError(heading);

            if (small.update(error) || large.update(error)) break;
            if (o.handoffRange > 0 && std::fabs(error) < o.handoffRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, error, lat.largeError)) break;

            double lateralRaw = clamp(lateralPID.update(error, dt), -o.speedCap, o.speedCap);
            double angularOut = clamp(angularPID.update(angularError, dt), -o.speedCap, o.speedCap);

            if (std::fabs(error) > lat.smallError && std::fabs(lateralRaw) < o.speedFloor) {
                lateralRaw = o.speedFloor * sign(error);
            }

            const double lateralOut = slew(lateralRaw, prevLateral, lat.slew, dt);
            angularOut = slew(angularOut, prevAngular, ang.slew, dt);
            prevLateral = lateralOut;
            prevAngular = angularOut;

            // Small floor: as the drive term decays to zero at the end, an
            // unbounded heading term would pivot the robot right as it settles.
            applyOutput(lateralOut, angularOut, o.speedCap, turnCapFor(lateralOut, lateralRaw, 10.0));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && o.speedFloor > 0);
    });
}
