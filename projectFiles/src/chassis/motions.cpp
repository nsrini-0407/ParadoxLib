#include "81yOdom/chassis/Chassis.hpp"
#include "81yOdom/utils/ExitCondition.hpp"
#include "81yOdom/utils/math.hpp"
#include <cmath>

// ─── Shared helpers ──────────────────────────────────────────────────────────

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

// ─── turnToHeading ───────────────────────────────────────────────────────────

void Chassis::turnToHeading(double theta, double timeoutMs, TurnToHeadingParams p) {
    requestMotionStart(p.async, [=, this] {
        ExitCondition small(angularSettings.smallError, angularSettings.smallErrorTimeout);
        ExitCondition large(angularSettings.largeError, angularSettings.largeErrorTimeout);

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
            if (p.earlyExitRange > 0 && std::fabs(error) < p.earlyExitRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, error, angularSettings.largeError)) break;

            double out = angularPID.update(error, dt);
            out = clamp(out, -p.maxSpeed, p.maxSpeed);
            out = slew(out, prevOut, angularSettings.slew, dt);

            // Floor to beat static friction - but only while there is real
            // error left. Applied inside the settle band it flips sign every
            // time error crosses zero and the robot buzzes instead of stopping.
            if (std::fabs(error) > angularSettings.smallError && std::fabs(out) < p.minSpeed) {
                out = p.minSpeed * sign(error);
            }
            prevOut = out;

            tank(out, -out);
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && p.minSpeed > 0);
    });
}

// ─── moveToPoint ─────────────────────────────────────────────────────────────

void Chassis::moveToPoint(double x, double y, double timeoutMs, MoveToPointParams p) {
    requestMotionStart(p.async, [=, this] {
        ExitCondition small(lateralSettings.smallError, lateralSettings.smallErrorTimeout);
        ExitCondition large(lateralSettings.largeError, lateralSettings.largeErrorTimeout);

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        Pose lastPose = odom.getPose();
        double prevLateral = 0.0, prevAngular = 0.0;
        double maxSpeed = p.maxSpeed;
        bool close = false, earlyExit = false;

        while (!cancelled() && pros::millis() - start < (uint32_t)timeoutMs) {
            const double dt = stepDt(lastMs);
            const Pose pose = odom.getPose();

            distTravelled.store(distTravelled.load() + pose.distanceTo(lastPose));
            lastPose = pose;

            const double d = pose.distanceTo(x, y);

            // Aim the front (or, reversed, the back) at the target. The robot's
            // OWN heading is what gets subtracted - not a flipped copy of the
            // target, which cancels to a constant 180.
            double targetHeading = pose.angleTo(x, y);
            if (!p.forwards) targetHeading = normalizeAngle(targetHeading + 180.0);
            const double angularError = pose.headingError(targetHeading);

            // Remaining distance projected onto the robot's forward axis.
            double lateralError = d * std::cos(toRad(angularError));
            if (!p.forwards) lateralError = -lateralError;

            // Inside CLOSE_RANGE the target bearing swings wildly for tiny
            // position errors (atan2 of a short vector), so stop steering and
            // lock the speed ceiling at whatever we were doing so the PID can
            // only decelerate from here, never re-accelerate into the target.
            if (!close && d < CLOSE_RANGE_IN) {
                close = true;
                maxSpeed = std::max(std::fabs(prevLateral), 30.0);
            }

            if (close && (small.update(lateralError) || large.update(lateralError))) break;
            if (p.earlyExitRange > 0 && d < p.earlyExitRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, d, lateralSettings.largeError)) break;

            double lateralRaw = lateralPID.update(lateralError, dt);
            double angularOut = close ? 0.0 : angularPID.update(angularError, dt);

            lateralRaw = clamp(lateralRaw, -maxSpeed, maxSpeed);
            angularOut = clamp(angularOut, -maxSpeed, maxSpeed);

            // Facing away from the target (|angularError| > 90) the projection
            // goes negative and P would drive us backward, away from it. Turn
            // first; only drive in the commanded direction.
            if (!close) lateralRaw = p.forwards ? std::max(lateralRaw, 0.0) : std::min(lateralRaw, 0.0);

            // Chaining floor. Applied before the slew, or the floor jumps the
            // output straight to minSpeed on loop 0 and the slew is a no-op.
            if ((!close || p.earlyExitRange > 0) && std::fabs(lateralRaw) < p.minSpeed) {
                lateralRaw = p.minSpeed * (p.forwards ? 1.0 : -1.0);
            }

            const double lateralOut = slew(lateralRaw, prevLateral, lateralSettings.slew, dt);
            angularOut = slew(angularOut, prevAngular, angularSettings.slew, dt);
            prevLateral = lateralOut;
            prevAngular = angularOut;

            applyOutput(lateralOut, angularOut, maxSpeed, turnCapFor(lateralOut, lateralRaw, maxSpeed));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && p.minSpeed > 0);
    });
}

// ─── moveToPose (boomerang) ──────────────────────────────────────────────────
//
// Drive toward a "carrot" placed `lead * distance` behind the target along the
// target heading. As the robot closes in, the carrot slides onto the target and
// the approach direction converges on the requested final heading, so the
// robot arrives already pointing the right way.

void Chassis::moveToPose(double x, double y, double theta, double timeoutMs, MoveToPoseParams p) {
    requestMotionStart(p.async, [=, this] {
        ExitCondition small(lateralSettings.smallError, lateralSettings.smallErrorTimeout);
        ExitCondition large(lateralSettings.largeError, lateralSettings.largeErrorTimeout);

        uint32_t wake = pros::millis();
        const uint32_t start = wake;
        uint32_t lastMs = wake;
        Pose lastPose = odom.getPose();
        double prevLateral = 0.0, prevAngular = 0.0;
        double maxSpeed = p.maxSpeed;
        bool close = false, earlyExit = false;
        // Reversed approaches: the robot arrives travelling backward, so its
        // direction of travel is heading+180. Flip BOTH the target heading (so
        // the carrot lands on the correct side) and the robot's own heading (so
        // the angular error still drives the robot's FRONT to `theta`). Flipping
        // only one of them leaves the final heading 180 off.
        const double travelTheta = p.forwards ? theta : normalizeAngle(theta + 180.0);
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

            // Carrot: `lead * d` behind the target along the direction of
            // travel; heading unit vector in this convention is (sin θ, cos θ).
            // Once close it snaps onto the target. Without the snap the lateral
            // error - which is measured to the carrot - would under-report the
            // real distance by (1 - lead), and the exit conditions would fire
            // 2-3 inches short of the target.
            double cx = x - d * p.lead * std::sin(tRad);
            double cy = y - d * p.lead * std::cos(tRad);
            if (close) { cx = x; cy = y; }

            const double robotTravelHeading = p.forwards ? pose.theta : normalizeAngle(pose.theta + 180.0);
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
            if (p.earlyExitRange > 0 && d < p.earlyExitRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, d, lateralSettings.largeError)) break;

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
                    const double maxSlip = std::sqrt(p.horizontalDrift * radius * 9.8);
                    lateralRaw = clamp(lateralRaw, -maxSlip, maxSlip);
                }
                lateralRaw = p.forwards ? std::max(lateralRaw, 0.0) : std::min(lateralRaw, 0.0);
            }

            if ((!close || p.earlyExitRange > 0) && std::fabs(lateralRaw) < p.minSpeed) {
                lateralRaw = p.minSpeed * (p.forwards ? 1.0 : -1.0);
            }

            const double lateralOut = slew(lateralRaw, prevLateral, lateralSettings.slew, dt);
            angularOut = slew(angularOut, prevAngular, angularSettings.slew, dt);
            prevLateral = lateralOut;
            prevAngular = angularOut;

            // Heading is a goal here, not a side-effect, so never strangle the
            // turn term: the cap only bites during drive ramp-up (see
            // turnCapFor); otherwise the desaturation shares power fairly.
            applyOutput(lateralOut, angularOut, maxSpeed, turnCapFor(lateralOut, lateralRaw, maxSpeed));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && p.minSpeed > 0);
    });
}

// ─── moveDistance ────────────────────────────────────────────────────────────
//
// Drive a signed distance along a fixed heading, holding that heading.
// Progress is the displacement PROJECTED onto the heading - never a raw
// distance, which is unsigned and reads backing-up as progress.

void Chassis::moveDistance(double distance, double timeoutMs, MoveDistanceParams p) {
    requestMotionStart(p.async, [=, this] {
        ExitCondition small(lateralSettings.smallError, lateralSettings.smallErrorTimeout);
        ExitCondition large(lateralSettings.largeError, lateralSettings.largeErrorTimeout);

        const Pose startPose = odom.getPose();
        const double heading = std::isfinite(p.heading) ? p.heading : startPose.theta;
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
            if (p.earlyExitRange > 0 && std::fabs(error) < p.earlyExitRange) { earlyExit = true; break; }
            if (stalled(pros::millis() - start, error, lateralSettings.largeError)) break;

            double lateralRaw = clamp(lateralPID.update(error, dt), -p.maxSpeed, p.maxSpeed);
            double angularOut = clamp(angularPID.update(angularError, dt), -p.maxSpeed, p.maxSpeed);

            if (std::fabs(error) > lateralSettings.smallError && std::fabs(lateralRaw) < p.minSpeed) {
                lateralRaw = p.minSpeed * sign(error);
            }

            const double lateralOut = slew(lateralRaw, prevLateral, lateralSettings.slew, dt);
            angularOut = slew(angularOut, prevAngular, angularSettings.slew, dt);
            prevLateral = lateralOut;
            prevAngular = angularOut;

            // Small floor: as the drive term decays to zero at the end, an
            // unbounded heading term would pivot the robot right as it settles.
            applyOutput(lateralOut, angularOut, p.maxSpeed, turnCapFor(lateralOut, lateralRaw, 10.0));
            pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
        }
        endMotion(earlyExit && p.minSpeed > 0);
    });
}
