#include "81yOdom/chassis/Chassis.hpp"
#include "81yOdom/utils/math.hpp"
#include <cstdio>

Chassis::Chassis(DrivetrainConfig drivetrain,
                 ControllerSettings lateralSettings,
                 ControllerSettings angularSettings,
                 OdomSensors sensors)
    : dt(drivetrain),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings),
      lateralPID(lateralSettings.toPID()),
      angularPID(angularSettings.toPID()),
      odom(sensors) {}

// Setup 

bool Chassis::calibrate(bool calibrateImu) {
    bool ok = true;

    // Motions end with brake(); on coast the robot would roll past every target.
    setBrakeMode(pros::MotorBrake::brake);

    const OdomSensors& s = odom.sensors();
    if (calibrateImu && s.imu) {
        ok = s.imu->calibrate();
        if (!ok) {
            printf("[81yOdom] IMU calibration FAILED after retries. Heading will be "
                   "wheel-derived if two vertical wheels exist, else held constant.\n");
        }
    }

    TrackingWheel* wheels[] = {s.vertical1, s.vertical2, s.horizontal1, s.horizontal2};
    for (TrackingWheel* w : wheels) if (w) w->reset();

    odom.setPose(0.0, 0.0, 0.0);
    odom.reset();

    if (!odomTaskStarted) {
        odomTaskStarted = true;
        pros::Task([this] {
            uint32_t wake = pros::millis();
            while (true) {
                odom.update();
                pros::Task::delay_until(&wake, CONTROL_PERIOD_MS);
            }
        }, "81y_odom");
    }
    return ok;
}

void Chassis::setPose(double x, double y, double theta) { odom.setPose(x, y, theta); }
void Chassis::setPose(Pose p) { odom.setPose(p); }

//  Motion framework 

void Chassis::requestMotionStart(bool async, std::function<void()> body) {
    // Only one motion may own the drivetrain. Starting a new one while another
    // runs cancels the old one and waits for its loop to unwind, so two tasks
    // never command the motors at once.
    if (motionRunning.load()) {
        cancelRequested.store(true);
        while (motionRunning.load()) pros::delay(5);
    }
    cancelRequested.store(false);
    distTravelled.store(0.0);
    lateralPID.reset();
    angularPID.reset();
    stallSince = 0;
    motionRunning.store(true);

    if (async) {
        // pros::Task heap-copies the callable and frees it when the body
        // returns, so nothing here has to outlive this call.
        pros::Task([this, body] {
            body();
            motionRunning.store(false);
        }, "81y_motion");
    } else {
        body();
        motionRunning.store(false);
    }
}

void Chassis::endMotion(bool keepMoving) {
    // Chained motions (minSpeed + earlyExitRange) hand the next motion a robot
    // that is still rolling; everything else comes to a stop.
    if (!keepMoving) brake();
}

void Chassis::waitUntilDone() {
    while (motionRunning.load()) pros::delay(5);
}

void Chassis::waitUntil(double dist) {
    while (motionRunning.load() && distTravelled.load() < dist) pros::delay(5);
}

void Chassis::cancelMotion() {
    if (!motionRunning.load()) return;
    cancelRequested.store(true);
    waitUntilDone();
    cancelRequested.store(false);
}

bool Chassis::stalled(double elapsedMs, double error, double largeErr) {
    if (!stall.enabled)                 { stallSince = 0; return false; }
    if (elapsedMs < stall.armAfterMs)   { stallSince = 0; return false; }
    // Inside largeError the exit conditions own the decision, not this.
    if (std::fabs(error) < largeErr)    { stallSince = 0; return false; }

    const bool stopped = std::fabs(odom.getLinearVelocity())  < stall.linearVel
                      && std::fabs(odom.getAngularVelocity()) < stall.angularVel;
    if (!stopped)                       { stallSince = 0; return false; }

    const uint32_t now = pros::millis();
    if (stallSince == 0) stallSince = now;
    return (now - stallSince) >= (uint32_t)stall.timeMs;
}

//  Output 

double Chassis::turnCapFor(double driveOut, double driveRaw, double floorWhenCruising) const {
    // "Ramping up" = the slew limiter is holding the drive below what the PID
    // asked for. That is the only window where an uncapped turn term can win
    // the mix and pivot the chassis. Everywhere else, allow at least the floor
    // so heading can still be held when the drive term is legitimately small.
    const bool rampingUp = std::fabs(driveOut) < std::fabs(driveRaw) - 1e-9;
    if (rampingUp) return std::fabs(driveOut) * turnAuthority;
    return std::max(std::fabs(driveOut) * turnAuthority, floorWhenCruising);
}

void Chassis::applyOutput(double drive, double turn, double maxSpeed, double turnCap) {
    turn = clamp(turn, -turnCap, turnCap);

    double l = drive + turn;
    double r = drive - turn;

    // Desaturate preserving the left/right ratio, so steering intent survives
    // saturation instead of one side clipping and the other not.
    const double m = std::max(std::fabs(l), std::fabs(r));
    if (m > maxSpeed && m > 0.0) {
        l *= maxSpeed / m;
        r *= maxSpeed / m;
    }
    tank(l, r);
}

//  Manual drive 

void Chassis::tank(double left, double right) {
    left  = clamp(left,  -100.0, 100.0);
    right = clamp(right, -100.0, 100.0);
    if (dt.leftMotors)  dt.leftMotors->move_voltage((int)(left  * 120.0));
    if (dt.rightMotors) dt.rightMotors->move_voltage((int)(right * 120.0));
}

void Chassis::arcade(double throttle, double turn) {
    // CW-positive: positive turn = turn right = left side faster.
    tank(throttle + turn, throttle - turn);
}

void Chassis::brake() {
    if (dt.leftMotors)  dt.leftMotors->brake();
    if (dt.rightMotors) dt.rightMotors->brake();
}

void Chassis::setBrakeMode(pros::MotorBrake mode) {
    if (dt.leftMotors)  dt.leftMotors->set_brake_mode_all(mode);
    if (dt.rightMotors) dt.rightMotors->set_brake_mode_all(mode);
}
