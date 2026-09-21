#pragma once
#include "81yOdom/odometry/Odom.hpp"
#include "81yOdom/pid/PID.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cmath>
#include <functional>

// ═════════════════════════════════════════════════════════════════════════════
//  Configuration structs. Fill three of these, hand them to Chassis, done.
// ═════════════════════════════════════════════════════════════════════════════

struct DrivetrainConfig {
    pros::MotorGroup* leftMotors  = nullptr;
    pros::MotorGroup* rightMotors = nullptr;
    double trackWidth    = 12.0;   // inches, center-to-center of left/right drive wheels
    double wheelDiameter = 3.25;   // inches, drive wheel - used by the encoder fallback
    double gearRatio     = 1.0;    // drive-wheel revs per motor rev (external gearing only)
};

// Tuning for one axis (lateral = driving, angular = turning).
//
// Units: error in inches (lateral) or degrees (angular); output in percent.
//
// Exit conditions: the motion ends when |error| has stayed under smallError
// for smallErrorTimeout ms, OR under largeError for largeErrorTimeout ms,
// whichever comes first. The large pair is the safety net - a motion that
// stalls 1.5" short still exits promptly instead of burning its timeout.
struct ControllerSettings {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double windupRange = 0.0;          // integrate only within this error; 0 = always

    double smallError        = 1.0;    // in or deg
    double smallErrorTimeout = 100;    // ms
    double largeError        = 3.0;    // in or deg
    double largeErrorTimeout = 500;    // ms

    double slew = 0.0;                 // max output change per second; 0 = off
    double dFilter = 0.6;              // derivative low-pass 0..1

    PIDConfig toPID() const {
        PIDConfig c;
        c.kP = kP; c.kI = kI; c.kD = kD;
        c.windupRange = windupRange;
        c.dFilter = dFilter;
        return c;
    }
};

// Stall detection. If the robot is far from target and not moving, the motion
// has hit something. Exiting promptly is nearly always better than waiting
// out the timeout in a match.
struct StallSettings {
    bool   enabled     = true;
    double linearVel   = 1.0;   // in/s  - below this counts as stopped
    double angularVel  = 5.0;   // deg/s
    double timeMs      = 300;   // stopped this long => exit
    double armAfterMs  = 400;   // ignore the first N ms while the drive spins up
};

// ─── Per-motion options. Everything has a sensible default; override only
//     what you need:   chassis.moveToPoint(24, 48, 3000, {.forwards=false});

struct MoveToPointParams {
    bool   forwards       = true;   // false = drive to the point in reverse
    double maxSpeed       = 100;    // percent
    double minSpeed       = 0;      // chaining: never drop below this, and exit at earlyExitRange
    double earlyExitRange = 0;      // inches; > 0 exits early WITHOUT braking (momentum carries)
    bool   async          = false;
};

// Boomerang controller: arrives at (x, y) already pointing at `theta`. Best
// for heading changes under ~120 deg. For a full reversal (arrive facing back
// the way you came) it lands ~1 in / ~4 deg off - do turnToHeading first, or
// chain a turnToHeading after it.
struct MoveToPoseParams {
    bool   forwards        = true;
    double maxSpeed        = 100;
    double minSpeed        = 0;
    double earlyExitRange  = 0;
    double lead            = 0.6;   // boomerang carrot distance factor, 0..1. Higher = wider arc
    double horizontalDrift = 2.0;   // cornering aggressiveness: ~2 all-omni, ~8 with traction wheels
    bool   async           = false;
};

struct TurnToHeadingParams {
    double maxSpeed       = 100;
    double minSpeed       = 0;      // floor to beat static friction on the last few degrees
    double earlyExitRange = 0;      // degrees
    bool   async          = false;
};

struct MoveDistanceParams {
    double maxSpeed       = 100;
    double minSpeed       = 0;
    double earlyExitRange = 0;
    double heading        = NAN;    // hold this heading (deg). NAN = hold whatever we start at
    bool   async          = false;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Chassis
// ═════════════════════════════════════════════════════════════════════════════
class Chassis {
    public:
        Chassis(DrivetrainConfig drivetrain,
                ControllerSettings lateralSettings,
                ControllerSettings angularSettings,
                OdomSensors sensors);

        // ── Setup ───────────────────────────────────────────────────────────
        // Calibrates the IMU (with retry), zeroes every sensor, and starts the
        // 10ms odometry task. Call ONCE from initialize(). Returns false if the
        // IMU could not be calibrated - odometry then falls back to
        // wheel-derived heading if two vertical wheels exist.
        bool calibrate(bool calibrateImu = true);

        void setPose(double x, double y, double theta);
        void setPose(Pose p);
        Pose getPose() const { return odom.getPose(); }

        // ── Motions (blocking unless params.async) ──────────────────────────
        void turnToHeading(double theta, double timeoutMs, TurnToHeadingParams p = {});
        void moveToPoint(double x, double y, double timeoutMs, MoveToPointParams p = {});
        void moveToPose(double x, double y, double theta, double timeoutMs, MoveToPoseParams p = {});
        void moveDistance(double distance, double timeoutMs, MoveDistanceParams p = {});

        // ── Async control ───────────────────────────────────────────────────
        void waitUntilDone();
        // Block until the running motion has travelled `dist` (inches for
        // drives, degrees for turns) or finished. Lets you fire an intake
        // partway through a move.
        void waitUntil(double dist);
        void cancelMotion();
        bool isInMotion() const { return motionRunning.load(); }

        // ── Manual drive (percent, -100..100) ───────────────────────────────
        void tank(double left, double right);
        void arcade(double throttle, double turn);
        void brake();
        void setBrakeMode(pros::MotorBrake mode);

        // ── Tuning / measurement (blocking, interactive, print to terminal) ─
        // Run these in order the first time you set up a robot:
        //   1. checkWheelDirections   2. measureWheelDiameter
        //   3. measureTrackingOffsets 4. measureImuScalar   5. tuneDriveBalance
        //
        // You push the robot forward, then to its right; it tells you which
        // wheels have `reversed` set wrong. Everything else assumes forward and
        // right read positive, so do this one first.
        void checkWheelDirections(pros::Controller& controller);
        // Equal open-loop voltage to both sides; reports per-side travel and
        // heading drift. Nonzero => mechanical asymmetry.
        void tuneDriveBalance(int ms = 1500, double percent = 50);
        // Spins in place under IMU control and COMPUTES every tracking wheel's
        // offset from what it rolled. Paste the printed values into your config.
        void measureTrackingOffsets(pros::Controller& controller, double degrees = 720);
        // You rotate the robot exactly `turns` times against a reference edge;
        // it reports the IMU scalar to correct drift.
        void measureImuScalar(pros::Controller& controller, int turns = 5);
        // You push the robot exactly `actualInches` along a tape measure; it
        // reports corrected wheel diameters.
        void measureWheelDiameter(pros::Controller& controller, double actualInches = 48);

        // ── Access ──────────────────────────────────────────────────────────
        Odom& getOdom() { return odom; }
        PID&  getLateralPID() { return lateralPID; }
        PID&  getAngularPID() { return angularPID; }
        ControllerSettings& lateral() { return lateralSettings; }
        ControllerSettings& angular() { return angularSettings; }
        StallSettings stall;                       // public: tweak freely

        // While the drive output is still slewing up from rest, heading
        // correction is capped to this fraction of it. Without the cap the turn
        // term can equal the (tiny) drive term on the opening loops and the mix
        // lands at (2*drive, 0): one side gets everything, the other nothing,
        // and the chassis pivots before it translates. 0.4 holds the worst case
        // near 2.3:1 from a 10 deg misalignment. Only active during ramp-up.
        double turnAuthority = 0.4;

        static constexpr int    CONTROL_PERIOD_MS = 10;
        static constexpr double CLOSE_RANGE_IN    = 7.5;   // moveToPoint/Pose switch to settle mode inside this

    private:
        // Motion framework
        void   requestMotionStart(bool async, std::function<void()> body);
        bool   cancelled() const { return cancelRequested.load(); }
        void   endMotion(bool keepMoving);
        // Drive/turn mixer: clamps |turn| to turnCap, desaturates preserving the
        // left/right ratio, sends voltage.
        void   applyOutput(double drive, double turn, double maxSpeed, double turnCap);
        // Turn cap for this cycle. See turnAuthority.
        double turnCapFor(double driveOut, double driveRaw, double floorWhenCruising) const;
        // True once the robot has been stationary while still far from target.
        bool   stalled(double elapsedMs, double error, double largeErr);

        DrivetrainConfig   dt;
        ControllerSettings lateralSettings;
        ControllerSettings angularSettings;
        PID lateralPID;
        PID angularPID;
        Odom odom;

        std::atomic<bool>   motionRunning{false};
        std::atomic<bool>   cancelRequested{false};
        std::atomic<double> distTravelled{0.0};
        bool odomTaskStarted = false;
        uint32_t stallSince = 0;
};
