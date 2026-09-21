#include "81yOdom/chassis/Chassis.hpp"
#include "81yOdom/utils/math.hpp"
#include <cmath>
#include <cstdio>

// Interactive measurement routines. Each prints exactly what to paste into
// your config. Run them from opcontrol on a controller button, with the PROS
// terminal open (`pros terminal`).

namespace {

void banner(const char* title) {
    printf("\n══════════════════════════════════════════════════\n  %s\n"
           "══════════════════════════════════════════════════\n", title);
}

void waitForA(pros::Controller& c, const char* instruction) {
    printf("%s\n  -> press A on the controller when ready\n", instruction);
    c.set_text(0, 0, "Press A        ");
    while (!c.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
    c.rumble(".");
}

double readOr(TrackingWheel* w, double fallback = NAN) {
    return w ? w->getDistance() : fallback;
}

const char* nm(int i) {
    static const char* names[] = {"vertical1  ", "vertical2  ", "horizontal1", "horizontal2"};
    return names[i];
}

} // namespace

// ─── 1. Wheel directions ─────────────────────────────────────────────────────

void Chassis::checkWheelDirections(pros::Controller& c) {
    banner("CHECK WHEEL DIRECTIONS");
    const OdomSensors& s = odom.sensors();
    TrackingWheel* w[] = {s.vertical1, s.vertical2, s.horizontal1, s.horizontal2};

    waitForA(c, "Step 1/2: press A, then push the robot STRAIGHT FORWARD about 12 in, then press A again.");
    double before[4]; for (int i = 0; i < 4; i++) before[i] = readOr(w[i]);
    waitForA(c, "Pushed forward? Press A.");
    bool allOk = true;
    for (int i = 0; i < 2; i++) {
        if (!w[i]) continue;
        const double d = readOr(w[i]) - before[i];
        const bool ok = d > 1.0;
        allOk &= ok;
        printf("  %s  read %+7.2f in   %s\n", nm(i), d,
               ok ? "OK (forward reads +)" : d < -1.0 ? "<< FLIP `reversed` on this wheel" : "<< barely moved - is it touching the floor?");
    }

    waitForA(c, "Step 2/2: press A, then push the robot to ITS RIGHT about 12 in, then press A again.");
    for (int i = 0; i < 4; i++) before[i] = readOr(w[i]);
    waitForA(c, "Pushed right? Press A.");
    for (int i = 2; i < 4; i++) {
        if (!w[i]) continue;
        const double d = readOr(w[i]) - before[i];
        const bool ok = d > 1.0;
        allOk &= ok;
        printf("  %s  read %+7.2f in   %s\n", nm(i), d,
               ok ? "OK (right reads +)" : d < -1.0 ? "<< FLIP `reversed` on this wheel" : "<< barely moved - is it touching the floor?");
    }
    printf(allOk ? "\n  All wheel directions correct.\n"
                 : "\n  Fix the flagged `reversed` flags before running the other measurements.\n");
    c.set_text(0, 0, allOk ? "Dirs OK        " : "FIX reversed   ");
}

// ─── 2. Wheel diameter ───────────────────────────────────────────────────────

void Chassis::measureWheelDiameter(pros::Controller& c, double actualInches) {
    banner("MEASURE WHEEL DIAMETER");
    const OdomSensors& s = odom.sensors();
    TrackingWheel* w[] = {s.vertical1, s.vertical2};

    char msg[160];
    snprintf(msg, sizeof(msg),
             "Mark the robot's start on a tape measure. Press A, push it STRAIGHT FORWARD exactly %.1f in, press A.",
             actualInches);
    waitForA(c, msg);
    double before[2]; for (int i = 0; i < 2; i++) before[i] = readOr(w[i]);
    waitForA(c, "Pushed? Press A.");

    for (int i = 0; i < 2; i++) {
        if (!w[i]) continue;
        const double measured = readOr(w[i]) - before[i];
        if (std::fabs(measured) < 1.0) { printf("  %s  barely moved - skipped\n", nm(i)); continue; }
        if (measured < 0) printf("  %s  read NEGATIVE - run checkWheelDirections first\n", nm(i));
        const double cfg = w[i]->getWheelDiameter();
        const double corrected = cfg * actualInches / std::fabs(measured);
        printf("  %s  read %7.2f in for %.1f actual  ->  diameter %.4f (was %.4f)  %+.2f%%\n",
               nm(i), measured, actualInches, corrected, cfg, (corrected / cfg - 1.0) * 100.0);
    }
    printf("  Paste the corrected diameters into your TrackingWheel constructors.\n");
}

// ─── 3. Tracking wheel offsets ───────────────────────────────────────────────
//
// Spin in place. A wheel at offset o from the center of rotation rolls o*θ:
//   vertical   dV = -xo*θ   =>  xo = -dV/θ
//   horizontal dH =  yo*θ   =>  yo =  dH/θ
// Requires directions to be correct (checkWheelDirections) - a reversed wheel
// gives a sign-flipped offset and the odometry then doubles the error instead
// of cancelling it.

void Chassis::measureTrackingOffsets(pros::Controller& c, double degrees) {
    banner("MEASURE TRACKING WHEEL OFFSETS");
    const OdomSensors& s = odom.sensors();
    if (!s.imu || !s.imu->isCalibrated()) {
        printf("  Needs a calibrated IMU. Call chassis.calibrate() first.\n");
        return;
    }
    TrackingWheel* w[] = {s.vertical1, s.vertical2, s.horizontal1, s.horizontal2};

    char msg[120];
    snprintf(msg, sizeof(msg), "Clear ~2 ft around the robot. It will spin in place %.0f deg. Press A to start.", degrees);
    waitForA(c, msg);

    double before[4]; for (int i = 0; i < 4; i++) before[i] = readOr(w[i]);
    const double r0 = s.imu->getRotation();

    // Slow controlled spin: P on remaining angle, capped, with a floor.
    const uint32_t start = pros::millis();
    while (pros::millis() - start < 15000) {
        const double turned = s.imu->getRotation() - r0;
        const double remaining = degrees - turned;
        if (remaining <= 0.0) break;
        double v = clamp(remaining * 0.6, 12.0, 40.0);
        tank(v, -v);
        pros::delay(10);
    }
    brake();
    pros::delay(600);   // let it fully settle before sampling

    const double thetaDeg = s.imu->getRotation() - r0;
    const double theta = toRad(thetaDeg);
    if (std::fabs(thetaDeg) < 90.0) {
        printf("  Only turned %.1f deg - drivetrain not moving? Aborting.\n", thetaDeg);
        return;
    }
    printf("  IMU says it turned %.1f deg\n\n", thetaDeg);

    for (int i = 0; i < 4; i++) {
        if (!w[i]) continue;
        const double travel = readOr(w[i]) - before[i];
        const bool vertical = i < 2;
        const double offset = vertical ? -travel / theta : travel / theta;
        const double cfg = w[i]->getOffset();
        const bool signMismatch = (cfg != 0.0) && (sign(offset) != sign(cfg));
        printf("  %s  rolled %+7.2f in  ->  offset %+.3f in   (configured %+.3f)%s\n",
               nm(i), travel, offset, cfg,
               signMismatch ? "   << SIGN DIFFERS from your config" : "");
    }

    if (s.vertical1 && s.vertical2) {
        const double dV1 = readOr(s.vertical1) - before[0];
        const double dV2 = readOr(s.vertical2) - before[1];
        const double span = s.vertical2->getOffset() - s.vertical1->getOffset();
        if (std::fabs(span) > 1e-6) {
            const double wheelTheta = toDeg((dV1 - dV2) / span);
            printf("\n  Cross-check: wheels imply %.1f deg vs IMU %.1f deg (%.1f%%)."
                   " Large gaps mean the diameters or offsets are off.\n",
                   wheelTheta, thetaDeg, (wheelTheta / thetaDeg - 1.0) * 100.0);
        }
    }
    printf("\n  Paste the measured offsets into your TrackingWheel constructors.\n");
    c.set_text(0, 0, "Offsets done   ");
}

// ─── 4. IMU scalar ───────────────────────────────────────────────────────────

void Chassis::measureImuScalar(pros::Controller& c, int turns) {
    banner("MEASURE IMU SCALAR");
    const OdomSensors& s = odom.sensors();
    if (!s.imu) { printf("  No IMU configured.\n"); return; }

    char msg[200];
    snprintf(msg, sizeof(msg),
             "Put a flat side of the robot against a straight edge. Press A, then rotate the robot exactly %d FULL turns "
             "and put the SAME side back against the edge. Press A.", turns);
    waitForA(c, msg);
    const double r0 = s.imu->getRotation();
    waitForA(c, "Back against the edge? Press A.");
    pros::delay(300);
    const double reported = std::fabs(s.imu->getRotation() - r0);

    if (reported < 90.0) { printf("  Only %.1f deg reported - aborting.\n", reported); return; }
    const double actual = 360.0 * turns;
    const double scalar = actual / reported * s.imu->getScalar();
    printf("  IMU reported %.2f deg for %.0f actual  ->  scalar %.5f  (drift %+.3f deg per turn)\n",
           reported, actual, scalar, (reported - actual) / turns);
    printf("  Apply with:  imu.setScalar(%.5f);   (before chassis.calibrate())\n", scalar);
    c.set_text(0, 0, "Scalar done    ");
}

// ─── 5. Drive balance ────────────────────────────────────────────────────────

void Chassis::tuneDriveBalance(int ms, double percent) {
    banner("DRIVE BALANCE (open loop, equal voltage both sides)");
    const OdomSensors& s = odom.sensors();
    const double l0 = readOr(s.vertical1), r0 = readOr(s.vertical2);
    const double h0 = s.imu ? s.imu->getRotation() : NAN;

    tank(percent, percent);
    pros::delay(ms);
    brake();
    pros::delay(400);

    const double dL = readOr(s.vertical1) - l0;
    const double dR = readOr(s.vertical2) - r0;
    const double dH = s.imu ? s.imu->getRotation() - h0 : NAN;

    if (std::isfinite(dL) && std::isfinite(dR)) {
        const double avg = (std::fabs(dL) + std::fabs(dR)) / 2.0;
        const double pct = avg > 0.01 ? (dL - dR) / avg * 100.0 : 0.0;
        printf("  L=%.2f in  R=%.2f in  diff=%+.2f in (%+.1f%%)", dL, dR, dL - dR, pct);
    } else {
        printf("  vertical travel: L=%.2f R=%.2f (need two vertical wheels for a per-side comparison)", dL, dR);
    }
    if (std::isfinite(dH)) printf("   heading drift=%+.2f deg", dH);
    printf("\n  Over ~1%% side difference or ~1 deg drift => mechanical asymmetry (dead/weak motor, polarity, friction). "
           "No controller tuning fixes that.\n");
}
