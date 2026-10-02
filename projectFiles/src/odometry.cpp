#include "81yOdom/odometry/Odom.hpp"
#include "81yOdom/utils/math.hpp"
#include "pros/rtos.hpp"
#include <cmath>


void Odom::setPose(Pose p) {
    if (s.imu) s.imu->setRotation(p.theta);
    ScopedLock lock(mutex);
    pose = p;
    prevRotation = p.theta;
    havePrevRotation = true;
}

Pose Odom::getPose() const {
    ScopedLock lock(mutex);
    return pose;
}

void Odom::reset() {
    TrackingWheel* wheels[] = {s.vertical1, s.vertical2, s.horizontal1, s.horizontal2};
    for (TrackingWheel* w : wheels) if (w) w->recordPosition();
    if (s.imu) {
        const double r = s.imu->getRotation();
        if (std::isfinite(r)) { prevRotation = r; havePrevRotation = true; }
    }
    lastUpdateMs = pros::millis();
}

bool Odom::anyWheelFaulted() const {
    TrackingWheel* wheels[] = {s.vertical1, s.vertical2, s.horizontal1, s.horizontal2};
    for (TrackingWheel* w : wheels) if (w && w->isFaulted()) return true;
    return false;
}

double Odom::getLinearVelocity()  const { ScopedLock l(mutex); return linVel; }
double Odom::getAngularVelocity() const { ScopedLock l(mutex); return angVel; }

void Odom::update() {
    //  Sample every sensor once, as close together as possible 
    const double dV1 = s.vertical1   ? s.vertical1->update()   : 0.0;
    const double dV2 = s.vertical2   ? s.vertical2->update()   : 0.0;
    const double dH1 = s.horizontal1 ? s.horizontal1->update() : 0.0;
    const double dH2 = s.horizontal2 ? s.horizontal2->update() : 0.0;
    const double imuRot = s.imu ? s.imu->getRotation() : NAN;

    const uint32_t now = pros::millis();
    double dt = (now - lastUpdateMs) / 1000.0;
    lastUpdateMs = now;
    if (dt <= 0.0 || dt > 0.5) dt = 0.01;   // first cycle / after a stall

    const bool v1ok = s.vertical1   && !s.vertical1->isFaulted();
    const bool v2ok = s.vertical2   && !s.vertical2->isFaulted();
    const bool h1ok = s.horizontal1 && !s.horizontal1->isFaulted();
    const bool h2ok = s.horizontal2 && !s.horizontal2->isFaulted();

    //  Heading change 
    double dThetaDeg = 0.0;
    if (std::isfinite(imuRot)) {
        if (havePrevRotation) dThetaDeg = imuRot - prevRotation;
        prevRotation = imuRot;
        havePrevRotation = true;
        headingFromImu = true;
    } else if (v1ok && v2ok) {
        // Wheel-derived fallback. Only trustworthy with two healthy verticals.
        const double span = s.vertical2->getOffset() - s.vertical1->getOffset();
        if (std::fabs(span) > 1e-6) {
            dThetaDeg = toDeg((dV1 - dV2) / span);
        }
        headingFromImu = false;
    }
    // else: no heading source this cycle; hold theta.

    const double dTheta = toRad(dThetaDeg);

    //  Forward displacement of the tracking center
    double dY = 0.0; int nV = 0;
    if (v1ok) { dY += dV1 + s.vertical1->getOffset() * dTheta; nV++; }
    if (v2ok) { dY += dV2 + s.vertical2->getOffset() * dTheta; nV++; }
    if (nV) dY /= nV;

    // Lateral displacement of the tracking center 
    double dX = 0.0; int nH = 0;
    if (h1ok) { dX += dH1 - s.horizontal1->getOffset() * dTheta; nH++; }
    if (h2ok) { dX += dH2 - s.horizontal2->getOffset() * dTheta; nH++; }
    if (nH) dX /= nH;

    //  Arc -> chord, then rotate into the field frame 
    // The wheels measured arc lengths. The straight-line displacement over the
    // cycle is the chord, 2*R*sin(dθ/2), and it points along the heading the
    // robot held at the MIDPOINT of the arc. The same scale applies to both
    // components - scaling only one of them was the original lateral bug.
    const double k = chordScale(dTheta);
    const double localX = dX * k;
    const double localY = dY * k;

    ScopedLock lock(mutex);
    const double newTheta = pose.theta + dThetaDeg;
    const double avg = toRad(newTheta) - dTheta / 2.0;
    const double fx = localY * std::sin(avg) + localX * std::cos(avg);
    const double fy = localY * std::cos(avg) - localX * std::sin(avg);

    pose.x += fx;
    pose.y += fy;
    pose.theta = normalizeAngle(newTheta);

    // Velocities. Forward speed is signed along the robot's own axis; heavy
    // filtering because these feed stall detection, not the control loop.
    linVel = lowPass(linVel, localY / dt, 0.7);
    angVel = lowPass(angVel, dThetaDeg / dt, 0.7);
}
