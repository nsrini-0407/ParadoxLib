#pragma once
#include <cmath>
#include "81yOdom/utils/math.hpp"

struct Pose {
    double x;       // inches, field x  (+ = right when facing +Y)
    double y;       // inches, field y  (+ = forward / "up" the field)
    double theta;   // degrees, CLOCKWISE positive, 0 = +Y, 90 = +X

    Pose(double x = 0.0, double y = 0.0, double theta = 0.0) 
        : x(x), y(y), theta(theta) {}

    // Straight-line distance to a point.
    double distanceTo(double tx, double ty) const {
        const double dx = tx - x, dy = ty - y;
        return std::sqrt(dx*dx + dy*dy);
    }
    double distanceTo(const Pose& p) const { return distanceTo(p.x, p.y); }

    // Bearing from this pose to a point, degrees CW from +Y. atan2(dx, dy) - not
    // (dy, dx) - because heading 0 is +Y, matching the integrator and the IMU.
    double angleTo(double tx, double ty) const {
        return toDeg(std::atan2(tx - x, ty - y));
    }
    double angleTo(const Pose& p) const { return angleTo(p.x, p.y); }

    // Shortest signed rotation from this heading to `targetAngle`, (-180, 180].
    double headingError(double targetAngle) const {
        return normalizeAngle(targetAngle - theta);
    }

    Pose operator+(const Pose& o) const { return {x + o.x, y + o.y, theta + o.theta}; }
    Pose operator-(const Pose& o) const { return {x - o.x, y - o.y, theta - o.theta}; }
};
