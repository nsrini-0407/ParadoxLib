#pragma once
#include <cmath>
#include <algorithm>

// Library-wide conventions. Every file in 81yOdom assumes these.
//
//   Distance   inches
//   Angle      degrees, CLOCKWISE positive, 0 = +Y (field "up" / robot forward)
//              90 = +X. This is the compass/bearing convention. It is what makes
//              the integrator  x += d*sin(theta), y += d*cos(theta)  and
//              Pose::angleTo = atan2(dx, dy) consistent with each other and
//              with the V5 IMU, which reports clockwise-positive natively.
//   Time       milliseconds for API arguments, seconds inside math
//   Output     percent, -100..100. 100 = 12000 mV.

constexpr double PI = 3.14159265358979;

inline double toRad(double deg) { return deg * PI / 180.0; }
inline double toDeg(double rad) { return rad * 180.0 / PI; }

inline double clamp(double val, double minVal, double maxVal) {
	return std::max(minVal, std::min(maxVal, val)); 
}

// Normalize to (-180, 180]. fmod rather than a while-loop: a faulted sensor
// hands us PROS_ERR_F (infinity), and `while (a <= -180) a += 360` never
// terminates on an infinity. Non-finite input passes through untouched so a
// caller's isfinite() guard still sees it.
inline double normalizeAngle(double angle) {
	if (!std::isfinite(angle)) return angle;
	angle = std::fmod(angle, 360.0);
	if (angle > 180.0) angle -= 360.0;
	else if (angle <= -180.0) angle += 360.0;
	return angle;
}

// Normalize to [0, 360).
inline double normalizeAngle360(double angle) {
	if (!std::isfinite(angle)) return angle;
	angle = std::fmod(angle, 360.0);
	if (angle < 0.0) angle += 360.0;
	return angle;
}

// -1, 0, or 1
inline double sign(double val) {
	if (val > 0) return 1.0;
	else if (val < 0) return -1.0;
	else return 0.0;
}

// Rate limiter: moves `current` toward `target` by at most maxChange*dt.
inline double slew(double target, double current, double maxChange, double dt) {
	if (maxChange <= 0.0) return target;   // 0 disables the limiter
	double maxDelta = maxChange * dt;
	return current + clamp(target - current, -maxDelta, maxDelta);
}

// First-order low-pass. alpha in [0,1): 0 = no filtering, ->1 = heavy.
inline double lowPass(double prev, double sample, double alpha) {
	return alpha * prev + (1.0 - alpha) * sample;
}

// Chord scale factor for arc odometry: 2*sin(t/2)/t, continuous -> 1 as t -> 0.
// Over an arc through angle t the straight-line displacement (the chord) is
// 2*R*sin(t/2), while the wheels measure the arc length R*t. This converts one
// to the other for BOTH the forward and lateral components.
inline double chordScale(double dThetaRad) {
	return (std::fabs(dThetaRad) < 1e-9) ? 1.0 : (2.0 * std::sin(dThetaRad / 2.0) / dThetaRad);
}
