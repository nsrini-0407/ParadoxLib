#include "81yOdom/odometry/Odom.hpp"
#include "81yOdom/utils/math.hpp"
#include "pros/rtos.h"
#include <cmath>

void Odom::update() {
    const double dL = left->getDelta();
    const double dR = right->getDelta();
    const double dB = back->getDelta();

    left->recordPosition();
    right->recordPosition();
    back->recordPosition();

    const double currTheta = imu->getHeading();
    const double dTheta = toRad(normalizeAngle(currTheta - prevTheta));
    prevTheta = currTheta;
    pose.theta = currTheta;

    double localX;
    double localY;
    if (std::fabs(dTheta) < 1e-6) {
        localX = dB - (backOffset * dTheta);
        localY = (dL + dR) / 2.0;
    } else {
        const double radius = ((dL / dTheta) + (dR / dTheta)) / 2.0;
        localY = radius * std::sin(dTheta);

        const double radiusB = dB / dTheta - backOffset;
        localX = radiusB * (1 - std::cos(dTheta));
    }

    const double avgTheta = toRad(pose.theta) - (dTheta / 2.0);
    pose.x += localY * std::sin(avgTheta) + localX * std::cos(avgTheta);
    pose.y += localY * std::cos(avgTheta) - localX * std::sin(avgTheta);
}
