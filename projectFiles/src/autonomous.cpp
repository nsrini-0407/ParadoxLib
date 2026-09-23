#include "autonomous.hpp"

void autonLeftWP(Chassis& chassis) {
    chassis.setPose(-46, -46, 0);

    chassis.moveToPoint(-46, -22, 2000);
    chassis.moveToPoint(-56, -22, 1500);
    chassis.moveToPoint(-46, -22, 1500);
    chassis.moveToPoint(-40, -14, 1500);
    chassis.moveToPoint(-22, -22, 2000);
    chassis.moveToPoint(-46, -6, 2000);
    chassis.moveToPoint(-68, -6, 1500);
    chassis.moveToPoint(-46, 22, 2500);
}

void autonRightWP(Chassis& chassis) {
    chassis.setPose(-12, -68, 0);

    chassis.moveToPoint(-22, -45, 2000);
    chassis.moveToPoint(-24, -23, 2000);
    chassis.moveToPoint(-11, -40, 1500);
    chassis.moveToPoint(-47, -45, 2500);
    chassis.moveToPoint(-32, -59, 2000);
    chassis.moveToPoint(21, -45, 3000);
}

void autonSkills(Chassis& chassis) {
    chassis.setPose(-68, 57, 0);

    chassis.moveToPoint(-44, 25, 2500);
    chassis.moveToPoint(-66, 24, 1500);
    chassis.moveToPoint(-45, -22, 3000);
    chassis.moveToPoint(-37, -12, 1500);
    chassis.moveToPoint(-22, -22, 1500);
    chassis.moveToPoint(-35, -31, 1500);
    chassis.moveToPoint(-45, -44, 1500);
    chassis.moveToPoint(-8, 1, 2500);
}
