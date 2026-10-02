#pragma once
#include "81yOdom/chassis/Chassis.hpp"
#include "pros/adi.hpp"

// Defined in main.cpp - shared here so autonomous.cpp can drive them too.
extern pros::ADIDigitalOut claw;
extern pros::ADIDigitalOut flipper;

void leftAuton(Chassis& chassis);
void rightAuton(Chassis& chassis);
void skillsAuton(Chassis& chassis);
