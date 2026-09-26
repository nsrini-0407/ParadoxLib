#pragma once
#include "main.h"

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup liftMotors;

extern pros::Motor leftFrontMotor;
extern pros::Motor leftBackMotor;
extern pros::Motor leftTopMotor;
extern pros::Motor rightFrontMotor;
extern pros::Motor rightBackMotor;
extern pros::Motor rightTopMotor;

extern pros::Rotation leftRotation;
extern pros::Rotation rightRotation;
extern pros::Rotation backRotation;
extern pros::Rotation liftRotation;

extern TrackingWheel leftWheel;
extern TrackingWheel rightWheel;
extern TrackingWheel backWheel;
extern IMU imu;

extern pros::ADIDigitalOut claw;
extern pros::ADIDigitalOut flipper;

extern ControllerSettings lateralSettings;
extern ControllerSettings angularSettings;
extern Chassis chassis;

extern bool flipperToggle;
extern bool clampToggle;

extern void moveLift(int targetLevel);
extern void scorePin();
