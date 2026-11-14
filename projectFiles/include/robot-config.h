#pragma once
#include "main.h"

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup chassisMotors;
extern pros::MotorGroup liftMotors;
extern pros::Motor roller;
extern pros::Motor flipper;
extern pros::Motor claw;
extern pros::Motor intake;

extern pros::Controller master;

extern pros::Motor leftFrontMotor;
extern pros::Motor leftBackMotor;
extern pros::Motor rightFrontMotor;
extern pros::Motor rightBackMotor;

extern pros::Rotation leftRotation;
extern pros::Rotation rightRotation;
extern pros::Rotation backRotation;
extern pros::Rotation liftRotation;
extern pros::Optical optical;

extern TrackingWheel leftWheel;
extern TrackingWheel rightWheel;
extern TrackingWheel backWheel;
extern IMU imu;


extern ControllerSettings lateralSettings;
extern ControllerSettings angularSettings;
extern Chassis chassis;

extern bool flipperToggle;
extern bool CT1;
extern bool CT2;
extern int autonselector;
extern int r;
extern double currRoll;
extern double currPitch;
extern bool correction;
extern std::string color; 
extern double redHue;
extern double blueHue;
extern double liftVelocity;

extern void moveLift(double targetLevel);
extern void liftControl();
extern void autoRoller(std::string team);
extern void skillsStart();
extern void flipperControl();


extern int autonselector;
extern int r;
