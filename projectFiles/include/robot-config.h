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

void spinRoller();   // spin drivetrain/roller against the field roller until it flips
void closeClamp();   // close the goal clamp around a mobile goal
void openClamp();    // release the goal clamp
void liftUp();        // raise the lift to scoring height
void liftDown();       // lower the lift back down
void intakePiece();  // run the intake to grab a pin/cup or matchload
void scoreGoal();    // dump the held piece(s) onto whichever goal is in front