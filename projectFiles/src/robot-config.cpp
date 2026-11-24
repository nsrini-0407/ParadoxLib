#include "main.h"
#include "pros/adi.h"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"


//Motors and sensors
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({1,2});
pros::MotorGroup rightMotors({-8,-9});
pros::MotorGroup chassisMotors({-8, -9, 1, 2});
pros::MotorGroup liftMotors({12,-13});
pros::MotorGroup roller({3, -20});
pros::Motor claw(-17);
//2223
//2707

pros::Motor leftFrontMotor(1);
pros::Motor leftBackMotor(2);
pros::Motor rightFrontMotor(-8);
pros::Motor rightBackMotor(-9);


pros::Rotation leftRotation(15);
pros::Rotation rightRotation(14);
pros::Rotation backRotation(19);
pros::Rotation liftRotation(16);
pros::Optical optical(7);
pros::Distance clampDistance(11);
pros::Distance backRightDist(4);
pros::Distance frontRightDist(6);
pros::Distance backSideDist(5);


TrackingWheel leftWheel (&leftRotation,  2, -1.5,  1.0, true);
TrackingWheel rightWheel(&rightRotation, 2, +1.5,  1.0, true);
TrackingWheel backWheel (&backRotation,  2, -1.25, 1.0, true);

IMU imu(21);


bool flipperToggle = false;
bool CT1 = false;
bool CT2 = false;
int autonselector = 1;
int r = -1;
double currRoll = 0;
double currPitch = 0;
bool correction = false;
std::string color; 
double redHue = 20;
double blueHue = 210;
bool runTip = true;
double liftVelocity = 0;
const double DOWN = 0;


//Odom configuration settings
ControllerSettings lateralSettings {
    .kP = 31,  .kI = 0,  .kD = 7.75,
    .windupRange       = 3.0,
    .smallError        = 0.5,   .smallErrorTimeout = 125,
    .largeError        = 2,   .largeErrorTimeout = 450,
    .slew              = 400,   // percent per second. 0 disables the ramp
    .dFilter           = 0.6,
};
ControllerSettings angularSettings {
    .kP = 0,  .kI = 0.0,  .kD = 0,
    .windupRange       = 10.0,
    .smallError        = 1.25,   .smallErrorTimeout = 125,
    .largeError        = 3.5,   .largeErrorTimeout = 450,
    .slew              = 150,
    .dFilter           = 0.6 ,
};

Chassis chassis(
    DrivetrainConfig{ .leftMotors = &leftMotors, .rightMotors = &rightMotors,
                      .trackWidth = 11.5, .wheelDiameter = 3.25, .gearRatio = 1.0 },
    lateralSettings,
    angularSettings,
    OdomSensors{ .vertical1 = &leftWheel, .vertical2 = &rightWheel,
                 .horizontal1 = &backWheel, .imu = &imu }
);

//Helper functions for autonomous routines

//This is a simplified PD loop for controlling the double reverse 4 bar to different level stages up and down 
// for stacking, flipping toggles, etc. - PD helps with keeping movements balanced and precise
void moveLift(double targetLevel) { //accepts the number of levels to move up/down as a parameter - sets the target position for the lift to move to
        const double kp = 9; //tuned kP constant
		double error; 
		double velocity;     
        const double exitNet = 750; //degrees of error that we will accept as "close enough" to the target positionerror = targetLevel - liftRotation.get_position();
        error = std::abs(targetLevel - liftRotation.get_position());
		while (true) {
            //if the difference between the target position and the current position is less than the exitNet, we are close enough to the target position and can stop moving the lift
			if (error < exitNet) { 
				liftMotors.brake();
            //if this isn't the case, we move to the target using the PD loop to adjust velocity in real time based on the error 
			} else if (error > exitNet) {
				while (error > exitNet) {
					error = std::abs(targetLevel - liftRotation.get_position());
					velocity = kp*error;
					liftMotors.move_voltage(velocity);	
                    pros::delay(10); //delay to save resources
				}
                pros::delay(10);
				
			}
			pros::delay(10); //delay to save resources
		}
    }

    void liftControl() { 
        while (true) {
            currRoll = imu.raw().get_roll();
            currPitch = imu.raw().get_pitch();
            liftVelocity = liftMotors.get_actual_velocity_all().at(0);  
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                runTip = false;
                correction = false;
            }
            if ((currRoll > 10 || currRoll < -10 || currPitch > 10 || currPitch < -10) && runTip) {
                correction = true;
                liftMotors.move_voltage(-12000);
                pros::delay(600);
                liftMotors.brake();
                correction = false;
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !correction) {
                //normal intake
                if (liftRotation.get_position() > 11750) {
                    liftMotors.brake();
                } else {
                    liftMotors.move_voltage(12000);
                    chassis.setArcadeMaxSpeed(80);
                }
            }else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !correction) {
                if (liftRotation.get_position() < 2850) {
                    liftMotors.brake();
                } else {
                    liftMotors.move_voltage(-9000);
                    chassis.setArcadeMaxSpeed(127);
                }
                // if (liftVelocity < 20) {
                //     roller.move_voltage(-12000);
                // }
            } else {
                liftMotors.brake();
            }
            pros::delay(10);
        }
    }

    void autoRoller(std::string team) {
        while (true) {
            if (master.get_digital(DIGITAL_B)) {
                if (team == "Red") {
                    if (std::abs(optical.get_hue() - redHue) > 20) {
                        roller.move_voltage(10500);
                    } else {
                        roller.brake();
                    }             
                } else if (team == "Blue") {
                    if (std::abs(optical.get_hue() - blueHue) > 20) {
                        roller.move_voltage(10500);
                    } else {
                        roller.brake();
                    } 
                }
            }
            else {
                roller.brake();
            }
            pros::delay(10);
        }
    }

    void skillsStart() {
		roller.move(-127);
		pros::delay(500);
		roller.brake();
		chassis.driveToPoint(0 ,-14.5, 1200, {.reverse = true, .speedFloor = 7.5 , .handoffRange = 1});
		chassis.turnToAngle(90, 1200, {.speedFloor = 15, .handoffRange = 0.5});
		chassis.driveToPoint(-13.5, -14.5, 750, {.reverse = true, .background = true});
		liftMotors.move_voltage(12000);
		pros::delay(175);
		liftMotors.brake();
		chassis.waitForMotion();
		liftMotors.move_voltage(-8000);
		pros::delay(200);
		liftMotors.brake();
		// claw.set_value(true);
		pros::delay(200);
    }