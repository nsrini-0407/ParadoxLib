#include "main.h"
#include "pros/adi.h"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"


//Motors and sensors
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-19,-20});
pros::MotorGroup rightMotors({17,18});
pros::MotorGroup chassisMotors({17, 18, -19, -20});
pros::MotorGroup liftMotors({-1,2});
pros::Motor roller(3);
pros::Motor claw(-4);
pros::Motor intake(7);
pros::Motor flipper(5);
//2223
//2707


pros::Motor leftFrontMotor(-19), leftBackMotor(-20);
pros::Motor rightFrontMotor(17), rightBackMotor(18);

pros::Rotation leftRotation(15);
pros::Rotation rightRotation(16);
pros::Rotation backRotation(14);
pros::Rotation liftRotation(6);
pros::Optical optical(13);

TrackingWheel leftWheel (&leftRotation,  2, -1.5,  1.0, false);
TrackingWheel rightWheel(&rightRotation, 2, +1.5,  1.0, false);
TrackingWheel backWheel (&backRotation,  2, -1.25, 1.0, false);

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
    .kP = 8.85,  .kI = 0,  .kD = 1.35,
    .windupRange       = 3.0,
    .smallError        = 1,   .smallErrorTimeout = 75,
    .largeError        = 2.15,   .largeErrorTimeout = 150,
    .slew              = 300,   // percent per second. 0 disables the ramp
    .dFilter           = 0.715,
};
ControllerSettings angularSettings {
    .kP = 2.675,  .kI = 0.0,  .kD = 0.215,
    .windupRange       = 10.0,
    .smallError        = 1,   .smallErrorTimeout = 50,
    .largeError        = 1.25,   .largeErrorTimeout = 75,
    .slew              = 0,
    .dFilter           = 0.75,
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
                liftMotors.move_voltage(-10000);
                pros::delay(500);
                liftMotors.brake();
                correction = false;
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !correction) {
                //normal intake
                liftMotors.move_voltage(12000);
                chassis.setArcadeMaxSpeed(80);
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
                        roller.move_voltage(-12000);
                    } else {
                        roller.brake();
                    }             
                } else if (team == "Blue") {
                    if (std::abs(optical.get_hue() - blueHue) > 20) {
                        roller.move_voltage(-12000);
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
		chassis.moveToPoint(0 ,-14.5, 1200, {.forwards = false, .minSpeed = 7.5 , .earlyExitRange = 1});
		chassis.turnToHeading(90, 1200, {.minSpeed = 15, .earlyExitRange = 0.5});
		chassis.moveToPoint(-13.5, -14.5, 750, {.forwards = false, .async = true});
		liftMotors.move_voltage(12000);
		pros::delay(175);
		liftMotors.brake();
		chassis.waitUntilDone();
		liftMotors.move_voltage(-8000);
		pros::delay(200);
		liftMotors.brake();
		// claw.set_value(true);
		pros::delay(200);
    }


    void flipperControl() {
        while (true) {
            if (master.get_digital_new_press(DIGITAL_DOWN)) {
                flipper.move_voltage(-12000);
                pros::delay(550);
                flipper.brake();
                pros::delay(300);
                flipper.move_voltage(12000);
                pros::delay(550);
                flipper.brake();
            }
            pros::delay(10);
        }
    }