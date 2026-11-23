#include "main.h"
#include "pros/adi.h"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"

#include <cmath>
#include <algorithm>


//Motors and sensors
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-8,-9,-10});
pros::MotorGroup rightMotors({1,2,3});
pros::MotorGroup chassisMotors({-8,-9,-10,1,2,3});
pros::MotorGroup liftMotors({4,-7});
pros::Motor roller(19);

pros::Motor leftFrontMotor(-8), leftBackMotor(-9),  leftTopMotor(-10);
pros::Motor rightFrontMotor(1), rightBackMotor(2), rightTopMotor(3);

pros::Rotation leftRotation(5);
pros::Rotation rightRotation(6);
pros::Rotation backRotation(20);
pros::Rotation liftRotation(8);
pros::Optical optical(18);

pros::Distance sideFrontDist(12);
pros::Distance sideRearDist(13);
pros::Distance backDist(14);

TrackingWheel leftWheel (&leftRotation,  2, -1.625,  1.0, true);
TrackingWheel rightWheel(&rightRotation, 2, +1.625,  1.0, true);
TrackingWheel backWheel (&backRotation,  2, +2.0, 1.0, true);

IMU imu(11);

pros::ADIDigitalOut claw ('A');
pros::ADIDigitalOut flipper ('B');

bool flipperToggle = false;
bool clawToggle = false;
int autonselector = 1;
int r = -1;
double currRoll = 0;
double currPitch = 0;
bool correction = false;
std::string color; 
double redHue = 20;
double blueHue = 210;
bool runTip = true;


constexpr double SIDE_SENSOR_DIR = -90.0;   // -90 left, +90 right
constexpr double BACK_SENSOR_DIR = 180.0;
constexpr double MAX_CORRECTION  = 6.0;
constexpr double MAX_SKEW_DEG    = 15.0;
constexpr double SIDE_AGREE_IN   = 1.5;
constexpr int    SAMPLES         = 5;


//Odom configuration settings
ControllerSettings lateralSettings {
    .kP = 7,  .kI = 0,  .kD = 1.05,
    .windupRange       = 3.0,
    .smallError        = 0.5,   .smallErrorTimeout = 75,
    .largeError        = 2.0,   .largeErrorTimeout = 125,
    .slew              = 300,   // percent per second. 0 disables the ramp
    .dFilter           = 0.715,
};
ControllerSettings angularSettings {
    .kP = 2.2,  .kI = 0.0,  .kD = 0.25,
    .windupRange       = 10.0,
    .smallError        = 0.5,   .smallErrorTimeout = 50,
    .largeError        = 1.0,   .largeErrorTimeout = 75,
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
void moveLift(int targetLevel) { //accepts the number of levels to move up/down as a parameter - sets the target position for the lift to move to
        const double kp = 8.25; //tuned kP constant
        const double stageConstant = 45; //each stage is 45 degrees apart, so we can use this to calculate the target position for each level
		double error; 
		double velocity;     
        const double exitNet = 3; //degrees of error that we will accept as "close enough" to the target position
		while (true) {
            //if the difference between the target position and the current position is less than the exitNet, we are close enough to the target position and can stop moving the lift
			if (std::abs((targetLevel * stageConstant) - liftRotation.get_position()) < exitNet) { 
				liftMotors.brake();
            //if this isn't the case, we move to the target using the PD loop to adjust velocity in real time based on the error 
			} else if (std::abs((targetLevel * stageConstant) - liftRotation.get_position()) > exitNet) {
				while (std::abs((targetLevel * stageConstant) - liftRotation.get_position()) > exitNet) {
					error = (targetLevel * stageConstant) - liftRotation.get_position();
					velocity = kp*error;
					liftMotors.move_voltage(velocity);	
				}
				
			}
			pros::delay(10); //delay to save resources
		}
    }

    void liftControl() { 
        while (true) {
            currRoll = imu.raw().get_roll();
            currPitch = imu.raw().get_pitch();
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
                liftMotors.move_voltage(-6000);
                chassis.setArcadeMaxSpeed(127);
            } else {
                liftMotors.brake();
            }
        }
            pros::delay(10);
    }

    void autoRoller(std::string team) {
        while (true) {
            if (master.get_digital(DIGITAL_R1)) {
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
            } else if (master.get_digital(DIGITAL_R2)) {
                roller.move_voltage(-12000);
            } else {
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
		claw.set_value(true);
		pros::delay(200);
    }

        
    // Distance Sensor Resets
    static double degToRad(double d) { return d * M_PI / 180.0; }

    static void readWalls(double& sideF, double& sideR, double& back) {
        pros::Distance* s[3] = {&sideFrontDist, &sideRearDist, &backDist};
        double sum[3] = {0, 0, 0};
        int n[3] = {0, 0, 0};
        for (int i = 0; i < SAMPLES; i++) {
            for (int j = 0; j < 3; j++) {
                const int mm = s[j]->get_distance();
                if (mm > 20 && mm < 2000) { sum[j] += mm; n[j]++; }
            }
            pros::delay(35);
        }
        auto avg = [&](int j) { return n[j] > SAMPLES / 2 ? sum[j] / n[j] / 25.4 : NAN; };
        sideF = avg(0); sideR = avg(1); back = avg(2);
    }

    static bool correctFromWall(double& x, double& y, double theta, double sensorDir,
                                double measured, double expected, const char* name) {
        if (expected <= 0 || !std::isfinite(measured)) return false;
        const double beam = theta + sensorDir;
        const long   wall = std::lround(beam / 90.0);
        const double skew = beam - wall * 90.0;
        if (std::fabs(skew) > MAX_SKEW_DEG) return false;
        const double err = measured * std::cos(degToRad(skew)) - expected;
        if (std::fabs(err) > MAX_CORRECTION) return false;
        switch (((wall % 4) + 4) % 4) {
            case 0: y -= err; break;
            case 1: x -= err; break;
            case 2: y += err; break;
            case 3: x += err; break;
        }
        printf("[recal] %s: read %.2f, expected %.2f, shift %.2f\n", name, measured, expected, err);
        return true;
    }

    // pass 0 to skip a sensor
    bool recalibrateOdom(double expectedSide, double expectedBack) {
        double sideF, sideR, back;
        readWalls(sideF, sideR, back);

        double side = NAN;
        if (std::isfinite(sideF) && std::isfinite(sideR))
            side = std::fabs(sideF - sideR) <= SIDE_AGREE_IN ? (sideF + sideR) / 2.0 : std::max(sideF, sideR);
        else if (std::isfinite(sideF)) side = sideF;
        else if (std::isfinite(sideR)) side = sideR;

        const Pose p = chassis.getPose();
        double x = p.x, y = p.y;
        const bool s = correctFromWall(x, y, p.theta, SIDE_SENSOR_DIR, side, expectedSide, "side");
        const bool b = correctFromWall(x, y, p.theta, BACK_SENSOR_DIR, back, expectedBack, "back");

        if (s || b) chassis.setPose(x, y, p.theta);
        printf("[recal] (%.2f, %.2f) -> (%.2f, %.2f)\n", p.x, p.y, x, y);
        return s || b;
    }

    void printWallReadings() {
        double f, r, b;
        readWalls(f, r, b);
        printf("[recal] sideF %.2f  sideR %.2f  back %.2f\n", f, r, b);
    }