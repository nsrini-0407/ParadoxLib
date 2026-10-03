#include "main.h"
#include "autonomous.hpp"
#include "liblvgl/lvgl.h"
#include <cstdio>

LV_IMAGE_DECLARE(logo);

static gui::OdomDebugData get_odom_debug_data() {
	const Pose pose = chassis.getPose();
	return {pose.x, pose.y, pose.theta};
}

void initialize() {
	pros::lcd::initialize(); 
	gui::setWatchedMotors({
        {"L Front", &leftFrontMotor}, {"L Back", &leftBackMotor}, {"L Top", &leftTopMotor},
        {"R Front", &rightFrontMotor}, {"R Back", &rightBackMotor}, {"R Top", &rightTopMotor},
        // {"Intake", &intakeMotor},   // once intakeMotor has a real port
    });
	gui::setLogoImage(&logo);
	gui::setOdomDebugProvider(get_odom_debug_data);
	gui::setAutonRoutines({
        {"Left WP",   [](){ leftAuton(chassis); }},
        {"Right WP",  [](){ rightAuton(chassis); }},
        {"Skills",    [](){ skillsAuton(chassis); }},
        {"Do Nothing",[](){}},
    });
	gui::init();
	// gui::forceSelectAuton("Skills");   // DEBUG: skip the touchscreen picker

	if (!chassis.calibrate()) {
			printf("[robot] IMU FAILED to calibrate - check port 11 / reseat the sensor\n");
			master.rumble("---");
		}
		master.rumble("."); 
		chassis.setPose(0, 0, 0); //CHANGE
		liftMotors.set_brake_mode_all(pros::MotorBrake::hold);
		liftMotors.set_gearing_all(pros::MotorCartridge::red);
	}

void disabled() {}
void competition_initialize() {}
	
void autonomous() {
	gui::forceSelectAuton("Skills");
	gui::runSelectedAuton();
	
	// chassisMotors.move(-75);
	// pros::delay(1100);
	// // chassis.moveToPose(23, -14.5, -90, 2500, {.forwards = false});
	// liftMotors.move_voltage(-12000);
	// pros::delay(350);
	// liftMotors.brake();
	// claw.set_value(true);

	// chassis.setPose(0,0,0);
	// roller.move(-127);
	// pros::delay(500);
	// roller.brake();
	// chassis.moveToPoint(0 ,-15.5, 1200, {.forwards = false, .minSpeed = 7.5 , .earlyExitRange = 1});
	// chassis.turnToHeading(90, 1200, {.minSpeed = 15, .earlyExitRange = 0.5});
	// chassis.moveToPoint(-14.5, -15.5, 1000, {.forwards = false, .async = true});
	// liftMotors.move_voltage(12000);
	// pros::delay(300);
	// liftMotors.brake();
	// chassis.waitUntilDone();
	// claw.set_value(true);
	// pros::delay(175);
	// chassis.moveToPoint(-6, -15.5, 1000, {.async = true});
	// liftMotors.move_voltage(-8000);
	// pros::delay(350);
	// liftMotors.brake();
	// chassis.waitUntilDone();
	// chassis.turnToHeading(145, 750, {.earlyExitRange = 0.5});
	// chassisMotors.move(-31.5);
	// pros::delay(1200);
	// chassisMotors.brake();
	// claw.set_value(false);
	// pros::delay(175);
	// chassis.turnToHeading(135, 800);
	// chassis.moveToPoint(-3.5, -13.5, 1000, {.async = true});
	// pros::delay(250);
	// liftMotors.move_voltage(12000);
	// pros::delay(500);
	// liftMotors.brake();
	// chassis.waitUntilDone();
	// chassis.turnToHeading(90, 1000);
	// chassis.moveToPoint(-14, -15, 1200, {.forwards = false});
	// liftMotors.move_voltage(-12000);
	// pros::delay(175);
	// claw.set_value(true);
	// liftMotors.brake();
	// chassis.moveToPoint(-6.5, -15, 1000, {.async = true});
	// liftMotors.move_voltage(-12000);
	// pros::delay(200);
	// liftMotors.brake();
	// chassis.waitUntilDone();
	// chassis.turnToHeading(30, 1000);
	// chassis.moveToPoint(-16.5, -32.5, 1200, {.forwards = false});
	// claw.set_value(false);
	// pros::delay(175);
	// chassis.turnToHeading(-124, 1000, {.async = true});
	// liftMotors.move_voltage(12000);
	// pros::delay(750);
	// liftMotors.brake();
	// chassis.waitUntilDone();
	// chassis.moveToPose(23, -15, -90, 2000, {.forwards = false});


}

void opcontrol() {
	
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	chassis.setBrakeMode(pros::MotorBrake::coast);   // driver feel; auton re-sets brake
	uint32_t lastPrint = 0;

	while (true) {
		// const bool bench = !pros::competition::is_connected();
		// if (bench) {
		// 	if (master.get_digital_new_press(DIGITAL_A))    { chassis.brake(); autonomous(); }
		// 	if (master.get_digital_new_press(DIGITAL_B))    chassis.tuneDriveBalance();
		// 	if (master.get_digital_new_press(DIGITAL_X))    chassis.checkWheelDirections(master);
		// 	if (master.get_digital_new_press(DIGITAL_Y))    chassis.measureTrackingOffsets(master);
		// 	if (master.get_digital_new_press(DIGITAL_UP))   chassis.measureWheelDiameter(master, 48);
		// 	if (master.get_digital_new_press(DIGITAL_DOWN)) chassis.measureImuScalar(master, 5);
		// }

		//arcade driving 
		const double throttle = master.get_analog(ANALOG_LEFT_Y)  * (100.0 / 127.0);
		const double turn     = master.get_analog(ANALOG_RIGHT_X) * (100.0 / 127.0);
		chassis.arcade(throttle, turn);

		if (pros::millis() - lastPrint >= 500) {
			lastPrint = pros::millis();
			const Pose p = chassis.getPose();
			printf("pose  x=%7.2f  y=%7.2f  th=%7.2f   v=%5.1f in/s  w=%6.1f deg/s  rejected=%lu%s\n",
			       p.x, p.y, p.theta,
			       chassis.getOdom().getLinearVelocity(), chassis.getOdom().getAngularVelocity(),
			       (unsigned long)chassis.getOdom().getRejectedSamples(),
			       chassis.getOdom().isHeadingFromImu() ? "" : "   [heading from WHEELS - IMU down]");
		}
		//l1 - up 4 bar, l2 down 4bar, b claw, down flipper
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                //normal intake
                liftMotors.move_voltage(12000);
            }else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftMotors.move_voltage(-12000);
            } else {
                liftMotors.brake();
            }
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			roller.move_voltage(-12000);
		} else {
			roller.brake();
		}

		//create the boolean toggle to control the states
		if (master.get_digital_new_press(DIGITAL_B)) { //if a new press is registered
			if (clawToggle) { //if the claw is extended/true
				claw.set_value(false); //close the claw piston
				clawToggle = false; //update boolean accordingly
			} else if (!flipperToggle) {//and vice versa! 
				claw.set_value(true);
				clawToggle = true;
			}
		} else if (master.get_digital_new_press(DIGITAL_DOWN)) {
			if (flipperToggle) {
				flipper.set_value(false);
				flipperToggle = false;
			} else if (!flipperToggle) {
				flipper.set_value(true);
				flipperToggle = true;
			}
		}

		pros::delay(20);
	}
}
