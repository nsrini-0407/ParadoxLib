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
	gui::forceSelectAuton("Skills");   // DEBUG: skip the touchscreen picker

	if (!chassis.calibrate()) {
			printf("[robot] IMU FAILED to calibrate - check port 11 / reseat the sensor\n");
			master.rumble("---");
		}
		master.rumble("."); 
		chassis.setPose(0, 0, 0); //CHANGE
		liftMotors.set_brake_mode_all(pros::MotorBrake::hold);
		liftMotors.set_gearing_all(pros::MotorCartridge::red);
		liftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
		optical.set_led_pwm(100);
		color = "Red";
	}

void disabled() {}
void competition_initialize() {}
	

void autonomous() {
	//gui::forceSelectAuton("Left WP");
	gui::runSelectedAuton();
}

void opcontrol() {
	
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	liftMotors.set_brake_mode_all(pros::MotorBrake::hold);
	chassis.setBrakeMode(pros::MotorBrake::coast);   // driver feel; auton re-sets brake
	uint32_t lastPrint = 0;
	pros::Task liftTask(liftControl);
	pros::Task rollTask([]{autoRoller(color);});

	while (true) {

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

		//create the boolean toggle to control the states
		if (master.get_digital_new_press(DIGITAL_B)) { //if a new press is registered
			if (clawToggle) { //if the claw is extended/true
				claw.set_value(false); //close the claw piston
				clawToggle = false; //update boolean accordingly
			} else if (!clawToggle) {//and vice versa! 
				claw.set_value(true);
				clawToggle = true;
			}
		}

		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			if (flipperToggle) {
				flipper.set_value(false);
				flipperToggle = false;
			} else if (!flipperToggle) {
				flipper.set_value(true);
				flipperToggle = true;
			}
		}

		if (master.get_digital_new_press(DIGITAL_X)) {
			chassis.setBrakeMode(pros::MotorBrake::hold);
			chassis.brake();
			rollTask.suspend();
			liftTask.suspend();
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
			rollTask.resume();
			liftTask.resume();
			chassis.setBrakeMode(pros::MotorBrake::coast);
		}
		pros::delay(20);
	}
}
