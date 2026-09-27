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

	if (!chassis.calibrate()) {
			printf("[robot] IMU FAILED to calibrate - check port 10 / reseat the sensor\n");
		}
		chassis.setPose(0, 0, 0);
		liftMotors.set_brake_mode_all(pros::MotorBrake::hold);
		liftMotors.set_gearing_all(pros::MotorCartridge::red);
	}

void disabled() {}
void competition_initialize() {}
	
void autonomous() {
	//gui::runSelectedAuton();
	chassis.setPose(0,0,0);
	chassis.turnToHeading(-45, 1500);
	chassis.moveToPoint(7.5,-9,2000, {.forwards = false});
	claw.set_value(true);
	pros::delay(300);
	chassis.moveToPoint(1.5,-4,1500);
	chassis.turnToHeading(48, 1500);
	chassisMotors.move(-27.5);
	pros::delay(1350);
	chassisMotors.brake();
	claw.set_value(false);
	pros::delay(200);
	chassis.turnToHeading(-65, 1200);
	chassis.moveToPoint(6, -11, 1200, {.forwards = false, .async = true});
	liftMotors.move_voltage(12000);
	pros::delay(450);
	liftMotors.brake();
	while(chassis.isInMotion()){
		pros::delay(10);
	}
	liftMotors.move_voltage(-12000);
	pros::delay(450);
	liftMotors.brake();
	claw.set_value(true);
	pros::delay(200);
	chassis.moveToPoint(0.5, -9.5, 1200);
	chassis.turnToHeading(-25, 1200);
	chassis.moveToPoint(8, -30.5, 1500, {.forwards = false});
	claw.set_value(false);
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
			printf("pose  x=%7.2f  y=%7.2f  th=%7.2f   v=%5.1f in/s  w=%6.1f deg/s%s\n",
			       p.x, p.y, p.theta,
			       chassis.getOdom().getLinearVelocity(), chassis.getOdom().getAngularVelocity(),
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
