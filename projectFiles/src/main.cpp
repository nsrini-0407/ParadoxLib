#include "main.h"
#include "autonomous.hpp"
#include "liblvgl/lvgl.h"
#include <cstdio>

LV_IMAGE_DECLARE(logo);

static gui::OdomDebugData get_odom_debug_data() {
	const Pose pose = chassis.getPose();
	// get_position() is centidegrees (same units moveLift uses); PROS_ERR means unplugged
	const int32_t liftRaw = liftRotation.get_position();
	const double lift = (liftRaw == PROS_ERR) ? NAN : (double)liftRaw;
	return {pose.x, pose.y, pose.theta, lift};
}

void initialize() {
	pros::lcd::initialize(); 
	gui::setWatchedMotors({
        {"L Front", &leftFrontMotor}, {"L Back", &leftBackMotor},
        {"R Front", &rightFrontMotor}, {"R Back", &rightBackMotor},
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
	//gui::forceSelectAuton("Left WP");   // DEBUG: skip the touchscreen picker

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
		chassis.setBrakeMode(pros::MotorBrake::hold);
		chassis.setPose(0,0,0);
	}

void disabled() {}
void competition_initialize() {}
	

void autonomous() {
	gui::forceSelectAuton("Left WP");
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
		const double turn     = -1 * master.get_analog(ANALOG_RIGHT_X) * (100.0 / 127.0);
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
		// if (master.get_digital(DIGITAL_R1) || master.get_digital(DIGITAL_R2)) { //if a new press is registered
		// 	if (master.get_digital(DIGITAL_R1)){
		// 		claw.move_voltage(12000);
		// 	}
		// 	if (master.get_digital(DIGITAL_R2)){
		// 		claw.move_voltage(-12000);
		// 	}
		// }

		if (master.get_digital_new_press(DIGITAL_R1)) {
			if (!CT1) {
				claw.move_voltage(12000);
				CT1 = true;
				CT2 = false;
			} else if (CT1) {
				claw.move_voltage(4000);
				CT1 = false;
				CT2 = false;
			}
		}
		if (master.get_digital_new_press(DIGITAL_R2)) {
			if (!CT2) {
				claw.move_voltage(-12000);
				CT2 = true;
				CT1 = false;
			} else if (CT2) {
				claw.move_voltage(4000);
				CT2 = false;
				CT1 = false;
			}
		}
		pros::delay(20);
	}
}
