#include "main.h"
	
#include "pros/motors.hpp"
#include "liblvgl/lvgl.h"

LV_IMAGE_DECLARE(logo_map);  // matches whatever name the converter used




pros::Rotation leftRotation(1); // Rotation sensor on port 1
pros::Rotation rightRotation(2); // Rotation sensor on port 2
pros::Rotation backRotation(3); // Rotation sensor on port 3

TrackingWheel leftWheel(&leftRotation, 2.75, 1.0); // 2.75 inch diameter, 1:1 gearing
TrackingWheel rightWheel(&rightRotation, 2.75, 1.0); // 2.75 inch diameter, 1:1 gearing
TrackingWheel backWheel(&backRotation, 2.75, 1.0); // 2.75 inch diameter, 1:1 gearing

IMU imu(4); // IMU sensor on port 4

Odom odom(&leftWheel, &rightWheel, &backWheel, &imu, 11.5, 4.0); // 11.5 inch track width, 4 inch back wheel offset

pros::Motor leftFrontMotor(12);
pros::Motor leftBackMotor(13);
pros::Motor rightFrontMotor(14);
pros::Motor rightBackMotor(15);
pros::Motor intakeMotor(16);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	gui::setLogoImage(&logo_map);
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	gui::setWatchedMotors({
        {"L Front", &leftFrontMotor},
        {"L Back",  &leftBackMotor},
        {"R Front", &rightFrontMotor},
        {"R Back",  &rightBackMotor},
        {"Intake",  &intakeMotor},
        // add every motor you'd want warning on for a hotswap
    });

    gui::init();
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

	gui::runSelectedAuton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}