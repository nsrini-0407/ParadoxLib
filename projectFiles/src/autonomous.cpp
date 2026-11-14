#include "main.h"
#include "autonomous.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

// WP route: flip roller, score preload + 1 field piece on our alliance goal,
// score 1 more piece on the neutral goal (already holding 1) -> 4 pins across
// 2 goals, 40 pts, qualifies for the Autonomous Win Point on our side alone.
void leftAuton(Chassis& chassis) {
	chassisMotors.move(30);
	roller.move(-127);
	pros::delay(500);
	roller.brake();
	chassisMotors.brake();
	chassis.moveToPoint(0, -16.5, 1500, {.forwards = false});
	chassis.turnToHeading(90, 700);
	chassis.setPose(-3, -16.5, 90);
	claw.move_voltage(-120000);
	chassis.moveToPoint(-15, -16.5, 800, {.forwards = false, .angular = {.kP = 2.25}});
	flipper.move_voltage(-12000);
	pros::delay(550);
	chassis.moveToPoint(-6.5, -16.5, 500, {.lateral = {.kP = 9.25}, .angular = {.kP = 2.25}});
	claw.move_voltage(12000);
	chassis.turnToHeading(142, 750, {.angular = {.smallError = 0.3, .largeError = 0.7}, .async = true});
	flipper.move_voltage(12000);
	pros::delay(400);
	flipper.brake();
	pros::delay(200);
	chassis.waitUntilDone();	
	chassis.moveToPoint(-16, -4.5, 800, {.forwards = false, .angular = {.kP = 1}});
	chassis.swingToHeading(185, DriveSide::LEFT, 500, {.async = true});
	chassis.waitUntilDone();
	pros::delay(400);
	chassis.turnToHeading(125, 675, {.async = true});
	liftMotors.move_voltage(12000);
	pros::delay(50);
	liftMotors.brake();
	chassis.waitUntilDone();
	chassis.moveToPoint(-2, -16.5, 1500);
	chassis.turnToHeading(90, 700, {.async = true});
	liftMotors.move_voltage(12000);
	pros::delay(500);
	liftMotors.brake();
	chassis.waitUntilDone();
	chassis.moveToPoint(-16, -16.5, 850, {.forwards = false});
	liftMotors.move_voltage(-9000);
	pros::delay(125);
	claw.move_voltage(-12000);
	pros::delay(50);
	liftMotors.brake();
	chassis.moveToPoint(-3, -16.5, 600, {.async = true});
	pros::delay(75);
	liftMotors.move_voltage(-12000);
	pros::delay(225);
	liftMotors.brake();
	chassis.waitUntilDone();
	chassis.turnToHeading(40, 700);
	chassis.moveToPoint(-18, -35, 1750, {.forwards = false, .maxSpeed = 85, .lateral = {.kP = 8.25}});
	claw.move_voltage(12000);
	pros::delay(300);
	liftMotors.move_voltage(12000);
	pros::delay(150);
	liftMotors.brake();
	chassis.turnToHeading(245, 850);
	chassis.moveToPoint(23, -20, 1300, {.forwards = false, .lateral = {.kP = 8.25}, .async = true});
	liftMotors.move_voltage(12000);
	pros::delay(600);
	liftMotors.brake();
	chassis.waitUntilDone();
	liftMotors.move_voltage(-10000);
	pros::delay(100);
	claw.move_voltage(-12000);
	pros::delay(50);
	liftMotors.brake();


	// chassis.moveToPoint(-2, -15, 1000, {.async = true});
	// liftMotors.move_voltage(12000);
	// pros::delay(700);
	// liftMotors.brake();
	// chassis.waitUntilDone();
	// chassis.turnToHeading(-90, 750);
	// chassis.moveToPoint(16, -20, 1000, {.forwards = false});
	// liftMotors.move_voltage(-12000);
	// pros::delay(100);
	// claw.move_voltage(-12000);
	// pros::delay(200);
	// liftMotors.brake();
}

// Same route mirrored (x negated) so it runs from the other starting tile.
void rightAuton(Chassis& chassis) {
    
}

// Skills: flip roller, preload -> neutral stake, 7x matchload <-> neutral
// stake, then 3 field pieces -> alliance stake, park midfield. ~118 pts max.
void skillsAuton(Chassis& chassis) {


}
