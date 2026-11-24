#include "main.h"
#include "autonomous.hpp"
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

// WP route: flip roller, score preload + 1 field piece on our alliance goal,
// score 1 more piece on the neutral goal (already holding 1) -> 4 pins across
// 2 goals, 40 pts, qualifies for the Autonomous Win Point on our side alone.
void leftAuton(Chassis& chassis) {
	// roller.move(-127);
	// pros::delay(500);
	// roller.brake();
	// chassisMotors.brake();
	// chassis.driveToPoint(0, -15.5, 1500, {.reverse = true});
	// chassis.turnToAngle(90, 700);
	// chassis.setPose(-3, -15.5, 90);
	// liftMotors.move_voltage(12000);
	// pros::delay(300);
	// liftMotors.brake();
	// chassis.driveToPoint(-15.5, -15.5, 1000, {.reverse = true, .turnTune = {.kP = 2.25}});
	// liftMotors.move_voltage(-8000);
	// pros::delay(250);
	// liftMotors.brake();
	// claw.move_voltage(-12000);
	// pros::delay(200);
	// chassis.driveToPoint(-6.5, -15.5, 500, {.driveTune = {.kP = 9.25}, .turnTune = {.kP = 2.25}});
	// claw.move_voltage(12000);
	// chassis.turnToAngle(142, 750, {.turnTune = {.smallError = 0.3, .largeError = 0.7}, .background = true});
	// // flipper.move_voltage(12000);
	// // pros::delay(400);
	// // flipper.brake();
	// // pros::delay(200);
	// if (liftRotation.get_position() > 3200) {
	// 	liftMotors.move_voltage(-12000);
	// 	pros::delay(350);
	// }
	// liftMotors.brake();
	// chassis.waitForMotion();	
	// chassis.driveToPoint(-16, -4.5, 800, {.reverse = true, .turnTune = {.kP = 1}});
	// chassis.swingToAngle(185, Pivot::LEFT, 500, {.background = true});
	// chassis.waitForMotion();
	// pros::delay(400);
	// chassis.turnToAngle(125, 675, {.background = true});
	// liftMotors.move_voltage(12000);
	// pros::delay(150);
	// liftMotors.brake();
	// chassis.waitForMotion();
	// chassis.driveToPoint(-2, -16.5, 1500);
	// chassis.turnToAngle(90, 700, {.background = true});
	// liftMotors.move_voltage(12000);
	// pros::delay(400);
	// liftMotors.brake();
	// chassis.waitForMotion();
	// chassis.driveToPoint(-16, -16.5, 900, {.reverse = true});
	// liftMotors.move_voltage(-9000);
	// pros::delay(125);
	// claw.move_voltage(-12000);
	// pros::delay(50);
	// liftMotors.brake();
	// chassis.driveToPoint(-3, -16.5, 600, {.background = true});
	// pros::delay(75);
	// liftMotors.move_voltage(-12000);
	// pros::delay(225);
	// liftMotors.brake();
	// chassis.waitForMotion();
	// chassis.turnToAngle(40, 700);
	// chassis.driveToPoint(-18.5, -36, 1750, {.reverse = true, .speedCap = 85, .driveTune = {.kP = 8.25}});
	// claw.move_voltage(12000);
	// pros::delay(300);
	// liftMotors.move_voltage(12000);
	// pros::delay(150);
	// liftMotors.brake();
	// chassis.turnToAngle(245, 850);
	// chassis.driveToPoint(23, -20, 1400, {.reverse = true, .driveTune = {.kP = 8}, .background = true});
	// liftMotors.move_voltage(12000);
	// pros::delay(600);
	// liftMotors.brake();
	// chassis.waitForMotion();
	// liftMotors.move_voltage(-10000);
	// pros::delay(100);
	// claw.move_voltage(-12000);
	// pros::delay(50);
	// liftMotors.brake();
	// claw.brake();
	chassis.setPose(0, 0, 0);
	pros::delay(500);
	chassis.driveToPoint(0, 12, 10000);
	chassis.driveToPoint(0, 36, 10000);

}

// Same route mirrored (x negated) so it runs from the other starting tile.
void rightAuton(Chassis& chassis) {
    roller.move(-127);
	pros::delay(500);
	roller.brake();
	chassisMotors.brake();
	chassis.driveToPoint(0, -15.5, 1500, {.reverse = true});
	chassis.turnToAngle(90*r, 700);
	chassis.setPose(-3*r, -15.5, 90*r);
	liftMotors.move_voltage(12000);
	pros::delay(300);
	liftMotors.brake();
	chassis.driveToPoint(-15.5*r, -15.5, 1000, {.reverse = true, .turnTune = {.kP = 2.25}});
	liftMotors.move_voltage(-8000);
	pros::delay(250);
	liftMotors.brake();
	claw.move_voltage(-12000);
	pros::delay(200);
	chassis.driveToPoint(-6.5*r, -15.5, 500, {.driveTune = {.kP = 9.25}, .turnTune = {.kP = 2.25}});
	claw.move_voltage(12000);
	chassis.turnToAngle(142*r, 750, {.turnTune = {.smallError = 0.3, .largeError = 0.7}, .background = true});
	// flipper.move_voltage(12000);
	// pros::delay(400);
	// flipper.brake();
	// pros::delay(200);
	if (liftRotation.get_position() > 3200) {
		liftMotors.move_voltage(-12000);
		pros::delay(350);
	}
	liftMotors.brake();
	chassis.waitForMotion();	
	chassis.driveToPoint(-16*r, -4.5, 800, {.reverse = true, .turnTune = {.kP = 1}});
	chassis.swingToAngle(185*r, Pivot::RIGHT, 500, {.background = true});
	chassis.waitForMotion();
	pros::delay(400);
	chassis.turnToAngle(125*r, 675, {.background = true});
	liftMotors.move_voltage(12000);
	pros::delay(150);
	liftMotors.brake();
	chassis.waitForMotion();
	chassis.driveToPoint(-2*r, -16.5, 1500);
	chassis.turnToAngle(90*r, 700, {.background = true});
	liftMotors.move_voltage(12000);
	pros::delay(400);
	liftMotors.brake();
	chassis.waitForMotion();
	chassis.driveToPoint(-16*r, -16.5, 900, {.reverse = true});
	liftMotors.move_voltage(-9000);
	pros::delay(125);
	claw.move_voltage(-12000);
	pros::delay(50);
	liftMotors.brake();
	pros::delay(750);
	chassis.driveToPoint(-3*r, -16.5, 600, {.background = true});
	pros::delay(450);
	liftMotors.move_voltage(-12000);
	pros::delay(225);
	liftMotors.brake();
	claw.brake();
	chassis.waitForMotion();
	chassis.turnToAngle(40*r, 700);
	chassis.driveToPoint(-18.5*r, -36, 1750, {.reverse = true, .speedCap = 85, .driveTune = {.kP = 8.25}});
	claw.move_voltage(12000);
	pros::delay(300);
	liftMotors.move_voltage(12000);
	pros::delay(150);
	liftMotors.brake();
	chassis.turnToAngle(245*r, 850);
	chassis.driveToPoint(23*r, -20, 1400, {.reverse = true, .driveTune = {.kP = 8}, .background = true});
	liftMotors.move_voltage(12000);
	pros::delay(600);
	liftMotors.brake();
	chassis.waitForMotion();
	liftMotors.move_voltage(-10000);
	pros::delay(100);
	claw.move_voltage(-12000);
	pros::delay(50);
	liftMotors.brake();
	claw.brake();
}

// Skills: flip roller, preload -> neutral stake, 7x matchload <-> neutral
// stake, then 3 field pieces -> alliance stake, park midfield. ~118 pts max.
void skillsAuton(Chassis& chassis) {


}
