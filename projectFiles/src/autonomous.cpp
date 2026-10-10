#include "main.h"
#include "autonomous.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

// WP route: flip roller, score preload + 1 field piece on our alliance goal,
// score 1 more piece on the neutral goal (already holding 1) -> 4 pins across
// 2 goals, 40 pts, qualifies for the Autonomous Win Point on our side alone.
void leftAuton(Chassis& chassis) {
        chassis.setPose(0,0,0);
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
		chassis.moveToPoint(-6,-14.5, 750);
		chassis.turnToHeading(146, 950, {.earlyExitRange = 0.5});
		chassisMotors.move(-37.5);
		pros::delay(1200);
		chassisMotors.brake();
		claw.set_value(false);
		pros::delay(275);
		liftMotors.move_voltage(12000);
		pros::delay(150);
		chassis.turnToHeading(120, 650, {.async = true});
		pros::delay(350);
		liftMotors.brake();
		chassis.waitUntilDone();
		chassis.moveToPoint(-3.5, -13, 1000);
		chassis.turnToHeading(90, 750, {.minSpeed = 10, .earlyExitRange = 0.25});
		chassis.moveToPoint(-14, -13, 700, {.forwards = false});
		liftMotors.move_voltage(-12000);
		pros::delay(175);
		claw.set_value(true);
		liftMotors.brake();
		chassis.moveToPoint(-6, -13, 1000, {.async = true});
		liftMotors.move_voltage(-12000);
		pros::delay(200);
		liftMotors.brake();
		chassis.waitUntilDone();
		chassis.turnToHeading(27, 700, {.minSpeed = 5, .earlyExitRange = 0.15});
		chassis.moveToPoint(-15.5, -32, 1200, {.forwards = false});
		claw.set_value(false);
		pros::delay(175);
		liftMotors.move_voltage(12000);
		pros::delay(150);
		chassis.turnToHeading(-115, 900, {.async = true});
		pros::delay(750);
		liftMotors.brake();
		chassis.waitUntilDone();
		chassis.moveToPoint(18, -16.5, 1400, {.forwards = false, .maxSpeed = 80, .lateral = {.slew = 150}, .async = true});
		pros::delay(1300);
		liftMotors.set_brake_mode_all(pros::MotorBrake::coast);
		liftMotors.move_voltage(-9000);
		pros::delay(375);
		liftMotors.brake();
		claw.set_value(true);
}

// Same route mirrored (x negated) so it runs from the other starting tile.
void rightAuton(Chassis& chassis) {
     	chassis.setPose(0,0,0);
		roller.move(-127);
		pros::delay(500);
		roller.brake();
		chassis.moveToPoint(0 ,-14.5, 1200, {.forwards = false, .minSpeed = 7.5 , .earlyExitRange = 1});
		chassis.turnToHeading(90*r, 1200, {.minSpeed = 15, .earlyExitRange = 0.5});
		chassis.moveToPoint(-13.5*r, -14.5, 750, {.forwards = false, .async = true});
		liftMotors.move_voltage(12000);
		pros::delay(175);
		liftMotors.brake();
		chassis.waitUntilDone();
		liftMotors.move_voltage(-8000);
		pros::delay(200);
		liftMotors.brake();
		claw.set_value(true);
		pros::delay(200);
		chassis.moveToPoint(-6*r,-14.5, 750);
		chassis.turnToHeading(146*r, 950, {.earlyExitRange = 0.5});
		chassisMotors.move(-37.5);
		pros::delay(1200);
		chassisMotors.brake();
		claw.set_value(false);
		pros::delay(275);
		liftMotors.move_voltage(12000);
		pros::delay(150);
		chassis.turnToHeading(120*r, 650, {.async = true});
		pros::delay(350);
		liftMotors.brake();
		chassis.waitUntilDone();
		chassis.moveToPoint(-3.5*r, -13, 1000);
		chassis.turnToHeading(90*r, 750, {.minSpeed = 10, .earlyExitRange = 0.25});
		chassis.moveToPoint(-14*r, -13, 700, {.forwards = false});
		liftMotors.move_voltage(-12000);
		pros::delay(175);
		claw.set_value(true);
		liftMotors.brake();
		chassis.moveToPoint(-6*r, -13, 1000, {.async = true});
		liftMotors.move_voltage(-12000);
		pros::delay(200);
		liftMotors.brake();
		chassis.waitUntilDone();
		// chassis.turnToHeading(27*r, 700, {.minSpeed = 5, .earlyExitRange = 0.15});
		// chassis.moveToPoint(-15.5*r, -32, 1200, {.forwards = false});
		// claw.set_value(false);
		// pros::delay(175);
		// liftMotors.move_voltage(12000);
		// pros::delay(150);
		// chassis.turnToHeading(-115*r, 900, {.async = true});
		// pros::delay(750);
		// liftMotors.brake();
		// chassis.waitUntilDone();
		// chassis.moveToPoint(22.5*r, -16.5, 1400, {.forwards = false, .maxSpeed = 80, .lateral = {.slew = 150}, .async = true});
		// pros::delay(1300);
		// liftMotors.set_brake_mode_all(pros::MotorBrake::coast);
		// liftMotors.move_voltage(-9000);
		// pros::delay(375);
		// liftMotors.brake();
		// claw.set_value(true);
}

// Skills: flip roller, preload -> neutral stake, 7x matchload <-> neutral
// stake, then 3 field pieces -> alliance stake, park midfield. ~118 pts max.
void skillsAuton(Chassis& chassis) {
        chassis.setPose(0,0,0);
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
		chassis.moveToPoint(-1,-14.5, 750);
		chassis.turnToHeading(125, 1200);
		chassis.moveToPoint(-11, -5.5, 1200, {.forwards = false});
		chassis.swingToHeading(90, DriveSide::RIGHT, 1500);
		chassis.moveToPoint(-34, -4.5, 1200, {.forwards = false});
		chassis.turnToHeading(67, 1200);
		chassis.moveToPoint(-56, -13, 1500, {.forwards = false});
		chassis.turnToHeading(180, 1200);
		chassis.moveToPoint(-55, -1.5, 1200, {.forwards = false});
		claw.set_value(false);
		chassisMotors.move(-50);
		pros::delay(500);
		chassisMotors.brake();
		pros::delay(150);
		chassis.moveToPoint(-55, -14.5, 1500, {.async = true});
		liftMotors.move_voltage(12000);
		pros::delay(250);
		liftMotors.brake();
		chassis.waitUntilDone();
		chassis.turnToHeading(-90, 1200);
		chassis.moveToPoint(-25, -14.5, 1500, {.forwards = false,.async = true});
		liftMotors.move_voltage(12000);
		pros::delay(350);
		liftMotors.brake();
		chassis.waitUntilDone();
		liftMotors.move_voltage(-10000);
		pros::delay(350);
		claw.set_value(true);
		liftMotors.brake();
		pros::delay(200);
		chassis.moveToPoint(-53, -14.5, 1500);
		chassis.turnToHeading(180, 1200);
		chassis.moveToPoint(-55, -1.5, 1200, {.forwards = false});
		claw.set_value(false);
		chassisMotors.move(-50);
		pros::delay(500);
		chassisMotors.brake();
		pros::delay(150);




		// chassis.turnToHeading(146, 950, {.earlyExitRange = 0.5});
		// chassisMotors.move(-37.5);
		// pros::delay(1200);
		// chassisMotors.brake();
		// claw.set_value(false);
		// pros::delay(275);
		// liftMotors.move_voltage(12000);
		// pros::delay(150);
		// chassis.turnToHeading(120, 650, {.async = true});
		// pros::delay(350);
		// liftMotors.brake();
		// chassis.waitUntilDone();
		// chassis.moveToPoint(-3.5, -13, 1000);
		// chassis.turnToHeading(90, 750, {.minSpeed = 10, .earlyExitRange = 0.25});
		// chassis.moveToPoint(-14, -13, 700, {.forwards = false});
		// liftMotors.move_voltage(-12000);
		// pros::delay(175);
		// claw.set_value(true);
		// liftMotors.brake();
		// chassis.moveToPoint(-6, -13, 1000, {.async = true});
		// liftMotors.move_voltage(-12000);
		// pros::delay(200);
		// liftMotors.brake();
		// chassis.waitUntilDone();
		// chassis.turnToHeading(27, 700, {.minSpeed = 5, .earlyExitRange = 0.15});
		// chassis.moveToPoint(-15.5, -32, 1200, {.forwards = false});
		// claw.set_value(false);
		// pros::delay(175);
		// liftMotors.move_voltage(12000);
		// pros::delay(300);
		// liftMotors.brake();
        // chassis.turnToHeading(163, 1000);
		// liftMotors.move_voltage(12000);
		// pros::delay(500);
		// liftMotors.brake();
        // chassis.moveToPoint(-17.5, -19.5, 1200, {.forwards = false, .maxSpeed = 30});
        // liftMotors.move_voltage(-6000);
        // pros::delay(300);
        // liftMotors.brake();
        // claw.set_value(true); //second cup down
        // pros::delay(350);
        // chassis.moveToPoint(-17.5, -34, 1500, {.maxSpeed = 30});
        // liftMotors.move_voltage(-12000);
        // pros::delay(400);
        // liftMotors.brake();
        // chassis.turnToHeading(230, 1000);
        // chassis.moveToPoint(16.5, -4, 4000, {.forwards = false,.lateral = {.kP = 5.85}});
        // chassis.swingToHeading(270, DriveSide::LEFT, 1500);
		// chassis.moveToPoint(35, -4, 3000, {.forwards = false});
		// chassis.turnToHeading(-45, 1200);
		// liftMotors.move(120000);
		// pros::delay(350);
		// liftMotors.brake();
        // chassis.moveToPoint(69, -24, 4000, {.forwards = false,.maxSpeed = 100});
        // chassis.moveToPoint(64, -19, 2000);
        // chassis.turnToHeading(180, 1200, {.async = true});
		// liftMotors.move(-9000);
		// pros::delay(350);
		// liftMotors.brake();
		// claw.set_value(false); //grab third cup
		// pros::delay(250);
        // chassis.moveToPoint(63.5, 3, 1200, {.forwards = false});
		// pros::delay(1000);
		// chassis.moveToPoint(64, -12.5, 4000, {.async = true});
		// liftMotors.move_voltage(12000);
		// pros::delay(200);
		// liftMotors.brake();
		// chassis.waitUntilDone();
		// chassis.turnToHeading(90, 1200);
		// liftMotors.move_voltage(9000);
		// pros::delay(250);
		// liftMotors.brake();
		// chassis.moveToPoint(35, -14.5, 1500, {.forwards = false});
		// liftMotors.set_brake_mode_all(pros::MotorBrake::coast);
		// liftMotors.brake();
		// liftMotors.move(0);
		// pros::delay(350);
		// claw.set_value(true); //release third cup
		// pros::delay(250);
		// // chassis.moveToPoint(64, -20, 1500);
		// // chassis.turnToHeading(180, 1200);
		// // chassis.moveToPoint(64, 3, 1200, {.forwards = false});
		

}
