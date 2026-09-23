#include "autonomous.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

void spinRoller()   { printf("[auton] spinRoller\n");   pros::delay(300); }
void closeClamp()   { printf("[auton] closeClamp\n");   pros::delay(200); }
void openClamp()    { printf("[auton] openClamp\n");    pros::delay(200); }
void liftUp()       { printf("[auton] liftUp\n");       pros::delay(300); }
void liftDown()     { printf("[auton] liftDown\n");     pros::delay(300); }
void intakePiece()  { printf("[auton] intakePiece\n");  pros::delay(300); }
void scoreGoal()    { printf("[auton] scoreGoal\n");    pros::delay(300); }

// WP route: flip roller, score preload + 1 field piece on our alliance goal,
// score 1 more piece on the neutral goal (already holding 1) -> 4 pins across
// 2 goals, 40 pts, qualifies for the Autonomous Win Point on our side alone.
void autonLeftWP(Chassis& chassis) {
    chassis.setPose(-48, -48, 0);        // start touching the roller
    spinRoller();                        // flip roller to our color

    chassis.moveToPoint(-48, -24, 1500); // drive to our alliance goal
    closeClamp();                        // clamp the alliance goal
    liftUp();                            // raise lift to scoring height
    scoreGoal();                         // score preload onto alliance goal

    chassis.moveToPoint(-36, -18, 1200); // drive to a pin/cup on the field
    intakePiece();                       // pick up the pin and cup
    scoreGoal();                         // score it onto the alliance goal
    liftDown();                          // lower lift
    openClamp();                         // release the alliance goal

    chassis.moveToPoint(-12, -6, 1500);  // drive to the neutral goal
    liftUp();                            // raise lift to scoring height
    scoreGoal();                         // score final pin (goal already has 1 -> 2 there)
    liftDown();                          // lower lift, route done
}

// Same route mirrored (x negated) so it runs from the other starting tile.
void autonRightWP(Chassis& chassis) {
    chassis.setPose(48, -48, 0);         // start touching the roller
    spinRoller();                        // flip roller to our color

    chassis.moveToPoint(48, -24, 1500);  // drive to our alliance goal
    closeClamp();                        // clamp the alliance goal
    liftUp();                            // raise lift to scoring height
    scoreGoal();                         // score preload onto alliance goal

    chassis.moveToPoint(36, -18, 1200);  // drive to a pin/cup on the field
    intakePiece();                       // pick up the pin and cup
    scoreGoal();                         // score it onto the alliance goal
    liftDown();                          // lower lift
    openClamp();                         // release the alliance goal

    chassis.moveToPoint(12, -6, 1500);   // drive to the neutral goal
    liftUp();                            // raise lift to scoring height
    scoreGoal();                         // score final pin (goal already has 1 -> 2 there)
    liftDown();                          // lower lift, route done
}

// Skills: flip roller, preload -> neutral stake, 7x matchload <-> neutral
// stake, then 3 field pieces -> alliance stake, park midfield. ~118 pts max.
void autonSkills(Chassis& chassis) {
    chassis.setPose(-48, 48, 0);         // start touching the roller
    spinRoller();                        // flip roller to red

    chassis.moveToPoint(-6, 0, 2000);    // drive to the neutral stake
    liftUp();                            // raise lift to scoring height
    scoreGoal();                         // score preload onto neutral stake

    chassis.moveToPoint(-36, -18, 1500); // matchload 1/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload

    chassis.moveToPoint(-36, -18, 1500); // matchload 2/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload

    chassis.moveToPoint(-36, -18, 1500); // matchload 3/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload

    chassis.moveToPoint(-36, -18, 1500); // matchload 4/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload

    chassis.moveToPoint(-36, -18, 1500); // matchload 5/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload

    chassis.moveToPoint(-36, -18, 1500); // matchload 6/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload

    chassis.moveToPoint(-36, -18, 1500); // matchload 7/7: drive to station
    intakePiece();                       // grab matchload
    chassis.moveToPoint(-6, 0, 1500);    // drive back to neutral stake
    scoreGoal();                         // score matchload
    liftDown();                          // lower lift, matchloads done

    chassis.moveToPoint(-44, 25, 1200);  // drive to field pin/cup 1
    intakePiece();                       // pick it up
    chassis.moveToPoint(-37, -12, 1200); // drive to field pin/cup 2
    intakePiece();                       // pick it up
    chassis.moveToPoint(-35, -31, 1200); // drive to field pin/cup 3
    intakePiece();                       // pick it up

    chassis.moveToPoint(-66, 24, 1500);  // drive to our alliance stake
    closeClamp();                        // clamp the alliance stake
    liftUp();                            // raise lift to scoring height
    scoreGoal();                         // score all 3 pieces onto alliance stake
    liftDown();                          // lower lift
    openClamp();                         // release the alliance stake

    chassis.moveToPoint(0, 0, 1500);     // park at midfield, program ends
}
