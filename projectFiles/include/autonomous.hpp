#pragma once
#include "81yOdom/chassis/Chassis.hpp"

void autonLeftWP(Chassis& chassis);
void autonRightWP(Chassis& chassis);
void autonSkills(Chassis& chassis);

// Mechanism stubs - no roller/intake/lift/clamp motors exist yet. Each just
// prints so timing is visible on the bench; swap in real motor code later.
void spinRoller();   // spin drivetrain/roller against the field roller until it flips
void closeClamp();   // close the goal clamp around a mobile goal
void openClamp();    // release the goal clamp
void liftUp();        // raise the lift to scoring height
void liftDown();       // lower the lift back down
void intakePiece();  // run the intake to grab a pin/cup or matchload
void scoreGoal();    // dump the held piece(s) onto whichever goal is in front
