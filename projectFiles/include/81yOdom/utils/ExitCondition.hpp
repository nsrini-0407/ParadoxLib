#pragma once
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>

// Fires once |error| has stayed inside `range` continuously for `timeMs`.
//
// Motions run two of these per controller in parallel - a tight range with a
// short dwell and a looser range with a longer dwell - and exit when EITHER
// fires. The tight one gives precision on a clean approach; the loose one
// guarantees a motion that stalls 1.5" short (bumped a game piece, carpet
// seam, low battery) still exits in well under a second instead of burning its
// whole timeout while the next motion in the auton waits.
class ExitCondition {
    public:
        // range <= 0 disables this condition (update() never returns true).
        ExitCondition(double range, double timeMs)
            : range(range), timeMs(timeMs) {}

        bool update(double error) {
            if (range <= 0.0) return false;
            if (std::fabs(error) < range) {
                if (!inRange) {
                    inRange = true;
                    enteredAt = pros::millis();
                }
                if (pros::millis() - enteredAt >= (uint32_t)timeMs) done = true;
            } else {
                inRange = false;
            }
            return done;
        }

        bool getExit() const { return done; }

        void reset() {
            inRange = false;
            done = false;
            enteredAt = 0;
        }

    private:
        double range;
        double timeMs;
        uint32_t enteredAt = 0;
        bool inRange = false;
        bool done = false;
};
