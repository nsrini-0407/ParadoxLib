#include "main.h"
#include "autonomous.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

// WP route: flip roller, score preload + 1 field piece on our alliance goal,
// score 1 more piece on the neutral goal (already holding 1) -> 4 pins across
// 2 goals, 40 pts, qualifies for the Autonomous Win Point on our side alone.
void leftAuton(Chassis& chassis) {

}

// Same route mirrored (x negated) so it runs from the other starting tile.
void rightAuton(Chassis& chassis) {
                          // lower lift, route done
}

// Skills: flip roller, preload -> neutral stake, 7x matchload <-> neutral
// stake, then 3 field pieces -> alliance stake, park midfield. ~118 pts max.
void skillsAuton(Chassis& chassis) {
    
}
