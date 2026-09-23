#pragma once
#include "pros/motors.hpp"
#include <string>
#include <vector>
#include <functional>


namespace gui {

//  name` shows on the button
// `run` is the function that gets called if this routine is selected.
struct AutonRoutine {
    std::string name;
    std::function<void()> run;
};

void runSelectedAuton();
void setAutonRoutines(std::vector<AutonRoutine> routines);

// Returns the index of the currently selected auton (-1 if none picked yet).
int getSelectedAutonIndex();
std::string getSelectedAutonName();

// A motor to display on the "Motors" tab, with a label.
struct WatchedMotor {
    std::string label;      // e.g. "L Front"
    pros::Motor* motor;     // pointer to your existing motor object
};

// Call once, before gui::init(), to tell the GUI which motors to monitor.
void setWatchedMotors(const std::vector<WatchedMotor>& motors);

void setLogoImage(const void* img_dsc);

// Supplies the Home tab with the robot's current field pose without making the
// GUI depend directly on the odometry implementation.
struct OdomDebugData {
    double x;
    double y;
    double heading;
};

using OdomDebugProvider = OdomDebugData (*)();
void setOdomDebugProvider(OdomDebugProvider provider);

void init();

}  // namespace gui
