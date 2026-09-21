#pragma once
#include "pros/rtos.hpp"
#include <cstdint>

class Timer {
    public:
        explicit Timer(double timeoutMs) 
        : startTime(pros::millis()), timeout(timeoutMs) {}

        bool isDone() const { return getElapsed() >= timeout; }

        // ms since construction / last reset
        double getElapsed() const { return (double)(pros::millis() - startTime); }

        void reset() { startTime = pros::millis(); }

    private:
        uint32_t startTime;
        double timeout;
};
