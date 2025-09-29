#include "danielib/danielib.hpp"
#include <cmath>

namespace danielib {
ExitCondition::ExitCondition(const float exitRange, const float exitTime) :
    exitRange(exitRange),
    exitTime(exitTime)
{}

bool ExitCondition::isDone() {
    return done;
}

bool ExitCondition::update(const float input) {
    const int curTime = pros::millis();
    if (std::fabs(input) > exitRange) startTime = -1;
    else if (startTime == -1) startTime = curTime;
    else if (curTime >= startTime + exitTime) done = true;
    return done;
}

void ExitCondition::reset() {
    startTime = -1;
    done = false;
}
}