#include "danielib/danielib.hpp"
#include "danielib/exit.hpp"
#include <cmath>

namespace danielib {
ExitCondition::ExitCondition(const float exitRange, const int exitTime) :
    exitRange(exitRange),
    exitTime(exitTime)
{}

bool ExitCondition::isDone() {
    return done;
}

// input = pid error
bool ExitCondition::update(const float input) {

    // get the current brain time
    const int currentTime = pros::millis();

    // if the error is greater than the exit range, set startTime to -1
    if (std::abs(input) > exitRange) startTime = -1;

    // "else if" runs only if the first statement was false, so:
    // if the error is WITHIN the exit range but startTime is still -1
    // (meaning this is the first time the error is within exitRange)
    // set startTime to currentTime
    else if (startTime == -1) startTime = currentTime;

    // if both of the above do not run (meaning the error has been within exitRange for at least 1 cycle)
    // check if current time is more than the start time plus whatever timeout
    // if it is, that means it has been inside the exit range for more than the exit time
    // so exit
    else if (currentTime >= startTime + exitTime) done = true;
    return done;
}

void ExitCondition::reset() {
    startTime = -1;
    done = false;
}
}