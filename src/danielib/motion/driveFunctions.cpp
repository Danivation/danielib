#include "danielib/danielib.hpp"

void danielib::Drivetrain::driveForDistance(float distance, int timeout) {
    const int startTime = pros::millis();

    float power = 0;
    float currentDistance = odomSensors->verticalTracker->getPosition();

    float error = 0;

    linearPID->reset();
    // todo: exit conditions
    while (pros::millis() < startTime + timeout) {
        currentDistance = odomSensors->verticalTracker->getPosition();
        error = distance - currentDistance;
        power = linearPID->update(error);

        leftMotors->move(power);
        rightMotors->move(power);

        pros::delay(10);
    }
}