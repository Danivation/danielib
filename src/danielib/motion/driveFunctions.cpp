#include "danielib/danielib.hpp"

void danielib::Drivetrain::driveForDistance(float distance, int timeout) {
    float power = 0;
    float currentDistance = odomSensors->verticalTracker->getPosition();

    float error = 0;

    linearPID->reset();
    while (true) {
        currentDistance = odomSensors->verticalTracker->getPosition();
        error = distance - currentDistance;
        power = linearPID->update(error);

        leftMotors->move(power);
        rightMotors->move(power);
    }
}