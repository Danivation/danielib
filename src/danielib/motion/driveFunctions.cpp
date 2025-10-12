#include "danielib/danielib.hpp"

void danielib::Drivetrain::driveForDistance(float distance, int timeout) {
    if (!isTracking()) return;
    const int startTime = pros::millis();
    ExitCondition linearExit(linearPID.exitRange, linearPID.exitTime);

    float power = 0;
    float currentDistance = odomSensors.verticalTracker.getPosition();
    float error = 0;

    linearPID.reset();
    linearExit.reset();
    while (pros::millis() < startTime + timeout && !linearExit.isDone()) {
        currentDistance = odomSensors.verticalTracker.getPosition();
        error = distance - currentDistance;
        power = linearPID.update(error);
        linearExit.update(error);

        leftMotors.move(power);
        rightMotors.move(power);

        pros::delay(10);
    }

    // stop motors
    leftMotors.move(0);
    rightMotors.move(0);
}