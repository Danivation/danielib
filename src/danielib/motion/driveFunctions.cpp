#include "danielib/danielib.hpp"

void danielib::Drivetrain::driveForDistance(float distance, int timeout, float maxSpeed) {
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

        power = std::clamp(power, -maxSpeed, maxSpeed);
        leftMotors.move(power);
        rightMotors.move(power);

        pros::delay(10);
    }

    // stop motors
    leftMotors.move(0);
    rightMotors.move(0);
}