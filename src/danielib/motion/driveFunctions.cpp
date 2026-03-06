#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"

void danielib::Drivetrain::driveForDistance(float distance, int timeout, float maxSpeed, float earlyExitRange) {
    if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { driveForDistance(distance, timeout, maxSpeed, earlyExitRange); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    const int startTime = pros::millis();
    ExitCondition linearExit(linearPID.exitRange, linearPID.exitTime);
    float linearMaxSlew = linearPID.slew;

    float power = 0;
    float currentDistance;
    float startPosition = odomSensors.verticalTracker.getPosition();
    float error = 0;
    bool motionChained = false;

    linearPID.reset();
    linearExit.reset();
    while (pros::millis() < startTime + timeout && !linearExit.isDone() && movementsEnabled && currentMovementEnabled) {
        currentDistance = odomSensors.verticalTracker.getPosition() - startPosition;
        error = distance - currentDistance;

        // exit if within exit range
        if (fabs(error) < earlyExitRange) {
            motionChained = true;
            break;
        }

        power = linearPID.update(error);
        linearExit.update(error);

        // calculate power
        power = std::clamp(power, -maxSpeed, maxSpeed);
        if (linearMaxSlew != 0) power = d_slew(power, prevLinearOut, linearMaxSlew);
        prevLinearOut = power;

        // move motors
        leftMotors.move(power);
        rightMotors.move(power);

        pros::delay(5);
    }

    if (!motionChained) {
        prevLinearOut = 0;
        prevAngularOut = 0;
    }

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}