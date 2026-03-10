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
    FILE* log_linearOut = fopen("/usd/log_linearOut.txt", "a");
    FILE* log_distance = fopen("/usd/log_distance.txt", "a");
    FILE* log_pose = fopen("/usd/log_pose.txt", "a");

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

    std::uint32_t time = pros::millis();
    while (pros::millis() < startTime + timeout && !linearExit.isDone() && movementsEnabled && currentMovementEnabled) {
        currentDistance = odomSensors.verticalTracker.getPosition() - startPosition;
        error = distance - currentDistance;

        // exit if within exit range
        if (std::abs(error) < earlyExitRange) {
            motionChained = true;
            break;
        }

        power = linearPID.update(error);
        linearExit.update(error);

        // calculate power
        power = std::clamp(power, -maxSpeed, maxSpeed);
        if (linearMaxSlew != 0 && std::abs(error) > 8) power = d_slew(power, prevLinearOut, linearMaxSlew);
        prevLinearOut = power;
        
        // move motors
        leftMotors.move(power);
        rightMotors.move(power);

        if (log_linearOut) fprintf(log_linearOut, "(%d,%.2f),", pros::millis() - startTime, power);
        if (log_distance) fprintf(log_distance, "(%d,%.2f),", pros::millis() - startTime, error);
        auto pose = getPose();
        if (log_pose) fprintf(log_pose, "(%.3f,%.3f),", pose.x, pose.y);

        pros::Task::delay_until(&time, 10);
    }

    if (!motionChained) {
        prevLinearOut = 0;
        prevAngularOut = 0;
    }

    if (log_linearOut) fclose(log_linearOut);
    if (log_distance) fclose(log_distance);
    if (log_pose) fclose(log_pose);

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}