#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"

void danielib::Drivetrain::turnToHeading(float heading, int timeout, float maxSpeed, bool slewEnabled) {
    // if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { turnToHeading(heading, timeout, maxSpeed, slewEnabled); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    FILE* log_angularOut = fopen("/usd/log_angularOut.txt", "a");
    FILE* log_angularError = fopen("/usd/log_angularError.txt", "a");
    FILE* log_pose = fopen("/usd/log_pose.txt", "a");

    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    const int startTime = pros::millis();
    ExitCondition angularExit(angularPID.exitRange, angularPID.exitTime);
    float angularMaxSlew = angularPID.slew;
    if (!slewEnabled) angularMaxSlew = 0;

    float power = 0;
    float currentHeading = odomSensors.imu.getHeading();
    float error = 0;

    angularPID.reset();
    angularExit.reset();

    std::uint32_t time = pros::millis();
    while (pros::millis() < startTime + timeout && !angularExit.isDone() && movementsEnabled && currentMovementEnabled) {
        currentHeading = odomSensors.imu.getHeading();
        error = d_reduce_to_180_180(heading - currentHeading);
        power = angularPID.update(error);
        angularExit.update(error);

        // calculate power
        power = std::clamp(power, -maxSpeed, maxSpeed);
        if (angularMaxSlew != 0) power = d_slew(power, prevAngularOut, angularMaxSlew);
        prevAngularOut = power;

        // move motors
        leftMotors.move(power);
        rightMotors.move(-power);

        if (log_angularOut) fprintf(log_angularOut, "(%d,%.2f),", pros::millis(), power);
        if (log_angularError) fprintf(log_angularError, "(%d,%.2f),", pros::millis(), error);
        auto pose = getPose();
        if (log_pose) fprintf(log_pose, "(%.3f,%.3f),", pose.x, pose.y);

        pros::Task::delay_until(&time, 10);
    }

    prevLinearOut = 0;
    prevAngularOut = 0;

    if (log_angularOut) fclose(log_angularOut);
    if (log_angularError) fclose(log_angularError);
    if (log_pose) fclose(log_pose);

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}

void danielib::Drivetrain::turnToPoint(float x, float y, int timeout, bool reverse, float maxSpeed, bool slewEnabled) {
    // if (!isTracking()) return;
    float angle = d_toDegrees(currentPose.angle({x, y, currentPose.theta}));
    if (reverse) angle = d_reduce_to_0_360(angle + 180);
    turnToHeading(angle, timeout, maxSpeed, slewEnabled);
}

void danielib::Drivetrain::swingToHeading(float heading, SwingSide side, int timeout, float maxSpeed, bool slewEnabled) {
    // if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { swingToHeading(heading, side, timeout, maxSpeed, slewEnabled); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    const int startTime = pros::millis();
    ExitCondition angularExit(swingAngularPID.exitRange, swingAngularPID.exitTime);
    float angularMaxSlew = swingAngularPID.slew;
    if (!slewEnabled) angularMaxSlew = 0;

    float power = 0;
    float currentHeading = odomSensors.imu.getHeading();
    float error = 0;

    swingAngularPID.reset();
    angularExit.reset();

    std::uint32_t time = pros::millis();
    while (pros::millis() < startTime + timeout && !angularExit.isDone() && movementsEnabled && currentMovementEnabled) {
        currentHeading = odomSensors.imu.getHeading();
        error = d_reduce_to_180_180(heading - currentHeading);
        power = swingAngularPID.update(error);
        angularExit.update(error);

        // clamp and slew output
        power = std::clamp(power, -maxSpeed, maxSpeed);
        if (angularMaxSlew != 0) power = d_slew(power, prevAngularOut, angularMaxSlew);
        prevAngularOut = power;

        // move motors
        if (side == SwingSide::LEFT) {
            leftMotors.move(power);
            rightMotors.brake();
        } else if (side == SwingSide::RIGHT) {
            leftMotors.brake();
            rightMotors.move(-power);
        }

        pros::Task::delay_until(&time, 10);
    }

    prevLinearOut = 0;
    prevAngularOut = 0;

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}