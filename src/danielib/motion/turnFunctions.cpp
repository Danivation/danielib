#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"

void danielib::Drivetrain::turnToHeading(float heading, int timeout, float maxSpeed) {
    if (!isTracking()) return;
    const int startTime = pros::millis();
    ExitCondition angularExit(angularPID.exitRange, angularPID.exitTime);

    float power = 0;
    float currentHeading = odomSensors.imu.getHeading();
    float error = 0;

    angularPID.reset();
    angularExit.reset();
    while (pros::millis() < startTime + timeout && !angularExit.isDone()) {
        currentHeading = odomSensors.imu.getHeading();
        error = d_reduce_to_180_180(heading - currentHeading);
        power = angularPID.update(error);
        angularExit.update(error);

        power = std::clamp(power, -maxSpeed, maxSpeed);
        leftMotors.move(power);
        rightMotors.move(-power);

        pros::delay(10);
    }

    // stop motors
    leftMotors.move(0);
    rightMotors.move(0);
}

void danielib::Drivetrain::turnToPoint(float x, float y, int timeout, float maxSpeed) {
    if (!isTracking()) return;
    float angle = d_toDegrees(currentPose.angle({x, y, currentPose.theta}));
    turnToHeading(angle, timeout, maxSpeed);
}