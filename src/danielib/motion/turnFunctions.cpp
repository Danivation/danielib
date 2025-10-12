#include "danielib/danielib.hpp"

void danielib::Drivetrain::turnToHeading(float heading, int timeout) {
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
        error = reduce_to_180_180(heading - currentHeading);
        power = angularPID.update(error);
        angularExit.update(error);

        leftMotors.move(power);
        rightMotors.move(-power);

        pros::delay(10);
    }

    // stop motors
    leftMotors.move(0);
    rightMotors.move(0);
}