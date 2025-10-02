#include "danielib/danielib.hpp"

void danielib::Drivetrain::turnToHeading(float heading, int timeout) {
    const int startTime = pros::millis();

    float power = 0;
    float currentHeading = odomSensors->imu->getHeading();

    float error = 0;

    angularPID->reset();
    while (pros::millis() < startTime + timeout) {
        currentHeading = odomSensors->imu->getHeading();
        error = reduce_to_180_180(heading - currentHeading);
        power = angularPID->update(error);

        leftMotors->move(power);
        rightMotors->move(-power);

        pros::delay(10);
    }
}