#include "danielib/danielib.hpp"

void danielib::Drivetrain::turnToHeading(float heading, int timeout) {
    float power = 0;
    float currentHeading = odomSensors->imu->getHeading();

    float error = 0;

    angularPID->reset();
    while (true) {
        currentHeading = odomSensors->imu->getHeading();
        error = reduce_to_180_180(heading - currentHeading);
        power = angularPID->update(error);

        leftMotors->move(power);
        rightMotors->move(-power);
    }
}