#include "danielib/danielib.hpp"

namespace danielib {

TrackerWheel::TrackerWheel(pros::Rotation& sensor, float wheelDiameter, float offset) :
    sensor(sensor),
    wheelDiameter(wheelDiameter),
    offset(offset)
{}

Inertial::Inertial(pros::Imu& sensor1, float scale1, pros::Imu* sensor2, float scale2) :
    sensor1(sensor1),
    scale1(scale1),
    sensor2(sensor2),
    scale2(scale2)
{}

Sensors::Sensors(TrackerWheel& verticalTracker, TrackerWheel& horizontalTracker, Inertial& imu, Localization& localization) :
    verticalTracker(verticalTracker),
    horizontalTracker(horizontalTracker),
    imu(imu),
    localization(localization)
{}

Drivetrain::Drivetrain(
    pros::MotorGroup& leftMotors, 
    pros::MotorGroup& rightMotors, 
    Sensors& odomSensors, 
    float trackWidth, 
    float wheelSize, 
    float wheelRPM, 
    PID& linearPID, 
    PID& angularPID, 
    PID& mtpLinearPID, 
    PID& mtpAngularPID,
    PID& swingAngularPID
) :
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    odomSensors(odomSensors),
    trackWidth(trackWidth),
    wheelSize(wheelSize),
    wheelRPM(wheelRPM),
    linearPID(linearPID),
    angularPID(angularPID),
    mtpLinearPID(mtpLinearPID),
    mtpAngularPID(mtpAngularPID),
    swingAngularPID(swingAngularPID)
{
    FILE* log_linearOut = fopen("/usd/log_linearOut.txt", "w");
    FILE* log_angularOut = fopen("/usd/log_angularOut.txt", "w");
    FILE* log_distance = fopen("/usd/log_distance.txt", "w");
    FILE* log_angularError = fopen("/usd/log_angularError.txt", "w");
    FILE* log_pose = fopen("/usd/log_pose.txt", "w");
    FILE* log_horiz = fopen("/usd/log_horiz.txt", "w");
    FILE* log_vert = fopen("/usd/log_vert.txt", "w");
    if (log_linearOut) fclose(log_linearOut);
    if (log_angularOut) fclose(log_angularOut);
    if (log_distance) fclose(log_distance);
    if (log_angularError) fclose(log_angularError);
    if (log_pose) fclose(log_pose);
    if (log_horiz) fclose(log_horiz);
    if (log_vert) fclose(log_vert);
}

void danielib::Drivetrain::stopAllMovements() {
    movementsEnabled = false;
    pros::delay(10);
}

void danielib::Drivetrain::stopMovement() {
    currentMovementEnabled = false;
    prevLinearOut = 0;
    prevAngularOut = 0;
    pros::delay(10);
}

void danielib::Drivetrain::waitUntilDone() {
    // wait until mutex is takeable and immediately give it up
    motionMutex.take();
    motionMutex.give();
}

// void danielib::Drivetrain::setSpeed() {
// }

} // namespace danielib