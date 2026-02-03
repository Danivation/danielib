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
{}

void danielib::Drivetrain::stopAllMovements() {
    movementsEnabled = false;
    pros::delay(5);
}

void danielib::Drivetrain::stopMovement() {
    currentMovementEnabled = false;
    pros::delay(5);
}

void danielib::Drivetrain::waitUntilDone() {
    // wait until mutex is takeable and immediately give it up
    motionMutex.take();
    motionMutex.give();
}

void danielib::Drivetrain::setSpeed() {
    
}

} // namespace danielib