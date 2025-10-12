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

Sensors::Sensors(TrackerWheel& verticalTracker, TrackerWheel& horizontalTracker, Inertial& imu) :
    verticalTracker(verticalTracker),
    horizontalTracker(horizontalTracker),
    imu(imu)
{}

Drivetrain::Drivetrain(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, Sensors& odomSensors, danielib::Localization& localization, float trackWidth, float wheelSize, float wheelRPM, PID& linearPID, PID& angularPID) :
    leftMotors(leftMotors),
    rightMotors(rightMotors),
    odomSensors(odomSensors),
    localization(localization),
    trackWidth(trackWidth),
    wheelSize(wheelSize),
    wheelRPM(wheelRPM),
    linearPID(linearPID),
    angularPID(angularPID)
{}
} // namespace danielib