#include "main.h"
#include "danielib/danielib.hpp"

void initialize() {
    pros::Rotation vertical_rotation(1);
    pros::Rotation horizontal_rotation(2);
    pros::Imu imu(3);

    pros::MotorGroup left_mg({3, 4, 5});
    pros::MotorGroup right_mg({6, 7, 8});

    danielib::tracker_wheel vertical_tracker(&vertical_rotation, 2.75, 0.5);
    danielib::tracker_wheel horizontal_tracker(&horizontal_rotation, 2.75, -1);
    danielib::inertial inertial(&imu);

    danielib::odometry odom(&vertical_tracker, &horizontal_tracker, &inertial);

    danielib::drivetrain drive(&left_mg, &right_mg, &odom, 11.5, 3.25, 450);
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {

}

void opcontrol() {

}