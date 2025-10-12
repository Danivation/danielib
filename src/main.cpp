#include "main.h"
#include "danielib/danielib.hpp"

pros::Controller master(CONTROLLER_MASTER);

pros::Rotation vertical_rotation(-20);
pros::Rotation horizontal_rotation(12);
pros::Imu imu(13);

pros::MotorGroup left_mg({17, 18, -19});
pros::MotorGroup right_mg({-14, -15, 16});

pros::Distance left_distance(10);
pros::Distance right_distance(5);

danielib::TrackerWheel vertical_tracker(vertical_rotation, 2.75, -1);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2.75, -7.75);
danielib::Inertial inertial(imu);

danielib::Sensors odom(vertical_tracker, horizontal_tracker, inertial);
danielib::MCL::Localization mcl({{left_distance, -5.5, 0, -90}, {right_distance, 3.5, -11, 180}});

danielib::PID linearPID(7.5, 0.1, 20, 1, 0.5, 100);
danielib::PID angularPID(2.3, 0.2, 13.7, 3, 1, 100);

danielib::Drivetrain chassis(left_mg, right_mg, odom, mcl, 11.5, 3.25, 450, linearPID, angularPID);

void screen_print() {
    //pros::lcd::initialize();
    master.clear();
    while (true) {
        // odom position
        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %.2f", reduce_to_0_360(chassis.getPose().theta));

        pros::lcd::print(4, "Distances: %d (left), %d (right)", left_distance.get_distance(), right_distance.get_distance());

        pros::delay(100);
    }
}

void initialize() {
    pros::lcd::initialize(); // initialze llemu
    pros::Task screen_task(screen_print);

    chassis.calibrate();
    chassis.startTracking();
    chassis.startLocalization(-24, -48, 0);
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {

}

void opcontrol() {

}