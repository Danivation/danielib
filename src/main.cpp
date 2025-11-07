#include "main.h"
#include "danielib/danielib.hpp"

/**
 * TODO:
 * (1) add timer to all movements and add an async parameter so things can run without blocking, add waitUntilDone()
 * (2) make all movements obey a global stop, add something like stopAllMovements()
**/

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-12, 14, 15}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({-17, -18, 20}, pros::MotorGearset::blue);
pros::Rotation vertical_rotation(8);
pros::Rotation horizontal_rotation(-16);
pros::Imu imu_1(9);
pros::Optical optical_middle(13);   // unused
pros::Optical optical_top(13);      // unused
pros::Motor intake(10);
pros::Motor hood(-2);
pros::adi::Pneumatics descore_left('e', false);  // unused, both buttons trigger descore right
pros::adi::Pneumatics descore_right('f', false);
pros::adi::Pneumatics lift('g', false);
pros::adi::Pneumatics loader('h', false);

danielib::TrackerWheel vertical_tracker(vertical_rotation, 2, 0);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2, 2.5);
danielib::Inertial inertial(imu_1, 1.00831946755);

danielib::Sensors odom(vertical_tracker, horizontal_tracker, inertial);
//danielib::MCL::Localization mcl({{-90, -5.5, 0, left_distance}, {180, 3.5, -11, right_distance}});
danielib::MCL::Localization mcl({});

danielib::PID linearPID(7.5, 0.1, 22.5, 1, 0.5, 100);
danielib::PID angularPID(2.3, 0.2, 13.7, 3, 1, 100);

danielib::Drivetrain chassis(left_mg, right_mg, odom, mcl, 11.5, 3.25, 450, linearPID, angularPID);

void screen_print() {
    //pros::lcd::initialize();
    master.clear();
    while (true) {
        // odom position
        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %.2f", d_reduce_to_0_360(chassis.getPose().theta));

        //pros::lcd::print(4, "Distances: %d (left), %d (right)", left_distance.get_distance(), right_distance.get_distance());

        pros::delay(100);
    }
}

void initialize() {
    pros::lcd::initialize(); // initialze llemu
    pros::Task screen_task(screen_print);

    chassis.calibrate();
    chassis.startTracking();
    chassis.setPose(0, 0, 0);
    //chassis.startLocalization(-24, -48, 0);

    chassis.moveToPose(24, -24, 270, 1000, true);
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {

}

void opcontrol() {

}