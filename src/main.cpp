#include "main.h"
#include "danielib/danielib.hpp"

/**
 * TODO:
 * (1) move to point (not pose)
 * (2) make both mtp algorithms use exit conditions
 * (3) add some sort of Timer class to all movements rather than timeouts
 * (4) new PIDs for heading correction, and a new movement for that
 * (5) improve exit conditions
 * (6) something like a Motion class to assume all motions
**/

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-14, 15, 16}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({17, -18, -19}, pros::MotorGearset::blue);
pros::Imu imu_1(4);
pros::Rotation vertical_rotation(6);
pros::Rotation horizontal_rotation(5);
pros::Optical optical_top(9);
pros::Motor intake(10);
pros::Motor hood(-2);
pros::adi::Pneumatics lift('B', true, true);
pros::adi::Pneumatics loader('H', false, false);
pros::adi::Pneumatics descore_right('A', false);
pros::adi::Pneumatics descore_left('C', false);  // unused, both buttons trigger descore right

pros::Optical optical_middle(22);   // unused

danielib::TrackerWheel vertical_tracker(vertical_rotation, 2, 0);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2, 1.8);
//danielib::Inertial inertial(imu_1, 1.0033);   // old imu
danielib::Inertial inertial(imu_1, 1.0029);     // new imu

danielib::Sensors odom(vertical_tracker, horizontal_tracker, inertial);
danielib::MCL::Localization mcl({});

danielib::PID linearPID(7.5, 0.1, 22.5, 1, 0.5, 100);
danielib::PID angularPID(2.3, 0.2, 13.7, 3, 1, 100);

danielib::PID mtpLinearPID(7.5, 0.1, 22.5, 1, 0.5, 100);
danielib::PID mtpAngularPID(3.13, 0.2, 13.7, 3, 1, 100);

// add pids for odom angular control specifically, and for heading hold during drives

danielib::Drivetrain chassis(left_mg, right_mg, odom, mcl, 11.5, 3.25, 450, linearPID, angularPID, mtpLinearPID, mtpAngularPID);

void screen_print() {
    while (true) {
        const auto pose = chassis.getPose();
        pros::lcd::print(0, "X: %.2f", pose.x);
        pros::lcd::print(1, "Y: %.2f", pose.y);
        pros::lcd::print(2, "Theta: %.2f", d_reduce_to_0_360(pose.theta));

        pros::delay(50);
    }
}

void controller_print() {
    master.clear();
    while (true) {
        const auto pose = chassis.getPose();
        master.print(0, 0, "(%.2f, %.2f, %.2f)      ", pose.x, pose.y, d_reduce_to_0_360(pose.theta));
        pros::delay(300);
    }
}

void print_to_displays() {
    pros::Task screenTask(screen_print);
    pros::Task controllerTask(controller_print);
}

void initialize() {
    pros::lcd::initialize(); // initialze llemu
    print_to_displays();

    left_mg.set_brake_mode_all(pros::MotorBrake::brake);
    right_mg.set_brake_mode_all(pros::MotorBrake::brake);

    chassis.calibrate();
    chassis.startTracking();
    chassis.setPose(0, 0, 0);
    pros::delay(500);

    chassis.moveToPoint(24, 24, 10000, false);
    chassis.turnToHeading(90, 1000);

    while (true) {
        pros::delay(10);
    }
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {

}

void opcontrol() {
    while (true) {
        pros::delay(10);
    }
}