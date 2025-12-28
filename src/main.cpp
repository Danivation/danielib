#include "main.h"
#include "danielib/danielib.hpp"

/**
 * TODO:
 * (!) distance resets
 * (*) move mcl distance sensors into the Sensors class to unify things
 * 
 * (*) make both mtp algorithms use exit conditions
 * (*) add some sort of Timer class to all movements rather than timeouts
 * (*) new PIDs for heading correction, and a new movement for that
 * (*) improve exit conditions
 * (*) something like a Motion class to assume all motions
**/


/* ---------------------------------------------------------------------------------------------- */
/*                                          DEVICE PORTS                                          */
/* ---------------------------------------------------------------------------------------------- */

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1, -2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({-8, 9, -10}, pros::MotorGearset::blue);
pros::Imu imu_1(4);
pros::Motor intake(5);
pros::Motor hood(-6);
pros::Rotation horizontal_rotation(7);
pros::Rotation vertical_rotation(11);

pros::Distance distance_left(12);
pros::Distance distance_right(17);
pros::Distance distance_front(20);

pros::adi::Pneumatics double_park('A', false);
pros::adi::Pneumatics descore_mid('B', false);
pros::adi::Pneumatics trapdoor('D', true, true);
pros::adi::Pneumatics wing('E', false);
pros::adi::Pneumatics loader('F', false);

pros::Optical optical_top(22);  // unused


/* ---------------------------------------------------------------------------------------------- */
/*                                         DANIELIB CONFIG                                        */
/* ---------------------------------------------------------------------------------------------- */

danielib::TrackerWheel vertical_tracker(vertical_rotation, 2, 0);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2, 0.84);
danielib::Inertial inertial(imu_1, 1.004);     // new imu

danielib::Beam left_beam(-90, -5, -3.4, distance_left);
danielib::Beam right_beam(90, 5, -3.4, distance_right);
danielib::Beam front_beam(0, 5.1, -3.2, distance_front);

danielib::Localization mcl({left_beam, right_beam, front_beam});
danielib::Sensors sensors(vertical_tracker, horizontal_tracker, inertial, mcl);

danielib::PID linearPID(7.5, 0.1, 22.5, 1, 0.5, 100);
danielib::PID angularPID(2.3, 0.2, 13.7, 3, 1, 100);
danielib::PID mtpLinearPID(7.2, 0.05, 25, 0, 0.75, 300);
danielib::PID mtpAngularPID(3.13, 0.2, 13.7, 0, 1, 100);

danielib::Drivetrain chassis(left_mg, right_mg, sensors, 11.5, 3.25, 450, linearPID, angularPID, mtpLinearPID, mtpAngularPID);




// convert vex field tiles to inches
constexpr double operator"" _tiles(long double value) {
    return value * 23.622;
}
constexpr double operator"" _tiles(unsigned long long value) {
    return static_cast<double>(value) * 23.622;
}


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

    autonomous();

    while (true) {
        pros::delay(10);
    }
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {
    chassis.setPose(0.75, -47, 270);

    // steal preload and push
    chassis.driveForDistance(8, 400);

    /* ---------------------------------------------------------------------------------------------- */
    /*                                      RIGHT LOADER + SCORE                                      */
    /* ---------------------------------------------------------------------------------------------- */

    // move back to loader
    chassis.moveToPoint(1.78_tiles, -1.9_tiles, 2000, true);
}

void opcontrol() {
    while (true) {
        pros::delay(10);
    }
}