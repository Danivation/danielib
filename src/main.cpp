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
pros::MotorGroup left_mg({2, -3, 4}, pros::MotorGears::blue);
pros::MotorGroup right_mg({-7, 8, -9}, pros::MotorGears::blue);
pros::Motor intake(1, pros::MotorGears::blue);
pros::Motor hood(-10, pros::MotorGears::blue);

pros::Imu imu_1(12);
pros::Imu imu_2(15);
pros::Rotation vertical_rotation(5);
pros::Rotation horizontal_rotation(6);

pros::Optical optical_top(13);
pros::Distance distance_left(14);
pros::Distance distance_front(19);
pros::Distance distance_right(20);

pros::adi::Pneumatics loader('A', false);
pros::adi::Pneumatics descore_mid('B', false);
pros::adi::Pneumatics wing('C', false);
pros::adi::Pneumatics trapdoor('D', true, true);
pros::adi::Pneumatics compressor('E', false);



/* ---------------------------------------------------------------------------------------------- */
/*                                         DANIELIB CONFIG                                        */
/* ---------------------------------------------------------------------------------------------- */

danielib::TrackerWheel vertical_tracker(vertical_rotation, 2, 0);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2, 1.57);
danielib::Inertial inertial(imu_1, 1.004);     // new imu

danielib::Beam left_beam(-90, -4.9, -3.4, distance_left);
danielib::Beam right_beam(90, 4.9, -3.4, distance_right);
danielib::Beam front_beam(0, 5.1, -3.2, distance_front);

danielib::Localization mcl({left_beam, right_beam, front_beam});
danielib::Sensors sensors(vertical_tracker, horizontal_tracker, inertial, mcl);

danielib::PID linearPID(8.2, 0.08, 58, 0.5, 1.5, 70, 0);
danielib::PID angularPID(2.3, 0.2, 13.7, 3, 2, 70, 0);

danielib::PID mtpLinearPID(8.17, 0.05, 54.5, 0.5, 1.5, 80);
danielib::PID mtpAngularPID(2.45, 0, 15, 0, 0, 0);

danielib::PID swingAngularPID(3.4, 0.2, 16.5, 3, 2, 100, 0);

danielib::Drivetrain chassis(left_mg, right_mg, sensors, 11.5, 3.25, 450, linearPID, angularPID, mtpLinearPID, mtpAngularPID, swingAngularPID);



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
    master.clear();
    print_to_displays();

    left_mg.set_brake_mode_all(pros::MotorBrake::brake);
    right_mg.set_brake_mode_all(pros::MotorBrake::brake);

    chassis.calibrate();
    chassis.startTracking();
    pros::delay(5);

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
    chassis.setPose(0, 0, 0);

    chassis.driveForDistance(24, 2000, 100, 0);
    wing.extend();
}

void opcontrol() {
    while (true) {
        pros::delay(10);
    }
}