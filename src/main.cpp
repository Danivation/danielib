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
pros::MotorGroup left_mg({18, -16, -11}, pros::MotorGears::blue);
pros::MotorGroup right_mg({-19, 17, 13}, pros::MotorGears::blue);
pros::MotorGroup bottom({12, 15}, pros::MotorGears::blue);
pros::Motor top(4, pros::MotorGears::rpm_200);

pros::Imu imu_1(2);
pros::Imu imu_2(22);
pros::Rotation vertical_rotation(-14);
pros::Rotation horizontal_rotation(10);

pros::Optical optical_top(22);
pros::Distance distance_left(9);
pros::Distance distance_front(3);
pros::Distance distance_right(20);

pros::adi::Pneumatics loader('H', false);
pros::adi::Pneumatics wing('G', false);
pros::adi::Pneumatics hood('E', false);
pros::adi::Pneumatics mid_ramp('A', false);
pros::adi::Pneumatics intake_raise('B', false);     // actual piston is reversed, where extending the piston is low


/* ---------------------------------------------------------------------------------------------- */
/*                                         DANIELIB CONFIG                                        */
/* ---------------------------------------------------------------------------------------------- */

danielib::TrackerWheel vertical_tracker(vertical_rotation, 2, 0.1);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2.744, -1.35);
danielib::Inertial inertial(imu_1, 1.0);     // G TEAM IMU

danielib::Beam left_beam(-90, -4.9, -3.4, distance_left);
danielib::Beam right_beam(90, 4.9, -3.4, distance_right);
danielib::Beam front_beam(0, 5.1, -3.2, distance_front);

danielib::Localization mcl({left_beam, right_beam, front_beam});
danielib::Sensors sensors(vertical_tracker, horizontal_tracker, inertial, mcl);

danielib::PID linearPID(7.97, 0.09, 50.12, 0.75, 1, 70, 2);
danielib::PID angularPID(3.1, 0.14, 30.9, 1, 1, 50);

danielib::PID mtpLinearPID(7.65, 0.09, 51.5, 0.75, 1, 90, 2);
danielib::PID mtpAngularPID(2.22, 0, 17.5, 0, 0, 0);

danielib::PID swingAngularPID(6.2, 0.28, 61.8, 2, 0, 0);

danielib::Drivetrain chassis(left_mg, right_mg, sensors, 10.8, 3.25, 450, linearPID, angularPID, mtpLinearPID, mtpAngularPID, swingAngularPID);






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
    imu_1.set_data_rate(5);
    horizontal_rotation.set_data_rate(5);
    vertical_rotation.set_data_rate(5);

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

    chassis.moveToPoint(1_tiles, 1_tiles, 2000, false, 100);
    wing.extend();
}

void opcontrol() {
    while (true) {
        pros::delay(10);
    }
}