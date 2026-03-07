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
pros::MotorGroup left_mg({18, -15, -12}, pros::MotorGears::blue);
pros::MotorGroup right_mg({-19, 17, 14}, pros::MotorGears::blue);
pros::MotorGroup bottom({16, 13}, pros::MotorGears::blue);

// pros::Motor bottom_full(12, pros::MotorGears::blue);
// pros::Motor bottom_half(15, pros::MotorGears::rpm_200);
pros::Motor top(2, pros::MotorGears::rpm_200);

pros::Imu imu_1(3);
pros::Imu imu_2(22);
pros::Rotation vertical_rotation(-11);
pros::Rotation horizontal_rotation(4);

pros::Optical optical_top(22);
pros::Distance distance_left(10);
pros::Distance distance_front(1);
pros::Distance distance_right(20);

pros::adi::Pneumatics loader('H', false);
pros::adi::Pneumatics wing('G', false);
pros::adi::Pneumatics hood('F', false);
pros::adi::Pneumatics odom_lift('E', false);
pros::adi::Pneumatics mid_ramp('B', false);
pros::adi::Pneumatics intake_raise('A', false);     // actual piston is reversed, where extending the piston is low


/* ---------------------------------------------------------------------------------------------- */
/*                                         DANIELIB CONFIG                                        */
/* ---------------------------------------------------------------------------------------------- */

// + offset is right or front, - offset is left or back
danielib::TrackerWheel vertical_tracker(vertical_rotation, 2, -0.2);
danielib::TrackerWheel horizontal_tracker(horizontal_rotation, 2.744, -1.2);
danielib::Inertial inertial(imu_1, 360/(360-0.7));          // G TEAM IMU GOOD

danielib::Beam left_beam(-90, -4.375, 2.9, distance_left);
danielib::Beam right_beam(90, 4.375, 2.9, distance_right);
danielib::Beam front_beam(0, -4, 4.2, distance_front);

danielib::Localization mcl({left_beam, right_beam, front_beam});
danielib::Sensors sensors(vertical_tracker, horizontal_tracker, inertial, mcl);

danielib::PID linearPID(7.4, 0.09, 25, 0.75, 1, 70, 6);
danielib::PID angularPID(3.1, 0, 11, 0, 0, 0, 0);

danielib::PID mtpLinearPID(7.33, 0, 29, 0, 1, 100, 6);
danielib::PID mtpAngularPID(6, 0, 0, 0, 0, 0, 0);

danielib::PID swingAngularPID(6.2, 0.28, 61.8, 2, 0, 0, 0);

danielib::Drivetrain chassis(left_mg, right_mg, sensors, 10.8, 3.25, 450, linearPID, angularPID, mtpLinearPID, mtpAngularPID, swingAngularPID);


// convert vex field tiles to inches
constexpr double operator""_tiles(long double value) {
    return value * 23.622;
}
constexpr double operator""_tiles(unsigned long long value) {
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
    FILE* log = fopen("/usd/log.txt", "w");
    if (log) fclose(log);

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

    // chassis.driveForDistance(1_tiles, 1500, 100, 0);
    chassis.moveToPoint(0.5_tiles, 1_tiles, 1500, false, 100, 6);
    // chassis.moveToPoint(0.25_tiles, 2_tiles, 1500, false, 100, 0);
}

void opcontrol() {
    while (true) {
        pros::delay(10);
    }
}