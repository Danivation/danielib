#include "main.h"
#include "danielib/danielib.hpp"

pros::Rotation vertical_rotation(-20);
pros::Rotation horizontal_rotation(12);
pros::Imu imu(13);

pros::MotorGroup left_mg({17, 18, -19});
pros::MotorGroup right_mg({-14, -15, 16});

danielib::TrackerWheel vertical_tracker(&vertical_rotation, 2.75, -1);
danielib::TrackerWheel horizontal_tracker(&horizontal_rotation, 2.75, -7.75);
danielib::Inertial inertial(&imu);

danielib::Sensors odom(&vertical_tracker, &horizontal_tracker, &inertial);

/* danielib::Controllers controllers({
    {3, 3, 4}
}); */

danielib::Drivetrain drive(&left_mg, &right_mg, &odom, /* &controllers, */ 11.5, 3.25, 450);

void screen_print() {
    //pros::lcd::initialize();
    while (true) {
        // odom position
        pros::lcd::print(0, "X: %f", drive.getPose().x);
        pros::lcd::print(1, "Y: %f", drive.getPose().y);
        pros::lcd::print(2, "Theta: %.2f", reduce_0_to_360(drive.getPose().theta));

        // delay to save resources
        pros::delay(100);
    }
}

void initialize() {
    pros::lcd::initialize(); // initialze llemu
    drive.calibrate();
    imu.set_rotation(0);
    horizontal_rotation.reset_position();
    vertical_rotation.reset_position();
    drive.setPose(0, 0, 0);

    pros::Task screen_task(screen_print);
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {

}

void opcontrol() {

}