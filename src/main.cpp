#include "main.h"
#include "danielib/danielib.hpp"

/* ---------------------------------------------------------------------------------------------- */
/*                                          DEVICE PORTS                                          */
/* ---------------------------------------------------------------------------------------------- */

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({2, -3, 4}, pros::MotorGears::blue);
pros::MotorGroup right_mg({-7, 8, -9}, pros::MotorGears::blue);
pros::Motor intake(1, pros::MotorGears::blue);
pros::Motor hood(-10, pros::MotorGears::blue);
pros::Imu imu_1(11);
pros::Rotation vertical_rotation(5);
pros::Rotation horizontal_rotation(6);

pros::Distance distance_left(12);
pros::Distance distance_right(20);
pros::Distance distance_front(17);
pros::Optical optical_top(16);

pros::adi::Pneumatics loader('A', false);
pros::adi::Pneumatics descore_mid('B', false);
pros::adi::Pneumatics wing('C', false);
pros::adi::Pneumatics trapdoor('D', true, true);
pros::adi::Pneumatics double_park('E', false);
pros::adi::Pneumatics grabber('F', false, true);



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
danielib::Sensors sensors(vertical_tracker, horizontal_tracker, inertial);

danielib::PID linearPID(7.5, 0.1, 22.5, 1, 0.5, 200);
danielib::PID angularPID(2.3, 0.2, 13.7, 3, 2, 100);
danielib::PID mtpLinearPID(7.2, 0.05, 25, 0, 1, 500);
danielib::PID mtpAngularPID(3.35, 0.2, 14);

danielib::Drivetrain chassis(left_mg, right_mg, sensors, mcl, 11.5, 3.25, 450, linearPID, angularPID);

void screen_print() {
    //pros::lcd::initialize();
    master.clear();
    while (true) {
        // odom position
        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %.2f", reduce_to_0_360(chassis.getPose().theta));

        pros::lcd::print(4, "Distances: %d (left), %d (right), %d (front)", distance_left.get_distance(), distance_right.get_distance(), distance_front.get_distance());

        pros::delay(100);
    }
}

void initialize() {
    pros::lcd::initialize(); // initialze llemu
    pros::Task screen_task(screen_print);

    chassis.calibrate();
    chassis.startTracking();
    chassis.startLocalization(24, -48, 180);
}

void competition_initialize() {

}

void disabled() {

}

void autonomous() {

}

void opcontrol() {

}