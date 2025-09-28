#pragma once
#include "pros/motors.h"
#include "pros/motor_group.hpp"
#include "danielib/tracking/odometry.hpp"
#include "danielib/units/units.hpp"

namespace danielib {
/**
 * @brief Creates a drivetrain object
 */
class drivetrain {
    public:
        drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, danielib::odometry odom_sensors, float track_width, float wheel_size, float wheel_rpm);
    protected:
        pros::MotorGroup* left_motors;
        pros::MotorGroup* right_motors;
        danielib::odometry odom_sensors;
        float track_width;
        float wheel_size;
        float wheel_rpm;
};
} // namespace danielib
