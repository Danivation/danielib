#pragma once
#include "pros/motors.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "danielib/sensors.hpp"
#include "danielib/pose.hpp"

namespace danielib {
/**
 * @brief Creates a drivetrain object
 */
class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup* left_motors, pros::MotorGroup* right_motors, Sensors* odom_sensors, float track_width, float wheel_size, float wheel_rpm);
        
        void setPose(float x, float y, float theta);
        void setPose(Pose pose);

        Pose getPose();

        pros::Task startOdom();
    protected:
        pros::MotorGroup* left_motors;
        pros::MotorGroup* right_motors;
        Sensors* odom_sensors;
        float track_width;
        float wheel_size;
        float wheel_rpm;
};
} // namespace danielib
