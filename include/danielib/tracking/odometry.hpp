#pragma once
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

namespace danielib {
class tracker_wheel {
    public:
        tracker_wheel(pros::Rotation* sensor, float wheel_diameter, float offset);
    protected:
        pros::Rotation* sensor;
        float wheel_diameter;
        float offset;
};

class inertial {
    public:
        inertial(pros::Imu* sensor, float scale);
        inertial(pros::Imu* sensor_1, pros::Imu* sensor_2, float scale_1, float scale_2);
};

/**
 * @brief Class for adding odometry to a drivetrain
 */
class odometry {
    public:
        odometry(tracker_wheel* vertical_tracker, tracker_wheel* horizontal_tracker, inertial imu);
};
} // namespace danielib