#pragma once
#include <optional>
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
        inertial(pros::Imu* sensor, float scale = 1.0f);
        inertial(pros::Imu* sensor_1, pros::Imu* sensor_2, float scale_1 = 1.0f, float scale_2 = 1.0f);
};

/**
 * @brief Class for adding odometry to a drivetrain
 */
class odometry {
    public:
        odometry(tracker_wheel* vertical_tracker, tracker_wheel* horizontal_tracker, inertial* imu);
};
} // namespace danielib