#pragma once
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

namespace danielib {
class TrackerWheel {
    public:
        TrackerWheel(pros::Rotation* sensor, float wheel_diameter, float offset);
    protected:
        pros::Rotation* sensor;
        float wheel_diameter;
        float offset;
};

class Inertial {
    public:
        Inertial(pros::Imu* sensor, float scale = 1.0f);
        Inertial(pros::Imu* sensor_1, pros::Imu* sensor_2, float scale_1 = 1.0f, float scale_2 = 1.0f);
};

/**
 * @brief Class for adding odometry to a drivetrain
 */
class Sensors {
    public:
        Sensors(TrackerWheel* vertical_tracker, TrackerWheel* horizontal_tracker, Inertial* imu);
};
} // namespace danielib