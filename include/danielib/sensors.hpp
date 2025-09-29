#pragma once
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

namespace danielib {
class TrackerWheel {
    public:
        TrackerWheel(pros::Rotation* sensor, float wheelDiameter, float offset);
    protected:
        pros::Rotation* sensor;
        const float wheelDiameter;
        const float offset;
};

class Inertial {
    public:
        Inertial(pros::Imu* sensor1, float scale1 = 1.0f, pros::Imu* sensor2 = nullptr, float scale2 = 1.0f);
    protected:
        pros::Imu* sensor1;
        const float scale1;
        pros::Imu* sensor2;
        const float scale2;
};

/**
 * @brief Class for adding odometry to a drivetrain
 */
class Sensors {
    public:
        Sensors(TrackerWheel* verticalTracker, TrackerWheel* horizontalTracker, Inertial* imu);
    protected:
        TrackerWheel* verticalTracker;
        TrackerWheel* horizontalTracker;
        Inertial* imu;
};
} // namespace danielib