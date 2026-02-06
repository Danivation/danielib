#pragma once
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "danielib/mcl.hpp"

namespace danielib {
class TrackerWheel {
    public:
        TrackerWheel(pros::Rotation& sensor, float wheelDiameter, float offset, float angle);

        void reset();
        float getPosition();
        float getAngle();
        float getOffset();

        pros::Rotation& sensor;
        const float wheelDiameter;
        const float offset;
        const float angle;
};

class Inertial {
    public:
        Inertial(pros::Imu& sensor1, float scale1 = 1.0f, pros::Imu* sensor2 = nullptr, float scale2 = 1.0f);

        void calibrate();
        float getRotation();
        float getHeading();
        void setRotation(float angle);
        void setHeading(float angle);

        pros::Imu& sensor1;
        const float scale1;
        pros::Imu* sensor2;
        const float scale2;
};

/**
 * @brief Class for adding odometry to a drivetrain
 */
class Sensors {
    public:
        Sensors(TrackerWheel& tracker1, TrackerWheel& tracker2, Inertial& imu, Localization& localization);

        TrackerWheel& tracker1;
        TrackerWheel& tracker2;
        Inertial& imu;
        Localization& localization;
};

} // namespace danielib