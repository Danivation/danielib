#pragma once
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

namespace danielib {
class TrackerWheel {
    public:
        TrackerWheel(pros::Rotation& sensor, float wheelDiameter, float offset);

        void reset();
        float getPosition();
        float getOffset();
    //protected:
        pros::Rotation& sensor;
        const float wheelDiameter;
        const float offset;
};

class Inertial {
    public:
        Inertial(pros::Imu& sensor1, float scale1 = 1.0f, pros::Imu* sensor2 = nullptr, float scale2 = 1.0f);

        void calibrate();
        float getRotation();
        float getHeading();
        void setRotation(float angle);
        void setHeading(float angle);
    //protected:
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
        Sensors(TrackerWheel& verticalTracker, TrackerWheel& horizontalTracker, Inertial& imu);
        TrackerWheel& verticalTracker;
        TrackerWheel& horizontalTracker;
        Inertial& imu;
};

/* class Controllers {
    public:
        Controllers(std::initializer_list<PID> ints) :
            ints(ints)
        {}
        std::initializer_list<PID> ints;
        
}; */
} // namespace danielib