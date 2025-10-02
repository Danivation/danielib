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
        Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, Sensors* odomSensors, /* Controllers* controllerSettings,  */float trackWidth, float wheelSize, float wheelRPM);

        void startTracking();
        void stopTracking();

        void calibrate();

        void setPose(float x, float y, float theta);
        void setPose(Pose pose);
        Pose getPose();

        void driveForDistance();
        void driveToPoint(); // if within like 2 degrees just drive, if not, turn then drive
        void turnToHeading(float heading, int timeout);
        void turnToPoint();
        void moveToPoint();
        void moveToPose();
        void followPath();
        void followPoints();
    protected:
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        Sensors* odomSensors;
        const float trackWidth;
        const float wheelSize;
        const float wheelRPM;
    private:
        pros::Task* trackingTask = nullptr;
        Pose currentPose = {0, 0, 0};

        float prevVertical = 0;
        float prevHorizontal = 0;
        float prevTheta = 0;
        void update();
};
} // namespace danielib
