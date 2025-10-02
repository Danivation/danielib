#pragma once
#include "pros/motors.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "danielib/sensors.hpp"
#include "danielib/pose.hpp"
#include "danielib/pid.hpp"

namespace danielib {
/**
 * @brief Creates a drivetrain object
 */
class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, Sensors* odomSensors, float trackWidth, float wheelSize, float wheelRPM, PID* linearPID, PID* angularPID);

        void startTracking();
        void stopTracking();

        void calibrate();

        void setPose(float x, float y, float theta);
        void setPose(Pose pose);
        Pose getPose();

        void driveForDistance(float distance, int timeout = 0);
        void driveToPoint(); // if within like 2 degrees just drive, if not, turn then drive
        void turnToHeading(float heading, int timeout = 0);
        void turnToPoint();
        void moveToPoint();
        void moveToPose();
        void followPath();
        void followPoints();
    private:
        PID* linearPID;
        PID* angularPID;
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        Sensors* odomSensors;
        const float trackWidth;
        const float wheelSize;
        const float wheelRPM;

        pros::Task* trackingTask = nullptr;
        Pose currentPose = {0, 0, 0};

        float prevVertical = 0;
        float prevHorizontal = 0;
        float prevTheta = 0;
        void update();
};
} // namespace danielib
