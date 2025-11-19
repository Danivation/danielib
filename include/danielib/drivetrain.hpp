#pragma once
#include <cmath>
#include "pros/motors.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "danielib/sensors.hpp"
#include "danielib/pose.hpp"
#include "danielib/pid.hpp"
#include "danielib/mcl.hpp"

namespace danielib {
/**
 * @brief Creates a drivetrain object
 */
class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, Sensors& odomSensors, danielib::Localization& localization, float trackWidth, float wheelSize, float wheelRPM, PID& linearPID, PID& angularPID);

        void startTracking();
        void startLocalization(float x, float y, float theta);
        void stopLocalization();
        void stopTracking();
        bool isTracking();

        void stopAllMovements();

        void calibrate();

        void setPose(float x, float y, float theta);
        void setPose(Pose pose);
        Pose getPose(bool inRadians = false);

        /**
         * @brief Drives straight for a given distance
         * 
         * @param distance target distance in inches
         * @param timeout timeout in ms
         */
        void driveForDistance(float distance, int timeout = infinityf(), float maxSpeed = 127);
        /**
         * @brief DNE - Turns to face a given target point, then drives straight to it
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout timeout in ms
         */
        void driveToPoint(float x, float y, int timeout = infinityf()); // if within like 2 degrees just drive, if not, turn then drive
        /**
         * @brief Turns to a given target heading
         * 
         * @param heading target heading in degrees
         * @param timeout timeout in ms
         */
        void turnToHeading(float heading, int timeout = infinityf(), float maxSpeed = 127);
        /**
         * @brief Turns to face a given target point
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout timeout in ms
         */
        void turnToPoint(float x, float y, int timeout = infinityf(), float maxSpeed = 127);
        /**
         * @brief DNE
         */
        void moveToPoint();
        /**
         * @brief Moves to a given target pose using a boomerang controller
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param heading heading in degrees
         * @param timeout timeout in ms
         * @param leadDist boomerang carrot point multiplier, higher number leads to curvier turns
         * @param driftFactor limits the speed the drivetrain can move to avoid slipping, higher number leads to faster movements but more slippage
         * @param maxSpeed max speed the drivetrain can move out of 127
         */
        void moveToPose(float x, float y, float heading, int timeout = infinityf(), bool reverse = false, float leadDist = 0.4, float driftFactor = 3, float maxSpeed = 127);
        /**
         * @brief DNE
         */
        void followPath();
        /**
         * @brief DNE
         */
        void followPoints();
    private:
        bool movementsEnabled = true;

        PID& linearPID;
        PID& angularPID;
        pros::MotorGroup& leftMotors;
        pros::MotorGroup& rightMotors;
        Sensors& odomSensors;
        danielib::Localization& localization;
        const float trackWidth;
        const float wheelSize;
        const float wheelRPM;

        pros::Task* trackingTask = nullptr;
        int trackingType = 0;
        Pose currentPose = {0, 0, 0};
        void update();

        bool newPose = false;

        float prevVertical = 0;
        float prevHorizontal = 0;
        float prevTheta = 0;

        // needed for mcl, change in pose since last odom update
        Pose prevPose = {0, 0, 0};
        Pose deltaPose = {0, 0, 0};
};
} // namespace danielib
