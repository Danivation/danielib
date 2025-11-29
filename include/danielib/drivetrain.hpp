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
        Drivetrain(
            pros::MotorGroup& leftMotors, 
            pros::MotorGroup& rightMotors, 
            Sensors& odomSensors, 
            danielib::Localization& localization, 
            float trackWidth, 
            float wheelSize, 
            float wheelRPM, 
            PID& linearPID, 
            PID& angularPID, 
            PID& mtpLinearPID, 
            PID& mtpAngularPID
        );

        void startTracking();
        void stopTracking();
        void startLocalization(float x, float y, float theta);
        void stopLocalization();
        void stopAllMovements();
        void stopMovement();
        bool isTracking();

        void calibrate();
        void setPose(float x, float y, float theta = infinityf());
        void setPose(Pose pose);
        Pose getPose(bool inRadians = false);

        /**
         * @brief Makes the following motion chained to this run in async mode
         * 
         * Example:
         * @code `chassis.async().moveToPose(x, y, theta);`
         */
        inline Drivetrain& async() {
            runAsync = true;
            return *this;
        }

        /**
         * @brief Drives straight for a given distance
         * 
         * @param distance target distance in inches
         * @param timeout timeout in ms
         * @param maxSpeed max speed the drivetrain can move out of 100
         */
        void driveForDistance(float distance, int timeout = infinityf(), float maxSpeed = 100);
        /**
         * @brief DNE - Turns to face a given target point, then drives straight to it
         */
        //void driveToPoint(float x, float y, int timeout = infinityf());
        /**
         * @brief Turns to a given target heading
         * 
         * @param heading target heading in degrees
         * @param timeout timeout in ms
         * @param maxSpeed max speed the drivetrain can move out of 100
         */
        void turnToHeading(float heading, int timeout = infinityf(), float maxSpeed = 100);
        /**
         * @brief Turns to face a given target point
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout timeout in ms
         * @param maxSpeed max speed the drivetrain can move out of 100
         */
        void turnToPoint(float x, float y, int timeout = infinityf(), float maxSpeed = 100);
        /**
         * @brief Moves to a given target point (not heading) using PIDs
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout timeout in ms
         * @param maxSpeed max speed the drivetrain can move out of 100
         */
        void moveToPoint(float x, float y, int timeout = infinityf(), bool reverse = false, float maxSpeed = 100);
        /**
         * @brief Moves to a given target pose using a boomerang controller
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param heading heading in degrees
         * @param timeout timeout in ms
         * @param leadDist boomerang carrot point multiplier, higher number leads to curvier turns
         * @param driftFactor limits the speed the drivetrain can move to avoid slipping, higher number leads to faster movements but more slippage
         * @param maxSpeed max speed the drivetrain can move out of 100
         * 
         * @note This does not obey exit conditions, only timeouts
         */
        void moveToPose(float x, float y, float heading, int timeout = infinityf(), bool reverse = false, float leadDist = 0.4, float driftFactor = 3, float maxSpeed = 100);
        /**
         * @brief DNE
         */
        //void followPath();
        /**
         * @brief DNE
         */
        //void followPoints();
    private:
        bool movementsEnabled = true;
        bool currentMovementEnabled = true;
        bool runAsync = false;

        PID& linearPID;
        PID& angularPID;
        PID& mtpLinearPID;
        PID& mtpAngularPID;
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
