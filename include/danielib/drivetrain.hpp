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
enum class SwingSide { LEFT, RIGHT };

class Drivetrain {
    public:
        Drivetrain(
            pros::MotorGroup& leftMotors, 
            pros::MotorGroup& rightMotors, 
            Sensors& odomSensors, 
            float trackWidth, 
            float wheelSize, 
            float wheelRPM, 
            PID& linearPID, 
            PID& angularPID, 
            PID& mtpLinearPID, 
            PID& mtpAngularPID,
            PID& swingAngularPID
        );

        bool isTracking();

        void startTracking();
        void stopTracking();
        void startLocalization(float x, float y, float theta);
        void stopLocalization();

        void stopAllMovements();
        void stopMovement();

        void waitUntilDone();
        void setSpeed();

        void calibrate();
        void setPose(float x, float y, float theta = infinityf());
        void setPose(Pose pose);

        /**
         * @brief Resets the pose based on distance sensors
         * 
         * @param beams distance sensor beams to consider when resetting pose
         */
        void distanceResetPose(std::initializer_list<Beam*> beams);

        /**
         * @brief Returns the current pose of the drivetrain
         */
        Pose getPose(bool inRadians = false);

        /**
         * @brief Makes the following motion chained to this run in async mode (not block execution of other things)
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
        void driveForDistance(float distance, int timeout = infinityf(), float maxSpeed = 100, float earlyExitRange = 0);

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
         * @brief Turns to a given target heading
         * 
         * @param heading target heading in degrees
         * @param side moving side
         * @param timeout timeout in ms
         * @param maxSpeed max speed the drivetrain can move out of 100
         */
        void swingToHeading(float heading, SwingSide side, int timeout = infinityf(), float maxSpeed = 100);

        /**
         * @brief Moves to a given target point (not heading) using PIDs
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout timeout in ms
         * @param maxSpeed max speed the drivetrain can move out of 100
         */
        void moveToPoint(float x, float y, int timeout = infinityf(), bool reverse = false, float maxSpeed = 100, float earlyExitRange = 0);

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
        void moveToPose(float x, float y, float heading, int timeout = infinityf(), bool reverse = false, float leadDist = 0.4, float driftFactor = 3, float maxSpeed = 100, float earlyExitRange = 0);

    private:
        PID& linearPID;
        PID& angularPID;
        PID& mtpLinearPID;
        PID& mtpAngularPID;
        PID& swingAngularPID;

        pros::MotorGroup& leftMotors;
        pros::MotorGroup& rightMotors;
        Sensors& odomSensors;
        const float trackWidth;
        const float wheelSize;
        const float wheelRPM;

        pros::Task* trackingTask = nullptr;
        int trackingType = 0;
        Pose currentPose = {0, 0, 0};
        void update();

        float prevVertical = 0;
        float prevHorizontal = 0;
        float prevTheta = 0;

        // needed for mcl, change in pose since last odom update
        Pose prevPose = {0, 0, 0};
        Pose deltaPose = {0, 0, 0};
        
        // motion vars
        bool newPose = false;
        pros::Mutex motionMutex;
        bool movementsEnabled = true;
        bool currentMovementEnabled = true;
        bool runAsync = false;
        int currentMaxSpeed = 0;
};
} // namespace danielib
