#include "danielib/danielib.hpp"
#include <cmath>

using namespace danielib;

void Drivetrain::update() {
    // deltas are changes since last update
    float deltaVertical = odomSensors->verticalTracker->getPosition() - prevVertical;
    float deltaHorizontal = odomSensors->horizontalTracker->getPosition() - prevHorizontal;
    float deltaTheta = toRadians(odomSensors->imu->getRotation()) - prevTheta;

    // find how much robot has traveled since last update
    float localX;
    float localY;

    // prevent divide by 0
    if (deltaTheta == 0) {
        localX = deltaHorizontal;
        localY = deltaVertical;
    } else {
        // convert wheel movement to local x and y deltas
        localX = 2*sinf(deltaTheta/2) * ((deltaHorizontal/deltaTheta) + odomSensors->horizontalTracker->getOffset());
        localY = 2*sinf(deltaTheta/2) * ((deltaVertical/deltaTheta) + odomSensors->verticalTracker->getOffset());
    }

    // convert cartesian coordinates (local) to polar coordinates
    float avgTheta = prevTheta + (deltaTheta / 2);
    float polarRadius = hypotf(localY, localX);
    float localPolarAngle = atan2f(localY, localX);

    // rotate polar coordinates according to the robot's local frame
    float globalPolarAngle = localPolarAngle - avgTheta;

    // convert polar coordinates back into cartesian coordinates (global)
    float deltaX = polarRadius * cosf(globalPolarAngle);
    float deltaY = polarRadius * sinf(globalPolarAngle);

    // update global positions
    currentPose.x += deltaX;
    currentPose.y += deltaY;
    currentPose.theta = odomSensors->imu->getRotation();

    // update previous values
    prevVertical = odomSensors->verticalTracker->getPosition();
    prevHorizontal = odomSensors->horizontalTracker->getPosition();
    prevTheta = toRadians(odomSensors->imu->getRotation());
}

void Drivetrain::setPose(float x, float y, float theta) {
    currentPose = Pose(x, y, theta);
}

void Drivetrain::setPose(Pose pose) {
    currentPose = pose;
}

Pose Drivetrain::getPose() {
    return currentPose;
}

void Drivetrain::startTracking() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                Drivetrain::update();
                pros::delay(10);
            }
        }};
    }
}

void Drivetrain::stopTracking() {
    if (trackingTask != nullptr) {
        trackingTask->remove();
        delete trackingTask;
        trackingTask = nullptr;
    }
}
