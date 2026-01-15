#include "danielib/danielib.hpp"
#include <cmath>

using namespace danielib;

void Drivetrain::update() {
    // deltas are changes since last update
    float deltaVertical = odomSensors.verticalTracker.getPosition() - prevVertical;
    float deltaHorizontal = odomSensors.horizontalTracker.getPosition() - prevHorizontal;
    float deltaTheta = angleError(toRadians(odomSensors.imu.getRotation()), prevTheta, true);

    // find how much robot has traveled since last update
    float localX;
    float localY;

    // prevent divide by 0
    if (deltaTheta == 0) {
        localX = deltaHorizontal;
        localY = deltaVertical;
    } else {
        // convert wheel movement to local x and y deltas
        localX = 2 * sinf(deltaTheta / 2) * ((deltaHorizontal / deltaTheta) + odomSensors.horizontalTracker.getOffset());
        localY = 2 * sinf(deltaTheta / 2) * ((deltaVertical / deltaTheta) + odomSensors.verticalTracker.getOffset());
    }

    // convert cartesian coordinates (local) to polar coordinates
    float avgTheta = prevTheta + (deltaTheta / 2);

    // mcl stuff
    prevPose = currentPose;

    // cooler math that works better (update global positions)
    currentPose.x += localY * sinf(avgTheta);
    currentPose.y += localY * cosf(avgTheta);
    currentPose.x += localX * -cosf(avgTheta);
    currentPose.y += localX * sinf(avgTheta);
    currentPose.theta = odomSensors.imu.getRotation();

    // update delta pose for mcl
    deltaPose = {currentPose.x - prevPose.x, currentPose.y - prevPose.y, deltaTheta};

    // update previous values
    prevVertical = odomSensors.verticalTracker.getPosition();
    prevHorizontal = odomSensors.horizontalTracker.getPosition();
    prevTheta = toRadians(odomSensors.imu.getRotation());
}

void Drivetrain::calibrate() {
    odomSensors.horizontalTracker.reset();
    odomSensors.verticalTracker.reset();
    odomSensors.imu.calibrate();
    //pros::delay(2500);
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void Drivetrain::setPose(float x, float y, float theta) {
    currentPose = Pose(x, y, theta);
    odomSensors.imu.setRotation(theta);
}

void Drivetrain::setPose(Pose pose) {
    currentPose = pose;
    odomSensors.imu.setRotation(pose.theta);
}

Pose Drivetrain::getPose(bool inRadians) {
    if (inRadians) return Pose(currentPose.x, currentPose.y, toRadians(currentPose.theta));
    return currentPose;
}

void Drivetrain::startTracking() {
    if (trackingType != 1) { 
        trackingType = 1;   // odom tracking only
        // reset pose to zero
        setPose(0, 0, 0);
        
        // make sure nothing was already running, then start the tracking task
        stopTracking();
        trackingTask = new pros::Task {[&] {
            while (true) {
                Drivetrain::update();
                pros::delay(10);
            }
        }};
    }
}

void Drivetrain::startLocalization(float x, float y, float theta) {
    if (trackingType != 2) {
        trackingType = 2;   // mcl tracking
        setPose(x, y, theta);
        localization.setPose({x, y, theta});

        // make sure nothing was already running, then start the tracking task
        stopTracking();
        trackingTask = new pros::Task {[&] {
            while (true) {
                for (auto& beam: localization.beams) {
                    beam.update();
                }

                Drivetrain::update();   // update odom
                auto pose = localization.run(deltaPose, localization.beams);
                currentPose.x = pose.x;
                currentPose.y = pose.y;
                pros::delay(200);
            }
        }};
    }
}

void Drivetrain::stopLocalization() {
    if (trackingType == 2) {
        trackingType = 1;
        stopTracking();
        startTracking();
    }
}

void Drivetrain::stopTracking() {
    if (trackingTask != nullptr) {
        trackingType = 0;   // no tracking
        trackingTask->remove(); // removes task from the rtos scheduler
        delete trackingTask;    // frees up memory allocated by the task
        trackingTask = nullptr; // resets the pointer to nullptr to signify no task running
    }
}

bool Drivetrain::isTracking() {
    return (trackingTask != nullptr);
}
