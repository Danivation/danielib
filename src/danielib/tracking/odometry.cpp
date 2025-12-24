#include "danielib/danielib.hpp"
#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"
#include <cmath>

using namespace danielib;

void Drivetrain::update() {
    // deltas are changes since last update
    float deltaVertical = odomSensors.verticalTracker.getPosition() - prevVertical;
    float deltaHorizontal = odomSensors.horizontalTracker.getPosition() - prevHorizontal;
    float deltaTheta = newPose ? 0 : d_angleError(d_toRadians(odomSensors.imu.getRotation()), prevTheta, true);

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

    // update global positions (cooler math that works better)
    currentPose.x += localY * sinf(avgTheta);
    currentPose.y += localY * cosf(avgTheta);
    currentPose.x += localX * -cosf(avgTheta);
    currentPose.y += localX * sinf(avgTheta);
    currentPose.theta = odomSensors.imu.getRotation();

    // update delta pose for mcl
    deltaPose = {currentPose.x - prevPose.x, currentPose.y - prevPose.y, deltaTheta};

    // update previous values
    prevPose = currentPose;
    prevVertical = odomSensors.verticalTracker.getPosition();
    prevHorizontal = odomSensors.horizontalTracker.getPosition();
    prevTheta = d_toRadians(odomSensors.imu.getRotation());
    newPose = false;
}

void Drivetrain::calibrate() {
    odomSensors.horizontalTracker.reset();
    odomSensors.verticalTracker.reset();
    odomSensors.imu.calibrate();
    //pros::delay(2500);
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void Drivetrain::setPose(float x, float y, float theta) {
    if (theta != infinityf()) odomSensors.imu.setRotation(theta);
    currentPose = Pose(x, y, theta);
    newPose = true;
}

void Drivetrain::setPose(Pose pose) {
    odomSensors.imu.setRotation(pose.theta);
    currentPose = pose;
    newPose = true;
}

void Drivetrain::distanceResetPose(std::initializer_list<Beam*> beams) {
    float sumX = 0;
    float sumY = 0;
    int countX = 0;
    int countY = 0;

    // loop through all beams
    for (Beam* beamPtr : beams) {
        if (!beamPtr) continue;  // skip null pointers
        
        Beam& beam = *beamPtr;
        
        // update beam distance (updates the actual beam object)
        beam.update();

        // skip bad readings
        float wallDistance = d_toInches(beam.distance);
        if (wallDistance <= 0) continue;

        // find global beam angle and position
        float robotAngle = d_fixRadians(d_toRadians(currentPose.theta));
        float sinRobotAngle = sinf(robotAngle);
        float cosRobotAngle = cosf(robotAngle);

        // calculate beam angle and position using offset
        float beamAngle = d_fixRadians(d_toRadians(currentPose.theta + beam.angleOffset));
        float beamX = currentPose.x + beam.yOffset * cosRobotAngle + beam.xOffset * sinRobotAngle;
        float beamY = currentPose.y + beam.yOffset * sinRobotAngle - beam.xOffset * cosRobotAngle;

        // calculate x and y positions of the wall based on beam distance
        // this is basically where it thinks the wall is based on that beam
        // (wallX, wallY) is the point on the wall that the beam is hitting
        float wallX = beamX + wallDistance * cosf(beamAngle);
        float wallY = beamY + wallDistance * sinf(beamAngle);

        // we need to filter this data to only use x and y positions that are actually on a wall
        // a beam facing one wall will not tell you anything useful about the other wall

        // Normalize angle to [0, 2π)
        float normalizedAngle = d_reduce_radians(beamAngle);

        float tolerance = d_toRadians(10);

        // Check if beam is aligned with east/west walls (pointing near 0° or 180°)
        bool pointingEast = (normalizedAngle < tolerance) || (normalizedAngle > 2 * M_PI - tolerance);
        bool pointingWest = (normalizedAngle > M_PI - tolerance) && (normalizedAngle < M_PI + tolerance);

        // Check if beam is aligned with north/south walls (pointing near 90° or 270°)
        bool pointingNorth = (normalizedAngle > M_PI_2 - tolerance) && (normalizedAngle < M_PI_2 + tolerance);
        bool pointingSouth = (normalizedAngle > 3 * M_PI_2 - tolerance) && (normalizedAngle < 3 * M_PI_2 + tolerance);
        
        if (pointingEast || pointingWest) {
            // Known wall position
            float knownWallX = pointingEast ? 70 : -70;
            
            // Calculate where the beam sensor must be
            float beamX = knownWallX - wallDistance * cosf(beamAngle);
            
            // Now calculate robot center from beam position
            // Reverse the offset transformation
            float robotX = beamX - (beam.yOffset * cosRobotAngle + beam.xOffset * sinRobotAngle);
            
            sumX += robotX;
            countX++;
        }

        if (pointingNorth || pointingSouth) {
            // Known wall position
            float knownWallY = pointingNorth ? 70 : -70;
            
            // Calculate where the beam sensor must be
            float beamY = knownWallY - wallDistance * sinf(beamAngle);
            
            // Now calculate robot center from beam position
            // Reverse the offset transformation
            float robotY = beamY - (beam.yOffset * sinRobotAngle - beam.xOffset * cosRobotAngle);
            
            sumY += robotY;
            countY++;
        }
    }

    // sets x and y based on averages of valid readings independently, keeps old value if no valid readings
    // if only x readings are valid, only x is changed (same for y)
    setPose(
        (countX > 0) ? (sumX / countX) : currentPose.x,
        (countY > 0) ? (sumY / countY) : currentPose.y
    );
}

Pose Drivetrain::getPose(bool inRadians) {
    if (inRadians) return Pose(currentPose.x, currentPose.y, d_toRadians(currentPose.theta));
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
        odomSensors.localization.setPose({x, y, theta});

        // make sure nothing was already running, then start the tracking task
        stopTracking();
        trackingTask = new pros::Task {[&] {
            while (true) {
                // iterates through beams and updates their distances
                for (auto& beam: odomSensors.localization.beams) {
                    beam.update();
                }

                Drivetrain::update();   // update odom
                Drivetrain::setPose(odomSensors.localization.run(deltaPose, odomSensors.localization.beams));   // update mcl
                pros::delay(50);
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
