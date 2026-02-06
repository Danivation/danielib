#include "danielib/danielib.hpp"
#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"
#include <cmath>

using namespace danielib;

void Drivetrain::update() {
    // deltas are changes since last update
    float deltaTracker1 = odomSensors.tracker1.getPosition() - prevTracker1;
    float deltaTracker2 = odomSensors.tracker2.getPosition() - prevTracker2;
    float deltaTheta = newPose ? 0 : d_angleError(d_toRadians(odomSensors.imu.getRotation()), prevTheta, true);
    newPose = false;

    // Get tracking wheel angles (in radians, 0 = forward/north)
    float angle1 = odomSensors.tracker1.getAngle();  // e.g., PI/4 for northeast (45°)
    float angle2 = odomSensors.tracker2.getAngle();  // e.g., 3*PI/4 for northwest (135°)

    // Account for wheel offsets rotating around the center
    float offset1Contribution = odomSensors.tracker1.getOffset() * deltaTheta;
    float offset2Contribution = odomSensors.tracker2.getOffset() * deltaTheta;

    // Subtract offset effects from wheel readings
    float correctedDelta1 = deltaTracker1 - offset1Contribution;
    float correctedDelta2 = deltaTracker2 - offset2Contribution;

    // Solve the system of equations:
    // correctedDelta1 = localX * sin(angle1) + localY * cos(angle1)
    // correctedDelta2 = localX * sin(angle2) + localY * cos(angle2)

    float sin1 = sinf(angle1);
    float cos1 = cosf(angle1);
    float sin2 = sinf(angle2);
    float cos2 = cosf(angle2);

    // Using Cramer's rule to solve for localX and localY
    float determinant = sin1 * cos2 - sin2 * cos1;
    float localX;
    float localY;

    // Check if wheels are parallel (determinant near zero)
    if (fabs(determinant) < 1e-6) {
        // Wheels are parallel - can't determine both X and Y independently
        // Fall back to assuming motion along average wheel direction
        localX = correctedDelta1 * sin1;  // or handle error appropriately
        localY = correctedDelta1 * cos1;
    } else {
        // Solve using Cramer's rule
        float localX = (correctedDelta1 * cos2 - correctedDelta2 * cos1) / determinant;
        float localY = (sin1 * correctedDelta2 - sin2 * correctedDelta1) / determinant;
    }

    // Apply arc correction for curved paths
    if (deltaTheta != 0) {
        float halfTheta = deltaTheta / 2;
        float sinHalfTheta = sinf(halfTheta);
        float correctionFactor = halfTheta / sinHalfTheta;
        
        localX *= correctionFactor;
        localY *= correctionFactor;
    }

    // Convert to global coordinates
    float avgTheta = prevTheta + (deltaTheta / 2);
    float sinTheta = sinf(avgTheta);
    float cosTheta = cosf(avgTheta);
    
    currentPose.x += localY * sinTheta + localX * cosTheta;
    currentPose.y += localY * cosTheta - localX * sinTheta;
    currentPose.theta = odomSensors.imu.getRotation();

    // update delta pose for mcl
    deltaPose = {currentPose.x - prevPose.x, currentPose.y - prevPose.y, deltaTheta};

    // update previous values
    prevPose = currentPose;
    prevTracker1 = odomSensors.tracker1.getPosition();
    prevTracker2 = odomSensors.tracker2.getPosition();
    prevTheta = d_toRadians(odomSensors.imu.getRotation());
}

void Drivetrain::calibrate() {
    odomSensors.tracker2.reset();
    odomSensors.tracker1.reset();
    odomSensors.imu.calibrate();
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void Drivetrain::setPose(float x, float y, float theta) {
    if (theta != infinityf()) odomSensors.imu.setRotation(theta);
    currentPose = Pose(x, y, theta);
    newPose = true;
    pros::delay(5);
}

void Drivetrain::setPose(Pose pose) {
    odomSensors.imu.setRotation(pose.theta);
    currentPose = pose;
    newPose = true;
    pros::delay(5);
}

void Drivetrain::distanceResetPose(std::initializer_list<Beam*> beams) {
    float sumX = 0;
    float sumY = 0;
    int countX = 0;
    int countY = 0;
    pros::delay(5);

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
        float sinBeamAngle = sinf(beamAngle);
        float cosBeamAngle = cosf(beamAngle);

        // calculate x and y positions of the wall based on beam distance
        // this is basically where it thinks the wall is based on that beam
        // (wallX, wallY) is the point on the wall that the beam is hitting
        float wallX = beamX + wallDistance * cosBeamAngle;
        float wallY = beamY + wallDistance * sinBeamAngle;

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
            float knownWallX = pointingEast ? 70.5 : -70.5;
            
            // Calculate where the beam sensor must be
            float beamX = knownWallX - wallDistance * cosBeamAngle;
            
            // Now calculate robot center from beam position
            // Reverse the offset transformation
            float robotX = beamX - (beam.yOffset * cosRobotAngle + beam.xOffset * sinRobotAngle);
            
            sumX += robotX;
            countX++;
        }

        if (pointingNorth || pointingSouth) {
            // Known wall position
            float knownWallY = pointingNorth ? 70.5 : -70.5;
            
            // Calculate where the beam sensor must be
            float beamY = knownWallY - wallDistance * sinBeamAngle;
            
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
    pros::delay(10);
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
                auto pose = odomSensors.localization.run(deltaPose, odomSensors.localization.beams);
                currentPose.x = pose.x;
                currentPose.y = pose.y;
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
