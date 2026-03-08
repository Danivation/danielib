#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"

void danielib::Drivetrain::moveToPose(float x, float y, float heading, int timeout, bool reverse, float leadDist, float driftFactor, float maxSpeed, float earlyExitRange) {
    if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { moveToPose(x, y, heading, timeout, reverse, leadDist, driftFactor, maxSpeed, earlyExitRange); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    const float closeDist = 6;  // distance for it to be considered close
    const float turnLockDist = 3;

    // tunable parameters and stuff
    float linearMaxSlew = mtpLinearPID.slew;
    float angularMaxSlew = mtpAngularPID.slew;

    const int startTime = pros::millis();
    ExitCondition linearExit(mtpLinearPID.exitRange, mtpLinearPID.exitTime);
    ExitCondition angularExit(mtpAngularPID.exitRange, mtpAngularPID.exitTime);

    mtpLinearPID.reset();
    linearExit.reset();
    mtpAngularPID.reset();
    angularExit.reset();

    // deal with everything in radians internally
    Pose targetPose(x, y, d_toRadians(heading));
    if (reverse) targetPose.theta = fmod(targetPose.theta + M_PI, 2 * M_PI);

    bool close = false;
    bool prevSameSide = false;
    bool motionChained = false;

    std::uint32_t time = pros::millis();
    while (pros::millis() < startTime + timeout && movementsEnabled && currentMovementEnabled) {
        Pose robotPose = getPose(true);

        // disable turning if robot is close to target
        float distance = robotPose.distance(targetPose);
        if (distance < closeDist && !close) {
            close = true;
            maxSpeed = fmax(fabs(prevLinearOut), 60);
        }

        // calculate carrot point for boomerang
        Pose carrotPose = targetPose - Pose(sin(targetPose.theta), cos(targetPose.theta)) * leadDist * distance;
        carrotPose.theta = targetPose.theta;
        if (close) carrotPose = targetPose;

        // calculate if the robot is on the same side of the endpoint line as the carrot point
        bool robotSide = (robotPose.y - targetPose.y) * -sin(targetPose.theta) <= (robotPose.x - targetPose.x) * cos(targetPose.theta) + earlyExitRange;
        bool carrotSide = (carrotPose.y - targetPose.y) * -sin(targetPose.theta) <= (carrotPose.x - targetPose.x) * cos(targetPose.theta) + earlyExitRange;
        bool sameSide = (robotSide == carrotSide);
        // exit if close
        if (!sameSide && prevSameSide && close) break;
        if (fabs(distance) < fabs(earlyExitRange)) {
            motionChained = true;
            break;
        }
        prevSameSide = sameSide;

        // calculate errors

        // if close, target heading is the heading of the target point
        // if not close, target heading is the heading to face the carrot point
        float angularError = close ? d_angleError(!reverse ? robotPose.theta : robotPose.theta + M_PI, targetPose.theta, true) :
                                     d_angleError(!reverse ? robotPose.theta : robotPose.theta + M_PI, robotPose.angle(carrotPose), true);
        //float angularError = d_angleError(robotPose.theta, carrotPose.theta, true);
        float linearError = robotPose.distance(carrotPose) * cos(angularError);

        // update exit conditions
        linearExit.update(distance);
        angularExit.update(d_toDegrees(angularError));

        // calculate outputs (angular is negative because radians increase ccw, todo: fix inconsistency)
        float linearOut = mtpLinearPID.update(linearError);
        if (reverse) linearOut = -linearOut;
        float angularOut = -mtpAngularPID.update(d_toDegrees(angularError));
        if (distance < turnLockDist) angularOut = 0;

        // clamp to max speed
        linearOut = std::clamp(linearOut, -maxSpeed, maxSpeed);
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // constrain outputs to avoid slipping
        if (!close && linearMaxSlew != 0) linearOut = d_slew(linearOut, prevLinearOut, linearMaxSlew);
        if (angularMaxSlew != 0) angularOut = d_slew(angularOut, prevAngularOut, angularMaxSlew);

        // todo: fix radian increasing ccw inconsistency, right now it works but its a temporary fix
        float radius = 1 / fabs(d_getCurvature(d_fixRadians(robotPose), d_fixRadians(carrotPose)));
        float maxSlipSpeed(sqrt(driftFactor * radius * 9.8));
        // only clamps to constrain slipping, not to clamp to maxSpeed
        linearOut = std::clamp(linearOut, -maxSlipSpeed, maxSlipSpeed);

        // update previous values
        prevLinearOut = linearOut;
        prevAngularOut = angularOut;

        // calculate and desaturate outputs, effectively clamps to max speed
        float leftPower = linearOut + angularOut;
        float rightPower = linearOut - angularOut;
        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move motors
        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        pros::Task::delay_until(&time, 10);
    }

    if (!motionChained) {
        prevLinearOut = 0;
        prevAngularOut = 0;
    }

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}

void danielib::Drivetrain::moveToPoint(float x, float y, int timeout, bool reverse, float maxSpeed, float earlyExitRange) {
    if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { moveToPoint(x, y, timeout, reverse, maxSpeed, earlyExitRange); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    FILE* log_linearOut = fopen("/usd/log_linearOut.txt", "a");
    FILE* log_angularOut = fopen("/usd/log_angularOut.txt", "a");
    FILE* log_distance = fopen("/usd/log_distance.txt", "a");
    FILE* log_pose = fopen("/usd/log_pose.txt", "a");

    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    // tunable parameters and stuff
    const float closeDist = 6;  // distance for it to be considered close
    const float lineDist = 6;   // distance where the target is the line instead of the point
    float linearMaxSlew = mtpLinearPID.slew;
    float angularMaxSlew = mtpAngularPID.slew;

    // pids and exit conditions
    ExitCondition linearExit(mtpLinearPID.exitRange, mtpLinearPID.exitTime);
    mtpLinearPID.reset();
    mtpAngularPID.reset();
    linearExit.reset();

    // poses in radians
    Pose robotPose = getPose(true);
    Pose targetPose(x, y, 0);

    // loop variables
    bool close = false;
    bool usingLine = false;
    bool prevSide = false;
    bool motionChained = false;

    // line calculation variables
    float lineAngle = 0;
    float lineNx = 0;
    float lineNy = 0;
    
    // linear end variables
    float linePosition = 0;
    float currentDistance = 0;
    float distanceToLine = 0;

    // timers
    const int startTime = pros::millis();
    std::uint32_t time = pros::millis();

    // main loop
    while (pros::millis() < startTime + timeout && !linearExit.isDone() && movementsEnabled && currentMovementEnabled) {
        // get pose and distance to target
        robotPose = getPose(true);
        float distance = robotPose.distance(targetPose);

        // exit if within motion chaining range
        if (std::abs(distance) < earlyExitRange) {
            motionChained = true;
            break;
        }

        // set close when within close circle
        if (std::abs(distance) < closeDist) {
            close = true;
        }

        // if not close, recalculate target pose angle (used for angular PID target)
        if (!close) {
            targetPose.theta = robotPose.angle(targetPose);
        }

        // run once, when passing into line dist circle
        if (std::abs(distance) < lineDist) {
            if (!usingLine) {
                usingLine = true;

                // lock line angle
                lineAngle = d_fixRadians(targetPose.theta);
                lineNx = std::cos(lineAngle);
                lineNy = std::sin(lineAngle);

                // distance deltas
                float dx = robotPose.x - targetPose.x;
                float dy = robotPose.y - targetPose.y;

                // robot heading vector
                float cosH = std::cos(d_fixRadians(robotPose.theta));
                float sinH = std::sin(d_fixRadians(robotPose.theta));

                // denominator (detect parallel case)
                float denom = cosH * lineNx + sinH * lineNy;

                // get distance to line from the robot's current heading
                if (std::abs(denom) > 1e-4) {
                    distanceToLine = -(dx * lineNx + dy * lineNy) / denom;
                } else {
                    // heading parallel to line, fall back to perpendicular distance
                    distanceToLine = dx * lineNx + dy * lineNy;
                }
                linePosition = odomSensors.verticalTracker.getPosition();
            }
            usingLine = true;
        }

        if (usingLine) {
            // once inside of line distance, turn into a linear pid
            currentDistance = odomSensors.verticalTracker.getPosition() - linePosition;
            distance = distanceToLine - currentDistance;
        }

        // calculate angular error
        float driveHeading = robotPose.theta;
        if (reverse) driveHeading += M_PI;
        driveHeading = std::remainder(driveHeading, 2*M_PI);
        float angularError = d_angleError(targetPose.theta, driveHeading, true);
        if (usingLine) angularError = 0;

        // calculate linear error and cosine scale
        float linearError = distance * std::cos(angularError);
        linearExit.update(distance);

        // calculate linear output
        float linearOut = mtpLinearPID.update(linearError);
        if (reverse) linearOut = -linearOut;

        // calculate angular output and set to 0 if close
        float angularOut = mtpAngularPID.update(d_toDegrees(angularError));
        if (usingLine) angularOut = d_slew(0, prevAngularOut, 2.5);

        // clamp outputs to max speed
        linearOut = std::clamp(linearOut, -maxSpeed, maxSpeed);
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // slew outputs to avoid slipping
        if (linearMaxSlew != 0) linearOut = d_slew(linearOut, prevLinearOut, linearMaxSlew);
        if (angularMaxSlew != 0) angularOut = d_slew(angularOut, prevAngularOut, angularMaxSlew);

        // update previous values
        prevLinearOut = linearOut;
        prevAngularOut = angularOut;

        // calculate and desaturate outputs
        // if either output is greater than max speed, ratio both so that the higher one is equal to max speed
        float leftPower = linearOut + angularOut;
        float rightPower = linearOut - angularOut;
        float ratio = std::max(std::abs(leftPower), std::abs(rightPower)) / maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // log data
        if (log_linearOut) fprintf(log_linearOut, "(%d,%.1f),", pros::millis() - startTime, linearOut);
        if (log_angularOut) fprintf(log_angularOut, "(%d,%.1f),", pros::millis() - startTime, angularOut);
        if (log_distance) fprintf(log_distance, "(%d,%.1f),", pros::millis() - startTime, distance);
        if (log_pose) fprintf(log_pose, "(%.1f,%.1f),", robotPose.x, robotPose.y);

        // move motors and delay
        leftMotors.move(leftPower);
        rightMotors.move(rightPower);
        pros::Task::delay_until(&time, 10);
    }

    if (!motionChained) {
        prevLinearOut = 0;
        prevAngularOut = 0;
    }

    if (log_linearOut) fclose(log_linearOut);
    if (log_angularOut) fclose(log_angularOut);
    if (log_distance) fclose(log_distance);
    if (log_pose) fclose(log_pose);

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}