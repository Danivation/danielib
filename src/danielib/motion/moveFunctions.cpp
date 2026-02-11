#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"

void danielib::Drivetrain::moveToPose(float x, float y, float heading, int timeout, bool reverse, float leadDist, float driftFactor, float maxSpeed, float earlyExitRange) {
    if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { moveToPose(x, y, heading, timeout, reverse, leadDist, driftFactor, maxSpeed); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    const float closeDist = 6;  // distance for it to be considered close
    const float turnLockDist = 3;

    // tunable parameters and stuff
    float linearMaxSlew = 20;
    float angularMaxSlew = 5;

    const int startTime = pros::millis();
    ExitCondition linearExit(linearPID.exitRange, linearPID.exitTime);
    ExitCondition angularExit(angularPID.exitRange, angularPID.exitTime);

    linearPID.reset();
    linearExit.reset();
    angularPID.reset();
    angularExit.reset();

    // deal with everything in radians internally
    Pose targetPose(x, y, d_toRadians(heading));
    if (reverse) targetPose.theta = fmod(targetPose.theta + M_PI, 2 * M_PI);

    bool close = false;
    bool prevSameSide = false;
    float prevLinearOut = 0;
    float prevAngularOut = 0;

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
        if (fabs(distance) < fabs(earlyExitRange)) break;
        prevSameSide = sameSide;

        // calculate errors

        // if close, target heading is the heading of the target point
        // if not close, target heading is the heading to face the carrot point
        float angularError = close ? d_angleError(!reverse ? robotPose.theta : robotPose.theta + M_PI, targetPose.theta, true) :
                                     d_angleError(!reverse ? robotPose.theta : robotPose.theta + M_PI, robotPose.angle(carrotPose), true);
        //float angularError = d_angleError(robotPose.theta, carrotPose.theta, true);
        float linearError = robotPose.distance(carrotPose) * cos(angularError);

        // update exit conditions
        linearExit.update(robotPose.distance(targetPose));
        angularExit.update(d_toDegrees(angularError));

        // calculate outputs (angular is negative because radians increase ccw, todo: fix inconsistency)
        float linearOut = linearPID.update(linearError);
        if (reverse) linearOut = -linearOut;
        float angularOut = -angularPID.update(d_toDegrees(angularError));
        if (distance < turnLockDist) angularOut = 0;

        // clamp to max speed
        linearOut = std::clamp(linearOut, -maxSpeed, maxSpeed);
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // constrain outputs to avoid slipping
        linearOut = d_slew(linearOut, prevLinearOut, linearMaxSlew);
        angularOut = d_slew(angularOut, prevAngularOut, angularMaxSlew);

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

        pros::delay(10);
    }

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}

// move to point max speed is always 100/127
void danielib::Drivetrain::moveToPoint(float x, float y, int timeout, bool reverse, float maxSpeed, float earlyExitRange) {
    if (!isTracking()) return;
    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { moveToPoint(x, y, timeout, reverse, maxSpeed); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();
    currentMovementEnabled = true;

    const float closeDist = 5.5;  // distance for it to be considered close

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

    Pose lastPose = getPose(true);

    // deal with everything in radians internally
    Pose targetPose(x, y, 0);
    targetPose.theta = lastPose.angle(targetPose);
    if (reverse) targetPose.theta = fmod(targetPose.theta + M_PI, 2 * M_PI);

    bool close = false;
    bool prevSide = false;
    float prevLinearOut = 0;
    float prevAngularOut = 0;

    // keep moving unless the timeout happens, the linear exit condition happens, or the movement is disabled
    while (pros::millis() < startTime + timeout && !linearExit.isDone() && movementsEnabled && currentMovementEnabled) {
        Pose robotPose = getPose(true);
        float distance = robotPose.distance(targetPose);

        // slew max speed down to 70 when close
        if (distance < closeDist) {
            close = true;
            if (linearMaxSlew != 0) maxSpeed = d_slew(fabs(prevLinearOut), 70, linearMaxSlew);
        }

        // recalculate target heading when not close
        lastPose = getPose(true);
        if (!close) targetPose.theta = lastPose.angle(targetPose);

        // calculate what side of the endpoint line the robot is on, or if it has passed the target
        bool robotSide = (robotPose.y - targetPose.y) * -sin(targetPose.theta) <= (robotPose.x - targetPose.x) * cos(targetPose.theta) + earlyExitRange;
        // exit if robot moves past target point
        if (robotSide != prevSide && close) break;
        if (fabs(distance) < fabs(earlyExitRange)) break;
        prevSide = robotSide;

        // calculate errors
        float angularError = d_angleError(!reverse ? robotPose.theta : robotPose.theta + M_PI, targetPose.theta, true);
        float linearError = robotPose.distance(targetPose) * cos(angularError);

        // update exit conditions
        linearExit.update(robotPose.distance(targetPose));
        angularExit.update(d_toDegrees(angularError));

        // calculate outputs (angular is negative because radians increase ccw, todo: fix inconsistency)
        float linearOut = mtpLinearPID.update(linearError);
        if (reverse) linearOut = -linearOut;
        float angularOut = -mtpAngularPID.update(d_toDegrees(angularError));
        if (close) angularOut = 0;

        // clamp outputs to max speed (should have negative effects but oh well)
        linearOut = std::clamp(linearOut, -maxSpeed, maxSpeed);
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // slew outputs to avoid slipping
        if (!close && linearMaxSlew != 0) linearOut = d_slew(linearOut, prevLinearOut, linearMaxSlew);
        if (angularMaxSlew != 0) angularOut = d_slew(angularOut, prevAngularOut, angularMaxSlew);

        // update previous values
        prevLinearOut = linearOut;
        prevAngularOut = angularOut;

        // calculate and desaturate outputs
        // if either output is greater than max speed, ratio both so that the higher one is equal to max speed
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

        pros::delay(10);
    }

    // stop motors
    leftMotors.brake();
    rightMotors.brake();
    motionMutex.give();
}