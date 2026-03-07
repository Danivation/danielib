#include "danielib/drivetrain.hpp"
#include "danielib/exit.hpp"
#include "danielib/utils.hpp"
#include "danielib/pid.hpp"
#include <cmath>

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


void danielib::Drivetrain::moveToPoint(
    float x,
    float y,
    int timeout,
    bool reverse,
    float maxSpeed,
    float earlyExitRange)
{
    if (!isTracking()) return;

    if (runAsync) {
        runAsync = false;
        pros::Task task([&]() { moveToPoint(x, y, timeout, reverse, maxSpeed, earlyExitRange); });
        pros::delay(10);  // give the task some time to start
        return;
    }

    motionMutex.take();

    FILE* log = fopen("/usd/log.txt", "a");

    currentMovementEnabled = true;
    maxSpeed *= 1.27;

    const float closeDist = 5;
    const float lineDist  = 6;
    const float curvatureGain = 0.02f;

    float linearMaxSlew  = mtpLinearPID.slew;
    float angularMaxSlew = mtpAngularPID.slew;

    const uint32_t startTime = pros::millis();

    ExitCondition linearExit(mtpLinearPID.exitRange, mtpLinearPID.exitTime);

    mtpLinearPID.reset();
    mtpAngularPID.reset();
    linearExit.reset();

    Pose target(x, y, 0);
    Pose robot = getPose(true);

    bool close = false;
    bool turnLock = false;
    bool motionChained = false;

    uint32_t time = pros::millis();

    // compute fixed exit line direction
    float exitHeading = std::atan2(target.y - robot.y, target.x - robot.x);
    if (reverse) exitHeading = d_sanitizeAngle(exitHeading + M_PI, true);

    float nx = -std::cos(exitHeading);
    float ny =  std::sin(exitHeading);

    // main loop
    while (pros::millis() < startTime + timeout && !linearExit.isDone() && movementsEnabled && currentMovementEnabled) {
        robot = getPose(true);

        // float dx = target.x - robot.x;
        // float dy = target.y - robot.y;

        // float distance = hypot(dx, dy);
        float distance = robot.distance(target);

        // set close if close
        if (distance < closeDist) {
            close = true;
        }

        // motion chain
        if (distance < std::abs(earlyExitRange)) {
            motionChained = true;
            break;
        }

        // heading to target
        // float targetHeading = atan2(dy, dx);
        float targetHeading = robot.angle(target);
        if (reverse) targetHeading = d_sanitizeAngle(targetHeading + M_PI, true);

        // angular error
        float angularError = d_angleError(targetHeading, robot.theta, true);

        // forward projection (cosine scaling)
        float heading = robot.theta;
        if (reverse) heading += M_PI;
        // float linearError =
        //       dx * cos(heading)
        //     + dy * sin(heading);
        float cosAngular = std::cos(angularError);
        float linearError = distance;
        if (cosAngular >= 0 && cosAngular <= 1) linearError *= cosAngular;

        // perpendicular distance to exit line
        float distanceToLine = (robot.x - target.x) * nx + (robot.y - target.y) * ny;

        // turning lock
        if (distance < lineDist) {
            turnLock = true;
            distance = std::abs(distanceToLine);
        }

        // update exit condition
        linearExit.update(distance);

        // PID outputs
        float linearOut  = mtpLinearPID.update(linearError);
        float angularOut = mtpAngularPID.update(angularError);
        if (reverse) linearOut = -linearOut;

        // curvature feedforward (smooth arcs)
        angularOut += linearOut * curvatureGain;

        // reduce turning strength near target
        float turnScale = std::clamp(distance / 12.0f, 0.25f, 1.0f);
        angularOut *= turnScale;

        // turn lock near final line
        if (close || turnLock) angularOut = d_slew(0, prevAngularOut, 4);

        // clamp outputs
        linearOut  = std::clamp(linearOut,  -maxSpeed, maxSpeed);
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // slew limiting
        if (distance > 8 && linearMaxSlew != 0) linearOut = d_slew(linearOut, prevLinearOut, linearMaxSlew);
        if (distance > 8 && angularMaxSlew != 0) angularOut = d_slew(angularOut, prevAngularOut, angularMaxSlew);

        prevLinearOut  = linearOut;
        prevAngularOut = angularOut;

        // differential drive mix
        float leftPower  = linearOut + angularOut;
        float rightPower = linearOut - angularOut;

        // desaturate output
        float ratio = std::max(std::abs(leftPower), std::abs(rightPower)) / maxSpeed;
        if (ratio > 1) {
            leftPower  /= ratio;
            rightPower /= ratio;
        }

        // move motors
        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        // log
        if (log) fprintf(log, "(%d,%.2f),", pros::millis() - startTime, distance);

        pros::Task::delay_until(&time, 10);
    }

    if (!motionChained) {
        prevLinearOut = 0;
        prevAngularOut = 0;
    }

    if (log) fclose(log);

    leftMotors.brake();
    rightMotors.brake();

    motionMutex.give();
}