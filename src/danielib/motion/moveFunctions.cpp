#include "danielib/danielib.hpp"

void danielib::Drivetrain::moveToPose(float x, float y, float heading, int timeout, float leadDist) {
    const float earlyExitRange = 0; // change this later to add support for motion chaining and stuff, and make the closeness distance also adjust with it

    if (!isTracking()) return;
    const int startTime = pros::millis();
    ExitCondition linearExit(linearPID->exitRange, linearPID->exitTime);
    ExitCondition angularExit(angularPID->exitRange, angularPID->exitTime);

    linearPID->reset();
    linearExit.reset();
    angularPID->reset();
    angularExit.reset();

    // deal with everything in radians internally
    Pose targetPose(x, y, toRadians(heading));

    bool close = false;
    bool prevSameSide = false;

    while (pros::millis() < startTime + timeout && (!linearExit.isDone() || !angularExit.isDone())) {
        Pose robotPose = getPose(true);

        // disable turning if robot is close to target
        float distance = robotPose.distance(targetPose);
        if (distance < 6 && close == false) close = true;

        // calculate carrot point for boomerang
        Pose carrotPose = targetPose - Pose(cos(targetPose.theta), sin(targetPose.theta)) * leadDist * distance;
        if (close) carrotPose = targetPose;

        // calculate if the robot is on the same side of the endpoint line as the carrot point
        bool robotSide = (robotPose.y - targetPose.y) * -sin(targetPose.theta) <= (robotPose.x - targetPose.x) * cos(targetPose.theta) + earlyExitRange;
        bool carrotSide = (carrotPose.y - targetPose.y) * -sin(targetPose.theta) <= (carrotPose.x - targetPose.x) * cos(targetPose.theta) + earlyExitRange;
        bool sameSide = (robotSide == carrotSide);
        // exit if close
        if (!sameSide && prevSameSide && close) break;
        prevSameSide = sameSide;

        // calculate errors
        float angularError = close ? angleError(robotPose.theta, targetPose.theta, true) : 
                                     angleError(robotPose.theta, robotPose.angle(carrotPose), true);
        float linearError = robotPose.distance(carrotPose);
        linearError *= cos(angleError(robotPose.theta, robotPose.angle(carrotPose), true));

        // update exit conditions
        linearExit.update(linearError);
        angularExit.update(toDegrees(angularError));

        // calculate outputs
        float linearOut = linearPID->update(linearError);
        float angularOut = angularPID->update(toDegrees(angularError));
        if (close) angularOut = 0;

        // desaturate outputs
        float leftPower = linearOut + angularOut;
        float rightPower = linearOut - angularOut;
        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower));
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move motors
        leftMotors->move(leftPower);
        rightMotors->move(rightPower);

        pros::delay(10);
    }

    // stop motors
    leftMotors->move(0);
    rightMotors->move(0);
}