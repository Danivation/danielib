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

    Pose targetPose(x, y, heading);

    bool close = false;
    bool prevSameSide = false;

    float power = 0;
    float currentDistance = odomSensors->verticalTracker->getPosition();
    float error = 0;

    while (pros::millis() < startTime + timeout && !linearExit.isDone()) {
        Pose robotPose = getPose();

        float distance = robotPose.distance(targetPose);

        if (distance < 6 && close == false) {
            close = true;
            // max speed stuff so it like yk doesnt do weird stuff
            // params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

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
        float angularError = reduce_to_180_180(robotPose.theta - targetPose.theta);


        error = distance - currentDistance;
        power = linearPID->update(error);
        linearExit.update(error);

        leftMotors->move(power);
        rightMotors->move(power);

        pros::delay(10);
    }

    // stop motors
    leftMotors->move(0);
    rightMotors->move(0);
}