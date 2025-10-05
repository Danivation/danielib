#include "danielib/danielib.hpp"

void danielib::Drivetrain::moveToPose(float x, float y, float heading, int timeout, float leadDist) {
    const float earlyExitRange = 0; // change this later to add support for motion chaining and stuff, and make the closeness distance also adjust with it
    const float closeDist = 5;  // distance for it to be considered close

    // tunable parameters and stuff
    float maxSpeed = 127;
    float driftFactor = 3;
    float linearMaxSlew = 20;
    float angularMaxSlew = 5;

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
    float prevLinearOut = 0;
    float prevAngularOut = 0;

    // counter so that the terminal doesnt get overloaded with data
    int loopCounter = 0;

    //while (pros::millis() < startTime + timeout && (!linearExit.isDone() || !angularExit.isDone())) {
    while (pros::millis() < startTime + timeout) {
        Pose robotPose = getPose(true);

        // disable turning if robot is close to target
        float distance = robotPose.distance(targetPose);
        if (distance < closeDist && close == false) {
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
        prevSameSide = sameSide;

        // calculate errors

        // if close, target heading is the heading of the target point
        // if not close, target heading is the heading to face the carrot point
        float angularError = close ? angleError(robotPose.theta, targetPose.theta, true) :
                                     angleError(robotPose.theta, robotPose.angle(carrotPose), true);
        //float angularError = angleError(robotPose.theta, carrotPose.theta, true);
        float linearError = robotPose.distance(carrotPose) * cos(angularError);

        // update exit conditions
        linearExit.update(linearError);
        angularExit.update(toDegrees(angularError));

        // calculate outputs (angular is negative because radians increase ccw, todo: fix inconsistency)
        float linearOut = linearPID->update(linearError);
        float angularOut = -angularPID->update(toDegrees(angularError));
        if (close) angularOut = 0;

        // clamp to max speed
        linearOut = std::clamp(linearOut, -maxSpeed, maxSpeed);
        angularOut = std::clamp(angularOut, -maxSpeed, maxSpeed);

        // constrain outputs to avoid slipping
        linearOut = slew(linearOut, prevLinearOut, linearMaxSlew);
        angularOut = slew(angularOut, prevAngularOut, angularMaxSlew);

        // todo: fix radian increasing ccw inconsistency
        float radius = 1 / fabs(getCurvature(fixRadians(robotPose), fixRadians(carrotPose)));
        float maxSlipSpeed(sqrt(driftFactor * radius * 9.8));
        linearOut = std::clamp(linearOut, -maxSlipSpeed, maxSlipSpeed);

        // update previous values
        prevLinearOut = linearOut;
        prevAngularOut = angularOut;

        // calculate and desaturate outputs
        float leftPower = linearOut + angularOut;
        float rightPower = linearOut - angularOut;
        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;  // 127 = max speed
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move motors
        leftMotors->move(leftPower);
        rightMotors->move(rightPower);

        // log info to terminal
        loopCounter++;
        if (loopCounter % 3 == 1) {
            printf(/* "R: (%.2f, %.2f, %.2f), T: (%.2f, %.2f, %.2f), C: (%.2f, %.2f, %.2f), */ "LE: %.2f,  AE: %.2f,  LO: %.2f,  AO: %.2f\n", 
/*                 robotPose.x, robotPose.y, toDegrees(robotPose.theta),
                targetPose.x, targetPose.y, toDegrees(targetPose.theta),
                carrotPose.x, carrotPose.y, toDegrees(carrotPose.theta), */
                linearError, toDegrees(angularError), linearOut, angularOut
            );
        }

        pros::delay(10);
    }

    // stop motors
    leftMotors->move(0);
    rightMotors->move(0);
}