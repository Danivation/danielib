#include "danielib/danielib.hpp"
#include <cmath>

namespace danielib {
PID::PID(float kP, float kI, float kD, float timeout, float windupRange, float exitRange, float exitTime) :
    kP(kP),
    kI(kI),
    kD(kD),
    timeout(timeout),
    windupRange(windupRange),
    exitRange(exitRange),
    exitTime(exitTime)
{}

float PID::update(const float target, const float current) {
    const float error = target - current;
    integral += error;

    // if the error switches signs, reset the integral
    if (sgn(error) != sgn(prevError)) {
        integral = 0;
    }
    // if the error is outside of the windup range, do not increase the integral
    if (fabs(error) > windupRange && windupRange != 0) {
        integral = 0;
    }

    // calculate derivative based on previous error
    const float derivative = error - prevError;
    prevError = error;

    // return the final output
    return (error * kP) + (integral * kI) + (derivative * kD);
}
} // namespace danielib