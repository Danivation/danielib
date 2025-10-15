#include "danielib/danielib.hpp"
#include "danielib/pid.hpp"
#include <cmath>

namespace danielib {
PID::PID(float kP, float kI, float kD, float windupRange, float exitRange, float exitTime) :
    kP(kP),
    kI(kI),
    kD(kD),
    windupRange(windupRange),
    exitRange(exitRange),
    exitTime(exitTime)
{}

float PID::update(const float error) {
    integral += error;

    // if the error switches signs, reset the integral
    if (d_sgn(error) != d_sgn(prevError)) {
        integral = 0;
    }
    // if the error is outside of the windup range, do not increase the integral
    if (fabs(error) > windupRange) {
        integral = 0;
    }

    // calculate derivative based on previous error
    const float derivative = error - prevError;
    prevError = error;

    // return the final output
    return (error * kP) + (integral * kI) + (derivative * kD);
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}
} // namespace danielib