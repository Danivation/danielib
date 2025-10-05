#pragma once
#include <cmath>
#include "danielib/pose.hpp"

// returns the sign of the input as -1, 0, or 1
template <typename T> constexpr auto sgn(const T& lhs) {
    auto q = lhs;
    if (q > 0) return T(1);
    if (q < 0) return T(-1);
    return T(0);
}

// degrees to radians
inline float toRadians(float angle) {
    return angle * (M_PI / 180.0f);
}

// radians to degrees
inline float toDegrees(float angle) {
    return angle * (180.0f / M_PI);
}

// reduce angle to range [0, 360)
inline float reduce_to_0_360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

// reduce angle to range [-180, 180)
inline float reduce_to_180_180(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle - 180.0f;
}

// reduce radians to range [0, 2pi)
inline float reduce_radians(float angle) {
    angle = fmodf(angle, M_PI * 2.0f);
    if (angle < 0) angle += M_PI * 2.0f;
    return angle;
}

inline constexpr float sanitizeAngle(float angle, bool radians = false) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}   

inline float angleError(float target, float position, bool radians = false) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;

    return std::remainder(rawError, max);
}

inline float slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

// get curvature between two poses by finding a circle using their headings, theta must be in radians and increase ccw
inline float getCurvature(danielib::Pose pose, danielib::Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

inline danielib::Pose fixRadians(danielib::Pose pose) {
    return danielib::Pose(pose.x, pose.y, M_PI_2 - pose.theta);
}
