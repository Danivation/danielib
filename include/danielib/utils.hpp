#pragma once
#include <cmath>

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