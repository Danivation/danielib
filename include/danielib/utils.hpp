#pragma once
#include <cmath>

template <typename T> constexpr auto sgn(const T& lhs) {
    auto q = lhs;
    if (q > 0) return T(1);
    if (q < 0) return T(-1);
    return T(0);
}

// degrees to radians
inline float toRadians(float angle) {
    // degrees to radians
    return angle * (M_PI / 180);
}

// radians to degrees
inline float toDegrees(float angle) {
    // radians to degrees
    return angle * (180 / M_PI);
}

// reduce to 0-360
inline float reduce_0_to_360(float angle) {
    // reduce to 0-360
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

// reduce to 0-2pi
inline float reduce_radians(float angle) {
    // reduce to 0-2pi
    while (angle < 0) angle += 2.0f * M_PI;
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    return angle;
}