#pragma once

template <typename T> constexpr auto sgn(const T& lhs) {
    auto q = lhs;
    if (q > 0) return T(1);
    if (q < 0) return T(-1);
    return T(0);
}