#include "danielib/pose.hpp"
#include <cmath>

danielib::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

danielib::Pose danielib::Pose::operator+(const danielib::Pose& other) const {
    return danielib::Pose(this->x + other.x, this->y + other.y, this->theta);
}

danielib::Pose danielib::Pose::operator-(const danielib::Pose& other) const {
    return danielib::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float danielib::Pose::operator*(const danielib::Pose& other) const { return this->x * other.x + this->y * other.y; }

danielib::Pose danielib::Pose::operator*(const float& other) const {
    return danielib::Pose(this->x * other, this->y * other, this->theta);
}

danielib::Pose danielib::Pose::operator/(const float& other) const {
    return danielib::Pose(this->x / other, this->y / other, this->theta);
}

danielib::Pose danielib::Pose::lerp(danielib::Pose other, float t) const {
    return danielib::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float danielib::Pose::distance(danielib::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

// fix so 0 rad is +Y by swapping the x and y
float danielib::Pose::angle(danielib::Pose other) const { return std::atan2(other.x - this->x, other.y - this->y); }

danielib::Pose danielib::Pose::rotate(float angle) const {
    return danielib::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}
