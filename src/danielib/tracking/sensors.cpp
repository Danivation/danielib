#include "danielib/danielib.hpp"
#include <cmath>

namespace danielib {
void TrackerWheel::reset() {
    sensor->reset_position();
}

float TrackerWheel::getPosition() {
    return (wheelDiameter * M_PI / 360) * (sensor->get_position() / 100);
}

float TrackerWheel::getOffset() {
    return offset;
}

void Inertial::calibrate() {
    sensor1->reset(false);
    if (sensor2 != nullptr) sensor2->reset(false);
    pros::delay(2500);
}

float Inertial::getRotation() {
    if (sensor2 != nullptr) {
        return ((sensor1->get_rotation() * scale1) + (sensor2->get_rotation() * scale2)) / 2;
    } else {
        return sensor1->get_rotation() * scale1;
    }
}

float Inertial::getHeading() {
    return reduce_0_to_360(this->getRotation());
}

void Inertial::setRotation(float angle) {
    sensor1->set_rotation(angle);
    if (sensor2 != nullptr) sensor2->set_rotation(angle);
}

void Inertial::setHeading(float angle) {
    sensor1->set_heading(angle);
    if (sensor2 != nullptr) sensor2->set_heading(angle);
}
}