#include "danielib/danielib.hpp"
#include <random>
#include <cmath>

namespace danielib::MCL {

// create random number generator
std::random_device rd;
std::mt19937 rng(rd());
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

Particle::Particle(Pose pose, float weight = -1) :
    x(pose.x),
    y(pose.y),
    theta(pose.theta),
    weight(weight)
{}

Particle::Particle(float x, float y, float theta, float weight = -1) :
    x(x),
    y(y),
    theta(theta),
    weight(weight)
{}

void Particle::updateDeltaNoise(Pose delta) {
    this->x += xyNoise * 2 * (dist(rng) - 0.5) + delta.x;
    this->y += xyNoise * 2 * (dist(rng) - 0.5) + delta.y;
    this->theta += thetaNoise * 2 * (dist(rng) - 0.5) + delta.theta;
}

Pose Particle::expectedPoint(Beam beam) {
    float globalTheta = this->theta + beam.angle;
    return Pose(
        this->x + beam.distance * cosf(globalTheta),
        this->y + beam.distance * sinf(globalTheta),
        globalTheta
    );
}

float Particle::distanceToWall(Pose point) {
    
}

}