#include "danielib/danielib.hpp"
#include <random>
#include <cmath>

namespace danielib::MCL {

// create random number generator
std::random_device rd;
std::mt19937 rng(rd());
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

Particle::Particle(const Pose& pose, float weight = -1) :
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

void Particle::updateDeltaNoise(const Pose& delta) {
    this->x += xyNoise * 2 * (dist(rng) - 0.5) + delta.x;
    this->y += xyNoise * 2 * (dist(rng) - 0.5) + delta.y;
    this->theta += thetaNoise * 2 * (dist(rng) - 0.5) + delta.theta;
}

float Particle::expectedDistance(const Beam& beam) {
    // this is like the one function where i have to do raidan stuff and /0 checks
    float beamAngle = toRadians(this->theta + beam.angleOffset);

    // use inches for field units, the field is about 140 inches across
    return std::min({
        fabs((this->x - 70) / cosf(beamAngle)),
        fabs((this->x + 70) / cosf(beamAngle)),
        fabs((this->y - 70) / sinf(beamAngle)),
        fabs((this->y + 70) / sinf(beamAngle))
    });
}

float Particle::gaussian(float x) {
    return (gaussianFactor * expf(-0.5 * powf((x / gaussianStDev), 2)) / (gaussianStDev * sqrtf(2 * M_PI)));
}

void Particle::updateWeight(std::span<const Beam> beams) {
    float sum = 0;
    for (const Beam& beam : beams) {
        sum += gaussian(expectedDistance(beam) - beam.distance);
    }
    this->weight = sum;
}

}