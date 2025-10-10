#include "danielib/danielib.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>
#include <span>

namespace danielib::MCL {

// create random number generator
std::random_device rd;
std::mt19937 rng(rd());
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

Beam::Beam(float angleOffset, float distance) :
    angleOffset(angleOffset),
    distance(distance)
{}

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

// this has to be fast
void Particle::updateDeltaNoise(const Pose& delta) {
    this->x += xyNoise * 2 * (dist(rng) - 0.5) + delta.x;
    this->y += xyNoise * 2 * (dist(rng) - 0.5) + delta.y;
    this->theta += thetaNoise * 2 * (dist(rng) - 0.5) + delta.theta;
}

float Particle::expectedDistance(const Beam& beam) {
    // this is like the one function where i have to do raidan stuff and /0 checks
    // fix radians so sin and cos work right
    float beamAngle = fixRadians(toRadians(this->theta + beam.angleOffset));

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

// add some check to make sure that beams of distances that are too long dont get included
// and maybe average these weights instead of summing
// and maybe make sure theyre positive
void Particle::updateWeight(std::span<const Beam> beams) {
    float sum = 1.0f;
    for (const Beam& beam : beams) {
        if (beam.distance >= 5000) break;       // exit if beam records nothing or is invalid
        sum *= gaussian(expectedDistance(beam) - beam.distance);
    }
    this->weight = sum;
}

Localization::Localization() :
    particles(numParticles, {0, 0, 0}),
    averagePose(0, 0, 0)
{}

Pose Localization::run(const Pose& delta, std::span<const Beam> beams) {
    this->update(delta);
    this->resample(beams);
    return this->averagePose;
}

void Localization::update(const Pose& delta) {
    // delta pose = odom deltas, so (delta x, delta y, delta theta)
    for (Particle& particle : this->particles) {
        particle.updateDeltaNoise(delta);
    }
}

void Localization::resample(std::span<const Beam> beams) {
    // update and sum particle weights
    float sumWeights = 0;
    std::vector<float> cumulativeWeights;
    cumulativeWeights.reserve(numParticles);
    for (Particle& particle : this->particles) {
        particle.updateWeight(beams);
        sumWeights += particle.weight;
        cumulativeWeights.push_back(sumWeights);
    }

    if (sumWeights <= 0) return;

    // actual resampling step
    float randomOffset = dist(rng) / static_cast<float>(numParticles);      // [0,1) / numParticles could be 0.5/100

    std::vector<float> offsets;
    offsets.reserve(numParticles);

    for (size_t i = 0; i < numParticles; ++i) {
        float offset = randomOffset + static_cast<float>(i) / numParticles;     // 1.5/100, 2.5/100, etc
        offsets.push_back(offset * sumWeights);                                 // scale by total weight
    }

    std::vector<Particle> newParticles;
    newParticles.reserve(numParticles);

    /**
     * for each offset, go through the cumulative weights until the weight at the current index exceeds the current offset
     * this means that it will keep the offsets aligned with the particles they fall into
     * if the first particle has a high weight, it will add it to the new list and then move onto the next offset
     * the index (and the particle its checking) wont change until the offset becomes larger than the current particle's weight
     * meaning that the offset (where we are looking at) has moved to the next particle
     */
    size_t index = 0;
    for (float offset: offsets) {
        while (index < cumulativeWeights.size() && cumulativeWeights[index] < offset) {
            ++index;
        }

        // catch edge cases
        if (index >= this->particles.size()) index = this->particles.size() - 1;

        // when looping stops, add the particle at the current index to the 
        newParticles.push_back(this->particles[index]);
    }

    // replace the old particles with the new ones
    this->particles = std::move(newParticles);

    // find new average pose
    float avgX = 0.0f;
    float avgY = 0.0f;
    float avgThetaSin = 0.0f;
    float avgThetaCos = 0.0f;

    for (const Particle& p : this->particles) {
        avgX += p.x;
        avgY += p.y;
        avgThetaSin += std::sin(p.theta);
        avgThetaCos += std::cos(p.theta);
    }

    avgX /= numParticles;
    avgY /= numParticles;
    avgThetaSin /= numParticles;
    avgThetaCos /= numParticles;

    float avgTheta = std::atan2(avgThetaSin, avgThetaCos);

    this->averagePose = {avgX, avgY, avgTheta};
}

} // namespace danielib::MCl